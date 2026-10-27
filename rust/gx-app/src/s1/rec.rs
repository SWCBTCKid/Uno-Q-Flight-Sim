//! S1 log records and their JSONL encoding.
//!
//! The simulation and reader threads build fixed-size [`Rec`] values (no
//! allocation, no formatting) and hand them to the bounded log drain; the I/O
//! thread formats them. Every line is one JSON object with `"k"` (kind) and
//! `"t"` (ns since the run epoch, `CLOCK_MONOTONIC`). The schema is
//! `gx-s1/1`; `tools/s1_check.py` is its consumer.

use gx_sim::autopilot::{ApWarning, Lateral, Vertical};
use gx_sim::{
    AircraftSnapshot, ApAction, ApInput, ApStatus, Command, ControlInput, ControlRecord, EdgeCounter, InputStatus, InputValidity, Levers, PauseReason,
    RunState, UnavailableReason,
};
use std::fmt::Write as _;

pub const SCHEMA: &str = "gx-s1/1";

/// Upper bounds (µs, inclusive) of the tick-lateness histogram buckets; the
/// last bucket is everything above 8333 µs (one full period).
pub const LATE_BUCKETS_US: [u64; 9] = [25, 50, 100, 200, 500, 1000, 2000, 4000, 8333];
pub const LATE_BUCKETS: usize = LATE_BUCKETS_US.len() + 1;

/// Busy-run length buckets (ticks, inclusive upper bounds); last is above.
pub const BUSY_RUN_BUCKETS: [u64; 7] = [1, 2, 4, 8, 16, 32, 64];
pub const BUSY_BUCKETS: usize = BUSY_RUN_BUCKETS.len() + 1;

pub fn bucket(bounds: &[u64], value: u64) -> usize {
    bounds.iter().position(|&b| value <= b).unwrap_or(bounds.len())
}

/// Which INPUT-002 gate refused a Start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gate {
    /// The control mailbox reader withheld it (input not valid, or the edge
    /// predates the current valid stretch).
    Mailbox,
    /// `SimCore` refused it (its input status was not `Valid`).
    Core,
}

/// Where a command came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// A stick button edge through the control mailbox.
    Stick,
    /// ENTER on the S1 console (test hook for the core gate).
    Console,
    /// The application itself (startup reset).
    App,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SimSecond {
    pub t: u64,
    pub ticks: u64,
    pub misses: u64,
    pub skipped: u64,
    pub late_max_ns: u64,
    pub work_max_ns: u64,
    pub late_hist: [u64; LATE_BUCKETS],
    pub busy_events: u64,
    pub drops: u64,
}

/// One missed 120 Hz deadline. Emitted only for a miss, so exact forensic
/// timing stays bounded instead of turning the log into a per-tick trace.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeadlineMiss {
    pub t: u64,
    pub tick: u64,
    pub due_ns: u64,
    pub wake_ns: u64,
    pub end_ns: u64,
    pub skipped: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Latency {
    /// Clock after the consuming tick finished (stepped and published).
    pub t: u64,
    pub tick: u64,
    pub seq: u64,
    /// Frames published by the I/O thread but overwritten before this read.
    pub skipped_frames: u64,
    pub frame_ts: u64,
    /// Tick wake (mailbox read) minus frame timestamp.
    pub consume_ns: u64,
    /// Tick end (snapshot offered) minus frame timestamp.
    pub publish_ns: u64,
    /// The snapshot of this tick reached the exchange (not busy).
    pub published: bool,
    pub running: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Visible {
    pub run: RunState,
    pub pause: Option<PauseReason>,
    pub status: InputStatus,
    pub warning: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReadRec {
    pub t: u64,
    pub ok: bool,
    pub seq: u64,
    pub generation: u64,
    pub tick: u64,
    /// Reader clock minus the latest snapshot's `published_at_ns`.
    pub age_ns: u64,
    /// Duration of `try_latest_pair` (slots pinned).
    pub copy_ns: u64,
    /// Current simulation state at read time, not the historic state carried
    /// by `latest`.
    pub paused: bool,
    pub pair: bool,
    pub warning: bool,
}

/// S2 record/replay: one per sim tick (see `SimDriver::with_trace`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TickRec {
    /// 1-based count of tick records; a gap means a dropped record.
    pub n: u64,
    /// Absolute `CLOCK_MONOTONIC` wake time: the mailbox read time.
    pub wake_ns: u64,
    /// Cumulative console Starts seen by this tick.
    pub console: u64,
    /// The mailbox record read at this tick if it was a new publication.
    pub record: Option<ControlRecord>,
    /// Duration of `SimLoop::tick` (JSBSim step + publish) when it stepped.
    pub step_ns: Option<u64>,
    /// `state_hash` of the state this tick produced (step or reset).
    pub hash: Option<u64>,
}

/// S2: a compact 10 Hz sample of the flight state and applied controls.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateRec {
    pub t: u64,
    pub tick: u64,
    pub generation: u64,
    pub vis: Visible,
    /// Controls the step used: roll, pitch, yaw, throttle, left/right brake.
    pub controls: [f32; 6],
    pub enu_m: [f32; 3],
    pub gs_kt: f32,
    pub ias_kt: f32,
    pub agl_ft: f32,
    pub vs_fpm: f32,
    /// Roll, pitch, heading in degrees.
    pub att_deg: [f32; 3],
    /// Body rates p, q, r in degrees per second.
    pub rates_dps: [f32; 3],
    /// Elevator, left aileron, rudder (JSBSim normalized positions), flaps, gear.
    pub surfaces: [f32; 5],
    pub n1: f32,
    pub wow: bool,
    /// Pitch trim switch (as in `controls`) and trim position (+ = nose up).
    pub trim: [f32; 2],
    /// Autopilot modes and targets of this state (AP-001).
    pub ap: ApStatus,
    /// Roll, pitch and throttle the core gave the backend for the step that
    /// led here: `controls` with the autopilot's and autothrottle's commands
    /// in place of the pilot's while they fly.
    pub applied: [f32; 3],
}

impl StateRec {
    /// `c` is the pilot's latched controls, `applied` what the core gave
    /// the backend ([`gx_sim::SimCore::applied_controls`]).
    pub fn new(t: u64, s: &AircraftSnapshot, c: ControlInput, applied: ControlInput, vis: Visible) -> Self {
        const MPS_TO_KT: f64 = 1.0 / 0.514_444;
        const M_TO_FT: f64 = 1.0 / 0.3048;
        let st = &s.state;
        let (enu, vel) = st.local_enu.map_or(([0.0; 3], [0.0; 3]), |p| (p.position_m, p.velocity_mps));
        let deg = |r: f64| r.to_degrees() as f32;
        Self {
            t,
            tick: s.simulation_tick,
            generation: s.generation,
            vis,
            controls: [c.roll, c.pitch, c.yaw, c.throttle, c.left_brake, c.right_brake],
            enu_m: [enu[0] as f32, enu[1] as f32, enu[2] as f32],
            gs_kt: ((vel[0] * vel[0] + vel[1] * vel[1]).sqrt() * MPS_TO_KT) as f32,
            ias_kt: (st.calibrated_airspeed_mps * MPS_TO_KT) as f32,
            agl_ft: (st.height_agl_m * M_TO_FT) as f32,
            vs_fpm: (st.vertical_speed_mps * M_TO_FT * 60.0) as f32,
            att_deg: [deg(st.roll_rad), deg(st.pitch_rad), deg(st.heading_rad)],
            rates_dps: [deg(st.angular_rate_rps[0]), deg(st.angular_rate_rps[1]), deg(st.angular_rate_rps[2])],
            surfaces: [
                st.elevator_position as f32,
                st.aileron_position as f32,
                st.rudder_position as f32,
                st.flap_position as f32,
                st.gear_position as f32,
            ],
            n1: st.engine_n1 as f32,
            wow: st.weight_on_wheels,
            trim: [c.trim, st.pitch_trim as f32],
            ap: st.autopilot,
            applied: [applied.roll, applied.pitch, applied.throttle],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Rec {
    SimSecond(SimSecond),
    DeadlineMiss(DeadlineMiss),
    Latency(Latency),
    BusyRun { t: u64, tick: u64, ticks: u64, attempts: u64 },
    /// Run state, pause reason, input status or warning changed.
    Input002 { t: u64, tick: u64, vis: Visible, loss_ms: u64 },
    StartRefused { t: u64, gate: Gate, source: Source, vis: Visible, loss_ms: u64 },
    StartAccepted { t: u64, source: Source, from_run: RunState, from_pause: Option<PauseReason> },
    Command { t: u64, command: Command, source: Source },
    Fault { t: u64, tick: u64 },
    Read(ReadRec),
    Tick(TickRec),
    State(StateRec),
}

pub fn run_str(run: RunState) -> &'static str {
    match run {
        RunState::Stopped => "stopped",
        RunState::Running => "running",
        RunState::Paused => "paused",
    }
}

pub fn pause_json(pause: Option<PauseReason>) -> &'static str {
    match pause {
        None => "null",
        Some(PauseReason::User) => "\"user\"",
        Some(PauseReason::Reset) => "\"reset\"",
        Some(PauseReason::InputLoss) => "\"input_loss\"",
        Some(PauseReason::InvalidWorldPose) => "\"invalid_world_pose\"",
    }
}

pub fn status_str(status: InputStatus) -> &'static str {
    match status {
        InputStatus::Valid => "valid",
        InputStatus::NoInput => "no_input",
        InputStatus::Stale => "stale",
        InputStatus::Unavailable(UnavailableReason::Disconnected) => "unavailable:disconnected",
        InputStatus::Unavailable(UnavailableReason::InputLoss) => "unavailable:input_loss",
        InputStatus::Unavailable(UnavailableReason::ResyncRequired) => {
            "unavailable:resync_required"
        }
    }
}

fn gate_str(gate: Gate) -> &'static str {
    match gate {
        Gate::Mailbox => "mailbox",
        Gate::Core => "core",
    }
}

fn source_str(source: Source) -> &'static str {
    match source {
        Source::Stick => "stick",
        Source::Console => "console",
        Source::App => "app",
    }
}

fn command_str(command: Command) -> &'static str {
    match command {
        Command::Start => "start",
        Command::Pause => "pause",
        Command::Reset => "reset",
    }
}

/// JSON string literal.
pub fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

pub fn list(values: &[u64]) -> String {
    let mut s = String::from("[");
    for (i, v) in values.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let _ = write!(s, "{v}");
    }
    s.push(']');
    s
}

fn vis_fields(vis: &Visible) -> String {
    format!(
        "\"run\":\"{}\",\"pause\":{},\"status\":\"{}\",\"warning\":{}",
        run_str(vis.run),
        pause_json(vis.pause),
        status_str(vis.status),
        vis.warning
    )
}

fn lateral_str(l: Lateral) -> &'static str {
    match l {
        Lateral::WingsLevel => "wings_level",
        Lateral::Heading => "heading",
        Lateral::Localizer => "localizer",
    }
}

fn vertical_str(v: Vertical) -> &'static str {
    match v {
        Vertical::LevelOff => "level_off",
        Vertical::AltHold => "alt_hold",
        Vertical::AltSelect => "alt_select",
        Vertical::Glideslope => "glideslope",
        Vertical::GsDescent => "gs_descent",
    }
}

fn warning_json(w: Option<ApWarning>) -> &'static str {
    match w {
        None => "null",
        Some(ApWarning::ApDisconnect) => "\"ap_disconnect\"",
        Some(ApWarning::AtDisconnect) => "\"at_disconnect\"",
        Some(ApWarning::NotAvailable) => "\"not_available\"",
        Some(ApWarning::NoIls) => "\"no_ils\"",
        Some(ApWarning::InputLossLevel) => "\"input_loss_level\"",
    }
}

/// The `state` record's `"ap"` object: modes, the approach index, the held
/// altitude, the panel targets (heading, altitude, speed), the height above
/// the glide path while an approach is armed (ft) and the commands applied
/// (roll, pitch, throttle).
fn ap_json(ap: &ApStatus, applied: &[f32; 3]) -> String {
    format!(
        "{{\"engaged\":{},\"lat\":\"{}\",\"vert\":\"{}\",\"loc_armed\":{},\"gs_armed\":{},\"approach\":{},\"at\":{},\"hold_ft\":{:.1},\"tgt\":{},\"warning\":{},\"gs_dev_ft\":{},\"cmd\":{}}}",
        ap.engaged,
        lateral_str(ap.lateral),
        vertical_str(ap.vertical),
        ap.loc_armed,
        ap.gs_armed,
        ap.approach.map_or("null".into(), |i| i.to_string()),
        ap.autothrottle,
        ap.hold_ft,
        f32_list(&[ap.input.heading_deg, ap.input.altitude_ft, ap.input.speed_kt]),
        warning_json(ap.warning),
        ap.gs_dev_ft.filter(|d| d.is_finite()).map_or("null".into(), |d| format!("{d:.1}")),
        f32_list(applied)
    )
}

fn validity_code(v: InputValidity) -> u8 {
    match v {
        InputValidity::Valid => 0,
        InputValidity::Unavailable(UnavailableReason::Disconnected) => 1,
        InputValidity::Unavailable(UnavailableReason::InputLoss) => 2,
        InputValidity::Unavailable(UnavailableReason::ResyncRequired) => 3,
    }
}

fn validity_from_code(code: u64) -> Option<InputValidity> {
    Some(match code {
        0 => InputValidity::Valid,
        1 => InputValidity::Unavailable(UnavailableReason::Disconnected),
        2 => InputValidity::Unavailable(UnavailableReason::InputLoss),
        3 => InputValidity::Unavailable(UnavailableReason::ResyncRequired),
        _ => return None,
    })
}

/// Number of entries in a `tick` record's `"r"` array.
pub const RECORD_FIELDS: usize = 27;
/// Logs written before the AP-001 autopilot panel have no panel entries;
/// they parse with `ApInput::DEFAULT` (no click, the default targets).
pub const RECORD_FIELDS_BEFORE_AP: usize = 22;
/// Logs written before the INPUT-005 levers have no lever entries; they
/// parse with the run's initial levers (which those runs held throughout).
pub const RECORD_FIELDS_BEFORE_LEVERS: usize = 19;
/// Logs written before pitch trim (S2..SPIKE-002 evidence) have no trim
/// switch entry; it parses as 0 (released).
pub const RECORD_FIELDS_BEFORE_TRIM: usize = 18;

/// A control record as a flat JSON array: publication, published_at_ns,
/// validity (0 valid, 1 disconnected, 2 input loss, 3 resync), roll, pitch,
/// yaw, throttle, left brake, right brake, input sequence, input timestamp,
/// then (count, last publication) for Start, Pause, Reset, then
/// valid_since_publication, then the pitch trim switch, then the gear lever
/// (1 down, 0 up), flap and speedbrake lever, then the autopilot panel:
/// serial, action code (`ApAction::code`), heading, altitude and speed
/// targets (each group appended last so every earlier index is unchanged). Floats use Rust's shortest round-trip
/// formatting, so parsing gives back the identical `f32`.
pub fn record_json(r: &ControlRecord) -> String {
    let c = r.controls;
    let e = r.edges;
    format!(
        "[{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}]",
        r.publication, r.published_at_ns, validity_code(r.validity),
        c.roll, c.pitch, c.yaw, c.throttle, c.left_brake, c.right_brake,
        r.input_sequence, r.input_timestamp_ns,
        e[0].count, e[0].last_publication, e[1].count, e[1].last_publication, e[2].count, e[2].last_publication,
        r.valid_since_publication, c.trim, u8::from(c.levers.gear_down), c.levers.flaps, c.levers.speedbrake,
        c.ap.serial, c.ap.action.code(), c.ap.heading_deg, c.ap.altitude_ft, c.ap.speed_kt
    )
}

/// Inverse of [`record_json`] on the array's items (already split); levers
/// absent from an older record are `Levers::default()`, an absent panel is
/// `ApInput::DEFAULT`.
pub fn record_from_items(items: &[&str]) -> Option<ControlRecord> {
    record_from_items_with(items, Levers::default())
}

/// [`record_from_items`] with `initial` for a record that predates the
/// levers.
pub fn record_from_items_with(items: &[&str], initial: Levers) -> Option<ControlRecord> {
    if ![RECORD_FIELDS, RECORD_FIELDS_BEFORE_AP, RECORD_FIELDS_BEFORE_LEVERS, RECORD_FIELDS_BEFORE_TRIM].contains(&items.len()) {
        return None;
    }
    let u = |i: usize| items[i].trim().parse::<u64>().ok();
    let f = |i: usize| items[i].trim().parse::<f32>().ok();
    let levers = if items.len() >= RECORD_FIELDS_BEFORE_AP {
        Levers {
            gear_down: match u(19)? { 0 => false, 1 => true, _ => return None },
            flaps: f(20)?,
            speedbrake: f(21)?,
        }
    } else {
        initial
    };
    let ap = if items.len() == RECORD_FIELDS {
        let serial = u32::try_from(u(22)?).ok()?;
        let action = ApAction::from_code(u8::try_from(u(23)?).ok()?)?;
        ApInput { serial, action, heading_deg: f(24)?, altitude_ft: f(25)?, speed_kt: f(26)? }
    } else {
        ApInput::DEFAULT
    };
    Some(ControlRecord {
        publication: u(0)?,
        published_at_ns: u(1)?,
        validity: validity_from_code(u(2)?)?,
        controls: ControlInput {
            roll: f(3)?, pitch: f(4)?, yaw: f(5)?, throttle: f(6)?, left_brake: f(7)?, right_brake: f(8)?,
            trim: if items.len() > RECORD_FIELDS_BEFORE_TRIM { f(18)? } else { 0.0 },
            levers,
            ap,
        },
        input_sequence: u(9)?,
        input_timestamp_ns: u(10)?,
        edges: [
            EdgeCounter { count: u(11)?, last_publication: u(12)? },
            EdgeCounter { count: u(13)?, last_publication: u(14)? },
            EdgeCounter { count: u(15)?, last_publication: u(16)? },
        ],
        valid_since_publication: u(17)?,
    })
}

fn f32_list(values: &[f32]) -> String {
    let mut s = String::from("[");
    for (i, v) in values.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        if v.is_finite() {
            let _ = write!(s, "{v:.4}");
        } else {
            s.push_str("null");
        }
    }
    s.push(']');
    s
}

/// One JSONL line (without newline). `epoch` is subtracted from every time.
pub fn to_json(rec: &Rec, epoch: u64) -> String {
    let rel = |t: u64| t.saturating_sub(epoch);
    match rec {
        Rec::SimSecond(s) => format!(
            "{{\"k\":\"sim_sec\",\"t\":{},\"ticks\":{},\"misses\":{},\"skipped\":{},\"late_max_ns\":{},\"work_max_ns\":{},\"late_hist\":{},\"busy_events\":{},\"drops\":{}}}",
            rel(s.t), s.ticks, s.misses, s.skipped, s.late_max_ns, s.work_max_ns,
            list(&s.late_hist), s.busy_events, s.drops
        ),
        Rec::DeadlineMiss(m) => format!(
            "{{\"k\":\"deadline_miss\",\"t\":{},\"tick\":{},\"due_ns\":{},\"wake_ns\":{},\"end_ns\":{},\"late_ns\":{},\"work_ns\":{},\"skipped\":{}}}",
            rel(m.t), m.tick, rel(m.due_ns), rel(m.wake_ns), rel(m.end_ns),
            m.wake_ns.saturating_sub(m.due_ns), m.end_ns.saturating_sub(m.wake_ns), m.skipped
        ),
        Rec::Latency(l) => format!(
            "{{\"k\":\"lat\",\"t\":{},\"tick\":{},\"seq\":{},\"skipped_frames\":{},\"frame_ts\":{},\"consume_ns\":{},\"publish_ns\":{},\"published\":{},\"running\":{}}}",
            rel(l.t), l.tick, l.seq, l.skipped_frames, rel(l.frame_ts), l.consume_ns,
            l.publish_ns, l.published, l.running
        ),
        Rec::BusyRun { t, tick, ticks, attempts } => format!(
            "{{\"k\":\"busy_run\",\"t\":{},\"tick\":{tick},\"ticks\":{ticks},\"attempts\":{attempts}}}",
            rel(*t)
        ),
        Rec::Input002 { t, tick, vis, loss_ms } => format!(
            "{{\"k\":\"input002\",\"t\":{},\"tick\":{tick},{},\"loss_ms\":{loss_ms}}}",
            rel(*t),
            vis_fields(vis)
        ),
        Rec::StartRefused { t, gate, source, vis, loss_ms } => format!(
            "{{\"k\":\"start_refused\",\"t\":{},\"gate\":\"{}\",\"source\":\"{}\",{},\"loss_ms\":{loss_ms}}}",
            rel(*t),
            gate_str(*gate),
            source_str(*source),
            vis_fields(vis)
        ),
        Rec::StartAccepted { t, source, from_run, from_pause } => format!(
            "{{\"k\":\"start_accepted\",\"t\":{},\"source\":\"{}\",\"from_run\":\"{}\",\"from_pause\":{}}}",
            rel(*t),
            source_str(*source),
            run_str(*from_run),
            pause_json(*from_pause)
        ),
        Rec::Command { t, command, source } => format!(
            "{{\"k\":\"cmd\",\"t\":{},\"cmd\":\"{}\",\"source\":\"{}\"}}",
            rel(*t),
            command_str(*command),
            source_str(*source)
        ),
        Rec::Fault { t, tick } => {
            format!("{{\"k\":\"fault\",\"t\":{},\"tick\":{tick},\"what\":\"invalid_world_pose\"}}", rel(*t))
        }
        Rec::Tick(k) => {
            let mut line = format!(
                "{{\"k\":\"tick\",\"t\":{},\"n\":{},\"w\":{},\"c\":{}",
                rel(k.wake_ns), k.n, k.wake_ns, k.console
            );
            if let Some(r) = &k.record {
                let _ = write!(line, ",\"r\":{}", record_json(r));
            }
            if let Some(ns) = k.step_ns {
                let _ = write!(line, ",\"s\":{ns}");
            }
            if let Some(h) = k.hash {
                let _ = write!(line, ",\"h\":\"{h:016x}\"");
            }
            line.push('}');
            line
        }
        Rec::State(s) => format!(
            "{{\"k\":\"state\",\"t\":{},\"tick\":{},\"gen\":{},{},\"ctl\":{},\"enu\":{},\"gs_kt\":{:.2},\"ias_kt\":{:.2},\"agl_ft\":{:.2},\"vs_fpm\":{:.1},\"att_deg\":{},\"rates_dps\":{},\"surf\":{},\"n1\":{:.2},\"wow\":{},\"trim\":{},\"ap\":{}}}",
            rel(s.t), s.tick, s.generation, vis_fields(&s.vis), f32_list(&s.controls), f32_list(&s.enu_m),
            s.gs_kt, s.ias_kt, s.agl_ft, s.vs_fpm, f32_list(&s.att_deg), f32_list(&s.rates_dps),
            f32_list(&s.surfaces), s.n1, s.wow, f32_list(&s.trim), ap_json(&s.ap, &s.applied)
        ),
        Rec::Read(r) => {
            if r.ok {
                format!(
                    "{{\"k\":\"read\",\"t\":{},\"ok\":true,\"seq\":{},\"gen\":{},\"tick\":{},\"age_ns\":{},\"copy_ns\":{},\"paused\":{},\"pair\":{},\"warning\":{}}}",
                    rel(r.t), r.seq, r.generation, r.tick, r.age_ns, r.copy_ns, r.paused,
                    r.pair, r.warning
                )
            } else {
                format!(
                    "{{\"k\":\"read\",\"t\":{},\"ok\":false,\"copy_ns\":{}}}",
                    rel(r.t),
                    r.copy_ns
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_records_round_trip_exactly_through_json() {
        let r = ControlRecord {
            publication: 77,
            published_at_ns: 123_456_789_012,
            validity: InputValidity::Unavailable(UnavailableReason::ResyncRequired),
            controls: ControlInput {
                roll: 0.1, pitch: -0.333_333_34, yaw: f32::MIN_POSITIVE, throttle: 1.0, left_brake: -0.0,
                right_brake: 0.987_654_3, trim: -1.0,
                levers: Levers { gear_down: false, flaps: 5.0 / 35.0, speedbrake: 0.5 },
                ap: ApInput { serial: 4_000_000_000, action: ApAction::Approach, heading_deg: 297.8, altitude_ft: 3000.0, speed_kt: 0.1 },
            },
            input_sequence: 9,
            input_timestamp_ns: 5,
            edges: [
                EdgeCounter { count: 3, last_publication: 70 },
                EdgeCounter { count: 0, last_publication: 0 },
                EdgeCounter { count: 1, last_publication: 12 },
            ],
            valid_since_publication: 66,
        };
        let json = record_json(&r);
        let items: Vec<&str> = json.trim_matches(|c| c == '[' || c == ']').split(',').collect();
        let back = record_from_items(&items).unwrap();
        assert_eq!(back, r);
        for (a, b) in [
            (back.controls.roll, r.controls.roll), (back.controls.pitch, r.controls.pitch),
            (back.controls.yaw, r.controls.yaw), (back.controls.left_brake, r.controls.left_brake),
        ] {
            assert_eq!(a.to_bits(), b.to_bits());
        }
        assert_eq!(back.controls.levers.flaps.to_bits(), r.controls.levers.flaps.to_bits());
        assert_eq!(back.controls.ap.speed_kt.to_bits(), r.controls.ap.speed_kt.to_bits());
        // A pre-AP (22-entry) record keeps its levers, with the default panel.
        let back = record_from_items(&items[..RECORD_FIELDS_BEFORE_AP]).unwrap();
        assert_eq!(back.controls.ap, ApInput::DEFAULT);
        assert_eq!(ControlRecord { controls: ControlInput { ap: r.controls.ap, ..back.controls }, ..back }, r);
        // A pre-lever (19-entry) record takes the run's initial levers.
        let initial = Levers { gear_down: true, flaps: 0.5, speedbrake: 0.0 };
        let back = record_from_items_with(&items[..RECORD_FIELDS_BEFORE_LEVERS], initial).unwrap();
        assert_eq!(back.controls.levers, initial);
        assert_eq!(back.controls.trim, -1.0);
        // A pre-trim (18-entry) record from S2..SPIKE-002 evidence still parses,
        // with the trim switch released.
        let old = &items[..RECORD_FIELDS_BEFORE_TRIM];
        let back = record_from_items(old).unwrap();
        assert_eq!(back.controls.trim, 0.0);
        assert_eq!(back.controls.levers, Levers::default());
        assert_eq!(back.controls.ap, ApInput::DEFAULT);
        assert_eq!(
            ControlRecord { controls: ControlInput { trim: -1.0, levers: r.controls.levers, ap: r.controls.ap, ..back.controls }, ..back },
            r
        );
        assert!(record_from_items(&items[..17]).is_none());
        assert!(record_from_items(&items[..20]).is_none());
        assert!(record_from_items(&items[..25]).is_none());
        let mut bad = items.clone();
        bad[19] = "2";
        assert!(record_from_items(&bad).is_none(), "gear lever is 0 or 1");
        let mut bad = items.clone();
        bad[23] = "7";
        assert!(record_from_items(&bad).is_none(), "unknown panel action");
    }

    #[test]
    fn buckets_and_escaping() {
        assert_eq!(bucket(&LATE_BUCKETS_US, 0), 0);
        assert_eq!(bucket(&LATE_BUCKETS_US, 25), 0);
        assert_eq!(bucket(&LATE_BUCKETS_US, 26), 1);
        assert_eq!(bucket(&LATE_BUCKETS_US, 8333), 8);
        assert_eq!(bucket(&LATE_BUCKETS_US, 8334), 9);
        assert_eq!(esc("a\"b\\c\n"), "\"a\\\"b\\\\c\\u000a\"");
    }

    #[test]
    fn records_encode_relative_time_and_input002_fields() {
        let vis = Visible {
            run: RunState::Paused,
            pause: Some(PauseReason::InputLoss),
            status: InputStatus::Unavailable(UnavailableReason::Disconnected),
            warning: true,
        };
        let line = to_json(&Rec::Input002 { t: 1_500, tick: 7, vis, loss_ms: 10_000 }, 500);
        assert_eq!(
            line,
            "{\"k\":\"input002\",\"t\":1000,\"tick\":7,\"run\":\"paused\",\"pause\":\"input_loss\",\"status\":\"unavailable:disconnected\",\"warning\":true,\"loss_ms\":10000}"
        );
        let line = to_json(
            &Rec::StartRefused { t: 600, gate: Gate::Core, source: Source::Console, vis, loss_ms: 3 },
            500,
        );
        assert!(line.starts_with("{\"k\":\"start_refused\",\"t\":100,\"gate\":\"core\",\"source\":\"console\""));
        let line = to_json(
            &Rec::DeadlineMiss(DeadlineMiss {
                t: 20_000, tick: 9, due_ns: 10_000, wake_ns: 18_500,
                end_ns: 20_000, skipped: 1,
            }),
            1_000,
        );
        assert_eq!(
            line,
            "{\"k\":\"deadline_miss\",\"t\":19000,\"tick\":9,\"due_ns\":9000,\"wake_ns\":17500,\"end_ns\":19000,\"late_ns\":8500,\"work_ns\":1500,\"skipped\":1}"
        );
    }
}

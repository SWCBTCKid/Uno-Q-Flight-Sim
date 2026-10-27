//! Operator prompts for the Chief's S2 session (headless, ~4 min), driven by
//! the observed state. Pure: the runtime feeds `Obs` every 100 ms and logs
//! each returned line as a `phase` record named after the phase it belongs
//! to; `tools/s2_check.py` uses the phase records as the sign-check windows.
//!
//! Timeline: connect -> start -> 8 sign holds (~60 s) -> lineup -> takeoff
//! (<= 2 min) -> end.
//!
//! S3 ([`Prompter::free_flight`]): the same up to the take-off, which then
//! does not end at 1,500 ft; after a minute (or 1,500 ft) the Chief is asked
//! to unplug the stick for the INPUT-002 banner (unplug_test), then flies
//! freely until the time is up (free). The prompts are drawn on screen.

use gx_sim::{InputStatus, RunState};

/// One held input of the sign check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hold {
    PullBack,
    PushForward,
    Right,
    Left,
    TwistRight,
    TwistLeft,
    ThrottleUp,
    ThrottleDown,
}

pub const HOLDS: [Hold; 8] = [
    Hold::PullBack,
    Hold::PushForward,
    Hold::Right,
    Hold::Left,
    Hold::TwistRight,
    Hold::TwistLeft,
    Hold::ThrottleUp,
    Hold::ThrottleDown,
];

impl Hold {
    pub fn name(self) -> &'static str {
        match self {
            Hold::PullBack => "sign_pull_back",
            Hold::PushForward => "sign_push_forward",
            Hold::Right => "sign_right",
            Hold::Left => "sign_left",
            Hold::TwistRight => "sign_twist_right",
            Hold::TwistLeft => "sign_twist_left",
            Hold::ThrottleUp => "sign_throttle_up",
            Hold::ThrottleDown => "sign_throttle_down",
        }
    }

    /// (input index in `Obs::controls`, expected sign, surface index in
    /// `Obs::surfaces` and the sign it must take, or `None` for N1).
    fn expectation(self) -> (usize, f64, Option<(usize, f64)>) {
        match self {
            Hold::PullBack => (1, 1.0, Some((0, -1.0))),
            Hold::PushForward => (1, -1.0, Some((0, 1.0))),
            Hold::Right => (0, 1.0, Some((1, 1.0))),
            Hold::Left => (0, -1.0, Some((1, -1.0))),
            Hold::TwistRight => (2, 1.0, Some((2, -1.0))),
            Hold::TwistLeft => (2, -1.0, Some((2, 1.0))),
            Hold::ThrottleUp => (3, 1.0, None),
            Hold::ThrottleDown => (3, -1.0, None),
        }
    }

    fn instruction(self) -> &'static str {
        match self {
            Hold::PullBack => "PULL the stick fully BACK (towards you) and hold",
            Hold::PushForward => "PUSH the stick fully FORWARD and hold",
            Hold::Right => "move the stick fully RIGHT and hold",
            Hold::Left => "move the stick fully LEFT and hold",
            Hold::TwistRight => "TWIST the stick fully RIGHT (clockwise) and hold",
            Hold::TwistLeft => "TWIST the stick fully LEFT (anticlockwise) and hold",
            Hold::ThrottleUp => "throttle lever fully FORWARD (+) and hold",
            Hold::ThrottleDown => "throttle lever fully BACK (-, idle) and hold",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Connect,
    Start,
    /// Centre before hold `i`.
    Ready(usize),
    Sign(usize),
    Lineup,
    Takeoff,
    /// S3: unplug the stick, see the banner and the 10 s pause, replug, TRIGGER.
    UnplugTest,
    /// S3: free flight until the time is up.
    Free,
    End,
    Done,
}

impl Phase {
    pub fn name(self) -> &'static str {
        match self {
            Phase::Connect => "connect",
            Phase::Start => "start",
            Phase::Ready(_) => "sign_ready",
            Phase::Sign(i) => HOLDS[i].name(),
            Phase::Lineup => "lineup",
            Phase::Takeoff => "takeoff",
            Phase::UnplugTest => "unplug_test",
            Phase::Free => "free",
            Phase::End => "end",
            Phase::Done => "done",
        }
    }
}

pub const READY_S: f64 = 2.0;
pub const HOLD_S: f64 = 5.5;
pub const LINEUP_S: f64 = 10.0;
pub const TAKEOFF_S: f64 = 120.0;
pub const END_AGL_FT: f64 = 1_500.0;
pub const STATUS_EVERY_S: f64 = 3.0;
/// S3: ask for the unplug test after this much running take-off time (or at
/// [`END_AGL_FT`]), and give up waiting for it after [`UNPLUG_WAIT_S`].
pub const UNPLUG_AFTER_S: f64 = 60.0;
pub const UNPLUG_WAIT_S: f64 = 45.0;

pub const BUTTONS_HELP: &str = "Bindings: START = button 1 (trigger), PAUSE = button 2, \
RESET = button 3 (also sets the parking brake again), PARKING BRAKE toggle = button 4. \
Hat = wheel brakes: back = both, left/right = that side. Buttons 5-12 are unmapped. Twist = rudder. \
Headless: nothing on screen, follow these prompts.";

/// What the prompter sees (relaxed atomics from the threads).
#[derive(Clone, Copy, Debug, Default)]
pub struct Obs {
    pub t_ns: u64,
    pub end_ns: u64,
    pub connected: bool,
    pub frames: u64,
    pub run: Option<RunState>,
    pub status: Option<InputStatus>,
    pub parking_brake: bool,
    pub ias_kt: f64,
    pub gs_kt: f64,
    pub agl_ft: f64,
    pub vs_fpm: f64,
    pub pitch_deg: f64,
    pub hdg_deg: f64,
    pub n1: f64,
    pub wow: bool,
    /// Snapshot generation: changes on every Reset.
    pub generation: u64,
    /// Applied controls: roll, pitch, yaw, throttle.
    pub controls: [f64; 4],
    /// JSBSim normalized positions: elevator, left aileron, rudder.
    pub surfaces: [f64; 3],
}

/// What one hold saw, for the live verdict line.
#[derive(Clone, Copy, Debug, Default)]
struct HoldSeen {
    best: f64,
    opposite: f64,
    surface_at_best: f64,
    n1_first: Option<f64>,
    n1_last: f64,
}

pub struct Prompter {
    pub phase: Phase,
    vr_kt: f64,
    aircraft: &'static str,
    entered_s: f64,
    /// Running seconds accumulated in the current timed phase.
    running_s: f64,
    last_t_s: Option<f64>,
    said_paused: bool,
    last_status_s: f64,
    said_brake_release: bool,
    said_brake_hint: bool,
    said_vr: bool,
    said_liftoff: bool,
    said_500: bool,
    nudged_input: bool,
    greeted: bool,
    seen: HoldSeen,
    free_flight: bool,
    /// SPIKE-002 showcase: after Start, skip calibration/sign and deliberate
    /// disconnect exercises and give the Chief the whole window to fly.
    direct_free_flight: bool,
    generation: Option<u64>,
    /// UnplugTest: input loss seen (then input valid and running again).
    unplug_lost: bool,
    title: Option<(&'static str, &'static str)>,
}

impl Prompter {
    pub fn new(aircraft: &'static str, vr_kt: f64) -> Self {
        Self {
            phase: Phase::Connect,
            vr_kt,
            aircraft,
            entered_s: 0.0,
            running_s: 0.0,
            last_t_s: None,
            said_paused: false,
            last_status_s: 0.0,
            said_brake_release: false,
            said_brake_hint: false,
            said_vr: false,
            said_liftoff: false,
            said_500: false,
            nudged_input: false,
            greeted: false,
            seen: HoldSeen::default(),
            free_flight: false,
            direct_free_flight: false,
            generation: None,
            unplug_lost: false,
            title: None,
        }
    }

    /// The step name of the first prompt, plus an extra sentence after it.
    pub fn titled(mut self, title: &'static str, hint: &'static str) -> Self {
        self.title = Some((title, hint));
        self
    }

    /// S3 script: no end at 1,500 ft; unplug test, then free flight until
    /// the time is up.
    pub fn free_flight(mut self) -> Self {
        self.free_flight = true;
        self
    }

    /// Visible showcase script: Start goes directly to free flight. Input
    /// loss safety remains active in the runtime; only the deliberate test is
    /// removed.
    pub fn direct_free_flight(mut self) -> Self {
        self.free_flight = true;
        self.direct_free_flight = true;
        self
    }

    fn enter(&mut self, phase: Phase, t_s: f64, out: &mut Vec<(Phase, String)>, text: String) {
        self.phase = phase;
        self.entered_s = t_s;
        self.running_s = 0.0;
        self.said_paused = false;
        out.push((phase, text));
    }

    fn observe_hold(&mut self, hold: Hold, o: &Obs) {
        let (idx, sign, surf) = hold.expectation();
        let v = o.controls[idx];
        // Throttle is unipolar: "towards" is 1 (up) or 0 (down).
        let toward = if idx == 3 { if sign > 0.0 { v } else { 1.0 - v } } else { v * sign };
        let away = if idx == 3 { 0.0 } else { -v * sign };
        let s = &mut self.seen;
        if toward > s.best {
            s.best = toward;
            s.surface_at_best = surf.map_or(0.0, |(k, _)| o.surfaces[k]);
        }
        s.opposite = s.opposite.max(away);
        s.n1_first.get_or_insert(o.n1);
        s.n1_last = o.n1;
    }

    fn verdict(&self, hold: Hold) -> String {
        let (idx, _, surf) = hold.expectation();
        let s = &self.seen;
        let names = ["roll", "pitch", "yaw", "throttle"];
        if idx < 3 && s.opposite >= 0.5 && s.best < 0.5 {
            return format!(
                "  !! {} input read the OPPOSITE way ({:+.2}): axis direction reversed - note it and carry on",
                names[idx], -s.opposite
            );
        }
        if s.best < 0.5 {
            return format!("  !! {} input not held (reached {:.2} of full): the checker may call this hold MISSING", names[idx], s.best);
        }
        match surf {
            Some((k, want)) => {
                let label = ["elevator", "aileron", "rudder"][k];
                let ok = s.surface_at_best * want >= 0.3;
                format!(
                    "  seen: {} input {:.2} of full -> {label} {:+.2} {}",
                    names[idx],
                    s.best,
                    s.surface_at_best,
                    if ok { "(sign OK)" } else { "!! WRONG SIGN or no response" }
                )
            }
            None => format!(
                "  seen: throttle held, N1 {:.0}% -> {:.0}%",
                s.n1_first.unwrap_or(0.0),
                s.n1_last
            ),
        }
    }

    fn status_line(o: &Obs) -> String {
        // WOW is meaningless in a reset/paused snapshot (JSBSim computes it
        // in a step, not in RunIC): never call a stopped aircraft AIRBORNE.
        let air = if o.run != Some(RunState::Running) {
            "NOT RUNNING"
        } else if o.wow {
            "ON GROUND"
        } else {
            "AIRBORNE "
        };
        format!(
            "  IAS {:5.1} kt  GS {:5.1} kt  HDG {:03.0}  AGL {:6.0} ft  VS {:+6.0} fpm  PITCH {:+5.1}  N1 {:5.1}%  {}  PARK {}",
            o.ias_kt,
            o.gs_kt,
            o.hdg_deg.rem_euclid(360.0),
            o.agl_ft,
            o.vs_fpm,
            o.pitch_deg,
            o.n1,
            air,
            if o.parking_brake { "SET" } else { "off" }
        )
    }

    /// Advance on one observation; returns `(phase, line)` prompts.
    pub fn step(&mut self, o: &Obs) -> Vec<(Phase, String)> {
        let mut out = Vec::new();
        let t_s = o.t_ns as f64 / 1e9;
        let dt = self.last_t_s.map_or(0.0, |l| (t_s - l).max(0.0));
        self.last_t_s = Some(t_s);
        let running = o.run == Some(RunState::Running);
        if running {
            self.running_s += dt;
        }
        if o.t_ns >= o.end_ns && self.phase != Phase::Done {
            if self.phase != Phase::End {
                let p = self.phase;
                out.push((p, format!("TIME UP during phase {}: the run ends now", p.name())));
            }
            self.phase = Phase::Done;
            return out;
        }
        // A Reset (new generation) re-arms the take-off call-outs; the reset
        // snapshot itself carries WOW false (ORG-024 false LIFT-OFF).
        if self.generation.is_some_and(|g| g != o.generation) {
            self.said_vr = false;
            self.said_liftoff = false;
            self.said_500 = false;
            if matches!(self.phase, Phase::Takeoff | Phase::UnplugTest | Phase::Free) {
                out.push((self.phase, "  RESET: the aircraft is back on the runway threshold; pull the trigger to continue".into()));
            }
        }
        self.generation = Some(o.generation);
        // Timed phases count running time only; a pause is announced.
        let timed = matches!(self.phase, Phase::Ready(_) | Phase::Sign(_) | Phase::Lineup | Phase::Takeoff | Phase::Free);
        if timed && !running && !self.said_paused {
            self.said_paused = true;
            out.push((self.phase, "  the sim is NOT running (paused?): press START (trigger) to continue".into()));
        } else if running {
            self.said_paused = false;
        }
        match self.phase {
            Phase::Connect => {
                if !self.greeted {
                    self.greeted = true;
                    let (title, hint) = self.title.unwrap_or((
                        if self.free_flight { "S3 FIRST PIXELS" } else { "S2 HEADLESS FLIGHT" },
                        "",
                    ));
                    out.push((Phase::Connect, format!(
                        "{}: JSBSim {} at the SFO synthetic runway threshold. \
                         Wiggle the stick so the input becomes valid.{}{}",
                        title,
                        self.aircraft,
                        if hint.is_empty() { "" } else { " " },
                        hint
                    )));
                }
                if o.status == Some(InputStatus::Valid) {
                    self.enter(Phase::Start, t_s, &mut out,
                        "PHASE START: press button 1 (the trigger). The parking brake starts SET.".into());
                } else if t_s - self.entered_s > 10.0 && !self.nudged_input {
                    self.nudged_input = true;
                    out.push((Phase::Connect, "  input not valid yet: is the stick plugged in? move it a little".into()));
                }
            }
            Phase::Start => {
                if running {
                    if self.direct_free_flight {
                        self.last_status_s = t_s;
                        self.enter(Phase::Free, t_s, &mut out, format!(
                            "PHASE FREE FLIGHT: release parking brake with button 4, take off at VR {:.0} kt, and fly until time expires. N1 and vertical speed are on the HUD.",
                            self.vr_kt
                        ));
                    } else {
                        self.enter(Phase::Ready(0), t_s, &mut out, format!(
                            "PHASE SIGN CHECK (about 60 s): 8 holds of {HOLD_S} s, one input at a time. Centre everything now. \
                             First: {}",
                            HOLDS[0].instruction()
                        ));
                    }
                }
            }
            Phase::Ready(i) => {
                if self.running_s >= READY_S {
                    self.enter(Phase::Sign(i), t_s, &mut out, format!("  HOLD NOW ({}/8): {}", i + 1, HOLDS[i].instruction()));
                }
            }
            Phase::Sign(i) => {
                if running {
                    self.observe_hold(HOLDS[i], o);
                }
                if self.running_s >= HOLD_S {
                    out.push((Phase::Sign(i), self.verdict(HOLDS[i])));
                    self.seen = HoldSeen::default();
                    if i + 1 < HOLDS.len() {
                        let prep = if HOLDS[i + 1] == Hold::ThrottleUp {
                            "RELEASE the stick and pull the throttle fully BACK (idle) now"
                        } else {
                            "RELEASE and centre"
                        };
                        self.enter(Phase::Ready(i + 1), t_s, &mut out, format!(
                            "  {prep}. Next: {}", HOLDS[i + 1].instruction()));
                    } else {
                        self.enter(Phase::Lineup, t_s, &mut out, format!(
                            "PHASE LINE-UP ({LINEUP_S:.0} s): release everything, stick centred, throttle fully BACK (idle). \
                             Next: take-off. Rotate speed VR = {:.0} kt.",
                            self.vr_kt
                        ));
                    }
                }
            }
            Phase::Lineup => {
                if self.running_s >= LINEUP_S {
                    self.last_status_s = t_s;
                    self.enter(Phase::Takeoff, t_s, &mut out, format!(
                        "PHASE TAKE-OFF (up to {TAKEOFF_S:.0} s): 1) press numbered button 4 to RELEASE the parking brake, \
                         2) throttle fully FORWARD, 3) keep the heading with twist (rudder), 4) at VR {:.0} kt \
                         pull back gently to about 10 deg pitch, 5) climb. Hat back = wheel brakes (left/right = one side).",
                        self.vr_kt
                    ));
                }
            }
            Phase::Takeoff => {
                if !o.parking_brake && !self.said_brake_release {
                    self.said_brake_release = true;
                    out.push((Phase::Takeoff, "  parking brake released".into()));
                }
                if o.parking_brake && !self.said_brake_hint && self.running_s > 10.0 {
                    self.said_brake_hint = true;
                    out.push((Phase::Takeoff, "  parking brake still SET: press numbered button 4".into()));
                }
                if running && o.ias_kt >= self.vr_kt && o.wow && !self.said_vr {
                    self.said_vr = true;
                    out.push((Phase::Takeoff, format!("  VR {:.0} kt: ROTATE NOW, pull back gently to ~10 deg pitch", self.vr_kt)));
                }
                // Lift-off needs a running sim and take-off speed: a reset or
                // paused snapshot reports WOW false with the gear on the runway.
                let flying = flying(running, o, self.vr_kt);
                if flying && o.agl_ft > 5.0 && !self.said_liftoff {
                    self.said_liftoff = true;
                    out.push((Phase::Takeoff, "  LIFT-OFF: hold about 10 deg pitch and climb, wings level".into()));
                }
                if flying && o.agl_ft > 500.0 && !self.said_500 {
                    self.said_500 = true;
                    out.push((Phase::Takeoff, "  500 ft: climbing, keep going".into()));
                }
                if t_s - self.last_status_s >= STATUS_EVERY_S {
                    self.last_status_s = t_s;
                    out.push((Phase::Takeoff, Self::status_line(o)));
                }
                if self.free_flight {
                    if self.running_s >= UNPLUG_AFTER_S || (flying && o.agl_ft >= END_AGL_FT) {
                        self.unplug_lost = false;
                        self.enter(Phase::UnplugTest, t_s, &mut out,
                            "PHASE UNPLUG TEST: unplug the stick USB cable now. Watch the red STICK LOST banner; \
                             after 10 s the sim pauses. Then plug the stick back in and pull the trigger."
                                .into());
                    }
                } else if self.running_s >= TAKEOFF_S || (flying && o.agl_ft >= END_AGL_FT) {
                    self.enter(Phase::End, t_s, &mut out,
                        format!("PHASE END: done ({}). Let go of the stick; the run stops in a few seconds.", Self::status_line(o).trim()));
                }
            }
            Phase::UnplugTest => {
                if o.status.is_some_and(|s| s != InputStatus::Valid) {
                    if !self.unplug_lost {
                        out.push((Phase::UnplugTest, "  stick lost: keep it unplugged until the sim pauses (10 s)".into()));
                    }
                    self.unplug_lost = true;
                }
                let back = o.status == Some(InputStatus::Valid) && running;
                if self.unplug_lost && back {
                    self.last_status_s = t_s;
                    self.enter(Phase::Free, t_s, &mut out,
                        "PHASE FREE FLIGHT: unplug test done. Fly as you like until the time is up.".into());
                } else if !self.unplug_lost && t_s - self.entered_s >= UNPLUG_WAIT_S {
                    self.last_status_s = t_s;
                    self.enter(Phase::Free, t_s, &mut out,
                        "PHASE FREE FLIGHT: no unplug seen (skipped). Fly as you like until the time is up.".into());
                }
            }
            Phase::Free => {
                if flying(running, o, self.vr_kt) && o.agl_ft > 5.0 && !self.said_liftoff {
                    self.said_liftoff = true;
                    out.push((Phase::Free, "  LIFT-OFF: climb, wings level".into()));
                }
                if t_s - self.last_status_s >= STATUS_EVERY_S {
                    self.last_status_s = t_s;
                    out.push((Phase::Free, Self::status_line(o)));
                }
            }
            Phase::End => {
                if t_s - self.entered_s >= 3.0 {
                    self.phase = Phase::Done;
                }
            }
            Phase::Done => {}
        }
        out
    }
}

/// Airborne for the call-outs: a running sim, WOW false and at least half
/// the rotate speed (a reset snapshot has WOW false on the runway).
fn flying(running: bool, o: &Obs, vr_kt: f64) -> bool {
    running && !o.wow && o.ias_kt >= 0.5 * vr_kt
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs(t_s: f64) -> Obs {
        Obs {
            t_ns: (t_s * 1e9) as u64,
            end_ns: 400_000_000_000,
            connected: true,
            run: Some(RunState::Paused),
            status: Some(InputStatus::Valid),
            wow: true,
            ..Obs::default()
        }
    }

    #[test]
    fn full_session_timeline() {
        let mut p = Prompter::new("global5000", 108.0);
        let mut names = Vec::new();
        let mut t = 0.0;
        let mut o = obs(t);
        o.status = Some(InputStatus::NoInput);
        for (ph, _) in p.step(&o) {
            names.push(ph.name());
        }
        let run = |p: &mut Prompter, o: &mut Obs, t: &mut f64, secs: f64, names: &mut Vec<&'static str>| {
            let end = *t + secs;
            while *t < end {
                *t += 0.1;
                o.t_ns = (*t * 1e9) as u64;
                for (ph, _) in p.step(o) {
                    if names.last() != Some(&ph.name()) {
                        names.push(ph.name());
                    }
                }
            }
        };
        o.status = Some(InputStatus::Valid);
        run(&mut p, &mut o, &mut t, 1.0, &mut names);
        assert_eq!(p.phase, Phase::Start);
        o.run = Some(RunState::Running);
        run(&mut p, &mut o, &mut t, 8.0 * (READY_S + HOLD_S) + 1.0, &mut names);
        assert_eq!(p.phase, Phase::Lineup);
        run(&mut p, &mut o, &mut t, LINEUP_S + 0.5, &mut names);
        assert_eq!(p.phase, Phase::Takeoff);
        o.ias_kt = 110.0;
        run(&mut p, &mut o, &mut t, 2.0, &mut names);
        o.wow = false;
        o.agl_ft = 1_600.0;
        run(&mut p, &mut o, &mut t, 4.0, &mut names);
        assert_eq!(p.phase, Phase::Done);
        let expected: Vec<&str> = ["connect", "start"]
            .into_iter()
            .chain(HOLDS.iter().flat_map(|h| ["sign_ready", h.name()]))
            .chain(["lineup", "takeoff", "end"])
            .collect();
        assert_eq!(names, expected);
        // The sign check takes about a minute.
        assert!((8.0 * (READY_S + HOLD_S) - 60.0).abs() <= 2.0);
    }

    #[test]
    fn holds_count_running_time_only_and_pauses_are_announced() {
        let mut p = Prompter::new("global5000", 108.0);
        let mut o = obs(0.0);
        p.step(&o);
        o.t_ns = 100_000_000;
        p.step(&o);
        o.run = Some(RunState::Running);
        o.t_ns = 200_000_000;
        p.step(&o);
        assert_eq!(p.phase, Phase::Ready(0));
        o.run = Some(RunState::Paused);
        let mut said = false;
        for k in 0..100 {
            o.t_ns = 300_000_000 + k * 100_000_000;
            said |= p.step(&o).iter().any(|(_, s)| s.contains("NOT running"));
        }
        assert!(said);
        assert_eq!(p.phase, Phase::Ready(0), "10 s paused does not advance the hold");
    }

    #[test]
    fn live_verdicts_flag_a_reversed_axis_and_confirm_a_good_one() {
        let mut p = Prompter::new("global5000", 108.0);
        let mut o = obs(0.0);
        p.step(&o);
        o.t_ns = 100_000_000;
        p.step(&o);
        o.run = Some(RunState::Running);
        let mut lines = Vec::new();
        let mut t = 0.2;
        // Hold 1 (pull back) reads the opposite way; hold 2 (push) is right.
        while lines.len() < 2 && t < 30.0 {
            o.t_ns = (t * 1e9) as u64;
            match p.phase {
                Phase::Sign(0) => { o.controls[1] = -1.0; o.surfaces[0] = 0.9; }
                Phase::Sign(1) => { o.controls[1] = -1.0; o.surfaces[0] = 0.9; }
                _ => { o.controls[1] = 0.0; o.surfaces[0] = 0.0; }
            }
            for (_, s) in p.step(&o) {
                if s.contains("seen:") || s.contains("!!") {
                    lines.push(s);
                }
            }
            t += 0.1;
        }
        assert!(lines[0].contains("OPPOSITE"), "{lines:?}");
        assert!(lines[1].contains("elevator +0.90 (sign OK)"), "{lines:?}");
    }

    fn to_takeoff(p: &mut Prompter, o: &mut Obs, t: &mut f64) {
        o.status = Some(InputStatus::Valid);
        o.run = Some(RunState::Running);
        while p.phase != Phase::Takeoff {
            *t += 0.1;
            o.t_ns = (*t * 1e9) as u64;
            p.step(o);
            assert!(*t < 200.0, "never reached take-off");
        }
    }

    /// ORG-024 / TEST-034: Pause then Reset during the take-off phase must not
    /// announce LIFT-OFF or AIRBORNE (the reset snapshot has WOW false and the
    /// CG height above 5 ft), and a later real lift-off is still announced.
    #[test]
    fn reset_during_takeoff_is_not_a_lift_off() {
        let mut p = Prompter::new("global5000", 108.0);
        let mut o = obs(0.0);
        let mut t = 0.0;
        to_takeoff(&mut p, &mut o, &mut t);
        let mut lines = Vec::new();
        let step = |p: &mut Prompter, o: &mut Obs, t: &mut f64, secs: f64, lines: &mut Vec<String>| {
            let end = *t + secs;
            while *t < end {
                *t += 0.1;
                o.t_ns = (*t * 1e9) as u64;
                lines.extend(p.step(o).into_iter().map(|(_, s)| s));
            }
        };
        o.agl_ft = 8.0;
        step(&mut p, &mut o, &mut t, 1.0, &mut lines);
        o.run = Some(RunState::Paused);
        step(&mut p, &mut o, &mut t, 1.0, &mut lines);
        o.generation += 1;
        o.wow = false;
        step(&mut p, &mut o, &mut t, 6.0, &mut lines);
        assert!(lines.iter().all(|l| !l.contains("LIFT-OFF") && !l.contains("AIRBORNE")), "{lines:#?}");
        assert!(lines.iter().any(|l| l.contains("RESET")), "{lines:#?}");
        // Running again on the runway after Start: WOW true, then a real lift-off.
        o.run = Some(RunState::Running);
        o.wow = true;
        step(&mut p, &mut o, &mut t, 1.0, &mut lines);
        o.ias_kt = 120.0;
        o.wow = false;
        o.agl_ft = 20.0;
        step(&mut p, &mut o, &mut t, 1.0, &mut lines);
        assert_eq!(lines.iter().filter(|l| l.contains("LIFT-OFF")).count(), 1, "{lines:#?}");
    }

    #[test]
    fn free_flight_script_runs_the_unplug_test_and_never_ends_early() {
        let mut p = Prompter::new("global5000", 108.0).free_flight();
        let mut o = obs(0.0);
        let mut t = 0.0;
        to_takeoff(&mut p, &mut o, &mut t);
        let mut names = Vec::new();
        let step = |p: &mut Prompter, o: &mut Obs, t: &mut f64, secs: f64, names: &mut Vec<&'static str>| {
            let end = *t + secs;
            while *t < end {
                *t += 0.1;
                o.t_ns = (*t * 1e9) as u64;
                for (ph, _) in p.step(o) {
                    if names.last() != Some(&ph.name()) {
                        names.push(ph.name());
                    }
                }
            }
        };
        // Airborne above 1,500 ft: S2 would end here; S3 asks for the unplug.
        o.ias_kt = 160.0;
        o.wow = false;
        o.agl_ft = 1_600.0;
        step(&mut p, &mut o, &mut t, 1.0, &mut names);
        assert_eq!(p.phase, Phase::UnplugTest);
        o.status = Some(InputStatus::Unavailable(gx_sim::UnavailableReason::Disconnected));
        step(&mut p, &mut o, &mut t, 10.0, &mut names);
        o.run = Some(RunState::Paused);
        step(&mut p, &mut o, &mut t, 3.0, &mut names);
        o.status = Some(InputStatus::Valid);
        step(&mut p, &mut o, &mut t, 2.0, &mut names);
        assert_eq!(p.phase, Phase::UnplugTest, "still paused until TRIGGER");
        o.run = Some(RunState::Running);
        step(&mut p, &mut o, &mut t, 100.0, &mut names);
        assert_eq!(p.phase, Phase::Free);
        assert_eq!(names, ["takeoff", "unplug_test", "free"]);
    }

    #[test]
    fn direct_free_flight_skips_all_sign_and_disconnect_exercises() {
        let mut p = Prompter::new("global5000", 108.0).direct_free_flight();
        let mut o = obs(0.0);
        let first = p.step(&o);
        assert!(first.iter().any(|(ph, _)| *ph == Phase::Connect));
        o.t_ns = 100_000_000;
        p.step(&o);
        assert_eq!(p.phase, Phase::Start);
        o.run = Some(RunState::Running);
        o.t_ns = 200_000_000;
        let start = p.step(&o);
        assert_eq!(p.phase, Phase::Free);
        assert!(start.iter().any(|(ph, text)| *ph == Phase::Free && text.contains("N1")));
        o.t_ns = 299_000_000_000;
        let lines = p.step(&o);
        assert_eq!(p.phase, Phase::Free);
        assert!(lines.iter().all(|(ph, text)| {
            !matches!(ph, Phase::Ready(_) | Phase::Sign(_) | Phase::UnplugTest)
                && !text.contains("UNPLUG")
        }));
        o.t_ns = o.end_ns;
        p.step(&o);
        assert_eq!(p.phase, Phase::Done);
    }

    #[test]
    fn time_up_is_reported_with_the_phase() {
        let mut p = Prompter::new("c172p", 55.0);
        let mut o = obs(0.0);
        o.end_ns = 1_000_000_000;
        p.step(&o);
        o.t_ns = 1_000_000_000;
        let out = p.step(&o);
        assert!(out.iter().any(|(_, s)| s.contains("TIME UP during phase")), "{out:?}");
        assert_eq!(p.phase, Phase::Done);
    }
}

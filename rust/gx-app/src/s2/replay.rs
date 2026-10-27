//! Offline replay of an S2 run: feed the logged mailbox records, at the
//! logged read times, into a fresh mailbox and the same `SimDriver` code over
//! a freshly loaded backend, and compare every tick's state hash.
//!
//! Why bit-exact is expected: the trajectory is a pure function of the
//! initial state and the per-tick inputs to `SimCore` (controls, commands,
//! input status, loss times). The log holds the exact reader inputs (each
//! new `ControlRecord`, float bits preserved, and the wake time that the
//! staleness and INPUT-002 timers use) and the console Starts, so the replay
//! reader returns identical `TickControls` (tested in gx-sim). JSBSim is
//! deterministic for identical inputs in the same binary on the same CPU:
//! its RNG is seeded with a constant (0), it reads no clock, and the same
//! machine code performs the same IEEE operations. Hence a replay on the
//! board with the same `gx-app`/`libJSBSim.so.1` must match every hash; any
//! difference is reported as a mismatch, never tolerated.

use crate::s1::driver::SimDriver;
use crate::s1::rec::{record_from_items_with, Rec};
use crate::s2::setup::Aircraft;
use crate::SnapshotSink;
use gx_sim::{control_mailbox, AircraftSnapshot, Approach, ControlRecord, Levers, PublishBusy, SimulationBackend};
use std::io::BufRead;

/// One `tick` record of the log.
#[derive(Clone, Debug, PartialEq)]
pub struct LoggedTick {
    pub n: u64,
    pub wake_ns: u64,
    pub console: u64,
    pub record: Option<ControlRecord>,
    pub hash: Option<u64>,
}

/// The `start` record fields replay needs.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LoggedStart {
    pub schema: String,
    pub commit: String,
    pub aircraft: String,
    pub backend: String,
}

fn raw<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat)? + pat.len();
    let rest = &line[i..];
    let end = if rest.starts_with('[') {
        rest.find(']')? + 1
    } else if let Some(stripped) = rest.strip_prefix('"') {
        stripped.find('"')? + 2
    } else {
        rest.find([',', '}'])?
    };
    Some(&rest[..end])
}

fn string(line: &str, key: &str) -> Option<String> {
    raw(line, key).map(|v| v.trim_matches('"').to_owned())
}

/// The levers a logged run started with (and that a pre-lever log held
/// throughout): the aircraft's [`Aircraft::initial_levers`].
pub fn initial_levers(start: &LoggedStart) -> Levers {
    Aircraft::parse(&start.aircraft).map_or(Levers::default(), Aircraft::initial_levers)
}

pub fn parse_tick(line: &str) -> Result<LoggedTick, String> {
    parse_tick_with(line, Levers::default())
}

/// [`parse_tick`] for a run that started with `initial` levers.
pub fn parse_tick_with(line: &str, initial: Levers) -> Result<LoggedTick, String> {
    let num = |k: &str| -> Result<u64, String> {
        raw(line, k).and_then(|v| v.parse().ok()).ok_or_else(|| format!("tick record without {k}: {line}"))
    };
    let record = match raw(line, "r") {
        None => None,
        Some(arr) => {
            let items: Vec<&str> = arr.trim_matches(|c| c == '[' || c == ']').split(',').collect();
            Some(record_from_items_with(&items, initial).ok_or_else(|| format!("bad control record: {line}"))?)
        }
    };
    let hash = match raw(line, "h") {
        None => None,
        Some(h) => Some(u64::from_str_radix(h.trim_matches('"'), 16).map_err(|_| format!("bad hash: {line}"))?),
    };
    Ok(LoggedTick { n: num("n")?, wake_ns: num("w")?, console: num("c")?, record, hash })
}

/// The start record and every tick record, in log order.
pub fn read_log(reader: impl BufRead) -> Result<(LoggedStart, Vec<LoggedTick>), String> {
    let mut start = None;
    let mut ticks = Vec::new();
    for line in reader.lines() {
        let line = line.map_err(|e| e.to_string())?;
        if line.starts_with("{\"k\":\"start\"") {
            start = Some(LoggedStart {
                schema: string(&line, "schema").unwrap_or_default(),
                commit: string(&line, "commit").unwrap_or_default(),
                aircraft: string(&line, "aircraft").unwrap_or_default(),
                backend: string(&line, "backend").unwrap_or_default(),
            });
        } else if line.starts_with("{\"k\":\"tick\"") {
            let initial = start.as_ref().map_or(Levers::default(), initial_levers);
            ticks.push(parse_tick_with(&line, initial)?);
        }
    }
    Ok((start.ok_or("no start record")?, ticks))
}

/// Replay outcome. `mismatches == 0 && gaps == 0` means the replay
/// reproduced every logged state hash bit-exactly.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReplayResult {
    pub ticks: u64,
    pub hashed: u64,
    pub mismatches: u64,
    pub first_mismatch_n: Option<u64>,
    /// Missing tick record numbers (dropped log records).
    pub gaps: u64,
    pub final_hash: Option<u64>,
}

struct NullSink;

impl SnapshotSink for NullSink {
    fn try_publish(&self, _snapshot: AircraftSnapshot) -> Result<(), PublishBusy> {
        Ok(())
    }
}

/// Replay `ticks` through a fresh mailbox, `SimDriver` and `backend`, whose
/// core starts at the run's `initial` levers ([`initial_levers`]), with no
/// ILS approaches.
pub fn replay<B>(backend: B, initial: Levers, ticks: &[LoggedTick]) -> Result<ReplayResult, String>
where
    B: SimulationBackend,
    B::Error: std::fmt::Debug,
{
    replay_with_approaches(backend, initial, Vec::new(), ticks)
}

/// [`replay`] of a run whose autopilot had `approaches` (the run's
/// `s2::setup::approaches` of the same world pack): the ILS selection and
/// track depend on them, so a run with an ILS replays bit-exactly only with
/// the same list.
pub fn replay_with_approaches<B>(
    backend: B,
    initial: Levers,
    approaches: Vec<Approach>,
    ticks: &[LoggedTick],
) -> Result<ReplayResult, String>
where
    B: SimulationBackend,
    B::Error: std::fmt::Debug,
{
    let (mut writer, reader) = control_mailbox();
    let t0 = ticks.first().map_or(0, |t| t.wake_ns);
    let mut driver = SimDriver::try_new_with_levers(backend, NullSink, reader, t0, initial, &mut |_| {})
        .map_err(|e| format!("initial reset: {e:?}"))?
        .with_approaches(approaches)
        .with_trace();
    let mut result = ReplayResult::default();
    let mut expected_n = 1;
    for t in ticks {
        if t.n != expected_n {
            result.gaps += t.n.saturating_sub(expected_n);
        }
        expected_n = t.n + 1;
        if let Some(r) = t.record {
            if !writer.publish_record(r) {
                return Err(format!("tick {}: control record out of order", t.n));
            }
        }
        let mut got = None;
        let wake = t.wake_ns;
        driver.tick(wake, t.console, &mut || wake, &mut |r| {
            if let Rec::Tick(k) = r {
                got = Some(k.hash);
            }
        });
        let got = got.ok_or("driver emitted no tick record")?;
        result.ticks += 1;
        if t.hash.is_some() {
            result.hashed += 1;
        }
        if got != t.hash {
            result.mismatches += 1;
            result.first_mismatch_n.get_or_insert(t.n);
        }
        if got.is_some() {
            result.final_hash = got;
        }
    }
    Ok(result)
}

impl ReplayResult {
    pub fn to_json(&self, log_sha256: &str, start: &LoggedStart, commit: &str) -> String {
        format!(
            "{{\"schema\":\"gx-s2-replay/1\",\"log_sha256\":\"{log_sha256}\",\"log_commit\":\"{}\",\"replay_commit\":\"{commit}\",\"aircraft\":\"{}\",\"backend\":\"{}\",\"ticks\":{},\"hashed_ticks\":{},\"mismatches\":{},\"first_mismatch_n\":{},\"gaps\":{},\"final_hash\":{},\"exact\":{}}}",
            start.commit,
            start.aircraft,
            start.backend,
            self.ticks,
            self.hashed,
            self.mismatches,
            self.first_mismatch_n.map_or("null".into(), |n| n.to_string()),
            self.gaps,
            self.final_hash.map_or("null".into(), |h| format!("\"{h:016x}\"")),
            self.mismatches == 0 && self.gaps == 0 && self.ticks > 0,
        )
    }
}

/// SHA-256 of a byte string (FIPS 180-4), so the replay result names the
/// exact log it replayed without a dependency.
pub fn sha256_hex(data: &[u8]) -> String {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    ];
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    ];
    let mut msg = data.to_vec();
    let bits = (data.len() as u64).wrapping_mul(8);
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bits.to_be_bytes());
    for block in msg.chunks(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([block[4 * i], block[4 * i + 1], block[4 * i + 2], block[4 * i + 3]]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16].wrapping_add(s0).wrapping_add(w[i - 7]).wrapping_add(s1);
        }
        let mut v = h;
        for i in 0..64 {
            let s1 = v[4].rotate_right(6) ^ v[4].rotate_right(11) ^ v[4].rotate_right(25);
            let ch = (v[4] & v[5]) ^ (!v[4] & v[6]);
            let t1 = v[7].wrapping_add(s1).wrapping_add(ch).wrapping_add(K[i]).wrapping_add(w[i]);
            let s0 = v[0].rotate_right(2) ^ v[0].rotate_right(13) ^ v[0].rotate_right(22);
            let maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
            let t2 = s0.wrapping_add(maj);
            v = [t1.wrapping_add(t2), v[0], v[1], v[2], v[3].wrapping_add(t1), v[4], v[5], v[6]];
        }
        for i in 0..8 {
            h[i] = h[i].wrapping_add(v[i]);
        }
    }
    h.iter().map(|x| format!("{x:08x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_known_answers() {
        assert_eq!(sha256_hex(b""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
        assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    }

    use crate::s1::driver::TICK_NS;
    use crate::s1::rec::to_json;
    use gx_sim::{
        AircraftState, ApAction, ApInput, CommandEdges, ControlInput, GeodeticPosition, LocalEnuPose, UnavailableReason,
    };

    /// An airborne point mass with a valid world pose: the stick rolls and
    /// pitches it, the throttle sets the speed, the attitude moves it.
    struct Glider {
        s: AircraftState,
    }

    impl Glider {
        fn new() -> Self {
            let mut g = Self { s: AircraftState::default() };
            g.place();
            g
        }

        fn place(&mut self) {
            self.s = AircraftState { height_agl_m: 1000.0, calibrated_airspeed_mps: 120.0, ..AircraftState::default() };
            self.s.local_enu = Some(LocalEnuPose { frame_id: 1, orientation_xyzw: [0.0, 0.0, 0.0, 1.0], ..LocalEnuPose::default() });
            self.pose();
        }

        fn pose(&mut self) {
            let p = self.s.local_enu.as_mut().unwrap();
            self.s.geodetic = GeodeticPosition {
                latitude_deg: 37.0 + p.position_m[1] / 111_320.0,
                longitude_deg: -122.0 + p.position_m[0] / 88_900.0,
                ellipsoid_height_m: self.s.height_agl_m,
                ..GeodeticPosition::default()
            };
        }
    }

    impl SimulationBackend for Glider {
        type Error = ();

        fn reset(&mut self) -> Result<AircraftState, ()> {
            self.place();
            Ok(self.s)
        }

        fn step(&mut self, c: ControlInput, dt: f64) -> Result<AircraftState, ()> {
            let s = &mut self.s;
            s.angular_rate_rps = [0.3 * f64::from(c.roll), 0.1 * f64::from(c.pitch), 0.0];
            s.roll_rad += s.angular_rate_rps[0] * dt;
            s.pitch_rad += s.angular_rate_rps[1] * dt;
            s.heading_rad = (s.heading_rad + 0.1 * s.roll_rad.tan() * dt).rem_euclid(std::f64::consts::TAU);
            s.calibrated_airspeed_mps += (60.0 * f64::from(c.throttle) + 90.0 - s.calibrated_airspeed_mps) * 0.1 * dt;
            s.vertical_speed_mps = s.calibrated_airspeed_mps * s.pitch_rad.sin();
            s.height_agl_m += s.vertical_speed_mps * dt;
            let (sh, ch) = s.heading_rad.sin_cos();
            let v = s.calibrated_airspeed_mps;
            let p = s.local_enu.as_mut().unwrap();
            p.velocity_mps = [v * sh, v * ch, s.vertical_speed_mps];
            for i in 0..3 {
                p.position_m[i] += p.velocity_mps[i] * dt;
            }
            self.pose();
            Ok(self.s)
        }
    }

    /// An autopilot run (panel clicks, an ILS, a loss with the AP on, a
    /// stick override, then a loss with it off) logged through the JSONL encoding,
    /// parsed back and replayed: every tick's hash matches.
    #[test]
    fn an_autopilot_run_replays_bit_exactly_from_its_log() {
        let approaches = vec![Approach::new("05", [8_000.0, 8_000.0], 45.0)];
        let (mut writer, reader) = control_mailbox();
        let t0 = 1_000_000_000;
        let mut lines = Vec::new();
        let mut states = Vec::new();
        let mut driver = SimDriver::try_new_with_levers(Glider::new(), NullSink, reader, t0, Levers::default(), &mut |_| {})
            .unwrap()
            .with_approaches(approaches.clone())
            .with_trace();
        let mut panel = ApInput::DEFAULT;
        let mut engaged_ticks = 0;
        let mut lvl_on_loss = false;
        let mut localizer = false;
        let mut gs_descent = false;
        for k in 1..=2400u64 {
            let now = t0 + k * TICK_NS;
            let click = |panel: &mut ApInput, action, f: fn(&mut ApInput)| {
                panel.serial += 1;
                panel.action = action;
                f(panel);
            };
            match k {
                20 => click(&mut panel, ApAction::Heading, |p| p.heading_deg = 45.0),
                30 => click(&mut panel, ApAction::Speed, |p| p.speed_kt = 260.0),
                40 => click(&mut panel, ApAction::Altitude, |p| p.altitude_ft = 3500.0),
                900 => click(&mut panel, ApAction::Approach, |_| {}),
                _ => {}
            }
            let lost = (600..700).contains(&k) || (1700..1800).contains(&k);
            if lost {
                writer.publish_unavailable(UnavailableReason::Disconnected, k, now, now);
            } else {
                let pitch = if (1200..1210).contains(&k) { 0.5 } else { 0.0 };
                let c = ControlInput { pitch, throttle: 0.5, ap: panel, ..ControlInput::default() };
                writer.publish_valid(c, k, now, CommandEdges { start: k == 2, ..CommandEdges::default() }, now);
            }
            driver.tick(now, 0, &mut || now, &mut |r| {
                match r {
                    Rec::Tick(_) => lines.push(to_json(&r, 0)),
                    Rec::State(_) => states.push(to_json(&r, 0)),
                    _ => {}
                }
            });
            let Some(st) = driver.last_snapshot().map(|s| s.state.autopilot) else { continue };
            engaged_ticks += u64::from(st.engaged);
            localizer |= st.lateral == gx_sim::autopilot::Lateral::Localizer;
            gs_descent |= st.vertical == gx_sim::autopilot::Vertical::GsDescent && st.gs_dev_ft.is_some_and(|d| d > 98.0);
            lvl_on_loss |= st.warning == Some(gx_sim::autopilot::ApWarning::InputLossLevel);
        }
        assert!(engaged_ticks > 1000, "the autopilot flew: {engaged_ticks} ticks");
        assert!(lvl_on_loss, "the second loss (AP off) engaged LVL");
        assert!(localizer, "the ILS captured the localizer");
        assert!(gs_descent, "above the path on the localizer: G/S descent (AP-002 W5b)");
        // The 10 Hz state log carries the modes and the applied commands.
        assert!(states.iter().any(|l| l.contains(r#""ap":{"engaged":true,"lat":"localizer","vert":"gs_descent""#)), "{}", states[100]);
        assert!(states.iter().any(|l| l.contains(r#""gs_dev_ft":null"#)) && states.iter().any(|l| l.contains(r#""gs_dev_ft":1"#)));
        assert!(states.iter().any(|l| l.contains(r#""warning":"input_loss_level""#)));
        assert!(states.iter().all(|l| l.contains(r#""cmd":["#)));
        let ticks: Vec<LoggedTick> = lines.iter().map(|l| parse_tick(l).unwrap()).collect();
        assert!(ticks.iter().any(|t| t.record.is_some_and(|r| r.controls.ap.serial == 4)), "the panel is logged");
        let result = replay_with_approaches(Glider::new(), Levers::default(), approaches, &ticks).unwrap();
        assert_eq!((result.ticks, result.mismatches, result.gaps), (2400, 0, 0), "{result:?}");
        assert!(result.hashed > 2390, "{result:?}");
    }

    #[test]
    fn tick_lines_parse() {
        let t = parse_tick("{\"k\":\"tick\",\"t\":5,\"n\":3,\"w\":1000,\"c\":1,\"s\":800,\"h\":\"00000000000000ff\"}").unwrap();
        assert_eq!((t.n, t.wake_ns, t.console, t.record, t.hash), (3, 1000, 1, None, Some(255)));
        assert!(parse_tick("{\"k\":\"tick\",\"t\":5}").is_err());
    }
}

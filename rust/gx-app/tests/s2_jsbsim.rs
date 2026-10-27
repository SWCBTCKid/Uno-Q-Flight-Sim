//! S2 JSBSim tests: run on the board (AArch64, feature `jsbsim`) with
//! `GX_JSBSIM_ROOT` pointing at the staged JSBSim data root. The FFI crates
//! cannot link on the Windows host; `tools/build_m1_s2.ps1` cross-builds
//! this test and `tools/m1_s2_board_tests.sh` runs it.
#![cfg(feature = "jsbsim")]

use gx_app::s1::driver::state_hash;
use gx_app::s1::synthetic_world_frame;
use gx_app::s2::setup::{jsbsim_setup, threshold, Aircraft};
use gx_app::WorldFrame;
use gx_sim::{ControlInput, JsbsimBackend, SimulationBackend, STEP_SECONDS};
use std::path::PathBuf;

fn root() -> PathBuf {
    PathBuf::from(std::env::var("GX_JSBSIM_ROOT").expect("set GX_JSBSIM_ROOT to the staged JSBSim data root"))
}

fn load(aircraft: Aircraft) -> (JsbsimBackend, WorldFrame) {
    let frame = synthetic_world_frame().unwrap();
    let t = threshold(&frame).unwrap();
    let b = JsbsimBackend::load(&root(), &jsbsim_setup(aircraft, &t), frame.sim_frame()).expect("load");
    (b, frame)
}

/// Airborne at 3000 ft AGL, 200 kt (global5000) / 90 kt (c172p), heading 090.
fn airborne(aircraft: Aircraft, kt: f64) -> JsbsimBackend {
    let frame = synthetic_world_frame().unwrap();
    let t = threshold(&frame).unwrap();
    let mut setup = jsbsim_setup(aircraft, &t);
    setup.ic.height_agl_m = 3000.0 * 0.3048;
    setup.ic.airspeed_kts = kt;
    setup.settings.retain(|(k, _)| !k.starts_with("fcs/flap") && !k.starts_with("gear/"));
    setup.settings.push(("gear/gear-cmd-norm".into(), 0.0));
    setup.settings.push(("fcs/flap-cmd-norm".into(), 0.0));
    JsbsimBackend::load(&root(), &setup, frame.sim_frame()).expect("load airborne")
}

fn run(b: &mut JsbsimBackend, c: ControlInput, steps: usize) -> gx_sim::AircraftState {
    let mut s = b.reset().unwrap();
    for _ in 0..steps {
        s = b.step(c.normalized(), STEP_SECONDS).unwrap();
    }
    s
}

#[test]
fn global5000_starts_parked_on_the_threshold_in_the_pack_frame() {
    let csv = std::path::Path::new("global5000.csv");
    let _ = std::fs::remove_file(csv);
    let (mut b, frame) = load(Aircraft::Global5000);
    b.reset().unwrap();
    assert!(!csv.exists(), "JSBSim must not open a CSV log (the staged model has no <output> directive)");
    let t = threshold(&frame).unwrap();
    let s0 = b.reset().unwrap();
    let pose = s0.local_enu.expect("valid local_enu pose");
    assert_eq!((pose.frame_id, pose.frame_generation), (frame.frame_id(), frame.frame_generation()));
    assert!((pose.position_m[0] - t.enu_m[0]).abs() < 1.0 && (pose.position_m[1] - t.enu_m[1]).abs() < 1.0, "{pose:?}");
    assert!((s0.heading_rad.to_degrees() - 90.0).abs() < 0.1, "{}", s0.heading_rad.to_degrees());
    // Parked: parking brake (full brakes), idle, 10 s.
    // As SimCore drives it: the aircraft's initial levers (gear down, flaps 15).
    let parked = ControlInput { left_brake: 1.0, right_brake: 1.0, levers: Aircraft::Global5000.initial_levers(), ..ControlInput::default() };
    let s = run(&mut b, parked, 1200);
    let p = s.local_enu.unwrap();
    let moved = ((p.position_m[0] - pose.position_m[0]).powi(2) + (p.position_m[1] - pose.position_m[1]).powi(2)).sqrt();
    eprintln!(
        "parked 10 s: moved {moved:.3} m, agl {:.3} m, enu z {:.3} m (runway surface 0.2 m), wow {}, n1 {:.1}, flaps {:.2}, gear {:.2}",
        s.height_agl_m, p.position_m[2], s.weight_on_wheels, s.engine_n1, s.flap_position, s.gear_position
    );
    assert!(s.weight_on_wheels);
    assert!(moved < 0.5, "moved {moved} m while parked");
    assert!(s.engine_n1 > 20.0 && s.engine_n1 < 40.0, "engines running at idle: N1 {}", s.engine_n1);
    assert!(s.gear_position > 0.99);
    assert!((s.flap_position - 0.5).abs() < 0.01, "take-off flaps {}", s.flap_position);
    // Full throttle on the parking brake: N1 rises, still parked.
    let s = run(&mut b, ControlInput { throttle: 1.0, ..parked }, 1200);
    let p2 = s.local_enu.unwrap();
    let moved = ((p2.position_m[0] - pose.position_m[0]).powi(2) + (p2.position_m[1] - pose.position_m[1]).powi(2)).sqrt();
    eprintln!("full throttle on brakes 10 s: N1 {:.1}, moved {moved:.3} m", s.engine_n1);
    assert!(s.engine_n1 > 80.0, "throttle forward -> N1 up: {}", s.engine_n1);
}

#[test]
fn surface_and_body_rate_signs_in_the_air() {
    let mut b = airborne(Aircraft::Global5000, 200.0);
    let base = run(&mut b, ControlInput { throttle: 0.6, ..ControlInput::default() }, 60);
    let cases: [(&str, ControlInput, usize, f64); 6] = [
        ("pull back -> q > 0 (nose up)", ControlInput { pitch: 0.5, throttle: 0.6, ..ControlInput::default() }, 1, 1.0),
        ("push -> q < 0", ControlInput { pitch: -0.5, throttle: 0.6, ..ControlInput::default() }, 1, -1.0),
        ("right -> p > 0 (right roll)", ControlInput { roll: 0.5, throttle: 0.6, ..ControlInput::default() }, 0, 1.0),
        ("left -> p < 0", ControlInput { roll: -0.5, throttle: 0.6, ..ControlInput::default() }, 0, -1.0),
        ("twist right -> r > 0 (right yaw)", ControlInput { yaw: 0.5, throttle: 0.6, ..ControlInput::default() }, 2, 1.0),
        ("twist left -> r < 0", ControlInput { yaw: -0.5, throttle: 0.6, ..ControlInput::default() }, 2, -1.0),
    ];
    for (name, c, axis, sign) in cases {
        let s = run(&mut b, c, 60);
        let d = s.angular_rate_rps[axis] - base.angular_rate_rps[axis];
        let surf = [s.aileron_position, s.elevator_position, s.rudder_position][axis];
        eprintln!("{name}: delta rate {:+.4} deg/s, surface {:+.3}", d.to_degrees(), surf);
        assert!(d * sign > 0.0005, "{name}: delta {d}");
    }
    let s = run(&mut b, ControlInput { pitch: 1.0, ..ControlInput::default() }, 30);
    assert!(s.elevator_position < -0.5, "pull back = elevator TE up (negative): {}", s.elevator_position);
    let s = run(&mut b, ControlInput { roll: 1.0, ..ControlInput::default() }, 30);
    assert!(s.aileron_position > 0.5, "stick right = left aileron positive: {}", s.aileron_position);
    let s = run(&mut b, ControlInput { yaw: 1.0, ..ControlInput::default() }, 30);
    assert!(s.rudder_position < -0.5, "twist right = rudder negative (JSBSim right yaw): {}", s.rudder_position);
}

#[test]
fn identical_inputs_give_bit_identical_states_across_instances_and_resets() {
    let mut a = load(Aircraft::Global5000).0;
    let mut b = load(Aircraft::Global5000).0;
    let mut x = 0x1234_5678_u64;
    let mut inputs = Vec::new();
    for i in 0..2400 {
        x = x.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        let r = |k: u32| ((x >> k) & 0xffff) as f32 / 32768.0 - 1.0;
        inputs.push(ControlInput {
            roll: r(0), pitch: r(16), yaw: r(32) * 0.3, throttle: if i > 300 { 1.0 } else { 0.0 },
            left_brake: if i < 300 { 1.0 } else { 0.0 }, right_brake: if i < 300 { 1.0 } else { 0.0 },
            trim: 0.0, levers: Aircraft::Global5000.initial_levers(), ap: gx_sim::ApInput::default(),
        }.normalized());
    }
    let trace = |m: &mut JsbsimBackend| -> Vec<u64> {
        let mut h = vec![state_hash(&m.reset().unwrap())];
        h.extend(inputs.iter().map(|&c| state_hash(&m.step(c, STEP_SECONDS).unwrap())));
        h
    };
    let ha = trace(&mut a);
    let hb = trace(&mut b);
    assert_eq!(ha, hb, "two instances diverged");
    let ha2 = trace(&mut a);
    assert_eq!(ha, ha2, "the same instance diverged after a reset");
    assert!(ha.windows(2).filter(|w| w[0] != w[1]).count() > 2000, "the state actually evolves");
}

#[test]
fn c172p_fallback_loads_and_responds_to_throttle() {
    let (mut b, _) = load(Aircraft::C172p);
    // As SimCore drives it: the aircraft's initial levers (gear down, flaps 15).
    let parked = ControlInput { left_brake: 1.0, right_brake: 1.0, levers: Aircraft::Global5000.initial_levers(), ..ControlInput::default() };
    let idle = run(&mut b, parked, 600);
    let full = run(&mut b, ControlInput { throttle: 1.0, ..parked }, 600);
    eprintln!("c172p: idle {:.1}% rated rpm, full {:.1}%, wow {}", idle.engine_n1, full.engine_n1, full.weight_on_wheels);
    assert!(full.engine_n1 > idle.engine_n1 + 20.0);
    assert!(full.local_enu.is_some() && full.weight_on_wheels);
}

/// W1 (787 plan): the stock JSBSim 787-8 at the BBJ preset parks on the
/// threshold, takes off at full thrust rotating at VR, and its gear, flap
/// and speedbrake channels follow the levers.
#[test]
fn b7878_bbj_parks_takes_off_and_follows_the_levers() {
    use gx_sim::Levers;
    let (mut b, _) = load(Aircraft::B7878);
    let weight = b.property("inertia/weight-lbs").unwrap();
    let initial = Aircraft::B7878.initial_levers();
    let parked = ControlInput { left_brake: 1.0, right_brake: 1.0, levers: initial, ..ControlInput::default() };
    let s = run(&mut b, parked, 600);
    let weight_after = b.property("inertia/weight-lbs").unwrap();
    eprintln!("787-8 weight {weight:.0} -> {weight_after:.0} lb, parked agl {:.2} m", s.height_agl_m);
    assert!((weight_after - 283_642.0).abs() < 600.0, "BBJ weight after settings: {weight_after}");
    assert!(s.weight_on_wheels && s.calibrated_airspeed_mps < 0.5, "parked: {s:?}");
    assert!((s.flap_position - f64::from(initial.flaps)).abs() < 0.01, "flaps 5 set");
    // Take-off: full thrust, rotate at VR with a pitch-hold pull.
    let vr_mps = Aircraft::B7878.vr_kt() * 0.514_444;
    let mut lift_off = None;
    for i in 0..120 * 90 {
        let rotate = s_ias(&b) >= vr_mps;
        let theta = b.property("attitude/theta-deg").unwrap();
        let pitch = if rotate { ((10.0 - theta) * 0.15).clamp(-1.0, 1.0) as f32 } else { 0.0 };
        let gear_up = lift_off.is_some_and(|t: usize| i > t + 120 * 5);
        let c = ControlInput { throttle: 1.0, pitch, levers: Levers { gear_down: !gear_up, ..initial }, ..ControlInput::default() };
        let s = b.step(c.normalized(), STEP_SECONDS).unwrap();
        if lift_off.is_none() && !s.weight_on_wheels && s.height_agl_m > 5.0 {
            lift_off = Some(i);
            eprintln!("lift-off at {:.1} s, {:.0} KIAS", i as f64 / 120.0, s.calibrated_airspeed_mps / 0.514_444);
        }
    }
    let s = b.state().unwrap();
    assert!(lift_off.is_some(), "the 787 lifts off within 90 s");
    assert!(s.height_agl_m > 300.0, "climbing: {} m", s.height_agl_m);
    assert!(s.gear_position < 0.01, "gear up after 15 s travel: {}", s.gear_position);
    // Speedbrake out in the air.
    for _ in 0..120 * 3 {
        let c = ControlInput { throttle: 0.6, levers: Levers { gear_down: false, flaps: 0.0, speedbrake: 1.0 }, ..ControlInput::default() };
        b.step(c.normalized(), STEP_SECONDS).unwrap();
    }
    let s = b.state().unwrap();
    assert!(s.speedbrake_position > 0.99, "speedbrake out: {}", s.speedbrake_position);
}

fn s_ias(b: &JsbsimBackend) -> f64 {
    b.state().unwrap().calibrated_airspeed_mps
}


// ---------------------------------------------------------------------------
// AP-001 W1: the autopilot on the stock JSBSim 787-8 (BBJ preset), driven
// through SimCore exactly as the sim thread drives it (panel clicks in
// `ControlInput.ap`, valid input every tick). Every acceptance number of the
// W1 row is an assertion; each case prints one `AP-W1 {...}` line with the
// measured values. `GX_AP_TRACE=DIR` writes a 10 Hz CSV per case;
// `GX_AP_GAINS=name=value,...` overrides gains (tuning only);
// `GX_WORLD_PACK` selects the pack for the ILS case (the SFO pack; default:
// the synthetic pack's 28R, same flat-field geometry).

mod ap {
    use super::root;
    use gx_app::s1::synthetic_world_frame;
    use gx_app::s2::setup::{approaches, enu_to_latlon, jsbsim_setup, threshold, Aircraft};
    use gx_app::WorldFrame;
    use gx_sim::autopilot::{Gains, Vertical};
    use gx_sim::{
        AircraftState, ApAction, ApInput, Approach, Command, ControlInput, InputStatus, JsbsimBackend, Levers, SimCore,
    };
    use std::io::Write as _;

    const FT: f64 = 0.3048;
    const KT: f64 = 0.514_444;
    const TICK_NS: u64 = 1_000_000_000 / 120;
    const DT: f64 = 1.0 / 120.0;

    pub fn gains() -> Gains {
        let mut g = Gains::DEFAULT;
        if let Ok(spec) = std::env::var("GX_AP_GAINS") {
            for kv in spec.split(',').filter(|s| !s.is_empty()) {
                let (k, v) = kv.split_once('=').expect("GX_AP_GAINS: name=value");
                let v: f64 = v.parse().expect("GX_AP_GAINS: number");
                *match k {
                    "roll_phi" => &mut g.roll_phi,
                    "roll_p" => &mut g.roll_p,
                    "heading" => &mut g.heading,
                    "loc" => &mut g.loc,
                    "vs_p" => &mut g.vs_p,
                    "vs_i" => &mut g.vs_i,
                    "pitch_theta" => &mut g.pitch_theta,
                    "pitch_q" => &mut g.pitch_q,
                    "alt" => &mut g.alt,
                    "gs" => &mut g.gs,
                    "at_p" => &mut g.at_p,
                    "at_i" => &mut g.at_i,
                    "bank_rate" => &mut g.bank_rate,
                    "turn" => &mut g.turn,
                    _ => panic!("GX_AP_GAINS: unknown gain {k}"),
                } = v;
            }
        }
        g
    }

    pub fn agl_ft(s: &AircraftState) -> f64 {
        s.height_agl_m / FT
    }
    pub fn vs_fpm(s: &AircraftState) -> f64 {
        s.vertical_speed_mps / FT * 60.0
    }
    pub fn ias_kt(s: &AircraftState) -> f64 {
        s.calibrated_airspeed_mps / KT
    }
    pub fn deg(r: f64) -> f64 {
        r.to_degrees()
    }
    pub fn wrap(d: f64) -> f64 {
        gx_sim::autopilot::wrap_deg(d)
    }

    pub struct Flight {
        pub core: SimCore<JsbsimBackend>,
        pub panel: ApInput,
        pub pilot: ControlInput,
        pub s: AircraftState,
        /// The state before `s`.
        pub prev: AircraftState,
        tick: u64,
        trace: Option<std::io::BufWriter<std::fs::File>>,
        pub approach: Option<Approach>,
    }

    /// Clean: gear up, flaps up.
    pub const CLEAN: Levers = Levers { gear_down: false, flaps: 0.0, speedbrake: 0.0 };

    impl Flight {
        /// The 787-8 BBJ airborne at `enu` (pack metres), `agl_ft`, `kt` and
        /// true `heading_deg` with `levers` (also its initial gear and flaps),
        /// running, autopilot off, throttle 0.6.
        pub fn new(name: &str, frame: &WorldFrame, enu: [f64; 2], heading_deg: f64, agl_ft: f64, kt: f64, levers: Levers) -> Self {
            let t = threshold(frame).unwrap();
            let mut setup = jsbsim_setup(Aircraft::B7878, &t);
            let (lat, lon) = enu_to_latlon(frame, enu, t.surface_msl_m + frame.msl_to_ellipsoid_m()).unwrap();
            setup.ic.latitude_deg = lat;
            setup.ic.longitude_deg = lon;
            setup.ic.height_agl_m = agl_ft * FT;
            setup.ic.airspeed_kts = kt;
            setup.ic.heading_deg = heading_deg;
            setup.settings.retain(|(k, _)| !k.starts_with("fcs/flap") && !k.starts_with("gear/"));
            setup.settings.push(("gear/gear-cmd-norm".into(), if levers.gear_down { 1.0 } else { 0.0 }));
            setup.settings.push(("fcs/flap-cmd-norm".into(), f64::from(levers.flaps)));
            let backend = JsbsimBackend::load(&root(), &setup, frame.sim_frame()).expect("load 787-8 airborne");
            let mut core = SimCore::with_levers(backend, levers).with_approaches(approaches(frame)).with_autopilot_gains(gains());
            core.update_input(InputStatus::Valid, 0);
            let s = core.command(Command::Reset, 0).unwrap().unwrap().state;
            core.command(Command::Start, 0).unwrap();
            let trace = std::env::var("GX_AP_TRACE").ok().map(|dir| {
                let mut w = std::io::BufWriter::new(std::fs::File::create(format!("{dir}/{name}.csv")).unwrap());
                writeln!(w, "t,phi,theta,q,hdg,agl_ft,vs_fpm,ias_kt,engaged,lat,vert,at,roll,pitch,thr,along_m,cross_m,gs_dev_ft,elev,n1,p,status_gs_dev_ft").unwrap();
                w
            });
            let mut f = Self {
                core,
                panel: ApInput::DEFAULT,
                pilot: ControlInput { throttle: 0.6, levers, ..ControlInput::default() },
                s,
                prev: s,
                tick: 0,
                trace,
                approach: None,
            };
            // The first panel read after a Reset only adopts its serial.
            f.step();
            f
        }

        pub fn t(&self) -> f64 {
            self.tick as f64 * DT
        }

        pub fn click(&mut self, action: ApAction) {
            self.panel.serial += 1;
            self.panel.action = action;
            self.step();
        }

        pub fn step(&mut self) -> AircraftState {
            self.tick += 1;
            let now = self.tick * TICK_NS;
            self.core.update_input(InputStatus::Valid, now);
            let c = ControlInput { ap: self.panel, ..self.pilot };
            self.prev = self.s;
            self.s = self.core.step(c, now).expect("step").expect("running").state;
            if self.tick % 12 == 0 {
                self.trace_line();
            }
            self.s
        }

        /// (along, cross, glideslope deviation ft) on `self.approach`.
        pub fn ils(&self) -> Option<(f64, f64, f64)> {
            let a = self.approach?;
            let p = self.s.local_enu?.position_m;
            let (along, cross) = a.track([p[0], p[1]]);
            Some((along, cross, (self.s.height_agl_m - a.path_height_m(along)) / FT))
        }

        fn trace_line(&mut self) {
            let s = self.s;
            let a = s.autopilot;
            let c = self.core.applied_controls();
            let (along, cross, dev) = self.ils().unwrap_or((f64::NAN, f64::NAN, f64::NAN));
            let t = self.t();
            if let Some(w) = self.trace.as_mut() {
                writeln!(
                    w,
                    "{t:.2},{:.3},{:.3},{:.3},{:.2},{:.1},{:.0},{:.2},{},{:?},{:?},{},{:.4},{:.4},{:.4},{along:.1},{cross:.2},{dev:.1},{:.4},{:.1},{:.3},{:.1}",
                    deg(s.roll_rad), deg(s.pitch_rad), deg(s.angular_rate_rps[1]), deg(s.heading_rad), agl_ft(&s), vs_fpm(&s), ias_kt(&s),
                    u8::from(a.engaged), a.lateral, a.vertical, u8::from(a.autothrottle), c.roll, c.pitch, c.throttle,
                    s.elevator_position, s.engine_n1, deg(s.angular_rate_rps[0]), a.gs_dev_ft.map_or(f64::NAN, f64::from)
                )
                .unwrap();
            }
        }

        /// Fly `seconds`, calling `each(state, seconds since the call)`.
        pub fn fly(&mut self, seconds: f64, mut each: impl FnMut(&AircraftState, f64)) {
            let start = self.tick;
            for _ in 0..(seconds * 120.0).round() as u64 {
                let s = self.step();
                each(&s, (self.tick - start) as f64 * DT);
            }
        }

        /// Engage ALT (panel altitude) and SPD and fly `seconds` to settle
        /// out of the (untrimmed) initial conditions.
        pub fn settle(&mut self, altitude_ft: f32, speed_kt: f32, seconds: f64) {
            self.panel.altitude_ft = altitude_ft;
            self.panel.speed_kt = speed_kt;
            self.click(ApAction::Altitude);
            self.click(ApAction::Speed);
            self.fly(seconds, |_, _| {});
            let a = self.s.autopilot;
            assert!(a.engaged && a.autothrottle && a.vertical == Vertical::AltHold, "settled: {a:?}");
        }

        /// Pitch-rate transient of the `seconds` after the autopilot handed
        /// the stick back on the last step (pilot stick neutral, trim as it
        /// is): max |q - q0| in deg/s and max |elevator - e0| (normalized),
        /// from the last state the autopilot flew.
        pub fn disconnect_bump(&mut self, seconds: f64) -> (f64, f64) {
            let q0 = deg(self.prev.angular_rate_rps[1]);
            let e0 = self.prev.elevator_position;
            let mut worst = (deg(self.s.angular_rate_rps[1]) - q0).abs();
            let mut elev = (self.s.elevator_position - e0).abs();
            self.fly(seconds, |s, _| {
                worst = worst.max((deg(s.angular_rate_rps[1]) - q0).abs());
                elev = elev.max((s.elevator_position - e0).abs());
            });
            (worst, elev)
        }
    }

    impl Drop for Flight {
        fn drop(&mut self) {
            if let Some(w) = self.trace.as_mut() {
                let _ = w.flush();
            }
        }
    }

    pub fn report(case: &str, fields: &[(&str, f64)], pass: bool) {
        let mut line = format!("AP-W1 {{\"case\":\"{case}\",\"pass\":{pass}");
        for (k, v) in fields {
            line.push_str(&format!(",\"{k}\":{v:.3}"));
        }
        line.push_str(&format!(",\"gains\":\"{:?}\"}}", gains()));
        println!("{line}");
    }

    pub fn cruise(name: &str, kt: f64) -> Flight {
        Flight::new(name, &synthetic_world_frame().unwrap(), [0.0, 0.0], 90.0, 3000.0, kt, CLEAN)
    }

    pub fn pack_frame() -> (WorldFrame, String) {
        match std::env::var("GX_WORLD_PACK") {
            Ok(path) => (gx_app::spike002::world_frame_from_pack(&path).unwrap(), path),
            Err(_) => (synthetic_world_frame().unwrap(), "synthetic".into()),
        }
    }

    pub use gx_sim::autopilot::{ApWarning as Warning, Lateral as Lat, Vertical as Vert};
}

use ap::{agl_ft, deg, ias_kt, report, vs_fpm, wrap, Flight};
use gx_sim::ApAction;

/// LVL from 30 deg bank and +2,000 fpm (3,000 ft, 250 kt): wings level
/// within +-2 deg and altitude within +-50 ft of the altitude held, from
/// 60 s on (checked to 120 s). Also the disconnect bump at cruise.
#[test]
fn b7878_ap_lvl_from_30_bank_and_2000_fpm() {
    let mut f = ap::cruise("lvl", 250.0);
    f.settle(3000.0, 250.0, 90.0);
    // AP off (the A/T stays on), stick neutral: the cruise disconnect bump.
    f.click(ApAction::Master);
    assert!(!f.s.autopilot.engaged && f.s.autopilot.autothrottle);
    let (bump_cruise, elev_cruise) = f.disconnect_bump(3.0);
    // The test pilot rolls to 30 deg and climbs at 2,000 fpm.
    let mut theta_cmd = deg(f.s.pitch_rad);
    let mut held = 0;
    for _ in 0..120 * 60 {
        let s = f.s;
        let (phi, p, q, theta) = (deg(s.roll_rad), deg(s.angular_rate_rps[0]), deg(s.angular_rate_rps[1]), deg(s.pitch_rad));
        let vs = vs_fpm(&s);
        theta_cmd = (theta_cmd + 0.02 * (2000.0 - vs) / 60.0 / 120.0).clamp(-5.0, 15.0);
        let th = theta_cmd + 0.03 * (2000.0 - vs) / 60.0;
        f.pilot.roll = (0.05 * (30.0 - phi) - 0.03 * p).clamp(-1.0, 1.0) as f32;
        f.pilot.pitch = (0.1 * (th - theta) - 0.05 * q).clamp(-1.0, 1.0) as f32;
        f.step();
        held = if (phi - 30.0).abs() < 0.5 && (vs - 2000.0).abs() < 30.0 { held + 1 } else { 0 };
        if held >= 120 {
            break;
        }
    }
    assert!(held >= 120, "test pilot reached 30 deg / 2000 fpm");
    // The click below is read on the next step, from this state.
    let (phi0, vs0) = (deg(f.s.roll_rad), vs_fpm(&f.s));
    f.pilot.roll = 0.0;
    f.pilot.pitch = 0.0;
    f.click(ApAction::Level);
    let mut worst_phi = 0.0f64;
    let mut alts = Vec::new();
    let mut level_at = None;
    f.fly(120.0, |s, t| {
        if t >= 60.0 {
            worst_phi = worst_phi.max(deg(s.roll_rad).abs());
            alts.push(agl_ft(s));
        }
        if level_at.is_none() && s.autopilot.vertical == ap::Vert::AltHold {
            level_at = Some(t);
        }
    });
    let st = f.s.autopilot;
    let hold = f64::from(st.hold_ft);
    let worst_alt = alts.iter().map(|a| (a - hold).abs()).fold(0.0, f64::max);
    let pass = st.engaged && st.vertical == ap::Vert::AltHold && worst_phi <= 2.0 && worst_alt <= 50.0;
    report(
        "lvl",
        &[
            ("start_bank_deg", phi0), ("start_vs_fpm", vs0), ("level_off_s", level_at.unwrap_or(f64::NAN)),
            ("hold_ft", hold), ("max_bank_60_120s_deg", worst_phi), ("max_alt_err_60_120s_ft", worst_alt),
            ("cruise_disconnect_bump_dps", bump_cruise), ("cruise_disconnect_elev_step", elev_cruise),
        ],
        pass,
    );
    assert!(pass, "LVL: bank {worst_phi} deg, altitude {worst_alt} ft, {st:?}");
}

/// HDG 90 deg turn (090 -> 180, 3,000 ft) at `kt`. W1: bank <= 27 deg,
/// settles within +-2 deg (the last 30 s of 150 s), altitude within +-100
/// ft. AP-002 W5: rolls in to the bank limit (within 0.5 deg of it) in
/// <= 6 s, bank never more than 2 deg past the limit, heading overshoot
/// <= 1 deg, altitude within +-30 ft; reports the peak roll rate (R1).
fn hdg_case(name: &str, kt: f64) {
    use gx_sim::autopilot::MAX_BANK_DEG;
    let mut f = ap::cruise(name, kt);
    f.settle(3000.0, kt as f32, 90.0);
    f.panel.heading_deg = 180.0;
    f.click(ApAction::Heading);
    let (mut bank, mut alt_err, mut settle_err, mut overshoot, mut p_max) = (0.0f64, 0.0f64, 0.0f64, f64::NEG_INFINITY, 0.0f64);
    let (mut within_at, mut roll_in_at) = (None, None);
    f.fly(150.0, |s, t| {
        let phi = deg(s.roll_rad).abs();
        bank = bank.max(phi);
        p_max = p_max.max(deg(s.angular_rate_rps[0]).abs());
        alt_err = alt_err.max((agl_ft(s) - 3000.0).abs());
        let e = wrap(deg(s.heading_rad) - 180.0);
        overshoot = overshoot.max(e);
        if t >= 120.0 {
            settle_err = settle_err.max(e.abs());
        }
        if within_at.is_none() && e.abs() <= 2.0 {
            within_at = Some(t);
        }
        if roll_in_at.is_none() && phi >= MAX_BANK_DEG - 0.5 {
            roll_in_at = Some(t);
        }
    });
    let roll_in = roll_in_at.unwrap_or(f64::INFINITY);
    let w1 = bank <= 27.0 && settle_err <= 2.0 && alt_err <= 100.0 && f.s.autopilot.lateral == ap::Lat::Heading;
    let w5 = roll_in <= 6.0 && bank <= MAX_BANK_DEG + 2.0 && overshoot <= 1.0 && alt_err <= 30.0;
    report(
        name,
        &[("kt", kt), ("max_bank_deg", bank), ("roll_in_s", roll_in), ("peak_roll_rate_dps", p_max), ("max_alt_err_ft", alt_err),
          ("heading_err_120_150s_deg", settle_err), ("overshoot_deg", overshoot), ("within_2deg_s", within_at.unwrap_or(f64::NAN))],
        w1 && w5,
    );
    assert!(w1, "{name} W1: bank {bank}, settle {settle_err}, altitude {alt_err}");
    assert!(w5, "{name} W5: roll-in {roll_in} s, bank {bank}, overshoot {overshoot}, altitude {alt_err}");
}

#[test]
fn b7878_ap_hdg_90_deg_turn() {
    hdg_case("hdg", 250.0);
}

#[test]
fn b7878_ap_hdg_90_deg_turn_at_290_kt() {
    hdg_case("hdg_290", 290.0);
}

/// ALT 3,000 -> 8,000 ft with SPD 250: VS <= 1,700 fpm, overshoot <= 150 ft,
/// then within +-50 ft (the last 60 s of 480 s).
#[test]
fn b7878_ap_alt_3000_to_8000_with_spd() {
    let mut f = ap::cruise("alt", 250.0);
    f.settle(3000.0, 250.0, 90.0);
    f.panel.altitude_ft = 8000.0;
    f.click(ApAction::Altitude);
    let (mut vs, mut top, mut hold_err, mut spd_err) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let mut capture_at = None;
    f.fly(480.0, |s, t| {
        vs = vs.max(vs_fpm(s).abs());
        top = top.max(agl_ft(s));
        spd_err = spd_err.max((ias_kt(s) - 250.0).abs());
        if t >= 420.0 {
            hold_err = hold_err.max((agl_ft(s) - 8000.0).abs());
        }
        if capture_at.is_none() && s.autopilot.vertical == ap::Vert::AltHold {
            capture_at = Some(t);
        }
    });
    let overshoot = top - 8000.0;
    let pass = vs <= 1700.0 && overshoot <= 150.0 && hold_err <= 50.0 && f.s.autopilot.vertical == ap::Vert::AltHold;
    report(
        "alt",
        &[("max_vs_fpm", vs), ("overshoot_ft", overshoot), ("alt_err_420_480s_ft", hold_err),
          ("capture_s", capture_at.unwrap_or(f64::NAN)), ("max_speed_err_kt", spd_err)],
        pass,
    );
    assert!(pass, "ALT: vs {vs}, overshoot {overshoot}, hold {hold_err}");
}

/// SPD `from` -> `to` kt (altitude hold at 3,000 ft), W1 and W1b: settled
/// within +-3 kt from 120 s on (checked to 180 s); W1b: overshoot past `to`
/// <= 3 kt and altitude within +-50 ft of 3,000 ft for the whole 180 s.
fn spd_case(name: &str, from: f64, to: f64) {
    let mut f = ap::cruise(name, from);
    f.settle(3000.0, from as f32, 120.0);
    let start = ias_kt(&f.s);
    f.panel.speed_kt = to as f32;
    let sign = (to - from).signum();
    let (mut err, mut over, mut alt_err) = (0.0f64, 0.0f64, 0.0f64);
    let mut within_at = None;
    f.fly(180.0, |s, t| {
        let e = ias_kt(s) - to;
        over = over.max(sign * e);
        alt_err = alt_err.max((agl_ft(s) - 3000.0).abs());
        if t >= 120.0 {
            err = err.max(e.abs());
        }
        if within_at.is_none() && e.abs() <= 3.0 {
            within_at = Some(t);
        }
    });
    let pass = err <= 3.0 && over <= 3.0 && alt_err <= 50.0 && f.s.autopilot.autothrottle;
    report(
        name,
        &[("start_kt", start), ("speed_err_120_180s_kt", err), ("overshoot_kt", over),
          ("within_3kt_s", within_at.unwrap_or(f64::NAN)), ("max_alt_err_ft", alt_err)],
        pass,
    );
    assert!(pass, "{name}: settled {err} kt, overshoot {over} kt, altitude {alt_err} ft");
}

#[test]
fn b7878_ap_spd_220_to_260() {
    spd_case("spd", 220.0, 260.0);
}

#[test]
fn b7878_ap_spd_260_to_220() {
    spd_case("spd_down", 260.0, 220.0);
}

/// ILS 28R from 12 nm on a 30 deg intercept at 3,000 ft, flaps 20, gear
/// down, 160 kt: below 1,500 ft the localizer within +-30 m and the
/// glideslope within +-50 ft; hands over at 100 ft. Then the disconnect
/// bump (stick neutral, no trim) over 3 s.
#[test]
fn b7878_ap_ils_28r_and_the_disconnect_bump() {
    let (frame, pack) = ap::pack_frame();
    let all = gx_app::s2::setup::approaches(&frame);
    let idx = all.iter().position(|a| a.name() == "28R").expect("runway end 28R in the pack");
    let a = all[idx];
    let c = a.course_rad;
    let (u, r) = ([c.sin(), c.cos()], [c.cos(), -c.sin()]);
    // The 12 nm point, 2,000 m right of the centreline, and 135 s at 160 kt
    // before it on the 30 deg intercept heading: 120 s settle out of the
    // initial conditions on altitude, heading and speed holds, then fly on to
    // 12 nm and press ILS there.
    let (along12, cross12) = (-12.0 * 1852.0, 2000.0);
    let p12 = [a.threshold_m[0] + along12 * u[0] + cross12 * r[0], a.threshold_m[1] + along12 * u[1] + cross12 * r[1]];
    let h = c.to_degrees() - 30.0;
    let back = 135.0 * 160.0 * 0.514_444;
    let p0 = [p12[0] - back * h.to_radians().sin(), p12[1] - back * h.to_radians().cos()];
    let levers = gx_sim::Levers { gear_down: true, flaps: 20.0 / 35.0, speedbrake: 0.0 };
    let mut f = Flight::new("ils", &frame, p0, h.rem_euclid(360.0), 3000.0, 160.0, levers);
    f.approach = Some(a);
    f.panel.heading_deg = h.rem_euclid(360.0) as f32;
    f.click(ApAction::Heading);
    f.settle(3000.0, 160.0, 120.0);
    while f.ils().unwrap().0 < along12 {
        f.step();
    }
    let (al, cr, _) = f.ils().unwrap();
    let start = (al / 1852.0, cr, agl_ft(&f.s), ias_kt(&f.s));
    f.click(ApAction::Approach);
    let st = f.s.autopilot;
    assert_eq!(st.approach, Some(idx as u8), "ILS armed on 28R: {st:?}");
    let (mut loc, mut gs, mut below) = (0.0f64, 0.0f64, 0usize);
    let (mut loc_at, mut gs_at) = (None, None);
    let mut handover = None;
    for _ in 0..120 * 900 {
        f.step();
        let st = f.s.autopilot;
        let (along, cross, dev) = f.ils().unwrap();
        if loc_at.is_none() && st.lateral == ap::Lat::Localizer {
            loc_at = Some(along / 1852.0);
        }
        if gs_at.is_none() && st.vertical == ap::Vert::Glideslope {
            gs_at = Some(along / 1852.0);
        }
        if st.engaged && agl_ft(&f.s) < 1500.0 {
            below += 1;
            loc = loc.max(cross.abs());
            gs = gs.max(dev.abs());
        }
        if !st.engaged {
            handover = Some((agl_ft(&f.s), st.warning, along, cross));
            break;
        }
    }
    let (ho_ft, ho_warning, ho_along, ho_cross) = handover.expect("the autopilot hands over");
    let (bump, elev) = f.disconnect_bump(3.0);
    let pass = below > 120 * 20
        && loc <= 30.0
        && gs <= 50.0
        && ho_warning == Some(ap::Warning::ApDisconnect)
        && ho_ft < 100.0
        && ho_ft > 90.0;
    report(
        "ils_28r",
        &[
            ("start_nm", start.0), ("start_cross_m", start.1), ("start_agl_ft", start.2), ("start_kt", start.3),
            ("loc_capture_nm", loc_at.unwrap_or(f64::NAN)), ("gs_capture_nm", gs_at.unwrap_or(f64::NAN)),
            ("max_loc_err_below_1500_m", loc), ("max_gs_err_below_1500_ft", gs), ("seconds_below_1500", below as f64 / 120.0),
            ("handover_agl_ft", ho_ft), ("handover_along_m", ho_along), ("handover_cross_m", ho_cross),
            ("ils_disconnect_bump_dps", bump), ("ils_disconnect_elev_step", elev),
        ],
        pass,
    );
    println!("AP-W1 ils pack: {pack}");
    assert!(pass, "ILS: loc {loc} m, gs {gs} ft, handover {ho_ft} ft {ho_warning:?}");
}

/// AP-002 W5b: ILS 28R entered on the localizer at 10 nm about 1,000 ft
/// above the path (flaps 20, gear down, 160 kt): G/S descent, G/S capture
/// at least 4 nm out, then within +-50 ft below 1,500 ft and hand-over at
/// 100 ft. `gs_dev_ft` is filled while the approach is armed.
#[test]
fn b7878_ap_ils_28r_from_above() {
    let (frame, pack) = ap::pack_frame();
    let all = gx_app::s2::setup::approaches(&frame);
    let idx = all.iter().position(|a| a.name() == "28R").expect("runway end 28R in the pack");
    let a = all[idx];
    let c = a.course_rad;
    let u = [c.sin(), c.cos()];
    let along10 = -10.0 * 1852.0;
    let alt_ft = a.path_height_m(along10) / 0.3048 + 1000.0;
    // 120 s settle at 160 kt on the extended centreline before 10 nm.
    let back = along10 - 125.0 * 160.0 * 0.514_444;
    let p0 = [a.threshold_m[0] + back * u[0], a.threshold_m[1] + back * u[1]];
    let levers = gx_sim::Levers { gear_down: true, flaps: 20.0 / 35.0, speedbrake: 0.0 };
    let mut f = Flight::new("ils_above", &frame, p0, c.to_degrees(), alt_ft, 160.0, levers);
    f.approach = Some(a);
    f.panel.heading_deg = c.to_degrees() as f32;
    f.click(ApAction::Heading);
    f.settle(alt_ft as f32, 160.0, 120.0);
    while f.ils().unwrap().0 < along10 {
        f.step();
    }
    let (_, cr0, dev0) = f.ils().unwrap();
    f.click(ApAction::Approach);
    let (mut loc, mut gs, mut dev_err, mut sink, mut ias_max) = (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let (mut descent_at, mut gs_at, mut handover) = (None, None, None);
    for _ in 0..120 * 900 {
        f.step();
        let st = f.s.autopilot;
        let (along, cross, dev) = f.ils().unwrap();
        if st.engaged {
            let shown = f64::from(st.gs_dev_ft.expect("gs_dev_ft while the approach is armed or captured"));
            dev_err = dev_err.max((shown - dev).abs());
        }
        if descent_at.is_none() && st.vertical == ap::Vert::GsDescent {
            descent_at = Some((along / 1852.0, dev));
        }
        if st.vertical == ap::Vert::GsDescent {
            sink = sink.max(-vs_fpm(&f.s));
            ias_max = ias_max.max(ias_kt(&f.s));
        }
        if gs_at.is_none() && st.vertical == ap::Vert::Glideslope {
            gs_at = Some(along / 1852.0);
        }
        if st.engaged && agl_ft(&f.s) < 1500.0 {
            loc = loc.max(cross.abs());
            gs = gs.max(dev.abs());
        }
        if !st.engaged {
            handover = Some((agl_ft(&f.s), st.warning));
            break;
        }
    }
    let (ho_ft, ho_warning) = handover.expect("hands over");
    let gs_nm = gs_at.unwrap_or(0.0);
    let (descent_nm, descent_dev) = descent_at.unwrap_or((f64::NAN, f64::NAN));
    let pass = descent_at.is_some()
        && gs_nm <= -4.0
        && gs <= 50.0
        && loc <= 30.0
        && dev_err < 1.0
        && ho_warning == Some(ap::Warning::ApDisconnect)
        && ho_ft < 100.0
        && ho_ft > 90.0;
    report(
        "ils_above",
        &[
            ("start_dev_ft", dev0), ("start_cross_m", cr0), ("descent_nm", descent_nm), ("descent_dev_ft", descent_dev),
            ("gs_capture_nm", gs_nm), ("max_sink_fpm", sink), ("max_ias_in_descent_kt", ias_max),
            ("max_loc_err_below_1500_m", loc), ("max_gs_err_below_1500_ft", gs), ("gs_dev_ft_err_ft", dev_err),
            ("handover_agl_ft", ho_ft),
        ],
        pass,
    );
    println!("AP-W1 ils_above pack: {pack}");
    assert!(pass, "ILS from above: G/S at {gs_nm} nm, gs {gs} ft, loc {loc} m, handover {ho_ft} ft");
}

/// An autopilot run on JSBSim (ALT, SPD, HDG, an input loss with the AP on,
/// a stick override, then a loss with it off: R4 LVL), logged as JSONL tick
/// records, parsed back and replayed on a fresh backend: bit-identical.
#[test]
fn b7878_ap_run_replays_bit_identically() {
    use gx_app::s1::driver::{SimDriver, TICK_NS};
    use gx_app::s1::rec::{to_json, Rec};
    use gx_app::s2::replay::{parse_tick, replay_with_approaches};
    use gx_sim::{control_mailbox, ApInput, CommandEdges, SnapshotExchange, UnavailableReason};
    let frame = synthetic_world_frame().unwrap();
    let t = threshold(&frame).unwrap();
    let backend = || {
        let mut setup = jsbsim_setup(Aircraft::B7878, &t);
        setup.ic.height_agl_m = 3000.0 * 0.3048;
        setup.ic.airspeed_kts = 250.0;
        setup.settings.retain(|(k, _)| !k.starts_with("fcs/flap") && !k.starts_with("gear/"));
        setup.settings.push(("gear/gear-cmd-norm".into(), 0.0));
        setup.settings.push(("fcs/flap-cmd-norm".into(), 0.0));
        JsbsimBackend::load(&root(), &setup, frame.sim_frame()).expect("load")
    };
    let levers = ap::CLEAN;
    let (mut writer, reader) = control_mailbox();
    let t0 = 1_000_000_000;
    let exchange = std::sync::Arc::new(SnapshotExchange::new());
    let mut driver = SimDriver::try_new_with_levers(backend(), exchange, reader, t0, levers, &mut |_| {})
        .unwrap()
        .with_approaches(gx_app::s2::setup::approaches(&frame))
        .with_trace();
    let mut lines = Vec::new();
    let mut panel = ApInput::DEFAULT;
    let (mut engaged, mut lvl) = (0u64, false);
    for k in 1..=120 * 60u64 {
        let now = t0 + k * TICK_NS;
        let mut click = |a: ApAction| {
            panel.serial += 1;
            panel.action = a;
        };
        match k {
            10 => click(ApAction::Altitude),
            20 => click(ApAction::Speed),
            600 => {
                panel.heading_deg = 180.0;
                click(ApAction::Heading)
            }
            _ => {}
        }
        if (1800..2400).contains(&k) || (4800..5400).contains(&k) {
            writer.publish_unavailable(UnavailableReason::Disconnected, k, now, now);
        } else {
            let pitch = if (3600..3612).contains(&k) { 0.5 } else { 0.0 };
            let c = ControlInput { pitch, throttle: 0.6, levers, ap: panel, ..ControlInput::default() };
            writer.publish_valid(c, k, now, CommandEdges { start: k == 2, ..CommandEdges::default() }, now);
        }
        driver.tick(now, 0, &mut || now, &mut |r| {
            if matches!(r, Rec::Tick(_)) {
                lines.push(to_json(&r, 0));
            }
        });
        if let Some(s) = driver.last_snapshot() {
            engaged += u64::from(s.state.autopilot.engaged);
            lvl |= s.state.autopilot.warning == Some(ap::Warning::InputLossLevel);
        }
    }
    assert!(engaged > 120 * 40 && lvl, "the autopilot flew ({engaged} ticks) and R4 engaged LVL ({lvl})");
    let ticks: Vec<_> = lines.iter().map(|l| parse_tick(l).unwrap()).collect();
    let result = replay_with_approaches(backend(), levers, gx_app::s2::setup::approaches(&frame), &ticks).unwrap();
    println!("AP-W1 {{\"case\":\"replay\",\"ticks\":{},\"hashed\":{},\"mismatches\":{},\"gaps\":{},\"engaged_ticks\":{engaged}}}", result.ticks, result.hashed, result.mismatches, result.gaps);
    assert_eq!((result.mismatches, result.gaps), (0, 0), "{result:?}");
    assert!(result.hashed > 120 * 59, "{result:?}");
}

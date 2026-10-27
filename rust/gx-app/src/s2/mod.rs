//! M1 step S2: the S1 thread layout with JSBSim (`SimCore<JsbsimBackend>`),
//! still headless, plus record/replay. See
//! `docs/architecture/m1-vertical-skeleton.md` (section *S2*) and
//! `tools/s2_check.py`.

pub mod phases;
pub mod replay;
pub mod runtime;
pub mod setup;

use crate::s1::synthetic_world_frame;
use crate::{FakeBackend, WorldFrame};
use setup::{Aircraft, Threshold};
use std::io::Write;

pub use crate::s1::DEFAULT_DEVICE;
pub const DEFAULT_SECONDS: u64 = 300;
pub const DEFAULT_JSBSIM_ROOT: &str = "./jsbsim";

pub const HELP: &str = "\
gx-app s2 - M1 step S2: headless ARCH-004 threads, JSBSim, real Logitech stick

Runs S1's threads (I/O SCHED_FIFO 40 -> control mailbox -> sim SCHED_FIFO 50
at 120 Hz absolute deadlines -> SnapshotExchange -> 30 Hz reader) with
SimCore<JsbsimBackend>: the aircraft starts on
the threshold of the GXWORLD 2.0 synthetic SFO pack's runway, in the pack's
frame and geoid separation. Every mailbox record the sim consumes is logged
with its tick (replayable), plus a 10 Hz state record. Prompts the Chief
through: connect, start, sign check (8 holds, ~60 s), line-up, take-off
(<= 2 min), end. No display, no sudo; the stick is not grabbed.

USAGE:
    gx-app s2 [OPTIONS]
    gx-app s2 replay --log RUN.jsonl --out REPLAY.json [--jsbsim-root DIR]

OPTIONS:
    --log PATH          JSONL log, created new (default: s2-run.jsonl)
    --seconds N         upper bound on the run in seconds (default: 300)
    --device PATH       stable stick path (default: the Extreme 3D Pro by-id path)
    --aircraft NAME     global5000 (default), 787-8 (BBJ 787) or c172p (fallback)
    --jsbsim-root DIR   JSBSim data root with aircraft/ engine/ systems/
                        (default: ./jsbsim)
    --autostart         one console Start as soon as input is valid (smoke)
    --stick neutral     smoke/timing only: the I/O thread publishes a valid,
                        centred, idle, fully-braked test input every 5 ms instead of reading the
                        device (logged as device synthetic:neutral; the
                        checker refuses it for a Chief session)
    --quiet             no prompts on stdout
    --fake              FakeBackend instead of JSBSim (host development only)
    --help              print this help and exit

Bindings: START trigger (button 1), PAUSE button 2, RESET button 3, PARKING
BRAKE toggle button 4 (Reset sets it again). Hat = wheel brakes: back both,
left/right that side. Buttons 5-12 are unmapped. ENTER on stdin is
a console Start (INPUT-002 gate applies).

Check on the laptop:  python3 tools/s2_check.py s2-run.jsonl --replay replay.json
";

struct Args {
    log: String,
    out: Option<String>,
    seconds: u64,
    device: String,
    aircraft: Aircraft,
    jsbsim_root: String,
    autostart: bool,
    prompts: bool,
    fake: bool,
    neutral_stick: bool,
    replay: bool,
}

fn parse(args: &[String]) -> Result<Option<Args>, String> {
    let mut a = Args {
        log: "s2-run.jsonl".into(),
        out: None,
        seconds: DEFAULT_SECONDS,
        device: DEFAULT_DEVICE.into(),
        aircraft: Aircraft::Global5000,
        jsbsim_root: DEFAULT_JSBSIM_ROOT.into(),
        autostart: false,
        prompts: true,
        fake: false,
        neutral_stick: false,
        replay: false,
    };
    let mut it = args.iter().peekable();
    if it.peek().map(|s| s.as_str()) == Some("replay") {
        it.next();
        a.replay = true;
    }
    while let Some(flag) = it.next() {
        let mut value = |name: &str| it.next().cloned().ok_or_else(|| format!("{name} needs a value"));
        match flag.as_str() {
            "--help" | "-h" => return Ok(None),
            "--log" => a.log = value("--log")?,
            "--out" => a.out = Some(value("--out")?),
            "--seconds" => {
                let v = value("--seconds")?;
                a.seconds = v.parse().map_err(|_| format!("--seconds: not a positive integer: {v}"))?;
                if a.seconds == 0 {
                    return Err("--seconds must be > 0".into());
                }
            }
            "--device" => a.device = value("--device")?,
            "--aircraft" => {
                let v = value("--aircraft")?;
                a.aircraft = Aircraft::parse(&v).ok_or_else(|| format!("--aircraft: global5000, 787-8 or c172p, not {v}"))?;
            }
            "--jsbsim-root" => a.jsbsim_root = value("--jsbsim-root")?,
            "--autostart" => a.autostart = true,
            "--quiet" => a.prompts = false,
            "--fake" => a.fake = true,
            "--stick" => match value("--stick")?.as_str() {
                "neutral" => a.neutral_stick = true,
                "device" => a.neutral_stick = false,
                other => return Err(format!("--stick: device or neutral, not {other}")),
            },
            other => return Err(format!("unknown argument: {other} (see --help)")),
        }
    }
    if a.replay && a.out.is_none() {
        return Err("replay needs --out".into());
    }
    Ok(Some(a))
}

pub(crate) fn create_new(path: &str) -> Result<std::fs::File, String> {
    std::fs::OpenOptions::new().write(true).create_new(true).open(path).map_err(|e| format!("create {path}: {e}"))
}

#[cfg(feature = "jsbsim")]
pub fn jsbsim_backend(
    root: &str,
    aircraft: Aircraft,
    threshold: &Threshold,
    frame: &WorldFrame,
) -> Result<gx_sim::JsbsimBackend, String> {
    let setup = setup::jsbsim_setup(aircraft, threshold);
    gx_sim::JsbsimBackend::load(std::path::Path::new(root), &setup, frame.sim_frame())
        .map_err(|e| format!("JSBSim {} from {root}: {e}", aircraft.name()))
}

/// Entry point for `gx-app s2 ...`; returns the exit code.
pub fn main(args: &[String]) -> i32 {
    let a = match parse(args) {
        Ok(Some(a)) => a,
        Ok(None) => {
            print!("{HELP}");
            return 0;
        }
        Err(e) => {
            eprintln!("gx-app s2: {e}");
            return 2;
        }
    };
    // JSBSim prints its model report and gear contacts at debug level 1;
    // keep the Chief's console to the prompts unless asked otherwise. Set
    // before any FDM exists (FGFDMExec reads it at construction), while
    // this process is still single-threaded.
    if std::env::var_os("JSBSIM_DEBUG").is_none() {
        std::env::set_var("JSBSIM_DEBUG", "0");
    }
    match if a.replay { replay_main(&a) } else { run_main(a) } {
        Ok(code) => code,
        Err(e) => {
            eprintln!("gx-app s2: {e}");
            1
        }
    }
}

pub(crate) fn world() -> Result<(WorldFrame, Threshold), String> {
    let frame = synthetic_world_frame().map_err(|e| format!("world pack: {e}"))?;
    let threshold = setup::threshold(&frame)?;
    Ok((frame, threshold))
}

pub(crate) fn commit() -> &'static str {
    option_env!("GX_COMMIT").unwrap_or("unknown")
}

fn replay_main(a: &Args) -> Result<i32, String> {
    let bytes = std::fs::read(&a.log).map_err(|e| format!("read {}: {e}", a.log))?;
    let (start, ticks) = replay::read_log(std::io::Cursor::new(&bytes))?;
    if start.schema != runtime::SCHEMA {
        return Err(format!("{} is not an S2 log (schema {})", a.log, start.schema));
    }
    if start.commit != commit() {
        return Err(format!("log was written by commit {}, this binary is {}: bit-exact replay needs the same binary", start.commit, commit()));
    }
    let (frame, threshold) = world()?;
    // The autopilot's ILS needs the run's approaches to replay bit-exactly.
    let approaches = setup::approaches(&frame);
    let result = match start.backend.as_str() {
        "fake" => replay::replay_with_approaches(FakeBackend::new(frame), replay::initial_levers(&start), approaches, &ticks)?,
        "jsbsim" => {
            #[cfg(feature = "jsbsim")]
            {
                let aircraft = Aircraft::parse(&start.aircraft).ok_or("unknown aircraft in log")?;
                replay::replay_with_approaches(jsbsim_backend(&a.jsbsim_root, aircraft, &threshold, &frame)?, aircraft.initial_levers(), approaches, &ticks)?
            }
            #[cfg(not(feature = "jsbsim"))]
            {
                let _ = threshold;
                return Err("this gx-app was built without the jsbsim feature".into());
            }
        }
        other => return Err(format!("unknown backend {other}")),
    };
    let json = result.to_json(&replay::sha256_hex(&bytes), &start, commit());
    let out = a.out.as_deref().expect("checked in parse");
    let mut f = create_new(out)?;
    writeln!(f, "{json}").map_err(|e| e.to_string())?;
    println!("{json}");
    Ok(if result.mismatches == 0 && result.gaps == 0 && result.ticks > 0 { 0 } else { 3 })
}

fn run_main(a: Args) -> Result<i32, String> {
    let (frame, threshold) = world()?;
    let file = create_new(&a.log)?;
    let cfg = runtime::S2Config {
        seconds: a.seconds,
        prompts: a.prompts,
        device: if a.neutral_stick { "synthetic:neutral".into() } else { a.device.clone() },
        commit: commit(),
        aircraft: a.aircraft,
        backend: if a.fake { "fake" } else { "jsbsim" },
        threshold,
        autostart: a.autostart,
    };
    if a.fake {
        return run_board(a, cfg, FakeBackend::new(frame), file);
    }
    #[cfg(feature = "jsbsim")]
    {
        let backend = jsbsim_backend(&a.jsbsim_root, a.aircraft, &threshold, &frame)?;
        run_board(a, cfg, backend, file)
    }
    #[cfg(not(feature = "jsbsim"))]
    {
        let _ = (cfg, file);
        Err("this gx-app was built without the jsbsim feature (use --fake on a host)".into())
    }
}

#[cfg(target_os = "linux")]
fn run_board<B>(a: Args, cfg: runtime::S2Config, backend: B, file: std::fs::File) -> Result<i32, String>
where
    B: gx_sim::SimulationBackend + Send + 'static,
    B::Error: std::fmt::Debug,
{
    let mut stdout = std::io::stdout();
    let code = if a.neutral_stick {
        runtime::run(cfg, crate::s1::rt::MonotonicClock, NeutralStick::default(), backend, file, &mut stdout)
    } else {
        let source =
            crate::s1::io::EvdevSource::new(a.device.into()).map_err(|e| format!("input connector thread: {e}"))?;
        runtime::run(cfg, crate::s1::rt::MonotonicClock, source, backend, file, &mut stdout)
    }
    .map_err(|e| e.to_string())?;
    if a.prompts {
        println!("log: {}", a.log);
    }
    Ok(code)
}

#[cfg(not(target_os = "linux"))]
fn run_board<B>(_a: Args, _cfg: runtime::S2Config, _backend: B, _file: std::fs::File) -> Result<i32, String> {
    Err("the board run needs Linux evdev".into())
}

/// Smoke/timing source (`--stick neutral`): a connected stick held centred
/// with the throttle at idle and no buttons, one report every 5 ms. An
/// untouched real Extreme 3D Pro may send no event at all (gx-input reads
/// no device state at connect), so it never becomes valid and an
/// unattended run cannot start the sim; this source lets the smoke measure
/// the JSBSim step under the real threads and RT policies.
#[derive(Default)]
pub struct NeutralStick {
    connected: bool,
    next_ns: u64,
    seq: u64,
}

impl crate::s1::io::InputSource for NeutralStick {
    fn poll(
        &mut self,
        now_ns: u64,
        fresh_now: &mut dyn FnMut() -> u64,
        bridge: &mut crate::s1::io::IoBridge,
        log: &mut crate::s1::io::IoLog,
    ) {
        if !self.connected {
            self.connected = true;
            bridge.connected();
            log.io(now_ns, "\"ev\":\"connected\",\"path\":\"synthetic:neutral\",\"by_id_match\":true");
        }
        if now_ns < self.next_ns {
            return;
        }
        self.next_ns = now_ns + 5_000_000;
        let frame = gx_input::ControlFrame {
            timestamp_ns: now_ns,
            sequence: self.seq,
            brake_left: 1.0,
            brake_right: 1.0,
            ..Default::default()
        };
        self.seq += 1;
        bridge.frame(&frame, fresh_now());
    }

    fn needs_resync(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_run_and_replay() {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        let a = parse(&s(&["--aircraft", "c172p", "--seconds", "30", "--autostart", "--quiet"])).unwrap().unwrap();
        assert_eq!((a.aircraft, a.seconds, a.autostart, a.prompts, a.replay), (Aircraft::C172p, 30, true, false, false));
        let a = parse(&s(&["replay", "--log", "x", "--out", "y"])).unwrap().unwrap();
        assert!(a.replay);
        assert!(parse(&s(&["replay", "--log", "x"])).is_err());
        assert!(parse(&s(&["--aircraft", "b747"])).is_err());
        assert!(parse(&s(&["--stick", "neutral"])).unwrap().unwrap().neutral_stick);
        assert!(parse(&s(&["--stick", "other"])).is_err());
        assert!(parse(&s(&["--help"])).unwrap().is_none());
    }
}

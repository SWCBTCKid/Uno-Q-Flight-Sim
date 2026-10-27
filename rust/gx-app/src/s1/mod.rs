//! M1 step S1: the ARCH-004 thread layout, headless, on the real stick.
//!
//! See `docs/architecture/m1-vertical-skeleton.md` (section *S1 board
//! binary*) and `tools/s1_check.py`.

pub mod driver;
pub mod io;
pub mod phases;
pub mod rec;
pub mod rt;
pub mod runtime;

use crate::{validate_world_pack, FakeBackend, InputPublisher, WorldFrame};
use driver::{Cadence, SimDriver};
use gx_input::ControlFrame;
use gx_sim::{control_mailbox, SnapshotExchange, SIM_HZ};
use gx_world::{synthetic_sfo_like, write_world, Limits};
use io::{IoBridge, IoLog};
use phases::{Obs, Phase, Prompter};
use rec::{esc, ReadRec, Rec};
use runtime::{IoTotals, ReaderTotals, SimResult};
use std::io::Write;
use std::sync::Arc;

pub const DEFAULT_DEVICE: &str = "/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick";
pub const DEFAULT_SECONDS: u64 = 300;

pub const HELP: &str = "\
gx-app s1 - M1 step S1: headless ARCH-004 threads on the real Logitech stick

Runs the I/O thread (EvdevInput, 1 ms poll, 5 ms heartbeat) -> control
mailbox -> simulation thread (SimCore<FakeBackend>, 120 Hz absolute
CLOCK_MONOTONIC deadlines) -> SnapshotExchange -> reader thread (30 Hz,
stands in for render). No display, no sudo, read-only input (the device is
not grabbed). Logs JSONL through a bounded nonblocking drain written by the
I/O thread. Prompts the operator through the INPUT-002 phases on stdout;
ENTER on stdin is a console Start (sent straight to the sim core).

USAGE:
    gx-app s1 [OPTIONS]

OPTIONS:
    --log PATH        JSONL log (default: s1-run.jsonl)
    --seconds N       run length in seconds (default: 300)
    --device PATH     stable stick path checked at every connect (default:
                      /dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick)
    --quiet           no prompts on stdout
    --synthetic       virtual-time host replica of the full Chief session
                      (scripted stick, no device, no sleeping); for the checker
    --script NAME     with --synthetic: full (default) or no-unplug
    --help            print this help and exit

Check a log on the laptop:  python3 tools/s1_check.py s1-run.jsonl
";

pub fn synthetic_world_frame() -> Result<WorldFrame, String> {
    let mut pack = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut pack).map_err(|e| e.to_string())?;
    let world = validate_world_pack(&pack).map_err(|e| e.to_string())?;
    Ok(WorldFrame::new(Arc::new(world)))
}

struct Args {
    log: String,
    seconds: u64,
    device: String,
    prompts: bool,
    synthetic: bool,
    script: Script,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Script {
    Full,
    NoUnplug,
}

fn parse(args: &[String]) -> Result<Option<Args>, String> {
    let mut a = Args {
        log: "s1-run.jsonl".into(),
        seconds: DEFAULT_SECONDS,
        device: DEFAULT_DEVICE.into(),
        prompts: true,
        synthetic: false,
        script: Script::Full,
    };
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        let mut value = |name: &str| it.next().cloned().ok_or_else(|| format!("{name} needs a value"));
        match flag.as_str() {
            "--help" | "-h" => return Ok(None),
            "--log" => a.log = value("--log")?,
            "--seconds" => {
                let v = value("--seconds")?;
                a.seconds = v.parse().map_err(|_| format!("--seconds: not a positive integer: {v}"))?;
                if a.seconds == 0 {
                    return Err("--seconds must be > 0".into());
                }
            }
            "--device" => a.device = value("--device")?,
            "--quiet" => a.prompts = false,
            "--synthetic" => a.synthetic = true,
            "--script" => {
                a.script = match value("--script")?.as_str() {
                    "full" => Script::Full,
                    "no-unplug" => Script::NoUnplug,
                    other => return Err(format!("--script: unknown script {other}")),
                }
            }
            other => return Err(format!("unknown argument: {other} (see --help)")),
        }
    }
    Ok(Some(a))
}

/// Entry point for `gx-app s1 ...`; returns the exit code.
pub fn main(args: &[String]) -> i32 {
    let a = match parse(args) {
        Ok(Some(a)) => a,
        Ok(None) => {
            print!("{HELP}");
            return 0;
        }
        Err(e) => {
            eprintln!("gx-app s1: {e}");
            return 2;
        }
    };
    let frame = match synthetic_world_frame() {
        Ok(f) => f,
        Err(e) => {
            eprintln!("gx-app s1: world pack: {e}");
            return 1;
        }
    };
    // Evidence logs are immutable run artifacts. Refuse to truncate a prior
    // run if the operator accidentally reuses a directory or command.
    let file = match std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&a.log)
    {
        Ok(f) => f,
        Err(e) => {
            eprintln!("gx-app s1: create {}: {e}", a.log);
            return 1;
        }
    };
    let commit = option_env!("GX_COMMIT").unwrap_or("unknown");
    if a.synthetic {
        let mut out = std::io::BufWriter::new(file);
        return match synthetic_session(a.script, a.seconds, commit, frame, &mut out).and_then(|()| out.flush()) {
            Ok(()) => {
                println!("synthetic S1 session ({:?}, {} s virtual) written to {}", a.script, a.seconds, a.log);
                0
            }
            Err(e) => {
                eprintln!("gx-app s1: {e}");
                1
            }
        };
    }
    run_board(a, commit, frame, file)
}

#[cfg(target_os = "linux")]
fn run_board(a: Args, commit: &'static str, frame: WorldFrame, file: std::fs::File) -> i32 {
    let cfg = runtime::S1Config { seconds: a.seconds, prompts: a.prompts, device: a.device.clone(), commit };
    // Started here, on the unpinned main thread, so the input connector
    // helper does not inherit the I/O thread's CPU pin.
    let source = match io::EvdevSource::new(a.device.into()) {
        Ok(source) => source,
        Err(e) => {
            eprintln!("gx-app s1: input connector thread: {e}");
            return 1;
        }
    };
    let mut stdout = std::io::stdout();
    match runtime::run(cfg, rt::MonotonicClock, source, frame, file, &mut stdout) {
        Ok(code) => {
            if a.prompts {
                println!("log: {}", a.log);
            }
            code
        }
        Err(e) => {
            eprintln!("gx-app s1: {e}");
            1
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn run_board(_a: Args, _commit: &'static str, _frame: WorldFrame, _file: std::fs::File) -> i32 {
    eprintln!("gx-app s1: the board run needs Linux evdev; use --synthetic on this host");
    1
}

/// A scripted Chief: timings in seconds of virtual time.
struct Plan {
    start_at: f64,
    unplug_at: Option<f64>,
    console_at: f64,
    replug_at: f64,
    resume_at: f64,
}

impl Plan {
    fn new(script: Script) -> Self {
        Plan {
            start_at: 25.0,
            unplug_at: (script == Script::Full).then_some(50.0),
            console_at: 61.0,
            replug_at: 63.0,
            resume_at: 70.0,
        }
    }
}

/// Deterministic single-thread replica of the Chief session in virtual time:
/// the same `IoBridge`, mailbox, `SimDriver`, exchange, `Prompter` and log
/// encoding as the board run, with a scripted stick (one report every tick,
/// stamped 3 ms before the tick) and no sleeping. Lateness is zero by
/// construction, so it validates the logic and the checker, not timing.
pub fn synthetic_session(
    script: Script,
    seconds: u64,
    commit: &str,
    frame: WorldFrame,
    out: &mut impl Write,
) -> std::io::Result<()> {
    const EPOCH: u64 = 1_000_000_000;
    let plan = Plan::new(script);
    let exchange = Arc::new(SnapshotExchange::new());
    let (writer, reader) = control_mailbox();
    let mut bridge = IoBridge::new(InputPublisher::new(writer));
    let mut log = IoLog::new(EPOCH);
    let mut recs: Vec<Rec> = Vec::new();
    writeln!(
        out,
        "{{\"k\":\"start\",\"t\":0,\"schema\":\"{}\",\"commit\":{},\"seconds\":{seconds},\"device\":\"synthetic\",\"synthetic\":true,\"script\":\"{script:?}\",\"clock\":\"virtual\",\"sim_hz\":{SIM_HZ},\"reader_hz\":30,\"tick_ns\":{}}}",
        rec::SCHEMA,
        esc(commit),
        driver::TICK_NS
    )?;
    let mut driver = SimDriver::new(FakeBackend::new(frame), Arc::clone(&exchange), reader, EPOCH, &mut |r| recs.push(r));
    let mut cadence = Cadence::new(EPOCH);
    let mut prompter = Prompter::new();
    let mut reader_totals = ReaderTotals::default();
    let mut seq = 0u64;
    let mut connected = true;
    let mut console = 0u64;
    let mut pressed_start = [false, false];
    let end = EPOCH + seconds * 1_000_000_000;
    bridge.connected();
    log.io(EPOCH, "\"ev\":\"connected\",\"path\":\"synthetic\",\"by_id_match\":true");
    let mut tick = 0u64;
    loop {
        let due = cadence.next_due();
        if due >= end {
            break;
        }
        let t_s = (due - EPOCH) as f64 / 1e9;
        let io_t = due - 3_000_000;
        // Scripted stick and console.
        let unplugged = plan.unplug_at.is_some_and(|u| t_s >= u && t_s < plan.replug_at);
        if unplugged && connected {
            connected = false;
            bridge.disconnected(io_t);
            log.io(io_t, "\"ev\":\"disconnected\"");
        }
        if !unplugged && !connected {
            connected = true;
            bridge.connected();
            log.io(io_t, "\"ev\":\"connected\",\"path\":\"synthetic\",\"by_id_match\":true");
        }
        if plan.unplug_at.is_some() && t_s >= plan.console_at && console == 0 {
            console = 1;
        }
        if connected {
            let mut buttons = 0;
            for (i, at) in [plan.start_at, plan.resume_at].into_iter().enumerate() {
                if t_s >= at && !pressed_start[i] && (i == 0 || plan.unplug_at.is_some()) {
                    pressed_start[i] = true;
                    buttons = 1 << crate::button::START;
                }
            }
            let f = ControlFrame {
                timestamp_ns: io_t,
                sequence: seq,
                roll: ((t_s * 0.7).sin() * 0.8) as f32,
                pitch: ((t_s * 0.3).cos() * 0.3) as f32,
                throttle: 0.4,
                buttons,
                buttons_pressed: buttons,
                ..ControlFrame::default()
            };
            seq += 1;
            bridge.frame(&f, io_t);
            if buttons != 0 {
                log.io(io_t, &io::button_body(&f));
            }
        } else {
            bridge.heartbeat_if_due(io_t, false);
        }

        let vis = driver.tick(due, console, &mut || due + 200_000, &mut |r| recs.push(r));
        let second = cadence.complete(due, due + 200_000);
        if let Some(miss) = cadence.take_miss() {
            recs.push(Rec::DeadlineMiss(miss));
        }
        if let Some(mut second) = second {
            second.busy_events = driver.publish_stats().busy_events;
            recs.push(Rec::SimSecond(second));
        }
        tick += 1;
        if tick % 4 == 0 {
            let now = due + 1_000_000;
            if let Some(pair) = exchange.try_latest_pair() {
                let paused = vis.run != gx_sim::RunState::Running;
                let age_ns = now - pair.latest.published_at_ns;
                if !paused {
                    reader_totals.age_max_running_ns = reader_totals.age_max_running_ns.max(age_ns);
                }
                reader_totals.reads += 1;
                recs.push(Rec::Read(ReadRec {
                    t: now,
                    ok: true,
                    seq: pair.latest.sequence,
                    generation: pair.latest.generation,
                    tick: pair.latest.simulation_tick,
                    age_ns,
                    copy_ns: 1_000,
                    paused,
                    pair: pair.previous.is_some(),
                    warning: vis.warning,
                }));
            }
        }
        if tick % 12 == 0 {
            let obs = Obs {
                t_ns: due - EPOCH,
                end_ns: end - EPOCH,
                connected,
                frames: bridge.frames,
                run: vis.run,
                pause: vis.pause,
                status: vis.status,
                loss_ms: driver.loss_ms(),
                console_starts: console,
                refused_console: driver.totals.refused_core_console,
                accepted_console: driver.totals.accepted_console,
                button_presses: bridge.button_presses,
                last_buttons: bridge.last_buttons_pressed,
            };
            for (phase, text) in prompter.step(&obs) {
                log.lines.push(format!(
                    "{{\"k\":\"phase\",\"t\":{},\"phase\":\"{}\",\"text\":{}}}",
                    due - EPOCH,
                    phase.name(),
                    esc(text.trim())
                ));
            }
        }
        for line in log.lines.drain(..) {
            writeln!(out, "{line}")?;
        }
        for r in recs.drain(..) {
            writeln!(out, "{}", rec::to_json(&r, EPOCH))?;
        }
    }
    let obs_end = Obs {
        t_ns: end - EPOCH,
        end_ns: end - EPOCH,
        connected,
        frames: bridge.frames,
        run: driver.run_state(),
        pause: None,
        status: gx_sim::InputStatus::Valid,
        loss_ms: 0,
        console_starts: console,
        refused_console: driver.totals.refused_core_console,
        accepted_console: driver.totals.accepted_console,
        button_presses: bridge.button_presses,
        last_buttons: 0,
    };
    for (phase, text) in prompter.step(&obs_end) {
        writeln!(
            out,
            "{{\"k\":\"phase\",\"t\":{},\"phase\":\"{}\",\"text\":{}}}",
            end - EPOCH,
            phase.name(),
            esc(text.trim())
        )?;
    }
    driver.finish(end, &mut |r| recs.push(r));
    for r in recs.drain(..) {
        writeln!(out, "{}", rec::to_json(&r, EPOCH))?;
    }
    debug_assert_eq!(prompter.phase, Phase::Done);
    let mut totals = driver.totals;
    totals.ticks = cadence.totals_ticks;
    totals.misses = cadence.totals_misses;
    totals.skipped = cadence.totals_skipped;
    totals.late_max_ns = cadence.late_max_ns;
    totals.work_max_ns = cadence.work_max_ns;
    totals.late_hist = cadence.late_hist;
    let sim = SimResult {
        totals,
        publish: driver.publish_stats(),
        mailbox: driver.mailbox_stats(),
        rt: "{\"policy\":\"virtual\"}".into(),
    };
    let io = IoTotals {
        publications: bridge.publications,
        max_gap_ns: bridge.max_gap_ns,
        max_loop_ns: 0,
        frames: bridge.frames,
        button_presses: bridge.button_presses,
        lines: 0,
        log_drops: 0,
    };
    let v = "{\"policy\":\"virtual\"}";
    writeln!(out, "{}", runtime::summary_line(end - EPOCH, &sim, &reader_totals, &io, [v, v, v, v, v]))?;
    writeln!(out, "{{\"k\":\"end\",\"t\":{}}}", end - EPOCH)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn session(script: Script, seconds: u64) -> String {
        let mut out = Vec::new();
        synthetic_session(script, seconds, "test", synthetic_world_frame().unwrap(), &mut out).unwrap();
        String::from_utf8(out).unwrap()
    }

    #[test]
    fn synthetic_full_session_shows_the_input002_sequence() {
        let log = session(Script::Full, 90);
        let lines: Vec<&str> = log.lines().collect();
        let find = |needle: &str| lines.iter().position(|l| l.contains(needle));
        let started = find("\"k\":\"start_accepted\",").expect("first start");
        let warning = find("\"run\":\"running\",\"pause\":null,\"status\":\"unavailable:disconnected\",\"warning\":true")
            .expect("warning while running");
        let paused = find("\"pause\":\"input_loss\",\"status\":\"unavailable:disconnected\"").expect("loss pause");
        let pause_line = lines[paused];
        let loss_ms: u64 = pause_line.rsplit("\"loss_ms\":").next().unwrap().trim_end_matches('}').parse().unwrap();
        assert!((10_000..10_000 + 9).contains(&loss_ms), "{pause_line}");
        let refused = find("\"k\":\"start_refused\",").expect("refused console start");
        assert!(lines[refused].contains("\"source\":\"console\""));
        let cleared = find("\"pause\":\"input_loss\",\"status\":\"valid\",\"warning\":false").expect("warning cleared while paused");
        let resumed = lines
            .iter()
            .rposition(|l| l.contains("\"k\":\"start_accepted\"") && l.contains("\"from_pause\":\"input_loss\""))
            .expect("resume from input_loss");
        assert!(started < warning && warning < paused && paused < refused && refused < cleared && cleared < resumed);
        assert!(log.contains("\"k\":\"summary\""));
        assert!(log.contains("\"phase\":\"free\""), "prompter reached free flight");
        assert!(log.lines().filter(|l| l.contains("\"k\":\"lat\"")).count() > 1000);
        // Every line is one JSON object.
        assert!(lines.iter().all(|l| l.starts_with('{') && l.ends_with('}')));
    }

    #[test]
    fn synthetic_no_unplug_session_has_no_loss() {
        let log = session(Script::NoUnplug, 60);
        assert!(!log.contains("input_loss\""));
        assert!(!log.contains("\"ev\":\"disconnected\""));
        assert!(log.contains("TIME UP during phase"));
    }

    #[test]
    fn parse_rejects_unknown_and_zero_seconds() {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        assert!(parse(&s(&["--bogus"])).is_err());
        assert!(parse(&s(&["--seconds", "0"])).is_err());
        assert!(parse(&s(&["--help"])).unwrap().is_none());
        let a = parse(&s(&["--seconds", "10", "--log", "x.jsonl", "--quiet"])).unwrap().unwrap();
        assert_eq!((a.seconds, a.log.as_str(), a.prompts), (10, "x.jsonl", false));
    }
}

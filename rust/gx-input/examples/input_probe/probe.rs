use gx_input::linux::{EvdevInput, PollStats, RuntimeEvent};
use gx_input::resync::KEY_MAX;
use gx_input::{ConnectionState, ControlFrame, DeviceIdentity, InputConfig, ResyncTrigger};
use std::fmt::Write as _;
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

const DEFAULT_DEVICE: &str = "/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick";
const DEFAULT_SECONDS: u64 = 35;
const DEFAULT_STALL_AT: u64 = 18;
const DEFAULT_STALL_MS: u64 = 3000;
const POLL_SLEEP: Duration = Duration::from_millis(1);

const HELP: &str = "\
input_probe - headless Logitech Extreme 3D Pro evdev smoke probe (TEST-027)

Reads the stick read-only through gx-input's EvdevInput and writes every
RuntimeEvent and ControlFrame as JSON lines. t_ns is monotonic time since
probe start (std::time::Instant). Opens no display, needs no sudo, does not
grab the device.

USAGE:
    input_probe [OPTIONS]

OPTIONS:
    --device PATH     evdev node to verify against (default:
                      /dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick)
    --out PATH        JSONL output file (default: input-probe.jsonl)
    --seconds N       hard stop after N seconds (default: 35)
    --stall-at S      stop reading at S seconds (default: 18)
    --stall-ms M      stall length in ms; 0 disables (default: 3000)
    --phase-prompts   print large operator prompts to stdout at fixed times
    --help            print this help and exit

Default timeline (with --phase-prompts):
    0 s  HANDS OFF          3 s  MOVE STICK NOW      10 s PRESS TRIGGER AND BUTTONS
    15 s START WIGGLING     18 s WIGGLE HARD NOW (stall, 3 s)
    21 s KEEP MOVING        32 s STOP                35 s end

Exit codes: 0 completed, 1 usage/IO error, 3 matched device is not --device.
";

struct Args {
    device: PathBuf,
    out: PathBuf,
    seconds: u64,
    stall_at: u64,
    stall_ms: u64,
    prompts: bool,
}

fn parse_args() -> Result<Option<Args>, String> {
    let mut args = Args {
        device: DEFAULT_DEVICE.into(),
        out: "input-probe.jsonl".into(),
        seconds: DEFAULT_SECONDS,
        stall_at: DEFAULT_STALL_AT,
        stall_ms: DEFAULT_STALL_MS,
        prompts: false,
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = |name: &str| it.next().ok_or_else(|| format!("{name} needs a value"));
        let number = |name: &str, v: String| {
            v.parse::<u64>()
                .map_err(|_| format!("{name}: not a non-negative integer: {v}"))
        };
        match flag.as_str() {
            "--help" | "-h" => return Ok(None),
            "--device" => args.device = value("--device")?.into(),
            "--out" => args.out = value("--out")?.into(),
            "--seconds" => args.seconds = number("--seconds", value("--seconds")?)?,
            "--stall-at" => args.stall_at = number("--stall-at", value("--stall-at")?)?,
            "--stall-ms" => args.stall_ms = number("--stall-ms", value("--stall-ms")?)?,
            "--phase-prompts" => args.prompts = true,
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    if args.stall_ms > 0 {
        if args.stall_at < 11 {
            return Err("--stall-at must be >= 11 (movement and button phases come first)".into());
        }
        let stall_end_s = args.stall_at * 1000 + args.stall_ms;
        if args.seconds * 1000 < stall_end_s + 8_000 {
            return Err(
                "--seconds must leave >= 8 s after the stall for post-resync movement".into(),
            );
        }
    }
    if args.seconds == 0 {
        return Err("--seconds must be > 0".into());
    }
    Ok(Some(args))
}

/// Prompt schedule in ms since start. Derived from the stall so one run covers
/// movement, buttons, a forced overflow and post-resync movement.
fn timeline(a: &Args) -> Vec<(u64, &'static str)> {
    let end = a.seconds * 1000;
    let mut t = vec![
        (0, "HANDS OFF - CONNECTING"),
        (3_000, "MOVE STICK NOW - ROLL, PITCH, TWIST, THROTTLE"),
    ];
    if a.stall_ms > 0 {
        let s = a.stall_at * 1000;
        t.push((s - 8_000, "PRESS TRIGGER AND BUTTONS NOW"));
        t.push((s - 3_000, "GET READY - START WIGGLING HARD"));
        t.push((s, "WIGGLE HARD NOW (stall) - DO NOT STOP"));
        t.push((s + a.stall_ms, "KEEP MOVING STICK AND PRESS TRIGGER"));
    } else {
        t.push((end / 2, "PRESS TRIGGER AND BUTTONS NOW"));
    }
    t.push((end.saturating_sub(3_000), "STOP - HANDS OFF"));
    t
}

fn banner(text: &str) {
    let bar = "#".repeat(text.len() + 12);
    println!("\n{bar}\n#####  {text}  #####\n{bar}\n");
}

fn esc(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            c if (c as u32) < 0x20 => {
                let _ = write!(o, "\\u{:04x}", c as u32);
            }
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn opt_str(s: &Option<String>) -> String {
    s.as_deref().map_or("null".into(), esc)
}

fn device_json(d: &DeviceIdentity) -> String {
    format!(
        "{{\"event_path\":{},\"name\":{},\"vendor_id\":{},\"product_id\":{},\"version\":{},\"phys\":{},\"uniq\":{}}}",
        esc(&d.event_path),
        esc(&d.name),
        d.vendor_id,
        d.product_id,
        d.version,
        opt_str(&d.physical_path),
        opt_str(&d.unique_id)
    )
}

fn frame_json(f: &ControlFrame) -> String {
    format!(
        "\"frame\":{{\"timestamp_ns\":{},\"sequence\":{},\"source_event_sequence\":{},\"roll\":{:?},\"pitch\":{:?},\"yaw\":{:?},\"throttle\":{:?},\"brake_left\":{:?},\"brake_right\":{:?},\"hat_x\":{:?},\"hat_y\":{:?},\"buttons\":{},\"buttons_pressed\":{},\"buttons_released\":{}}}",
        f.timestamp_ns,
        f.sequence,
        f.source_event_sequence,
        f.roll,
        f.pitch,
        f.yaw,
        f.throttle,
        f.brake_left,
        f.brake_right,
        f.hat_x,
        f.hat_y,
        f.buttons,
        f.buttons_pressed,
        f.buttons_released
    )
}

fn trigger_name(t: ResyncTrigger) -> &'static str {
    match t {
        ResyncTrigger::ReportBoundary => "report_boundary",
        ResyncTrigger::EventLimit => "event_limit",
        ResyncTrigger::Timeout => "timeout",
    }
}

fn event_json(t_ns: u64, e: &RuntimeEvent) -> String {
    let body = match e {
        RuntimeEvent::State(s) => match s {
            ConnectionState::Disconnected => "\"kind\":\"state\",\"state\":\"disconnected\"".into(),
            ConnectionState::Connected(d) => format!(
                "\"kind\":\"state\",\"state\":\"connected\",\"device\":{}",
                device_json(d)
            ),
            ConnectionState::ResyncRequired { device, loss } => format!(
                "\"kind\":\"state\",\"state\":\"resync_required\",\"device\":{},\"loss\":{{\"timestamp_ns\":{},\"source_event_sequence\":{}}}",
                device_json(device),
                loss.timestamp_ns,
                loss.source_event_sequence
            ),
            ConnectionState::Error {
                message,
                retry_at_ns,
            } => format!(
                "\"kind\":\"state\",\"state\":\"error\",\"message\":{},\"retry_at_ns\":{}",
                esc(message),
                retry_at_ns
            ),
        },
        RuntimeEvent::Input(i) => format!(
            "\"kind\":\"input\",\"timestamp_ns\":{},\"sequence\":{},\"type\":{},\"code\":{},\"value\":{}",
            i.timestamp_ns, i.sequence, i.event_type, i.code, i.value
        ),
        RuntimeEvent::Frame(f) => format!("\"kind\":\"frame\",{}", frame_json(f)),
        RuntimeEvent::LossDetected(l) => format!(
            "\"kind\":\"loss_detected\",\"timestamp_ns\":{},\"source_event_sequence\":{}",
            l.timestamp_ns, l.source_event_sequence
        ),
        RuntimeEvent::ResyncRequired(r) => format!(
            "\"kind\":\"resync_required\",\"loss_timestamp_ns\":{},\"loss_source_event_sequence\":{},\"discarded_events\":{},\"trigger\":\"{}\"",
            r.loss.timestamp_ns,
            r.loss.source_event_sequence,
            r.discarded_events,
            trigger_name(r.trigger)
        ),
        RuntimeEvent::DeviceStateRead(live) => {
            let axes: Vec<String> = live
                .axes
                .iter()
                .map(|a| {
                    format!(
                        "{{\"code\":{},\"value\":{},\"minimum\":{},\"maximum\":{}}}",
                        a.code, a.value, a.minimum, a.maximum
                    )
                })
                .collect();
            let keys: Vec<String> = (0..=KEY_MAX)
                .filter(|&c| live.key_pressed(c))
                .map(|c| c.to_string())
                .collect();
            format!(
                "\"kind\":\"device_state_read\",\"axes\":[{}],\"pressed_keys\":[{}]",
                axes.join(","),
                keys.join(",")
            )
        }
        RuntimeEvent::ResyncCompleted {
            snapshot,
            frame_sequence,
        } => {
            let axes: Vec<String> = snapshot
                .axes
                .iter()
                .map(|(c, v)| format!("[{c},{v}]"))
                .collect();
            let buttons: Vec<String> = snapshot
                .pressed_buttons
                .iter()
                .map(|c| c.to_string())
                .collect();
            format!(
                "\"kind\":\"resync_completed\",\"frame_sequence\":{},\"snapshot\":{{\"timestamp_ns\":{},\"source_event_sequence\":{},\"axes\":[{}],\"pressed_buttons\":[{}]}}",
                frame_sequence,
                snapshot.timestamp_ns,
                snapshot.source_event_sequence,
                axes.join(","),
                buttons.join(",")
            )
        }
        RuntimeEvent::ResyncAttemptFailed {
            attempt,
            reason,
            retry_at_ns,
        } => format!(
            "\"kind\":\"resync_attempt_failed\",\"attempt\":{},\"reason\":{},\"retry_at_ns\":{}",
            attempt,
            esc(reason),
            retry_at_ns.map_or("null".into(), |v| v.to_string())
        ),
        RuntimeEvent::Background {
            operation,
            duration_ns,
        } => format!("\"kind\":\"background\",\"operation\":\"{operation}\",\"duration_ns\":{duration_ns}"),
    };
    format!("{{\"t_ns\":{t_ns},{body}}}")
}

fn stats_add(total: &mut PollStats, s: PollStats) {
    total.bytes_read += s.bytes_read;
    total.events_read += s.events_read;
    total.frames_emitted += s.frames_emitted;
    total.losses_detected += s.losses_detected;
    total.suppressed_events += s.suppressed_events;
    total.resyncs_required += s.resyncs_required;
    total.resyncs_completed += s.resyncs_completed;
    total.resync_failures += s.resync_failures;
}

fn event_device(e: &RuntimeEvent) -> Option<&DeviceIdentity> {
    match e {
        RuntimeEvent::State(ConnectionState::Connected(d))
        | RuntimeEvent::State(ConnectionState::ResyncRequired { device: d, .. }) => Some(d),
        _ => None,
    }
}

pub fn run() -> i32 {
    let args = match parse_args() {
        Ok(Some(a)) => a,
        Ok(None) => {
            print!("{HELP}");
            return 0;
        }
        Err(e) => {
            eprintln!("input_probe: {e}\n\n{HELP}");
            return 1;
        }
    };
    match probe(&args) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("input_probe: {e}");
            1
        }
    }
}

fn probe(args: &Args) -> io::Result<i32> {
    let resolved = fs::canonicalize(&args.device).map_err(|e| {
        io::Error::new(
            e.kind(),
            format!("{}: {e} (is the stick plugged in?)", args.device.display()),
        )
    })?;
    let resolved = resolved.to_string_lossy().into_owned();
    let mut out = BufWriter::new(File::create(&args.out)?);
    let start = Instant::now();
    let now_ns = || start.elapsed().as_nanos() as u64;
    writeln!(
        out,
        "{{\"t_ns\":{},\"kind\":\"start\",\"probe\":\"input_probe\",\"commit\":{},\"device\":{},\"resolved\":{},\"seconds\":{},\"stall_at_s\":{},\"stall_ms\":{},\"phase_prompts\":{},\"clock\":\"monotonic_since_start\"}}",
        now_ns(),
        esc(option_env!("GX_PROBE_COMMIT").unwrap_or("unknown")),
        esc(&args.device.to_string_lossy()),
        esc(&resolved),
        args.seconds,
        args.stall_at,
        args.stall_ms,
        args.prompts
    )?;

    let mut input = EvdevInput::new(InputConfig::extreme_3d_pro());
    let prompts = timeline(args);
    let mut next_prompt = 0;
    let stall_start_ns = args.stall_at * 1_000_000_000;
    let mut stall_done = args.stall_ms == 0;
    let end_ns = args.seconds * 1_000_000_000;
    let mut total = PollStats::default();
    let mut wrong_device: Option<String> = None;
    let mut lines = 0_u64;

    loop {
        let now = now_ns();
        if now >= end_ns {
            break;
        }
        while next_prompt < prompts.len() && now >= prompts[next_prompt].0 * 1_000_000 {
            let text = prompts[next_prompt].1;
            if args.prompts {
                banner(text);
            }
            writeln!(
                out,
                "{{\"t_ns\":{now},\"kind\":\"prompt\",\"text\":{}}}",
                esc(text)
            )?;
            next_prompt += 1;
        }
        if !stall_done && now >= stall_start_ns {
            stall_done = true;
            writeln!(
                out,
                "{{\"t_ns\":{now},\"kind\":\"stall_begin\",\"stall_ms\":{}}}",
                args.stall_ms
            )?;
            out.flush()?;
            std::thread::sleep(Duration::from_millis(args.stall_ms));
            writeln!(out, "{{\"t_ns\":{},\"kind\":\"stall_end\"}}", now_ns())?;
            continue;
        }
        let mut write_err = None;
        let stats = input.poll_nonblocking(now, |e| {
            if let Some(d) = event_device(&e) {
                if d.event_path != resolved && wrong_device.is_none() {
                    wrong_device = Some(d.event_path.clone());
                }
            }
            lines += 1;
            if let Err(err) = writeln!(out, "{}", event_json(now, &e)) {
                write_err.get_or_insert(err);
            }
        });
        if let Some(e) = write_err {
            return Err(e);
        }
        stats_add(&mut total, stats);
        if let Some(path) = wrong_device.take() {
            writeln!(
                out,
                "{{\"t_ns\":{},\"kind\":\"abort\",\"reason\":\"matched device {} is not {}\"}}",
                now_ns(),
                path,
                resolved
            )?;
            out.flush()?;
            eprintln!("input_probe: matched device {path} is not {resolved}; aborting");
            return Ok(3);
        }
        if stats.events_read > 0 || stats.resyncs_required > 0 {
            out.flush()?;
        }
        std::thread::sleep(POLL_SLEEP);
    }
    writeln!(
        out,
        "{{\"t_ns\":{},\"kind\":\"end\",\"lines\":{},\"stats\":{{\"bytes_read\":{},\"events_read\":{},\"frames_emitted\":{},\"losses_detected\":{},\"suppressed_events\":{},\"resyncs_required\":{},\"resyncs_completed\":{},\"resync_failures\":{}}}}}",
        now_ns(),
        lines,
        total.bytes_read,
        total.events_read,
        total.frames_emitted,
        total.losses_detected,
        total.suppressed_events,
        total.resyncs_required,
        total.resyncs_completed,
        total.resync_failures
    )?;
    out.flush()?;
    println!(
        "input_probe: done; {} events, {} frames, {} losses, {} resyncs completed -> {}",
        total.events_read,
        total.frames_emitted,
        total.losses_detected,
        total.resyncs_completed,
        args.out.display()
    );
    Ok(0)
}

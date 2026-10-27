//! M1 step S3 (first pixels): the S2 layout (JSBSim global5000, real stick,
//! control mailbox, INPUT-002, FIFO sim/I/O threads) with a render thread
//! that exclusively owns the display. See `runtime` and `gx_render::linux`.

pub mod mouse;
pub mod runtime;

use crate::s2::runtime::{run_with_ap, ApExtras, Extras, S2Config, SnapshotCtx};
use crate::s2::setup::Aircraft;
use crate::s2::{commit, create_new, world, NeutralStick};
use crate::s1::rt::{Clock, MonotonicClock};
use crate::FakeBackend;
use std::sync::atomic::AtomicBool;

/// Which step's binary this is: S3 (horizon) or S4 (SFO world). Everything
/// else (threads, CLI, signals) is shared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    S3,
    S4,
}

impl Step {
    fn name(self) -> &'static str {
        match self {
            Step::S3 => "s3",
            Step::S4 => "s4",
        }
    }
}

pub const SCHEMA: &str = "gx-s3/1";
pub const DEFAULT_SECONDS: u64 = 240;
pub const DEFAULT_CARD: &str = "/dev/dri/card0";
pub const DEFAULT_RENDER_NODE: &str = "/dev/dri/renderD128";

pub const HELP: &str = "\
gx-app s3 - M1 step S3: first pixels

The S2 thread layout (I/O SCHED_FIFO 40 -> control mailbox -> JSBSim sim
SCHED_FIFO 50 at 120 Hz -> SnapshotExchange) with a render thread
(SCHED_OTHER, CPU 3) that owns DRM/KMS + GBM + EGL/GLES and presents at
30 fps: artificial horizon, IAS/heading/AGL numbers, the INPUT-002 banner,
a PAUSED overlay and the operator prompts. The sim never waits on it.

  --display kms        DP-1 1280x720@60 on /dev/dri/card0: needs DRM master,
                       i.e. LightDM stopped. Run ONLY through the M0-001C
                       watchdog launcher (tools/m1_s3_launch.sh).
  --display offscreen  the same frames on the render node, nothing on screen
                       (non-display board smoke; no master, desktop untouched)
  --display fake       no GPU (host development)

USAGE:
    gx-app s3 [OPTIONS]

OPTIONS:
    --log PATH          JSONL log, created new (default: s3-run.jsonl)
    --seconds N         run length in seconds (default: 240)
    --display MODE      kms (default), offscreen or fake
    --card PATH         KMS device (default: /dev/dri/card0)
    --render-node PATH  offscreen device (default: /dev/dri/renderD128)
    --device PATH       stable stick path (default: the Extreme 3D Pro by-id path)
    --aircraft NAME     global5000 (default), 787-8 or c172p
    --jsbsim-root DIR   JSBSim data root (default: ./jsbsim)
    --stick neutral     smoke only: synthetic centred stick (see gx-app s2 --help)
    --autostart         one console Start as soon as input is valid (smoke)
    --mouse PATH|none   autopilot panel mouse (default: the first
                        /dev/input/by-id/*-event-mouse, re-scanned each second;
                        none = no mouse thread, the panel shows NO MOUSE)
    --fake              FakeBackend instead of JSBSim (host development only)
    --quiet             no prompts on stdout
    --help              print this help and exit

SIGTERM/SIGINT/SIGHUP end the run like the time limit (display restored,
log complete). Check: python3 tools/s3_check.py s3-run.jsonl
";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Display {
    Kms,
    Offscreen,
    Fake,
}

pub(crate) struct Args {
    log: String,
    seconds: u64,
    display: Display,
    card: String,
    render_node: String,
    device: String,
    aircraft: Aircraft,
    jsbsim_root: String,
    neutral_stick: bool,
    autostart: bool,
    fake: bool,
    prompts: bool,
    /// SPIKE-002 (S4 only): GXWORLD pack file instead of the built-in
    /// synthetic one, GXVISUAL sidecar (selects the treatment renderer),
    /// offscreen trace and capture directory.
    world: Option<String>,
    visual: Option<String>,
    trace: bool,
    capture_dir: Option<String>,
    /// AP-001 panel mouse; `None` = `--mouse none`.
    mouse: Option<mouse::MouseSelect>,
}

#[cfg(test)]
fn parse(args: &[String]) -> Result<Option<Args>, String> {
    parse_step(args, Step::S3)
}

pub(crate) fn parse_step(args: &[String], step: Step) -> Result<Option<Args>, String> {
    let mut a = Args {
        log: format!("{}-run.jsonl", step.name()),
        seconds: DEFAULT_SECONDS,
        display: Display::Kms,
        card: DEFAULT_CARD.into(),
        render_node: DEFAULT_RENDER_NODE.into(),
        device: crate::s1::DEFAULT_DEVICE.into(),
        aircraft: Aircraft::Global5000,
        jsbsim_root: crate::s2::DEFAULT_JSBSIM_ROOT.into(),
        neutral_stick: false,
        autostart: false,
        fake: false,
        prompts: true,
        world: None,
        visual: None,
        trace: false,
        capture_dir: None,
        mouse: Some(mouse::MouseSelect::Auto),
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
            "--display" => {
                a.display = match value("--display")?.as_str() {
                    "kms" => Display::Kms,
                    "offscreen" => Display::Offscreen,
                    "fake" => Display::Fake,
                    other => return Err(format!("--display: kms, offscreen or fake, not {other}")),
                }
            }
            "--card" => a.card = value("--card")?,
            "--render-node" => a.render_node = value("--render-node")?,
            "--device" => a.device = value("--device")?,
            "--aircraft" => {
                let v = value("--aircraft")?;
                a.aircraft = Aircraft::parse(&v).ok_or_else(|| format!("--aircraft: global5000, 787-8 or c172p, not {v}"))?;
            }
            "--jsbsim-root" => a.jsbsim_root = value("--jsbsim-root")?,
            "--stick" => match value("--stick")?.as_str() {
                "neutral" => a.neutral_stick = true,
                "device" => a.neutral_stick = false,
                other => return Err(format!("--stick: device or neutral, not {other}")),
            },
            "--mouse" => {
                let v = value("--mouse")?;
                a.mouse = match v.as_str() {
                    "none" => None,
                    "" => return Err("--mouse: a path or none".into()),
                    path => Some(mouse::MouseSelect::Path(path.into())),
                };
            }
            "--autostart" => a.autostart = true,
            "--fake" => a.fake = true,
            "--quiet" => a.prompts = false,
            "--world" if step == Step::S4 => a.world = Some(value("--world")?),
            "--visual" if step == Step::S4 => a.visual = Some(value("--visual")?),
            "--trace" if step == Step::S4 => a.trace = true,
            "--capture-dir" if step == Step::S4 => a.capture_dir = Some(value("--capture-dir")?),
            other => return Err(format!("unknown argument: {other} (see --help)")),
        }
    }
    if a.visual.is_some() && a.world.is_none() {
        return Err("--visual needs the matching --world pack".into());
    }
    if (a.trace || a.capture_dir.is_some()) && a.display == Display::Kms {
        return Err("--trace/--capture-dir are for --display offscreen or fake only".into());
    }
    if a.capture_dir.is_some() && !a.trace {
        return Err("--capture-dir needs --trace".into());
    }
    Ok(Some(a))
}

/// Set by SIGTERM/SIGINT/SIGHUP; the runtime ends the run like the time limit.
pub static STOP: AtomicBool = AtomicBool::new(false);

#[cfg(target_os = "linux")]
fn install_stop_signals() {
    extern "C" fn on_signal(_: libc::c_int) {
        STOP.store(true, std::sync::atomic::Ordering::Relaxed);
    }
    for sig in [libc::SIGTERM, libc::SIGINT, libc::SIGHUP] {
        // SAFETY: the handler only stores to an atomic (async-signal-safe).
        unsafe {
            let mut sa: libc::sigaction = std::mem::zeroed();
            sa.sa_sigaction = on_signal as extern "C" fn(libc::c_int) as usize;
            sa.sa_flags = libc::SA_RESTART;
            libc::sigemptyset(&mut sa.sa_mask);
            libc::sigaction(sig, &sa, std::ptr::null_mut());
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn install_stop_signals() {}

/// Entry point for `gx-app s3 ...`; returns the exit code.
pub fn main(args: &[String]) -> i32 {
    main_step(args, Step::S3, HELP)
}

pub(crate) fn main_step(args: &[String], step: Step, help: &str) -> i32 {
    let a = match parse_step(args, step) {
        Ok(Some(a)) => a,
        Ok(None) => {
            print!("{help}");
            return 0;
        }
        Err(e) => {
            eprintln!("gx-app {}: {e}", step.name());
            return 2;
        }
    };
    if std::env::var_os("JSBSIM_DEBUG").is_none() {
        std::env::set_var("JSBSIM_DEBUG", "0");
    }
    match run_main(a, step) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("gx-app {}: {e}", step.name());
            1
        }
    }
}

fn run_main(a: Args, step: Step) -> Result<i32, String> {
    let (frame, threshold) = match &a.world {
        None => world()?,
        Some(path) => {
            let frame = crate::spike002::world_frame_from_pack(path)?;
            let threshold = crate::s2::setup::threshold(&frame)?;
            (frame, threshold)
        }
    };
    let photo = match &a.visual {
        None => None,
        Some(path) => {
            let t0 = MonotonicClock.now_ns();
            let file = std::fs::File::open(path).map_err(|e| format!("visual {path}: {e}"))?;
            let file_bytes = file.metadata().map_err(|e| format!("visual {path}: {e}"))?.len();
            let visual = gx_world::read_visual(std::io::BufReader::with_capacity(1 << 20, file), gx_world::VisualLimits::default())
                .map_err(|e| format!("visual {path}: {e}"))?;
            if !visual.matches_world(frame.world()) {
                return Err(format!("visual {path}: frame identity does not match the world pack"));
            }
            let read_ns = MonotonicClock.now_ns() - t0;
            Some(std::sync::Arc::new(runtime::PhotoAsset { visual, path: path.clone(), file_bytes, read_ns }))
        }
    };
    if a.trace {
        let opts = crate::spike002::RenderOpts {
            trace: Some(crate::spike002::TraceOrigin {
                threshold_enu: threshold.enu_m,
                heading_deg: threshold.heading_deg,
                runway_z: threshold.surface_msl_m - (frame.origin().ellipsoid_height_m - frame.msl_to_ellipsoid_m()),
            }),
            capture_dir: a.capture_dir.clone(),
        };
        let _ = crate::spike002::RENDER_OPTS.set(opts);
    }
    let file = create_new(&a.log)?;
    gx_render::aircraft::set_active(match a.aircraft {
        Aircraft::B7878 => gx_render::aircraft::Geometry::b7878(),
        _ => gx_render::aircraft::Geometry::global5000(),
    });
    let _ = runtime::HUD_FLAP_DETENTS.set(a.aircraft.flap_detents().iter().map(|d| (d.norm, d.label)).collect());
    let approaches = crate::s2::setup::approaches(&frame);
    let _ = runtime::HUD_APPROACHES.set(approaches.iter().map(|a| a.name().to_owned()).collect());
    let ap = ApExtras { mouse: a.mouse.clone(), approaches };
    let cfg = S2Config {
        seconds: a.seconds,
        prompts: a.prompts,
        device: if a.neutral_stick { "synthetic:neutral".into() } else { a.device.clone() },
        commit: commit(),
        aircraft: a.aircraft,
        backend: if a.fake { "fake" } else { "jsbsim" },
        threshold,
        autostart: a.autostart,
    };
    install_stop_signals();
    let look = match step {
        Step::S3 => runtime::Look::Horizon,
        Step::S4 => match photo {
            None => runtime::Look::World(frame.world_arc()),
            Some(photo) => runtime::Look::Photo(frame.world_arc(), photo),
        },
    };
    if a.fake {
        return run_board(a, cfg, ap, FakeBackend::new(frame), file, step, look);
    }
    #[cfg(feature = "jsbsim")]
    {
        let backend = crate::s2::jsbsim_backend(&a.jsbsim_root, a.aircraft, &threshold, &frame)?;
        run_board(a, cfg, ap, backend, file, step, look)
    }
    #[cfg(not(feature = "jsbsim"))]
    {
        let _ = (cfg, ap, file, look);
        Err("this gx-app was built without the jsbsim feature (use --fake on a host)".into())
    }
}


fn extras(step: Step, direct_free_flight: bool) -> Extras {
    match step {
        Step::S3 => Extras { schema: SCHEMA, free_flight: true, stop: Some(&STOP), ..Extras::default() },
        Step::S4 => Extras {
            schema: crate::s4::SCHEMA,
            free_flight: true,
            direct_free_flight,
            stop: Some(&STOP),
            title: Some("S4 SFO WORLD"),
            hint: Some(crate::s4::CAMERA_HINT),
        },
    }
}

/// The render thread body for `display` (S3 horizon).
pub fn consumer<C: Clock>(
    display: Display,
    card: String,
    render_node: String,
) -> impl FnOnce(SnapshotCtx<C>) -> std::io::Result<(crate::s1::runtime::ReaderTotals, String)> + Send + 'static {
    consumer_look(display, card, render_node, runtime::Look::Horizon)
}

/// The render thread body for `display` drawing `look` (S4: the world; the
/// KMS/offscreen targets then carry a depth buffer).
pub fn consumer_look<C: Clock>(
    display: Display,
    card: String,
    render_node: String,
    look: runtime::Look,
) -> impl FnOnce(SnapshotCtx<C>) -> std::io::Result<(crate::s1::runtime::ReaderTotals, String)> + Send + 'static {
    let depth = matches!(look, runtime::Look::World(_) | runtime::Look::Photo(..));
    move |ctx: SnapshotCtx<C>| match display {
        Display::Fake => {
            let clock = ctx.clock.clone();
            runtime::render_loop_look(ctx, move || Ok(runtime::FakePresenter::new(clock)), look)
        }
        #[cfg(target_os = "linux")]
        Display::Kms => runtime::render_loop_look(ctx, move || {
            let open = if depth { gx_render::linux::KmsDisplay::open_world } else { gx_render::linux::KmsDisplay::open };
            open(&card).map_err(|e| e.to_string())
        }, look),
        #[cfg(target_os = "linux")]
        Display::Offscreen => runtime::render_loop_look(ctx, move || {
            let open = if depth { gx_render::linux::Offscreen::open_world } else { gx_render::linux::Offscreen::open };
            open(&render_node)
                .map(|inner| runtime::OffscreenPresenter { inner, presented: 0 })
                .map_err(|e| e.to_string())
        }, look),
        #[cfg(not(target_os = "linux"))]
        Display::Kms | Display::Offscreen => {
            let _ = (&card, &render_node, depth, look);
            Err(std::io::Error::other("the KMS/offscreen display needs Linux"))
        }
    }
}

#[cfg(target_os = "linux")]
fn run_board<B>(a: Args, cfg: S2Config, ap: ApExtras, backend: B, file: std::fs::File, step: Step, look: runtime::Look) -> Result<i32, String>
where
    B: gx_sim::SimulationBackend + Send + 'static,
    B::Error: std::fmt::Debug,
{
    let mut stdout = std::io::stdout();
    let direct_free_flight = matches!(&look, runtime::Look::Photo(..));
    let render = consumer_look::<MonotonicClock>(a.display, a.card.clone(), a.render_node.clone(), look);
    let code = if a.neutral_stick {
        run_with_ap(cfg, extras(step, direct_free_flight), ap, MonotonicClock, NeutralStick::default(), backend, file, &mut stdout, "gx-render", render)
    } else {
        let source = crate::s1::io::EvdevSource::new(a.device.clone().into())
            .map_err(|e| format!("input connector thread: {e}"))?;
        run_with_ap(cfg, extras(step, direct_free_flight), ap, MonotonicClock, source, backend, file, &mut stdout, "gx-render", render)
    }
    .map_err(|e| e.to_string())?;
    if a.prompts {
        println!("log: {}", a.log);
    }
    Ok(code)
}

#[cfg(not(target_os = "linux"))]
fn run_board<B>(a: Args, cfg: S2Config, ap: ApExtras, backend: B, file: std::fs::File, step: Step, look: runtime::Look) -> Result<i32, String>
where
    B: gx_sim::SimulationBackend + Send + 'static,
    B::Error: std::fmt::Debug,
{
    // Host development: synthetic stick, fake display only.
    if a.display != Display::Fake || !a.neutral_stick {
        return Err("on this host only --display fake --stick neutral runs".into());
    }
    let mut stdout = std::io::stdout();
    let direct_free_flight = matches!(&look, runtime::Look::Photo(..));
    let render = consumer_look::<MonotonicClock>(a.display, a.card.clone(), a.render_node.clone(), look);
    run_with_ap(cfg, extras(step, direct_free_flight), ap, MonotonicClock, NeutralStick::default(), backend, file, &mut stdout, "gx-render", render)
        .map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    #[test]
    fn parses_display_modes_and_defaults() {
        let a = parse(&s(&[])).unwrap().unwrap();
        assert_eq!((a.display, a.seconds, a.aircraft), (Display::Kms, DEFAULT_SECONDS, Aircraft::Global5000));
        let a = parse(&s(&["--display", "offscreen", "--stick", "neutral", "--autostart", "--seconds", "20"])).unwrap().unwrap();
        assert_eq!((a.display, a.neutral_stick, a.autostart, a.seconds), (Display::Offscreen, true, true, 20));
        assert!(parse(&s(&["--display", "x11"])).is_err());
        assert!(parse(&s(&["--help"])).unwrap().is_none());
        // SPIKE-002 options are S4-only, and the treatment needs its world.
        assert!(parse(&s(&["--world", "w.gxworld"])).is_err());
        let s4 = |v: &[&str]| parse_step(&s(v), Step::S4);
        let a = s4(&["--world", "w", "--visual", "v", "--display", "offscreen", "--trace", "--capture-dir", "c"]).unwrap().unwrap();
        assert_eq!((a.world.as_deref(), a.visual.as_deref(), a.trace, a.capture_dir.as_deref()), (Some("w"), Some("v"), true, Some("c")));
        assert!(s4(&["--visual", "v"]).is_err());
        assert!(s4(&["--trace"]).is_err(), "no trace on the visible KMS run");
        assert!(s4(&["--display", "offscreen", "--capture-dir", "c"]).is_err());
    }

    #[test]
    fn mouse_defaults_to_auto_detection_and_takes_a_path_or_none() {
        assert_eq!(parse(&s(&[])).unwrap().unwrap().mouse, Some(mouse::MouseSelect::Auto));
        let a = parse(&s(&["--mouse", "/dev/input/by-id/usb-3554_2.4G_Receiver-if01-event-mouse"])).unwrap().unwrap();
        assert_eq!(a.mouse, Some(mouse::MouseSelect::Path("/dev/input/by-id/usb-3554_2.4G_Receiver-if01-event-mouse".into())));
        assert_eq!(parse(&s(&["--mouse", "none"])).unwrap().unwrap().mouse, None);
        assert_eq!(parse_step(&s(&["--mouse", "none"]), Step::S4).unwrap().unwrap().mouse, None, "S4 too");
        assert!(parse(&s(&["--mouse"])).is_err());
        assert!(parse(&s(&["--mouse", ""])).is_err());
    }

    #[test]
    fn photoreal_s4_uses_the_direct_free_flight_script() {
        let e = extras(Step::S4, true);
        assert!(e.free_flight && e.direct_free_flight);
        let ordinary = extras(Step::S4, false);
        assert!(ordinary.free_flight && !ordinary.direct_free_flight);
    }
}

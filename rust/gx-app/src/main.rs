use std::sync::Arc;

use gx_app::{
    button, validate_world_pack, FakeBackend, OfflineApp, WorldFrame, SIM_TICKS_PER_RENDER,
};
use gx_input::{code, ControlMachine, InputConfig, InputEvent};
use gx_world::{synthetic_sfo_like, write_world, Limits};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("s1") => std::process::exit(gx_app::s1::main(&args[1..])),
        Some("s2") => std::process::exit(gx_app::s2::main(&args[1..])),
        Some("s3") => std::process::exit(gx_app::s3::main(&args[1..])),
        Some("s4") => std::process::exit(gx_app::s4::main(&args[1..])),
        Some("rtprobe") => std::process::exit(gx_app::s4::rt_probe_main()),
        Some("spike002-world") => std::process::exit(gx_app::spike002::tool_main(&args[1..])),
        Some("--help") | Some("-h") => {
            println!("gx-app            finite host dry run (prints one ok=true line)");
            println!("gx-app s1 --help  M1 step S1 board binary (headless threads + real stick)");
            println!("gx-app s2 --help  M1 step S2 board binary (S1 threads + JSBSim + record/replay)");
            println!("gx-app s3 --help  M1 step S3 board binary (S2 layout + render thread on DRM/KMS + GLES)");
            println!("gx-app s4 --help  M1 step S4 board binary (S3 + SFO world, Global 5000, chase/cockpit cameras)");
            println!("gx-app spike002-world SRC OUT  host tool: SPIKE-002 world source -> GXWORLD 2.0 pack");
            println!("gx-app rtprobe    sim/I-O realtime setup as a run gets it (exit 0 only with FIFO 50/40)");
            return;
        }
        Some(other) => {
            eprintln!("gx-app: unknown argument {other} (try --help)");
            std::process::exit(2);
        }
        None => {}
    }
    if let Err(error) = dry_run() {
        eprintln!("gx-app dry-run failed: {error}");
        std::process::exit(1);
    }
}

fn dry_run() -> Result<(), String> {
    let mut pack = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut pack)
        .map_err(|error| error.to_string())?;
    let world = validate_world_pack(&pack).map_err(|error| error.to_string())?;

    let mut input = ControlMachine::new(InputConfig::extreme_3d_pro());
    let events = [
        event(1, code::EV_ABS, code::ABS_X, 768),
        event(2, code::EV_ABS, code::ABS_Y, 384),
        event(3, code::EV_ABS, code::ABS_THROTTLE, 64),
        event(
            4,
            code::EV_KEY,
            code::BTN_TRIGGER + u16::from(button::START),
            1,
        ),
        event(5, code::EV_SYN, code::SYN_REPORT, 0),
    ];
    let mut frame = None;
    for event in events {
        frame = input
            .apply(event)
            .map_err(|error| format!("input: {error:?}"))?
            .or(frame);
    }

    let chunks = world.chunks.len();
    let frame_ref = WorldFrame::new(Arc::new(world));
    let mut app = OfflineApp::new(FakeBackend::new(frame_ref.clone()));
    let input_frame = frame.ok_or("no complete input frame")?;
    // Reset places the aircraft at the world origin; then the input frame's
    // Start edge and axes apply.
    let mut reset = input_frame.clone();
    reset.buttons_pressed = 1 << button::RESET;
    app.ingest_controls(&reset, 0)
        .map_err(|error| format!("reset: {error:?}"))?;
    app.ingest_controls(&input_frame, 1)
        .map_err(|error| format!("command: {error:?}"))?;
    let pair = app
        .advance_render_sample([8_333_333, 16_666_666, 25_000_000, 33_333_333])
        .map_err(|error| format!("simulation: {error:?}"))?
        .ok_or("no render snapshot")?;

    let pose = pair.latest.state.local_enu.ok_or("snapshot without local pose")?;
    if pose.frame_id != frame_ref.frame_id()
        || pose.position_m != frame_ref.enu_m(pair.latest.state.geodetic)
    {
        return Err("snapshot pose is not in the world frame".into());
    }
    if pair.latest.input.warning() || pair.latest.pause_reason.is_some() {
        return Err(format!("unexpected input warning or pause: {:?}", pair.latest.input));
    }
    let stats = app.publish_stats();
    if stats.pending || stats.published != stats.offered - stats.superseded {
        return Err(format!("unpublished snapshot: {stats:?}"));
    }

    println!(
        "ok=true world_chunks={} sim_hz=120 render_hz=30 ticks_per_render={} tick={} sequence={} generation={} interpolation={} enu_m=[{:.3},{:.3},{:.3}] published={} busy={} input={:?} input_warning={}",
        chunks,
        SIM_TICKS_PER_RENDER,
        pair.latest.simulation_tick,
        pair.latest.sequence,
        pair.latest.generation,
        pair.previous.is_some(),
        pose.position_m[0],
        pose.position_m[1],
        pose.position_m[2],
        stats.published,
        stats.busy_events,
        pair.latest.input.status,
        pair.latest.input.warning()
    );
    Ok(())
}

fn event(sequence: u64, event_type: u16, code: u16, value: i32) -> InputEvent {
    InputEvent {
        timestamp_ns: sequence * 1_000,
        sequence,
        event_type,
        code,
        value,
    }
}

//! AP-001 W2 on the host: the S4 render thread draws the autopilot panel in
//! every world-look frame (`render_summary.world.panel_frames`), the S3
//! horizon look draws none.

use gx_app::s1::rt::{Clock, MonotonicClock};
use gx_app::s1::runtime::{log_channel, Board};
use gx_app::s1::synthetic_world_frame;
use gx_app::s2::runtime::{PromptSlot, SnapshotCtx, TextTx};
use gx_app::s3::runtime::{render_loop_look, FakePresenter, Look};
use gx_sim::autopilot::{Lateral, Vertical};
use gx_sim::{AircraftSnapshot, AircraftState, ApStatus, InputStatus, LocalEnuPose, RunState, SnapshotExchange};
use std::sync::atomic::Ordering;
use std::sync::mpsc::sync_channel;
use std::sync::Arc;

fn run(look: Look, frame_id: u64, generation: u64) -> Vec<String> {
    let exchange = Arc::new(SnapshotExchange::new());
    let snapshot = AircraftSnapshot {
        sequence: 1,
        generation: 1,
        simulation_tick: 1,
        simulation_time_ns: 1_000_000_000 / 120,
        published_at_ns: MonotonicClock.now_ns(),
        state: AircraftState {
            local_enu: Some(LocalEnuPose {
                frame_id,
                frame_generation: generation,
                position_m: [-1_469.9755, 0.0, 300.0],
                orientation_xyzw: [1.0, 0.0, 0.0, 0.0],
                ..Default::default()
            }),
            heading_rad: std::f64::consts::FRAC_PI_2,
            height_agl_m: 300.0,
            autopilot: ApStatus { engaged: true, lateral: Lateral::Heading, vertical: Vertical::AltHold, hold_ft: 1000.0, ..ApStatus::default() },
            ..Default::default()
        },
        ..Default::default()
    };
    exchange.try_publish(snapshot).unwrap();
    let board = Arc::new(Board::default());
    board.set_vis(RunState::Running, None, InputStatus::Valid);
    let (log, rx_rec) = log_channel();
    std::mem::forget(rx_rec);
    let (tx, rx) = sync_channel(65_536);
    let c = SnapshotCtx {
        clock: MonotonicClock,
        epoch: MonotonicClock.now_ns(),
        exchange,
        board: Arc::clone(&board),
        log,
        text: TextTx::new(tx),
        prompt: Arc::new(PromptSlot::default()),
    };
    let thread = std::thread::spawn(move || {
        let clock = c.clock;
        render_loop_look(c, move || Ok(FakePresenter::new(clock)), look)
    });
    std::thread::sleep(std::time::Duration::from_millis(250));
    board.stop_sim.store(true, Ordering::Relaxed);
    thread.join().unwrap().unwrap();
    rx.try_iter().collect()
}

fn field(line: &str, key: &str) -> u64 {
    let at = line.find(&format!("\"{key}\":")).unwrap_or_else(|| panic!("{key} in {line}")) + key.len() + 3;
    line[at..].split(|c: char| !c.is_ascii_digit()).next().unwrap().parse().unwrap()
}

#[test]
fn every_s4_frame_draws_the_panel_and_s3_draws_none() {
    let frame = synthetic_world_frame().unwrap();
    let (id, generation) = (frame.frame_id(), frame.world().georeference.frame_generation);
    let lines = run(Look::World(frame.world_arc()), id, generation);
    let frames = lines.iter().filter(|l| l.starts_with("{\"k\":\"frame\"")).count() as u64;
    let rs = lines.iter().find(|l| l.starts_with("{\"k\":\"render_summary\"")).unwrap();
    assert!(frames > 0 && field(rs, "panel_frames") == frames, "{frames} frames: {rs}");
    assert_eq!(field(rs, "panel_frames"), field(rs, "frames"), "every presented frame: {rs}");

    let lines = run(Look::Horizon, id, generation);
    let rs = lines.iter().find(|l| l.starts_with("{\"k\":\"render_summary\"")).unwrap();
    assert!(!rs.contains("panel_frames"), "S3 has no panel: {rs}");
}

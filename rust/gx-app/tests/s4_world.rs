//! S4 render thread on the host: the S3 layout drawing the world pack
//! through the fake presenter; the camera button; the on-runway check on a
//! JSBSim-like threshold pose; the HUD-only fallback for a pose from another
//! frame.

use gx_app::s1::rt::{Clock, MonotonicClock};
use gx_app::s1::runtime::{log_channel, Board};
use gx_app::s1::synthetic_world_frame;
use gx_app::s2::runtime::{run_with, Extras, PromptSlot, S2Config, SnapshotCtx, TextTx};
use gx_app::s2::setup::{threshold, Aircraft};
use gx_app::s2::NeutralStick;
use gx_app::s3::runtime::{render_loop_look, FakePresenter, Look};
use gx_app::s3::{consumer_look, Display};
use gx_app::FakeBackend;
use gx_sim::{AircraftSnapshot, AircraftState, InputStatus, LocalEnuPose, RunState, SnapshotExchange};
use std::io::Write;
use std::sync::atomic::Ordering;
use std::sync::mpsc::sync_channel;
use std::sync::{Arc, Mutex};

#[derive(Clone, Default)]
struct Shared(Arc<Mutex<Vec<u8>>>);

impl Write for Shared {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

/// The settled S3 JSBSim pose at the threshold (ENU z 2.7234 m, east, level).
fn threshold_snapshot(sequence: u64, published_at_ns: u64, frame_id: u64, generation: u64, wow: bool) -> AircraftSnapshot {
    AircraftSnapshot {
        sequence,
        generation: 1,
        simulation_tick: sequence,
        simulation_time_ns: sequence * (1_000_000_000 / 120),
        published_at_ns,
        state: AircraftState {
            local_enu: Some(LocalEnuPose {
                frame_id,
                frame_generation: generation,
                position_m: [-1_469.9755, 0.0, 2.7234],
                orientation_xyzw: [1.0, 0.0, 0.0, 0.0],
                ..Default::default()
            }),
            heading_rad: std::f64::consts::FRAC_PI_2,
            height_agl_m: 2.6914,
            weight_on_wheels: wow,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn ctx(exchange: &Arc<SnapshotExchange>, board: &Arc<Board>) -> (SnapshotCtx<MonotonicClock>, std::sync::mpsc::Receiver<String>) {
    let (log, rx_rec) = log_channel();
    std::mem::forget(rx_rec);
    let (tx, rx) = sync_channel(65_536);
    let c = SnapshotCtx {
        clock: MonotonicClock,
        epoch: MonotonicClock.now_ns(),
        exchange: Arc::clone(exchange),
        board: Arc::clone(board),
        log,
        text: TextTx::new(tx),
        prompt: Arc::new(PromptSlot::default()),
    };
    (c, rx)
}

#[test]
fn s4_layout_draws_the_world_every_frame_through_the_fake_presenter() {
    let frame = synthetic_world_frame().unwrap();
    let cfg = S2Config {
        seconds: 3,
        prompts: false,
        device: "synthetic:neutral".into(),
        commit: "test",
        aircraft: Aircraft::Global5000,
        backend: "fake",
        threshold: threshold(&frame).unwrap(),
        autostart: true,
    };
    let sink = Shared::default();
    let extras = Extras { schema: gx_app::s4::SCHEMA, free_flight: true, direct_free_flight: false, stop: None, title: Some("S4 SFO WORLD"), hint: Some(gx_app::s4::CAMERA_HINT) };
    let render = consumer_look::<MonotonicClock>(Display::Fake, String::new(), String::new(), Look::World(frame.world_arc()));
    let mut stdout = Vec::new();
    let code = run_with(cfg, extras, MonotonicClock, NeutralStick::default(), FakeBackend::new(frame), sink.clone(), &mut stdout, "gx-render", render).unwrap();
    assert_eq!(code, 0);
    let log = String::from_utf8(sink.0.lock().unwrap().clone()).unwrap();
    let lines: Vec<&str> = log.lines().collect();
    assert!(lines[0].contains("\"schema\":\"gx-s4/1\""), "{}", lines[0]);
    let of = |k: &str| lines.iter().filter(move |l| l.starts_with(&format!("{{\"k\":\"{k}\""))).copied().collect::<Vec<_>>();
    let world = of("world");
    assert_eq!(world.len(), 1);
    assert!(world[0].contains("\"camera_button\":\"button 5 / BTN_TOP2\"") && world[0].contains("\"lights\":"), "{}", world[0]);
    let frames = of("frame");
    assert!(frames.len() >= 60, "{} frames", frames.len());
    assert!(frames.iter().skip(2).all(|f| f.contains("\"cam\":\"chase\"") && f.contains("\"draws\":4")), "{:?}", &frames[..3]);
    assert!(frames.iter().skip(2).all(|f| f.contains("\"vs_fpm\":") && f.contains("\"n1_pct\":")));
    let rs = of("render_summary");
    assert!(rs[0].contains("\"world\":{\"frames\":") && rs[0].contains("\"failure\":null"), "{}", rs[0]);
    assert!(of("ground_check").iter().any(|g| g.contains("\"when\":\"reset\"")));
    assert!(of("phase").iter().any(|p| p.contains("S4 SFO WORLD") && p.contains("Button 5")));
}

#[test]
fn camera_button_switches_view_and_threshold_pose_is_on_the_runway() {
    let frame = synthetic_world_frame().unwrap();
    let (id, generation) = (frame.frame_id(), frame.world().georeference.frame_generation);
    let exchange = Arc::new(SnapshotExchange::new());
    exchange.try_publish(threshold_snapshot(1, MonotonicClock.now_ns(), id, generation, true)).unwrap();
    let board = Arc::new(Board::default());
    board.set_vis(RunState::Paused, Some(gx_sim::PauseReason::Reset), InputStatus::Valid);
    let (c, rx) = ctx(&exchange, &board);
    let look = Look::World(frame.world_arc());
    let thread = std::thread::spawn(move || {
        let clock = c.clock;
        render_loop_look(c, move || Ok(FakePresenter::new(clock)), look)
    });
    std::thread::sleep(std::time::Duration::from_millis(300));
    board.camera_presses.store(1, Ordering::Relaxed);
    std::thread::sleep(std::time::Duration::from_millis(300));
    board.camera_presses.store(2, Ordering::Relaxed);
    std::thread::sleep(std::time::Duration::from_millis(300));
    board.stop_sim.store(true, Ordering::Relaxed);
    thread.join().unwrap().unwrap();
    let lines: Vec<String> = rx.try_iter().collect();
    let cams: Vec<&String> = lines.iter().filter(|l| l.starts_with("{\"k\":\"camera\"")).collect();
    assert_eq!(cams.len(), 2, "{cams:?}");
    assert!(cams[0].contains("\"camera\":\"cockpit\"") && cams[1].contains("\"camera\":\"chase\""));
    let frames: Vec<&String> = lines.iter().filter(|l| l.starts_with("{\"k\":\"frame\"")).collect();
    assert!(frames.iter().any(|f| f.contains("\"cam\":\"cockpit\"") && f.contains("\"draws\":3")), "cockpit draws no aircraft");
    let g: Vec<&String> = lines.iter().filter(|l| l.starts_with("{\"k\":\"ground_check\"")).collect();
    assert_eq!(g.len(), 1, "one check per generation while not running: {g:?}");
    assert!(g[0].contains("\"on_runway\":true") && g[0].contains("\"when\":\"reset\""), "{}", g[0]);
}

#[test]
fn a_pose_from_another_frame_is_never_drawn_in_the_pack() {
    let frame = synthetic_world_frame().unwrap();
    let exchange = Arc::new(SnapshotExchange::new());
    exchange.try_publish(threshold_snapshot(1, MonotonicClock.now_ns(), frame.frame_id() + 1, 1, true)).unwrap();
    let board = Arc::new(Board::default());
    board.set_vis(RunState::Running, None, InputStatus::Valid);
    let (c, rx) = ctx(&exchange, &board);
    let look = Look::World(frame.world_arc());
    let thread = std::thread::spawn(move || {
        let clock = c.clock;
        render_loop_look(c, move || Ok(FakePresenter::new(clock)), look)
    });
    std::thread::sleep(std::time::Duration::from_millis(250));
    board.stop_sim.store(true, Ordering::Relaxed);
    thread.join().unwrap().unwrap();
    let lines: Vec<String> = rx.try_iter().collect();
    let frames: Vec<&String> = lines.iter().filter(|l| l.starts_with("{\"k\":\"frame\"")).collect();
    assert!(!frames.is_empty() && frames.iter().all(|f| f.contains("\"cam\":\"none\"")), "{frames:?}");
    let rs = lines.iter().find(|l| l.starts_with("{\"k\":\"render_summary\"")).unwrap();
    assert!(rs.contains("\"world\":{\"frames\":0,") && !rs.contains("\"frame_mismatch\":0"), "{rs}");
    assert!(!lines.iter().any(|l| l.starts_with("{\"k\":\"ground_check\"")));
}

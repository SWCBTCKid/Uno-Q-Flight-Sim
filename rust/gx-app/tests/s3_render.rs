//! S3 render thread on the host: the real S2 thread layout with the fake
//! presenter, the render-side log of what was drawn (INPUT-002 banner and
//! the paused overlay), and the ARCH-004 guarantee that a stalled render
//! thread never holds snapshot slots or makes the simulation's publish busy.

use gx_app::s1::rt::{Clock, MonotonicClock};
use gx_app::s1::runtime::{log_channel, Board};
use gx_app::s1::synthetic_world_frame;
use gx_app::s2::runtime::{run_with, Extras, PromptSlot, S2Config, SnapshotCtx, TextTx};
use gx_app::s2::setup::{threshold, Aircraft};
use gx_app::s2::NeutralStick;
use gx_app::s3::runtime::{render_loop, Counts, FakePresenter, Presented, Presenter};
use gx_app::s3::{consumer, Display, SCHEMA};
use gx_app::FakeBackend;
use gx_render::scene::Vertex;
use gx_sim::{AircraftSnapshot, AircraftState, InputStatus, LocalEnuPose, PauseReason, RunState, SnapshotExchange, UnavailableReason};
use std::io::Write;
use std::sync::atomic::Ordering;
use std::sync::mpsc::sync_channel;
use std::sync::{Arc, Barrier, Mutex};

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

fn snapshot(sequence: u64, published_at_ns: u64) -> AircraftSnapshot {
    AircraftSnapshot {
        sequence,
        generation: 1,
        simulation_tick: sequence,
        simulation_time_ns: sequence * (1_000_000_000 / 120),
        published_at_ns,
        state: AircraftState {
            local_enu: Some(LocalEnuPose {
                frame_id: 7,
                frame_generation: 1,
                position_m: [sequence as f64, 0.0, 2.5],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                ..Default::default()
            }),
            ..Default::default()
        },
        ..Default::default()
    }
}

fn ctx(exchange: &Arc<SnapshotExchange>, board: &Arc<Board>) -> (SnapshotCtx<MonotonicClock>, std::sync::mpsc::Receiver<String>) {
    let (log, _rx) = log_channel();
    std::mem::forget(_rx); // keep the Rec channel open for the whole test
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
fn s3_layout_renders_every_frame_through_the_fake_presenter() {
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
    let extras = Extras { schema: SCHEMA, free_flight: true, stop: None, ..Extras::default() };
    let render = consumer::<MonotonicClock>(Display::Fake, String::new(), String::new());
    let mut stdout = Vec::new();
    let code = run_with(cfg, extras, MonotonicClock, NeutralStick::default(), FakeBackend::new(frame), sink.clone(), &mut stdout, "gx-render", render).unwrap();
    assert_eq!(code, 0);
    let log = String::from_utf8(sink.0.lock().unwrap().clone()).unwrap();
    let lines: Vec<&str> = log.lines().collect();
    assert!(lines[0].contains("\"schema\":\"gx-s3/1\""), "{}", lines[0]);
    let of = |k: &str| lines.iter().filter(move |l| l.starts_with(&format!("{{\"k\":\"{k}\""))).copied().collect::<Vec<_>>();
    assert_eq!(of("display").len(), 1);
    assert!(of("display")[0].contains("\"target\":\"fake\"") && of("display")[0].contains("\"policy\":\"other_nice0\",\"cpu\":3"));
    let frames = of("frame");
    assert!(frames.len() >= 60, "about 30 fps for 3 s: {} frames", frames.len());
    let summary = of("render_summary");
    assert_eq!(summary.len(), 1);
    assert!(summary[0].contains(&format!("\"frames\":{}", frames.len())), "{}", summary[0]);
    assert!(summary[0].contains("\"failure\":null") && summary[0].contains("\"text_drops\":0"), "{}", summary[0]);
    // The sim ran (autostart) and every frame drew the running state.
    assert!(frames.iter().any(|f| f.contains("\"run\":\"running\"")));
    // Start-up reset: paused until the (auto)start, then nothing over the horizon.
    let drawn = of("drawn");
    assert!(drawn[0].contains("\"overlay_text\":\"PAUSED (RESET) - PULL TRIGGER\""), "{drawn:#?}");
    assert!(drawn.last().unwrap().contains("\"banner\":\"none\",\"banner_text\":null,\"overlay\":\"none\""), "{drawn:#?}");
    let s = of("summary");
    assert!(s[0].contains("\"drops\":{\"sim\":0,\"reader\":0,\"io\":0}"), "{}", s[0]);
    assert!(of("phase").iter().any(|p| p.contains("S3 FIRST PIXELS")));
}

#[test]
fn render_log_records_the_input002_banner_shown_and_cleared() {
    let exchange = Arc::new(SnapshotExchange::new());
    exchange.try_publish(snapshot(1, MonotonicClock.now_ns())).unwrap();
    let board = Arc::new(Board::default());
    board.set_vis(RunState::Running, None, InputStatus::Valid);
    let (c, rx) = ctx(&exchange, &board);
    let thread = std::thread::spawn(move || {
        let clock = c.clock;
        render_loop(c, move || Ok(FakePresenter::new(clock)))
    });
    let lost = InputStatus::Unavailable(UnavailableReason::Disconnected);
    let steps = [
        (RunState::Running, None, InputStatus::Valid, 0),
        (RunState::Running, None, lost, 3_200),
        (RunState::Paused, Some(PauseReason::InputLoss), lost, 10_000),
        (RunState::Paused, Some(PauseReason::InputLoss), InputStatus::Valid, 0),
        (RunState::Running, None, InputStatus::Valid, 0),
    ];
    for (run, pause, status, loss) in steps {
        board.loss_ms.store(loss, Ordering::Relaxed);
        board.set_vis(run, pause, status);
        std::thread::sleep(std::time::Duration::from_millis(250));
    }
    board.stop_sim.store(true, Ordering::Relaxed);
    thread.join().unwrap().unwrap();
    let lines: Vec<String> = rx.try_iter().collect();
    let drawn: Vec<(String, String)> = lines
        .iter()
        .filter(|l| l.starts_with("{\"k\":\"drawn\""))
        .map(|l| {
            let field = |k: &str| l.split(&format!("\"{k}\":\"")).nth(1).unwrap().split('"').next().unwrap().to_owned();
            (field("banner"), field("overlay"))
        })
        .collect();
    let want = [("none", "none"), ("stick_lost", "none"), ("paused_stick_lost", "none"), ("none", "paused_input_loss"), ("none", "none")];
    assert_eq!(drawn, want.map(|(a, b)| (a.to_owned(), b.to_owned())), "{lines:#?}");
    assert!(lines.iter().any(|l| l.contains("\"banner_text\":\"STICK LOST - 3 S\"")));
    assert!(lines.iter().any(|l| l.contains("\"banner_text\":\"PAUSED - PRESS TRIGGER WHEN THE STICK IS BACK\"")));
    assert!(lines.iter().any(|l| l.contains("\"overlay_text\":\"PAUSED (STICK LOST) - PULL TRIGGER\"")));
}

/// Blocks inside its first present until released.
struct Blocking {
    entered: Arc<Barrier>,
    release: Arc<Barrier>,
    first: bool,
}

impl Presenter for Blocking {
    fn present(&mut self, _verts: &[Vertex]) -> Result<Presented, String> {
        if std::mem::take(&mut self.first) {
            self.entered.wait();
            self.release.wait();
        }
        Ok(Presented { status: "ok", gpu_status: "unavailable", ..Presented::default() })
    }
    fn next_wake_ns(&self) -> Option<u64> {
        None
    }
    fn info_json(&self) -> String {
        "\"target\":\"blocking\"".into()
    }
    fn counts(&self) -> Counts {
        Counts::default()
    }
    fn shutdown(self) -> Result<String, String> {
        Ok(String::new())
    }
}

#[test]
fn a_stalled_render_thread_never_makes_the_sim_publish_busy() {
    let exchange = Arc::new(SnapshotExchange::new());
    exchange.try_publish(snapshot(1, MonotonicClock.now_ns())).unwrap();
    let board = Arc::new(Board::default());
    let (entered, release) = (Arc::new(Barrier::new(2)), Arc::new(Barrier::new(2)));
    let (c, _rx) = ctx(&exchange, &board);
    let (e, r) = (Arc::clone(&entered), Arc::clone(&release));
    let thread = std::thread::spawn(move || render_loop(c, move || Ok(Blocking { entered: e, release: r, first: true })));
    entered.wait();
    // The render thread is stuck inside present(): it must hold no pins.
    for seq in 2..500 {
        exchange.try_publish(snapshot(seq, MonotonicClock.now_ns())).expect("publish never busy while the renderer stalls");
    }
    board.stop_sim.store(true, Ordering::Relaxed);
    release.wait();
    thread.join().unwrap().unwrap();
}

//! Real threads, real clock, scripted stick: the S1 runtime starts, ticks,
//! logs through the drain and shuts down cleanly. Timing is not asserted
//! (host schedulers vary); the board run measures it.

use gx_app::s1::io::{InputSource, IoBridge, IoLog};
use gx_app::s1::rt::MonotonicClock;
use gx_app::s1::runtime::{run, S1Config};
use gx_app::s1::synthetic_world_frame;
use gx_input::ControlFrame;
use std::io::Write;
use std::sync::{Arc, Mutex};

struct Scripted {
    start_ns: Option<u64>,
    next_ns: u64,
    seq: u64,
}

impl InputSource for Scripted {
    fn poll(
        &mut self,
        now_ns: u64,
        fresh_now: &mut dyn FnMut() -> u64,
        bridge: &mut IoBridge,
        log: &mut IoLog,
    ) {
        let start = *self.start_ns.get_or_insert_with(|| {
            bridge.connected();
            log.io(now_ns, "\"ev\":\"connected\",\"path\":\"scripted\"");
            now_ns
        });
        if now_ns < self.next_ns {
            return;
        }
        self.next_ns = now_ns + 10_000_000;
        let pressed = if self.seq == 30 {
            1 // Start
        } else if self.seq == 60 {
            1 << 1 // Pause
        } else {
            0
        };
        let frame = ControlFrame {
            timestamp_ns: now_ns,
            sequence: self.seq,
            roll: ((now_ns - start) as f64 * 1e-9).sin() as f32,
            buttons: pressed,
            buttons_pressed: pressed,
            ..ControlFrame::default()
        };
        self.seq += 1;
        bridge.frame(&frame, fresh_now());
    }

    fn needs_resync(&self) -> bool {
        false
    }
}

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

#[test]
fn s1_threads_run_log_and_stop() {
    let sink = Shared::default();
    let cfg = S1Config { seconds: 2, prompts: false, device: "scripted".into(), commit: "test" };
    let source = Scripted { start_ns: None, next_ns: 0, seq: 0 };
    let mut stdout = Vec::new();
    let code = run(cfg, MonotonicClock, source, synthetic_world_frame().unwrap(), sink.clone(), &mut stdout)
        .expect("run");
    assert_eq!(code, 0);
    let log = String::from_utf8(sink.0.lock().unwrap().clone()).unwrap();
    let lines: Vec<&str> = log.lines().collect();
    assert!(lines[0].starts_with("{\"k\":\"start\""), "{}", lines[0]);
    assert!(lines.last().unwrap().starts_with("{\"k\":\"end\""));
    let count = |k: &str| lines.iter().filter(|l| l.starts_with(&format!("{{\"k\":\"{k}\""))).count();
    assert!(count("sim_sec") >= 1);
    assert_eq!(count("io_final"), 1, "final partial I/O window is evidence");
    assert!(count("read") >= 30, "reader at 30 Hz for 2 s");
    assert!(count("lat") >= 50, "a latency sample per new frame");
    assert_eq!(count("start_accepted"), 1, "the scripted trigger started the sim");
    assert!(
        lines.iter().any(|line| line.starts_with("{\"k\":\"read\"") && line.contains("\"paused\":true")),
        "reader classifies the live paused state rather than the prior running snapshot"
    );
    assert!(
        lines.iter().any(|line| line.starts_with("{\"k\":\"lat\"")
            && line.contains("\"published\":false,\"running\":false")),
        "paused input frames without a snapshot offer are not called published"
    );
    assert_eq!(count("summary"), 1);
    let summary = lines.iter().find(|l| l.starts_with("{\"k\":\"summary\"")).unwrap();
    assert!(summary.contains("\"drops\":{\"sim\":0,\"reader\":0,\"io\":0}"), "{summary}");
    // Ticks: about 240 in 2 s; allow for a slow host.
    let ticks: u64 = summary.split("\"ticks\":").nth(1).unwrap().split(',').next().unwrap().parse().unwrap();
    assert!((150..=245).contains(&ticks), "{ticks}");
}

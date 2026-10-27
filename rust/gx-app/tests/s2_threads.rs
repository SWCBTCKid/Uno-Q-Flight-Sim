//! S2 runtime with real threads and clock, a scripted stick and the fake
//! backend: the log carries one tick record per tick and replaying it
//! through a fresh mailbox and SimDriver reproduces every state hash.
//! (JSBSim itself is exercised on the board: tests/s2_jsbsim.rs.)

use gx_app::s1::io::{InputSource, IoBridge, IoLog};
use gx_app::s1::rt::MonotonicClock;
use gx_app::s1::synthetic_world_frame;
use gx_app::s2::replay::{initial_levers, read_log, replay};
use gx_app::s2::runtime::{run, S2Config};
use gx_app::s2::setup::{threshold, Aircraft};
use gx_app::FakeBackend;
use gx_input::ControlFrame;
use std::io::Write;
use std::sync::{Arc, Mutex};

struct Scripted {
    start_ns: Option<u64>,
    next_ns: u64,
    seq: u64,
}

impl InputSource for Scripted {
    fn poll(&mut self, now_ns: u64, fresh_now: &mut dyn FnMut() -> u64, bridge: &mut IoBridge, log: &mut IoLog) {
        let start = *self.start_ns.get_or_insert_with(|| {
            bridge.connected();
            log.io(now_ns, "\"ev\":\"connected\",\"path\":\"scripted\"");
            now_ns
        });
        if now_ns < self.next_ns {
            return;
        }
        self.next_ns = now_ns + 7_000_000;
        let s = (now_ns - start) as f64 * 1e-9;
        // An unmapped button-4 press after ~1 s; a Reset at ~2 s.
        let pressed = match self.seq {
            150 => 1 << 3,
            300 => 1 << 2,
            320 => 1,
            _ => 0,
        };
        let frame = ControlFrame {
            timestamp_ns: now_ns,
            sequence: self.seq,
            roll: (s * 1.3).sin() as f32,
            pitch: (s * 0.7).cos() as f32 * 0.5,
            yaw: (s * 2.1).sin() as f32 * 0.3,
            throttle: (0.5 + 0.5 * (s * 0.4).sin()) as f32,
            hat_x: if self.seq % 200 > 180 { 1.0 } else { 0.0 },
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
fn s2_threads_log_ticks_and_the_replay_reproduces_every_hash() {
    let frame = synthetic_world_frame().unwrap();
    let cfg = S2Config {
        seconds: 3,
        prompts: false,
        device: "scripted".into(),
        commit: "test",
        aircraft: Aircraft::Global5000,
        backend: "fake",
        threshold: threshold(&frame).unwrap(),
        autostart: true,
    };
    let sink = Shared::default();
    let source = Scripted { start_ns: None, next_ns: 0, seq: 0 };
    let mut stdout = Vec::new();
    let code = run(cfg, MonotonicClock, source, FakeBackend::new(frame.clone()), sink.clone(), &mut stdout).unwrap();
    assert_eq!(code, 0);
    let log = String::from_utf8(sink.0.lock().unwrap().clone()).unwrap();
    let lines: Vec<&str> = log.lines().collect();
    assert!(lines[0].contains("\"schema\":\"gx-s2/1\""), "{}", lines[0]);
    let count = |k: &str| lines.iter().filter(|l| l.starts_with(&format!("{{\"k\":\"{k}\""))).count();
    assert_eq!(count("autostart"), 1);
    assert!(count("state") >= 15, "10 Hz state records");
    if cfg!(target_os = "linux") {
        assert!(count("mem") >= 2, "RSS sampled every second");
    }
    let summary = lines.iter().find(|l| l.starts_with("{\"k\":\"summary\"")).unwrap();
    assert!(summary.contains("\"drops\":{\"sim\":0,\"reader\":0,\"io\":0}"), "{summary}");
    assert!(summary.contains("\"log\":{\"policy\":\"other_nice0\""), "{summary}");

    let (start, ticks) = read_log(log.as_bytes()).unwrap();
    assert_eq!((start.backend.as_str(), start.aircraft.as_str()), ("fake", "global5000"));
    assert!(ticks.len() >= 200, "{} tick records", ticks.len());
    assert_eq!(count("tick"), ticks.len());
    let hashed = ticks.iter().filter(|t| t.hash.is_some()).count();
    assert!(hashed >= 100, "the sim ran ({hashed} hashed ticks)");
    assert!(lines.iter().any(|l| l.starts_with("{\"k\":\"cmd\"") && l.contains("\"reset\",\"source\":\"stick\"")));
    // The original profile leaves button 4 and the hat unmapped.
    let states: Vec<&&str> = lines.iter().filter(|l| l.starts_with("{\"k\":\"state\"")).collect();
    assert!(states.iter().any(|l| l.contains("\"run\":\"running\"")));

    let result = replay(FakeBackend::new(frame.clone()), initial_levers(&start), &ticks).unwrap();
    assert_eq!(result.mismatches, 0, "{result:?}");
    assert_eq!(result.gaps, 0);
    assert_eq!(result.ticks as usize, ticks.len());

    // A perturbed record must be caught.
    let mut bad = ticks.clone();
    let k = bad.iter().position(|t| t.hash.is_some() && t.record.is_some_and(|r| r.controls.throttle > 0.2)).unwrap();
    let mut r = bad[k].record.unwrap();
    r.controls.throttle = f32::from_bits(r.controls.throttle.to_bits() + 1);
    bad[k].record = Some(r);
    let result = replay(FakeBackend::new(frame), initial_levers(&start), &bad).unwrap();
    assert!(result.mismatches > 0, "a one-ulp throttle change is detected: {result:?}");
}

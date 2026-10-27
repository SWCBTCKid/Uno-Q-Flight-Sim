//! Two-thread seam test: a simulation thread that exclusively owns
//! `SimCore` (inside `SimLoop`) publishes at 120 Hz tick deadlines from a
//! fake clock, while a reader thread polls `try_latest_pair` as a renderer
//! would.
//!
//! The first test scripts controls on the simulation thread itself. The
//! second adds an I/O thread: gx-input `ControlMachine` reports (including
//! `SYN_DROPPED` loss, a held `Disarmed` state and snapshot resync) are
//! published through the gx-sim control mailbox, read by the simulation
//! thread at each tick boundary, and the resulting snapshots are read by a
//! render thread. It checks untorn controls end to end, that no command edge
//! is lost, fail-closed neutral controls, and the 100 ms stale bound. It
//! also holds one disconnect for more than 10 s of sim time (INPUT-002): the
//! sim must pause for input loss at exactly 1200 ticks, stay paused when
//! input returns (warning cleared), and resume only on an explicit Start.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Barrier};
use std::thread;

use gx_app::{button, validate_world_pack, FakeBackend, InputPublisher, SimLoop, WorldFrame};
use gx_input::{
    code as gx_code, ControlFrame, ControlMachine, DeviceSnapshot, InputConfig, InputEvent,
    MachineOutput,
};
use gx_sim::{
    control_mailbox, AircraftSnapshot, Command, ControlInput, ControlStatus, FixedRate,
    InputStatus, InputValidity, PauseReason, RunState, SnapshotExchange, DEFAULT_STALE_AFTER_NS,
    INPUT_LOSS_PAUSE_AFTER_NS, SIM_HZ,
};
use gx_world::{synthetic_sfo_like, write_world, Limits};

const TICKS: u64 = 120_000;
const RESET_EVERY: u64 = 3_000;
const EPOCH_NS: u64 = 5_000_000_000;

fn script(tick: u64) -> Option<ControlFrame> {
    let pressed = match tick % RESET_EVERY {
        0 => 1 << button::RESET,
        1 => 1 << button::START,
        _ if tick % 97 == 0 => 0,
        _ => return None,
    };
    Some(ControlFrame {
        yaw: ((tick % 7) as f32 - 3.0) / 3.0,
        throttle: 0.6,
        buttons_pressed: pressed,
        ..ControlFrame::default()
    })
}

fn check_snapshot(frame: &WorldFrame, s: &AircraftSnapshot) {
    let pose = s.state.local_enu.expect("published snapshot without pose");
    assert_eq!(pose.frame_id, frame.frame_id());
    assert_eq!(
        pose.position_m,
        frame.enu_m(s.state.geodetic),
        "torn snapshot: pose does not match its own geodetic position (seq {})",
        s.sequence
    );
    assert_eq!(
        s.simulation_time_ns,
        FixedRate::elapsed_ns(SIM_HZ, s.simulation_tick),
        "torn snapshot: tick/time mismatch (seq {})",
        s.sequence
    );
    assert_eq!(s.state.engine_n1, if s.simulation_tick == 0 { 0.0 } else { 0.6f32.into() });
}

#[test]
fn sim_thread_publishes_and_render_thread_never_sees_torn_or_mixed_snapshots() {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    let frame = WorldFrame::new(Arc::new(validate_world_pack(&bytes).unwrap()));

    let exchange = Arc::new(SnapshotExchange::new());
    let done = Arc::new(AtomicBool::new(false));
    let start = Arc::new(Barrier::new(2));

    let sim = {
        let exchange = Arc::clone(&exchange);
        let done = Arc::clone(&done);
        let start = Arc::clone(&start);
        let backend = FakeBackend::new(frame.clone());
        thread::spawn(move || {
            // Constructed here: SimCore and the backend never leave this thread.
            let mut sim = SimLoop::new(backend, exchange);
            let mut rate = FixedRate::new(EPOCH_NS, SIM_HZ);
            let mut last_sequence = 0;
            start.wait();
            for _ in 0..TICKS {
                let deadline = rate.next_deadline();
                let now = deadline.at_ns; // fake clock: wake exactly on time
                if let Some(input) = script(deadline.tick) {
                    if let Some(s) = sim.ingest_controls(&input, now).unwrap() {
                        last_sequence = s.sequence;
                    }
                }
                if let Some(s) = sim.tick(now).unwrap() {
                    last_sequence = s.sequence;
                }
                rate.complete_tick(now);
                thread::yield_now();
            }
            // Drain the one pending snapshot, if any; the reader keeps polling.
            let mut attempts = 0;
            while !sim.flush() {
                attempts += 1;
                assert!(attempts < 1_000_000, "pending snapshot never published");
                thread::yield_now();
            }
            done.store(true, Ordering::SeqCst);
            (sim.publish_stats(), rate.counters(), last_sequence)
        })
    };

    let reader = {
        let exchange = Arc::clone(&exchange);
        let done = Arc::clone(&done);
        let start = Arc::clone(&start);
        let frame = frame.clone();
        thread::spawn(move || {
            start.wait();
            let mut last: Option<AircraftSnapshot> = None;
            let mut observations = 0u64;
            let mut interpolatable = 0u64;
            let mut reset_observed_without_previous = 0u64;
            let mut generations = BTreeSet::new();
            loop {
                let finished = done.load(Ordering::SeqCst);
                if let Some(pair) = exchange.try_latest_pair() {
                    let latest = pair.latest;
                    check_snapshot(&frame, &latest);
                    if let Some(prev) = last {
                        assert!(latest.sequence >= prev.sequence, "sequence went backwards");
                        assert!(latest.generation >= prev.generation, "generation went backwards");
                    }
                    match pair.previous {
                        Some(previous) => {
                            check_snapshot(&frame, &previous);
                            assert_eq!(previous.generation, latest.generation, "mixed generations");
                            assert!(previous.sequence < latest.sequence);
                            assert!(previous.simulation_tick < latest.simulation_tick);
                            interpolatable += 1;
                        }
                        None if latest.simulation_tick == 0 => reset_observed_without_previous += 1,
                        None => {}
                    }
                    assert!(
                        latest.simulation_tick != 0 || pair.previous.is_none(),
                        "interpolation across reset"
                    );
                    generations.insert(latest.generation);
                    observations += 1;
                    last = Some(latest);
                }
                if finished {
                    break;
                }
            }
            (last.unwrap(), observations, interpolatable, reset_observed_without_previous, generations)
        })
    };

    let (stats, timing, last_sequence) = sim.join().unwrap();
    let (final_seen, observations, interpolatable, resets_seen, generations) =
        reader.join().unwrap();

    assert_eq!(timing.ticks, TICKS);
    assert_eq!(timing.missed_deadlines, 0);
    assert!(!stats.pending);
    assert_eq!(stats.offered, stats.published + stats.superseded);
    assert_eq!(final_seen.sequence, last_sequence, "reader must see the final publication");
    assert_eq!(final_seen.generation, TICKS / RESET_EVERY);
    assert!(observations > 100, "too little overlap: {observations}");
    assert!(interpolatable > 0);
    assert!(generations.len() > 1, "reader never observed a reset");
    eprintln!(
        "seam: offered={} published={} superseded={} busy={} max_consecutive_busy={} \
         reader_observations={observations} interpolatable={interpolatable} \
         resets_seen={resets_seen} generations_seen={}",
        stats.offered,
        stats.published,
        stats.superseded,
        stats.busy_events,
        stats.max_consecutive_busy,
        generations.len()
    );
}

// ---------------------------------------------------------------------------
// Three threads: I/O -> control mailbox -> sim -> SnapshotExchange -> render.
// ---------------------------------------------------------------------------

const IO_REPORTS: u64 = 60_000;
/// The I/O thread may run at most this many reports ahead of sim ticks, so
/// several reports (and edges) usually land between two ticks.
const IO_REPORTS_PER_TICK: u64 = 3;
const LOSS_EVERY: u64 = 4_999;
const DISARMED_TICKS: u64 = 6;
const TAIL_TICKS: u64 = 30;
const TICK_NS: u64 = 1_000_000_000 / SIM_HZ as u64;
/// INPUT-002: one disconnect longer than 10 s, then valid input without a
/// Start for a while, then an explicit Start.
const LONG_LOSS_AT: u64 = 30_000;
const LONG_LOSS_TICKS: u64 = 1_300;
const RECOVERED_WITHOUT_START_TICKS: u64 = 60;
const LOSS_PAUSE_TICKS: u64 = INPUT_LOSS_PAUSE_AFTER_NS / 1_000_000_000 * SIM_HZ as u64; // 1200

/// Every report sets all four flight axes from one raw value, so a control
/// tuple that mixes two reports is detectable.
fn raw_for(report: u64) -> i32 {
    (report % 250) as i32 + 3
}

type Tuple = [u32; 4];

fn tuple_of(c: &ControlInput) -> Tuple {
    [c.roll.to_bits(), c.pitch.to_bits(), c.yaw.to_bits(), c.throttle.to_bits()]
}

/// Expected normalized tuple for every raw value the I/O thread uses.
fn expected_tuples() -> BTreeSet<Tuple> {
    let config = InputConfig::extreme_3d_pro();
    let cal = |code: u16| config.axes.iter().find(|a| a.evdev_code == code).unwrap().calibration;
    (0..250)
        .map(|r| r + 3)
        .map(|raw| {
            tuple_of(&ControlInput {
                roll: cal(gx_code::ABS_X).bipolar(raw),
                pitch: cal(gx_code::ABS_Y).bipolar(raw),
                yaw: cal(gx_code::ABS_RZ).bipolar(raw),
                throttle: cal(gx_code::ABS_THROTTLE).unipolar(raw),
                ..ControlInput::default()
            })
        })
        .collect()
}

fn snapshot_tuple(s: &AircraftSnapshot) -> Tuple {
    tuple_of(&ControlInput {
        roll: s.state.aileron_position as f32,
        pitch: s.state.elevator_position as f32,
        yaw: s.state.rudder_position as f32,
        throttle: s.state.engine_n1 as f32,
        ..ControlInput::default()
    })
}

struct Events {
    sequence: u64,
    time_ns: u64,
}

impl Events {
    fn next(&mut self, event_type: u16, code: u16, value: i32) -> InputEvent {
        self.sequence += 1;
        self.time_ns += 1_000;
        InputEvent { timestamp_ns: self.time_ns, sequence: self.sequence, event_type, code, value }
    }
}

/// Sets the flag when dropped, including on panic, so the other threads
/// stop waiting and the panic surfaces at `join` instead of hanging.
struct SetOnDrop(Arc<AtomicBool>);

impl Drop for SetOnDrop {
    fn drop(&mut self) {
        self.0.store(true, Ordering::SeqCst);
    }
}

/// One complete valid report: all four axes from `raw`, then `keys`.
fn send_report(
    machine: &mut ControlMachine,
    publisher: &mut InputPublisher,
    ev: &mut Events,
    totals: &mut IoTotals,
    raw: i32,
    keys: &[(u16, i32)],
) {
    let mut events = vec![
        ev.next(gx_code::EV_ABS, gx_code::ABS_X, raw),
        ev.next(gx_code::EV_ABS, gx_code::ABS_Y, raw),
        ev.next(gx_code::EV_ABS, gx_code::ABS_RZ, raw),
        ev.next(gx_code::EV_ABS, gx_code::ABS_THROTTLE, raw),
    ];
    for &(key, value) in keys {
        events.push(ev.next(gx_code::EV_KEY, key, value));
    }
    events.push(ev.next(gx_code::EV_SYN, gx_code::SYN_REPORT, 0));
    for event in events {
        let out = machine.apply_detailed(event).unwrap();
        if let MachineOutput::Frame(f) = &out {
            totals.frames += 1;
            for (k, b) in [button::START, button::PAUSE, button::RESET].iter().enumerate() {
                if f.buttons_pressed & (1 << b) != 0 {
                    totals.edges[k] += 1;
                }
            }
        }
        publisher.publish_output(&out, ev.time_ns);
    }
}

#[derive(Debug, Default)]
struct IoTotals {
    edges: [u64; 3],
    frames: u64,
    losses: u64,
    resyncs_required: u64,
}

#[test]
fn io_thread_controls_reach_sim_untorn_with_every_edge_and_fail_closed_staleness() {
    let mut bytes = Vec::new();
    write_world(&synthetic_sfo_like(), Limits::default(), &mut bytes).unwrap();
    let frame = WorldFrame::new(Arc::new(validate_world_pack(&bytes).unwrap()));
    let expected = Arc::new(expected_tuples());
    let neutral = tuple_of(&ControlInput::default());
    assert!(!expected.contains(&neutral));

    let (writer, mut reader) = control_mailbox();
    let exchange = Arc::new(SnapshotExchange::new());
    let sim_ticks = Arc::new(AtomicU64::new(0));
    let io_done = Arc::new(AtomicBool::new(false));
    let sim_done = Arc::new(AtomicBool::new(false));
    let start = Arc::new(Barrier::new(3));

    let io = {
        let sim_ticks = Arc::clone(&sim_ticks);
        let io_done = Arc::clone(&io_done);
        let sim_done = Arc::clone(&sim_done);
        let start = Arc::clone(&start);
        thread::spawn(move || {
            let _done = SetOnDrop(Arc::clone(&io_done));
            let mut machine = ControlMachine::new(InputConfig::extreme_3d_pro());
            let mut publisher = InputPublisher::new(writer);
            let mut ev = Events { sequence: 0, time_ns: 0 };
            let mut totals = IoTotals::default();
            let buttons = [button::START, button::PAUSE, button::RESET];
            // Press every 13/17/401 reports, release on the next report.
            let period = [13u64, 17, 401];
            start.wait();
            for report in 1..=IO_REPORTS {
                while sim_ticks.load(Ordering::Relaxed) * IO_REPORTS_PER_TICK < report {
                    if sim_done.load(Ordering::SeqCst) {
                        return totals; // sim thread exited early; its join reports why
                    }
                    thread::yield_now();
                }
                let now = ev.time_ns;
                let wait_ticks = |n: u64, publisher: &mut InputPublisher, needs_resync: bool, t: u64| {
                    let until = sim_ticks.load(Ordering::Relaxed) + n;
                    while sim_ticks.load(Ordering::Relaxed) < until {
                        if sim_done.load(Ordering::SeqCst) {
                            return false;
                        }
                        publisher.heartbeat(needs_resync, t);
                        thread::yield_now();
                    }
                    true
                };
                if report == LONG_LOSS_AT {
                    // INPUT-002: make sure the core runs (release + press
                    // Start), then disconnect for > 10 s of sim time.
                    let start_key = gx_code::BTN_TRIGGER + u16::from(button::START);
                    let raw = raw_for(report);
                    let (m, p, e, t) = (&mut machine, &mut publisher, &mut ev, &mut totals);
                    send_report(m, p, e, t, raw, &[(start_key, 0)]);
                    send_report(m, p, e, t, raw, &[(start_key, 1)]);
                    if !wait_ticks(4, &mut publisher, false, ev.time_ns) { return totals; }
                    publisher.publish_disconnected(ev.time_ns);
                    if !wait_ticks(LONG_LOSS_TICKS, &mut publisher, false, ev.time_ns) { return totals; }
                    // Device back: valid reports, no Start yet. The core must
                    // stay paused for input loss with the warning cleared.
                    let (m, p, e, t) = (&mut machine, &mut publisher, &mut ev, &mut totals);
                    send_report(m, p, e, t, raw, &[(start_key, 0)]);
                    if !wait_ticks(RECOVERED_WITHOUT_START_TICKS, &mut publisher, false, ev.time_ns) {
                        return totals;
                    }
                    let (m, p, e, t) = (&mut machine, &mut publisher, &mut ev, &mut totals);
                    send_report(m, p, e, t, raw, &[(start_key, 1)]);
                    continue;
                }
                if report % LOSS_EVERY == 0 {
                    // Kernel queue overflow mid-report: fail closed, hold
                    // Disarmed for a few sim ticks with heartbeats, then
                    // resync from a full device snapshot.
                    let raw = raw_for(report);
                    for event in [
                        ev.next(gx_code::EV_ABS, gx_code::ABS_X, raw),
                        ev.next(gx_code::EV_SYN, gx_code::SYN_DROPPED, 0),
                        ev.next(gx_code::EV_ABS, gx_code::ABS_Y, raw),
                        ev.next(gx_code::EV_SYN, gx_code::SYN_REPORT, 0),
                    ] {
                        let out = machine.apply_detailed(event).unwrap();
                        match out {
                            MachineOutput::LossDetected(_) => totals.losses += 1,
                            MachineOutput::ResyncRequired(_) => totals.resyncs_required += 1,
                            MachineOutput::Frame(_) => panic!("frame while dropping"),
                            MachineOutput::None => {}
                        }
                        publisher.publish_output(&out, ev.time_ns);
                    }
                    let until = sim_ticks.load(Ordering::Relaxed) + DISARMED_TICKS;
                    while sim_ticks.load(Ordering::Relaxed) < until {
                        if sim_done.load(Ordering::SeqCst) {
                            return totals;
                        }
                        assert!(machine.needs_resync());
                        publisher.heartbeat(machine.needs_resync(), ev.time_ns);
                        thread::yield_now();
                    }
                    ev.sequence += 1;
                    ev.time_ns += 1_000;
                    let resync = machine
                        .resynchronize(DeviceSnapshot {
                            timestamp_ns: ev.time_ns,
                            source_event_sequence: ev.sequence,
                            axes: [
                                gx_code::ABS_X, gx_code::ABS_Y, gx_code::ABS_RZ,
                                gx_code::ABS_THROTTLE, gx_code::ABS_HAT0X, gx_code::ABS_HAT0Y,
                            ]
                            .into_iter()
                            .map(|c| (c, if c >= gx_code::ABS_HAT0X { 0 } else { raw }))
                            .collect(),
                            pressed_buttons: Vec::new(),
                        })
                        .unwrap();
                    assert_eq!(resync.buttons_pressed, 0, "resync must not invent edges");
                    totals.frames += 1;
                    publisher.publish_frame(&resync, ev.time_ns);
                    continue;
                }
                let raw = raw_for(report);
                let mut events = vec![
                    ev.next(gx_code::EV_ABS, gx_code::ABS_X, raw),
                    ev.next(gx_code::EV_ABS, gx_code::ABS_Y, raw),
                    ev.next(gx_code::EV_ABS, gx_code::ABS_RZ, raw),
                    ev.next(gx_code::EV_ABS, gx_code::ABS_THROTTLE, raw),
                ];
                for (k, b) in buttons.iter().enumerate() {
                    let key = gx_code::BTN_TRIGGER + u16::from(*b);
                    // The final reports press Start so the tail runs.
                    let press = report % period[k] == 0 || (k == 0 && report == IO_REPORTS);
                    let release = report % period[k] == 1 || (k == 0 && report == IO_REPORTS - 1);
                    if press {
                        events.push(ev.next(gx_code::EV_KEY, key, 1));
                    } else if release {
                        events.push(ev.next(gx_code::EV_KEY, key, 0));
                    }
                }
                events.push(ev.next(gx_code::EV_SYN, gx_code::SYN_REPORT, 0));
                for event in events {
                    let out = machine.apply_detailed(event).unwrap();
                    if let MachineOutput::Frame(f) = &out {
                        totals.frames += 1;
                        for (k, b) in buttons.iter().enumerate() {
                            if f.buttons_pressed & (1 << b) != 0 {
                                totals.edges[k] += 1;
                            }
                        }
                    }
                    publisher.publish_output(&out, now);
                }
            }
            // Fall silent: no heartbeat from here on, so the sim must go stale.
            // `_done` sets `io_done` as this closure returns.
            totals
        })
    };

    let sim = {
        let exchange = Arc::clone(&exchange);
        let sim_ticks = Arc::clone(&sim_ticks);
        let io_done = Arc::clone(&io_done);
        let sim_done = Arc::clone(&sim_done);
        let start = Arc::clone(&start);
        let expected = Arc::clone(&expected);
        let backend = FakeBackend::new(frame.clone());
        thread::spawn(move || {
            let _done = SetOnDrop(Arc::clone(&sim_done));
            let mut sim = SimLoop::new(backend, exchange);
            let mut rate = FixedRate::new(EPOCH_NS, SIM_HZ);
            let mut consumed = [0u64; 3];
            let mut delivered = [0u64; 3];
            let mut status_counts = [0u64; 4]; // valid, no-input, stale, unavailable
            let mut last_new_publication = (0u64, 0u64); // (publication, tick)
            let mut tail_left: Option<u64> = None;
            let mut tail = Vec::new();
            let mut last_sequence = 0;
            let mut loss_start_tick: Option<u64> = None;
            let mut loss_pauses = 0u64;
            let mut loss_pause_lifts = 0u64;
            let mut paused_for_loss_with_valid_input = 0u64;
            let mut warning_cleared_while_paused = 0u64;
            start.wait();
            loop {
                // Seen before this tick's read, so this read returns the I/O
                // thread's final publication (its swap happens-before the
                // SeqCst store of `io_done`).
                let io_finished = io_done.load(Ordering::SeqCst);
                let deadline = rate.next_deadline();
                let now = deadline.at_ns;
                let (prev_state, prev_reason) = (sim.run_state(), sim.pause_reason());
                let tick = sim.poll_controls(&mut reader, now).unwrap();
                let input = tick.input;
                // INPUT-002 checks.
                if input.status == ControlStatus::Valid {
                    loss_start_tick = None;
                } else if loss_start_tick.is_none() {
                    loss_start_tick = Some(deadline.tick);
                }
                let reason = sim.pause_reason();
                if reason == Some(PauseReason::InputLoss) && prev_reason != Some(PauseReason::InputLoss) {
                    loss_pauses += 1;
                    assert_eq!(prev_state, RunState::Running);
                    assert_eq!(deadline.tick - loss_start_tick.unwrap(), LOSS_PAUSE_TICKS, "pause not at 10 s");
                    let s = tick.status.expect("loss pause publishes a status snapshot");
                    assert_eq!((s.pause_reason, s.input.loss_ms), (Some(PauseReason::InputLoss), 10_000));
                    assert!(s.input.warning());
                } else if prev_state == RunState::Running && sim.run_state() == RunState::Running {
                    if let Some(start) = loss_start_tick {
                        assert!(deadline.tick - start < LOSS_PAUSE_TICKS, "still running after 10 s loss");
                    }
                }
                if prev_reason == Some(PauseReason::InputLoss) && sim.run_state() == RunState::Running {
                    assert_eq!(input.status, ControlStatus::Valid, "unpaused without valid input");
                    assert!(input.commands.iter().any(|c| c == Command::Start), "unpaused without Start");
                    loss_pause_lifts += 1;
                }
                if reason == Some(PauseReason::InputLoss) && input.status == ControlStatus::Valid {
                    paused_for_loss_with_valid_input += 1;
                    if let Some(s) = tick.status {
                        assert_eq!(s.input.status, InputStatus::Valid);
                        assert!(!s.input.warning());
                        assert_eq!(s.pause_reason, Some(PauseReason::InputLoss));
                        warning_cleared_while_paused += 1;
                    }
                }
                // Commands: each kind whose counter advanced is delivered once, except
                // a Start withheld by the INPUT-002 gate.
                let batch: Vec<_> = input.commands.iter().collect();
                for (k, cmd) in [Command::Start, Command::Pause, Command::Reset].iter().enumerate() {
                    let count = input.record.edges[k].count;
                    let advanced = count != consumed[k];
                    let withheld = *cmd == Command::Start && input.rejected_start;
                    assert_eq!(advanced, batch.contains(cmd) || withheld, "edge for {cmd:?} lost or invented");
                    assert!(!(batch.contains(cmd) && withheld));
                    consumed[k] = count;
                    if batch.contains(cmd) {
                        delivered[k] += 1;
                    }
                }
                // Controls: a whole valid tuple, or exactly neutral.
                let applied = tuple_of(&sim.controls());
                match input.status {
                    ControlStatus::Valid => {
                        status_counts[0] += 1;
                        assert_eq!(input.record.validity, InputValidity::Valid);
                        assert!(expected.contains(&applied), "torn controls {applied:?}");
                    }
                    other => {
                        status_counts[match other {
                            ControlStatus::NoInput => 1,
                            ControlStatus::Stale { .. } => 2,
                            _ => 3,
                        }] += 1;
                        assert_eq!(applied, neutral, "{other:?} must give neutral controls");
                    }
                }
                if input.record.publication != last_new_publication.0 {
                    last_new_publication = (input.record.publication, deadline.tick);
                }
                if let Some(s) = tick.reset {
                    last_sequence = s.sequence;
                }
                if let Some(s) = sim.tick(now).unwrap() {
                    last_sequence = s.sequence;
                    assert_eq!(snapshot_tuple(&s), applied, "step used other controls");
                }
                rate.complete_tick(now);
                sim_ticks.store(deadline.tick + 1, Ordering::Relaxed);
                if io_finished && tail_left.is_none() {
                    tail_left = Some(TAIL_TICKS);
                }
                if let Some(left) = tail_left.as_mut() {
                    tail.push((deadline.tick - last_new_publication.1, input.status));
                    *left -= 1;
                    if *left == 0 {
                        break;
                    }
                }
                assert!(deadline.tick < 50_000_000, "runaway sim loop");
                thread::yield_now();
            }
            let mut attempts = 0;
            while !sim.flush() {
                attempts += 1;
                assert!(attempts < 1_000_000, "pending snapshot never published");
                thread::yield_now();
            }
            let loss = (loss_pauses, loss_pause_lifts, paused_for_loss_with_valid_input, warning_cleared_while_paused, sim.refused_starts());
            (consumed, delivered, status_counts, tail, sim.run_state(), reader.stats(), last_sequence, loss)
        })
    };

    let render = {
        let exchange = Arc::clone(&exchange);
        let sim_done = Arc::clone(&sim_done);
        let start = Arc::clone(&start);
        let frame = frame.clone();
        let expected = Arc::clone(&expected);
        thread::spawn(move || {
            start.wait();
            let mut last: Option<AircraftSnapshot> = None;
            let mut observations = 0u64;
            let mut neutral_seen = 0u64;
            loop {
                let finished = sim_done.load(Ordering::SeqCst);
                if let Some(pair) = exchange.try_latest_pair() {
                    for s in [Some(pair.latest), pair.previous].into_iter().flatten() {
                        let pose = s.state.local_enu.unwrap();
                        assert_eq!(pose.position_m, frame.enu_m(s.state.geodetic), "torn snapshot");
                        let t = snapshot_tuple(&s);
                        if t == neutral {
                            neutral_seen += 1;
                        } else {
                            assert!(expected.contains(&t), "snapshot carries torn controls {t:?}");
                        }
                    }
                    if let Some(prev) = pair.previous {
                        assert_eq!(prev.generation, pair.latest.generation);
                    }
                    if let Some(prev) = last {
                        assert!(pair.latest.sequence >= prev.sequence);
                    }
                    observations += 1;
                    last = Some(pair.latest);
                }
                if finished {
                    break;
                }
            }
            (last.unwrap(), observations, neutral_seen)
        })
    };

    let sim_result = sim.join();
    let io_totals = io.join().unwrap();
    let (consumed, delivered, status, tail, run_state, stats, last_sequence, loss) = sim_result.unwrap();
    let (loss_pauses, loss_pause_lifts, paused_valid, warning_cleared, refused_starts) = loss;
    assert!(loss_pauses >= 1, "the > 10 s loss never paused the sim");
    assert_eq!(loss_pause_lifts, loss_pauses, "every loss pause lifted by an explicit Start");
    assert_eq!(warning_cleared, loss_pauses, "warning clears once per recovery while paused");
    assert!(paused_valid >= RECOVERED_WITHOUT_START_TICKS - 5, "auto-unpause? {paused_valid}");
    let (final_seen, observations, neutral_seen) = render.join().unwrap();

    // No lost command edges: every edge gx-input produced is in the final
    // counters, and every advance was delivered (collapsed edges counted).
    assert_eq!(consumed, io_totals.edges, "edge counters");
    assert_eq!(
        stats.commands_delivered + stats.rejected_starts + stats.coalesced_edges,
        io_totals.edges.iter().sum::<u64>()
    );
    assert_eq!(stats.commands_delivered, delivered.iter().sum::<u64>());
    assert!(delivered.iter().all(|&d| d > 0));
    assert!(io_totals.losses > 0 && io_totals.losses == io_totals.resyncs_required);
    assert!(status[3] > 0, "Disarmed never reached the sim as unavailable");

    // Stale policy: valid up to 12 ticks (99.99 ms) after the last new
    // publication, stale (neutral) from the 13th tick (108.3 ms > 100 ms).
    let bound_ticks = DEFAULT_STALE_AFTER_NS / TICK_NS; // 12
    for (age_ticks, s) in &tail {
        if *age_ticks <= bound_ticks {
            assert_eq!(*s, ControlStatus::Valid, "age {age_ticks} ticks");
        } else {
            assert!(matches!(s, ControlStatus::Stale { .. }), "age {age_ticks}: {s:?}");
        }
    }
    assert!(tail.last().unwrap().0 > bound_ticks, "tail too short to go stale");
    assert_eq!(run_state, RunState::Running, "stale input neutralizes, never pauses");
    assert_eq!(final_seen.sequence, last_sequence);
    assert_eq!(snapshot_tuple(&final_seen), neutral, "final stale steps use neutral controls");
    assert!(observations > 100);
    eprintln!(
        "io->sim->render: loss_pauses={loss_pauses} paused_with_valid_input={paused_valid} \
         rejected_starts={} core_refused_starts={refused_starts} \
         reports={} frames={} edges={:?} delivered={:?} coalesced={} \
         losses={} reads={} new_pubs={} skipped_pubs={} valid={} no_input={} stale={} \
         unavailable={} render_obs={observations} render_neutral={neutral_seen}",
        stats.rejected_starts, IO_REPORTS, io_totals.frames, io_totals.edges, delivered, stats.coalesced_edges,
        io_totals.losses, stats.reads, stats.new_publications, stats.skipped_publications,
        status[0], status[1], status[2], status[3]
    );
}

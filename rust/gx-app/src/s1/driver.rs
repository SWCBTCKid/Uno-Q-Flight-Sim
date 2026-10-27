//! Simulation-thread logic, separated from the clock so it runs unchanged
//! under the real 120 Hz loop and under virtual time in host tests.

use super::rec::{
    bucket, DeadlineMiss, Gate, Latency, Rec, SimSecond, Source, StateRec, TickRec, Visible, BUSY_BUCKETS,
    BUSY_RUN_BUCKETS, LATE_BUCKETS, LATE_BUCKETS_US,
};
use crate::{PublishStats, SimLoop, SnapshotSink};
use gx_sim::{
    AircraftSnapshot, AircraftState, Command, ControlReader, ControlStatus, FixedRate, InputValidity, ReaderStats,
    RunState, SimulationBackend, SIM_HZ,
};

pub const TICK_NS: u64 = 1_000_000_000 / SIM_HZ as u64;

/// Counts across the run, returned when the simulation thread ends.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SimTotals {
    pub ticks: u64,
    pub misses: u64,
    pub skipped: u64,
    pub late_max_ns: u64,
    pub work_max_ns: u64,
    pub late_hist: [u64; LATE_BUCKETS],
    pub busy_runs: u64,
    pub busy_run_hist: [u64; BUSY_BUCKETS],
    pub refused_mailbox: u64,
    pub refused_core_stick: u64,
    pub refused_core_console: u64,
    pub accepted_starts: u64,
    pub accepted_console: u64,
    pub latency_samples: u64,
    pub faults: u64,
    pub drops: u64,
}

/// One `state` record every this many stepped ticks (10 Hz) when tracing.
pub const STATE_EVERY_TICKS: u64 = 12;

/// S2 record/replay trace: one `tick` record per sim tick (the mailbox
/// record when a new publication was consumed, the console Start count, the
/// step cost and a hash of the state produced) plus a 10 Hz `state` record.
#[derive(Clone, Copy, Debug, Default)]
struct Trace {
    n: u64,
    last_publication: u64,
    stepped: u64,
}

/// FNV-1a over the bit patterns of every field of the state, so equal hashes
/// mean a bit-identical state (to within FNV's 64-bit collision odds).
pub fn state_hash(s: &AircraftState) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |v: u64| {
        for b in v.to_le_bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    let g = s.geodetic;
    for v in [g.latitude_deg, g.longitude_deg, g.ellipsoid_height_m, s.height_agl_m] {
        eat(v.to_bits());
    }
    match s.local_enu {
        Some(p) => {
            eat(p.frame_id);
            eat(p.frame_generation);
            for v in p.position_m.into_iter().chain(p.velocity_mps).chain(p.orientation_xyzw) {
                eat(v.to_bits());
            }
        }
        None => eat(u64::MAX),
    }
    for v in [s.roll_rad, s.pitch_rad, s.heading_rad]
        .into_iter()
        .chain(s.angular_rate_rps)
        .chain([
            s.calibrated_airspeed_mps, s.vertical_speed_mps, s.gear_position, s.flap_position,
            s.spoiler_position, s.elevator_position, s.aileron_position, s.rudder_position, s.engine_n1,
        ])
    {
        eat(v.to_bits());
    }
    eat(u64::from(s.weight_on_wheels));
    h
}

/// Everything the S1 simulation thread owns besides the clock.
pub struct SimDriver<B, S> {
    sim: SimLoop<B, S>,
    reader: ControlReader,
    last_frame_seq: Option<u64>,
    seen_console: u64,
    prev: Option<Visible>,
    last_busy_events: u64,
    busy_run_ticks: u64,
    busy_run_attempts: u64,
    trace: Option<Trace>,
    last: Option<AircraftSnapshot>,
    pub totals: SimTotals,
}

impl<B: SimulationBackend, S: SnapshotSink> SimDriver<B, S>
where
    B::Error: std::fmt::Debug,
{
    /// Resets the core to its initial position (paused, reason `Reset`), so
    /// the first stick Start begins the run.
    pub fn new(backend: B, sink: S, reader: ControlReader, now_ns: u64, out: &mut impl FnMut(Rec)) -> Self {
        Self::try_new(backend, sink, reader, now_ns, out).expect("backend reset at the initial position")
    }

    /// As [`SimDriver::new`], reporting a failed initial reset.
    pub fn try_new(
        backend: B,
        sink: S,
        reader: ControlReader,
        now_ns: u64,
        out: &mut impl FnMut(Rec),
    ) -> Result<Self, crate::AppError<B::Error>> {
        Self::try_new_with_levers(backend, sink, reader, now_ns, gx_sim::Levers::default(), out)
    }

    /// As [`SimDriver::try_new`] with the aircraft's initial levers (INPUT-005).
    pub fn try_new_with_levers(
        backend: B,
        sink: S,
        reader: ControlReader,
        now_ns: u64,
        levers: gx_sim::Levers,
        out: &mut impl FnMut(Rec),
    ) -> Result<Self, crate::AppError<B::Error>> {
        let mut sim = SimLoop::with_levers(backend, sink, levers);
        sim.command(Command::Reset, now_ns)?;
        out(Rec::Command { t: now_ns, command: Command::Reset, source: Source::App });
        Ok(Self {
            sim,
            reader,
            last_frame_seq: None,
            seen_console: 0,
            prev: None,
            last_busy_events: 0,
            busy_run_ticks: 0,
            busy_run_attempts: 0,
            trace: None,
            last: None,
            totals: SimTotals::default(),
        })
    }

    /// The runway ends the autopilot's ILS can fly (I4,
    /// `s2::setup::approaches`). Call on the simulation thread, before the
    /// first tick; a replay must use the same list.
    pub fn with_approaches(mut self, approaches: Vec<gx_sim::Approach>) -> Self {
        self.sim = self.sim.with_approaches(approaches);
        self
    }

    /// Emit the S2 `tick` and `state` records (record/replay trace).
    pub fn with_trace(mut self) -> Self {
        self.trace = Some(Trace::default());
        self
    }

    /// The latest snapshot this driver produced (step or reset).
    pub fn last_snapshot(&self) -> Option<&AircraftSnapshot> {
        self.last.as_ref()
    }

    /// Number of `tick` records emitted (0 without a trace).
    pub fn traced_ticks(&self) -> u64 {
        self.trace.map_or(0, |t| t.n)
    }

    /// Controls latched for the next step (neutral unless input is valid).
    pub fn controls(&self) -> gx_sim::ControlInput {
        self.sim.controls()
    }

    fn visible(&self) -> Visible {
        let health = self.sim.input_health();
        Visible {
            run: self.sim.run_state(),
            pause: self.sim.pause_reason(),
            status: health.status,
            warning: health.warning(),
        }
    }

    pub fn loss_ms(&self) -> u64 {
        self.sim.input_health().loss_ms
    }

    pub fn publish_stats(&self) -> PublishStats {
        self.sim.publish_stats()
    }

    pub fn mailbox_stats(&self) -> ReaderStats {
        self.reader.stats()
    }

    pub fn run_state(&self) -> RunState {
        self.sim.run_state()
    }

    /// One tick boundary: read the mailbox, apply commands (and any console
    /// Start), step, publish, and emit the measurements. `end_clock` is read
    /// once after the step and publish. Returns the visible state.
    pub fn tick(
        &mut self,
        wake_ns: u64,
        console_starts: u64,
        end_clock: &mut dyn FnMut() -> u64,
        out: &mut impl FnMut(Rec),
    ) -> Visible {
        let publish_before = self.sim.publish_stats();
        let before = self.visible();
        let refused_before = self.sim.refused_starts();
        let mut reset_snapshot = None;
        let input = match self.sim.poll_controls(&mut self.reader, wake_ns) {
            Ok(tick) => {
                reset_snapshot = tick.reset;
                Some(tick.input)
            }
            Err(_) => {
                self.totals.faults += 1;
                out(Rec::Fault { t: wake_ns, tick: self.sim.simulation_tick() });
                None
            }
        };
        let mut latency = None;
        if let Some(input) = input {
            for command in input.commands.iter() {
                out(Rec::Command { t: wake_ns, command, source: Source::Stick });
            }
            let after = self.visible();
            if input.rejected_start {
                self.totals.refused_mailbox += 1;
                out(Rec::StartRefused {
                    t: wake_ns,
                    gate: Gate::Mailbox,
                    source: Source::Stick,
                    vis: after,
                    loss_ms: self.loss_ms(),
                });
            }
            if self.sim.refused_starts() > refused_before {
                self.totals.refused_core_stick += 1;
                out(Rec::StartRefused {
                    t: wake_ns,
                    gate: Gate::Core,
                    source: Source::Stick,
                    vis: after,
                    loss_ms: self.loss_ms(),
                });
            }
            let started = input.commands.iter().any(|c| c == Command::Start);
            if started && before.run != RunState::Running && after.run == RunState::Running {
                self.totals.accepted_starts += 1;
                out(Rec::StartAccepted {
                    t: wake_ns,
                    source: Source::Stick,
                    from_run: before.run,
                    from_pause: before.pause,
                });
            }
            // Stick-to-snapshot latency: the first tick that sees a new
            // valid frame sequence consumes that frame.
            let record = input.record;
            if input.status == ControlStatus::Valid
                && record.validity == InputValidity::Valid
                && self.last_frame_seq != Some(record.input_sequence)
            {
                let skipped = self
                    .last_frame_seq
                    .filter(|&last| record.input_sequence > last)
                    .map_or(0, |last| record.input_sequence - last - 1);
                self.last_frame_seq = Some(record.input_sequence);
                latency = Some((record.input_sequence, record.input_timestamp_ns, skipped));
            }
        }
        if console_starts != self.seen_console {
            self.seen_console = console_starts;
            let prior = self.visible();
            let refused = self.sim.refused_starts();
            out(Rec::Command { t: wake_ns, command: Command::Start, source: Source::Console });
            if self.sim.command(Command::Start, wake_ns).is_err() {
                self.totals.faults += 1;
            }
            if self.sim.refused_starts() > refused {
                self.totals.refused_core_console += 1;
                out(Rec::StartRefused {
                    t: wake_ns,
                    gate: Gate::Core,
                    source: Source::Console,
                    vis: self.visible(),
                    loss_ms: self.loss_ms(),
                });
            } else if prior.run != RunState::Running && self.sim.run_state() == RunState::Running {
                self.totals.accepted_starts += 1;
                self.totals.accepted_console += 1;
                out(Rec::StartAccepted {
                    t: wake_ns,
                    source: Source::Console,
                    from_run: prior.run,
                    from_pause: prior.pause,
                });
            }
        }

        let step_start = if self.trace.is_some() { end_clock() } else { 0 };
        let stepped = match self.sim.tick(wake_ns) {
            Ok(snapshot) => snapshot,
            Err(_) => {
                self.totals.faults += 1;
                out(Rec::Fault { t: wake_ns, tick: self.sim.simulation_tick() });
                None
            }
        };
        let end = end_clock();
        if let Some(s) = stepped.or(reset_snapshot) {
            self.last = Some(s);
        }
        if self.trace.is_some() {
            self.emit_trace(wake_ns, console_starts, end.saturating_sub(step_start), stepped, reset_snapshot, out);
        }
        let stats = self.sim.publish_stats();
        let running = self.sim.run_state() == RunState::Running;
        if let Some((seq, frame_ts, skipped_frames)) = latency {
            self.totals.latency_samples += 1;
            out(Rec::Latency(Latency {
                t: end,
                tick: self.sim.simulation_tick(),
                seq,
                skipped_frames,
                frame_ts,
                consume_ns: wake_ns.saturating_sub(frame_ts),
                publish_ns: end.saturating_sub(frame_ts),
                published: stats.offered > publish_before.offered
                    && stats.published > publish_before.published
                    && !stats.pending,
                running,
            }));
        }

        // ISS-034: a busy run is consecutive ticks with >= 1 PublishBusy.
        let busy = stats.busy_events - self.last_busy_events;
        self.last_busy_events = stats.busy_events;
        if busy > 0 {
            self.busy_run_ticks += 1;
            self.busy_run_attempts += busy;
        } else if self.busy_run_ticks > 0 {
            self.end_busy_run(end, out);
        }

        let vis = self.visible();
        if self.prev != Some(vis) {
            self.prev = Some(vis);
            out(Rec::Input002 {
                t: wake_ns,
                tick: self.sim.simulation_tick(),
                vis,
                loss_ms: self.loss_ms(),
            });
        }
        vis
    }

    fn emit_trace(
        &mut self,
        wake_ns: u64,
        console_starts: u64,
        step_ns: u64,
        stepped: Option<AircraftSnapshot>,
        reset: Option<AircraftSnapshot>,
        out: &mut impl FnMut(Rec),
    ) {
        let read = self.sim.last_read();
        let controls = self.sim.controls();
        let applied = self.sim.applied_controls();
        let visible = self.visible();
        let Some(trace) = self.trace.as_mut() else { return };
        trace.n += 1;
        let record = read.map(|r| r.record).filter(|r| r.publication != trace.last_publication);
        if let Some(r) = record {
            trace.last_publication = r.publication;
        }
        let produced = stepped.or(reset);
        out(Rec::Tick(TickRec {
            n: trace.n,
            wake_ns,
            console: console_starts,
            record,
            step_ns: stepped.map(|_| step_ns),
            hash: produced.map(|s| state_hash(&s.state)),
        }));
        let sample = match (stepped, reset) {
            (Some(s), _) => {
                trace.stepped += 1;
                (trace.stepped % STATE_EVERY_TICKS == 0).then_some(s)
            }
            (None, Some(r)) => Some(r),
            (None, None) => None,
        };
        if let Some(s) = sample {
            out(Rec::State(StateRec::new(wake_ns, &s, controls, applied, visible)));
        }
    }

    fn end_busy_run(&mut self, t: u64, out: &mut impl FnMut(Rec)) {
        self.totals.busy_runs += 1;
        self.totals.busy_run_hist[bucket(&BUSY_RUN_BUCKETS, self.busy_run_ticks)] += 1;
        out(Rec::BusyRun {
            t,
            tick: self.sim.simulation_tick(),
            ticks: self.busy_run_ticks,
            attempts: self.busy_run_attempts,
        });
        self.busy_run_ticks = 0;
        self.busy_run_attempts = 0;
    }

    /// Close an open busy run at shutdown.
    pub fn finish(&mut self, t: u64, out: &mut impl FnMut(Rec)) {
        if self.busy_run_ticks > 0 {
            self.end_busy_run(t, out);
        }
    }
}

/// Absolute-deadline cadence accounting for the 120 Hz loop.
///
/// Tick `k` is due at `epoch + floor(k / 120 s)` (gx-sim `FixedRate`
/// arithmetic, no drift). **Lateness** is wake time minus due time. A tick
/// **misses its deadline** when its work (mailbox read, step, publish) has
/// not finished by the next tick's due time, i.e. the 120 Hz cadence slipped.
/// After a miss the loop does not run catch-up ticks: it continues at the
/// first deadline after the overrun and counts the periods skipped.
#[derive(Clone, Debug)]
pub struct Cadence {
    epoch_ns: u64,
    next_tick: u64,
    pub second: SimSecond,
    pub totals_ticks: u64,
    pub totals_misses: u64,
    pub totals_skipped: u64,
    pub late_max_ns: u64,
    pub work_max_ns: u64,
    pub late_hist: [u64; LATE_BUCKETS],
    second_start: u64,
    last_miss: Option<DeadlineMiss>,
}

impl Cadence {
    pub fn new(epoch_ns: u64) -> Self {
        Self {
            epoch_ns,
            next_tick: 1,
            second: SimSecond::default(),
            totals_ticks: 0,
            totals_misses: 0,
            totals_skipped: 0,
            late_max_ns: 0,
            work_max_ns: 0,
            late_hist: [0; LATE_BUCKETS],
            second_start: epoch_ns,
            last_miss: None,
        }
    }

    pub fn due(&self, tick: u64) -> u64 {
        self.epoch_ns + FixedRate::elapsed_ns(SIM_HZ, tick)
    }

    pub fn next_due(&self) -> u64 {
        self.due(self.next_tick)
    }

    /// Return the detail for the most recently completed missed tick once.
    pub fn take_miss(&mut self) -> Option<DeadlineMiss> {
        self.last_miss.take()
    }

    /// Account for the tick due at `next_due()`, which woke at `wake` and
    /// finished at `end`. Returns a per-second summary when one is complete.
    pub fn complete(&mut self, wake: u64, end: u64) -> Option<SimSecond> {
        let tick = self.next_tick;
        let due = self.next_due();
        let late = wake.saturating_sub(due);
        let work = end.saturating_sub(wake);
        let following = self.due(self.next_tick + 1);
        let mut skipped = 0;
        let missed = end > following;
        if missed {
            let mut k = self.next_tick + 1;
            while self.due(k) <= end {
                k += 1;
            }
            skipped = k - (self.next_tick + 1);
            self.next_tick = k;
            self.last_miss = Some(DeadlineMiss {
                t: end, tick, due_ns: due, wake_ns: wake, end_ns: end, skipped,
            });
        } else {
            self.next_tick += 1;
        }
        let b = bucket(&LATE_BUCKETS_US, late / 1_000);
        let s = &mut self.second;
        s.ticks += 1;
        s.late_max_ns = s.late_max_ns.max(late);
        s.work_max_ns = s.work_max_ns.max(work);
        s.late_hist[b] += 1;
        s.misses += u64::from(missed);
        s.skipped += skipped;
        self.late_hist[b] += 1;
        self.totals_ticks += 1;
        self.totals_misses += u64::from(missed);
        self.totals_skipped += skipped;
        self.late_max_ns = self.late_max_ns.max(late);
        self.work_max_ns = self.work_max_ns.max(work);
        if wake.saturating_sub(self.second_start) >= 1_000_000_000 {
            let mut done = std::mem::take(&mut self.second);
            done.t = wake;
            self.second_start = wake;
            return Some(done);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{button, s1::synthetic_world_frame, FakeBackend, InputPublisher};
    use gx_input::ControlFrame;
    use gx_sim::{control_mailbox, SnapshotExchange};
    use std::sync::Arc;

    #[test]
    fn cadence_counts_overruns_as_misses_and_skips_without_catch_up() {
        let mut c = Cadence::new(0);
        // Tick 1 due 8_333_333: wakes 40 µs late, done well within period.
        assert_eq!(c.next_due(), 8_333_333);
        c.complete(8_373_333, 8_400_000);
        assert_eq!((c.totals_misses, c.totals_skipped), (0, 0));
        assert_eq!(c.next_due(), 16_666_666);
        // Tick 2 overruns past tick 3 (25 ms) and tick 4 (33.3 ms).
        c.complete(16_666_700, 34_000_000);
        assert_eq!((c.totals_misses, c.totals_skipped), (1, 2));
        assert_eq!(c.next_due(), FixedRate::elapsed_ns(SIM_HZ, 5));
        // A wake exactly on time is lateness 0 (bucket 0).
        let due = c.next_due();
        c.complete(due, due + 1_000);
        assert_eq!(c.late_hist[0], 2);
        assert_eq!(c.late_hist[1], 1);
        assert_eq!(c.totals_ticks, 3);
    }

    /// The sim thread runs `SCHED_FIFO` 50 (BOARD-001). It must block every
    /// iteration: after any tick, overrun or not, the next deadline is not
    /// before the tick's end, and after an overrun it is strictly after it,
    /// so `clock_nanosleep(TIMER_ABSTIME)` sleeps instead of returning at
    /// once and a slow step can never turn the loop into a spin.
    #[test]
    fn next_deadline_never_precedes_the_end_of_the_tick_so_a_fifo_sim_sleeps() {
        let mut c = Cadence::new(0);
        let mut x = 12_345u64;
        for _ in 0..20_000 {
            // Pseudo-random lateness and work, up to ~4 periods of overrun.
            x = x.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            let due = c.next_due();
            let wake = due + (x >> 40) % 3_000_000;
            let end = wake + (x >> 20) % (4 * TICK_NS);
            let misses = c.totals_misses;
            c.complete(wake, end);
            if c.totals_misses > misses {
                assert!(c.next_due() > end, "after an overrun the next deadline is in the future");
            } else {
                assert!(c.next_due() >= end);
            }
        }
        assert!(c.totals_misses > 0 && c.totals_misses < c.totals_ticks);
    }

    #[test]
    fn cadence_records_a_late_wake_miss_with_bounded_detail() {
        let mut c = Cadence::new(0);
        let due = c.next_due();
        let wake = due + TICK_NS + 500_000;
        let end = wake + 75_000;
        c.complete(wake, end);
        let miss = c.take_miss().expect("one missed deadline record");
        assert_eq!((miss.tick, miss.due_ns, miss.wake_ns, miss.end_ns), (1, due, wake, end));
        assert_eq!(miss.skipped, 1);
        assert_eq!((c.totals_misses, c.totals_skipped), (1, 1));
        assert!(c.take_miss().is_none(), "detail is emitted at most once");
    }

    #[test]
    fn latency_published_requires_a_snapshot_offer_to_reach_the_exchange() {
        let (writer, reader) = control_mailbox();
        let mut input = InputPublisher::new(writer);
        let exchange = Arc::new(SnapshotExchange::new());
        let mut recs = Vec::new();
        let mut driver = SimDriver::new(
            FakeBackend::new(synthetic_world_frame().unwrap()), Arc::clone(&exchange), reader, 1,
            &mut |r| recs.push(r),
        );

        input.publish_frame(
            &ControlFrame { timestamp_ns: 10, sequence: 1, ..ControlFrame::default() }, 10,
        );
        driver.tick(20, 0, &mut || 30, &mut |r| recs.push(r));
        let first = recs.iter().find_map(|r| match r {
            Rec::Latency(l) if l.seq == 1 => Some(*l), _ => None,
        }).unwrap();
        assert!(!first.running);
        assert!(first.published, "the input-health transition published");

        input.publish_frame(
            &ControlFrame { timestamp_ns: 35, sequence: 2, ..ControlFrame::default() }, 35,
        );
        driver.tick(40, 0, &mut || 45, &mut |r| recs.push(r));
        let second = recs.iter().find_map(|r| match r {
            Rec::Latency(l) if l.seq == 2 => Some(*l), _ => None,
        }).unwrap();
        assert!(!second.running);
        assert!(!second.published, "an unchanged paused frame offered no snapshot");

        input.publish_frame(
            &ControlFrame {
                timestamp_ns: 50, sequence: 3, buttons_pressed: 1 << button::START,
                ..ControlFrame::default()
            },
            50,
        );
        driver.tick(60, 0, &mut || 70, &mut |r| recs.push(r));
        let third = recs.iter().find_map(|r| match r {
            Rec::Latency(l) if l.seq == 3 => Some(*l), _ => None,
        }).unwrap();
        assert!(third.running);
        assert!(third.published, "the running tick published this frame's snapshot");
    }
}

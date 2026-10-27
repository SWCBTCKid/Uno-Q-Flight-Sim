//! The ARCH-004 thread layout for S1 (headless).
//!
//! - **I/O thread** owns the input source and the [`IoBridge`] (mailbox
//!   writer), polls every 1 ms on absolute deadlines and heartbeats every
//!   5 ms. It does no file I/O and no blocking device work (open/close run on
//!   gx-input's connector helper); its log lines go through a bounded
//!   channel like everyone else's.
//! - **Log thread** (`gx-log`) is the only writer of the JSONL log: it drains
//!   the bounded log channels of the other threads and writes them, so a
//!   slow `write`/`flush` can never delay a mailbox publication.
//! - **Simulation thread** owns `SimCore<FakeBackend>` (inside
//!   [`SimDriver`]) and the mailbox reader; it ticks at 120 Hz on absolute
//!   `CLOCK_MONOTONIC` deadlines, reads the mailbox once per tick, publishes
//!   to `SnapshotExchange`, and never waits on another thread. Its only
//!   cross-thread operations are the mailbox swap, the exchange publish,
//!   relaxed atomic status stores and `SyncSender::try_send` into a bounded
//!   log channel (a full channel drops the record and counts it; the
//!   `try_send` never waits for channel capacity; full/disconnected outcomes
//!   are counted explicitly).
//! - **Reader thread** stands in for render: `try_latest_pair` at 30 Hz.
//! - **Main thread** prints the phase prompts; a stdin thread counts ENTER
//!   presses (console Start).

use super::driver::{Cadence, SimDriver, SimTotals};
use super::io::{InputSource, IoBridge, IoLog};
use super::phases::{Obs, Phase, Prompter};
use super::rec::{self, esc, list, ReadRec, Rec, BUSY_RUN_BUCKETS, LATE_BUCKETS_US, SCHEMA};
use super::rt::{self, Clock};
use crate::{FakeBackend, InputPublisher, PublishStats, WorldFrame};
use gx_sim::{
    control_mailbox, FixedRate, InputStatus, PauseReason, ReaderStats, RunState, SnapshotExchange,
    UnavailableReason, SIM_HZ,
};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread;

pub const READER_HZ: u32 = 30;
pub const IO_POLL_NS: u64 = 1_000_000;
const IO_CPU: usize = 1;
const SIM_CPU: usize = 2;
const READER_CPU: usize = 3;
const LOG_CHANNEL_CAPACITY: usize = 16_384;

pub struct S1Config {
    pub seconds: u64,
    pub prompts: bool,
    pub device: String,
    pub commit: &'static str,
}

/// Status the main thread reads for prompts. Written with relaxed stores;
/// a slightly stale prompt is harmless.
#[derive(Default)]
pub struct Board {
    pub stop_sim: AtomicBool,
    pub stop_io: AtomicBool,
    pub console_starts: AtomicU64,
    vis: AtomicU64,
    pub loss_ms: AtomicU64,
    pub refused_console: AtomicU64,
    pub accepted_console: AtomicU64,
    pub connected: AtomicBool,
    pub frames: AtomicU64,
    pub button_presses: AtomicU64,
    pub last_buttons: AtomicU64,
    /// S2's numbered-button-4 parking-brake latch.
    pub parking_brake: AtomicBool,
    /// S4: camera-button presses so far (the render thread picks the view).
    pub camera_presses: AtomicU64,
    /// AP-001: the panel cursor, packed by [`pack_cursor`] (0 = hidden).
    cursor: AtomicU64,
    /// AP-001: a mouse is open (false: the panel shows NO MOUSE).
    mouse_ok: AtomicBool,
    /// AP-002: see [`Board::exit_armed`].
    pub exit_armed: AtomicBool,
}

/// Cursor packing: bit 63 = shown, then x and y in 1/64 virtual pixel
/// (`gx_render::scene` 1280x720 fits in 17 bits each), so one relaxed load
/// gives a consistent position.
const CURSOR_SHOWN: u64 = 1 << 63;
const CURSOR_SCALE: f32 = 64.0;

fn pack_cursor(cursor: Option<(f32, f32)>) -> u64 {
    let Some((x, y)) = cursor else { return 0 };
    let q = |v: f32| (v.clamp(0.0, 65_535.0) * CURSOR_SCALE).round() as u64;
    CURSOR_SHOWN | (q(x) << 24) | q(y)
}

fn unpack_cursor(v: u64) -> Option<(f32, f32)> {
    (v & CURSOR_SHOWN != 0).then(|| {
        let mask = (1u64 << 24) - 1;
        (((v >> 24) & mask) as f32 / CURSOR_SCALE, (v & mask) as f32 / CURSOR_SCALE)
    })
}

fn encode_vis(run: RunState, pause: Option<PauseReason>, status: InputStatus) -> u64 {
    let r = match run {
        RunState::Stopped => 0,
        RunState::Running => 1,
        RunState::Paused => 2,
    };
    let p = match pause {
        None => 0,
        Some(PauseReason::User) => 1,
        Some(PauseReason::Reset) => 2,
        Some(PauseReason::InputLoss) => 3,
        Some(PauseReason::InvalidWorldPose) => 4,
    };
    let s = match status {
        InputStatus::Valid => 0,
        InputStatus::NoInput => 1,
        InputStatus::Stale => 2,
        InputStatus::Unavailable(UnavailableReason::Disconnected) => 3,
        InputStatus::Unavailable(UnavailableReason::InputLoss) => 4,
        InputStatus::Unavailable(UnavailableReason::ResyncRequired) => 5,
    };
    r | (p << 4) | (s << 8)
}

fn decode_vis(v: u64) -> (RunState, Option<PauseReason>, InputStatus) {
    let run = match v & 0xf {
        1 => RunState::Running,
        2 => RunState::Paused,
        _ => RunState::Stopped,
    };
    let pause = match (v >> 4) & 0xf {
        1 => Some(PauseReason::User),
        2 => Some(PauseReason::Reset),
        3 => Some(PauseReason::InputLoss),
        4 => Some(PauseReason::InvalidWorldPose),
        _ => None,
    };
    let status = match (v >> 8) & 0xf {
        0 => InputStatus::Valid,
        2 => InputStatus::Stale,
        3 => InputStatus::Unavailable(UnavailableReason::Disconnected),
        4 => InputStatus::Unavailable(UnavailableReason::InputLoss),
        5 => InputStatus::Unavailable(UnavailableReason::ResyncRequired),
        _ => InputStatus::NoInput,
    };
    (run, pause, status)
}

impl Board {
    pub fn set_vis(&self, run: RunState, pause: Option<PauseReason>, status: InputStatus) {
        self.vis.store(encode_vis(run, pause, status), Ordering::Relaxed);
    }

    pub fn vis(&self) -> (RunState, Option<PauseReason>, InputStatus) {
        decode_vis(self.vis.load(Ordering::Relaxed))
    }

    /// AP-001: the cursor in virtual 1280x720 pixels, `None` while hidden
    /// (no mouse, or no mouse activity for 10 s). The render thread reads it.
    pub fn cursor(&self) -> Option<(f32, f32)> {
        unpack_cursor(self.cursor.load(Ordering::Relaxed))
    }

    /// AP-001: a mouse is open. The mouse thread writes it.
    pub fn mouse_ok(&self) -> bool {
        self.mouse_ok.load(Ordering::Relaxed)
    }

    /// AP-002 (interface, Platform implements the writer): the panel's EXIT
    /// was clicked once and awaits the confirming click.
    pub fn exit_armed(&self) -> bool {
        self.exit_armed.load(Ordering::Relaxed)
    }

    /// Mouse thread only.
    pub fn set_cursor(&self, cursor: Option<(f32, f32)>) {
        self.cursor.store(pack_cursor(cursor), Ordering::Relaxed);
    }

    /// Mouse thread only.
    pub fn set_mouse_ok(&self, ok: bool) {
        self.mouse_ok.store(ok, Ordering::Relaxed);
    }
}

/// Bounded, nonblocking log producer handle. Never blocks the caller.
pub struct LogTx {
    tx: SyncSender<Rec>,
    pub dropped: u64,
}

impl LogTx {
    pub fn send(&mut self, rec: Rec) {
        match self.tx.try_send(rec) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => self.dropped += 1,
        }
    }
}

pub fn log_channel() -> (LogTx, Receiver<Rec>) {
    let (tx, rx) = sync_channel(LOG_CHANNEL_CAPACITY);
    (LogTx { tx, dropped: 0 }, rx)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ReaderTotals {
    pub reads: u64,
    pub none: u64,
    pub age_max_running_ns: u64,
    pub copy_max_ns: u64,
    pub drops: u64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct IoTotals {
    pub publications: u64,
    pub max_gap_ns: u64,
    pub max_loop_ns: u64,
    pub frames: u64,
    pub button_presses: u64,
    pub lines: u64,
    /// I/O-thread log lines dropped because the log channel was full.
    pub log_drops: u64,
}

pub struct SimResult {
    pub totals: SimTotals,
    pub publish: PublishStats,
    pub mailbox: ReaderStats,
    pub rt: String,
}

/// Run the S1 layout for `cfg.seconds` and write the JSONL log to `out`.
/// Returns the process exit code (0 = completed; the checker judges PASS).
pub fn run<C: Clock, S: InputSource + Send + 'static, W: Write + Send + 'static>(
    cfg: S1Config,
    clock: C,
    source: S,
    frame: WorldFrame,
    out: W,
    stdout: &mut dyn Write,
) -> io::Result<i32> {
    let epoch = clock.now_ns();
    let board = Arc::new(Board::default());
    board.set_vis(RunState::Stopped, None, InputStatus::NoInput);
    let exchange = Arc::new(SnapshotExchange::new());
    let (writer, reader) = control_mailbox();
    let (sim_tx, sim_rx) = log_channel();
    let (read_tx, read_rx) = log_channel();
    let (main_tx, main_rx) = sync_channel::<String>(1024);
    let (io_tx, io_rx) = sync_channel::<String>(LOG_CHANNEL_CAPACITY);
    let header = start_line(&cfg, epoch);
    let stop_log = Arc::new(AtomicBool::new(false));

    let log_thread = {
        let clock = clock.clone();
        let stop_log = Arc::clone(&stop_log);
        thread::Builder::new().name("gx-log".into()).spawn(move || {
            log_loop(clock, epoch, header, out, stop_log, io_rx, main_rx, sim_rx, read_rx)
        })?
    };

    // I/O thread next, so the mailbox has publications before tick 1.
    let io_thread = {
        let board = Arc::clone(&board);
        let clock = clock.clone();
        thread::Builder::new().name("gx-io".into()).spawn(move || {
            io_loop(clock, epoch, source, IoBridge::new(InputPublisher::new(writer)), board, io_tx)
        })?
    };

    let sim_thread = {
        let board = Arc::clone(&board);
        let clock = clock.clone();
        let exchange = Arc::clone(&exchange);
        let mut log = sim_tx;
        thread::Builder::new().name("gx-sim".into()).spawn(move || {
            let rt = rt::setup_sim_thread(SIM_CPU);
            let start = clock.now_ns();
            let mut driver = SimDriver::new(FakeBackend::new(frame), exchange, reader, start, &mut |r| log.send(r));
            // First deadline 10 ms out so tick 1 is not born late.
            let mut cadence = Cadence::new(start + 10_000_000 - FixedRate::elapsed_ns(SIM_HZ, 1));
            while !board.stop_sim.load(Ordering::Relaxed) {
                let due = cadence.next_due();
                clock.sleep_until(due);
                let wake = clock.now_ns();
                let console = board.console_starts.load(Ordering::Relaxed);
                let vis = driver.tick(wake, console, &mut || clock.now_ns(), &mut |r| log.send(r));
                let end = clock.now_ns();
                board.set_vis(vis.run, vis.pause, vis.status);
                board.loss_ms.store(driver.loss_ms(), Ordering::Relaxed);
                board.refused_console.store(driver.totals.refused_core_console, Ordering::Relaxed);
                board.accepted_console.store(driver.totals.accepted_console, Ordering::Relaxed);
                let second = cadence.complete(wake, end);
                if let Some(miss) = cadence.take_miss() {
                    log.send(Rec::DeadlineMiss(miss));
                }
                if let Some(mut second) = second {
                    second.busy_events = driver.publish_stats().busy_events;
                    second.drops = log.dropped;
                    log.send(Rec::SimSecond(second));
                }
            }
            let now = clock.now_ns();
            driver.finish(now, &mut |r| log.send(r));
            let mut totals = driver.totals;
            totals.ticks = cadence.totals_ticks;
            totals.misses = cadence.totals_misses;
            totals.skipped = cadence.totals_skipped;
            totals.late_max_ns = cadence.late_max_ns;
            totals.work_max_ns = cadence.work_max_ns;
            totals.late_hist = cadence.late_hist;
            totals.drops = log.dropped;
            SimResult { totals, publish: driver.publish_stats(), mailbox: driver.mailbox_stats(), rt }
        })?
    };

    let reader_thread = {
        let board = Arc::clone(&board);
        let clock = clock.clone();
        let exchange = Arc::clone(&exchange);
        let log = read_tx;
        thread::Builder::new()
            .name("gx-reader".into())
            .spawn(move || reader_loop(clock, exchange, board, log))?
    };

    // Console: each line on stdin is one console Start.
    {
        let board = Arc::clone(&board);
        let _ = thread::Builder::new().name("gx-stdin".into()).spawn(move || {
            rt::setup_background_thread();
            let mut line = String::new();
            while std::io::stdin().read_line(&mut line).map_or(false, |n| n > 0) {
                board.console_starts.fetch_add(1, Ordering::Relaxed);
                line.clear();
            }
        });
    }

    let main_rt = rt::setup_background_thread();
    let end_ns = epoch + cfg.seconds * 1_000_000_000;
    let mut prompter = Prompter::new();
    if cfg.prompts {
        writeln!(stdout, "gx-app s1: headless ARCH-004 threads, {} s run, stick {}", cfg.seconds, cfg.device)?;
        writeln!(stdout, "{}", super::phases::BUTTONS_HELP)?;
    }
    loop {
        let now = clock.now_ns();
        let (run, pause, status) = board.vis();
        let obs = Obs {
            t_ns: now - epoch,
            end_ns: end_ns - epoch,
            connected: board.connected.load(Ordering::Relaxed),
            frames: board.frames.load(Ordering::Relaxed),
            run,
            pause,
            status,
            loss_ms: board.loss_ms.load(Ordering::Relaxed),
            console_starts: board.console_starts.load(Ordering::Relaxed),
            refused_console: board.refused_console.load(Ordering::Relaxed),
            accepted_console: board.accepted_console.load(Ordering::Relaxed),
            button_presses: board.button_presses.load(Ordering::Relaxed),
            last_buttons: board.last_buttons.load(Ordering::Relaxed),
        };
        for (phase, text) in prompter.step(&obs) {
            let t_s = (now - epoch) as f64 / 1e9;
            if cfg.prompts {
                writeln!(stdout, "[{t_s:6.1} s] {text}")?;
                stdout.flush()?;
            }
            let _ = main_tx.try_send(format!(
                "{{\"k\":\"phase\",\"t\":{},\"phase\":\"{}\",\"text\":{}}}",
                now - epoch,
                phase.name(),
                esc(text.trim())
            ));
        }
        if prompter.phase == Phase::Done {
            break;
        }
        clock.sleep_until(now + 100_000_000);
    }

    board.stop_sim.store(true, Ordering::Relaxed);
    let sim = sim_thread.join().map_err(|_| io::Error::other("sim thread panicked"))?;
    let (reader_totals, reader_rt) =
        reader_thread.join().map_err(|_| io::Error::other("reader thread panicked"))?;
    board.stop_io.store(true, Ordering::Relaxed);
    let (mut io, io_rt) = io_thread.join().map_err(|_| io::Error::other("io thread panicked"))?;
    stop_log.store(true, Ordering::Release);
    let (mut out, lines, log_rt) = log_thread.join().map_err(|_| io::Error::other("log thread panicked"))??;
    io.lines = lines;

    let duration_ns = clock.now_ns() - epoch;
    let rts = [sim.rt.as_str(), io_rt.as_str(), reader_rt.as_str(), main_rt.as_str(), log_rt.as_str()];
    writeln!(out, "{}", summary_line(duration_ns, &sim, &reader_totals, &io, rts))?;
    let t = &sim.totals;
    let p = &sim.publish;
    writeln!(out, "{{\"k\":\"end\",\"t\":{duration_ns}}}")?;
    out.flush()?;
    if cfg.prompts {
        writeln!(
            stdout,
            "summary: ticks={} deadline_misses={} skipped={} late_max={:.3} ms latency_samples={} busy_events={} busy_runs={} max_busy_run={} refused_starts(mailbox/core-stick/core-console)={}/{}/{} log_drops={}",
            t.ticks, t.misses, t.skipped, t.late_max_ns as f64 / 1e6, t.latency_samples, p.busy_events, t.busy_runs,
            p.max_consecutive_busy, t.refused_mailbox, t.refused_core_stick, t.refused_core_console,
            t.drops + reader_totals.drops
        )?;
    }
    Ok(0)
}

/// The reader ("render" stand-in) thread: `try_latest_pair` at 30 Hz on
/// absolute deadlines, one `read` record per attempt.
pub(crate) fn reader_loop<C: Clock>(
    clock: C,
    exchange: Arc<SnapshotExchange>,
    board: Arc<Board>,
    mut log: LogTx,
) -> (ReaderTotals, String) {
    let rt = rt::setup_reader_thread(READER_CPU);
    let start = clock.now_ns();
    let mut totals = ReaderTotals::default();
    let mut k = 1u64;
    while !board.stop_sim.load(Ordering::Relaxed) {
        let due = start + FixedRate::elapsed_ns(READER_HZ, k);
        clock.sleep_until(due);
        let t0 = clock.now_ns();
        let pair = exchange.try_latest_pair();
        let t1 = clock.now_ns();
        let copy_ns = t1 - t0;
        totals.reads += 1;
        totals.copy_max_ns = totals.copy_max_ns.max(copy_ns);
        let rec = match pair {
            Some(pair) => {
                let latest = pair.latest;
                let age_ns = t1.saturating_sub(latest.published_at_ns);
                // A frozen snapshot is expected while paused. Use the
                // current packed sim state, not historic snapshot
                // state, to classify running-only staleness.
                let (run, _, status) = board.vis();
                let paused = run != RunState::Running;
                if !paused {
                    totals.age_max_running_ns = totals.age_max_running_ns.max(age_ns);
                }
                ReadRec {
                    t: t1,
                    ok: true,
                    seq: latest.sequence,
                    generation: latest.generation,
                    tick: latest.simulation_tick,
                    age_ns,
                    copy_ns,
                    paused,
                    pair: pair.previous.is_some(),
                    warning: status != InputStatus::Valid,
                }
            }
            None => {
                totals.none += 1;
                ReadRec {
                    t: t1, ok: false, seq: 0, generation: 0, tick: 0, age_ns: 0, copy_ns,
                    paused: false, pair: false, warning: false,
                }
            }
        };
        log.send(Rec::Read(rec));
        // Next 30 Hz deadline strictly in the future (no catch-up).
        k += 1;
        let now = clock.now_ns();
        while start + FixedRate::elapsed_ns(READER_HZ, k) <= now {
            k += 1;
        }
    }
    totals.drops = log.dropped;
    (totals, rt)
}

pub fn start_line(cfg: &S1Config, epoch: u64) -> String {
    format!(
        "{{\"k\":\"start\",\"t\":0,\"schema\":\"{SCHEMA}\",\"commit\":{},\"seconds\":{},\"device\":{},\"clock\":\"CLOCK_MONOTONIC\",\"epoch_monotonic_ns\":{epoch},\"sim_hz\":{SIM_HZ},\"reader_hz\":{READER_HZ},\"io_poll_ns\":{IO_POLL_NS},\"heartbeat_ns\":{},\"tick_ns\":{},\"buttons\":{{\"start\":\"BTN_TRIGGER\",\"pause\":\"BTN_THUMB\",\"reset\":\"BTN_THUMB2\"}}}}",
        esc(cfg.commit),
        cfg.seconds,
        esc(&cfg.device),
        super::io::HEARTBEAT_EVERY_NS,
        super::driver::TICK_NS,
    )
}

pub(crate) fn poll_source_once<C: Clock, S: InputSource>(
    clock: &C,
    epoch: u64,
    source: &mut S,
    bridge: &mut IoBridge,
    log: &mut IoLog,
) -> (u64, u64) {
    let started = clock.now_ns();
    let mut fresh_now = || clock.now_ns();
    source.poll(started, &mut fresh_now, bridge, log);
    let done = clock.now_ns();
    let needs_resync = source.needs_resync();
    // AP-001: a panel change goes out in the same loop iteration that sees
    // it (<= one I/O period after the click), on a valid record only.
    if let Some(v) = bridge.panel_if_changed(done, needs_resync) {
        let i = v.input;
        log.io(
            done,
            &format!(
                "\"ev\":\"panel\",\"serial\":{},\"action\":{},\"hdg\":{},\"alt\":{},\"spd\":{},\"latency_ns\":{}",
                i.serial,
                i.action.code(),
                i.heading_deg,
                i.altitude_ft,
                i.speed_kt,
                done.saturating_sub(v.changed_at_ns),
            ),
        );
    }
    bridge.heartbeat_if_due(done, needs_resync);
    let source_poll_ns = done.saturating_sub(started);
    for gap in bridge.take_gaps() {
        log.io(
            gap.publication_ns,
            &format!(
                "\"ev\":\"publication_gap\",\"gap_ns\":{},\"previous_ns\":{},\"publication_ns\":{},\"kind\":\"{}\",\"connected\":{},\"needs_resync\":{},\"source_poll_ns\":{}",
                gap.gap_ns,
                gap.previous_ns.saturating_sub(epoch),
                gap.publication_ns.saturating_sub(epoch),
                gap.kind,
                bridge.connected,
                needs_resync,
                source_poll_ns,
            ),
        );
    }
    (started, done)
}

/// Bounded, nonblocking sender for the I/O thread's preformatted lines.
fn send_line(tx: &SyncSender<String>, line: String, totals: &mut IoTotals) {
    if tx.try_send(line).is_err() {
        totals.log_drops += 1;
    }
}

/// The I/O thread: poll, publish, heartbeat. Never writes the log file and
/// never waits on another thread.
pub(crate) fn io_loop<C: Clock, S: InputSource>(
    clock: C,
    epoch: u64,
    mut source: S,
    mut bridge: IoBridge,
    board: Arc<Board>,
    log_tx: SyncSender<String>,
) -> (IoTotals, String) {
    let rt = rt::setup_io_thread(IO_CPU);
    let mut log = IoLog::new(epoch);
    let mut totals = IoTotals::default();
    let mut next = clock.now_ns();
    let mut last_second = next;
    let mut second_frames = 0;
    let mut second_max_loop = 0;
    loop {
        let (now, _) = poll_source_once(&clock, epoch, &mut source, &mut bridge, &mut log);
        board.connected.store(bridge.connected, Ordering::Relaxed);
        board.frames.store(bridge.frames, Ordering::Relaxed);
        board.button_presses.store(bridge.button_presses, Ordering::Relaxed);
        board.last_buttons.store(bridge.last_buttons_pressed, Ordering::Relaxed);
        board.parking_brake.store(bridge.parking_brake().unwrap_or(false), Ordering::Relaxed);
        board.camera_presses.store(bridge.camera_presses, Ordering::Relaxed);
        let stopping = board.stop_io.load(Ordering::Relaxed);
        for line in log.lines.drain(..) {
            send_line(&log_tx, line, &mut totals);
        }
        let done = clock.now_ns();
        second_max_loop = second_max_loop.max(done - now);
        totals.max_loop_ns = totals.max_loop_ns.max(done - now);
        if now - last_second >= 1_000_000_000 {
            let frames = bridge.frames - second_frames;
            second_frames = bridge.frames;
            let gap = std::mem::take(&mut bridge.window_max_gap_ns);
            let line = format!(
                "{{\"k\":\"io_sec\",\"t\":{},\"frames\":{frames},\"publications\":{},\"max_pub_gap_ns\":{gap},\"max_loop_ns\":{second_max_loop},\"connected\":{}}}",
                now - epoch,
                bridge.publications,
                bridge.connected
            );
            send_line(&log_tx, line, &mut totals);
            second_max_loop = 0;
            last_second = now;
        }
        if stopping {
            let final_now = clock.now_ns();
            let line = format!(
                "{{\"k\":\"io_final\",\"t\":{},\"frames\":{},\"publications\":{},\"max_pub_gap_ns\":{},\"max_loop_ns\":{},\"connected\":{}}}",
                final_now.saturating_sub(epoch),
                bridge.frames - second_frames,
                bridge.publications,
                bridge.window_max_gap_ns,
                second_max_loop,
                bridge.connected,
            );
            send_line(&log_tx, line, &mut totals);
            break;
        }
        next = next_io_deadline(next, clock.now_ns());
        clock.sleep_until(next);
    }
    totals.publications = bridge.publications;
    totals.max_gap_ns = bridge.max_gap_ns;
    totals.frames = bridge.frames;
    totals.button_presses = bridge.button_presses;
    (totals, rt)
}

/// Next absolute I/O poll deadline after one ending at `previous`, given
/// the time `now` just before sleeping. The 1 ms grid is kept while on time;
/// after an overrun the next deadline is a full period after `now`, never in
/// the past, so the (possibly `SCHED_FIFO`) I/O thread blocks every iteration
/// and can never spin.
fn next_io_deadline(previous: u64, now: u64) -> u64 {
    let next = previous + IO_POLL_NS;
    if next <= now {
        now + IO_POLL_NS
    } else {
        next
    }
}

/// Log writer period. Channel capacities cover far more than this at every
/// producer's rate; the writer has no latency requirement of its own.
const LOG_PERIOD_NS: u64 = 5_000_000;

/// The log thread: the only writer of the JSONL file. Drains every channel
/// until `stop` is set and the channels are empty; returns the writer (the
/// caller appends the summary) and the number of lines written.
#[allow(clippy::too_many_arguments)]
pub(crate) fn log_loop<C: Clock, W: Write>(
    clock: C,
    epoch: u64,
    header: String,
    out: W,
    stop: Arc<AtomicBool>,
    io_rx: Receiver<String>,
    main_rx: Receiver<String>,
    sim_rx: Receiver<Rec>,
    read_rx: Receiver<Rec>,
) -> io::Result<(io::BufWriter<W>, u64, String)> {
    let rt = rt::setup_log_thread();
    let mut out = io::BufWriter::with_capacity(1 << 16, out);
    writeln!(out, "{header}")?;
    let mut lines = 0u64;
    let mut last_flush = clock.now_ns();
    loop {
        // Read the flag first: everything sent before it was set is drained
        // by this final pass.
        let stopping = stop.load(Ordering::Acquire);
        for line in io_rx.try_iter().chain(main_rx.try_iter()) {
            lines += 1;
            writeln!(out, "{line}")?;
        }
        for r in sim_rx.try_iter().chain(read_rx.try_iter()) {
            lines += 1;
            writeln!(out, "{}", rec::to_json(&r, epoch))?;
        }
        if stopping {
            break;
        }
        let now = clock.now_ns();
        if now - last_flush >= 1_000_000_000 {
            out.flush()?;
            last_flush = now;
        }
        clock.sleep_until(now + LOG_PERIOD_NS);
    }
    Ok((out, lines, rt))
}

/// The final `summary` line (shared by the threaded run and the synthetic
/// session).
pub fn summary_line(
    duration_ns: u64,
    sim: &SimResult,
    reader: &ReaderTotals,
    io: &IoTotals,
    rts: [&str; 5],
) -> String {
    let t = &sim.totals;
    let p = &sim.publish;
    let m = &sim.mailbox;
    format!(
        "{{\"k\":\"summary\",\"t\":{duration_ns},\"duration_ns\":{duration_ns},\
\"sim\":{{\"ticks\":{},\"misses\":{},\"skipped\":{},\"late_max_ns\":{},\"work_max_ns\":{},\"late_hist_le_us\":{},\"late_hist\":{},\"faults\":{},\"latency_samples\":{}}},\
\"publish\":{{\"offered\":{},\"published\":{},\"busy_events\":{},\"superseded\":{},\"max_consecutive_busy\":{},\"pending\":{},\"busy_runs\":{},\"busy_run_le_ticks\":{},\"busy_run_hist\":{}}},\
\"mailbox\":{{\"reads\":{},\"new_publications\":{},\"skipped_publications\":{},\"commands_delivered\":{},\"coalesced_edges\":{},\"rejected_starts\":{},\"stale_reads\":{},\"unavailable_reads\":{},\"no_input_reads\":{}}},\
\"starts\":{{\"refused_mailbox\":{},\"refused_core_stick\":{},\"refused_core_console\":{},\"accepted\":{}}},\
\"reader\":{{\"reads\":{},\"none\":{},\"age_max_running_ns\":{},\"copy_max_ns\":{}}},\
\"io\":{{\"publications\":{},\"max_pub_gap_ns\":{},\"max_loop_ns\":{},\"frames\":{},\"button_presses\":{},\"lines\":{}}},\
\"drops\":{{\"sim\":{},\"reader\":{},\"io\":{}}},\
\"rt\":{{\"sim\":{},\"io\":{},\"reader\":{},\"main\":{},\"log\":{}}}}}",
        t.ticks, t.misses, t.skipped, t.late_max_ns, t.work_max_ns, list(&LATE_BUCKETS_US), list(&t.late_hist), t.faults, t.latency_samples,
        p.offered, p.published, p.busy_events, p.superseded, p.max_consecutive_busy, p.pending, t.busy_runs, list(&BUSY_RUN_BUCKETS), list(&t.busy_run_hist),
        m.reads, m.new_publications, m.skipped_publications, m.commands_delivered, m.coalesced_edges, m.rejected_starts, m.stale_reads, m.unavailable_reads, m.no_input_reads,
        t.refused_mailbox, t.refused_core_stick, t.refused_core_console, t.accepted_starts,
        reader.reads, reader.none, reader.age_max_running_ns, reader.copy_max_ns,
        io.publications, io.max_gap_ns, io.max_loop_ns, io.frames, io.button_presses, io.lines,
        t.drops, reader.drops, io.log_drops,
        rts[0], rts[1], rts[2], rts[3], rts[4],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s1::io::InputSource;
    use gx_input::ControlFrame;
    use gx_sim::{control_mailbox, ControlStatus, UnavailableReason};

    #[derive(Clone)]
    struct FakeClock(Arc<AtomicU64>);

    impl FakeClock {
        fn new(now: u64) -> Self {
            Self(Arc::new(AtomicU64::new(now)))
        }

        fn advance(&self, ns: u64) {
            self.0.fetch_add(ns, Ordering::Relaxed);
        }
    }

    impl Clock for FakeClock {
        fn now_ns(&self) -> u64 {
            self.0.load(Ordering::Relaxed)
        }

        fn sleep_until(&self, deadline_ns: u64) {
            self.0.fetch_max(deadline_ns, Ordering::Relaxed);
        }
    }

    struct SlowDisconnect {
        clock: FakeClock,
        calls: u8,
    }

    struct SlowConnect {
        clock: FakeClock,
        calls: u8,
    }

    impl InputSource for SlowConnect {
        fn poll(
            &mut self,
            now_ns: u64,
            fresh_now: &mut dyn FnMut() -> u64,
            bridge: &mut IoBridge,
            _log: &mut IoLog,
        ) {
            if self.calls == 0 {
                bridge.disconnected(fresh_now());
            } else {
                self.clock.advance(15_000_000);
                bridge.connected();
                bridge.frame(
                    &ControlFrame { timestamp_ns: now_ns, sequence: 2, ..ControlFrame::default() },
                    fresh_now(),
                );
            }
            self.calls += 1;
        }

        fn needs_resync(&self) -> bool {
            false
        }
    }

    impl InputSource for SlowDisconnect {
        fn poll(
            &mut self,
            now_ns: u64,
            fresh_now: &mut dyn FnMut() -> u64,
            bridge: &mut IoBridge,
            _log: &mut IoLog,
        ) {
            if self.calls == 0 {
                bridge.connected();
                bridge.frame(
                    &ControlFrame { timestamp_ns: now_ns, sequence: 1, ..ControlFrame::default() },
                    fresh_now(),
                );
            } else {
                self.clock.advance(20_000_000);
                bridge.disconnected(fresh_now());
            }
            self.calls += 1;
        }

        fn needs_resync(&self) -> bool {
            false
        }
    }

    #[test]
    fn io_deadline_is_always_in_the_future_so_a_fifo_io_thread_sleeps() {
        // On time: keep the 1 ms grid.
        assert_eq!(next_io_deadline(10_000_000, 10_400_000), 11_000_000);
        // Overrun (loop ended past the next grid point): a full period from now.
        assert_eq!(next_io_deadline(10_000_000, 11_000_000), 12_000_000);
        assert_eq!(next_io_deadline(10_000_000, 25_300_000), 26_300_000);
        for now in [0u64, 999_999, 1_000_000, 7_654_321, 50_000_000] {
            assert!(next_io_deadline(0, now) > now);
        }
    }

    #[test]
    fn slow_poll_uses_physical_publication_time_and_attributes_fail_closed_gap() {
        let epoch = 1_000_000_000;
        let clock = FakeClock::new(epoch);
        let (writer, mut reader) = control_mailbox();
        let mut bridge = IoBridge::new(InputPublisher::new(writer));
        let mut source = SlowDisconnect { clock: clock.clone(), calls: 0 };
        let mut log = IoLog::new(epoch);

        poll_source_once(&clock, epoch, &mut source, &mut bridge, &mut log);
        poll_source_once(&clock, epoch, &mut source, &mut bridge, &mut log);

        let record = reader.read(clock.now_ns());
        assert_eq!(record.status, ControlStatus::Unavailable(UnavailableReason::Disconnected));
        assert_eq!(bridge.max_gap_ns, 20_000_000);
        let gap = log.lines.iter().find(|line| line.contains("publication_gap")).unwrap();
        assert!(gap.contains("\"gap_ns\":20000000"), "{gap}");
        assert!(gap.contains("\"kind\":\"disconnected\""), "{gap}");
        assert!(gap.contains("\"connected\":false"), "{gap}");
        assert!(gap.contains("\"source_poll_ns\":20000000"), "{gap}");
        assert!(gap.contains("\"publication_ns\":20000000"), "{gap}");
    }

    #[test]
    fn slow_connect_gap_is_attributed_to_the_frame_that_restores_valid_input() {
        let epoch = 2_000_000_000;
        let clock = FakeClock::new(epoch);
        let (writer, mut reader) = control_mailbox();
        let mut bridge = IoBridge::new(InputPublisher::new(writer));
        let mut source = SlowConnect { clock: clock.clone(), calls: 0 };
        let mut log = IoLog::new(epoch);

        poll_source_once(&clock, epoch, &mut source, &mut bridge, &mut log);
        poll_source_once(&clock, epoch, &mut source, &mut bridge, &mut log);

        assert_eq!(reader.read(clock.now_ns()).status, ControlStatus::Valid);
        let gap = log.lines.iter().find(|line| line.contains("publication_gap")).unwrap();
        assert!(gap.contains("\"gap_ns\":15000000"), "{gap}");
        assert!(gap.contains("\"kind\":\"frame\""), "{gap}");
        assert!(gap.contains("\"connected\":true"), "{gap}");
        assert!(gap.contains("\"source_poll_ns\":15000000"), "{gap}");
    }

    #[test]
    fn board_cursor_starts_hidden_and_round_trips_in_virtual_pixels() {
        let board = Board::default();
        assert_eq!((board.cursor(), board.mouse_ok()), (None, false));
        for c in [(0.0, 0.0), (1279.0, 719.0), (640.25, 359.5), (12.015625, 700.984375)] {
            board.set_cursor(Some(c));
            assert_eq!(board.cursor(), Some(c));
        }
        board.set_cursor(None);
        board.set_mouse_ok(true);
        assert_eq!((board.cursor(), board.mouse_ok()), (None, true));
    }

    /// One valid frame at the first poll, then a held stick (no records).
    struct HeldStick {
        polls: u32,
        resync: bool,
    }

    impl InputSource for HeldStick {
        fn poll(&mut self, now_ns: u64, fresh_now: &mut dyn FnMut() -> u64, bridge: &mut IoBridge, _log: &mut IoLog) {
            if self.polls == 0 {
                bridge.connected();
                bridge.frame(&ControlFrame { timestamp_ns: now_ns, sequence: 1, ..ControlFrame::default() }, fresh_now());
            }
            self.polls += 1;
        }

        fn needs_resync(&self) -> bool {
            self.resync
        }
    }

    /// W3: the I/O loop republishes a valid record within 2 ms of a panel
    /// change, wherever in the 1 ms poll period the click lands, and never
    /// while the source needs resync.
    #[test]
    fn a_panel_change_is_published_valid_within_2_ms_on_the_io_grid() {
        use gx_sim::{ApAction, ApInput};
        let epoch = 3_000_000_000;
        let clock = FakeClock::new(epoch);
        let (writer, mut reader) = control_mailbox();
        let (mut panel, panel_r) = crate::ap_panel(ApInput::DEFAULT);
        let mut bridge = IoBridge::new(InputPublisher::new(writer).with_panel(panel_r));
        let mut source = HeldStick { polls: 0, resync: false };
        let mut log = IoLog::new(epoch);
        let mut next = clock.now_ns();
        let mut iterate = |source: &mut HeldStick, bridge: &mut IoBridge, log: &mut IoLog| {
            clock.sleep_until(next);
            poll_source_once(&clock, epoch, source, bridge, log);
            next = next_io_deadline(next, clock.now_ns());
        };
        iterate(&mut source, &mut bridge, &mut log);
        let mut serial = 0;
        for offset_ns in [1, 250_000, 999_999] {
            serial += 1;
            let changed_at = clock.now_ns() + offset_ns;
            clock.sleep_until(changed_at);
            panel.store(ApInput { serial, action: ApAction::Heading, ..ApInput::DEFAULT }, changed_at);
            iterate(&mut source, &mut bridge, &mut log);
            let t = reader.read(clock.now_ns());
            assert_eq!((t.status, t.controls.ap.serial), (ControlStatus::Valid, serial));
            let latency = t.record.published_at_ns - changed_at;
            assert!(latency <= 2_000_000, "published {latency} ns after the change");
            let line = log.lines.iter().rev().find(|l| l.contains("\"ev\":\"panel\"")).unwrap();
            assert!(line.contains(&format!("\"serial\":{serial},")) && line.contains(&format!("\"latency_ns\":{latency}")), "{line}");
        }
        assert_eq!(bridge.panel_publications, 3);
        // Needs resync: the heartbeat fails closed and the panel never revives it.
        source.resync = true;
        for _ in 0..10 {
            panel.store(ApInput { serial: serial + 1, ..ApInput::DEFAULT }, clock.now_ns());
            iterate(&mut source, &mut bridge, &mut log);
        }
        let t = reader.read(clock.now_ns());
        assert_eq!(t.status, ControlStatus::Unavailable(UnavailableReason::ResyncRequired));
        assert_eq!(bridge.panel_publications, 3);
    }
}

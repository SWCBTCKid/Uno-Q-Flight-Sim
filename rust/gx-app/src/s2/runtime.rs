//! The S2 thread layout: S1's (ARCH-004, headless) with any
//! `SimulationBackend` (JSBSim on the board), the record trace, flight
//! status for the prompts and an RSS sample per second.
//!
//! Threads and scheduling are S1's unchanged: I/O `SCHED_FIFO` 40 on CPU 1,
//! sim `SCHED_FIFO` 50 on CPU 2 (BOARD-001), reader on CPU 3, log thread
//! explicitly `SCHED_OTHER`, prompter/stdin nice +10. The backend is built
//! (JSBSim loaded) by the caller on the unprivileged main thread and moved
//! into the sim thread, which then owns it exclusively.
//!
//! [`run_with`] is the same layout with an injected 30 Hz snapshot consumer
//! in place of the headless reader: S3 passes its render thread. Nothing on
//! the simulation, trace or shutdown path differs.
//!
//! [`run_with_ap`] (AP-001, S3/S4) adds the autopilot panel: a `gx-mouse`
//! thread (`SCHED_OTHER`, CPU 0; `crate::s3::mouse`) writes the panel
//! seqlock that every I/O publication carries, and the runway approaches go
//! to the sim thread. Every run carries the panel; without a mouse it never
//! changes from `ApInput::DEFAULT`.

use super::phases::{Obs, Phase, Prompter, BUTTONS_HELP};
use super::setup::{Aircraft, Threshold};
use crate::s1::driver::{Cadence, SimDriver};
use crate::s1::io::{InputSource, IoBridge};
use crate::s1::rec::{esc, Rec};
use crate::s1::rt::{self, Clock};
use crate::s1::runtime::{
    io_loop, log_channel, log_loop, reader_loop, summary_line, Board, SimResult, IO_POLL_NS, READER_HZ,
};
use crate::{InputPublisher, StickLevers, StickParkingBrake};
use gx_sim::{control_mailbox, FixedRate, InputStatus, RunState, SimulationBackend, SnapshotExchange, SIM_HZ};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread;

pub const SCHEMA: &str = "gx-s2/1";
const SIM_CPU: usize = 2;

pub struct S2Config {
    pub seconds: u64,
    pub prompts: bool,
    pub device: String,
    pub commit: &'static str,
    pub aircraft: Aircraft,
    pub backend: &'static str,
    pub threshold: Threshold,
    /// Issue one console Start as soon as input is valid (unattended smoke).
    pub autostart: bool,
}

/// What differs between the S2 run and a run with an injected snapshot
/// consumer (S3). `Default` is exactly S2.
pub struct Extras {
    /// `schema` of the start record.
    pub schema: &'static str,
    /// S3 prompt script ([`Prompter::free_flight`]).
    pub free_flight: bool,
    /// Skip the scripted sign/disconnect exercises and fly for the full run.
    pub direct_free_flight: bool,
    /// Ends the run like the time limit when set (SIGTERM/SIGINT handler).
    pub stop: Option<&'static AtomicBool>,
    /// Name of the step in the first prompt (default: S2 / S3 by script).
    pub title: Option<&'static str>,
    /// Extra sentence appended to the first prompt (S4: the camera button).
    pub hint: Option<&'static str>,
}

impl Default for Extras {
    fn default() -> Self {
        Self { schema: SCHEMA, free_flight: false, direct_free_flight: false, stop: None, title: None, hint: None }
    }
}

/// The latest operator prompts, for a consumer that shows them (S3 draws
/// them on screen). The main thread writes; the consumer only `try_lock`s.
#[derive(Default)]
pub struct PromptSlot {
    inner: Mutex<(u64, Vec<String>)>,
}

impl PromptSlot {
    /// Keep the newest `keep` lines.
    pub fn push(&self, line: &str, keep: usize) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.0 += 1;
        g.1.push(line.to_owned());
        let n = g.1.len();
        if n > keep {
            g.1.drain(..n - keep);
        }
    }

    /// The lines if they changed since `seen`; never blocks.
    pub fn try_get_newer(&self, seen: u64) -> Option<(u64, Vec<String>)> {
        let g = self.inner.try_lock().ok()?;
        (g.0 != seen).then(|| (g.0, g.1.clone()))
    }
}

/// Nonblocking text-record sender into the log (one JSON object per line).
pub struct TextTx {
    tx: SyncSender<String>,
    pub dropped: u64,
}

impl TextTx {
    pub fn new(tx: SyncSender<String>) -> Self {
        Self { tx, dropped: 0 }
    }

    pub fn send(&mut self, line: String) {
        match self.tx.try_send(line) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => self.dropped += 1,
        }
    }
}

/// What the injected snapshot consumer gets; it runs on its own thread.
pub struct SnapshotCtx<C> {
    pub clock: C,
    pub epoch: u64,
    pub exchange: Arc<SnapshotExchange>,
    pub board: Arc<Board>,
    pub log: crate::s1::runtime::LogTx,
    pub text: TextTx,
    pub prompt: Arc<PromptSlot>,
}

/// AP-001: what the autopilot panel adds to a run. `Default`: no mouse
/// thread (the panel shows NO MOUSE) and no approaches.
#[derive(Default)]
pub struct ApExtras {
    pub mouse: Option<crate::s3::mouse::MouseSelect>,
    pub approaches: Vec<gx_sim::Approach>,
}

/// Flight values for the prompter (and the panel's value sync), written by
/// the sim thread every tick (relaxed; a slightly stale value is harmless).
#[derive(Default)]
struct Flight {
    ias_kt: AtomicU64,
    gs_kt: AtomicU64,
    agl_ft: AtomicU64,
    vs_fpm: AtomicU64,
    pitch_deg: AtomicU64,
    hdg_deg: AtomicU64,
    n1: AtomicU64,
    wow: AtomicBool,
    generation: AtomicU64,
    controls: [AtomicU64; 4],
    surfaces: [AtomicU64; 3],
}

fn put(a: &AtomicU64, v: f64) {
    a.store(v.to_bits(), Ordering::Relaxed);
}

fn get(a: &AtomicU64) -> f64 {
    f64::from_bits(a.load(Ordering::Relaxed))
}

pub fn start_line(cfg: &S2Config, epoch: u64) -> String {
    start_line_with_schema(cfg, epoch, SCHEMA)
}

pub fn start_line_with_schema(cfg: &S2Config, epoch: u64, schema: &str) -> String {
    let t = &cfg.threshold;
    format!(
        "{{\"k\":\"start\",\"t\":0,\"schema\":\"{schema}\",\"commit\":{},\"seconds\":{},\"device\":{},\"clock\":\"CLOCK_MONOTONIC\",\"epoch_monotonic_ns\":{epoch},\"sim_hz\":{SIM_HZ},\"reader_hz\":{READER_HZ},\"io_poll_ns\":{IO_POLL_NS},\"heartbeat_ns\":{},\"tick_ns\":{},\
\"backend\":\"{}\",\"aircraft\":\"{}\",\"vr_kt\":{},\"autostart\":{},\
\"ic\":{{\"runway\":\"{}\",\"enu_m\":[{},{}],\"heading_deg\":{},\"lat\":{:.9},\"lon\":{:.9},\"surface_msl_m\":{:.4},\"parking_brake\":true}},\
\"buttons\":{{\"start\":\"button 1 / BTN_TRIGGER\",\"pause\":\"button 2 / BTN_THUMB\",\"reset\":\"button 3 / BTN_THUMB2\",\"parking_brake\":\"button 4 / BTN_TOP\",\"trim_nose_down\":\"button 7 / BTN_BASE (held)\",\"trim_nose_up\":\"button 8 / BTN_BASE2 (held)\"}},\"hat\":\"wheel brakes: back both, left left, right right\",\"reset_sets_parking_brake\":true}}",
        esc(cfg.commit),
        cfg.seconds,
        esc(&cfg.device),
        crate::s1::io::HEARTBEAT_EVERY_NS,
        crate::s1::driver::TICK_NS,
        cfg.backend,
        cfg.aircraft.name(),
        cfg.aircraft.vr_kt(),
        cfg.autostart,
        t.runway,
        t.enu_m[0],
        t.enu_m[1],
        t.heading_deg,
        t.latitude_deg,
        t.longitude_deg,
        t.surface_msl_m,
    )
}

/// Resident set size in KiB from `/proc/self/statm` (None off Linux).
fn rss_kib() -> Option<u64> {
    let statm = std::fs::read_to_string("/proc/self/statm").ok()?;
    let pages: u64 = statm.split_whitespace().nth(1)?.parse().ok()?;
    Some(pages * 4)
}

/// Run the S2 layout and write the JSONL log to `out`. Returns the exit code
/// (0 = completed; the checker judges PASS).
pub fn run<C, S, W, B>(
    cfg: S2Config,
    clock: C,
    source: S,
    backend: B,
    out: W,
    stdout: &mut dyn Write,
) -> io::Result<i32>
where
    C: Clock,
    S: InputSource + Send + 'static,
    W: Write + Send + 'static,
    B: SimulationBackend + Send + 'static,
    B::Error: std::fmt::Debug,
{
    run_with(cfg, Extras::default(), clock, source, backend, out, stdout, "gx-reader", |ctx: SnapshotCtx<C>| {
        Ok(reader_loop(ctx.clock, ctx.exchange, ctx.board, ctx.log))
    })
}

/// [`run`] with `consumer` as the 30 Hz snapshot thread (named `thread_name`)
/// instead of the headless reader. The consumer must return when
/// `board.stop_sim` is set; an `Err` ends the run (logged as `fatal`) after
/// an orderly shutdown of every other thread.
#[allow(clippy::too_many_arguments)]
pub fn run_with<C, S, W, B, F>(
    cfg: S2Config,
    extras: Extras,
    clock: C,
    source: S,
    backend: B,
    out: W,
    stdout: &mut dyn Write,
    thread_name: &'static str,
    consumer: F,
) -> io::Result<i32>
where
    C: Clock,
    S: InputSource + Send + 'static,
    W: Write + Send + 'static,
    B: SimulationBackend + Send + 'static,
    B::Error: std::fmt::Debug,
    F: FnOnce(SnapshotCtx<C>) -> io::Result<(crate::s1::runtime::ReaderTotals, String)> + Send + 'static,
{
    run_with_ap(cfg, extras, ApExtras::default(), clock, source, backend, out, stdout, thread_name, consumer)
}

/// [`run_with`] plus the AP-001 autopilot panel ([`ApExtras`]).
#[allow(clippy::too_many_arguments)]
pub fn run_with_ap<C, S, W, B, F>(
    cfg: S2Config,
    extras: Extras,
    ap: ApExtras,
    clock: C,
    source: S,
    backend: B,
    out: W,
    stdout: &mut dyn Write,
    thread_name: &'static str,
    consumer: F,
) -> io::Result<i32>
where
    C: Clock,
    S: InputSource + Send + 'static,
    W: Write + Send + 'static,
    B: SimulationBackend + Send + 'static,
    B::Error: std::fmt::Debug,
    F: FnOnce(SnapshotCtx<C>) -> io::Result<(crate::s1::runtime::ReaderTotals, String)> + Send + 'static,
{
    let epoch = clock.now_ns();
    let board = Arc::new(Board::default());
    board.set_vis(RunState::Stopped, None, InputStatus::NoInput);
    let flight = Arc::new(Flight::default());
    let exchange = Arc::new(SnapshotExchange::new());
    let (writer, reader) = control_mailbox();
    let initial_levers = cfg.aircraft.initial_levers();
    let (sim_tx, sim_rx) = log_channel();
    let (read_tx, read_rx) = log_channel();
    let (main_tx, main_rx) = sync_channel::<String>(4096);
    let (io_tx, io_rx) = sync_channel::<String>(16_384);
    let header = start_line_with_schema(&cfg, epoch, extras.schema);
    let prompt_slot = Arc::new(PromptSlot::default());
    let stop_log = Arc::new(AtomicBool::new(false));
    let (panel_writer, panel_reader) = crate::ap_panel(gx_sim::ApInput::DEFAULT);
    let ApExtras { mouse, approaches } = ap;

    let log_thread = {
        let clock = clock.clone();
        let stop_log = Arc::clone(&stop_log);
        thread::Builder::new().name("gx-log".into()).spawn(move || {
            log_loop(clock, epoch, header, out, stop_log, io_rx, main_rx, sim_rx, read_rx)
        })?
    };

    let io_thread = {
        let board = Arc::clone(&board);
        let clock = clock.clone();
        let flaps: Vec<f32> = cfg.aircraft.flap_detents().iter().map(|d| d.norm).collect();
        let publisher = InputPublisher::with_parking_brake(writer, StickParkingBrake::new(true))
            .with_levers(StickLevers::new(&flaps, cfg.aircraft.takeoff_flap_index()))
            .with_panel(panel_reader);
        board.parking_brake.store(true, Ordering::Relaxed);
        thread::Builder::new().name("gx-io".into()).spawn(move || {
            io_loop(clock, epoch, source, IoBridge::new(publisher), board, io_tx)
        })?
    };

    let sim_thread = {
        let board = Arc::clone(&board);
        let flight = Arc::clone(&flight);
        let clock = clock.clone();
        let exchange = Arc::clone(&exchange);
        let mut log = sim_tx;
        thread::Builder::new().name("gx-sim".into()).spawn(move || -> Result<(SimResult, u64), String> {
            // The initial reset (JSBSim RunIC) runs before the thread turns
            // realtime; stick Resets later run at FIFO 50 like any tick.
            let start = clock.now_ns();
            let driver = SimDriver::try_new_with_levers(
                backend, exchange, reader, start, initial_levers, &mut |r| log.send(r),
            );
            let rt = rt::setup_sim_thread(SIM_CPU);
            let mut driver = match driver {
                Ok(d) => d.with_trace().with_approaches(approaches),
                Err(e) => {
                    board.stop_sim.store(true, Ordering::Relaxed);
                    return Err(format!("initial reset failed: {e:?}"));
                }
            };
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
                if let Some(s) = driver.last_snapshot() {
                    let st = &s.state;
                    let vel = st.local_enu.map_or([0.0; 3], |p| p.velocity_mps);
                    put(&flight.ias_kt, st.calibrated_airspeed_mps / 0.514_444);
                    put(&flight.gs_kt, (vel[0] * vel[0] + vel[1] * vel[1]).sqrt() / 0.514_444);
                    put(&flight.agl_ft, st.height_agl_m / 0.3048);
                    put(&flight.vs_fpm, st.vertical_speed_mps / 0.3048 * 60.0);
                    put(&flight.pitch_deg, st.pitch_rad.to_degrees());
                    put(&flight.hdg_deg, st.heading_rad.to_degrees());
                    put(&flight.n1, st.engine_n1);
                    flight.wow.store(st.weight_on_wheels, Ordering::Relaxed);
                    flight.generation.store(s.generation, Ordering::Relaxed);
                    let c = driver.controls();
                    for (a, v) in flight.controls.iter().zip([c.roll, c.pitch, c.yaw, c.throttle]) {
                        put(a, f64::from(v));
                    }
                    for (a, v) in flight.surfaces.iter().zip([st.elevator_position, st.aileron_position, st.rudder_position]) {
                        put(a, v);
                    }
                }
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
            let traced = driver.traced_ticks();
            Ok((SimResult { totals, publish: driver.publish_stats(), mailbox: driver.mailbox_stats(), rt }, traced))
        })?
    };

    let reader_thread = {
        let ctx = SnapshotCtx {
            clock: clock.clone(),
            epoch,
            exchange: Arc::clone(&exchange),
            board: Arc::clone(&board),
            log: read_tx,
            text: TextTx::new(main_tx.clone()),
            prompt: Arc::clone(&prompt_slot),
        };
        let board = Arc::clone(&board);
        thread::Builder::new().name(thread_name.into()).spawn(move || {
            let r = consumer(ctx);
            // A consumer that fails (or returns early) ends the run.
            board.stop_sim.store(true, Ordering::Relaxed);
            r
        })?
    };

    let mouse_thread = match mouse {
        None => None,
        Some(select) => {
            let f = Arc::clone(&flight);
            let current = move |t: gx_render::panel::Target| match t {
                gx_render::panel::Target::Heading => get(&f.hdg_deg),
                gx_render::panel::Target::Altitude => get(&f.agl_ft),
                gx_render::panel::Target::Speed => get(&f.ias_kt),
            };
            let ctx = crate::s3::mouse::MouseCtx {
                clock: clock.clone(),
                epoch,
                board: Arc::clone(&board),
                panel: panel_writer,
                current: Box::new(current),
                text: TextTx::new(main_tx.clone()),
                stop: extras.stop,
            };
            crate::s3::mouse::spawn(select, ctx)?
        }
    };

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
    let mut prompter = Prompter::new(cfg.aircraft.name(), cfg.aircraft.vr_kt());
    if extras.free_flight {
        prompter = prompter.free_flight();
    }
    if extras.direct_free_flight {
        prompter = prompter.direct_free_flight();
    }
    if let Some(title) = extras.title {
        prompter = prompter.titled(title, extras.hint.unwrap_or(""));
    }
    let mut autostarted = false;
    let mut next_mem = epoch;
    if cfg.prompts {
        writeln!(stdout, "gx-app s2: headless ARCH-004 threads + JSBSim {} ({}), up to {} s, stick {}",
            cfg.aircraft.name(), cfg.backend, cfg.seconds, cfg.device)?;
        writeln!(stdout, "{BUTTONS_HELP}")?;
    }
    loop {
        let now = clock.now_ns();
        let (run, _pause, status) = board.vis();
        if now >= next_mem {
            next_mem = now + 1_000_000_000;
            if let Some(kib) = rss_kib() {
                let _ = main_tx.try_send(format!("{{\"k\":\"mem\",\"t\":{},\"rss_kib\":{kib}}}", now - epoch));
            }
        }
        if cfg.autostart && !autostarted && status == InputStatus::Valid {
            autostarted = true;
            board.console_starts.fetch_add(1, Ordering::Relaxed);
            let _ = main_tx.try_send(format!("{{\"k\":\"autostart\",\"t\":{}}}", now - epoch));
        }
        let obs = Obs {
            t_ns: now - epoch,
            end_ns: end_ns - epoch,
            connected: board.connected.load(Ordering::Relaxed),
            frames: board.frames.load(Ordering::Relaxed),
            run: Some(run),
            status: Some(status),
            parking_brake: board.parking_brake.load(Ordering::Relaxed),
            ias_kt: get(&flight.ias_kt),
            gs_kt: get(&flight.gs_kt),
            agl_ft: get(&flight.agl_ft),
            vs_fpm: get(&flight.vs_fpm),
            pitch_deg: get(&flight.pitch_deg),
            hdg_deg: get(&flight.hdg_deg),
            n1: get(&flight.n1),
            wow: flight.wow.load(Ordering::Relaxed),
            generation: flight.generation.load(Ordering::Relaxed),
            controls: std::array::from_fn(|i| get(&flight.controls[i])),
            surfaces: std::array::from_fn(|i| get(&flight.surfaces[i])),
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
            // Periodic status lines repeat what a display already shows.
            if !text.starts_with("  IAS") {
                prompt_slot.push(text.trim(), 2);
            }
        }
        if let Some(stop) = extras.stop {
            if stop.load(Ordering::Relaxed) {
                let _ = main_tx.try_send(format!("{{\"k\":\"stop_signal\",\"t\":{}}}", now - epoch));
                break;
            }
        }
        if prompter.phase == Phase::Done || board.stop_sim.load(Ordering::Relaxed) {
            break;
        }
        clock.sleep_until(now + 100_000_000);
    }

    board.stop_sim.store(true, Ordering::Relaxed);
    let sim = sim_thread.join().map_err(|_| io::Error::other("sim thread panicked"))?;
    let reader_result = reader_thread.join().map_err(|_| io::Error::other("snapshot thread panicked"))?;
    if let Some(mouse) = mouse_thread {
        mouse.join().map_err(|_| io::Error::other("mouse thread panicked"))?;
    }
    board.stop_io.store(true, Ordering::Relaxed);
    let (mut io, io_rt) = io_thread.join().map_err(|_| io::Error::other("io thread panicked"))?;
    stop_log.store(true, Ordering::Release);
    let (mut out, lines, log_rt) = log_thread.join().map_err(|_| io::Error::other("log thread panicked"))??;
    io.lines = lines;
    let (reader_totals, reader_rt) = match reader_result {
        Ok(v) => v,
        Err(e) => {
            writeln!(out, "{{\"k\":\"fatal\",\"t\":{},\"what\":{}}}", clock.now_ns() - epoch, esc(&format!("{thread_name}: {e}")))?;
            out.flush()?;
            return Err(e);
        }
    };
    let (sim, traced) = match sim {
        Ok(v) => v,
        Err(e) => {
            writeln!(out, "{{\"k\":\"fatal\",\"t\":{},\"what\":{}}}", clock.now_ns() - epoch, esc(&e))?;
            out.flush()?;
            return Err(io::Error::other(e));
        }
    };

    let duration_ns = clock.now_ns() - epoch;
    let rts = [sim.rt.as_str(), io_rt.as_str(), reader_rt.as_str(), main_rt.as_str(), log_rt.as_str()];
    writeln!(out, "{}", summary_line(duration_ns, &sim, &reader_totals, &io, rts))?;
    writeln!(out, "{{\"k\":\"s2_summary\",\"t\":{duration_ns},\"tick_records\":{traced},\"final_phase\":\"{}\"}}", prompter.phase.name())?;
    writeln!(out, "{{\"k\":\"end\",\"t\":{duration_ns}}}")?;
    out.flush()?;
    if cfg.prompts {
        let t = &sim.totals;
        writeln!(
            stdout,
            "summary: ticks={} deadline_misses={} skipped={} late_max={:.3} ms work_max={:.3} ms tick_records={} log_drops={}",
            t.ticks, t.misses, t.skipped, t.late_max_ns as f64 / 1e6, t.work_max_ns as f64 / 1e6, traced,
            t.drops + reader_totals.drops
        )?;
    }
    Ok(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::s1::rt::MonotonicClock;

    #[derive(Clone, Default)]
    struct Sink(Arc<Mutex<Vec<u8>>>);

    impl Write for Sink {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// AP-002 W7: the stop flag a confirmed EXIT sets (the SIGTERM flag)
    /// ends a long run early through the ordinary shutdown: every thread
    /// joined, the summary and end records written, exit code 0 - the same
    /// tail as a run that reaches its time limit.
    #[test]
    fn the_exit_stop_flag_ends_a_long_run_like_the_time_limit() {
        static STOP: AtomicBool = AtomicBool::new(false);
        let frame = crate::s1::synthetic_world_frame().unwrap();
        let cfg = S2Config {
            seconds: 10_800,
            prompts: false,
            device: "synthetic:neutral".into(),
            commit: "test",
            aircraft: super::super::setup::Aircraft::Global5000,
            backend: "fake",
            threshold: super::super::setup::threshold(&frame).unwrap(),
            autostart: true,
        };
        let extras = Extras { free_flight: true, stop: Some(&STOP), ..Extras::default() };
        let sink = Sink::default();
        let setter = std::thread::spawn(|| {
            std::thread::sleep(std::time::Duration::from_millis(1_500));
            STOP.store(true, Ordering::Relaxed);
        });
        let t0 = std::time::Instant::now();
        let code = run_with_ap(
            cfg, extras, ApExtras::default(), MonotonicClock, crate::s2::NeutralStick::default(),
            crate::FakeBackend::new(frame), sink.clone(), &mut Vec::new(), "gx-reader",
            |ctx: SnapshotCtx<MonotonicClock>| Ok(reader_loop(ctx.clock, ctx.exchange, ctx.board, ctx.log)),
        )
        .unwrap();
        setter.join().unwrap();
        assert_eq!(code, 0);
        assert!(t0.elapsed().as_secs() < 10, "ended by the flag, not the 3 h backstop");
        let log = String::from_utf8(sink.0.lock().unwrap().clone()).unwrap();
        let kinds: Vec<&str> = log.lines().map(|l| l.split('"').nth(3).unwrap_or("")).collect();
        assert!(kinds.contains(&"stop_signal"), "{kinds:?}");
        assert_eq!(&kinds[kinds.len() - 3..], ["summary", "s2_summary", "end"]);
        assert!(log.contains("\"k\":\"io_final\""), "the I/O thread finished");
    }
}

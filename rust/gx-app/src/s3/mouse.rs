//! AP-001 W3: the mouse side of the autopilot panel (plan
//! the AP-002 mouse-panel interface).
//!
//! A `SCHED_OTHER` thread pinned to CPU 0 (off the realtime CPUs: I/O FIFO
//! 40 on 1, sim FIFO 50 on 2; render on 3) owns the evdev mouse
//! (`gx_input::mouse`). It moves the cursor in the virtual 1280x720 HUD
//! space, hit-tests clicks against `gx_render::panel` and writes the
//! resulting [`ApInput`] into the [`ApPanelWriter`] seqlock; the I/O thread
//! publishes it. It also publishes the cursor and `mouse_ok` on the
//! [`Board`] for the render thread.
//!
//! Mapping ([`PanelMouse`]):
//! * `Hit::Action(a)`: `serial += 1`, `action = a` (the core applies it once);
//! * `Hit::Step(t, d)`: the target moves `d` coarse steps (`Target::steps().0`);
//! * `Hit::Value(t)`: a click syncs the target to the current heading, AGL or
//!   IAS (rounded to the fine step); the wheel over it moves fine steps.
//!
//! Targets are then wrapped/clamped by `ApInput::normalized`. A click on a
//! hidden cursor (no activity for 10 s) only reveals it.
//!
//! **EXIT** (AP-002 W7, [`ExitLatch`]): the first click arms it for
//! [`EXIT_WINDOW_NS`] (`Board::exit_armed`, the panel shows `EXIT?`); a
//! second EXIT click inside that window ends the run through the same stop
//! flag as SIGTERM, i.e. exactly like the time limit (clean shutdown, logs
//! complete, the controller restores the desktop). The window running out,
//! or a click on any other button, disarms it.
//!
//! No mouse (none found, open refused, unplugged) is not an error: the run
//! goes on, `mouse_ok` is false (the panel says NO MOUSE) and the thread
//! retries once a second.

use crate::s1::rt::{self, Clock};
use crate::s1::runtime::Board;
use crate::s1::rec::esc;
use crate::s2::runtime::TextTx;
use crate::ApPanelWriter;
use gx_input::mouse::{MouseReport, Pointer, DEFAULT_SENSITIVITY};
use gx_render::panel::{hit, Hit, Target};
use gx_render::scene::{HEIGHT, WIDTH};
use gx_sim::ApInput;
use std::io;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// The mouse thread's CPU: not a realtime one (see the module docs).
pub const MOUSE_CPU: usize = 0;
/// Longest wait for mouse records; also the cursor-hide resolution and the
/// stop latency.
pub const MOUSE_WAIT_MS: i32 = 100;
pub const REOPEN_NS: u64 = 1_000_000_000;

/// `--mouse`: an explicit node, or the first `/dev/input/by-id/*-event-mouse`
/// at every (re)open.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MouseSelect {
    Auto,
    Path(PathBuf),
}

impl MouseSelect {
    pub fn resolve(&self) -> Option<PathBuf> {
        match self {
            Self::Auto => gx_input::mouse::find_mouse(std::path::Path::new(gx_input::mouse::DEFAULT_BY_ID)),
            Self::Path(p) => Some(p.clone()),
        }
    }

    pub fn describe(&self) -> String {
        match self {
            Self::Auto => format!("auto:{}/*-event-mouse", gx_input::mouse::DEFAULT_BY_ID),
            Self::Path(p) => p.to_string_lossy().into_owned(),
        }
    }
}

/// The current heading (deg true), AGL (ft) and IAS (kt) for a value sync.
pub type Current = dyn Fn(Target) -> f64 + Send;

/// How long a first EXIT click waits for the confirming one.
pub const EXIT_WINDOW_NS: u64 = 3_000_000_000;

/// EXIT's arm/confirm state. Pure; host-tested.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ExitLatch {
    armed_at_ns: Option<u64>,
    confirmed: bool,
}

impl ExitLatch {
    /// An EXIT click: arms, or confirms if armed and inside the window.
    /// Returns true on the confirming click (once).
    pub fn press(&mut self, now_ns: u64) -> bool {
        if self.confirmed {
            return false;
        }
        if self.armed(now_ns) {
            self.armed_at_ns = None;
            self.confirmed = true;
            true
        } else {
            self.armed_at_ns = Some(now_ns);
            false
        }
    }

    /// A click anywhere else on the panel.
    pub fn disarm(&mut self) {
        self.armed_at_ns = None;
    }

    /// Armed and still inside the window at `now_ns`.
    pub fn armed(&self, now_ns: u64) -> bool {
        self.armed_at_ns.is_some_and(|t| now_ns.saturating_sub(t) < EXIT_WINDOW_NS)
    }

    pub fn confirmed(&self) -> bool {
        self.confirmed
    }
}

/// Panel state on the mouse thread. Pure; host-tested.
#[derive(Clone, Debug)]
pub struct PanelMouse {
    pointer: Pointer,
    ap: ApInput,
    /// The panel layout (`gx_render::panel::hit`; tests substitute one).
    hit: fn(f32, f32) -> Option<Hit>,
    pub exit: ExitLatch,
    pub reports: u64,
    /// Clicks that hit a panel button.
    pub clicks: u64,
    pub changes: u64,
}

fn get(ap: &ApInput, t: Target) -> f32 {
    match t {
        Target::Heading => ap.heading_deg,
        Target::Altitude => ap.altitude_ft,
        Target::Speed => ap.speed_kt,
    }
}

fn set(ap: &mut ApInput, t: Target, v: f32) {
    match t {
        Target::Heading => ap.heading_deg = v,
        Target::Altitude => ap.altitude_ft = v,
        Target::Speed => ap.speed_kt = v,
    }
}

/// The panel after a click on `hit`.
pub fn click(ap: ApInput, hit: Hit, current: &Current) -> ApInput {
    let mut ap = ap;
    match hit {
        Hit::Action(gx_sim::ApAction::None) => {}
        Hit::Action(action) => {
            ap.serial = ap.serial.wrapping_add(1);
            ap.action = action;
        }
        Hit::Step(t, d) => {
            let v = get(&ap, t) + f32::from(d) * t.steps().0;
            set(&mut ap, t, v);
        }
        Hit::Value(t) => {
            let now = current(t);
            if now.is_finite() {
                let fine = f64::from(t.steps().1);
                set(&mut ap, t, ((now / fine).round() * fine) as f32);
            }
        }
        // EXIT is not an autopilot input: `PanelMouse::press` handles it.
        Hit::Exit => {}
    }
    ap.normalized()
}

/// The panel after `detents` wheel detents (+ = up) over `hit`: only a
/// value box responds, by fine steps.
pub fn wheel(ap: ApInput, hit: Hit, detents: i32) -> ApInput {
    let mut ap = ap;
    if let Hit::Value(t) = hit {
        let v = get(&ap, t) + detents as f32 * t.steps().1;
        set(&mut ap, t, v);
    }
    ap.normalized()
}

impl PanelMouse {
    pub fn new(initial: ApInput) -> Self {
        Self {
            pointer: Pointer::new(WIDTH, HEIGHT, DEFAULT_SENSITIVITY),
            ap: initial.normalized(),
            hit,
            exit: ExitLatch::default(),
            reports: 0,
            clicks: 0,
            changes: 0,
        }
    }

    /// The same with another layout (tests).
    pub fn with_hit(mut self, hit: fn(f32, f32) -> Option<Hit>) -> Self {
        self.hit = hit;
        self
    }

    /// A click on `h` at `now_ns`: EXIT arms/confirms, any other button
    /// disarms EXIT and goes to [`click`]. True if the panel changed.
    pub fn press(&mut self, h: Hit, now_ns: u64, current: &Current) -> bool {
        self.clicks += 1;
        if h == Hit::Exit {
            self.exit.press(now_ns);
            return false;
        }
        self.exit.disarm();
        let ap = click(self.ap, h, current);
        let changed = ap != self.ap;
        self.ap = ap;
        self.changes += u64::from(changed);
        changed
    }

    pub fn exit_armed(&self, now_ns: u64) -> bool {
        self.exit.armed(now_ns)
    }

    pub fn ap(&self) -> ApInput {
        self.ap
    }

    pub fn cursor(&self, now_ns: u64) -> Option<(f32, f32)> {
        self.pointer.position(now_ns)
    }

    /// Apply one report; true if the panel changed (store it then).
    pub fn report(&mut self, r: &MouseReport, now_ns: u64, current: &Current) -> bool {
        self.reports += 1;
        let Some((x, y)) = self.pointer.apply(r, now_ns) else { return false };
        let Some(h) = (self.hit)(x, y) else { return false };
        let mut changed = false;
        if r.wheel != 0 {
            let ap = wheel(self.ap, h, r.wheel);
            changed = ap != self.ap;
            self.ap = ap;
            self.changes += u64::from(changed);
        }
        if r.left_pressed {
            changed |= self.press(h, now_ns, current);
        }
        changed
    }
}

/// An open mouse.
pub trait MouseDevice {
    /// Wait up to `timeout_ms`, then deliver every complete report. `Err`:
    /// the device is gone; it is dropped and reopened.
    fn wait_and_drain(&mut self, timeout_ms: i32, report: &mut dyn FnMut(MouseReport)) -> io::Result<()>;
    /// The node opened, for the log.
    fn node(&self) -> String;
}

#[cfg(target_os = "linux")]
impl MouseDevice for gx_input::mouse::linux::MouseReader {
    fn wait_and_drain(&mut self, timeout_ms: i32, report: &mut dyn FnMut(MouseReport)) -> io::Result<()> {
        gx_input::mouse::linux::MouseReader::wait_and_drain(self, timeout_ms, report)
    }

    fn node(&self) -> String {
        self.node.to_string_lossy().into_owned()
    }
}

/// The evdev opener for [`mouse_loop`].
#[cfg(target_os = "linux")]
pub fn open_evdev(path: &std::path::Path) -> io::Result<gx_input::mouse::linux::MouseReader> {
    gx_input::mouse::linux::MouseReader::open(path)
}

/// Start the `gx-mouse` thread (evdev; Linux only). Elsewhere there is no
/// mouse: `None`, and `mouse_ok` stays false.
pub fn spawn<C: Clock>(select: MouseSelect, ctx: MouseCtx<C>) -> io::Result<Option<std::thread::JoinHandle<String>>> {
    #[cfg(target_os = "linux")]
    {
        std::thread::Builder::new().name("gx-mouse".into()).spawn(move || mouse_loop(select, open_evdev, ctx)).map(Some)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (select, ctx);
        Ok(None)
    }
}

/// `SCHED_OTHER` nice 0 on [`MOUSE_CPU`] (all unprivileged), whatever
/// spawned it. JSON of what took.
pub fn setup_mouse_thread() -> String {
    let outcome = |r: io::Result<()>| match r {
        Ok(()) => "ok".to_owned(),
        Err(e) => e.raw_os_error().map_or_else(|| e.to_string(), |c| format!("errno{c}")),
    };
    format!(
        "{{\"policy\":\"other\",\"cpu\":{MOUSE_CPU},\"sched_other\":\"{}\",\"affinity\":\"{}\"}}",
        outcome(rt::set_sched_other()),
        outcome(rt::set_thread_affinity(MOUSE_CPU)),
    )
}

/// Everything the mouse thread owns.
pub struct MouseCtx<C> {
    pub clock: C,
    pub epoch: u64,
    pub board: Arc<Board>,
    pub panel: ApPanelWriter,
    pub current: Box<Current>,
    pub text: TextTx,
    /// The run's stop flag (S3/S4: `s3::STOP`, the SIGTERM flag) that a
    /// confirmed EXIT sets; `None` ends the run through `board.stop_sim`.
    pub stop: Option<&'static AtomicBool>,
}

/// The mouse thread body: (re)open, read, map, publish, until
/// `board.stop_sim`. Logs `{"k":"mouse",...}` lines (open, failures when
/// they change, close, and a final count line).
pub fn mouse_loop<C, D, O>(select: MouseSelect, open: O, ctx: MouseCtx<C>) -> String
where
    C: Clock,
    D: MouseDevice,
    O: FnMut(&std::path::Path) -> io::Result<D>,
{
    let initial = ctx.panel.value().input;
    mouse_loop_with(select, open, ctx, PanelMouse::new(initial))
}

/// [`mouse_loop`] with a given [`PanelMouse`] (tests: another layout).
pub fn mouse_loop_with<C, D, O>(select: MouseSelect, mut open: O, ctx: MouseCtx<C>, mut mouse: PanelMouse) -> String
where
    C: Clock,
    D: MouseDevice,
    O: FnMut(&std::path::Path) -> io::Result<D>,
{
    let rt = setup_mouse_thread();
    let MouseCtx { clock, epoch, board, mut panel, current, mut text, stop } = ctx;
    let mut exit_sent = false;
    let line = |text: &mut TextTx, now: u64, body: String| {
        text.send(format!("{{\"k\":\"mouse\",\"t\":{},{body}}}", now.saturating_sub(epoch)));
    };
    line(&mut text, clock.now_ns(), format!("\"ev\":\"start\",\"select\":{},\"rt\":{rt}", esc(&select.describe())));
    let mut device: Option<D> = None;
    let mut next_open = 0u64;
    let mut last_failure = String::new();
    let (mut opens, mut closes) = (0u64, 0u64);
    board.set_mouse_ok(false);
    board.set_cursor(None);
    while !board.stop_sim.load(Ordering::Relaxed) {
        let now = clock.now_ns();
        match device.as_mut() {
            None => {
                if now >= next_open {
                    next_open = now + REOPEN_NS;
                    let failure = match select.resolve() {
                        None => "no mouse found".to_owned(),
                        Some(path) => match open(&path) {
                            Ok(d) => {
                                opens += 1;
                                last_failure.clear();
                                line(&mut text, clock.now_ns(), format!(
                                    "\"ev\":\"open\",\"path\":{},\"node\":{}",
                                    esc(&path.to_string_lossy()),
                                    esc(&d.node())
                                ));
                                device = Some(d);
                                board.set_mouse_ok(true);
                                String::new()
                            }
                            Err(e) => format!("{}: {e}", path.to_string_lossy()),
                        },
                    };
                    if !failure.is_empty() && failure != last_failure {
                        line(&mut text, clock.now_ns(), format!("\"ev\":\"unavailable\",\"why\":{}", esc(&failure)));
                        last_failure = failure;
                    }
                }
                if device.is_none() {
                    clock.sleep_until(now + MOUSE_WAIT_MS as u64 * 1_000_000);
                }
            }
            Some(d) => {
                let result = d.wait_and_drain(MOUSE_WAIT_MS, &mut |r| {
                    let t = clock.now_ns();
                    if mouse.report(&r, t, &*current) {
                        panel.store(mouse.ap(), t);
                    }
                    board.set_cursor(mouse.cursor(t));
                });
                if let Err(e) = result {
                    closes += 1;
                    device = None;
                    board.set_mouse_ok(false);
                    mouse.exit.disarm();
                    next_open = clock.now_ns() + REOPEN_NS;
                    line(&mut text, clock.now_ns(), format!("\"ev\":\"closed\",\"why\":{}", esc(&e.to_string())));
                }
            }
        }
        let t = clock.now_ns();
        board.set_cursor(if device.is_some() { mouse.cursor(t) } else { None });
        board.exit_armed.store(mouse.exit_armed(t), Ordering::Relaxed);
        if mouse.exit.confirmed() && !exit_sent {
            exit_sent = true;
            line(&mut text, t, format!("\"ev\":\"exit\",\"via\":\"{}\"", if stop.is_some() { "stop_flag" } else { "stop_sim" }));
            match stop {
                Some(flag) => flag.store(true, Ordering::Relaxed),
                None => board.stop_sim.store(true, Ordering::Relaxed),
            }
        }
    }
    board.set_cursor(None);
    board.exit_armed.store(false, Ordering::Relaxed);
    let p = panel.value().input;
    line(&mut text, clock.now_ns(), format!(
        "\"ev\":\"final\",\"opens\":{opens},\"closes\":{closes},\"reports\":{},\"clicks\":{},\"changes\":{},\"serial\":{},\"hdg\":{},\"alt\":{},\"spd\":{}",
        mouse.reports, mouse.clicks, mouse.changes, p.serial, p.heading_deg, p.altitude_ft, p.speed_kt
    ));
    rt
}

#[cfg(test)]
mod tests {
    use super::*;
    use gx_render::panel::BUTTONS;
    use gx_sim::ApAction;
    use std::collections::VecDeque;
    use std::sync::atomic::AtomicU64;

    fn centre(h: Hit) -> (f32, f32) {
        let b = BUTTONS.iter().find(|b| b.hit == h).unwrap();
        (b.x + b.w / 2.0, gx_render::panel::TOP + 30.0)
    }

    fn flight(t: Target) -> f64 {
        match t {
            Target::Heading => -87.6,
            Target::Altitude => 4_449.0,
            Target::Speed => 231.4,
        }
    }

    /// Move the (visible) cursor to `to` and optionally click / wheel there.
    fn at(m: &mut PanelMouse, now: u64, to: (f32, f32), left_pressed: bool, wheel: i32) -> bool {
        let (x, y) = m.cursor(now).expect("cursor shown");
        let dx = (to.0 - x).round() as i32;
        let dy = (to.1 - y).round() as i32;
        m.report(&MouseReport { dx, dy, wheel, left_pressed }, now, &flight)
    }

    fn shown() -> PanelMouse {
        let mut m = PanelMouse::new(ApInput::DEFAULT);
        assert!(!m.report(&MouseReport { dx: 1, ..Default::default() }, 1, &flight), "reveal only");
        m
    }

    #[test]
    fn action_buttons_bump_the_serial_once_per_click() {
        let mut m = shown();
        for (k, a) in [ApAction::Master, ApAction::Level, ApAction::Heading, ApAction::Altitude, ApAction::Speed, ApAction::Approach].into_iter().enumerate() {
            assert!(at(&mut m, 2, centre(Hit::Action(a)), true, 0));
            assert_eq!((m.ap().serial, m.ap().action), (k as u32 + 1, a));
        }
        // The same button twice: a new serial each time.
        let p = centre(Hit::Action(ApAction::Master));
        at(&mut m, 3, p, true, 0);
        at(&mut m, 4, p, true, 0);
        assert_eq!((m.ap().serial, m.ap().action), (8, ApAction::Master));
        // Moving over a button without a click changes nothing; a gap click neither.
        assert!(!at(&mut m, 5, centre(Hit::Action(ApAction::Level)), false, 0));
        assert!(!at(&mut m, 6, (90.0, gx_render::panel::TOP + 30.0), true, 0));
        assert!(!at(&mut m, 7, (640.0, 300.0), true, 0), "above the strip");
        assert_eq!(m.ap().serial, 8);
        assert_eq!(m.clicks, 8, "gap and off-panel clicks are not counted");
    }

    #[test]
    fn steps_move_the_targets_and_wrap_or_clamp_like_normalized() {
        let mut m = shown();
        let minus = |t| centre(Hit::Step(t, -1));
        let plus = |t| centre(Hit::Step(t, 1));
        at(&mut m, 2, minus(Target::Heading), true, 0);
        assert_eq!(m.ap().heading_deg, 355.0, "000 - 5 wraps");
        at(&mut m, 3, plus(Target::Heading), true, 0);
        assert_eq!(m.ap().heading_deg, 0.0, "355 + 5 wraps to 000");
        at(&mut m, 4, plus(Target::Altitude), true, 0);
        assert_eq!(m.ap().altitude_ft, 3_500.0);
        for _ in 0..10 {
            at(&mut m, 5, minus(Target::Altitude), true, 0);
        }
        assert_eq!(m.ap().altitude_ft, 0.0, "clamped at the bottom");
        for _ in 0..30 {
            at(&mut m, 6, plus(Target::Speed), true, 0);
        }
        assert_eq!(m.ap().speed_kt, 350.0, "clamped at the top");
        assert_eq!(m.ap().serial, 0, "steps are not actions");
    }

    #[test]
    fn a_value_click_syncs_to_the_flight_and_the_wheel_fine_steps_it() {
        let mut m = shown();
        let value = |t| centre(Hit::Value(t));
        at(&mut m, 2, value(Target::Heading), true, 0);
        assert_eq!(m.ap().heading_deg, 272.0, "-87.6 true -> 272");
        at(&mut m, 3, value(Target::Altitude), true, 0);
        assert_eq!(m.ap().altitude_ft, 4_400.0, "AGL to the 100 ft step");
        at(&mut m, 4, value(Target::Speed), true, 0);
        assert_eq!(m.ap().speed_kt, 231.0);
        at(&mut m, 5, value(Target::Heading), false, 3);
        assert_eq!(m.ap().heading_deg, 275.0);
        at(&mut m, 6, value(Target::Altitude), false, -2);
        assert_eq!(m.ap().altitude_ft, 4_200.0);
        at(&mut m, 7, value(Target::Speed), false, 1);
        assert_eq!(m.ap().speed_kt, 232.0);
        // The wheel elsewhere does nothing, even over a step or action button.
        let before = m.ap();
        assert!(!at(&mut m, 8, centre(Hit::Step(Target::Speed, 1)), false, 5));
        assert!(!at(&mut m, 9, centre(Hit::Action(ApAction::Master)), false, 5));
        assert!(!at(&mut m, 10, (640.0, 100.0), false, 5));
        assert_eq!(m.ap(), before);
        // A non-finite flight value leaves the target alone.
        let nan = |_: Target| f64::NAN;
        assert_eq!(click(before, Hit::Value(Target::Speed), &nan), before);
    }

    #[test]
    fn a_click_on_a_hidden_cursor_only_reveals_it() {
        let mut m = shown();
        let p = centre(Hit::Action(ApAction::Master));
        at(&mut m, 2, p, false, 0);
        let later = 2 + gx_input::mouse::HIDE_AFTER_NS;
        assert_eq!(m.cursor(later), None);
        assert!(!m.report(&MouseReport { left_pressed: true, ..Default::default() }, later, &flight));
        assert_eq!(m.ap().serial, 0);
        assert!(m.report(&MouseReport { left_pressed: true, ..Default::default() }, later + 1, &flight));
        assert_eq!(m.ap().serial, 1);
    }

    #[derive(Clone)]
    struct StepClock(Arc<AtomicU64>);

    impl Clock for StepClock {
        fn now_ns(&self) -> u64 {
            self.0.load(Ordering::Relaxed)
        }
        fn sleep_until(&self, deadline_ns: u64) {
            self.0.fetch_max(deadline_ns, Ordering::Relaxed);
        }
    }

    /// Scripted device: each call delivers one batch (advancing the clock
    /// by the wait), then fails once the script is used up.
    struct Scripted {
        batches: VecDeque<Vec<MouseReport>>,
        clock: StepClock,
        board: Arc<Board>,
        stop_after_fail: bool,
    }

    impl MouseDevice for Scripted {
        fn wait_and_drain(&mut self, timeout_ms: i32, report: &mut dyn FnMut(MouseReport)) -> io::Result<()> {
            self.clock.0.fetch_add(timeout_ms as u64 * 1_000_000, Ordering::Relaxed);
            match self.batches.pop_front() {
                Some(b) => {
                    b.into_iter().for_each(report);
                    assert!(self.board.mouse_ok(), "mouse_ok while open");
                    Ok(())
                }
                None => {
                    if self.stop_after_fail {
                        self.board.stop_sim.store(true, Ordering::Relaxed);
                    }
                    Err(io::Error::other("ENODEV"))
                }
            }
        }
        fn node(&self) -> String {
            "/dev/input/event3".into()
        }
    }

    #[test]
    fn the_loop_maps_clicks_into_the_panel_reopens_after_unplug_and_reports_no_mouse() {
        let clock = StepClock(Arc::new(AtomicU64::new(5_000_000_000)));
        let board = Arc::new(Board::default());
        let (writer, reader) = crate::ap_panel(ApInput::DEFAULT);
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let reveal = MouseReport { dx: 1, ..Default::default() };
        let (x, y) = centre(Hit::Action(ApAction::Level));
        let to_level = MouseReport { dx: (x - 641.0) as i32, dy: (y - 360.0) as i32, ..Default::default() };
        let press = MouseReport { left_pressed: true, ..Default::default() };
        let mut opens = 0;
        let ctx = MouseCtx {
            clock: clock.clone(),
            epoch: 0,
            board: Arc::clone(&board),
            panel: writer,
            current: Box::new(flight),
            text: TextTx::new(tx),
            stop: None,
        };
        let (c2, b2) = (clock.clone(), Arc::clone(&board));
        let rt = mouse_loop(MouseSelect::Path("/dev/input/by-id/test-event-mouse".into()), move |p| {
            assert_eq!(p, std::path::Path::new("/dev/input/by-id/test-event-mouse"));
            opens += 1;
            match opens {
                1 => Ok(Scripted { batches: vec![vec![reveal, to_level], vec![press]].into(), clock: c2.clone(), board: Arc::clone(&b2), stop_after_fail: false }),
                2 => Err(io::Error::from(io::ErrorKind::PermissionDenied)),
                _ => Ok(Scripted { batches: vec![vec![press]].into(), clock: c2.clone(), board: Arc::clone(&b2), stop_after_fail: true }),
            }
        }, ctx);
        assert!(rt.contains("\"cpu\":0"), "{rt}");
        let v = reader.load().unwrap();
        // First device: Level once. The pointer survives the reopen and is
        // still shown (within 10 s), so the third device's click lands on
        // Level again; the refused open in between changes nothing.
        assert_eq!((v.input.serial, v.input.action), (2, ApAction::Level));
        assert!(!board.mouse_ok(), "closed at the end");
        assert_eq!(board.cursor(), None);
        let lines: Vec<String> = rx.try_iter().collect();
        let evs: Vec<&str> = lines.iter().map(|l| l.split("\"ev\":\"").nth(1).unwrap().split('"').next().unwrap()).collect();
        assert_eq!(evs, ["start", "open", "closed", "unavailable", "open", "closed", "final"], "{lines:#?}");
        assert!(lines[3].contains("permission denied") || lines[3].contains("Permission denied") || lines[3].contains("PermissionDenied"), "{}", lines[3]);
        assert!(lines.last().unwrap().contains("\"opens\":2,\"closes\":2,\"reports\":4,\"clicks\":2,\"changes\":2"), "{}", lines.last().unwrap());
    }

    #[test]
    fn exit_arms_on_the_first_click_and_confirms_only_inside_3_s() {
        let mut e = ExitLatch::default();
        assert!(!e.armed(0));
        assert!(!e.press(1_000), "first click arms");
        assert!(e.armed(1_000) && e.armed(1_000 + EXIT_WINDOW_NS - 1));
        assert!(e.press(1_000 + EXIT_WINDOW_NS - 1), "second click inside the window confirms");
        assert!(e.confirmed() && !e.armed(1_000 + EXIT_WINDOW_NS - 1));
        assert!(!e.press(1_000 + EXIT_WINDOW_NS), "confirmed once only");

        let mut e = ExitLatch::default();
        e.press(0);
        assert!(!e.armed(EXIT_WINDOW_NS), "the window runs out");
        assert!(!e.press(EXIT_WINDOW_NS), "a late click re-arms, it does not confirm");
        assert!(e.armed(EXIT_WINDOW_NS) && !e.confirmed());
        e.disarm();
        assert!(!e.press(EXIT_WINDOW_NS + 1) && !e.confirmed(), "after a disarm the next click arms again");
    }

    #[test]
    fn exit_is_not_an_autopilot_input_and_another_button_disarms_it() {
        let mut m = shown();
        let before = m.ap();
        assert!(!m.press(Hit::Exit, 10, &flight));
        assert!(m.exit_armed(10));
        assert_eq!(m.ap(), before, "no serial, no action");
        assert!(m.press(Hit::Action(ApAction::Level), 20, &flight));
        assert!(!m.exit_armed(20), "a click elsewhere disarms EXIT");
        assert!(!m.press(Hit::Exit, 30, &flight));
        assert!(!m.exit.confirmed(), "that EXIT click only re-armed");
        m.press(Hit::Exit, 40, &flight);
        assert!(m.exit.confirmed());
        assert_eq!(m.ap().serial, 1);
    }

    fn all_exit(_: f32, _: f32) -> Option<Hit> {
        Some(Hit::Exit)
    }

    /// The thread: arm shows on the Board, the window running out clears it,
    /// and a confirmed double click sets the run's stop flag (the SIGTERM
    /// path), once.
    #[test]
    fn the_loop_shows_exit_armed_and_a_confirmed_exit_sets_the_stop_flag() {
        static STOP: AtomicBool = AtomicBool::new(false);
        let clock = StepClock(Arc::new(AtomicU64::new(1_000_000_000)));
        let board = Arc::new(Board::default());
        let (writer, _reader) = crate::ap_panel(ApInput::DEFAULT);
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let reveal = MouseReport { dx: 1, ..Default::default() };
        let press = MouseReport { left_pressed: true, ..Default::default() };
        // 100 ms per batch: reveal; arm; 3 s idle (disarms); arm; confirm.
        let mut batches: VecDeque<Vec<MouseReport>> = vec![vec![reveal], vec![press]].into();
        batches.extend((0..30).map(|_| Vec::new()));
        batches.extend([vec![press], vec![press]]);
        batches.extend((0..5).map(|_| Vec::new()));
        struct Watch {
            inner: Scripted,
            armed: Vec<bool>,
            stop_seen_at: Option<usize>,
        }
        impl MouseDevice for Watch {
            fn wait_and_drain(&mut self, t: i32, r: &mut dyn FnMut(MouseReport)) -> io::Result<()> {
                self.armed.push(self.inner.board.exit_armed());
                if STOP.load(Ordering::Relaxed) && self.stop_seen_at.is_none() {
                    self.stop_seen_at = Some(self.armed.len());
                    self.inner.board.stop_sim.store(true, Ordering::Relaxed);
                }
                self.inner.wait_and_drain(t, r)
            }
            fn node(&self) -> String {
                self.inner.node()
            }
        }
        let watch = std::sync::Arc::new(std::sync::Mutex::new(None::<(Vec<bool>, Option<usize>)>));
        let ctx = MouseCtx {
            clock: clock.clone(),
            epoch: 0,
            board: Arc::clone(&board),
            panel: writer,
            current: Box::new(flight),
            text: TextTx::new(tx),
            stop: Some(&STOP),
        };
        let (c2, b2, w2) = (clock.clone(), Arc::clone(&board), Arc::clone(&watch));
        let mut device = Some(Watch {
            inner: Scripted { batches, clock: c2, board: b2, stop_after_fail: true },
            armed: Vec::new(),
            stop_seen_at: None,
        });
        struct Report(Watch, std::sync::Arc<std::sync::Mutex<Option<(Vec<bool>, Option<usize>)>>>);
        impl MouseDevice for Report {
            fn wait_and_drain(&mut self, t: i32, r: &mut dyn FnMut(MouseReport)) -> io::Result<()> {
                let res = self.0.wait_and_drain(t, r);
                *self.1.lock().unwrap() = Some((self.0.armed.clone(), self.0.stop_seen_at));
                res
            }
            fn node(&self) -> String {
                self.0.node()
            }
        }
        let initial = ApInput::DEFAULT;
        mouse_loop_with(
            MouseSelect::Path("/dev/input/by-id/test-event-mouse".into()),
            move |_| device.take().map(|d| Report(d, Arc::clone(&w2))).ok_or_else(|| io::Error::other("once")),
            ctx,
            PanelMouse::new(initial).with_hit(all_exit),
        );
        let (armed, stop_seen_at) = watch.lock().unwrap().clone().unwrap();
        // armed[k] is the Board flag seen before batch k is delivered.
        assert!(!armed[1], "not armed before the first EXIT click");
        assert!(armed[2], "the first click shows EXIT? on the Board");
        assert!(armed[30], "still armed 2.8 s later");
        assert!(!armed[32], "3 s after the click it is disarmed");
        assert!(armed[33] && !armed[34], "re-armed, then confirmed (no longer armed)");
        assert!(STOP.load(Ordering::Relaxed), "a confirmed EXIT sets the stop flag");
        assert_eq!(stop_seen_at, Some(35), "within one loop pass of the confirming click");
        assert!(!board.exit_armed());
        let lines: Vec<String> = rx.try_iter().collect();
        assert_eq!(lines.iter().filter(|l| l.contains("\"ev\":\"exit\"")).count(), 1, "{lines:#?}");
        assert!(lines.iter().any(|l| l.contains("\"ev\":\"exit\",\"via\":\"stop_flag\"")));
    }
}

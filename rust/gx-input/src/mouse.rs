//! Relative mouse input for the autopilot panel (AP-001 W3): evdev record
//! decoding into one report per `SYN_REPORT`, and the cursor in a virtual
//! screen. Platform independent and host-testable; the Linux reader is
//! [`linux::MouseReader`].
//!
//! The mouse is not a flight control: it never feeds the [`crate::ControlMachine`]
//! and has no loss semantics of its own. After `SYN_DROPPED` the records up
//! to the next `SYN_REPORT` are discarded (evdev protocol), so a click is
//! acted on only from a complete report; dropped motion only moves the cursor
//! less.

use crate::code::{EV_KEY, EV_SYN, SYN_DROPPED, SYN_REPORT};

pub const EV_REL: u16 = 0x02;
pub const REL_X: u16 = 0x00;
pub const REL_Y: u16 = 0x01;
pub const REL_WHEEL: u16 = 0x08;
pub const BTN_LEFT: u16 = 0x110;

/// Cursor pixels per mouse count. 1:1 is the usual desktop feel on the
/// 1280x720 display (an 800 DPI mouse crosses it in 1.6 inches).
pub const DEFAULT_SENSITIVITY: f32 = 1.0;
/// The cursor hides after this long without mouse activity.
pub const HIDE_AFTER_NS: u64 = 10_000_000_000;

/// One complete mouse report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MouseReport {
    pub dx: i32,
    pub dy: i32,
    /// Detents, + = away from the user (up).
    pub wheel: i32,
    /// `BTN_LEFT` went down in this report (release and autorepeat ignored).
    pub left_pressed: bool,
}

impl MouseReport {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

/// Accumulates evdev records into [`MouseReport`]s.
#[derive(Clone, Debug, Default)]
pub struct MouseDecoder {
    pending: MouseReport,
    dropping: bool,
    pub dropped_reports: u64,
}

impl MouseDecoder {
    /// One record; returns the report it completes, if non-empty.
    pub fn feed(&mut self, event_type: u16, code: u16, value: i32) -> Option<MouseReport> {
        match (event_type, code) {
            (EV_SYN, SYN_DROPPED) => {
                self.pending = MouseReport::default();
                self.dropping = true;
                self.dropped_reports += 1;
                None
            }
            (EV_SYN, SYN_REPORT) => {
                let report = std::mem::take(&mut self.pending);
                if std::mem::take(&mut self.dropping) || report.is_empty() {
                    None
                } else {
                    Some(report)
                }
            }
            _ if self.dropping => None,
            (EV_REL, REL_X) => {
                self.pending.dx = self.pending.dx.saturating_add(value);
                None
            }
            (EV_REL, REL_Y) => {
                self.pending.dy = self.pending.dy.saturating_add(value);
                None
            }
            (EV_REL, REL_WHEEL) => {
                self.pending.wheel = self.pending.wheel.saturating_add(value);
                None
            }
            (EV_KEY, BTN_LEFT) => {
                if value == 1 {
                    self.pending.left_pressed = true;
                }
                None
            }
            _ => None,
        }
    }

    /// One raw 64-bit Linux `struct input_event` (24 bytes: `timeval`, then
    /// `type`, `code`, `value`, native endian).
    pub fn feed_record(&mut self, record: &[u8; INPUT_EVENT_BYTES]) -> Option<MouseReport> {
        let event_type = u16::from_ne_bytes([record[16], record[17]]);
        let code = u16::from_ne_bytes([record[18], record[19]]);
        let value = i32::from_ne_bytes([record[20], record[21], record[22], record[23]]);
        self.feed(event_type, code, value)
    }
}

/// `sizeof(struct input_event)` on 64-bit Linux (aarch64, x86_64).
pub const INPUT_EVENT_BYTES: usize = 24;

/// The cursor in a `width` x `height` virtual screen, clamped to it, hidden
/// [`HIDE_AFTER_NS`] after the last activity and before the first.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pointer {
    x: f32,
    y: f32,
    width: f32,
    height: f32,
    sensitivity: f32,
    last_activity_ns: Option<u64>,
}

impl Pointer {
    /// Starts hidden at the centre of the screen.
    pub fn new(width: f32, height: f32, sensitivity: f32) -> Self {
        Self { x: width / 2.0, y: height / 2.0, width, height, sensitivity, last_activity_ns: None }
    }

    /// Whether the cursor shows at `now_ns`.
    pub fn visible(&self, now_ns: u64) -> bool {
        self.last_activity_ns.is_some_and(|t| now_ns.saturating_sub(t) < HIDE_AFTER_NS)
    }

    /// The cursor position if it shows at `now_ns`.
    pub fn position(&self, now_ns: u64) -> Option<(f32, f32)> {
        self.visible(now_ns).then_some((self.x, self.y))
    }

    /// Apply a report's motion. Returns the position at which its click, if
    /// any, lands: `None` if the cursor was hidden before this report, so a
    /// click on an invisible cursor only reveals it and hits nothing. Any
    /// report counts as activity.
    pub fn apply(&mut self, report: &MouseReport, now_ns: u64) -> Option<(f32, f32)> {
        let was_visible = self.visible(now_ns);
        self.x = (self.x + report.dx as f32 * self.sensitivity).clamp(0.0, self.width - 1.0);
        self.y = (self.y + report.dy as f32 * self.sensitivity).clamp(0.0, self.height - 1.0);
        self.last_activity_ns = Some(now_ns);
        was_visible.then_some((self.x, self.y))
    }
}

/// The first `*-event-mouse` entry (by name) of a udev `by-id` directory,
/// e.g. `/dev/input/by-id/usb-3554_2.4G_Receiver-if01-event-mouse`. `None`
/// when the directory is missing or has no mouse.
pub fn find_mouse(by_id: &std::path::Path) -> Option<std::path::PathBuf> {
    let mut found: Vec<std::path::PathBuf> = std::fs::read_dir(by_id)
        .ok()?
        .filter_map(|e| e.ok())
        .filter(|e| e.file_name().to_str().is_some_and(|n| n.ends_with("-event-mouse")))
        .map(|e| e.path())
        .collect();
    found.sort();
    found.into_iter().next()
}

pub const DEFAULT_BY_ID: &str = "/dev/input/by-id";

#[cfg(target_os = "linux")]
pub mod linux {
    //! Blocking-with-timeout evdev mouse reader for a `SCHED_OTHER` thread
    //! of its own. `open` (which can sleep ~50 ms in `usbhid_open`) and
    //! every read happen on that thread, never on a realtime one.

    use super::{MouseDecoder, MouseReport, INPUT_EVENT_BYTES};
    use std::ffi::{c_int, c_short, c_ulong};
    use std::fs::{File, OpenOptions};
    use std::io::{self, Read};
    use std::os::fd::AsRawFd;
    use std::os::unix::fs::OpenOptionsExt;
    use std::path::{Path, PathBuf};

    const O_NONBLOCK: i32 = 0x800;
    const POLLIN: c_short = 0x1;

    #[repr(C)]
    struct PollFd {
        fd: c_int,
        events: c_short,
        revents: c_short,
    }

    extern "C" {
        fn poll(fds: *mut PollFd, nfds: c_ulong, timeout: c_int) -> c_int;
    }

    /// An open mouse event node.
    pub struct MouseReader {
        file: File,
        /// The node actually opened (symlink resolved).
        pub node: PathBuf,
        decoder: MouseDecoder,
        partial: [u8; INPUT_EVENT_BYTES],
        partial_len: usize,
        pub events: u64,
    }

    impl MouseReader {
        /// Open `path` (a `by-id` symlink or an event node) read-only and
        /// nonblocking. Never grabs the device.
        pub fn open(path: &Path) -> io::Result<Self> {
            let file = OpenOptions::new().read(true).custom_flags(O_NONBLOCK).open(path)?;
            let node = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_owned());
            Ok(Self { file, node, decoder: MouseDecoder::default(), partial: [0; INPUT_EVENT_BYTES], partial_len: 0, events: 0 })
        }

        /// Reports dropped by `SYN_DROPPED` so far.
        pub fn dropped_reports(&self) -> u64 {
            self.decoder.dropped_reports
        }

        /// Wait up to `timeout_ms` for records, then drain the fd to
        /// `EAGAIN`, calling `report` per complete report. `Err` means the
        /// device is gone (unplug: `ENODEV`) or unusable; reopen it.
        pub fn wait_and_drain(&mut self, timeout_ms: i32, report: &mut dyn FnMut(MouseReport)) -> io::Result<()> {
            let mut pfd = PollFd { fd: self.file.as_raw_fd(), events: POLLIN, revents: 0 };
            // SAFETY: one valid pollfd for the duration of the call.
            let rc = unsafe { poll(&mut pfd, 1, timeout_ms) };
            if rc < 0 {
                let e = io::Error::last_os_error();
                return if e.kind() == io::ErrorKind::Interrupted { Ok(()) } else { Err(e) };
            }
            if rc == 0 {
                return Ok(());
            }
            let mut buf = [0u8; INPUT_EVENT_BYTES * 64];
            loop {
                match self.file.read(&mut buf) {
                    Ok(0) => return Err(io::Error::from(io::ErrorKind::UnexpectedEof)),
                    Ok(n) => {
                        for &b in &buf[..n] {
                            self.partial[self.partial_len] = b;
                            self.partial_len += 1;
                            if self.partial_len == INPUT_EVENT_BYTES {
                                self.partial_len = 0;
                                self.events += 1;
                                if let Some(r) = self.decoder.feed_record(&self.partial) {
                                    report(r);
                                }
                            }
                        }
                    }
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(()),
                    Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                    Err(e) => return Err(e),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(event_type: u16, code: u16, value: i32) -> [u8; INPUT_EVENT_BYTES] {
        let mut r = [0u8; INPUT_EVENT_BYTES];
        r[..8].copy_from_slice(&123i64.to_ne_bytes());
        r[8..16].copy_from_slice(&456i64.to_ne_bytes());
        r[16..18].copy_from_slice(&event_type.to_ne_bytes());
        r[18..20].copy_from_slice(&code.to_ne_bytes());
        r[20..].copy_from_slice(&value.to_ne_bytes());
        r
    }

    #[test]
    fn decodes_one_report_per_syn_report_from_raw_24_byte_records() {
        let mut d = MouseDecoder::default();
        assert_eq!(d.feed_record(&record(EV_REL, REL_X, 3)), None);
        assert_eq!(d.feed_record(&record(EV_REL, REL_X, 2)), None);
        assert_eq!(d.feed_record(&record(EV_REL, REL_Y, -4)), None);
        assert_eq!(d.feed_record(&record(EV_REL, REL_WHEEL, -1)), None);
        assert_eq!(d.feed_record(&record(EV_KEY, BTN_LEFT, 1)), None);
        assert_eq!(
            d.feed_record(&record(EV_SYN, SYN_REPORT, 0)),
            Some(MouseReport { dx: 5, dy: -4, wheel: -1, left_pressed: true })
        );
        // Release, autorepeat, other buttons/axes and empty reports: nothing.
        for (t, c, v) in [(EV_KEY, BTN_LEFT, 0), (EV_KEY, BTN_LEFT, 2), (EV_KEY, 0x111, 1), (EV_REL, 0x06, 1), (0x04, 4, 9)] {
            assert_eq!(d.feed(t, c, v), None);
        }
        assert_eq!(d.feed(EV_SYN, SYN_REPORT, 0), None);
    }

    #[test]
    fn syn_dropped_discards_up_to_the_next_report_so_a_dropped_click_never_lands() {
        let mut d = MouseDecoder::default();
        d.feed(EV_REL, REL_X, 7);
        d.feed(EV_SYN, SYN_DROPPED, 0);
        d.feed(EV_KEY, BTN_LEFT, 1);
        assert_eq!(d.feed(EV_SYN, SYN_REPORT, 0), None);
        assert_eq!(d.dropped_reports, 1);
        d.feed(EV_REL, REL_Y, 1);
        assert_eq!(d.feed(EV_SYN, SYN_REPORT, 0), Some(MouseReport { dy: 1, ..Default::default() }));
    }

    #[test]
    fn pointer_clamps_to_the_screen_and_hides_after_ten_seconds() {
        let mut p = Pointer::new(1280.0, 720.0, DEFAULT_SENSITIVITY);
        assert_eq!(p.position(0), None, "hidden before the first activity");
        let mv = |dx, dy| MouseReport { dx, dy, ..Default::default() };
        assert_eq!(p.apply(&mv(10, -20), 1_000), None, "the revealing report hits nothing");
        assert_eq!(p.position(1_000), Some((650.0, 340.0)));
        assert_eq!(p.apply(&mv(-5000, 5000), 2_000), Some((0.0, 719.0)));
        p.apply(&mv(99_999, -99_999), 3_000);
        assert_eq!(p.position(3_000), Some((1279.0, 0.0)));
        assert!(p.visible(3_000 + HIDE_AFTER_NS - 1));
        assert_eq!(p.position(3_000 + HIDE_AFTER_NS), None);
        // A click on the hidden cursor reveals it where it was, hitting nothing.
        let click = MouseReport { left_pressed: true, ..Default::default() };
        assert_eq!(p.apply(&click, 3_000 + HIDE_AFTER_NS), None);
        assert_eq!(p.apply(&click, 3_001 + HIDE_AFTER_NS), Some((1279.0, 0.0)));
    }

    #[test]
    fn sensitivity_scales_counts_to_pixels() {
        let mut p = Pointer::new(1280.0, 720.0, 0.5);
        p.apply(&MouseReport::default(), 0);
        assert_eq!(p.apply(&MouseReport { dx: 10, dy: 4, ..Default::default() }, 1), Some((645.0, 362.0)));
    }

    #[test]
    fn find_mouse_picks_the_first_event_mouse_by_name_and_nothing_else() {
        let dir = std::env::temp_dir().join(format!("gx-input-mouse-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        assert_eq!(find_mouse(&dir), None, "missing directory");
        std::fs::create_dir_all(&dir).unwrap();
        for name in ["usb-Logitech_Logitech_Extreme_3D-event-joystick", "usb-3554_2.4G_Receiver-event-kbd", "usb-3554_2.4G_Receiver-if01-mouse"] {
            std::fs::write(dir.join(name), b"").unwrap();
        }
        assert_eq!(find_mouse(&dir), None, "no *-event-mouse");
        for name in ["usb-zz-event-mouse", "usb-3554_2.4G_Receiver-if01-event-mouse"] {
            std::fs::write(dir.join(name), b"").unwrap();
        }
        assert_eq!(find_mouse(&dir), Some(dir.join("usb-3554_2.4G_Receiver-if01-event-mouse")));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}

//! Linux evdev adapter. Opening and every read are nonblocking; discovery is
//! rate-limited by `InputConfig::reconnect_interval_ns`.
//!
//! # Blocking work at reconnect
//!
//! `O_NONBLOCK` makes reads nonblocking but not `open(2)`/`close(2)`. On a
//! USB HID stick the first open sleeps in the kernel (`usbhid_open` waits
//! 50 ms for the device queue to settle) and the last close waits for an RCU
//! grace period (`input_close_device`). Selection itself reads sysfs. A
//! realtime owner therefore calls [`EvdevInput::start_background_connector`]:
//! selection, identity checks, `open` and `close` then run on a helper thread
//! and the owning thread only adopts a ready fd or hands one back, without
//! waiting (see [`RuntimeEvent::Background`]).
//!
//! # Live resync after `SYN_DROPPED` (ISS-032)
//!
//! 1. `SYN_DROPPED` clears controls; the machine discards records until the
//!    discard boundary (`SYN_REPORT`, event limit or timeout) and becomes
//!    `Disarmed`. The adapter emits `ResyncRequired` and enters
//!    `ConnectionState::ResyncRequired`, keeping the fd open.
//! 2. The adapter keeps draining the fd to `EAGAIN`. Those records are still
//!    recorded as `Input` but produce no frames.
//! 3. Once drained, it reads `EVIOCGABS` for every configured axis and
//!    `EVIOCGKEY` (full bitmap) on the same fd and hands the complete
//!    snapshot to `ControlMachine::resynchronize`. Only that call re-arms.
//!    The re-arm frame is emitted and the state returns to `Connected`.
//! 4. On ioctl failure or a rejected snapshot (e.g. a missing axis) the
//!    machine stays disarmed and the attempt is retried after a bounded
//!    backoff (`ResyncPolicy`). Once the budget is spent, the fd is closed and
//!    the existing reconnect path reopens it after `reconnect_interval_ns`,
//!    with a fresh budget. `ENODEV`/`ENXIO` goes to the reconnect path at once.
//!
//! Records that reach the kernel queue between the final `EAGAIN` and the
//! ioctls are already reflected in the snapshot and are applied after it. They
//! carry values no newer than the snapshot. An axis that moved twice inside
//! that microsecond window can briefly show its intermediate value until the
//! next report; this is the same residual window libevdev accepts.

use crate::resync::{
    attempt_resync, device_gone, eviocgabs, DeviceStateSource, InputAbsinfo, LiveSnapshot,
    ResyncFailure, ResyncOutcome, ResyncPolicy, ResyncScheduler, RetryDecision, EVIOCGKEY_FULL,
    KEY_BITMAP_BYTES,
};
use crate::{
    ConnectionState, ControlFrame, ControlMachine, DeviceIdentity, DeviceSnapshot, InputConfig,
    InputEvent, InputLoss, MachineError, MachineOutput, ResyncRequired,
};
use std::ffi::{c_int, c_ulong};
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read};
use std::os::fd::AsRawFd;
use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::thread;
use std::time::Instant;

const O_NONBLOCK: i32 = 0x800;
const INPUT_EVENT_BYTES: usize = 24; // 64-bit Linux: timeval + type + code + value.

extern "C" {
    // glibc: int ioctl(int fd, unsigned long request, ...);
    fn ioctl(fd: c_int, request: c_ulong, ...) -> c_int;
    fn sched_setscheduler(pid: c_int, policy: c_int, param: *const c_int) -> c_int;
}

/// Put the calling thread in `SCHED_OTHER` (always permitted: it only lowers
/// priority). The connector blocks in `open`/`close` by design, so it must
/// never inherit a realtime policy from the thread that spawned it.
fn demote_to_sched_other() {
    let priority: c_int = 0;
    // SAFETY: pid 0 is the calling thread; `struct sched_param` is a single
    // int, and `priority` outlives the call.
    unsafe {
        sched_setscheduler(0, 0, &priority);
    }
}

/// `EVIOCGABS`/`EVIOCGKEY` on an open evdev fd. Both are read-only queries.
struct EvdevFd<'a>(&'a File);

impl DeviceStateSource for EvdevFd<'_> {
    fn abs_info(&mut self, code: u16) -> io::Result<InputAbsinfo> {
        let mut info = InputAbsinfo::default();
        let request = eviocgabs(code) as c_ulong;
        // SAFETY: the fd is open for the borrow's lifetime, and the kernel
        // writes exactly size_of::<InputAbsinfo>() bytes (the size encoded in
        // `request`) into `info`, whose layout is checked at compile time.
        let rc = unsafe { ioctl(self.0.as_raw_fd(), request, &mut info as *mut InputAbsinfo) };
        if rc < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(info)
    }

    fn key_bitmap(&mut self, bits: &mut [u8; KEY_BITMAP_BYTES]) -> io::Result<()> {
        // SAFETY: as above; the kernel writes at most KEY_BITMAP_BYTES (the
        // length encoded in EVIOCGKEY_FULL) into `bits`.
        let rc = unsafe {
            ioctl(
                self.0.as_raw_fd(),
                EVIOCGKEY_FULL as c_ulong,
                bits.as_mut_ptr(),
            )
        };
        if rc < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
}

fn trimmed(path: &Path) -> io::Result<String> {
    Ok(fs::read_to_string(path)?.trim().to_owned())
}

fn hex_u16(path: &Path) -> io::Result<u16> {
    u16::from_str_radix(&trimmed(path)?, 16)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}

fn optional_text(path: &Path) -> Option<String> {
    trimmed(path).ok().filter(|s| !s.is_empty())
}

/// Enumerates event nodes through sysfs and reads stable USB/device metadata.
/// Malformed/unrelated entries are skipped; an unreadable sysfs root is an
/// error so callers can distinguish platform failure from "not connected".
pub fn discover(sys_class_input: &Path, dev_input: &Path) -> io::Result<Vec<DeviceIdentity>> {
    let mut entries = Vec::new();
    for entry in fs::read_dir(sys_class_input)? {
        let entry = match entry {
            Ok(v) => v,
            Err(_) => continue,
        };
        let file_name = entry.file_name();
        let event_name = file_name.to_string_lossy();
        if !event_name.starts_with("event") {
            continue;
        }
        let identity = identity_for_event(&entry.path(), dev_input.join(event_name.as_ref()));
        if let Ok(identity) = identity {
            entries.push(identity);
        }
    }
    entries.sort_by(|a, b| a.event_path.cmp(&b.event_path));
    Ok(entries)
}

fn identity_for_event(sys_event: &Path, event_path: PathBuf) -> io::Result<DeviceIdentity> {
    let device = sys_event.join("device");
    Ok(DeviceIdentity {
        event_path: event_path.to_string_lossy().into_owned(),
        name: trimmed(&device.join("name"))?,
        vendor_id: hex_u16(&device.join("id/vendor"))?,
        product_id: hex_u16(&device.join("id/product"))?,
        version: hex_u16(&device.join("id/version"))?,
        physical_path: optional_text(&device.join("phys")),
        unique_id: optional_text(&device.join("uniq")),
    })
}

/// glibc `makedev`.
fn makedev(major: u64, minor: u64) -> u64 {
    ((major & 0xffff_f000) << 32)
        | ((major & 0x0000_0fff) << 8)
        | ((minor & 0xffff_ff00) << 12)
        | (minor & 0x0000_00ff)
}

/// The `major:minor` device number sysfs reports for an event node.
fn sysfs_dev(sys_event: &Path) -> io::Result<u64> {
    let text = trimmed(&sys_event.join("dev"))?;
    let bad = || io::Error::new(io::ErrorKind::InvalidData, format!("bad sysfs dev {text:?}"));
    let (major, minor) = text.split_once(':').ok_or_else(bad)?;
    Ok(makedev(major.parse().map_err(|_| bad())?, minor.parse().map_err(|_| bad())?))
}

/// What reconnect looks for. Owned (and cloned into the background
/// connector) so selection does not borrow the adapter.
#[derive(Clone, Debug)]
struct Selector {
    matcher: crate::DeviceMatcher,
    sys_class_input: PathBuf,
    dev_input: PathBuf,
    exact_path: Option<PathBuf>,
}

/// Select, identity-check and open the device. `Ok(None)` means "not
/// present"; `Err` carries the message for `ConnectionState::Error`.
fn select_and_open(sel: &Selector) -> Result<Option<(DeviceIdentity, File)>, String> {
    let open = |path: &Path| {
        OpenOptions::new()
            .read(true)
            .custom_flags(O_NONBLOCK)
            .open(path)
            .map_err(|e| format!("open {}: {e}", path.display()))
    };
    let Some(exact) = &sel.exact_path else {
        let found = discover(&sel.sys_class_input, &sel.dev_input)
            .map_err(|e| format!("input discovery: {e}"))?
            .into_iter()
            .find(|device| sel.matcher.matches(device));
        return match found {
            Some(device) => {
                let file = open(Path::new(&device.event_path))?;
                Ok(Some((device, file)))
            }
            None => Ok(None),
        };
    };
    let discovery = |e: io::Error| format!("input discovery: {e}");
    let resolved = match fs::canonicalize(exact) {
        Ok(path) => path,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(discovery(error)),
    };
    let dev_root = fs::canonicalize(&sel.dev_input).map_err(discovery)?;
    if resolved.parent() != Some(dev_root.as_path()) {
        return Err(format!(
            "input discovery: {} resolves outside {}",
            exact.display(),
            dev_root.display()
        ));
    }
    let event_name = resolved
        .file_name()
        .and_then(|name| name.to_str())
        .filter(|name| name.starts_with("event"))
        .ok_or_else(|| format!("input discovery: {} is not an event node", resolved.display()))?
        .to_owned();
    let sys_event = sel.sys_class_input.join(&event_name);
    let identity = identity_for_event(&sys_event, resolved.clone()).map_err(discovery)?;
    if !sel.matcher.matches(&identity) {
        return Err(format!(
            "input discovery: {} identity does not match: {} {:04x}:{:04x}",
            exact.display(),
            identity.name,
            identity.vendor_id,
            identity.product_id
        ));
    }
    // Open the node whose identity was read (not the symlink, which could be
    // retargeted in between), then prove the fd is that node: its device
    // number must equal sysfs's, and the stable path must still point at it.
    let file = open(&resolved)?;
    let expected = sysfs_dev(&sys_event).map_err(discovery)?;
    let actual = file
        .metadata()
        .map_err(|e| format!("fstat {}: {e}", resolved.display()))?
        .rdev();
    if actual != expected {
        return Err(format!(
            "input discovery: opened {} is device {actual:#x}, sysfs says {expected:#x}",
            resolved.display()
        ));
    }
    match fs::canonicalize(exact) {
        Ok(again) if again == resolved => Ok(Some((identity, file))),
        _ => Err(format!(
            "input discovery: {} changed target during open",
            exact.display()
        )),
    }
}

enum Job {
    Connect,
    Close(File),
}

enum Report {
    Connect {
        result: Result<Option<(DeviceIdentity, File)>, String>,
        duration_ns: u64,
    },
    Closed {
        duration_ns: u64,
    },
}

/// Helper thread that performs reconnect selection, `open` and `close`.
struct Connector {
    jobs: Sender<Job>,
    reports: Receiver<Report>,
    pending: bool,
}

impl Connector {
    fn spawn(sel: Selector) -> io::Result<Self> {
        let (jobs, job_rx) = mpsc::channel::<Job>();
        let (report_tx, reports) = mpsc::channel();
        thread::Builder::new()
            .name("gx-input-conn".into())
            .spawn(move || {
                demote_to_sched_other();
                for job in job_rx {
                    let started = Instant::now();
                    let report = match job {
                        Job::Connect => {
                            let result = select_and_open(&sel);
                            Report::Connect { result, duration_ns: elapsed_ns(started) }
                        }
                        Job::Close(file) => {
                            drop(file);
                            Report::Closed { duration_ns: elapsed_ns(started) }
                        }
                    };
                    if report_tx.send(report).is_err() {
                        break;
                    }
                }
            })?;
        Ok(Self { jobs, reports, pending: false })
    }
}

fn elapsed_ns(started: Instant) -> u64 {
    u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX)
}

#[derive(Clone, Debug, PartialEq)]
pub enum RuntimeEvent {
    State(ConnectionState),
    /// Canonical record suitable for a recording stream.
    Input(InputEvent),
    Frame(ControlFrame),
    LossDetected(InputLoss),
    ResyncRequired(ResyncRequired),
    /// Raw `EVIOCGABS`/`EVIOCGKEY` measurement behind a live resync that
    /// re-armed (axis ranges included as calibration evidence).
    DeviceStateRead(LiveSnapshot),
    ResyncCompleted {
        snapshot: DeviceSnapshot,
        frame_sequence: u64,
    },
    /// A live resync attempt failed; controls remain disarmed. `retry_at_ns`
    /// is `None` when the budget is spent and the reconnect path takes over.
    ResyncAttemptFailed {
        attempt: u32,
        reason: String,
        retry_at_ns: Option<u64>,
    },
    /// The background connector finished a blocking operation off the
    /// owner's thread: `"connect"` (selection, identity checks and `open`,
    /// whatever its outcome) or `"close"`. Evidence only.
    Background {
        operation: &'static str,
        duration_ns: u64,
    },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PollStats {
    pub bytes_read: usize,
    pub events_read: usize,
    pub frames_emitted: usize,
    pub losses_detected: usize,
    pub suppressed_events: usize,
    pub resyncs_required: usize,
    pub resyncs_completed: usize,
    pub resync_failures: usize,
}

/// Stateful Linux source intended to be owned by one I/O thread.
pub struct EvdevInput {
    config: InputConfig,
    machine: ControlMachine,
    selector: Selector,
    connector: Option<Connector>,
    file: Option<File>,
    active_device: Option<DeviceIdentity>,
    state: ConnectionState,
    next_scan_ns: u64,
    event_sequence: u64,
    partial: [u8; INPUT_EVENT_BYTES],
    partial_len: usize,
    resync_policy: ResyncPolicy,
    resync: ResyncScheduler,
}

impl EvdevInput {
    pub fn new(config: InputConfig) -> Self {
        Self::with_roots(config, "/sys/class/input", "/dev/input")
    }

    pub fn with_roots(
        config: InputConfig,
        sys_class_input: impl Into<PathBuf>,
        dev_input: impl Into<PathBuf>,
    ) -> Self {
        Self {
            machine: ControlMachine::new(config.clone()),
            selector: Selector {
                matcher: config.matcher.clone(),
                sys_class_input: sys_class_input.into(),
                dev_input: dev_input.into(),
                exact_path: None,
            },
            connector: None,
            config,
            file: None,
            active_device: None,
            state: ConnectionState::Disconnected,
            next_scan_ns: 0,
            event_sequence: 0,
            partial: [0; INPUT_EVENT_BYTES],
            partial_len: 0,
            resync_policy: ResyncPolicy::default(),
            resync: ResyncScheduler::default(),
        }
    }

    /// Use one configured stable path instead of enumerating every input
    /// device at reconnect. The symlink target must be an event node under
    /// `dev_input` whose sysfs identity matches `config`; the node itself is
    /// opened and its device number must equal sysfs's.
    pub fn with_exact_path(config: InputConfig, path: impl Into<PathBuf>) -> Self {
        let mut input = Self::new(config);
        input.selector.exact_path = Some(path.into());
        input
    }

    /// Move reconnect selection, identity checks, `open(2)` and `close(2)` to
    /// a helper thread (see the module docs). The helper inherits the
    /// caller's CPU affinity and priority, so call this before pinning the
    /// owning thread. Without it those run inline in `poll_nonblocking`.
    pub fn start_background_connector(&mut self) -> io::Result<()> {
        if self.connector.is_none() {
            self.connector = Some(Connector::spawn(self.selector.clone())?);
        }
        Ok(())
    }

    #[cfg(test)]
    fn with_roots_and_exact_path(
        config: InputConfig,
        sys_class_input: impl Into<PathBuf>,
        dev_input: impl Into<PathBuf>,
        path: impl Into<PathBuf>,
    ) -> Self {
        let mut input = Self::with_roots(config, sys_class_input, dev_input);
        input.selector.exact_path = Some(path.into());
        input
    }

    pub fn set_resync_policy(&mut self, policy: ResyncPolicy) {
        self.resync_policy = policy;
    }

    pub fn state(&self) -> &ConnectionState {
        &self.state
    }

    fn set_state(&mut self, state: ConnectionState, emit: &mut impl FnMut(RuntimeEvent)) {
        if self.state != state {
            self.state = state.clone();
            emit(RuntimeEvent::State(state));
        }
    }

    fn schedule_retry(&mut self, now_ns: u64) {
        self.next_scan_ns = now_ns.saturating_add(self.config.reconnect_interval_ns);
    }

    /// Collect finished background work (never waits).
    fn drain_connector(&mut self, emit: &mut impl FnMut(RuntimeEvent)) {
        loop {
            let Some(connector) = self.connector.as_mut() else {
                return;
            };
            let report = match connector.reports.try_recv() {
                Ok(report) => report,
                Err(TryRecvError::Empty) => return,
                Err(TryRecvError::Disconnected) => {
                    // The helper died: fail closed and fall back to inline.
                    self.connector = None;
                    self.set_state(
                        ConnectionState::Error {
                            message: "input connector thread exited".into(),
                            retry_at_ns: self.next_scan_ns,
                        },
                        emit,
                    );
                    return;
                }
            };
            match report {
                Report::Closed { duration_ns } => emit(RuntimeEvent::Background {
                    operation: "close",
                    duration_ns,
                }),
                Report::Connect { result, duration_ns } => {
                    connector.pending = false;
                    emit(RuntimeEvent::Background {
                        operation: "connect",
                        duration_ns,
                    });
                    self.finish_connect(result, emit);
                }
            }
        }
    }

    fn connect_if_due(&mut self, now_ns: u64, emit: &mut impl FnMut(RuntimeEvent)) {
        self.drain_connector(emit);
        if self.file.is_some() || now_ns < self.next_scan_ns {
            return;
        }
        if self.connector.as_ref().is_some_and(|c| c.pending) {
            return;
        }
        self.schedule_retry(now_ns);
        if let Some(connector) = self.connector.as_mut() {
            if connector.jobs.send(Job::Connect).is_ok() {
                connector.pending = true;
            }
            // A send error means the helper died; the next drain reports it.
            return;
        }
        let result = select_and_open(&self.selector);
        self.finish_connect(result, emit);
    }

    fn finish_connect(
        &mut self,
        result: Result<Option<(DeviceIdentity, File)>, String>,
        emit: &mut impl FnMut(RuntimeEvent),
    ) {
        match result {
            Ok(Some((device, file))) => {
                if self.file.is_some() {
                    // Cannot happen (one request at a time); never leak.
                    self.release_file(file);
                    return;
                }
                self.file = Some(file);
                self.active_device = Some(device.clone());
                self.partial_len = 0;
                self.event_sequence = 0;
                self.resync.reset();
                let state = match self.machine.pending_loss() {
                    Some(loss) => ConnectionState::ResyncRequired { device, loss },
                    None => ConnectionState::Connected(device),
                };
                self.set_state(state, emit);
            }
            Ok(None) => self.set_state(ConnectionState::Disconnected, emit),
            Err(message) => self.set_state(
                ConnectionState::Error {
                    message,
                    retry_at_ns: self.next_scan_ns,
                },
                emit,
            ),
        }
    }

    /// Close an fd: on the helper thread when there is one.
    fn release_file(&mut self, file: File) {
        if let Some(connector) = &self.connector {
            if let Err(mpsc::SendError(Job::Close(file))) = connector.jobs.send(Job::Close(file)) {
                drop(file);
            }
        } else {
            drop(file);
        }
    }

    fn close_fd(&mut self, now_ns: u64) {
        if let Some(file) = self.file.take() {
            self.release_file(file);
        }
        self.active_device = None;
        self.partial_len = 0;
        self.machine.disconnect();
        self.schedule_retry(now_ns);
    }

    fn lose_connection(
        &mut self,
        now_ns: u64,
        error: Option<io::Error>,
        emit: &mut impl FnMut(RuntimeEvent),
    ) {
        self.close_fd(now_ns);
        match error {
            Some(e) if matches!(e.raw_os_error(), Some(6 | 19)) => {
                self.set_state(ConnectionState::Disconnected, emit)
            }
            Some(e) => self.set_state(
                ConnectionState::Error {
                    message: format!("evdev read: {e}"),
                    retry_at_ns: self.next_scan_ns,
                },
                emit,
            ),
            None => self.set_state(ConnectionState::Disconnected, emit),
        }
    }

    /// Discard boundary reached: the machine is `Disarmed`. Keep the fd; the
    /// poll drains it to `EAGAIN` and then attempts a live snapshot.
    fn require_resync(
        &mut self,
        required: ResyncRequired,
        stats: &mut PollStats,
        emit: &mut impl FnMut(RuntimeEvent),
    ) {
        stats.resyncs_required += 1;
        emit(RuntimeEvent::ResyncRequired(required));
        self.resync.reset();
        if let Some(device) = self.active_device.clone() {
            self.set_state(
                ConnectionState::ResyncRequired {
                    device,
                    loss: required.loss,
                },
                emit,
            );
        }
    }

    /// One `EVIOCGABS`/`EVIOCGKEY` snapshot attempt on the open, drained fd.
    fn try_live_resync(
        &mut self,
        now_ns: u64,
        stats: &mut PollStats,
        emit: &mut impl FnMut(RuntimeEvent),
    ) {
        let Some(file) = self.file.as_ref() else {
            return;
        };
        let outcome = attempt_resync(
            &mut self.machine,
            &mut EvdevFd(file),
            &self.config,
            now_ns,
            self.event_sequence,
        );
        let failure = match outcome {
            ResyncOutcome::Rearmed {
                live,
                snapshot,
                frame,
            } => {
                // The snapshot consumed this sequence number.
                self.event_sequence = self.event_sequence.wrapping_add(1);
                self.resync.reset();
                stats.resyncs_completed += 1;
                stats.frames_emitted += 1;
                emit(RuntimeEvent::DeviceStateRead(live));
                emit(RuntimeEvent::ResyncCompleted {
                    snapshot,
                    frame_sequence: frame.sequence,
                });
                emit(RuntimeEvent::Frame(frame));
                if let Some(device) = self.active_device.clone() {
                    self.set_state(ConnectionState::Connected(device), emit);
                }
                return;
            }
            ResyncOutcome::Failed(ResyncFailure::Ioctl(_, e)) if device_gone(&e) => {
                self.lose_connection(now_ns, Some(e), emit);
                return;
            }
            ResyncOutcome::Failed(f) => f,
        };
        stats.resync_failures += 1;
        let reason = match &failure {
            ResyncFailure::Ioctl(which, e) => format!("{which:?} ioctl: {e}"),
            ResyncFailure::Rejected(e) => format!("snapshot rejected: {e:?}"),
        };
        let decision = self.resync.record_failure(&self.resync_policy, now_ns);
        let attempt = self.resync.failures();
        let retry_at_ns = match decision {
            RetryDecision::RetryAt(t) => Some(t),
            RetryDecision::GiveUp => None,
        };
        emit(RuntimeEvent::ResyncAttemptFailed {
            attempt,
            reason: reason.clone(),
            retry_at_ns,
        });
        if decision == RetryDecision::GiveUp {
            // Fall back to the reconnect path; controls stay disarmed and a
            // reopened fd gets a fresh attempt budget.
            self.close_fd(now_ns);
            self.set_state(
                ConnectionState::Error {
                    message: format!("live resync failed {attempt} times: {reason}"),
                    retry_at_ns: self.next_scan_ns,
                },
                emit,
            );
        }
    }

    /// Completes recovery only from a full ioctl-derived snapshot. Until this
    /// succeeds the adapter continues recording raw events but emits no frames.
    pub fn complete_resync(
        &mut self,
        mut snapshot: DeviceSnapshot,
        mut emit: impl FnMut(RuntimeEvent),
    ) -> Result<(), MachineError> {
        if self.file.is_none() || !matches!(self.state, ConnectionState::ResyncRequired { .. }) {
            return Err(MachineError::ResyncDeviceNotConnected);
        }
        // The adapter owns this sequence domain. The snapshot is the next
        // canonical input record after any raw records already drained.
        snapshot.source_event_sequence = self.event_sequence;
        let recorded_snapshot = snapshot.clone();
        let frame = self.machine.resynchronize(snapshot)?;
        self.event_sequence = self.event_sequence.wrapping_add(1);
        self.resync.reset();
        let frame_sequence = frame.sequence;
        emit(RuntimeEvent::ResyncCompleted {
            snapshot: recorded_snapshot,
            frame_sequence,
        });
        emit(RuntimeEvent::Frame(frame));
        if let Some(device) = self.active_device.clone() {
            self.set_state(ConnectionState::Connected(device), &mut emit);
        }
        Ok(())
    }

    /// Drains currently available events and returns immediately on EAGAIN.
    /// While the machine awaits a snapshot, a drained poll also makes at most
    /// one live resync attempt (see the module docs).
    /// `now_ns` must be from the application's monotonic clock. It becomes the
    /// canonical ingest timestamp rather than trusting evdev's configurable
    /// kernel clock domain.
    pub fn poll_nonblocking(
        &mut self,
        now_ns: u64,
        mut emit: impl FnMut(RuntimeEvent),
    ) -> PollStats {
        self.connect_if_due(now_ns, &mut emit);
        let mut stats = PollStats::default();
        if self.file.is_some() {
            if let Some(required) = self.machine.check_resync_timeout(now_ns) {
                self.require_resync(required, &mut stats, &mut emit);
            }
        }
        let mut drained = false;
        let mut read_buffer = [0_u8; 1024];
        loop {
            let result = match self.file.as_mut() {
                Some(file) => file.read(&mut read_buffer),
                None => break,
            };
            match result {
                Ok(0) => {
                    self.lose_connection(now_ns, None, &mut emit);
                    break;
                }
                Ok(n) => {
                    stats.bytes_read += n;
                    for byte in &read_buffer[..n] {
                        self.partial[self.partial_len] = *byte;
                        self.partial_len += 1;
                        if self.partial_len == INPUT_EVENT_BYTES {
                            self.partial_len = 0;
                            let event_type =
                                u16::from_ne_bytes([self.partial[16], self.partial[17]]);
                            let event_code =
                                u16::from_ne_bytes([self.partial[18], self.partial[19]]);
                            let value = i32::from_ne_bytes([
                                self.partial[20],
                                self.partial[21],
                                self.partial[22],
                                self.partial[23],
                            ]);
                            let event = InputEvent {
                                timestamp_ns: now_ns,
                                sequence: self.event_sequence,
                                event_type,
                                code: event_code,
                                value,
                            };
                            self.event_sequence = self.event_sequence.wrapping_add(1);
                            stats.events_read += 1;
                            emit(RuntimeEvent::Input(event));
                            match self.machine.apply_detailed(event) {
                                Ok(MachineOutput::Frame(frame)) => {
                                    stats.frames_emitted += 1;
                                    emit(RuntimeEvent::Frame(frame));
                                }
                                Ok(MachineOutput::LossDetected(loss)) => {
                                    stats.losses_detected += 1;
                                    emit(RuntimeEvent::LossDetected(loss));
                                }
                                Ok(MachineOutput::ResyncRequired(required)) => {
                                    stats.suppressed_events += 1;
                                    self.require_resync(required, &mut stats, &mut emit);
                                }
                                Ok(MachineOutput::None) => {
                                    if self.machine.needs_resync() {
                                        stats.suppressed_events += 1;
                                    }
                                }
                                Err(e) => {
                                    let message = format!("input state error: {e:?}");
                                    self.lose_connection(
                                        now_ns,
                                        Some(io::Error::new(io::ErrorKind::InvalidData, message)),
                                        &mut emit,
                                    );
                                    return stats;
                                }
                            }
                        }
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    drained = true;
                    break;
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    self.lose_connection(now_ns, Some(e), &mut emit);
                    break;
                }
            }
        }
        if drained
            && self.file.is_some()
            && self.machine.awaiting_snapshot()
            && self.resync.due(now_ns)
        {
            self.try_live_resync(now_ns, &mut stats, &mut emit);
        }
        stats
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::symlink;

    fn exact_fixture(vendor: &str) -> (PathBuf, PathBuf, PathBuf) {
        let root = std::env::temp_dir().join(format!(
            "gx-input-exact-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let sys = root.join("sys/class/input");
        let dev = root.join("dev/input");
        let by_id = dev.join("by-id/joystick");
        fs::create_dir_all(sys.join("event8/device/id")).unwrap();
        fs::create_dir_all(by_id.parent().unwrap()).unwrap();
        fs::write(dev.join("event8"), []).unwrap();
        fs::write(sys.join("event8/device/name"), "Logitech Logitech Extreme 3D\n").unwrap();
        fs::write(sys.join("event8/device/id/vendor"), format!("{vendor}\n")).unwrap();
        fs::write(sys.join("event8/device/id/product"), "c215\n").unwrap();
        fs::write(sys.join("event8/device/id/version"), "0110\n").unwrap();
        // The fixture node is a regular file: device number 0:0.
        fs::write(sys.join("event8/dev"), "0:0\n").unwrap();
        symlink("../event8", &by_id).unwrap();
        (sys, dev, by_id)
    }

    #[test]
    fn missing_sysfs_is_an_explicit_error() {
        let result = discover(
            Path::new("/definitely/not/a/sysfs/path"),
            Path::new("/dev/input"),
        );
        assert!(result.is_err());
    }

    #[test]
    fn disconnected_poll_is_nonblocking_and_rate_limited() {
        let mut input = EvdevInput::with_roots(
            InputConfig::extreme_3d_pro(),
            "/definitely/missing",
            "/dev/input",
        );
        let mut states = Vec::new();
        assert_eq!(
            input.poll_nonblocking(10, |e| states.push(e)),
            PollStats::default()
        );
        assert!(matches!(
            states.as_slice(),
            [RuntimeEvent::State(ConnectionState::Error { .. })]
        ));
        states.clear();
        assert_eq!(
            input.poll_nonblocking(11, |e| states.push(e)),
            PollStats::default()
        );
        assert!(states.is_empty());
    }

    #[test]
    fn exact_path_selects_only_its_validated_node_without_enumeration() {
        let (sys, dev, by_id) = exact_fixture("046d");
        // A malformed unrelated entry would be visited by full discovery.
        fs::create_dir_all(sys.join("event0/device")).unwrap();
        let input = EvdevInput::with_roots_and_exact_path(
            InputConfig::extreme_3d_pro(),
            &sys,
            &dev,
            &by_id,
        );
        let (identity, _file) = select_and_open(&input.selector).unwrap().unwrap();
        assert_eq!(
            identity.event_path,
            fs::canonicalize(dev.join("event8")).unwrap().to_string_lossy()
        );
        fs::remove_dir_all(sys.ancestors().nth(3).unwrap()).unwrap();
    }

    #[test]
    fn exact_path_rejects_the_wrong_device_identity() {
        let (sys, dev, by_id) = exact_fixture("ffff");
        let input = EvdevInput::with_roots_and_exact_path(
            InputConfig::extreme_3d_pro(),
            &sys,
            &dev,
            &by_id,
        );
        let error = select_and_open(&input.selector).unwrap_err();
        assert!(error.contains("identity does not match"), "{error}");
        fs::remove_dir_all(sys.ancestors().nth(3).unwrap()).unwrap();
    }

    #[test]
    fn exact_path_rejects_an_fd_whose_device_number_differs_from_sysfs() {
        let (sys, dev, by_id) = exact_fixture("046d");
        fs::write(sys.join("event8/dev"), "13:72\n").unwrap();
        let input = EvdevInput::with_roots_and_exact_path(
            InputConfig::extreme_3d_pro(),
            &sys,
            &dev,
            &by_id,
        );
        let error = select_and_open(&input.selector).unwrap_err();
        assert!(error.contains("sysfs says"), "{error}");
        fs::remove_dir_all(sys.ancestors().nth(3).unwrap()).unwrap();
    }

    #[test]
    fn missing_stable_path_is_disconnected_not_an_error() {
        let (sys, dev, by_id) = exact_fixture("046d");
        fs::remove_file(&by_id).unwrap();
        let input = EvdevInput::with_roots_and_exact_path(
            InputConfig::extreme_3d_pro(),
            &sys,
            &dev,
            &by_id,
        );
        assert!(select_and_open(&input.selector).unwrap().is_none());
        fs::remove_dir_all(sys.ancestors().nth(3).unwrap()).unwrap();
    }

    #[test]
    fn background_connector_opens_and_closes_off_the_polling_thread() {
        let (sys, dev, by_id) = exact_fixture("046d");
        let mut input = EvdevInput::with_roots_and_exact_path(
            InputConfig::extreme_3d_pro(),
            &sys,
            &dev,
            &by_id,
        );
        input.start_background_connector().unwrap();
        let mut events = Vec::new();
        // The first poll only queues the request: nothing blocks, no state.
        input.poll_nonblocking(1, |e| events.push(e));
        assert!(events.is_empty(), "{events:?}");
        assert!(input.connector.as_ref().unwrap().pending);
        // Poll until the helper's result is adopted. The fixture node is an
        // empty regular file, so the adopting poll reads EOF and hands the fd
        // straight back to the helper to close.
        let mut now = 2;
        while !events.iter().any(|e| matches!(e, RuntimeEvent::Background { operation: "close", .. })) {
            assert!(now < 5_000, "connector made no progress: {events:?}");
            std::thread::sleep(std::time::Duration::from_millis(1));
            input.poll_nonblocking(now, |e| events.push(e));
            now += 1;
        }
        let kinds: Vec<&str> = events
            .iter()
            .map(|e| match e {
                RuntimeEvent::Background { operation, .. } => *operation,
                RuntimeEvent::State(ConnectionState::Connected(_)) => "connected",
                RuntimeEvent::State(ConnectionState::Disconnected) => "disconnected",
                _ => "other",
            })
            .collect();
        assert_eq!(kinds, ["connect", "connected", "disconnected", "close"]);
        // No second request before the reconnect interval.
        assert!(!input.connector.as_ref().unwrap().pending);
        fs::remove_dir_all(sys.ancestors().nth(3).unwrap()).unwrap();
    }
}

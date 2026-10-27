//! Live evdev state snapshot after `SYN_DROPPED` (ISS-032).
//!
//! The kernel's documented recovery (Documentation/input/event-codes.rst,
//! `SYN_DROPPED`) is: discard events up to and including the next
//! `SYN_REPORT`, then query the device's current state with the `EVIOCG*`
//! ioctls. This module holds the platform-independent part of that recovery
//! so it can be unit-tested on any host:
//!
//! * the hand-written evdev ABI (`input_absinfo`, ioctl numbers, key bitmap
//!   length) with compile-time size/offset/number checks;
//! * the [`DeviceStateSource`] seam that the Linux adapter implements with
//!   real ioctls and tests implement with fakes;
//! * snapshot assembly and the bounded retry policy ([`attempt_resync`],
//!   [`ResyncScheduler`]).
//!
//! Re-arming is never decided here: every snapshot goes through
//! [`ControlMachine::resynchronize`], which keeps its existing rules (machine
//! must be `Disarmed`, every configured axis present, only known buttons,
//! monotonic sequence/time).

use crate::{ControlFrame, ControlMachine, DeviceSnapshot, InputConfig, MachineError};
use std::io;
use std::mem::{align_of, offset_of, size_of};

// ---------------------------------------------------------------- evdev ABI --
// Source: Linux 6.16 uapi, Debian linux-libc-dev 6.16.7-1 (the package
// pinned by tools/fetch_m0_scanout_headers.py for the board's 6.16.7 kernel,
// sha256 aa15f58c...0f811b1e):
//   <linux/input.h>              struct input_absinfo; EVIOCGABS; EVIOCGKEY
//   <linux/input-event-codes.h>  KEY_MAX 0x2ff, KEY_CNT, ABS_MAX 0x3f
//   <asm-generic/ioctl.h>        _IOC encoding; arm64's <asm/ioctl.h> is
//                                exactly `#include <asm-generic/ioctl.h>`
//                                (NRBITS 8, TYPEBITS 8, SIZEBITS 14,
//                                DIRBITS 2, _IOC_READ 2).

/// `struct input_absinfo` (six `__s32`, 24 bytes, no padding).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputAbsinfo {
    pub value: i32,
    pub minimum: i32,
    pub maximum: i32,
    pub fuzz: i32,
    pub flat: i32,
    pub resolution: i32,
}

const _: () = {
    assert!(size_of::<InputAbsinfo>() == 24);
    assert!(align_of::<InputAbsinfo>() == 4);
    assert!(offset_of!(InputAbsinfo, value) == 0);
    assert!(offset_of!(InputAbsinfo, minimum) == 4);
    assert!(offset_of!(InputAbsinfo, maximum) == 8);
    assert!(offset_of!(InputAbsinfo, fuzz) == 12);
    assert!(offset_of!(InputAbsinfo, flat) == 16);
    assert!(offset_of!(InputAbsinfo, resolution) == 20);
};

/// `KEY_MAX` from `<linux/input-event-codes.h>`.
pub const KEY_MAX: u16 = 0x2ff;
/// `ABS_MAX` from `<linux/input-event-codes.h>`.
pub const ABS_MAX: u16 = 0x3f;
/// Bytes needed for a `KEY_CNT`-bit bitmap: (0x2ff + 1) / 8 = 96.
pub const KEY_BITMAP_BYTES: usize = (KEY_MAX as usize + 1) / 8;

const IOC_NRSHIFT: u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + 8;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + 8;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + 14;
const IOC_READ: u32 = 2;
const IOC_SIZE_LIMIT: usize = 1 << 14;

/// `_IOC(dir, type, nr, size)` for the asm-generic encoding used by arm64.
pub const fn ioc(dir: u32, ty: u8, nr: u8, size: usize) -> u32 {
    assert!(size < IOC_SIZE_LIMIT);
    (dir << IOC_DIRSHIFT)
        | ((ty as u32) << IOC_TYPESHIFT)
        | ((nr as u32) << IOC_NRSHIFT)
        | ((size as u32) << IOC_SIZESHIFT)
}

/// `EVIOCGABS(abs)` = `_IOR('E', 0x40 + abs, struct input_absinfo)`.
pub const fn eviocgabs(abs: u16) -> u32 {
    assert!(abs <= ABS_MAX);
    ioc(IOC_READ, b'E', 0x40 + abs as u8, size_of::<InputAbsinfo>())
}

/// `EVIOCGKEY(len)` = `_IOC(_IOC_READ, 'E', 0x18, len)`.
pub const fn eviocgkey(len: usize) -> u32 {
    ioc(IOC_READ, b'E', 0x18, len)
}

/// Full-bitmap `EVIOCGKEY`, as passed to `ioctl(2)`.
pub const EVIOCGKEY_FULL: u32 = eviocgkey(KEY_BITMAP_BYTES);

// Independent literals: the encoder above must reproduce the numbers the C
// macros give with the pinned headers (checked with `zig cc -target
// aarch64-linux-gnu` static_asserts against linux-libc-dev 6.16.7-1).
const _: () = {
    assert!(KEY_BITMAP_BYTES == 96);
    assert!(eviocgabs(0) == 0x8018_4540);
    assert!(eviocgabs(crate::code::ABS_THROTTLE) == 0x8018_4546);
    assert!(eviocgabs(ABS_MAX) == 0x8018_457f);
    assert!(EVIOCGKEY_FULL == 0x8060_4518);
};

// ------------------------------------------------------------------ seam --

/// Which ioctl a snapshot read failed in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapshotIoctl {
    Abs(u16),
    Key,
}

/// Read access to the device's current state. The Linux adapter implements
/// this with `EVIOCGABS`/`EVIOCGKEY` on its open fd; tests use fakes.
pub trait DeviceStateSource {
    fn abs_info(&mut self, code: u16) -> io::Result<InputAbsinfo>;
    fn key_bitmap(&mut self, bits: &mut [u8; KEY_BITMAP_BYTES]) -> io::Result<()>;
}

/// One axis as measured by `EVIOCGABS`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisState {
    pub code: u16,
    pub value: i32,
    pub minimum: i32,
    pub maximum: i32,
}

/// Raw measurement of every configured axis and the full key bitmap.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LiveSnapshot {
    pub axes: Vec<AxisState>,
    pub key_bits: [u8; KEY_BITMAP_BYTES],
}

impl LiveSnapshot {
    pub fn key_pressed(&self, code: u16) -> bool {
        let code = code as usize;
        code / 8 < KEY_BITMAP_BYTES && self.key_bits[code / 8] & (1 << (code % 8)) != 0
    }

    /// Builds the machine snapshot. An axis whose reported range is
    /// degenerate (`minimum >= maximum`) is omitted: evdev answers
    /// `EVIOCGABS` for an axis the device does not have with all zeros, so
    /// such an axis is *missing*, and `resynchronize` rejects the snapshot.
    /// Only configured buttons are reported as pressed.
    pub fn to_device_snapshot(&self, config: &InputConfig, timestamp_ns: u64) -> DeviceSnapshot {
        DeviceSnapshot {
            timestamp_ns,
            source_event_sequence: 0,
            axes: self
                .axes
                .iter()
                .filter(|a| a.minimum < a.maximum)
                .map(|a| (a.code, a.value))
                .collect(),
            pressed_buttons: config
                .buttons
                .iter()
                .copied()
                .take(64)
                .filter(|&c| self.key_pressed(c))
                .collect(),
        }
    }
}

/// Reads every configured axis, then the key bitmap. Any ioctl failure aborts
/// the read: a partial device read is never turned into a snapshot.
pub fn read_live_snapshot(
    source: &mut impl DeviceStateSource,
    config: &InputConfig,
) -> Result<LiveSnapshot, (SnapshotIoctl, io::Error)> {
    let mut axes = Vec::with_capacity(config.axes.len());
    for binding in &config.axes {
        let code = binding.evdev_code;
        if code > ABS_MAX {
            return Err((
                SnapshotIoctl::Abs(code),
                io::Error::new(io::ErrorKind::InvalidInput, "axis code above ABS_MAX"),
            ));
        }
        let info = source
            .abs_info(code)
            .map_err(|e| (SnapshotIoctl::Abs(code), e))?;
        axes.push(AxisState {
            code,
            value: info.value,
            minimum: info.minimum,
            maximum: info.maximum,
        });
    }
    let mut key_bits = [0_u8; KEY_BITMAP_BYTES];
    source
        .key_bitmap(&mut key_bits)
        .map_err(|e| (SnapshotIoctl::Key, e))?;
    Ok(LiveSnapshot { axes, key_bits })
}

// --------------------------------------------------------------- policy --

/// Bounded retry for snapshot attempts on one open fd. After `max_attempts`
/// failures the adapter closes the fd and falls back to the reconnect path
/// (rate-limited by `InputConfig::reconnect_interval_ns`), which starts a
/// fresh budget after reopening. Controls stay disarmed throughout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResyncPolicy {
    pub max_attempts: u32,
    pub initial_backoff_ns: u64,
    pub max_backoff_ns: u64,
}

impl Default for ResyncPolicy {
    /// 4 attempts, 10/20/40 ms apart: worst case ~70 ms of extra dead stick
    /// before the 1 s reconnect fallback.
    fn default() -> Self {
        Self {
            max_attempts: 4,
            initial_backoff_ns: 10_000_000,
            max_backoff_ns: 80_000_000,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ResyncScheduler {
    failures: u32,
    next_attempt_ns: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetryDecision {
    RetryAt(u64),
    GiveUp,
}

impl ResyncScheduler {
    /// Fresh budget, first attempt due immediately.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn failures(&self) -> u32 {
        self.failures
    }

    pub fn due(&self, now_ns: u64) -> bool {
        now_ns >= self.next_attempt_ns
    }

    pub fn record_failure(&mut self, policy: &ResyncPolicy, now_ns: u64) -> RetryDecision {
        self.failures = self.failures.saturating_add(1);
        if self.failures >= policy.max_attempts.max(1) {
            return RetryDecision::GiveUp;
        }
        let shift = (self.failures - 1).min(32);
        let backoff = policy
            .initial_backoff_ns
            .saturating_mul(1_u64 << shift)
            .min(policy.max_backoff_ns);
        self.next_attempt_ns = now_ns.saturating_add(backoff);
        RetryDecision::RetryAt(self.next_attempt_ns)
    }
}

/// Errnos meaning the device node is gone (`ENXIO`, `ENODEV`): use the
/// reconnect path rather than retrying on this fd.
pub fn device_gone(error: &io::Error) -> bool {
    matches!(error.raw_os_error(), Some(6 | 19))
}

#[derive(Debug)]
pub enum ResyncFailure {
    Ioctl(SnapshotIoctl, io::Error),
    Rejected(MachineError),
}

#[derive(Debug)]
pub enum ResyncOutcome {
    /// The machine re-armed. `frame` is its first frame, built from `snapshot`.
    Rearmed {
        live: LiveSnapshot,
        snapshot: DeviceSnapshot,
        frame: ControlFrame,
    },
    Failed(ResyncFailure),
}

/// One live resync attempt: measure, build a complete snapshot, and let the
/// machine decide. On failure the machine is untouched (still disarmed).
pub fn attempt_resync(
    machine: &mut ControlMachine,
    source: &mut impl DeviceStateSource,
    config: &InputConfig,
    timestamp_ns: u64,
    source_event_sequence: u64,
) -> ResyncOutcome {
    let live = match read_live_snapshot(source, config) {
        Ok(v) => v,
        Err((which, e)) => return ResyncOutcome::Failed(ResyncFailure::Ioctl(which, e)),
    };
    let mut snapshot = live.to_device_snapshot(config, timestamp_ns);
    snapshot.source_event_sequence = source_event_sequence;
    match machine.resynchronize(snapshot.clone()) {
        Ok(frame) => ResyncOutcome::Rearmed {
            live,
            snapshot,
            frame,
        },
        Err(e) => ResyncOutcome::Failed(ResyncFailure::Rejected(e)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{code, InputEvent, MachineOutput};
    use std::collections::{BTreeMap, VecDeque};

    /// Scripted ioctl results. Axes not in `axes` behave like evdev for an
    /// unsupported axis (all-zero `input_absinfo`).
    #[derive(Default)]
    struct FakeDevice {
        axes: BTreeMap<u16, InputAbsinfo>,
        pressed: Vec<u16>,
        abs_errors: VecDeque<io::Error>,
        key_errors: VecDeque<io::Error>,
        abs_calls: usize,
        key_calls: usize,
    }

    impl FakeDevice {
        fn extreme_3d(values: [i32; 6], pressed: &[u16]) -> Self {
            let ranges = [(0, 1023), (0, 1023), (0, 255), (0, 255), (-1, 1), (-1, 1)];
            let codes = [
                code::ABS_X,
                code::ABS_Y,
                code::ABS_RZ,
                code::ABS_THROTTLE,
                code::ABS_HAT0X,
                code::ABS_HAT0Y,
            ];
            let mut axes = BTreeMap::new();
            for i in 0..6 {
                axes.insert(
                    codes[i],
                    InputAbsinfo {
                        value: values[i],
                        minimum: ranges[i].0,
                        maximum: ranges[i].1,
                        ..InputAbsinfo::default()
                    },
                );
            }
            Self {
                axes,
                pressed: pressed.to_vec(),
                ..Self::default()
            }
        }
    }

    impl DeviceStateSource for FakeDevice {
        fn abs_info(&mut self, code: u16) -> io::Result<InputAbsinfo> {
            self.abs_calls += 1;
            if let Some(e) = self.abs_errors.pop_front() {
                return Err(e);
            }
            Ok(self.axes.get(&code).copied().unwrap_or_default())
        }

        fn key_bitmap(&mut self, bits: &mut [u8; KEY_BITMAP_BYTES]) -> io::Result<()> {
            self.key_calls += 1;
            if let Some(e) = self.key_errors.pop_front() {
                return Err(e);
            }
            for &c in &self.pressed {
                bits[c as usize / 8] |= 1 << (c % 8);
            }
            Ok(())
        }
    }

    fn ev(sequence: u64, event_type: u16, code: u16, value: i32) -> InputEvent {
        InputEvent {
            timestamp_ns: 1_000 + sequence,
            sequence,
            event_type,
            code,
            value,
        }
    }

    /// Full stick right + trigger held, one report, then a queue overflow and
    /// the discard boundary: the machine ends up `Disarmed`.
    fn disarmed_after_drop() -> ControlMachine {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(ev(1, code::EV_ABS, code::ABS_X, 1023)).unwrap();
        m.apply(ev(2, code::EV_ABS, code::ABS_THROTTLE, 0)).unwrap();
        m.apply(ev(3, code::EV_KEY, code::BTN_TRIGGER, 1)).unwrap();
        let before = m
            .apply(ev(4, code::EV_SYN, code::SYN_REPORT, 0))
            .unwrap()
            .unwrap();
        assert_eq!(
            (before.roll, before.throttle, before.buttons),
            (1.0, 1.0, 1)
        );
        assert!(matches!(
            m.apply_detailed(ev(5, code::EV_SYN, code::SYN_DROPPED, 0)),
            Ok(MachineOutput::LossDetected(_))
        ));
        m.apply_detailed(ev(6, code::EV_ABS, code::ABS_Y, 0))
            .unwrap();
        assert!(matches!(
            m.apply_detailed(ev(7, code::EV_SYN, code::SYN_REPORT, 0)),
            Ok(MachineOutput::ResyncRequired(_))
        ));
        assert!(m.awaiting_snapshot());
        m
    }

    fn assert_stays_disarmed(m: &mut ControlMachine, next_sequence: u64) {
        assert!(m.awaiting_snapshot());
        assert_eq!(
            m.apply(ev(next_sequence, code::EV_SYN, code::SYN_REPORT, 0))
                .unwrap(),
            None,
            "no frame may escape while disarmed"
        );
    }

    #[test]
    fn ioctl_numbers_match_aarch64_uapi() {
        assert_eq!(eviocgabs(code::ABS_X), 0x8018_4540);
        assert_eq!(eviocgabs(code::ABS_Y), 0x8018_4541);
        assert_eq!(eviocgabs(code::ABS_HAT0Y), 0x8018_4551);
        assert_eq!(EVIOCGKEY_FULL, 0x8060_4518);
        assert_eq!(size_of::<InputAbsinfo>(), 24);
    }

    #[test]
    fn ioctl_failure_retries_with_backoff_and_stays_fail_closed() {
        let mut m = disarmed_after_drop();
        let config = InputConfig::extreme_3d_pro();
        let policy = ResyncPolicy::default();
        let mut sched = ResyncScheduler::default();
        let mut dev = FakeDevice::extreme_3d([511, 511, 127, 255, 0, 0], &[]);
        dev.abs_errors.push_back(io::Error::from_raw_os_error(4)); // EINTR
        dev.key_errors.push_back(io::Error::from_raw_os_error(5)); // EIO

        let mut now = 10_000;
        assert!(sched.due(now));
        let ResyncOutcome::Failed(ResyncFailure::Ioctl(SnapshotIoctl::Abs(code::ABS_X), e)) =
            attempt_resync(&mut m, &mut dev, &config, now, 8)
        else {
            panic!("first ABS ioctl must fail");
        };
        assert!(!device_gone(&e));
        assert_eq!(dev.key_calls, 0, "read aborts at the first failing ioctl");
        assert_eq!(
            sched.record_failure(&policy, now),
            RetryDecision::RetryAt(now + 10_000_000)
        );
        assert_stays_disarmed(&mut m, 8);
        assert!(!sched.due(now + 9_999_999));

        now += 10_000_000;
        assert!(sched.due(now));
        assert!(matches!(
            attempt_resync(&mut m, &mut dev, &config, now, 9),
            ResyncOutcome::Failed(ResyncFailure::Ioctl(SnapshotIoctl::Key, _))
        ));
        assert_eq!(
            sched.record_failure(&policy, now),
            RetryDecision::RetryAt(now + 20_000_000)
        );
        assert_stays_disarmed(&mut m, 9);

        // Third attempt (errors exhausted) succeeds.
        now += 20_000_000;
        assert!(matches!(
            attempt_resync(&mut m, &mut dev, &config, now, 10),
            ResyncOutcome::Rearmed { .. }
        ));
        assert!(!m.awaiting_snapshot());
    }

    #[test]
    fn retry_budget_is_bounded_then_gives_up() {
        let policy = ResyncPolicy::default();
        let mut sched = ResyncScheduler::default();
        assert_eq!(
            sched.record_failure(&policy, 0),
            RetryDecision::RetryAt(10_000_000)
        );
        assert_eq!(
            sched.record_failure(&policy, 0),
            RetryDecision::RetryAt(20_000_000)
        );
        assert_eq!(
            sched.record_failure(&policy, 0),
            RetryDecision::RetryAt(40_000_000)
        );
        assert_eq!(sched.record_failure(&policy, 0), RetryDecision::GiveUp);
        sched.reset();
        assert!(sched.due(0));
        assert_eq!(sched.failures(), 0);
    }

    #[test]
    fn vanished_device_is_classified_for_reconnect() {
        assert!(device_gone(&io::Error::from_raw_os_error(19))); // ENODEV
        assert!(device_gone(&io::Error::from_raw_os_error(6))); // ENXIO
        assert!(!device_gone(&io::Error::from_raw_os_error(4))); // EINTR
    }

    #[test]
    fn missing_axis_keeps_machine_disarmed() {
        let mut m = disarmed_after_drop();
        let config = InputConfig::extreme_3d_pro();
        let mut dev = FakeDevice::extreme_3d([0, 511, 127, 255, 0, 0], &[]);
        // Device lacks the throttle: evdev reports all-zero absinfo.
        dev.axes.remove(&code::ABS_THROTTLE);
        assert!(matches!(
            attempt_resync(&mut m, &mut dev, &config, 10_000, 8),
            ResyncOutcome::Failed(ResyncFailure::Rejected(MachineError::ResyncMissingAxis {
                code: code::ABS_THROTTLE
            }))
        ));
        assert_stays_disarmed(&mut m, 8);
    }

    #[test]
    fn partial_read_never_becomes_a_snapshot() {
        let mut m = disarmed_after_drop();
        let config = InputConfig::extreme_3d_pro();
        let mut dev = FakeDevice::extreme_3d([0, 511, 127, 255, 0, 0], &[]);
        // Fails midway through the axes: X succeeds, Y fails.
        struct FailSecond(FakeDevice);
        impl DeviceStateSource for FailSecond {
            fn abs_info(&mut self, code: u16) -> io::Result<InputAbsinfo> {
                if self.0.abs_calls == 1 {
                    self.0.abs_calls += 1;
                    return Err(io::Error::from_raw_os_error(5));
                }
                self.0.abs_info(code)
            }
            fn key_bitmap(&mut self, bits: &mut [u8; KEY_BITMAP_BYTES]) -> io::Result<()> {
                self.0.key_bitmap(bits)
            }
        }
        dev.abs_calls = 0;
        let mut dev = FailSecond(dev);
        assert!(matches!(
            attempt_resync(&mut m, &mut dev, &config, 10_000, 8),
            ResyncOutcome::Failed(ResyncFailure::Ioctl(SnapshotIoctl::Abs(code::ABS_Y), _))
        ));
        assert_stays_disarmed(&mut m, 8);
    }

    #[test]
    fn successful_snapshot_rearms_with_snapshot_values_not_stale_ones() {
        let mut m = disarmed_after_drop();
        let config = InputConfig::extreme_3d_pro();
        // Real state now: stick full left, throttle idle (raw 255 inverted),
        // trigger released, button 3 held. Pre-drop was roll +1, throttle 1,
        // trigger held.
        let held = code::BTN_TRIGGER + 3;
        let mut dev = FakeDevice::extreme_3d([0, 511, 127, 255, 1, 0], &[held, 0x2ff]);
        let ResyncOutcome::Rearmed {
            live,
            snapshot,
            frame,
        } = attempt_resync(&mut m, &mut dev, &config, 10_000, 8)
        else {
            panic!("complete snapshot must re-arm");
        };
        assert_eq!(dev.abs_calls, config.axes.len());
        assert_eq!(dev.key_calls, 1);
        assert_eq!(
            live.axes[0],
            AxisState {
                code: code::ABS_X,
                value: 0,
                minimum: 0,
                maximum: 1023
            }
        );
        // Unconfigured keys (0x2ff) are not reported to the machine.
        assert_eq!(snapshot.pressed_buttons, vec![held]);
        assert_eq!(snapshot.source_event_sequence, 8);
        assert_eq!(frame.timestamp_ns, 10_000);
        assert_eq!(frame.source_event_sequence, 8);
        assert_eq!(frame.roll, -1.0);
        assert_eq!(frame.throttle, 0.0);
        assert_eq!(frame.hat_x, 1.0);
        assert_eq!(frame.buttons, 1 << 3);
        assert_eq!(frame.buttons_pressed, 0, "snapshot is state, not an edge");
        // The next live report carries the snapshot state, not pre-drop state.
        let next = m
            .apply(InputEvent {
                timestamp_ns: 10_001,
                sequence: 9,
                event_type: code::EV_SYN,
                code: code::SYN_REPORT,
                value: 0,
            })
            .unwrap()
            .unwrap();
        assert_eq!(next.sequence, frame.sequence + 1);
        assert_eq!(
            (next.roll, next.throttle, next.buttons),
            (-1.0, 0.0, 1 << 3)
        );
    }

    #[test]
    fn snapshot_while_still_dropping_is_rejected() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply_detailed(ev(1, code::EV_SYN, code::SYN_DROPPED, 0))
            .unwrap();
        assert!(!m.awaiting_snapshot(), "boundary not reached yet");
        let mut dev = FakeDevice::extreme_3d([511, 511, 127, 255, 0, 0], &[]);
        assert!(matches!(
            attempt_resync(&mut m, &mut dev, &InputConfig::extreme_3d_pro(), 10_000, 2),
            ResyncOutcome::Failed(ResyncFailure::Rejected(MachineError::ResyncNotRequired))
        ));
    }
}

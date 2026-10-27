//! Deterministic flight-control input processing.
//!
//! The platform-independent [`ControlMachine`] consumes canonical events and
//! publishes one immutable [`ControlFrame`] per evdev `SYN_REPORT`. Linux I/O
//! lives in the target-gated `linux` module and is deliberately nonblocking.
//! Recording the canonical events (including their timestamp and sequence) is
//! sufficient for replay.

use std::collections::BTreeMap;

/// Logitech's USB identity for the Extreme 3D Pro.
pub const LOGITECH_VENDOR_ID: u16 = 0x046d;
pub const EXTREME_3D_PRO_PRODUCT_ID: u16 = 0xc215;

/// Canonical evdev event types/codes used by the default profile.
pub mod code {
    pub const EV_SYN: u16 = 0x00;
    pub const EV_KEY: u16 = 0x01;
    pub const EV_ABS: u16 = 0x03;
    pub const SYN_REPORT: u16 = 0;
    /// Kernel notification that this client's evdev queue overflowed.
    pub const SYN_DROPPED: u16 = 3;
    pub const ABS_X: u16 = 0;
    pub const ABS_Y: u16 = 1;
    pub const ABS_RZ: u16 = 5;
    pub const ABS_THROTTLE: u16 = 6;
    pub const ABS_HAT0X: u16 = 16;
    pub const ABS_HAT0Y: u16 = 17;
    pub const BTN_TRIGGER: u16 = 0x120;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceIdentity {
    pub event_path: String,
    pub name: String,
    pub vendor_id: u16,
    pub product_id: u16,
    pub version: u16,
    pub physical_path: Option<String>,
    pub unique_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceMatcher {
    pub vendor_id: u16,
    pub product_id: u16,
    /// Optional case-insensitive substring. USB IDs remain authoritative.
    pub name_contains: Option<String>,
}

impl DeviceMatcher {
    pub fn extreme_3d_pro() -> Self {
        Self {
            vendor_id: LOGITECH_VENDOR_ID,
            product_id: EXTREME_3D_PRO_PRODUCT_ID,
            name_contains: Some("Extreme 3D".into()),
        }
    }

    pub fn matches(&self, device: &DeviceIdentity) -> bool {
        if device.vendor_id != self.vendor_id || device.product_id != self.product_id {
            return false;
        }
        self.name_contains.as_ref().map_or(true, |needle| {
            device.name.to_lowercase().contains(&needle.to_lowercase())
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisCalibration {
    pub min: i32,
    pub center: i32,
    pub max: i32,
    /// Fraction of travel removed around center, in `[0, 0.95]`.
    pub deadzone: f32,
    /// Blend from linear (0) to cubic (1), in `[0, 1]`.
    pub expo: f32,
    pub invert: bool,
}

impl AxisCalibration {
    pub fn bipolar(self, raw: i32) -> f32 {
        let side = if raw >= self.center {
            (raw - self.center) as f32 / (self.max - self.center).max(1) as f32
        } else {
            (raw - self.center) as f32 / (self.center - self.min).max(1) as f32
        };
        let mut x = side.clamp(-1.0, 1.0);
        if self.invert {
            x = -x;
        }
        let deadzone = self.deadzone.clamp(0.0, 0.95);
        let magnitude = x.abs();
        x = if magnitude <= deadzone {
            0.0
        } else {
            x.signum() * ((magnitude - deadzone) / (1.0 - deadzone))
        };
        let expo = self.expo.clamp(0.0, 1.0);
        ((1.0 - expo) * x + expo * x * x * x).clamp(-1.0, 1.0)
    }

    pub fn unipolar(self, raw: i32) -> f32 {
        let mut x = (raw - self.min) as f32 / (self.max - self.min).max(1) as f32;
        x = x.clamp(0.0, 1.0);
        if self.invert {
            x = 1.0 - x;
        }
        x
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlAxis {
    Roll,
    Pitch,
    Yaw,
    Throttle,
    BrakeLeft,
    BrakeRight,
    HatX,
    HatY,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxisRange {
    Bipolar,
    Unipolar,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisBinding {
    pub evdev_code: u16,
    pub control: ControlAxis,
    pub range: AxisRange,
    pub calibration: AxisCalibration,
}

#[derive(Clone, Debug, PartialEq)]
pub struct InputConfig {
    pub matcher: DeviceMatcher,
    pub axes: Vec<AxisBinding>,
    /// Logical button index -> evdev key code.
    pub buttons: Vec<u16>,
    pub reconnect_interval_ns: u64,
    /// Maximum records consumed while searching for the report boundary after
    /// `SYN_DROPPED`. Reaching either bound forces a closed-source resync.
    pub max_resync_events: u32,
    pub max_resync_wait_ns: u64,
}

impl InputConfig {
    /// INPUT-003 dead zone of the 10-bit roll and pitch axes (fraction of travel).
    pub const ROLL_PITCH_DEADZONE: f32 = 0.020;
    /// INPUT-003 dead zone of the 8-bit twist (yaw) axis.
    pub const YAW_DEADZONE: f32 = 0.063;

    /// Baseline profile for the commonly reported Extreme 3D Pro ranges.
    /// Production startup should replace these with measured EVIOCGABS values
    /// or saved user calibration before enabling controls.
    pub fn extreme_3d_pro() -> Self {
        // INPUT-003: linear (expo 0) on every flight axis; JSBSim's FCS
        // already maps commands linearly to the surfaces. The dead zones are
        // the measured released-stick deflection plus one raw count
        // (tools/analyze_m1_stick_jitter.py on the TEST-028 probe log): the
        // roll/pitch gimbal rests up to 9 counts (1.76 %) off 511, the twist
        // jitters 127..134 (5.47 %) hands off.
        let bipolar_10 = AxisCalibration {
            min: 0,
            center: 511,
            max: 1023,
            deadzone: Self::ROLL_PITCH_DEADZONE,
            expo: 0.0,
            invert: false,
        };
        let bipolar_8 = AxisCalibration {
            min: 0,
            center: 127,
            max: 255,
            deadzone: Self::YAW_DEADZONE,
            expo: 0.0,
            invert: false,
        };
        Self {
            matcher: DeviceMatcher::extreme_3d_pro(),
            axes: vec![
                AxisBinding {
                    evdev_code: code::ABS_X,
                    control: ControlAxis::Roll,
                    range: AxisRange::Bipolar,
                    calibration: bipolar_10,
                },
                // The physical Extreme 3D Pro reports ABS_Y increasing as
                // the stick moves toward the pilot. Therefore pull-back is
                // positive pitch without inversion. S2's 2026-10-17 live
                // sign check caught and confirmed the historical reversal.
                AxisBinding {
                    evdev_code: code::ABS_Y,
                    control: ControlAxis::Pitch,
                    range: AxisRange::Bipolar,
                    calibration: bipolar_10,
                },
                AxisBinding {
                    evdev_code: code::ABS_RZ,
                    control: ControlAxis::Yaw,
                    range: AxisRange::Bipolar,
                    calibration: bipolar_8,
                },
                AxisBinding {
                    evdev_code: code::ABS_THROTTLE,
                    control: ControlAxis::Throttle,
                    range: AxisRange::Unipolar,
                    calibration: AxisCalibration {
                        invert: true,
                        ..bipolar_8
                    },
                },
                AxisBinding {
                    evdev_code: code::ABS_HAT0X,
                    control: ControlAxis::HatX,
                    range: AxisRange::Bipolar,
                    calibration: AxisCalibration {
                        min: -1,
                        center: 0,
                        max: 1,
                        deadzone: 0.0,
                        expo: 0.0,
                        invert: false,
                    },
                },
                AxisBinding {
                    evdev_code: code::ABS_HAT0Y,
                    control: ControlAxis::HatY,
                    range: AxisRange::Bipolar,
                    calibration: AxisCalibration {
                        min: -1,
                        center: 0,
                        max: 1,
                        deadzone: 0.0,
                        expo: 0.0,
                        invert: false,
                    },
                },
            ],
            buttons: (code::BTN_TRIGGER..code::BTN_TRIGGER + 12).collect(),
            reconnect_interval_ns: 1_000_000_000,
            max_resync_events: 256,
            max_resync_wait_ns: 100_000_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InputEvent {
    /// Monotonic time assigned at ingestion; replay preserves it verbatim.
    pub timestamp_ns: u64,
    /// Strictly increasing within a connection. Detects dropped records.
    pub sequence: u64,
    pub event_type: u16,
    pub code: u16,
    pub value: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ControlFrame {
    pub timestamp_ns: u64,
    pub sequence: u64,
    pub source_event_sequence: u64,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
    pub brake_left: f32,
    pub brake_right: f32,
    pub hat_x: f32,
    pub hat_y: f32,
    /// Bit N is logical button N. The profile is capped at 64 buttons.
    pub buttons: u64,
    /// Rising/falling edges observed since the preceding `SYN_REPORT`.
    pub buttons_pressed: u64,
    pub buttons_released: u64,
}

impl Default for ControlFrame {
    fn default() -> Self {
        Self {
            timestamp_ns: 0,
            sequence: 0,
            source_event_sequence: 0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            brake_left: 0.0,
            brake_right: 0.0,
            hat_x: 0.0,
            hat_y: 0.0,
            buttons: 0,
            buttons_pressed: 0,
            buttons_released: 0,
        }
    }
}

/// Pure deterministic event-to-frame reducer. No I/O, clocks or allocation on
/// the event path after construction.
pub struct ControlMachine {
    config: InputConfig,
    axis_index: BTreeMap<u16, usize>,
    button_index: BTreeMap<u16, usize>,
    pending: ControlFrame,
    next_frame_sequence: u64,
    last_event_sequence: Option<u64>,
    last_event_timestamp_ns: Option<u64>,
    sync: SyncState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SyncState {
    Synchronized,
    Dropping {
        loss: InputLoss,
        discarded_events: u32,
    },
    Disarmed {
        loss: InputLoss,
        discarded_events: u32,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InputLoss {
    pub timestamp_ns: u64,
    pub source_event_sequence: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResyncTrigger {
    ReportBoundary,
    EventLimit,
    Timeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResyncRequired {
    pub loss: InputLoss,
    pub discarded_events: u32,
    pub trigger: ResyncTrigger,
}

#[derive(Clone, Debug, PartialEq)]
pub enum MachineOutput {
    None,
    Frame(ControlFrame),
    LossDetected(InputLoss),
    ResyncRequired(ResyncRequired),
}

/// Complete device state obtained out of band (normally EVIOCGABS and
/// EVIOCGKEY). Resync rejects snapshots missing any configured absolute axis.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceSnapshot {
    pub timestamp_ns: u64,
    pub source_event_sequence: u64,
    pub axes: Vec<(u16, i32)>,
    pub pressed_buttons: Vec<u16>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MachineError {
    NonMonotonicEventSequence { previous: u64, received: u64 },
    NonMonotonicTimestamp { previous: u64, received: u64 },
    ResyncNotRequired,
    ResyncMissingAxis { code: u16 },
    ResyncDuplicateAxis { code: u16 },
    ResyncUnknownButton { code: u16 },
    ResyncDeviceNotConnected,
}

impl ControlMachine {
    pub fn new(config: InputConfig) -> Self {
        let axis_index = config
            .axes
            .iter()
            .enumerate()
            .map(|(i, a)| (a.evdev_code, i))
            .collect();
        let button_index = config
            .buttons
            .iter()
            .copied()
            .take(64)
            .enumerate()
            .map(|(i, c)| (c, i))
            .collect();
        Self {
            config,
            axis_index,
            button_index,
            pending: ControlFrame::default(),
            next_frame_sequence: 0,
            last_event_sequence: None,
            last_event_timestamp_ns: None,
            sync: SyncState::Synchronized,
        }
    }

    /// Applies one event. A complete frame is returned only at `SYN_REPORT`.
    pub fn apply(&mut self, event: InputEvent) -> Result<Option<ControlFrame>, MachineError> {
        Ok(match self.apply_detailed(event)? {
            MachineOutput::Frame(frame) => Some(frame),
            _ => None,
        })
    }

    /// Detailed form used by platform adapters to record loss/resync telemetry.
    pub fn apply_detailed(&mut self, event: InputEvent) -> Result<MachineOutput, MachineError> {
        if let Some(previous) = self.last_event_sequence {
            if event.sequence <= previous {
                return Err(MachineError::NonMonotonicEventSequence {
                    previous,
                    received: event.sequence,
                });
            }
        }
        if let Some(previous) = self.last_event_timestamp_ns {
            if event.timestamp_ns < previous {
                return Err(MachineError::NonMonotonicTimestamp {
                    previous,
                    received: event.timestamp_ns,
                });
            }
        }
        self.last_event_sequence = Some(event.sequence);
        self.last_event_timestamp_ns = Some(event.timestamp_ns);
        self.pending.source_event_sequence = event.sequence;

        if event.event_type == code::EV_SYN && event.code == code::SYN_DROPPED {
            let loss = InputLoss {
                timestamp_ns: event.timestamp_ns,
                source_event_sequence: event.sequence,
            };
            self.clear_controls();
            self.sync = SyncState::Dropping {
                loss,
                discarded_events: 0,
            };
            return Ok(MachineOutput::LossDetected(loss));
        }

        match self.sync {
            SyncState::Dropping {
                loss,
                discarded_events,
            } => {
                let discarded_events = discarded_events.saturating_add(1);
                let trigger = if event.event_type == code::EV_SYN && event.code == code::SYN_REPORT
                {
                    Some(ResyncTrigger::ReportBoundary)
                } else if discarded_events >= self.config.max_resync_events {
                    Some(ResyncTrigger::EventLimit)
                } else if event.timestamp_ns.saturating_sub(loss.timestamp_ns)
                    >= self.config.max_resync_wait_ns
                {
                    Some(ResyncTrigger::Timeout)
                } else {
                    None
                };
                if let Some(trigger) = trigger {
                    self.sync = SyncState::Disarmed {
                        loss,
                        discarded_events,
                    };
                    return Ok(MachineOutput::ResyncRequired(ResyncRequired {
                        loss,
                        discarded_events,
                        trigger,
                    }));
                }
                self.sync = SyncState::Dropping {
                    loss,
                    discarded_events,
                };
                return Ok(MachineOutput::None);
            }
            SyncState::Disarmed { .. } => return Ok(MachineOutput::None),
            SyncState::Synchronized => {}
        }

        if event.event_type == code::EV_ABS {
            if let Some(&index) = self.axis_index.get(&event.code) {
                let binding = self.config.axes[index];
                let value = match binding.range {
                    AxisRange::Bipolar => binding.calibration.bipolar(event.value),
                    AxisRange::Unipolar => binding.calibration.unipolar(event.value),
                };
                match binding.control {
                    ControlAxis::Roll => self.pending.roll = value,
                    ControlAxis::Pitch => self.pending.pitch = value,
                    ControlAxis::Yaw => self.pending.yaw = value,
                    ControlAxis::Throttle => self.pending.throttle = value,
                    ControlAxis::BrakeLeft => self.pending.brake_left = value,
                    ControlAxis::BrakeRight => self.pending.brake_right = value,
                    ControlAxis::HatX => self.pending.hat_x = value,
                    ControlAxis::HatY => self.pending.hat_y = value,
                }
            }
        } else if event.event_type == code::EV_KEY {
            if let Some(&index) = self.button_index.get(&event.code) {
                let mask = 1_u64 << index;
                let was_pressed = self.pending.buttons & mask != 0;
                let is_pressed = event.value != 0;
                if is_pressed {
                    self.pending.buttons |= mask;
                    if !was_pressed {
                        self.pending.buttons_pressed |= mask;
                    }
                } else {
                    self.pending.buttons &= !mask;
                    if was_pressed {
                        self.pending.buttons_released |= mask;
                    }
                }
            }
        } else if event.event_type == code::EV_SYN && event.code == code::SYN_REPORT {
            let mut frame = self.pending.clone();
            frame.timestamp_ns = event.timestamp_ns;
            frame.sequence = self.next_frame_sequence;
            self.next_frame_sequence = self.next_frame_sequence.wrapping_add(1);
            self.pending.sequence = self.next_frame_sequence;
            self.pending.buttons_pressed = 0;
            self.pending.buttons_released = 0;
            return Ok(MachineOutput::Frame(frame));
        }
        Ok(MachineOutput::None)
    }

    /// Enforces the time bound even when the device produces no more records.
    pub fn check_resync_timeout(&mut self, now_ns: u64) -> Option<ResyncRequired> {
        let SyncState::Dropping {
            loss,
            discarded_events,
        } = self.sync
        else {
            return None;
        };
        if now_ns.saturating_sub(loss.timestamp_ns) < self.config.max_resync_wait_ns {
            return None;
        }
        self.sync = SyncState::Disarmed {
            loss,
            discarded_events,
        };
        Some(ResyncRequired {
            loss,
            discarded_events,
            trigger: ResyncTrigger::Timeout,
        })
    }

    pub fn needs_resync(&self) -> bool {
        !matches!(self.sync, SyncState::Synchronized)
    }

    /// True once the dropped-stream discard boundary has been reached and only
    /// a complete snapshot ([`Self::resynchronize`]) can re-arm output.
    pub fn awaiting_snapshot(&self) -> bool {
        matches!(self.sync, SyncState::Disarmed { .. })
    }

    pub fn pending_loss(&self) -> Option<InputLoss> {
        match self.sync {
            SyncState::Synchronized => None,
            SyncState::Dropping { loss, .. } | SyncState::Disarmed { loss, .. } => Some(loss),
        }
    }

    /// Re-arms only from a complete, externally measured device snapshot.
    pub fn resynchronize(
        &mut self,
        snapshot: DeviceSnapshot,
    ) -> Result<ControlFrame, MachineError> {
        if !matches!(self.sync, SyncState::Disarmed { .. }) {
            return Err(MachineError::ResyncNotRequired);
        }
        if let Some(previous) = self.last_event_sequence {
            if snapshot.source_event_sequence <= previous {
                return Err(MachineError::NonMonotonicEventSequence {
                    previous,
                    received: snapshot.source_event_sequence,
                });
            }
        }
        if let Some(previous) = self.last_event_timestamp_ns {
            if snapshot.timestamp_ns < previous {
                return Err(MachineError::NonMonotonicTimestamp {
                    previous,
                    received: snapshot.timestamp_ns,
                });
            }
        }
        let mut values = BTreeMap::new();
        for (code, value) in snapshot.axes {
            if values.insert(code, value).is_some() {
                return Err(MachineError::ResyncDuplicateAxis { code });
            }
        }
        for binding in &self.config.axes {
            if !values.contains_key(&binding.evdev_code) {
                return Err(MachineError::ResyncMissingAxis {
                    code: binding.evdev_code,
                });
            }
        }
        let mut pressed = 0_u64;
        for code in snapshot.pressed_buttons {
            let Some(&index) = self.button_index.get(&code) else {
                return Err(MachineError::ResyncUnknownButton { code });
            };
            pressed |= 1_u64 << index;
        }
        self.clear_controls();
        for binding in &self.config.axes {
            let raw = values[&binding.evdev_code];
            let value = match binding.range {
                AxisRange::Bipolar => binding.calibration.bipolar(raw),
                AxisRange::Unipolar => binding.calibration.unipolar(raw),
            };
            match binding.control {
                ControlAxis::Roll => self.pending.roll = value,
                ControlAxis::Pitch => self.pending.pitch = value,
                ControlAxis::Yaw => self.pending.yaw = value,
                ControlAxis::Throttle => self.pending.throttle = value,
                ControlAxis::BrakeLeft => self.pending.brake_left = value,
                ControlAxis::BrakeRight => self.pending.brake_right = value,
                ControlAxis::HatX => self.pending.hat_x = value,
                ControlAxis::HatY => self.pending.hat_y = value,
            }
        }
        self.pending.buttons = pressed;
        self.pending.timestamp_ns = snapshot.timestamp_ns;
        self.pending.source_event_sequence = snapshot.source_event_sequence;
        self.pending.sequence = self.next_frame_sequence;
        let frame = self.pending.clone();
        self.next_frame_sequence = self.next_frame_sequence.wrapping_add(1);
        self.pending.sequence = self.next_frame_sequence;
        self.last_event_sequence = Some(snapshot.source_event_sequence);
        self.last_event_timestamp_ns = Some(snapshot.timestamp_ns);
        self.sync = SyncState::Synchronized;
        Ok(frame)
    }

    fn clear_controls(&mut self) {
        let next = self.next_frame_sequence;
        self.pending = ControlFrame {
            sequence: next,
            ..ControlFrame::default()
        };
    }

    /// Clears latched controls on disconnect so a stale throttle/button cannot
    /// survive reconnection. Frame and event sequences intentionally continue.
    pub fn disconnect(&mut self) {
        self.clear_controls();
        if let SyncState::Dropping {
            loss,
            discarded_events,
        } = self.sync
        {
            self.sync = SyncState::Disarmed {
                loss,
                discarded_events,
            };
        }
        self.last_event_sequence = None;
        self.last_event_timestamp_ns = None;
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connected(DeviceIdentity),
    /// Device has reopened after loss, but frames remain suppressed until an
    /// EVIOCGABS/EVIOCGKEY snapshot is supplied.
    ResyncRequired {
        device: DeviceIdentity,
        loss: InputLoss,
    },
    Error {
        message: String,
        retry_at_ns: u64,
    },
}

#[cfg(target_os = "linux")]
pub mod linux;
pub mod mouse;
pub mod resync;

#[cfg(test)]
mod tests {
    use super::*;

    fn event(sequence: u64, event_type: u16, code: u16, value: i32) -> InputEvent {
        InputEvent {
            timestamp_ns: 1_000 + sequence,
            sequence,
            event_type,
            code,
            value,
        }
    }

    #[test]
    fn matches_real_usb_identity_not_name_alone() {
        let matcher = DeviceMatcher::extreme_3d_pro();
        let mut d = DeviceIdentity {
            event_path: "/dev/input/event7".into(),
            name: "Logitech Extreme 3D".into(),
            vendor_id: 0x046d,
            product_id: 0xc215,
            version: 0x110,
            physical_path: None,
            unique_id: None,
        };
        assert!(matcher.matches(&d));
        d.product_id = 1;
        assert!(!matcher.matches(&d));
    }

    #[test]
    fn calibration_deadzone_expo_and_clamp_are_stable() {
        let c = AxisCalibration {
            min: 0,
            center: 50,
            max: 100,
            deadzone: 0.1,
            expo: 0.5,
            invert: false,
        };
        assert_eq!(c.bipolar(50), 0.0);
        assert_eq!(c.bipolar(55), 0.0);
        assert_eq!(c.bipolar(-100), -1.0);
        assert_eq!(c.bipolar(200), 1.0);
        let after_deadzone: f32 = (0.54 - 0.1) / 0.9;
        let expected = 0.5 * (after_deadzone + after_deadzone.powi(3));
        assert!((c.bipolar(77) - expected).abs() < 0.001);
    }

    #[test]
    fn input003_profile_is_linear_with_the_measured_dead_zones() {
        let cfg = InputConfig::extreme_3d_pro();
        for b in &cfg.axes {
            assert_eq!(b.calibration.expo, 0.0, "{:?} must be linear", b.control);
        }
        let cal = |c: ControlAxis| cfg.axes.iter().find(|b| b.control == c).unwrap().calibration;
        let (roll, pitch, yaw) = (cal(ControlAxis::Roll), cal(ControlAxis::Pitch), cal(ControlAxis::Yaw));
        assert_eq!((roll.deadzone, pitch.deadzone, yaw.deadzone), (0.020, 0.020, 0.063));
        // Released-stick rests from the TEST-028 probe stay at zero ...
        assert_eq!(pitch.bipolar(520), 0.0, "pitch rest +9 counts (1.76 %)");
        assert_eq!(roll.bipolar(509), 0.0);
        assert_eq!(yaw.bipolar(134), 0.0, "twist hands-off jitter up to 134 (5.47 %)");
        // ... and just outside them the response is linear from zero.
        let just_out = pitch.bipolar(511 + 12); // 2.34 %
        assert!(just_out > 0.0 && just_out < 0.005, "{just_out}");
        let half = roll.bipolar(511 + 256); // 50 % deflection
        assert!((half - (0.5 - 0.02) / 0.98).abs() < 1e-6, "linear: {half}");
        assert_eq!((roll.bipolar(1023), roll.bipolar(0)), (1.0, -1.0));
        assert_eq!((yaw.bipolar(255), yaw.bipolar(0)), (1.0, -1.0));
    }

    #[test]
    fn one_frame_per_syn_report_with_latched_state() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        assert_eq!(
            m.apply(event(1, code::EV_ABS, code::ABS_X, 1023)).unwrap(),
            None
        );
        assert_eq!(
            m.apply(event(2, code::EV_KEY, code::BTN_TRIGGER, 1))
                .unwrap(),
            None
        );
        let first = m
            .apply(event(3, code::EV_SYN, code::SYN_REPORT, 0))
            .unwrap()
            .unwrap();
        assert_eq!(first.sequence, 0);
        assert_eq!(first.source_event_sequence, 3);
        assert_eq!(first.timestamp_ns, 1003);
        assert_eq!(first.roll, 1.0);
        assert_eq!(first.buttons, 1);
        assert_eq!(first.buttons_pressed, 1);
        let second = m
            .apply(event(4, code::EV_SYN, code::SYN_REPORT, 0))
            .unwrap()
            .unwrap();
        assert_eq!(second.sequence, 1);
        assert_eq!(second.roll, 1.0);
        assert_eq!(second.buttons, 1);
        assert_eq!(second.buttons_pressed, 0);
        assert_eq!(second.buttons_released, 0);
    }

    #[test]
    fn pitch_and_throttle_default_directions_are_flight_friendly() {
        // Stick fully back (ABS_Y max) and throttle fully forward
        // (ABS_THROTTLE min) must mean nose-up and full power.
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(event(1, code::EV_ABS, code::ABS_Y, 1023)).unwrap();
        m.apply(event(2, code::EV_ABS, code::ABS_THROTTLE, 0))
            .unwrap();
        let f = m.apply(event(3, code::EV_SYN, 0, 0)).unwrap().unwrap();
        assert_eq!(f.pitch, 1.0);
        assert_eq!(f.throttle, 1.0);

        m.apply(event(4, code::EV_ABS, code::ABS_Y, 0)).unwrap();
        let f = m.apply(event(5, code::EV_SYN, 0, 0)).unwrap().unwrap();
        assert_eq!(f.pitch, -1.0, "stick fully forward must command nose down");
    }

    #[test]
    fn rejects_duplicate_or_out_of_order_events() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(event(8, code::EV_KEY, code::BTN_TRIGGER, 1))
            .unwrap();
        assert_eq!(
            m.apply(event(8, code::EV_SYN, 0, 0)),
            Err(MachineError::NonMonotonicEventSequence {
                previous: 8,
                received: 8
            })
        );
    }

    #[test]
    fn rejects_a_timestamp_that_moves_backwards() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(event(8, code::EV_KEY, code::BTN_TRIGGER, 1))
            .unwrap();
        let mut older = event(9, code::EV_SYN, 0, 0);
        older.timestamp_ns = 1;
        assert_eq!(
            m.apply(older),
            Err(MachineError::NonMonotonicTimestamp {
                previous: 1008,
                received: 1
            })
        );
    }

    #[test]
    fn press_and_release_in_one_report_preserves_both_edges() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(event(1, code::EV_KEY, code::BTN_TRIGGER, 1))
            .unwrap();
        m.apply(event(2, code::EV_KEY, code::BTN_TRIGGER, 0))
            .unwrap();
        let f = m.apply(event(3, code::EV_SYN, 0, 0)).unwrap().unwrap();
        assert_eq!(f.buttons, 0);
        assert_eq!(f.buttons_pressed, 1);
        assert_eq!(f.buttons_released, 1);
    }

    #[test]
    fn disconnect_clears_controls_but_frame_sequence_continues() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(event(1, code::EV_KEY, code::BTN_TRIGGER, 1))
            .unwrap();
        assert_eq!(
            m.apply(event(2, code::EV_SYN, 0, 0))
                .unwrap()
                .unwrap()
                .sequence,
            0
        );
        m.disconnect();
        let f = m.apply(event(1, code::EV_SYN, 0, 0)).unwrap().unwrap();
        assert_eq!(f.sequence, 1);
        assert_eq!(f.buttons, 0);
        assert_eq!(f.throttle, 0.0);
    }

    #[test]
    fn replay_is_bit_for_bit_deterministic() {
        let stream = [
            event(1, code::EV_ABS, code::ABS_X, 123),
            event(2, code::EV_ABS, code::ABS_RZ, 240),
            event(3, code::EV_KEY, code::BTN_TRIGGER + 2, 1),
            event(4, code::EV_SYN, 0, 0),
            event(5, code::EV_KEY, code::BTN_TRIGGER + 2, 0),
            event(6, code::EV_SYN, 0, 0),
        ];
        let run = || {
            let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
            stream
                .iter()
                .filter_map(|e| m.apply(*e).unwrap())
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn syn_dropped_discards_partial_report_and_disarms_at_boundary() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply(event(1, code::EV_ABS, code::ABS_X, 1023)).unwrap();
        let loss = InputLoss {
            timestamp_ns: 1002,
            source_event_sequence: 2,
        };
        assert_eq!(
            m.apply_detailed(event(2, code::EV_SYN, code::SYN_DROPPED, 0)),
            Ok(MachineOutput::LossDetected(loss))
        );
        assert_eq!(
            m.apply_detailed(event(3, code::EV_ABS, code::ABS_Y, 0)),
            Ok(MachineOutput::None)
        );
        assert_eq!(
            m.apply_detailed(event(4, code::EV_SYN, code::SYN_REPORT, 0)),
            Ok(MachineOutput::ResyncRequired(ResyncRequired {
                loss,
                discarded_events: 2,
                trigger: ResyncTrigger::ReportBoundary,
            }))
        );
        // Even a plausible complete-looking report after the boundary cannot
        // escape while state is unknown.
        assert_eq!(
            m.apply(event(5, code::EV_ABS, code::ABS_X, 0)).unwrap(),
            None
        );
        assert_eq!(
            m.apply(event(6, code::EV_SYN, code::SYN_REPORT, 0))
                .unwrap(),
            None
        );
        assert!(m.needs_resync());
    }

    #[test]
    fn dropped_stream_has_event_and_time_bounds() {
        let mut config = InputConfig::extreme_3d_pro();
        config.max_resync_events = 2;
        config.max_resync_wait_ns = 10;
        let mut m = ControlMachine::new(config.clone());
        m.apply_detailed(event(1, code::EV_SYN, code::SYN_DROPPED, 0))
            .unwrap();
        assert_eq!(
            m.apply_detailed(event(2, code::EV_ABS, code::ABS_X, 1))
                .unwrap(),
            MachineOutput::None
        );
        assert!(matches!(
            m.apply_detailed(event(3, code::EV_ABS, code::ABS_Y, 1))
                .unwrap(),
            MachineOutput::ResyncRequired(ResyncRequired {
                trigger: ResyncTrigger::EventLimit,
                discarded_events: 2,
                ..
            })
        ));

        let mut m = ControlMachine::new(config);
        m.apply_detailed(event(1, code::EV_SYN, code::SYN_DROPPED, 0))
            .unwrap();
        assert!(matches!(
            m.check_resync_timeout(1011),
            Some(ResyncRequired {
                trigger: ResyncTrigger::Timeout,
                discarded_events: 0,
                ..
            })
        ));
    }

    #[test]
    fn only_complete_external_snapshot_can_rearm_after_loss() {
        let mut m = ControlMachine::new(InputConfig::extreme_3d_pro());
        m.apply_detailed(event(1, code::EV_SYN, code::SYN_DROPPED, 0))
            .unwrap();
        m.apply_detailed(event(2, code::EV_SYN, code::SYN_REPORT, 0))
            .unwrap();
        let incomplete = DeviceSnapshot {
            timestamp_ns: 2_000,
            source_event_sequence: 20,
            axes: vec![(code::ABS_X, 511)],
            pressed_buttons: vec![],
        };
        assert_eq!(
            m.resynchronize(incomplete),
            Err(MachineError::ResyncMissingAxis { code: code::ABS_Y })
        );
        assert!(m.needs_resync());

        let frame = m
            .resynchronize(DeviceSnapshot {
                timestamp_ns: 2_001,
                source_event_sequence: 21,
                axes: vec![
                    (code::ABS_X, 1023),
                    (code::ABS_Y, 511),
                    (code::ABS_RZ, 127),
                    (code::ABS_THROTTLE, 0),
                    (code::ABS_HAT0X, 0),
                    (code::ABS_HAT0Y, 0),
                ],
                pressed_buttons: vec![code::BTN_TRIGGER],
            })
            .unwrap();
        assert_eq!(frame.roll, 1.0);
        assert_eq!(frame.throttle, 1.0);
        assert_eq!(frame.buttons, 1);
        assert_eq!(frame.buttons_pressed, 0);
        assert!(!m.needs_resync());
        assert!(matches!(
            m.apply_detailed(InputEvent {
                timestamp_ns: 2_002,
                sequence: 22,
                event_type: code::EV_SYN,
                code: code::SYN_REPORT,
                value: 0,
            })
            .unwrap(),
            MachineOutput::Frame(_)
        ));
    }
}

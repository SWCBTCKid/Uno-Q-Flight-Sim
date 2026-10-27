//! Host-runnable vertical seam for input, simulation and world assets.
//!
//! This crate deliberately owns no device, clock, graphics or JSBSim adapter.
//! Platform loops provide complete input frames and monotonic timestamps.
//! [`InputPublisher`] is the I/O-thread-owned side of the gx-sim control
//! mailbox, [`SimLoop`] is the simulation-thread-owned part, and
//! [`OfflineApp`] drives both serially, through the mailbox, for the finite
//! dry run. See `docs/architecture/m1-vertical-skeleton.md`
//! for what is and is not demonstrated.

use std::sync::atomic::{fence, AtomicU32, Ordering};
use std::sync::Arc;

use gx_input::{ControlFrame, MachineOutput};
use gx_sim::{
    control_mailbox, AircraftSnapshot, AircraftState, Command, CommandEdges, ControlInput,
    ControlReader, ControlStatus, ControlWriter, CoreError, GeodeticDatum, GeodeticPosition,
    InputHealth, InputStatus, InputValidity, PauseReason, LocalEnuPose, PublishBusy, RunState, SimCore, SimulationBackend,
    SnapshotExchange, SnapshotPair, TickControls, UnavailableReason, Wgs84EnuFrame, SIM_HZ,
    STEP_SECONDS,
};
use gx_world::{read_world, FormatError, GeodeticPoint, Limits, WorldAsset};

pub mod s1;
pub mod s2;
pub mod s3;
pub mod s4;
pub mod spike002;

pub const RENDER_HZ: u32 = 30;
pub const SIM_TICKS_PER_RENDER: u32 = SIM_HZ / RENDER_HZ;

/// Logical Logitech button assignments at the application boundary.
pub mod button {
    pub const START: u8 = 0;
    pub const PAUSE: u8 = 1;
    pub const RESET: u8 = 2;
    /// S4: numbered button 5 (`BTN_TOP2`) switches chase/cockpit camera.
    /// Render-side only: it never reaches the simulation.
    pub const CAMERA: u8 = 4;
    /// Pitch trim (controls diagnosis): numbered base button 7 (`BTN_BASE`)
    /// held = trim nose **down**, numbered base button 8 (`BTN_BASE2`) held =
    /// trim nose **up**. Both held = no trim. See `gx_sim::trim`.
    pub const TRIM_NOSE_DOWN: u8 = 6;
    pub const TRIM_NOSE_UP: u8 = 7;
    /// INPUT-005 levers ([`crate::StickLevers`]), all press-edge triggered:
    /// numbered button 6 (`BTN_PINKIE`) cycles the speedbrake, base buttons
    /// 9 / 10 move the flap lever up / down one detent, base button 11
    /// toggles the gear.
    pub const SPEEDBRAKE: u8 = 5;
    pub const FLAPS_UP: u8 = 8;
    pub const FLAPS_DOWN: u8 = 9;
    pub const GEAR: u8 = 10;
}

/// The pitch trim switch from the held trim buttons: +1 nose up, -1 nose
/// down, 0 released or both held.
pub fn trim_from_frame(frame: &ControlFrame) -> f32 {
    let held = |b: u8| frame.buttons & (1 << b) != 0;
    match (held(button::TRIM_NOSE_UP), held(button::TRIM_NOSE_DOWN)) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

/// Converts the controller's documented normalized convention to the FDM
/// convention. Brakes pass through even though the initial stick profile has
/// no brake axes yet.
pub fn controls_from_frame(frame: &ControlFrame) -> ControlInput {
    ControlInput {
        roll: frame.roll,
        pitch: frame.pitch,
        yaw: frame.yaw,
        throttle: frame.throttle,
        left_brake: frame.brake_left,
        right_brake: frame.brake_right,
        trim: trim_from_frame(frame),
        // Replaced by `StickLevers` where the runtime binds the levers.
        levers: gx_sim::Levers::default(),
        // Replaced by the autopilot panel where the runtime binds one.
        ap: gx_sim::ApInput::default(),
    }
    .normalized()
}

/// Produces at most one transition. Safety actions win if simultaneous edges
/// arrive in one report: Reset, then Pause, then Start.
pub fn command_from_frame(frame: &ControlFrame) -> Option<Command> {
    let pressed = frame.buttons_pressed;
    if pressed & (1 << button::RESET) != 0 {
        Some(Command::Reset)
    } else if pressed & (1 << button::PAUSE) != 0 {
        Some(Command::Pause)
    } else if pressed & (1 << button::START) != 0 {
        Some(Command::Start)
    } else {
        None
    }
}

/// Start/Pause/Reset press edges in one report, for the control mailbox.
/// Unlike [`command_from_frame`] this keeps every edge; the mailbox reader
/// orders them (see `gx_sim::control_mailbox`).
pub fn edges_from_frame(frame: &ControlFrame) -> CommandEdges {
    let pressed = frame.buttons_pressed;
    CommandEdges {
        start: pressed & (1 << button::START) != 0,
        pause: pressed & (1 << button::PAUSE) != 0,
        reset: pressed & (1 << button::RESET) != 0,
    }
}

/// I/O-thread side of the control mailbox: turns gx-input results into
/// publications. Never blocks.
///
/// Fail-closed mapping: `LossDetected` publishes `InputLoss` and
/// `ResyncRequired` publishes `ResyncRequired`, both with neutral axes, at
/// once (not at the next heartbeat). The heartbeat also refuses to republish a
/// valid record while the caller says the machine needs resync, so a missed
/// output cannot keep stale valid axes alive.
///
/// With a panel ([`InputPublisher::with_panel`]) every valid publication
/// carries the panel's latest [`gx_sim::ApInput`], and
/// [`InputPublisher::republish_panel`] publishes a panel change at once,
/// but only on top of a valid record: never while the input is lost,
/// disconnected or needs resync (the next valid frame then carries it).
pub struct InputPublisher {
    writer: ControlWriter,
    parking_brake: Option<StickParkingBrake>,
    levers: Option<StickLevers>,
    panel: Option<ApPanelReader>,
    /// The panel value last stamped into a publication.
    ap: ApPanelValue,
}

impl InputPublisher {
    pub fn new(writer: ControlWriter) -> Self {
        Self { writer, parking_brake: None, levers: None, panel: None, ap: ApPanelValue::default() }
    }

    /// S2 parking-brake binding: numbered button 4 toggles both brakes.
    pub fn with_parking_brake(writer: ControlWriter, parking_brake: StickParkingBrake) -> Self {
        Self { parking_brake: Some(parking_brake), ..Self::new(writer) }
    }

    /// AP-001 autopilot panel (the mouse thread writes it).
    pub fn with_panel(mut self, panel: ApPanelReader) -> Self {
        self.panel = Some(panel);
        self
    }

    /// The panel value last published (for logs).
    pub fn panel_value(&self) -> ApPanelValue {
        self.ap
    }

    /// The panel's latest value, or the last one stamped if the read was
    /// torn on every retry (never a torn value).
    fn panel_now(&self) -> ApPanelValue {
        self.panel.as_ref().and_then(ApPanelReader::load).unwrap_or(self.ap)
    }

    /// Publish a panel change now, on top of the current valid record (its
    /// axes, levers, input sequence and timestamp; no command edges).
    /// Returns false (nothing published) when the panel is unchanged, the
    /// read was torn, or the current record is not valid or `needs_resync`
    /// (`ControlMachine::needs_resync()`): fail closed, as the heartbeat.
    pub fn republish_panel(&mut self, needs_resync: bool, now_ns: u64) -> bool {
        let Some(value) = self.panel.as_ref().and_then(ApPanelReader::load) else { return false };
        let r = *self.writer.current();
        if value.input == self.ap.input || needs_resync || r.validity != InputValidity::Valid {
            return false;
        }
        self.ap = value;
        let controls = ControlInput { ap: value.input, ..r.controls };
        self.writer.publish_valid(controls, r.input_sequence, r.input_timestamp_ns, CommandEdges::default(), now_ns);
        true
    }

    /// INPUT-005 gear/flap/speedbrake buttons. Without it every publication
    /// carries `Levers::default()` (gear down, clean).
    pub fn with_levers(mut self, levers: StickLevers) -> Self {
        self.levers = Some(levers);
        self
    }

    pub fn parking_brake(&self) -> Option<&StickParkingBrake> {
        self.parking_brake.as_ref()
    }

    pub fn levers(&self) -> Option<&StickLevers> {
        self.levers.as_ref()
    }

    pub fn publish_frame(&mut self, frame: &ControlFrame, now_ns: u64) {
        let mut controls = controls_from_frame(frame);
        if let Some(parking_brake) = self.parking_brake.as_mut() {
            parking_brake.apply(frame, &mut controls);
        }
        if let Some(levers) = self.levers.as_mut() {
            levers.apply(frame, &mut controls);
        }
        if self.panel.is_some() {
            self.ap = self.panel_now();
            controls.ap = self.ap.input;
        }
        self.writer.publish_valid(
            controls,
            frame.sequence,
            frame.timestamp_ns,
            edges_from_frame(frame),
            now_ns,
        );
    }

    /// Publish whatever a `ControlMachine::apply_detailed` call produced.
    pub fn publish_output(&mut self, output: &MachineOutput, now_ns: u64) {
        match output {
            MachineOutput::None => {}
            MachineOutput::Frame(frame) => self.publish_frame(frame, now_ns),
            MachineOutput::LossDetected(loss) => self.writer.publish_unavailable(
                UnavailableReason::InputLoss,
                loss.source_event_sequence,
                loss.timestamp_ns,
                now_ns,
            ),
            MachineOutput::ResyncRequired(required) => self.writer.publish_unavailable(
                UnavailableReason::ResyncRequired,
                required.loss.source_event_sequence,
                required.loss.timestamp_ns,
                now_ns,
            ),
        }
    }

    pub fn publish_disconnected(&mut self, now_ns: u64) {
        let r = *self.writer.current();
        self.writer.publish_unavailable(
            UnavailableReason::Disconnected,
            r.input_sequence,
            r.input_timestamp_ns,
            now_ns,
        );
    }

    /// Call at least every `gx_sim::HEARTBEAT_PERIOD_NS`. `needs_resync` is
    /// `ControlMachine::needs_resync()`.
    pub fn heartbeat(&mut self, needs_resync: bool, now_ns: u64) {
        let r = *self.writer.current();
        if needs_resync && r.validity == InputValidity::Valid {
            self.writer.publish_unavailable(
                UnavailableReason::ResyncRequired,
                r.input_sequence,
                r.input_timestamp_ns,
                now_ns,
            );
        } else {
            self.writer.heartbeat(now_ns);
        }
    }
}

/// One panel state: the [`gx_sim::ApInput`] and when the mouse thread
/// stored it (monotonic ns; the I/O log reports the publication latency).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ApPanelValue {
    pub input: gx_sim::ApInput,
    pub changed_at_ns: u64,
}

const AP_WORDS: usize = 7;

/// AP-001 I3: the autopilot panel shared from the mouse thread (one
/// [`ApPanelWriter`]) to the FIFO 40 I/O thread ([`ApPanelReader`]).
///
/// A seqlock over atomic words, so neither side ever blocks or takes a lock
/// and no access is a data race: the writer makes `seq` odd, stores the
/// words, then makes it even again (release); a reader loads `seq`
/// (acquire), the words, then `seq` again after an acquire fence, and
/// accepts the words only if both loads are the same even number. A torn
/// read is therefore detected and retried a bounded number of times, then
/// skipped ([`ApPanelReader::load`] returns `None`); it is never used.
pub struct ApShared {
    seq: AtomicU32,
    words: [AtomicU32; AP_WORDS],
}

fn ap_encode(v: &ApPanelValue) -> [u32; AP_WORDS] {
    let i = &v.input;
    [
        i.serial,
        u32::from(i.action.code()),
        i.heading_deg.to_bits(),
        i.altitude_ft.to_bits(),
        i.speed_kt.to_bits(),
        v.changed_at_ns as u32,
        (v.changed_at_ns >> 32) as u32,
    ]
}

fn ap_decode(w: [u32; AP_WORDS]) -> Option<ApPanelValue> {
    let action = gx_sim::ApAction::from_code(u8::try_from(w[1]).ok()?)?;
    let input = gx_sim::ApInput {
        serial: w[0],
        action,
        heading_deg: f32::from_bits(w[2]),
        altitude_ft: f32::from_bits(w[3]),
        speed_kt: f32::from_bits(w[4]),
    }
    .normalized();
    Some(ApPanelValue { input, changed_at_ns: u64::from(w[5]) | (u64::from(w[6]) << 32) })
}

/// Create the panel cell holding `initial`. The writer is unique (not
/// `Clone`); readers may be cloned.
pub fn ap_panel(initial: gx_sim::ApInput) -> (ApPanelWriter, ApPanelReader) {
    let value = ApPanelValue { input: initial.normalized(), changed_at_ns: 0 };
    let shared = Arc::new(ApShared { seq: AtomicU32::new(0), words: ap_encode(&value).map(AtomicU32::new) });
    (ApPanelWriter { shared: Arc::clone(&shared), value }, ApPanelReader { shared })
}

/// The mouse thread's handle: the only writer.
pub struct ApPanelWriter {
    shared: Arc<ApShared>,
    value: ApPanelValue,
}

impl ApPanelWriter {
    /// The value last stored.
    pub fn value(&self) -> ApPanelValue {
        self.value
    }

    /// Store `input` (normalized) stamped `now_ns`. Never blocks.
    pub fn store(&mut self, input: gx_sim::ApInput, now_ns: u64) {
        self.value = ApPanelValue { input: input.normalized(), changed_at_ns: now_ns };
        let s = &self.shared;
        let seq = s.seq.load(Ordering::Relaxed);
        s.seq.store(seq.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        for (w, v) in s.words.iter().zip(ap_encode(&self.value)) {
            w.store(v, Ordering::Relaxed);
        }
        s.seq.store(seq.wrapping_add(2), Ordering::Release);
    }
}

/// Reader handle (the I/O thread).
#[derive(Clone)]
pub struct ApPanelReader {
    shared: Arc<ApShared>,
}

/// Attempts per [`ApPanelReader::load`] before a torn read is skipped.
pub const AP_READ_ATTEMPTS: u32 = 4;

impl ApPanelReader {
    /// One attempt: `None` if the writer was mid-store.
    pub fn try_load(&self) -> Option<ApPanelValue> {
        let s = &self.shared;
        let before = s.seq.load(Ordering::Acquire);
        if before & 1 != 0 {
            return None;
        }
        let words = std::array::from_fn(|i| s.words[i].load(Ordering::Relaxed));
        fence(Ordering::Acquire);
        if s.seq.load(Ordering::Relaxed) != before {
            return None;
        }
        ap_decode(words)
    }

    /// Up to [`AP_READ_ATTEMPTS`] attempts; never blocks or sleeps.
    pub fn load(&self) -> Option<ApPanelValue> {
        (0..AP_READ_ATTEMPTS).find_map(|_| self.try_load())
    }
}

#[cfg(test)]
mod ap_panel_tests {
    use super::*;
    use gx_sim::{ApAction, ApInput, ControlStatus};

    fn panel(serial: u32, heading: f32) -> ApInput {
        ApInput { serial, action: ApAction::Heading, heading_deg: heading, ..ApInput::DEFAULT }
    }

    #[test]
    fn a_store_is_read_back_whole_and_normalized() {
        let (mut w, r) = ap_panel(ApInput::DEFAULT);
        assert_eq!(r.load().unwrap().input, ApInput::DEFAULT);
        w.store(ApInput { altitude_ft: 1e9, ..panel(3, -90.0) }, 1 << 40);
        let v = r.load().unwrap();
        assert_eq!((v.input.serial, v.input.action, v.input.heading_deg, v.input.altitude_ft), (3, ApAction::Heading, 270.0, 41_000.0));
        assert_eq!(v.changed_at_ns, 1 << 40);
        assert_eq!(w.value(), v);
    }

    #[test]
    fn a_read_while_the_writer_is_mid_store_is_skipped_not_used() {
        let (mut w, r) = ap_panel(ApInput::DEFAULT);
        w.store(panel(1, 10.0), 5);
        // Simulate the writer stopped between its two seq stores.
        let seq = r.shared.seq.load(Ordering::Relaxed);
        r.shared.seq.store(seq + 1, Ordering::Relaxed);
        r.shared.words[2].store(99.0f32.to_bits(), Ordering::Relaxed);
        assert_eq!(r.try_load(), None);
        assert_eq!(r.load(), None, "bounded retries, then skipped");
        // A seq that moved during the read is also refused: the writer finished.
        r.shared.seq.store(seq + 2, Ordering::Relaxed);
        assert_eq!(r.load().unwrap().input.heading_deg, 99.0);
    }

    /// Two threads: every field of every value the writer stores derives
    /// from one counter, so a mixture of two stores is detectable. The
    /// reader must never accept one, and must accept many reads.
    #[test]
    fn two_threads_never_see_a_torn_value() {
        let (mut w, r) = ap_panel(ApInput::DEFAULT);
        let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let reader = {
            let stop = Arc::clone(&stop);
            std::thread::spawn(move || {
                let (mut ok, mut skipped, mut torn) = (0u64, 0u64, 0u64);
                while !stop.load(Ordering::Relaxed) {
                    match r.try_load() {
                        None => skipped += 1,
                        Some(v) => {
                            let n = v.changed_at_ns;
                            let i = v.input;
                            let consistent = u64::from(i.serial) == n
                                && i.heading_deg == (n % 360) as f32
                                && i.altitude_ft == (n % 41_000) as f32
                                && i.speed_kt == 100.0 + (n % 250) as f32
                                && i.action == ApAction::from_code((n % 7) as u8).unwrap();
                            if consistent { ok += 1 } else { torn += 1 }
                        }
                    }
                }
                (ok, skipped, torn)
            })
        };
        for n in 1..=2_000_000u64 {
            let input = ApInput {
                serial: n as u32,
                action: ApAction::from_code((n % 7) as u8).unwrap(),
                heading_deg: (n % 360) as f32,
                altitude_ft: (n % 41_000) as f32,
                speed_kt: 100.0 + (n % 250) as f32,
            };
            w.store(input, n);
        }
        stop.store(true, Ordering::Relaxed);
        let (ok, skipped, torn) = reader.join().unwrap();
        assert_eq!(torn, 0, "accepted torn values (ok {ok}, skipped {skipped})");
        assert!(ok > 1000, "too few successful reads: {ok} (skipped {skipped})");
    }

    fn valid_frame(sequence: u64) -> ControlFrame {
        ControlFrame { sequence, timestamp_ns: 1_000 + sequence, roll: 0.25, ..ControlFrame::default() }
    }

    #[test]
    fn every_valid_publication_carries_the_panel_and_a_change_republishes_at_once() {
        let (writer, mut reader) = control_mailbox();
        let (mut panel_w, panel_r) = ap_panel(ApInput::DEFAULT);
        let mut p = InputPublisher::new(writer).with_panel(panel_r);
        panel_w.store(panel(1, 90.0), 5);
        p.publish_frame(&valid_frame(7), 10);
        let t = reader.read(11);
        assert_eq!((t.status, t.controls.ap), (ControlStatus::Valid, panel(1, 90.0)));
        // Unchanged: nothing to republish.
        assert!(!p.republish_panel(false, 12));
        // A click: published at once, on the same frame (axes, sequence), no edges.
        panel_w.store(panel(2, 95.0), 13);
        assert!(p.republish_panel(false, 14));
        let t = reader.read(15);
        assert_eq!(t.status, ControlStatus::Valid);
        assert_eq!(t.controls.ap, panel(2, 95.0));
        assert_eq!((t.controls.roll, t.record.input_sequence, t.record.published_at_ns), (0.25, 7, 14));
        assert!(t.commands.is_empty());
        assert_eq!(p.panel_value().changed_at_ns, 13);
        // Heartbeats keep carrying it.
        p.heartbeat(false, 20);
        assert_eq!(reader.read(21).controls.ap, panel(2, 95.0));
    }

    #[test]
    fn a_panel_change_is_never_republished_valid_while_input_is_lost_or_needs_resync() {
        let (writer, mut reader) = control_mailbox();
        let (mut panel_w, panel_r) = ap_panel(ApInput::DEFAULT);
        let mut p = InputPublisher::new(writer).with_panel(panel_r);
        // Never published: nothing valid to republish on.
        panel_w.store(panel(1, 10.0), 1);
        assert!(!p.republish_panel(false, 2));
        assert_eq!(reader.read(3).status, ControlStatus::NoInput);
        p.publish_frame(&valid_frame(1), 4);
        // Needs resync: refused, the record stays as it was.
        panel_w.store(panel(2, 20.0), 5);
        assert!(!p.republish_panel(true, 6));
        assert_eq!(reader.read(7).controls.ap, panel(1, 10.0));
        let reasons = [
            MachineOutput::LossDetected(gx_input::InputLoss { timestamp_ns: 8, source_event_sequence: 2 }),
            MachineOutput::ResyncRequired(gx_input::ResyncRequired {
                loss: gx_input::InputLoss { timestamp_ns: 8, source_event_sequence: 2 },
                trigger: gx_input::ResyncTrigger::ReportBoundary,
                discarded_events: 0,
            }),
        ];
        let mut serial = 2;
        for (k, output) in reasons.iter().enumerate() {
            p.publish_output(output, 8);
            serial += 1;
            panel_w.store(panel(serial, 30.0), 9);
            assert!(!p.republish_panel(false, 10));
            let t = reader.read(11);
            assert_ne!(t.status, ControlStatus::Valid, "reason {k}");
            assert_eq!(t.controls, ControlInput::default(), "neutral, panel not published");
            p.publish_frame(&valid_frame(10 + k as u64), 12);
        }
        p.publish_disconnected(13);
        serial += 1;
        panel_w.store(panel(serial, 40.0), 14);
        assert!(!p.republish_panel(false, 15));
        assert_ne!(reader.read(16).status, ControlStatus::Valid);
        // The first valid frame after recovery carries the latest panel.
        p.publish_frame(&valid_frame(20), 17);
        let t = reader.read(18);
        assert_eq!((t.status, t.controls.ap), (ControlStatus::Valid, panel(serial, 40.0)));
    }

    #[test]
    fn without_a_panel_publications_carry_the_default() {
        let (writer, mut reader) = control_mailbox();
        let mut p = InputPublisher::new(writer);
        p.publish_frame(&valid_frame(1), 2);
        assert_eq!(reader.read(3).controls.ap, ApInput::DEFAULT);
        assert!(!p.republish_panel(false, 4));
    }
}

/// The S2+ stick brake bindings (ORG-024, ISS-039), applied on the I/O thread:
///
/// * numbered button 4 (`BTN_TOP`, logical index 3) toggles the parking brake,
///   which forces both brakes to 1.0 while set;
/// * the hat brakes the wheels: **back** = both brakes, **left** = left brake
///   only, **right** = right brake only (differential braking/steering on the
///   ground); back-left / back-right brake that side only; hat forward does
///   nothing. Hat brakes are full (1.0) while held: the hat is a switch;
/// * a **Reset** press (button 3) re-sets the parking brake, so the aircraft
///   is again held at the threshold after Reset, as the prompts say.
///
/// Button 5 is the camera, 7/8 the pitch trim, 6 and 9-11 the levers
/// ([`button`], [`StickLevers`]); 12 remains unmapped. INPUT-002 still
/// publishes neutral, released controls on input loss (this binding is not
/// consulted then).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StickParkingBrake {
    set: bool,
}

/// Hat deflection beyond which a hat direction counts as pressed (the
/// Extreme 3D Pro hat reports -1, 0 or +1 per axis).
pub const HAT_PRESSED: f32 = 0.5;

/// (left, right) wheel brake from the hat alone; see [`StickParkingBrake`].
/// `hat_y > 0` is the hat pulled back (towards the pilot; evdev ABS_HAT0Y +1).
pub fn hat_brakes(frame: &ControlFrame) -> (f32, f32) {
    let left = frame.hat_x < -HAT_PRESSED;
    let right = frame.hat_x > HAT_PRESSED;
    let back = frame.hat_y > HAT_PRESSED;
    let l = left || (back && !right);
    let r = right || (back && !left);
    (if l { 1.0 } else { 0.0 }, if r { 1.0 } else { 0.0 })
}

impl StickParkingBrake {
    pub const BUTTON: u8 = 3;

    pub fn new(set: bool) -> Self {
        Self { set }
    }

    pub fn is_set(&self) -> bool {
        self.set
    }

    pub fn apply(&mut self, frame: &ControlFrame, controls: &mut ControlInput) {
        if frame.buttons_pressed & (1 << button::RESET) != 0 {
            self.set = true;
        } else if frame.buttons_pressed & (1 << Self::BUTTON) != 0 {
            self.set = !self.set;
        }
        let (l, r) = hat_brakes(frame);
        controls.left_brake = controls.left_brake.max(l);
        controls.right_brake = controls.right_brake.max(r);
        if self.set {
            controls.left_brake = 1.0;
            controls.right_brake = 1.0;
        }
    }
}

/// The INPUT-005 lever bindings, applied on the I/O thread like
/// [`StickParkingBrake`]. These are virtual levers moved by press edges
/// ([`button`]):
///
/// * button 6 cycles the speedbrake: stowed -> half -> full -> stowed;
/// * base buttons 9 / 10 move the flap lever up / down one detent of the
///   aircraft's list, stopping at the ends;
/// * base button 11 toggles the gear lever;
/// * a **Reset** press restores the initial levers (gear down, take-off
///   flap, speedbrake stowed), which is also what `SimCore` resets to.
///
/// The levers ride in every valid publication. On input loss `SimCore` keeps
/// the last valid levers (`gx_sim::levers`); with weight on wheels it holds
/// the gear down whatever the lever says.
#[derive(Clone, Debug, PartialEq)]
pub struct StickLevers {
    detents: Vec<f32>,
    initial_flap: usize,
    flap: usize,
    gear_down: bool,
    speedbrake_step: u8,
}

impl StickLevers {
    /// `flap_detents` are the `fcs/flap-cmd-norm` values from up to full;
    /// `initial_flap` indexes the take-off detent.
    pub fn new(flap_detents: &[f32], initial_flap: usize) -> Self {
        assert!(initial_flap < flap_detents.len(), "initial flap detent out of range");
        Self {
            detents: flap_detents.to_vec(),
            initial_flap,
            flap: initial_flap,
            gear_down: true,
            speedbrake_step: 0,
        }
    }

    /// The levers this binding publishes.
    pub fn levers(&self) -> gx_sim::Levers {
        gx_sim::Levers {
            gear_down: self.gear_down,
            flaps: self.detents[self.flap],
            speedbrake: f32::from(self.speedbrake_step) / 2.0,
        }
    }

    pub fn flap_index(&self) -> usize {
        self.flap
    }

    pub fn apply(&mut self, frame: &ControlFrame, controls: &mut ControlInput) {
        let pressed = |b: u8| frame.buttons_pressed & (1 << b) != 0;
        if pressed(button::RESET) {
            self.flap = self.initial_flap;
            self.gear_down = true;
            self.speedbrake_step = 0;
        } else {
            if pressed(button::SPEEDBRAKE) {
                self.speedbrake_step = (self.speedbrake_step + 1) % 3;
            }
            // Both flap buttons in one report cancel out.
            match (pressed(button::FLAPS_UP), pressed(button::FLAPS_DOWN)) {
                (true, false) => self.flap = self.flap.saturating_sub(1),
                (false, true) => self.flap = (self.flap + 1).min(self.detents.len() - 1),
                _ => {}
            }
            if pressed(button::GEAR) {
                self.gear_down = !self.gear_down;
            }
        }
        controls.levers = self.levers();
    }
}

/// Result of one tick-boundary mailbox read on the simulation thread.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlTick {
    /// What was read, including the controls now latched (neutral unless
    /// `input.status == Valid`).
    pub input: TickControls,
    /// The last reset snapshot produced by this read's commands, if any.

    pub reset: Option<AircraftSnapshot>,
    /// The INPUT-002 status snapshot published by this read (pause for
    /// input loss, or a warning change while not running), if any.
    pub status: Option<AircraftSnapshot>,
}

pub fn validate_world_pack(bytes: &[u8]) -> Result<WorldAsset, FormatError> {
    read_world(bytes, Limits::default())
}

/// The local ENU frame of one validated world pack, shared read-only by the
/// simulation side (to produce `LocalEnuPose`) and, later, the renderer.
///
/// Frame id, generation and the MSL-to-ellipsoid geoid separation all come
/// from the pack's validated georeference block (GXWORLD 2.0); nothing is
/// derived here.
#[derive(Clone, Debug)]
pub struct WorldFrame {
    world: Arc<WorldAsset>,
}

impl WorldFrame {
    /// `world` must come from [`validate_world_pack`]; its origin and
    /// georeference are then finite, in range and non-zero.
    pub fn new(world: Arc<WorldAsset>) -> Self {
        Self { world }
    }

    pub fn world(&self) -> &WorldAsset {
        &self.world
    }

    /// A shared handle to the pack (the S4 render thread draws it).
    pub fn world_arc(&self) -> Arc<WorldAsset> {
        Arc::clone(&self.world)
    }

    pub fn frame_id(&self) -> u64 {
        self.world.georeference.frame_id
    }

    pub fn frame_generation(&self) -> u64 {
        self.world.georeference.frame_generation
    }

    /// Signed metres to add to an MSL height (pack geoid model) to get the
    /// WGS84 ellipsoid height used by this frame.
    pub fn msl_to_ellipsoid_m(&self) -> f64 {
        self.world.georeference.msl_to_ellipsoid_m()
    }

    /// The gx-sim georeference for an MSL-reporting FDM (JSBSim), carrying
    /// this pack's frame identity and geoid separation.
    pub fn sim_frame(&self) -> Wgs84EnuFrame {
        Wgs84EnuFrame::new(
            self.frame_id(),
            self.frame_generation(),
            self.origin(),
            self.msl_to_ellipsoid_m(),
        )
        .expect("validated world pack has a valid frame")
    }

    /// The world origin as a gx-sim geodetic position (WGS84 ellipsoid).
    pub fn origin(&self) -> GeodeticPosition {
        let origin = self.world.origin;
        GeodeticPosition {
            datum: GeodeticDatum::Wgs84Ellipsoid,
            latitude_deg: origin.latitude_deg,
            longitude_deg: origin.longitude_deg,
            ellipsoid_height_m: origin.height_m,
        }
    }

    /// ENU metres of `position` relative to the pack origin, computed by
    /// gx-world's conversion.
    pub fn enu_m(&self, position: GeodeticPosition) -> [f64; 3] {
        debug_assert_eq!(position.datum, GeodeticDatum::Wgs84Ellipsoid);
        let enu = self.world.geodetic_to_enu(GeodeticPoint {
            latitude_deg: position.latitude_deg,
            longitude_deg: position.longitude_deg,
            height_m: position.ellipsoid_height_m,
        });
        [enu.east_m, enu.north_m, enu.up_m]
    }

    /// Full local pose in this frame. Attitude is roll/pitch/heading (rad,
    /// aerospace body-to-NED), returned as a body-to-ENU quaternion.
    pub fn pose(
        &self,
        position: GeodeticPosition,
        velocity_enu_mps: [f64; 3],
        roll_pitch_heading_rad: [f64; 3],
    ) -> LocalEnuPose {
        LocalEnuPose {
            frame_id: self.frame_id(),
            frame_generation: self.frame_generation(),
            position_m: self.enu_m(position),
            velocity_mps: velocity_enu_mps,
            orientation_xyzw: body_to_enu_quaternion(roll_pitch_heading_rad),
        }
    }
}

/// Body-to-ENU unit quaternion `[x, y, z, w]` from aerospace Euler angles.
fn body_to_enu_quaternion([roll, pitch, heading]: [f64; 3]) -> [f64; 4] {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (heading * 0.5).sin_cos();
    // Body-to-NED, Z-Y-X order.
    let ned = [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ];
    // NED-to-ENU is a half turn about (1, 1, 0)/sqrt(2): q = (h, h, 0, 0).
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let (ax, ay, az, aw) = (h, h, 0.0, 0.0);
    let [bx, by, bz, bw] = ned;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

#[derive(Debug, PartialEq)]
pub enum AppError<E> {
    Backend(E),
    /// The backend produced a state without a valid geodetic position and
    /// local ENU pose. `SimCore` has paused and published nothing.
    InvalidWorldPose,
}

fn map_core_error<E>(error: CoreError<E>) -> AppError<E> {
    match error {
        CoreError::Backend(error) => AppError::Backend(error),
        CoreError::InvalidWorldPose => AppError::InvalidWorldPose,
    }
}

/// Where the simulation thread publishes. Implemented by the gx-sim
/// exchange; tests substitute sinks that report busy on demand.
pub trait SnapshotSink {
    fn try_publish(&self, snapshot: AircraftSnapshot) -> Result<(), PublishBusy>;
}

impl SnapshotSink for SnapshotExchange {
    fn try_publish(&self, snapshot: AircraftSnapshot) -> Result<(), PublishBusy> {
        SnapshotExchange::try_publish(self, snapshot)
    }
}

impl<S: SnapshotSink + ?Sized> SnapshotSink for Arc<S> {
    fn try_publish(&self, snapshot: AircraftSnapshot) -> Result<(), PublishBusy> {
        (**self).try_publish(snapshot)
    }
}

/// Publication accounting. Every snapshot offered is exactly one of
/// published, superseded, or (at most one) pending:
/// `offered == published + superseded + pending as u64`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PublishStats {
    pub offered: u64,
    pub published: u64,
    /// `try_publish` attempts that returned `PublishBusy` (first tries and
    /// retries alike).
    pub busy_events: u64,
    /// Pending snapshots replaced by a newer one before they could publish.
    pub superseded: u64,
    /// Longest run of consecutive busy attempts.
    pub max_consecutive_busy: u64,
    pub pending: bool,
}

/// Busy policy: **keep newest, retry next tick, never wait.**
///
/// `PublishBusy` means the exchange's two non-visible slots are both pinned
/// by readers mid-copy; the snapshot was not stored and readers keep seeing
/// the previous publication. The simulation thread must not wait for them
/// (ARCH-004), so the snapshot is held in one pending slot and retried at the
/// next tick boundary. If the next tick produces a newer snapshot first, the
/// newer one replaces it (latest-value semantics; the renderer only needs the
/// newest state, and a reset's new generation is carried by every later
/// snapshot too). Every busy attempt and every replacement is counted.
pub struct SnapshotPublisher<S> {
    sink: S,
    pending: Option<AircraftSnapshot>,
    consecutive_busy: u64,
    stats: PublishStats,
}

impl<S: SnapshotSink> SnapshotPublisher<S> {
    pub fn new(sink: S) -> Self {
        Self { sink, pending: None, consecutive_busy: 0, stats: PublishStats::default() }
    }

    /// Offer a newly produced snapshot and try to publish it now.
    pub fn offer(&mut self, snapshot: AircraftSnapshot) -> bool {
        self.stats.offered += 1;
        if self.pending.replace(snapshot).is_some() {
            self.stats.superseded += 1;
        }
        self.flush()
    }

    /// Retry the pending snapshot, if any. Returns true when nothing remains
    /// pending. One nonblocking attempt.
    pub fn flush(&mut self) -> bool {
        let Some(snapshot) = self.pending else { return true };
        match self.sink.try_publish(snapshot) {
            Ok(()) => {
                self.pending = None;
                self.consecutive_busy = 0;
                self.stats.published += 1;
            }
            Err(PublishBusy(_)) => {
                self.consecutive_busy += 1;
                self.stats.busy_events += 1;
                self.stats.max_consecutive_busy =
                    self.stats.max_consecutive_busy.max(self.consecutive_busy);
            }
        }
        self.stats.pending = self.pending.is_some();
        self.pending.is_none()
    }

    pub fn stats(&self) -> PublishStats {
        self.stats
    }
}

/// Everything the simulation thread owns: the core (and through it the
/// backend), the latest controls it has applied, and the publisher.
pub struct SimLoop<B, S> {
    core: SimCore<B>,
    publisher: SnapshotPublisher<S>,
    controls: ControlInput,
    last_read: Option<TickControls>,
}

impl<B: SimulationBackend, S: SnapshotSink> SimLoop<B, S> {
    pub fn new(backend: B, sink: S) -> Self {
        Self::with_levers(backend, sink, gx_sim::Levers::default())
    }

    /// A loop whose core starts at, and resets to, `initial` levers.
    pub fn with_levers(backend: B, sink: S, initial: gx_sim::Levers) -> Self {
        Self {
            core: SimCore::with_levers(backend, initial),
            publisher: SnapshotPublisher::new(sink),
            controls: ControlInput::default(),
            last_read: None,
        }
    }

    /// The runway ends the autopilot's ILS can fly (I4,
    /// `s2::setup::approaches`), in the backend's world frame.
    pub fn with_approaches(mut self, approaches: Vec<gx_sim::Approach>) -> Self {
        self.core = self.core.with_approaches(approaches);
        self
    }

    /// Production path: read the control mailbox once at a tick boundary,
    /// apply its commands in the reader's order, and latch its controls
    /// (neutral when invalid, stale or absent) for the following steps.
    /// Call before [`SimLoop::tick`]. If a command fails, the remaining
    /// commands of this read are dropped (the core is paused) and the
    /// controls are still latched.
    pub fn poll_controls(
        &mut self,
        reader: &mut ControlReader,
        now_ns: u64,
    ) -> Result<ControlTick, AppError<B::Error>> {
        let input = reader.read(now_ns);
        self.controls = input.controls;
        self.last_read = Some(input);
        debug_assert!(
            input.status == ControlStatus::Valid || input.controls == ControlInput::default()
        );
        // INPUT-002: status first (10 s loss pause, warning, Start gate),
        // then this read's commands.
        let status = self.core.update_input(input.status.into(), now_ns);
        if let Some(status) = status {
            self.publisher.offer(status);
        }
        let mut reset = None;
        for command in input.commands.iter() {
            if let Some(snapshot) = self.core.command(command, now_ns).map_err(map_core_error)? {
                self.publisher.offer(snapshot);
                reset = Some(snapshot);
            }
        }
        Ok(ControlTick { input, reset, status })
    }

    /// Direct, single-thread injection of one complete input report (unit
    /// tests only; it bypasses the mailbox and applies at most one command,
    /// per [`command_from_frame`]).
    pub fn ingest_controls(
        &mut self,
        frame: &ControlFrame,
        now_ns: u64,
    ) -> Result<Option<AircraftSnapshot>, AppError<B::Error>> {
        self.controls = controls_from_frame(frame);
        if let Some(status) = self.core.update_input(InputStatus::Valid, now_ns) {
            self.publisher.offer(status);
        }
        let Some(command) = command_from_frame(frame) else {
            return Ok(None);
        };
        let snapshot = self.core.command(command, now_ns).map_err(map_core_error)?;
        if let Some(snapshot) = snapshot {
            self.publisher.offer(snapshot);
        }
        Ok(snapshot)
    }

    /// One 120 Hz tick: step if running, then publish or retry the pending
    /// snapshot. Never waits on readers.
    pub fn tick(&mut self, now_ns: u64) -> Result<Option<AircraftSnapshot>, AppError<B::Error>> {
        let snapshot = self.core.step(self.controls, now_ns).map_err(map_core_error);
        match snapshot {
            Ok(Some(snapshot)) => {
                self.publisher.offer(snapshot);
                Ok(Some(snapshot))
            }
            other => {
                self.publisher.flush();
                other
            }
        }
    }

    /// Apply one application-originated command (the startup reset, or the
    /// S1 console Start) at a tick boundary, outside the control mailbox.
    /// `SimCore`'s own INPUT-002 gate still applies: Start is refused unless
    /// the input status latched by the last [`SimLoop::poll_controls`] is
    /// `Valid`.
    pub fn command(
        &mut self,
        command: Command,
        now_ns: u64,
    ) -> Result<Option<AircraftSnapshot>, AppError<B::Error>> {
        let snapshot = self.core.command(command, now_ns).map_err(map_core_error)?;
        if let Some(snapshot) = snapshot {
            self.publisher.offer(snapshot);
        }
        Ok(snapshot)
    }

    /// Retry a pending snapshot without stepping (for shutdown or tests).
    pub fn flush(&mut self) -> bool {
        self.publisher.flush()
    }

    pub fn run_state(&self) -> RunState {
        self.core.state()
    }

    pub fn simulation_tick(&self) -> u64 {
        self.core.simulation_tick()
    }

    pub fn publish_stats(&self) -> PublishStats {
        self.publisher.stats()
    }

    pub fn pause_reason(&self) -> Option<PauseReason> {
        self.core.pause_reason()
    }

    pub fn input_health(&self) -> InputHealth {
        self.core.input_health()
    }

    /// Start commands the core refused because input was not valid.
    pub fn refused_starts(&self) -> u64 {
        self.core.refused_starts()
    }

    /// Controls latched for the next step.
    pub fn controls(&self) -> ControlInput {
        self.controls
    }

    /// Controls the last step gave the backend (the autopilot's roll, pitch
    /// and throttle while it flies).
    pub fn applied_controls(&self) -> ControlInput {
        self.core.applied_controls()
    }

    pub fn autopilot(&self) -> gx_sim::ApStatus {
        self.core.autopilot()
    }

    /// The most recent mailbox read by [`SimLoop::poll_controls`], kept even
    /// when applying its commands failed (the record log needs it).
    pub fn last_read(&self) -> Option<TickControls> {
        self.last_read
    }
}

/// Serial driver for the dry run: the I/O publisher, the sim loop with its
/// mailbox reader, and a reader handle on the same exchange, called from one
/// thread. Every report goes through the control mailbox, as in the threaded
/// app, so staleness applies: no publication for 100 ms of tick time gives
/// neutral controls.
pub struct OfflineApp<B> {
    input: InputPublisher,
    controls: ControlReader,
    sim: SimLoop<B, Arc<SnapshotExchange>>,
    exchange: Arc<SnapshotExchange>,
    last_input: Option<TickControls>,
}

impl<B: SimulationBackend> OfflineApp<B> {
    pub fn new(backend: B) -> Self {
        assert_eq!(SIM_HZ % RENDER_HZ, 0, "render rate must divide sim rate");
        let exchange = Arc::new(SnapshotExchange::new());
        let (writer, controls) = control_mailbox();
        Self {
            input: InputPublisher::new(writer),
            controls,
            sim: SimLoop::new(backend, Arc::clone(&exchange)),
            exchange,
            last_input: None,
        }
    }

    /// Publish one report into the mailbox (I/O side), then consume it at a
    /// tick boundary (sim side). Returns the reset snapshot if the report
    /// reset the core.
    pub fn ingest_controls(
        &mut self,
        frame: &ControlFrame,
        now_ns: u64,
    ) -> Result<Option<AircraftSnapshot>, AppError<B::Error>> {
        self.input.publish_frame(frame, now_ns);
        self.poll(now_ns).map(|tick| tick.reset)
    }

    /// I/O-side access, e.g. to publish loss or heartbeats.
    pub fn input(&mut self) -> &mut InputPublisher {
        &mut self.input
    }

    /// The most recent mailbox read.
    pub fn last_input(&self) -> Option<TickControls> {
        self.last_input
    }

    fn poll(&mut self, now_ns: u64) -> Result<ControlTick, AppError<B::Error>> {
        let tick = self.sim.poll_controls(&mut self.controls, now_ns)?;
        self.last_input = Some(tick.input);
        Ok(tick)
    }

    /// Advance the four fixed simulation steps associated with one 30 fps
    /// presentation interval, then read the latest pair as a renderer would.
    pub fn advance_render_sample(
        &mut self,
        tick_times_ns: [u64; SIM_TICKS_PER_RENDER as usize],
    ) -> Result<Option<SnapshotPair>, AppError<B::Error>> {
        for now_ns in tick_times_ns {
            self.poll(now_ns)?;
            self.sim.tick(now_ns)?;
        }
        Ok(self.exchange.try_latest_pair())
    }

    pub fn exchange(&self) -> Arc<SnapshotExchange> {
        Arc::clone(&self.exchange)
    }

    pub fn run_state(&self) -> RunState {
        self.sim.run_state()
    }

    pub fn simulation_tick(&self) -> u64 {
        self.sim.simulation_tick()
    }

    pub fn publish_stats(&self) -> PublishStats {
        self.sim.publish_stats()
    }
}

/// Height of the fake aircraft's reference point above the (flat) terrain.
pub const FAKE_GEAR_HEIGHT_M: f64 = 2.5;
const METRES_PER_DEGREE: f64 = 111_320.0;

/// Deterministic backend used only by the host dry run and seam tests. It
/// starts at the world origin on flat terrain at the origin's ellipsoid
/// height, rolls along its heading at `100 * throttle` m/s, and derives the
/// local pose from the world pack through [`WorldFrame`].
#[derive(Clone, Debug)]
pub struct FakeBackend {
    frame: WorldFrame,
    state: AircraftState,
}

impl FakeBackend {
    /// Starts in the reset state, so stepping before the first reset is
    /// still at the origin.
    pub fn new(frame: WorldFrame) -> Self {
        let mut backend = Self { frame, state: AircraftState::default() };
        backend.place_at_origin();
        backend
    }

    fn place_at_origin(&mut self) {
        let mut geodetic = self.frame.origin();
        geodetic.ellipsoid_height_m += FAKE_GEAR_HEIGHT_M;
        self.state = AircraftState {
            geodetic,
            height_agl_m: FAKE_GEAR_HEIGHT_M,
            gear_position: 1.0,
            weight_on_wheels: true,
            ..AircraftState::default()
        };
        self.update_pose();
    }

    pub fn frame(&self) -> &WorldFrame {
        &self.frame
    }

    fn velocity_enu(&self) -> [f64; 3] {
        let speed = self.state.calibrated_airspeed_mps;
        let (sin_h, cos_h) = self.state.heading_rad.sin_cos();
        [speed * sin_h, speed * cos_h, 0.0]
    }

    fn update_pose(&mut self) {
        let velocity = self.velocity_enu();
        self.state.local_enu = Some(self.frame.pose(
            self.state.geodetic,
            velocity,
            [self.state.roll_rad, self.state.pitch_rad, self.state.heading_rad],
        ));
    }
}

impl SimulationBackend for FakeBackend {
    type Error = std::convert::Infallible;

    fn reset(&mut self) -> Result<AircraftState, Self::Error> {
        self.place_at_origin();
        Ok(self.state)
    }

    fn step(
        &mut self,
        controls: ControlInput,
        dt_seconds: f64,
    ) -> Result<AircraftState, Self::Error> {
        debug_assert_eq!(dt_seconds, STEP_SECONDS);
        self.state.roll_rad += f64::from(controls.roll) * dt_seconds;
        self.state.pitch_rad += f64::from(controls.pitch) * dt_seconds;
        self.state.heading_rad += f64::from(controls.yaw) * dt_seconds;
        self.state.calibrated_airspeed_mps = f64::from(controls.throttle) * 100.0;
        let [east, north, _] = self.velocity_enu();
        let latitude = self.state.geodetic.latitude_deg;
        self.state.geodetic.latitude_deg += north * dt_seconds / METRES_PER_DEGREE;
        self.state.geodetic.longitude_deg +=
            east * dt_seconds / (METRES_PER_DEGREE * latitude.to_radians().cos());
        self.state.aileron_position = f64::from(controls.roll);
        self.state.elevator_position = f64::from(controls.pitch);
        self.state.rudder_position = f64::from(controls.yaw);
        self.state.engine_n1 = f64::from(controls.throttle);
        self.update_pose();
        Ok(self.state)
    }
}

#[cfg(test)]
mod trim_binding_tests {
    use super::*;

    fn held(mask: u64) -> ControlFrame {
        ControlFrame { buttons: mask, ..Default::default() }
    }

    #[test]
    fn base_buttons_seven_and_eight_are_the_trim_switch() {
        assert_eq!(controls_from_frame(&held(1 << 7)).trim, 1.0, "button 8 = nose up");
        assert_eq!(controls_from_frame(&held(1 << 6)).trim, -1.0, "button 7 = nose down");
        assert_eq!(controls_from_frame(&held((1 << 6) | (1 << 7))).trim, 0.0, "both = none");
        assert_eq!(controls_from_frame(&held(0)).trim, 0.0);
        // Held, not edge-triggered: a press edge alone does not trim.
        let edge = ControlFrame { buttons_pressed: 1 << 7, ..Default::default() };
        assert_eq!(controls_from_frame(&edge).trim, 0.0);
    }

    #[test]
    fn trim_buttons_never_command_start_pause_reset_or_the_parking_brake() {
        let f = ControlFrame { buttons: 0b1100_0000, buttons_pressed: 0b1100_0000, ..Default::default() };
        assert_eq!(command_from_frame(&f), None);
        assert_eq!(edges_from_frame(&f), CommandEdges::default());
        let mut p = StickParkingBrake::new(false);
        let mut c = ControlInput::default();
        p.apply(&f, &mut c);
        assert!(!p.is_set());
        for b in [button::START, button::PAUSE, button::RESET, StickParkingBrake::BUTTON, button::CAMERA] {
            assert!(b != button::TRIM_NOSE_UP && b != button::TRIM_NOSE_DOWN);
        }
    }
}

#[cfg(test)]
mod lever_binding_tests {
    use super::*;
    use gx_sim::Levers;

    const DETENTS: [f32; 4] = [0.0, 0.25, 0.5, 1.0];

    fn press(l: &mut StickLevers, mask: u64) -> Levers {
        let mut c = ControlInput::default();
        l.apply(&ControlFrame { buttons_pressed: mask, buttons: mask, ..Default::default() }, &mut c);
        c.levers
    }

    #[test]
    fn starts_at_the_take_off_set_and_publishes_it_without_presses() {
        let mut l = StickLevers::new(&DETENTS, 1);
        assert_eq!(press(&mut l, 0), Levers { gear_down: true, flaps: 0.25, speedbrake: 0.0 });
    }

    #[test]
    fn flap_buttons_step_one_detent_and_stop_at_the_ends() {
        let mut l = StickLevers::new(&DETENTS, 1);
        let down = 1 << button::FLAPS_DOWN;
        let up = 1 << button::FLAPS_UP;
        assert_eq!(press(&mut l, down).flaps, 0.5);
        assert_eq!(press(&mut l, down).flaps, 1.0);
        assert_eq!(press(&mut l, down).flaps, 1.0, "full is the end");
        for _ in 0..5 {
            press(&mut l, up);
        }
        assert_eq!(l.flap_index(), 0, "up is the end");
        assert_eq!(press(&mut l, up | down).flaps, 0.0, "both in one report cancel");
        // Held, not pressed: nothing moves.
        let mut c = ControlInput::default();
        l.apply(&ControlFrame { buttons: down, ..Default::default() }, &mut c);
        assert_eq!(c.levers.flaps, 0.0);
    }

    #[test]
    fn gear_toggles_and_speedbrake_cycles_stowed_half_full() {
        let mut l = StickLevers::new(&DETENTS, 0);
        assert!(!press(&mut l, 1 << button::GEAR).gear_down);
        assert!(press(&mut l, 1 << button::GEAR).gear_down);
        let sb = 1 << button::SPEEDBRAKE;
        assert_eq!([press(&mut l, sb).speedbrake, press(&mut l, sb).speedbrake, press(&mut l, sb).speedbrake], [0.5, 1.0, 0.0]);
    }

    #[test]
    fn reset_restores_the_initial_levers_and_wins_over_lever_presses() {
        let mut l = StickLevers::new(&DETENTS, 1);
        press(&mut l, (1 << button::GEAR) | (1 << button::SPEEDBRAKE));
        press(&mut l, 1 << button::FLAPS_DOWN);
        let all = (1 << button::RESET) | (1 << button::GEAR) | (1 << button::FLAPS_DOWN);
        assert_eq!(press(&mut l, all), Levers { gear_down: true, flaps: 0.25, speedbrake: 0.0 });
    }

    #[test]
    fn lever_buttons_are_distinct_and_never_commands() {
        let levers = [button::SPEEDBRAKE, button::FLAPS_UP, button::FLAPS_DOWN, button::GEAR];
        let taken = [button::START, button::PAUSE, button::RESET, StickParkingBrake::BUTTON, button::CAMERA,
                     button::TRIM_NOSE_DOWN, button::TRIM_NOSE_UP];
        for (i, b) in levers.iter().enumerate() {
            assert!(!taken.contains(b) && !levers[i + 1..].contains(b), "button index {b}");
            let f = ControlFrame { buttons: 1 << b, buttons_pressed: 1 << b, ..Default::default() };
            assert_eq!(command_from_frame(&f), None);
            assert_eq!(edges_from_frame(&f), CommandEdges::default());
            assert_eq!(controls_from_frame(&f).trim, 0.0);
        }
    }

    #[test]
    fn publisher_carries_the_levers_and_loss_records_are_neutral() {
        let (writer, mut reader) = control_mailbox();
        let mut p = InputPublisher::new(writer).with_levers(StickLevers::new(&DETENTS, 2));
        let f = ControlFrame { sequence: 1, buttons_pressed: 1 << button::GEAR, ..Default::default() };
        p.publish_frame(&f, 10);
        let t = reader.read(11);
        assert_eq!(t.controls.levers, Levers { gear_down: false, flaps: 0.5, speedbrake: 0.0 });
        p.publish_disconnected(12);
        assert_eq!(reader.read(13).controls, ControlInput::default(), "SimCore holds the levers, not the mailbox");
    }
}

#[cfg(test)]
mod parking_brake_tests {
    use super::*;

    fn apply(p: &mut StickParkingBrake, f: ControlFrame) -> (f32, f32) {
        let mut c = ControlInput::default();
        p.apply(&f, &mut c);
        (c.left_brake, c.right_brake)
    }

    #[test]
    fn only_numbered_button_four_toggles_parking_brake() {
        let mut parking = StickParkingBrake::new(true);
        assert_eq!(apply(&mut parking, ControlFrame { hat_x: 1.0, hat_y: -1.0, ..Default::default() }), (1.0, 1.0));
        assert_eq!(apply(&mut parking, ControlFrame { buttons_pressed: 1 << StickParkingBrake::BUTTON, ..Default::default() }), (0.0, 0.0));
        assert!(!parking.is_set());
        apply(&mut parking, ControlFrame { buttons_pressed: 1 << 4, ..Default::default() });
        assert!(!parking.is_set(), "numbered button 5 stays unmapped");
    }

    #[test]
    fn iss039_hat_back_brakes_both_left_right_brake_one_side() {
        let mut p = StickParkingBrake::new(false);
        let hat = |x: f32, y: f32| ControlFrame { hat_x: x, hat_y: y, ..Default::default() };
        assert_eq!(apply(&mut p, hat(0.0, 1.0)), (1.0, 1.0), "back = both");
        assert_eq!(apply(&mut p, hat(-1.0, 0.0)), (1.0, 0.0), "left = left");
        assert_eq!(apply(&mut p, hat(1.0, 0.0)), (0.0, 1.0), "right = right");
        assert_eq!(apply(&mut p, hat(-1.0, 1.0)), (1.0, 0.0), "back-left = left");
        assert_eq!(apply(&mut p, hat(1.0, 1.0)), (0.0, 1.0), "back-right = right");
        assert_eq!(apply(&mut p, hat(0.0, -1.0)), (0.0, 0.0), "forward = none");
        assert_eq!(apply(&mut p, hat(0.0, 0.0)), (0.0, 0.0), "centred = released");
        assert!(!p.is_set(), "the hat never latches the parking brake");
        // The parking brake still wins while set.
        let mut set = StickParkingBrake::new(true);
        assert_eq!(apply(&mut set, hat(-1.0, 0.0)), (1.0, 1.0));
    }

    #[test]
    fn iss039_reset_re_sets_the_parking_brake() {
        let mut p = StickParkingBrake::new(false);
        let reset = ControlFrame { buttons_pressed: 1 << button::RESET, ..Default::default() };
        assert_eq!(apply(&mut p, reset.clone()), (1.0, 1.0));
        assert!(p.is_set());
        // Reset while set keeps it set (no toggle); Reset + button 4 together: Reset wins.
        apply(&mut p, reset);
        assert!(p.is_set());
        let both = ControlFrame { buttons_pressed: (1 << button::RESET) | (1 << StickParkingBrake::BUTTON), ..Default::default() };
        apply(&mut p, both);
        assert!(p.is_set());
        // Pause and Start do not touch it.
        let mut q = StickParkingBrake::new(false);
        apply(&mut q, ControlFrame { buttons_pressed: (1 << button::PAUSE) | (1 << button::START), ..Default::default() });
        assert!(!q.is_set());
    }
}

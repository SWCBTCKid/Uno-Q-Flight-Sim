//! I/O-thread side: gx-input runtime events into the control mailbox.

use super::rec::esc;
use crate::InputPublisher;
use gx_input::{ControlFrame, InputLoss, MachineOutput, ResyncRequired};

/// The I/O thread republishes after this long without a publication. The
/// control mailbox requires at least every 10 ms (`HEARTBEAT_PERIOD_NS`);
/// 5 ms leaves margin for one late I/O loop iteration.
pub const HEARTBEAT_EVERY_NS: u64 = 5_000_000;
pub const HEARTBEAT_LIMIT_NS: u64 = 10_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PublicationGap {
    pub previous_ns: u64,
    pub publication_ns: u64,
    pub gap_ns: u64,
    pub kind: &'static str,
}

/// I/O-thread log lines (preformatted JSON), stamped relative to `epoch`.
pub struct IoLog {
    pub epoch: u64,
    pub lines: Vec<String>,
}

impl IoLog {
    pub fn new(epoch: u64) -> Self {
        Self { epoch, lines: Vec::new() }
    }

    /// `body` is the JSON members after `"k":"io","t":..,` (no braces).
    pub fn io(&mut self, now_ns: u64, body: &str) {
        self.lines.push(format!(
            "{{\"k\":\"io\",\"t\":{},{body}}}",
            now_ns.saturating_sub(self.epoch)
        ));
    }
}

/// Fail-closed mapping from gx-input results to mailbox publications, plus
/// heartbeat and publication-gap accounting. Host-testable; the Linux
/// [`EvdevSource`] translates `RuntimeEvent`s into these calls.
pub struct IoBridge {
    publisher: InputPublisher,
    last_publication_ns: Option<u64>,
    pub publications: u64,
    /// Longest interval between two consecutive publications.
    pub max_gap_ns: u64,
    /// Same, since the I/O loop last took it (per-second log).
    pub window_max_gap_ns: u64,
    pub frames: u64,
    pub last_buttons_pressed: u64,
    pub button_presses: u64,
    /// S4 camera-button (numbered button 5) press edges.
    pub camera_presses: u64,
    /// AP-001 panel changes republished by [`IoBridge::panel_if_changed`].
    pub panel_publications: u64,
    pub connected: bool,
    gaps: Vec<PublicationGap>,
}

impl IoBridge {
    pub fn new(publisher: InputPublisher) -> Self {
        Self {
            publisher,
            last_publication_ns: None,
            publications: 0,
            max_gap_ns: 0,
            window_max_gap_ns: 0,
            frames: 0,
            last_buttons_pressed: 0,
            button_presses: 0,
            camera_presses: 0,
            panel_publications: 0,
            connected: false,
            gaps: Vec::new(),
        }
    }

    fn published(&mut self, now_ns: u64, kind: &'static str) {
        if let Some(last) = self.last_publication_ns {
            let gap = now_ns.saturating_sub(last);
            self.max_gap_ns = self.max_gap_ns.max(gap);
            self.window_max_gap_ns = self.window_max_gap_ns.max(gap);
            if gap > HEARTBEAT_LIMIT_NS {
                self.gaps.push(PublicationGap {
                    previous_ns: last,
                    publication_ns: now_ns,
                    gap_ns: gap,
                    kind,
                });
            }
        }
        self.last_publication_ns = Some(now_ns);
        self.publications += 1;
    }

    /// A complete report (including the re-arm frame after a resync).
    pub fn frame(&mut self, frame: &ControlFrame, now_ns: u64) {
        self.publisher.publish_frame(frame, now_ns);
        self.frames += 1;
        if frame.buttons_pressed != 0 {
            self.last_buttons_pressed = frame.buttons_pressed;
            self.button_presses += 1;
            if frame.buttons_pressed & (1 << crate::button::CAMERA) != 0 {
                self.camera_presses += 1;
            }
        }
        self.published(now_ns, "frame");
    }

    /// `SYN_DROPPED`: neutral and `Unavailable(InputLoss)` at once.
    pub fn loss(&mut self, loss: InputLoss, now_ns: u64) {
        self.publisher.publish_output(&MachineOutput::LossDetected(loss), now_ns);
        self.published(now_ns, "loss");
    }

    /// Discard boundary reached: `Unavailable(ResyncRequired)` at once.
    pub fn resync_required(&mut self, required: ResyncRequired, now_ns: u64) {
        self.publisher.publish_output(&MachineOutput::ResyncRequired(required), now_ns);
        self.published(now_ns, "resync_required");
    }

    pub fn connected(&mut self) {
        self.connected = true;
    }

    /// The fd closed (unplug, read error, failed resync): neutral and
    /// `Unavailable(Disconnected)` until a new valid frame.
    pub fn disconnected(&mut self, now_ns: u64) {
        self.connected = false;
        self.publisher.publish_disconnected(now_ns);
        self.published(now_ns, "disconnected");
    }

    /// AP-001: publish a panel change now (see
    /// [`InputPublisher::republish_panel`]: only on top of a valid record,
    /// never with `needs_resync`). Returns the value published.
    pub fn panel_if_changed(&mut self, now_ns: u64, needs_resync: bool) -> Option<crate::ApPanelValue> {
        if !self.publisher.republish_panel(needs_resync, now_ns) {
            return None;
        }
        self.panel_publications += 1;
        self.published(now_ns, "panel");
        Some(self.publisher.panel_value())
    }

    /// Republish if nothing was published for [`HEARTBEAT_EVERY_NS`]. With
    /// `needs_resync` a valid record is replaced by `ResyncRequired` instead.
    pub fn heartbeat_if_due(&mut self, now_ns: u64, needs_resync: bool) -> bool {
        let due = self
            .last_publication_ns
            .map_or(true, |last| now_ns.saturating_sub(last) >= HEARTBEAT_EVERY_NS);
        if due {
            self.publisher.heartbeat(needs_resync, now_ns);
            self.published(now_ns, "heartbeat");
        }
        due
    }

    pub fn take_gaps(&mut self) -> Vec<PublicationGap> {
        std::mem::take(&mut self.gaps)
    }

    /// S2's numbered-button-4 parking-brake latch, when configured.
    pub fn parking_brake(&self) -> Option<bool> {
        self.publisher.parking_brake().map(|b| b.is_set())
    }
}

/// The I/O thread's device. One implementation per platform plus scripted
/// sources for tests.
pub trait InputSource {
    /// Drain currently available device records and publish through `bridge`.
    /// Connected reads are nonblocking; a reconnect may perform bounded
    /// exact-path/sysfs identity checks.
    fn poll(
        &mut self,
        now_ns: u64,
        fresh_now: &mut dyn FnMut() -> u64,
        bridge: &mut IoBridge,
        log: &mut IoLog,
    );
    /// gx-input is waiting for a resync snapshot (heartbeat must not keep a
    /// valid record alive).
    fn needs_resync(&self) -> bool;
}

/// JSON body for a button-press frame (logical bits plus evdev codes).
pub fn button_body(frame: &ControlFrame) -> String {
    format!(
        "\"ev\":\"buttons\",\"seq\":{},\"pressed\":{},\"held\":{}",
        frame.sequence, frame.buttons_pressed, frame.buttons
    )
}

pub fn text_body(ev: &str, key: &str, value: &str) -> String {
    format!("\"ev\":\"{ev}\",\"{key}\":{}", esc(value))
}

#[cfg(target_os = "linux")]
pub use linux::EvdevSource;

#[cfg(target_os = "linux")]
mod linux {
    use super::{button_body, text_body, InputSource, IoBridge, IoLog};
    use crate::s1::rec::esc;
    use gx_input::linux::{EvdevInput, RuntimeEvent};
    use gx_input::{ConnectionState, InputConfig, ResyncTrigger};
    use std::path::PathBuf;

    /// `EvdevInput` on the Extreme 3D Pro: the stable udev `by_id` path is
    /// re-resolved at every reconnect (once a second) because the `eventN`
    /// node can change across an unplug; the node's USB identity is checked
    /// in sysfs and the opened fd is tied to it. Selection, `open` and
    /// `close` run on gx-input's background connector, never in `poll`:
    /// `usbhid` sleeps ~50 ms in the first open and the last close waits for
    /// an RCU grace period. Reads are nonblocking; live resync is inline.
    pub struct EvdevSource {
        input: EvdevInput,
        by_id: PathBuf,
        pub events: u64,
    }

    impl EvdevSource {
        /// Spawns the connector helper; it inherits the calling thread's
        /// affinity and priority.
        pub fn new(by_id: PathBuf) -> std::io::Result<Self> {
            let mut input = EvdevInput::with_exact_path(InputConfig::extreme_3d_pro(), by_id.clone());
            input.start_background_connector()?;
            Ok(Self { input, by_id, events: 0 })
        }
    }

    impl InputSource for EvdevSource {
        fn poll(
            &mut self,
            now_ns: u64,
            fresh_now: &mut dyn FnMut() -> u64,
            bridge: &mut IoBridge,
            log: &mut IoLog,
        ) {
            let by_id = &self.by_id;
            let events = &mut self.events;
            self.input.poll_nonblocking(now_ns, |event| match event {
                RuntimeEvent::Input(_) => *events += 1,
                RuntimeEvent::Frame(frame) => {
                    let publication_ns = fresh_now();
                    bridge.frame(&frame, publication_ns);
                    if frame.buttons_pressed != 0 {
                        log.io(publication_ns, &button_body(&frame));
                    }
                }
                RuntimeEvent::LossDetected(loss) => {
                    let publication_ns = fresh_now();
                    bridge.loss(loss, publication_ns);
                    log.io(
                        publication_ns,
                        &format!("\"ev\":\"loss_detected\",\"source_event_sequence\":{}", loss.source_event_sequence),
                    );
                }
                RuntimeEvent::ResyncRequired(required) => {
                    let publication_ns = fresh_now();
                    bridge.resync_required(required, publication_ns);
                    let trigger = match required.trigger {
                        ResyncTrigger::ReportBoundary => "report_boundary",
                        ResyncTrigger::EventLimit => "event_limit",
                        ResyncTrigger::Timeout => "timeout",
                    };
                    log.io(
                        publication_ns,
                        &format!(
                            "\"ev\":\"resync_required\",\"trigger\":\"{trigger}\",\"discarded\":{}",
                            required.discarded_events
                        ),
                    );
                }
                RuntimeEvent::DeviceStateRead(_) => log.io(fresh_now(), "\"ev\":\"device_state_read\""),
                RuntimeEvent::ResyncCompleted { frame_sequence, .. } => log.io(
                    fresh_now(),
                    &format!("\"ev\":\"resync_completed\",\"frame_sequence\":{frame_sequence}"),
                ),
                RuntimeEvent::Background { operation, duration_ns } => log.io(
                    fresh_now(),
                    &format!("\"ev\":\"background\",\"op\":\"{operation}\",\"duration_ns\":{duration_ns}"),
                ),
                RuntimeEvent::ResyncAttemptFailed { attempt, reason, retry_at_ns } => log.io(
                    fresh_now(),
                    &format!(
                        "\"ev\":\"resync_failed\",\"attempt\":{attempt},\"reason\":{},\"gave_up\":{}",
                        esc(&reason),
                        retry_at_ns.is_none()
                    ),
                ),
                RuntimeEvent::State(state) => match state {
                    ConnectionState::Connected(device) => {
                        let publication_ns = fresh_now();
                        bridge.connected();
                        let resolved = std::fs::canonicalize(by_id)
                            .map(|p| p.to_string_lossy().into_owned())
                            .ok();
                        let matches = resolved.as_deref() == Some(device.event_path.as_str());
                        log.io(
                            publication_ns,
                            &format!(
                                "\"ev\":\"connected\",\"path\":{},\"name\":{},\"by_id\":{},\"by_id_resolved\":{},\"by_id_match\":{matches}",
                                esc(&device.event_path),
                                esc(&device.name),
                                esc(&by_id.to_string_lossy()),
                                resolved.as_deref().map_or("null".to_owned(), esc),
                            ),
                        );
                    }
                    ConnectionState::ResyncRequired { device, .. } => {
                        bridge.connected();
                        log.io(fresh_now(), &text_body("resync_pending", "path", &device.event_path));
                    }
                    ConnectionState::Disconnected => {
                        let publication_ns = fresh_now();
                        bridge.disconnected(publication_ns);
                        log.io(publication_ns, "\"ev\":\"disconnected\"");
                    }
                    ConnectionState::Error { message, .. } => {
                        let publication_ns = fresh_now();
                        bridge.disconnected(publication_ns);
                        log.io(publication_ns, &text_body("error", "message", &message));
                    }
                },
            });
        }

        fn needs_resync(&self) -> bool {
            matches!(self.input.state(), ConnectionState::ResyncRequired { .. })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gx_sim::{control_mailbox, Command, ControlStatus, UnavailableReason};

    fn frame(sequence: u64, roll: f32, pressed: u64) -> ControlFrame {
        ControlFrame {
            timestamp_ns: 1_000 + sequence,
            sequence,
            roll,
            buttons_pressed: pressed,
            ..ControlFrame::default()
        }
    }

    #[test]
    fn runtime_mapping_fails_closed_and_recovers_after_replug() {
        let (writer, mut reader) = control_mailbox();
        let mut io = IoBridge::new(InputPublisher::new(writer));
        let mut t = 1_000_000;
        io.connected();
        io.frame(&frame(0, 0.5, 0), t);
        let r = reader.read(t);
        assert_eq!(r.status, ControlStatus::Valid);
        assert_eq!(r.controls.roll, 0.5);
        assert_eq!((r.record.input_sequence, r.record.input_timestamp_ns), (0, 1_000));

        // Heartbeat keeps a held stick valid and carries the same frame.
        t += HEARTBEAT_EVERY_NS;
        assert!(io.heartbeat_if_due(t, false));
        assert!(!io.heartbeat_if_due(t + 1, false), "not due again yet");
        let r = reader.read(t);
        assert_eq!((r.status, r.record.input_sequence), (ControlStatus::Valid, 0));

        // SYN_DROPPED -> InputLoss, neutral, at once.
        t += 1;
        io.loss(InputLoss { timestamp_ns: t, source_event_sequence: 9 }, t);
        let r = reader.read(t);
        assert_eq!(r.status, ControlStatus::Unavailable(UnavailableReason::InputLoss));
        assert_eq!(r.controls.roll, 0.0);

        // A resync completes with a frame: valid again.
        io.frame(&frame(1, 0.25, 0), t + 1);
        assert_eq!(reader.read(t + 1).status, ControlStatus::Valid);

        // Heartbeat while gx-input needs resync never keeps valid axes.
        t += HEARTBEAT_EVERY_NS + 1;
        io.heartbeat_if_due(t, true);
        assert_eq!(
            reader.read(t).status,
            ControlStatus::Unavailable(UnavailableReason::ResyncRequired)
        );

        // Unplug: Disconnected, and the heartbeat keeps it so.
        io.frame(&frame(2, 0.25, 0), t + 1);
        io.disconnected(t + 2);
        assert!(!io.connected);
        for _ in 0..3 {
            t += HEARTBEAT_EVERY_NS;
            io.heartbeat_if_due(t, false);
            let r = reader.read(t);
            assert_eq!(r.status, ControlStatus::Unavailable(UnavailableReason::Disconnected));
            assert!(r.commands.is_empty());
        }

        // Replug: the first frame (here with a Start edge) is valid and its
        // Start is delivered, because it is in the new valid stretch.
        io.connected();
        io.frame(&frame(3, 0.0, 1 << crate::button::START), t + 1);
        let r = reader.read(t + 1);
        assert_eq!(r.status, ControlStatus::Valid);
        assert_eq!(r.commands.iter().collect::<Vec<_>>(), vec![Command::Start]);
        assert!(!r.rejected_start);
        assert_eq!(io.button_presses, 1);
        assert!(io.max_gap_ns < 2 * HEARTBEAT_EVERY_NS);
        // S4 camera button (numbered 5): counted for the renderer, never a
        // command for the sim.
        assert_eq!(io.camera_presses, 0);
        io.frame(&frame(4, 0.0, 1 << crate::button::CAMERA), t + 2);
        assert_eq!(io.camera_presses, 1);
        assert!(reader.read(t + 2).commands.is_empty());
    }
}

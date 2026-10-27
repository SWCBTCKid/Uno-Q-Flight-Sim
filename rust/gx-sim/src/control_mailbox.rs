//! I/O-to-simulation **latest-value control mailbox** (ARCH-004, ISS-033).
//!
//! One I/O thread owns the [`ControlWriter`]; one 120 Hz simulation thread
//! owns the [`ControlReader`]. Both handles are created together by
//! [`control_mailbox`], are not `Clone`, and so enforce single-writer /
//! single-reader at the type level. Neither side ever blocks, spins, retries
//! or waits for the other: [`ControlWriter::publish_valid`] and
//! [`ControlReader::read`] are each one copy plus at most one atomic swap.
//!
//! # What crosses
//!
//! Every publication is one complete [`ControlRecord`]: normalized axes, the
//! input validity, the source frame's sequence and timestamp, a monotonic
//! publication number, and cumulative per-command edge counters. Continuous
//! axes are **latest-wins**: a record overwritten before the simulation reads
//! it is simply gone. Command edges are not lost that way because they are
//! carried as counters inside every record (see *Command edges*).
//!
//! # Memory-ordering argument (triple buffer)
//!
//! The mailbox is a three-slot SPSC triple buffer. `slots` are plain
//! `UnsafeCell<ControlRecord>`s; the only shared atomic is `middle: AtomicU8`,
//! holding a slot index (bits 0..=1) and a `DIRTY` bit (bit 2).
//!
//! 1. **Ownership invariant.** At every point in `middle`'s modification
//!    order, the three indices `{writer.back, middle & INDEX, reader.front}`
//!    are a permutation of `{0, 1, 2}`. It holds initially (0, 1, 2) and is
//!    preserved because each side changes ownership only by `middle.swap`,
//!    which exchanges its private index with the shared one atomically. RMWs
//!    on one location always read the latest value in its modification order,
//!    so two swaps can never both obtain the same index. A slot is therefore
//!    accessed only by the side whose private index names it.
//! 2. **Writer to reader.** The writer stores the record into `back`
//!    (non-atomic), then `swap(back | DIRTY, AcqRel)`. The reader's next
//!    `swap(front, AcqRel)` that returns that index reads-from exactly that
//!    writer swap (the reader is the only one to clear `DIRTY`, and the
//!    writer's later swaps would return the index to the writer, not the
//!    reader). Release-to-acquire on `middle` gives *write happens-before
//!    read*, so the reader copies a complete, untorn record.
//! 3. **Reader to writer.** The reader copies `front` (non-atomic) before its
//!    `swap` hands `front` to `middle` (release). The writer's swap that later
//!    returns that index reads-from the reader's swap (acquire), so the
//!    reader's copy happens-before the writer's next overwrite. If the writer's
//!    swap instead reads-from its own previous swap, it recovers its own
//!    unread slot (latest-wins overwrite), which is sequenced-before.
//! 4. **No data race.** By 1-3 every pair of non-atomic accesses to one slot,
//!    at least one of which is a write, is ordered by happens-before. This is
//!    the C++20/Rust memory model argument and does not depend on x86 TSO; on
//!    AArch64 the AcqRel swap is `SWPAL` (LSE) or an `LDAXR`/`STLXR` pair, and
//!    the copy itself needs no fences.
//! 5. **Hint load.** The reader's preliminary `middle.load(Relaxed)` only
//!    decides whether to swap. Missing a just-set `DIRTY` delays that record
//!    to the next tick; a seen `DIRTY` stays set until the reader's own swap.
//!
//! `tests::protocol_model_never_overlaps_slot_access` exhaustively enumerates
//! interleavings of the index protocol and checks invariant 1 and that the
//! reader never reads a slot the writer owns. The two-thread tear test and
//! `gx-app/tests/seam_threads.rs` check payload coherence under contention.
//!
//! # Command edges (bounded, lossless)
//!
//! The writer keeps one [`EdgeCounter`] per [`Command`]: a wrapping count of
//! edges and the publication number of the most recent one. Every record
//! carries all three counters, so any later record subsumes any earlier
//! one: overwriting an unread record cannot lose an edge. The reader keeps the
//! counts it last consumed; on each read it reports every command whose count
//! advanced, **once per kind**, ordered by its most recent edge's publication
//! number. Edges within one publication (one input report) are ordered
//! `Start`, `Pause`, `Reset`, so the higher-precedence command is applied last
//! and determines the outcome. Because no simulation step runs between edges
//! delivered in one read, replaying the last occurrence of each kind in that
//! order produces the same run state as replaying every edge in arrival order
//! (`Start`/`Reset` set the state unconditionally, `Pause` is idempotent).
//! Pause and Reset edges published before an input loss are still delivered;
//! Start is gated (below). Repeated edges of one kind collapse to one; a burst of several `Reset`s
//! yields one reset and one generation step. Collapsed edges are counted in
//! [`TickControls::coalesced_edges`] and [`ReaderStats`]. State is fixed:
//! three `(u64, u64)` pairs. Only valid input frames create edges.
//!
//! **Start gating (INPUT-002).** Each record also carries
//! `valid_since_publication`, the publication at which input last became
//! valid. The reader withholds a pending Start (consumed, counted in
//! `rejected_starts`, never delivered later) unless the status is `Valid`
//! *and* the Start's latest edge is at or after `valid_since_publication`.
//! So a Start pressed before a loss, or read while input is lost or stale,
//! can never lift a pause; the pilot must press Start again with live input.
//! `SimCore` independently refuses Start unless its input status is `Valid`.
//!
//! # Validity and staleness
//!
//! A record is [`InputValidity::Valid`] or `Unavailable` with a reason. The
//! writer stores **neutral** ([`ControlInput::default`]) axes in every
//! unavailable record, and the reader independently returns neutral axes
//! whenever the result is not [`ControlStatus::Valid`], so gx-input's
//! fail-closed `Disarmed`/`ResyncRequired` states never reach the FDM as the
//! last values.
//!
//! **Stale** means the reader has observed no *new* publication for more
//! than `stale_after_ns` of **simulation clock** time (default
//! [`DEFAULT_STALE_AFTER_NS`], 100 ms = 12 ticks). It is measured on the
//! reader's clock only, from the moment the reader first saw the current
//! publication number, so it needs no agreement between I/O and simulation
//! clocks. Because an idle joystick emits no events, the I/O thread must
//! republish ([`ControlWriter::heartbeat`]) at least every
//! [`HEARTBEAT_PERIOD_NS`]; staleness therefore detects a stalled or dead I/O
//! thread, not a stick held still. A stale result returns neutral axes and
//! [`ControlStatus::Stale`]; the simulation keeps stepping (it never waits)
//! and the status is logged. Fresh valid publications restore control
//! immediately. Before the first publication the status is
//! [`ControlStatus::NoInput`] (neutral).

use crate::{Command, ControlInput};
use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

/// Default staleness bound: 100 ms, i.e. 12 ticks at 120 Hz.
pub const DEFAULT_STALE_AFTER_NS: u64 = 100_000_000;
/// The I/O thread republishes at least this often, even without input.
pub const HEARTBEAT_PERIOD_NS: u64 = 10_000_000;

const INDEX: u8 = 0b011;
const DIRTY: u8 = 0b100;

/// Why controls are not valid. Neutral axes accompany every reason.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UnavailableReason {
    /// No device is open.
    Disconnected,
    /// gx-input detected queue loss (`SYN_DROPPED`) and is discarding.
    InputLoss,
    /// gx-input is `Disarmed` until a complete device snapshot resyncs it.
    ResyncRequired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputValidity {
    Valid,
    Unavailable(UnavailableReason),
}

/// Cumulative edge count for one command plus the publication number of the
/// most recent edge (0 = never).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EdgeCounter {
    pub count: u64,
    pub last_publication: u64,
}

/// Command edges observed in one input report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CommandEdges {
    pub start: bool,
    pub pause: bool,
    pub reset: bool,
}

/// One complete publication. All fields are written together.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlRecord {
    /// 1-based, +1 on every publication including heartbeats; 0 = never.
    pub publication: u64,
    /// I/O-thread monotonic clock at publication (telemetry only).
    pub published_at_ns: u64,
    pub validity: InputValidity,
    /// Normalized; neutral unless `validity` is `Valid`.
    pub controls: ControlInput,
    /// Source input frame sequence and timestamp (for unavailable records,
    /// the loss's source sequence and timestamp).
    pub input_sequence: u64,
    pub input_timestamp_ns: u64,
    /// Indexed by [`command_index`]: Start, Pause, Reset.
    pub edges: [EdgeCounter; 3],
    /// Publication at which `validity` last became `Valid` (0 = never).
    pub valid_since_publication: u64,
}

impl ControlRecord {
    const NEVER: Self = Self {
        publication: 0,
        published_at_ns: 0,
        validity: InputValidity::Unavailable(UnavailableReason::Disconnected),
        controls: ControlInput {
            roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, left_brake: 0.0, right_brake: 0.0, trim: 0.0,
            levers: crate::Levers { gear_down: true, flaps: 0.0, speedbrake: 0.0 },
            ap: crate::ApInput::DEFAULT,
        },
        input_sequence: 0,
        input_timestamp_ns: 0,
        edges: [EdgeCounter { count: 0, last_publication: 0 }; 3],
        valid_since_publication: 0,
    };
}

/// Edge order inside one publication: lowest precedence first.
const COMMANDS: [Command; 3] = [Command::Start, Command::Pause, Command::Reset];

pub fn command_index(command: Command) -> usize {
    match command {
        Command::Start => 0,
        Command::Pause => 1,
        Command::Reset => 2,
    }
}

struct Shared {
    slots: [UnsafeCell<ControlRecord>; 3],
    middle: AtomicU8,
}

// Safety: see the module-level ordering argument. Each slot is accessed only
// by the unique handle whose private index names it, and ownership moves only
// through AcqRel swaps on `middle`.
unsafe impl Sync for Shared {}

/// Create the connected writer/reader pair. Both start owning a slot holding
/// the "never published" record.
pub fn control_mailbox() -> (ControlWriter, ControlReader) {
    control_mailbox_with_stale_after(DEFAULT_STALE_AFTER_NS)
}

pub fn control_mailbox_with_stale_after(stale_after_ns: u64) -> (ControlWriter, ControlReader) {
    let shared = Arc::new(Shared {
        slots: [
            UnsafeCell::new(ControlRecord::NEVER),
            UnsafeCell::new(ControlRecord::NEVER),
            UnsafeCell::new(ControlRecord::NEVER),
        ],
        middle: AtomicU8::new(1),
    });
    (
        ControlWriter { shared: Arc::clone(&shared), back: 0, current: ControlRecord::NEVER },
        ControlReader {
            shared,
            front: 2,
            consumed: [0; 3],
            seen_publication: 0,
            seen_at_ns: 0,
            stale_after_ns,
            stats: ReaderStats::default(),
        },
    )
}

/// I/O-thread handle. `Send`, not `Clone`.
pub struct ControlWriter {
    shared: Arc<Shared>,
    back: u8,
    /// The writer's own copy of the latest record (counters, heartbeat).
    current: ControlRecord,
}

impl ControlWriter {
    /// Publish a complete valid input frame and the command edges it carried.
    pub fn publish_valid(
        &mut self,
        controls: ControlInput,
        input_sequence: u64,
        input_timestamp_ns: u64,
        edges: CommandEdges,
        now_ns: u64,
    ) {
        let publication = self.current.publication + 1;
        for (command, edge) in COMMANDS.into_iter().zip([edges.start, edges.pause, edges.reset]) {
            if edge {
                let counter = &mut self.current.edges[command_index(command)];
                counter.count = counter.count.wrapping_add(1);
                counter.last_publication = publication;
            }
        }
        if self.current.validity != InputValidity::Valid {
            self.current.valid_since_publication = publication;
        }
        self.current.validity = InputValidity::Valid;
        self.current.controls = controls.normalized();
        self.current.input_sequence = input_sequence;
        self.current.input_timestamp_ns = input_timestamp_ns;
        self.commit(now_ns);
    }

    /// Fail closed: neutral axes, no edges, with the reason.
    pub fn publish_unavailable(
        &mut self,
        reason: UnavailableReason,
        input_sequence: u64,
        input_timestamp_ns: u64,
        now_ns: u64,
    ) {
        self.current.validity = InputValidity::Unavailable(reason);
        self.current.controls = ControlInput::default();
        self.current.input_sequence = input_sequence;
        self.current.input_timestamp_ns = input_timestamp_ns;
        self.commit(now_ns);
    }

    /// Republish the current state (same validity, axes and counters) with a
    /// new publication number. Call at least every [`HEARTBEAT_PERIOD_NS`].
    pub fn heartbeat(&mut self, now_ns: u64) {
        self.commit(now_ns);
    }

    pub fn current(&self) -> &ControlRecord {
        &self.current
    }

    /// **Replay only.** Publish a record captured from a reader
    /// ([`TickControls::record`]) exactly as it was, publication number and
    /// stamp included, so a fresh reader fed the same records at the same
    /// read times returns the same [`TickControls`] as the live run did.
    /// Records must be replayed in capture order (publication numbers
    /// strictly increasing); a non-increasing one is ignored and returns
    /// false. Never used by a live input path.
    pub fn publish_record(&mut self, record: ControlRecord) -> bool {
        if record.publication <= self.current.publication {
            return false;
        }
        self.current = record;
        self.store();
        true
    }

    fn commit(&mut self, now_ns: u64) {
        self.current.publication += 1;
        self.current.published_at_ns = now_ns;
        self.store();
    }

    fn store(&mut self) {
        // Safety: `back` is owned exclusively by this writer (invariant 1).
        unsafe { *self.shared.slots[usize::from(self.back)].get() = self.current; }
        let previous = self.shared.middle.swap(self.back | DIRTY, Ordering::AcqRel);
        self.back = previous & INDEX;
    }
}

/// Simulation-tick view of the mailbox.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlStatus {
    Valid,
    /// Nothing has ever been published.
    NoInput,
    /// No new publication for `age_ns` (> stale bound) of simulation time.
    Stale { age_ns: u64 },
    Unavailable(UnavailableReason),
}

/// Up to one command of each kind, in application order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CommandBatch {
    commands: [Option<Command>; 3],
}

impl CommandBatch {
    pub fn iter(&self) -> impl Iterator<Item = Command> + '_ {
        self.commands.iter().flatten().copied()
    }
    pub fn is_empty(&self) -> bool {
        self.commands[0].is_none()
    }
    pub fn len(&self) -> usize {
        self.iter().count()
    }
}

/// What the simulation applies at one tick boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TickControls {
    /// Neutral unless `status == Valid`.
    pub controls: ControlInput,
    pub status: ControlStatus,
    pub commands: CommandBatch,
    /// Edges in this read beyond the one reported per kind.
    pub coalesced_edges: u64,
    /// A Start edge was consumed but not delivered: input was not valid, or
    /// the Start predates the current valid stretch (INPUT-002).
    pub rejected_start: bool,
    /// The record this read was taken from.
    pub record: ControlRecord,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReaderStats {
    pub reads: u64,
    pub new_publications: u64,
    /// Publications overwritten before any read (latest-wins), inferred from
    /// publication-number gaps.
    pub skipped_publications: u64,
    pub commands_delivered: u64,
    pub coalesced_edges: u64,
    /// Start kinds consumed but withheld (see [`TickControls::rejected_start`]).
    /// `commands_delivered + rejected_starts + coalesced_edges` equals all
    /// edges consumed.
    pub rejected_starts: u64,
    pub stale_reads: u64,
    pub unavailable_reads: u64,
    pub no_input_reads: u64,
}

/// Simulation-thread handle. `Send`, not `Clone`.
pub struct ControlReader {
    shared: Arc<Shared>,
    front: u8,
    consumed: [u64; 3],
    seen_publication: u64,
    seen_at_ns: u64,
    stale_after_ns: u64,
    stats: ReaderStats,
}

impl ControlReader {
    /// One nonblocking read at a tick boundary, on the simulation clock.
    pub fn read(&mut self, now_ns: u64) -> TickControls {
        if self.shared.middle.load(Ordering::Relaxed) & DIRTY != 0 {
            let previous = self.shared.middle.swap(self.front, Ordering::AcqRel);
            self.front = previous & INDEX;
        }
        // Safety: `front` is owned exclusively by this reader (invariant 1).
        let record = unsafe { *self.shared.slots[usize::from(self.front)].get() };
        self.stats.reads += 1;

        if record.publication != self.seen_publication {
            debug_assert!(record.publication > self.seen_publication);
            self.stats.new_publications += 1;
            self.stats.skipped_publications += record.publication - self.seen_publication - 1;
            self.seen_publication = record.publication;
            self.seen_at_ns = now_ns;
        }

        let status = if record.publication == 0 {
            self.stats.no_input_reads += 1;
            ControlStatus::NoInput
        } else if now_ns.saturating_sub(self.seen_at_ns) > self.stale_after_ns {
            self.stats.stale_reads += 1;
            ControlStatus::Stale { age_ns: now_ns.saturating_sub(self.seen_at_ns) }
        } else {
            match record.validity {
                InputValidity::Valid => ControlStatus::Valid,
                InputValidity::Unavailable(reason) => {
                    self.stats.unavailable_reads += 1;
                    ControlStatus::Unavailable(reason)
                }
            }
        };
        // Start gating (INPUT-002): a Start is applied only if input is valid
        // now and the Start was pressed within the current valid stretch, so
        // an edge pressed before a loss can never lift a loss pause.
        let start_ok = status == ControlStatus::Valid
            && record.edges[command_index(Command::Start)].last_publication
                >= record.valid_since_publication;
        let mut pending: [(u64, usize); 3] = [(0, 0); 3];
        let mut n = 0;
        let mut coalesced = 0u64;
        let mut rejected_start = false;
        for (i, counter) in record.edges.iter().enumerate() {
            let delta = counter.count.wrapping_sub(self.consumed[i]);
            if delta != 0 {
                self.consumed[i] = counter.count;
                coalesced += delta - 1;
                if COMMANDS[i] == Command::Start && !start_ok {
                    rejected_start = true;
                    continue;
                }
                pending[n] = (counter.last_publication, i);
                n += 1;
            }
        }
        // Most recent edge last; ties (same report) in Start, Pause, Reset order.
        pending[..n].sort_unstable();
        let mut commands = CommandBatch::default();
        for (slot, &(_, i)) in commands.commands.iter_mut().zip(&pending[..n]) {
            *slot = Some(COMMANDS[i]);
        }
        self.stats.commands_delivered += n as u64;
        self.stats.coalesced_edges += coalesced;
        self.stats.rejected_starts += u64::from(rejected_start);

        let controls = if status == ControlStatus::Valid {
            record.controls
        } else {
            ControlInput::default()
        };
        TickControls { controls, status, commands, coalesced_edges: coalesced, rejected_start, record }
    }

    pub fn stats(&self) -> ReaderStats {
        self.stats
    }

    pub fn stale_after_ns(&self) -> u64 {
        self.stale_after_ns
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Barrier;

    fn axes(v: f32) -> ControlInput {
        ControlInput { roll: v, pitch: -v, yaw: v * 0.5, throttle: v.abs(), ..Default::default() }
    }

    const NONE: CommandEdges = CommandEdges { start: false, pause: false, reset: false };
    const START: CommandEdges = CommandEdges { start: true, pause: false, reset: false };
    const PAUSE: CommandEdges = CommandEdges { start: false, pause: true, reset: false };
    const RESET: CommandEdges = CommandEdges { start: false, pause: false, reset: true };

    fn commands(t: &TickControls) -> Vec<Command> {
        t.commands.iter().collect()
    }

    #[test]
    fn replayed_records_reproduce_every_read() {
        // Live: a writer with edges, a loss, heartbeats and a stale gap; the
        // reader's view of each tick is captured with the record it read.
        let (mut w, mut r) = control_mailbox();
        let mut live = Vec::new();
        let mut t = 0u64;
        for i in 0..400u64 {
            t += 8_333_333;
            match i % 97 {
                0 => w.publish_valid(axes(0.4), i, t - 5, START, t - 3),
                1 => w.publish_valid(axes(-0.2), i, t - 5, PAUSE, t - 2),
                40 => w.publish_unavailable(UnavailableReason::InputLoss, i, t - 4, t - 1),
                41 => w.publish_valid(axes(0.1), i, t - 5, RESET, t - 1),
                60..=75 => {} // I/O silent: goes stale.
                _ => w.heartbeat(t - 1),
            }
            live.push((t, r.read(t)));
        }
        // Replay: publish each newly seen record, then read at the same time.
        let (mut w2, mut r2) = control_mailbox();
        let mut last = 0;
        for (t, tick) in &live {
            if tick.record.publication != last {
                last = tick.record.publication;
                assert!(w2.publish_record(tick.record));
            }
            assert_eq!(r2.read(*t), *tick);
        }
        assert_eq!(r2.stats(), r.stats());
        assert!(!w2.publish_record(live[0].1.record), "out-of-order record refused");
    }

    #[test]
    fn no_input_until_first_publication_then_latest_wins() {
        let (mut w, mut r) = control_mailbox();
        let t = r.read(0);
        assert_eq!(t.status, ControlStatus::NoInput);
        assert_eq!(t.controls, ControlInput::default());
        w.publish_valid(axes(0.1), 1, 10, NONE, 1);
        w.publish_valid(axes(0.2), 2, 20, NONE, 2);
        w.publish_valid(axes(0.3), 3, 30, NONE, 3);
        let t = r.read(4);
        assert_eq!(t.status, ControlStatus::Valid);
        assert_eq!(t.controls, axes(0.3));
        assert_eq!((t.record.input_sequence, t.record.input_timestamp_ns), (3, 30));
        assert_eq!(r.stats().skipped_publications, 2);
        // Re-reading with nothing new returns the same latest value.
        assert_eq!(r.read(5).controls, axes(0.3));
        assert_eq!(r.stats().new_publications, 1);
    }

    #[test]
    fn edges_survive_overwrite_and_replay_in_arrival_order() {
        let (mut w, mut r) = control_mailbox();
        // Five reports between two ticks: R, S, P, S, (axes only).
        w.publish_valid(axes(0.0), 1, 1, RESET, 1);
        w.publish_valid(axes(0.0), 2, 2, START, 2);
        w.publish_valid(axes(0.0), 3, 3, PAUSE, 3);
        w.publish_valid(axes(0.0), 4, 4, START, 4);
        w.publish_valid(axes(0.5), 5, 5, NONE, 5);
        let t = r.read(6);
        assert_eq!(commands(&t), vec![Command::Reset, Command::Pause, Command::Start]);
        assert_eq!(t.coalesced_edges, 1, "second Start collapses");
        assert_eq!(t.controls, axes(0.5));
        // Consumed exactly once.
        assert!(r.read(7).commands.is_empty());
    }

    #[test]
    fn simultaneous_edges_apply_highest_precedence_last() {
        let (mut w, mut r) = control_mailbox();
        w.publish_valid(axes(0.0), 1, 1, CommandEdges { start: true, pause: true, reset: true }, 1);
        assert_eq!(commands(&r.read(2)), vec![Command::Start, Command::Pause, Command::Reset]);
        w.publish_valid(axes(0.0), 2, 2, CommandEdges { start: true, pause: true, reset: false }, 3);
        assert_eq!(commands(&r.read(4)), vec![Command::Start, Command::Pause]);
    }

    #[test]
    fn pause_and_reset_before_input_loss_are_still_delivered_start_is_not() {
        let (mut w, mut r) = control_mailbox();
        w.publish_valid(axes(0.9), 1, 1, CommandEdges { start: true, pause: true, reset: true }, 1);
        w.publish_unavailable(UnavailableReason::InputLoss, 9, 2, 2);
        let t = r.read(3);
        assert_eq!(commands(&t), vec![Command::Pause, Command::Reset]);
        assert!(t.rejected_start);
        assert_eq!(t.status, ControlStatus::Unavailable(UnavailableReason::InputLoss));
        assert_eq!(t.controls, ControlInput::default(), "never the last valid axes");
        assert_eq!(r.stats().rejected_starts, 1);
    }

    #[test]
    fn start_from_before_a_loss_never_survives_into_the_next_valid_stretch() {
        let (mut w, mut r) = control_mailbox();
        w.publish_valid(axes(0.1), 1, 1, NONE, 1);
        assert!(r.read(1).commands.is_empty());
        // Start, loss and recovery all land between two ticks.
        w.publish_valid(axes(0.1), 2, 2, START, 2);
        w.publish_unavailable(UnavailableReason::ResyncRequired, 2, 3, 3);
        w.publish_valid(axes(0.2), 3, 4, NONE, 4);
        let t = r.read(5);
        assert_eq!(t.status, ControlStatus::Valid);
        assert!(t.commands.is_empty() && t.rejected_start, "pre-loss Start withheld");
        // Not redelivered later either; a fresh Start is.
        assert!(r.read(6).commands.is_empty());
        w.publish_valid(axes(0.2), 4, 5, START, 7);
        assert_eq!(commands(&r.read(8)), vec![Command::Start]);
    }

    #[test]
    fn unavailable_and_heartbeat_stay_neutral_until_valid_again() {
        let (mut w, mut r) = control_mailbox();
        w.publish_valid(axes(0.7), 1, 1, NONE, 1);
        assert_eq!(r.read(1).controls, axes(0.7));
        w.publish_unavailable(UnavailableReason::ResyncRequired, 1, 2, 2);
        w.heartbeat(3);
        let t = r.read(4);
        assert_eq!(t.status, ControlStatus::Unavailable(UnavailableReason::ResyncRequired));
        assert_eq!(t.controls, ControlInput::default());
        assert_eq!(t.record.controls, ControlInput::default());
        w.publish_valid(axes(-0.2), 2, 5, NONE, 5);
        assert_eq!(r.read(6).controls, axes(-0.2));
    }

    #[test]
    fn stale_after_bound_without_new_publication_and_recovers() {
        let (mut w, mut r) = control_mailbox_with_stale_after(100);
        w.publish_valid(axes(0.4), 1, 1, NONE, 0);
        assert_eq!(r.read(1000).status, ControlStatus::Valid); // first seen at 1000
        assert_eq!(r.read(1100).status, ControlStatus::Valid); // exactly at bound
        let t = r.read(1101);
        assert_eq!(t.status, ControlStatus::Stale { age_ns: 101 });
        assert_eq!(t.controls, ControlInput::default());
        w.heartbeat(1102); // I/O thread alive again: same axes, fresh
        let t = r.read(1103);
        assert_eq!((t.status, t.controls), (ControlStatus::Valid, axes(0.4)));
        assert_eq!(r.stats().stale_reads, 1);
    }

    /// Exhaustive model of the index protocol: every interleaving of writer
    /// steps (begin write, end write, swap) and reader steps (hint load,
    /// swap, begin read, end read). Checks the permutation invariant, that a
    /// slot being written is never being read, and that the reader never
    /// observes publications going backwards.
    #[test]
    fn protocol_model_never_overlaps_slot_access() {
        #[derive(Clone, Copy)]
        struct State {
            middle: u8,
            back: u8,
            front: u8,
            slot_pub: [u32; 3],
            w_pc: u8, w_done: u32, writing: bool,
            r_pc: u8, r_done: u32, reading: bool, hint_dirty: bool, last_seen: u32,
        }
        const WRITES: u32 = 4;
        const READS: u32 = 3;
        fn check(s: &State) {
            let mut seen = [false; 3];
            for i in [s.back, s.middle & INDEX, s.front] {
                assert!(!seen[usize::from(i)], "index duplicated");
                seen[usize::from(i)] = true;
            }
            assert!(!(s.writing && s.reading && s.back == s.front), "overlapping access");
        }
        fn explore(s: State, count: &mut u64) {
            check(&s);
            let mut progressed = false;
            if s.w_done < WRITES {
                progressed = true;
                let mut n = s;
                match s.w_pc {
                    0 => { n.writing = true; n.w_pc = 1; }
                    1 => {
                        n.slot_pub[usize::from(s.back)] = s.w_done + 1;
                        n.writing = false; n.w_pc = 2;
                    }
                    _ => {
                        let prev = n.middle;
                        n.middle = s.back | DIRTY;
                        n.back = prev & INDEX;
                        n.w_pc = 0; n.w_done += 1;
                    }
                }
                explore(n, count);
            }
            if s.r_done < READS {
                progressed = true;
                let mut n = s;
                match s.r_pc {
                    0 => { n.hint_dirty = s.middle & DIRTY != 0; n.r_pc = 1; }
                    1 => {
                        if s.hint_dirty {
                            let prev = n.middle;
                            n.middle = s.front;
                            n.front = prev & INDEX;
                        }
                        n.r_pc = 2;
                    }
                    2 => { n.reading = true; n.r_pc = 3; }
                    _ => {
                        let p = s.slot_pub[usize::from(s.front)];
                        assert!(p >= s.last_seen, "publication went backwards");
                        n.last_seen = p;
                        n.reading = false; n.r_pc = 0; n.r_done += 1;
                    }
                }
                explore(n, count);
            }
            if !progressed { *count += 1; }
        }
        let start = State {
            middle: 1, back: 0, front: 2, slot_pub: [0; 3],
            w_pc: 0, w_done: 0, writing: false,
            r_pc: 0, r_done: 0, reading: false, hint_dirty: false, last_seen: 0,
        };
        let mut terminal = 0;
        explore(start, &mut terminal);
        assert!(terminal > 10_000, "interleavings explored: {terminal}");
    }

    /// Payload coherence under real contention: every field of a record is a
    /// function of its publication number, so a torn copy is detectable.
    #[test]
    fn two_thread_reads_are_never_torn_and_edges_are_never_lost() {
        const N: u64 = 200_000;
        let (mut w, mut r) = control_mailbox_with_stale_after(u64::MAX);
        let barrier = Arc::new(Barrier::new(2));
        let wb = Arc::clone(&barrier);
        let writer = std::thread::spawn(move || {
            wb.wait();
            for i in 1..=N {
                let v = (i % 1000) as f32 / 1000.0;
                let edges = CommandEdges { start: i % 3 == 0, pause: i % 5 == 0, reset: i % 7 == 0 };
                w.publish_valid(axes(v), i, i * 10, edges, i);
            }
        });
        barrier.wait();
        let mut totals = [0u64; 3];
        let mut last_input = 0;
        let mut now = 0;
        loop {
            now += 1;
            let finished = writer.is_finished();
            let t = r.read(now);
            let rec = t.record;
            if rec.publication != 0 {
                let i = rec.input_sequence;
                assert_eq!(rec.publication, i, "publication/sequence torn");
                assert_eq!(rec.input_timestamp_ns, i * 10);
                assert_eq!(rec.published_at_ns, i);
                assert_eq!(rec.controls, axes((i % 1000) as f32 / 1000.0).normalized());
                assert_eq!(rec.edges[0].count, i / 3);
                assert_eq!(rec.edges[1].count, i / 5);
                assert_eq!(rec.edges[2].count, i / 7);
                assert!(i >= last_input, "went backwards");
                last_input = i;
            }
            for c in t.commands.iter() {
                totals[command_index(c)] += 1;
            }
            if finished && rec.publication == N { break; }
        }
        writer.join().unwrap();
        let s = r.stats();
        assert_eq!(s.commands_delivered + s.coalesced_edges, N / 3 + N / 5 + N / 7,
            "every edge delivered or counted as coalesced");
        assert_eq!(s.new_publications + s.skipped_publications, N);
        assert!(s.new_publications > 1);
        for (k, expected) in [N / 3, N / 5, N / 7].into_iter().enumerate() {
            assert!(totals[k] >= 1 && totals[k] <= expected);
        }
    }
}

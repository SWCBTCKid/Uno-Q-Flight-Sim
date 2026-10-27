use crate::AircraftSnapshot;
use std::array;
use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};

const SLOT_COUNT: usize = 3;
const NO_SLOT: usize = 3;
const EMPTY_DESCRIPTOR: u64 = 0;
const MAX_VERSION: u64 = (1u64 << 60) - 1;
const WRITER: usize = usize::MAX;

struct Slot {
    /// 0 = free, 1.. = pinned readers, usize::MAX = writer reservation.
    ownership: AtomicUsize,
    value: UnsafeCell<MaybeUninit<AircraftSnapshot>>,
}

impl Slot {
    fn new() -> Self {
        Self { ownership: AtomicUsize::new(0), value: UnsafeCell::new(MaybeUninit::uninit()) }
    }

    fn try_pin(&self) -> bool {
        let readers = self.ownership.load(Ordering::SeqCst);
        readers != WRITER
            && readers < WRITER - 1
            && self.ownership.compare_exchange(
                readers, readers + 1, Ordering::SeqCst, Ordering::SeqCst
            ).is_ok()
    }

    fn unpin(&self) { self.ownership.fetch_sub(1, Ordering::SeqCst); }
}

// Safety: access to `value` follows the reader-pinning protocol below. There
// is one writer. It writes only after atomically changing a slot from free to
// WRITER. Readers copy only after atomically incrementing that same word and
// revalidating the SeqCst descriptor. No read and write can therefore overlap.
unsafe impl Sync for Slot {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SnapshotPair {
    pub previous: Option<AircraftSnapshot>,
    pub latest: AircraftSnapshot,
}

/// Both unpublished slots are still pinned. The writer never waits and can
/// retry the same snapshot later.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PublishBusy(pub AircraftSnapshot);

/// Bounded single-writer, multi-reader triple buffer. The descriptor contains
/// a 60-bit non-repeating version and current/previous indices. See the design
/// document for the AArch64 ordering proof.
pub struct SnapshotExchange {
    slots: [Slot; SLOT_COUNT],
    descriptor: AtomicU64,
}

impl Default for SnapshotExchange {
    fn default() -> Self { Self::new() }
}

impl SnapshotExchange {
    pub fn new() -> Self {
        Self {
            slots: array::from_fn(|_| Slot::new()),
            descriptor: AtomicU64::new(EMPTY_DESCRIPTOR),
        }
    }

    /// Scan exactly three slots; never wait for a reader.
    pub fn try_publish(&self, snapshot: AircraftSnapshot) -> Result<(), PublishBusy> {
        assert_ne!(snapshot.sequence, 0, "snapshot sequence zero is reserved");
        assert!(snapshot.sequence <= MAX_VERSION, "snapshot sequence exhausted");
        let old = self.descriptor.load(Ordering::SeqCst);
        assert!(old == EMPTY_DESCRIPTOR || snapshot.sequence > (old >> 4),
            "snapshot sequence must increase on every successful publication");
        let (current, previous) = decode_slots(old);
        let candidate = (0..SLOT_COUNT).find(|&index| {
            index != current
                && index != previous
                && self.slots[index].ownership.compare_exchange(
                    0, WRITER, Ordering::SeqCst, Ordering::SeqCst
                ).is_ok()
        });
        let Some(candidate) = candidate else { return Err(PublishBusy(snapshot)); };

        // The single writer owns this unpublished, unpinned slot.
        unsafe { (*self.slots[candidate].value.get()).write(snapshot); }
        self.descriptor.store(encode(snapshot.sequence, candidate, current), Ordering::SeqCst);
        self.slots[candidate].ownership.store(0, Ordering::SeqCst);
        Ok(())
    }

    /// At most three pin/revalidation attempts, then `None`.
    pub fn try_latest_pair(&self) -> Option<SnapshotPair> {
        for _ in 0..SLOT_COUNT {
            let observed = self.descriptor.load(Ordering::SeqCst);
            if observed == EMPTY_DESCRIPTOR { return None; }
            let (current, previous) = decode_slots(observed);
            if !self.slots[current].try_pin() { continue; }
            if previous != NO_SLOT && !self.slots[previous].try_pin() {
                self.slots[current].unpin();
                continue;
            }
            if self.descriptor.load(Ordering::SeqCst) != observed {
                self.unpin(current, previous);
                continue;
            }

            let latest = unsafe { (*self.slots[current].value.get()).assume_init_read() };
            let mut prior = if previous == NO_SLOT {
                None
            } else {
                Some(unsafe { (*self.slots[previous].value.get()).assume_init_read() })
            };
            self.unpin(current, previous);
            if prior.is_some_and(|p| !same_interpolation_frame(p, latest)) { prior = None; }
            return Some(SnapshotPair { previous: prior, latest });
        }
        None
    }

    fn unpin(&self, current: usize, previous: usize) {
        if previous != NO_SLOT {
            self.slots[previous].unpin();
        }
        self.slots[current].unpin();
    }
}

fn same_interpolation_frame(a: AircraftSnapshot, b: AircraftSnapshot) -> bool {
    if a.generation != b.generation { return false; }
    match (a.state.local_enu, b.state.local_enu) {
        (Some(a), Some(b)) => {
            a.frame_id == b.frame_id && a.frame_generation == b.frame_generation
        }
        _ => false,
    }
}

fn encode(version: u64, current: usize, previous: usize) -> u64 {
    debug_assert!(version > 0 && version <= MAX_VERSION);
    debug_assert!(current < SLOT_COUNT);
    debug_assert!(previous < SLOT_COUNT || previous == NO_SLOT);
    (version << 4) | ((previous as u64) << 2) | current as u64
}

fn decode_slots(descriptor: u64) -> (usize, usize) {
    if descriptor == EMPTY_DESCRIPTOR { return (NO_SLOT, NO_SLOT); }
    ((descriptor & 0x3) as usize, ((descriptor >> 2) & 0x3) as usize)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AircraftState, GeodeticPosition, LocalEnuPose};
    use std::sync::{Arc, Barrier};

    fn snapshot(sequence: u64, generation: u64) -> AircraftSnapshot {
        AircraftSnapshot {
            sequence, generation, simulation_tick: sequence,
            simulation_time_ns: sequence * 10, published_at_ns: sequence * 11,
            state: AircraftState {
                geodetic: GeodeticPosition {
                    latitude_deg: sequence as f64, longitude_deg: -122.0,
                    ellipsoid_height_m: sequence as f64 + 100.0, ..Default::default()
                },
                local_enu: Some(LocalEnuPose {
                    frame_id: 9, frame_generation: 2,
                    position_m: [sequence as f64; 3],
                    velocity_mps: [sequence as f64 + 0.5; 3],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                }),
                weight_on_wheels: sequence % 2 == 0,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn returns_consistent_latest_and_previous() {
        let x = SnapshotExchange::new();
        assert_eq!(x.try_latest_pair(), None);
        x.try_publish(snapshot(1, 0)).unwrap();
        assert_eq!(x.try_latest_pair().unwrap(), SnapshotPair { previous: None, latest: snapshot(1, 0) });
        x.try_publish(snapshot(2, 0)).unwrap();
        assert_eq!(x.try_latest_pair().unwrap(), SnapshotPair { previous: Some(snapshot(1, 0)), latest: snapshot(2, 0) });
        x.try_publish(snapshot(3, 1)).unwrap();
        assert_eq!(x.try_latest_pair().unwrap(), SnapshotPair { previous: None, latest: snapshot(3, 1) });

        let mut changed_frame = snapshot(4, 1);
        changed_frame.state.local_enu.as_mut().unwrap().frame_generation = 3;
        x.try_publish(changed_frame).unwrap();
        assert_eq!(x.try_latest_pair().unwrap().previous, None);
    }

    /// Models the weak-memory slot-reuse finding. A delayed reader pins a stale
    /// slot just as it becomes the writer's only candidate. The SeqCst pin is
    /// ordered before the writer's zero check here, so publication reports
    /// Busy. In the opposite order, descriptor revalidation observes change.
    #[test]
    fn stale_reader_pin_prevents_slot_reuse() {
        let x = SnapshotExchange::new();
        x.try_publish(snapshot(1, 0)).unwrap();
        let stale_descriptor = x.descriptor.load(Ordering::SeqCst);
        let (stale_current, _) = decode_slots(stale_descriptor);
        x.try_publish(snapshot(2, 0)).unwrap();
        x.try_publish(snapshot(3, 0)).unwrap();
        assert!(x.slots[stale_current].try_pin());
        assert_eq!(x.try_publish(snapshot(4, 0)), Err(PublishBusy(snapshot(4, 0))));
        assert_ne!(x.descriptor.load(Ordering::SeqCst), stale_descriptor);
        x.slots[stale_current].unpin();
        x.try_publish(snapshot(4, 0)).unwrap();
    }

    #[test]
    fn publication_is_consistent_under_contention() {
        let exchange = Arc::new(SnapshotExchange::new());
        let writer = Arc::clone(&exchange);
        let barrier = Arc::new(Barrier::new(2));
        let writer_barrier = Arc::clone(&barrier);
        let thread = std::thread::spawn(move || {
            writer_barrier.wait();
            let mut sequence = 1;
            while sequence <= 100_000 {
                if writer.try_publish(snapshot(sequence, 0)).is_ok() { sequence += 1; }
            }
        });
        barrier.wait();
        let mut observations = 0;
        for _ in 0..200_000 {
            if let Some(pair) = exchange.try_latest_pair() {
                let sequence = pair.latest.sequence;
                let pose = pair.latest.state.local_enu.unwrap();
                assert_eq!(pair.latest.state.geodetic.latitude_deg, sequence as f64);
                assert_eq!(pose.position_m, [sequence as f64; 3]);
                assert_eq!(pose.velocity_mps, [sequence as f64 + 0.5; 3]);
                assert_eq!(pair.latest.state.weight_on_wheels, sequence % 2 == 0);
                if let Some(previous) = pair.previous {
                    assert_eq!(previous.sequence + 1, pair.latest.sequence);
                }
                observations += 1;
            }
            if thread.is_finished() && observations > 0 { break; }
        }
        thread.join().unwrap();
        assert_eq!(exchange.try_latest_pair().unwrap().latest.sequence, 100_000);
        assert!(observations > 0);
    }
}

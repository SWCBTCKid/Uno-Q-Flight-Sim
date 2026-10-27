//! Drift-free cadence and deadline accounting. All values use the same
//! caller-supplied monotonic nanosecond epoch.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Deadline {
    pub tick: u64,
    pub at_ns: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TimingCounters {
    pub ticks: u64,
    pub missed_deadlines: u64,
    pub skipped_periods: u64,
    pub max_lateness_ns: u64,
}

/// Absolute-deadline scheduler. Deadlines are derived from `epoch`, never
/// accumulated, so the 1/120 s fractional nanosecond cannot drift.
#[derive(Clone, Copy, Debug)]
pub struct FixedRate {
    epoch_ns: u64,
    hz: u32,
    next_tick: u64,
    counters: TimingCounters,
}

impl FixedRate {
    pub fn new(epoch_ns: u64, hz: u32) -> Self {
        assert!(hz > 0, "rate must be non-zero");
        Self { epoch_ns, hz, next_tick: 1, counters: TimingCounters::default() }
    }

    /// Exact floor(tick * 1e9 / hz), saturated to u64.
    pub const fn elapsed_ns(hz: u32, tick: u64) -> u64 {
        if hz == 0 { return 0; }
        let ns = (tick as u128 * 1_000_000_000u128) / hz as u128;
        if ns > u64::MAX as u128 { u64::MAX } else { ns as u64 }
    }

    pub fn next_deadline(&self) -> Deadline {
        Deadline {
            tick: self.next_tick,
            at_ns: self.epoch_ns.saturating_add(Self::elapsed_ns(self.hz, self.next_tick)),
        }
    }

    /// Account for an attempted tick. A wake exactly on its deadline is on
    /// time. If multiple whole periods elapsed, report them as skipped while
    /// advancing to the first deadline strictly after `actual_ns`.
    pub fn complete_tick(&mut self, actual_ns: u64) {
        let due = self.next_deadline().at_ns;
        self.counters.ticks = self.counters.ticks.saturating_add(1);
        if actual_ns > due {
            self.counters.missed_deadlines = self.counters.missed_deadlines.saturating_add(1);
            self.counters.max_lateness_ns = self.counters.max_lateness_ns.max(actual_ns - due);
        }
        self.next_tick = self.next_tick.saturating_add(1);
        if actual_ns >= self.epoch_ns {
            // First tick whose floored absolute deadline is strictly later
            // than actual_ns. Compute it directly: a long suspend must not
            // create an unbounded catch-up loop.
            let delta_plus_one = (actual_ns - self.epoch_ns) as u128 + 1;
            let numerator = delta_plus_one * self.hz as u128;
            let future = (numerator + 999_999_999) / 1_000_000_000;
            let future = future.min(u64::MAX as u128) as u64;
            if future > self.next_tick {
                self.counters.skipped_periods = self
                    .counters
                    .skipped_periods
                    .saturating_add(future - self.next_tick);
                self.next_tick = future;
            }
        }
    }

    pub fn counters(&self) -> TimingCounters { self.counters }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cadence_has_no_fractional_drift() {
        assert_eq!(FixedRate::elapsed_ns(120, 1), 8_333_333);
        assert_eq!(FixedRate::elapsed_ns(120, 2), 16_666_666);
        assert_eq!(FixedRate::elapsed_ns(120, 3), 25_000_000);
        assert_eq!(FixedRate::elapsed_ns(120, 120), 1_000_000_000);
        assert_eq!(FixedRate::elapsed_ns(120, 120 * 60 * 60), 3_600_000_000_000);
    }

    #[test]
    fn counts_late_and_skipped_deadlines_separately() {
        let mut s = FixedRate::new(1_000, 100);
        s.complete_tick(10_001_000); // exactly due
        s.complete_tick(25_001_000); // 5 ms late; no full deadline skipped
        s.complete_tick(55_001_000); // due at 30 ms; skips 40 and 50 ms
        assert_eq!(s.counters(), TimingCounters {
            ticks: 3, missed_deadlines: 2, skipped_periods: 2,
            max_lateness_ns: 25_000_000,
        });
        assert_eq!(s.next_deadline().at_ns, 60_001_000);
    }

    #[test]
    fn long_suspend_is_accounted_without_catch_up_ticks() {
        let mut s = FixedRate::new(0, 120);
        s.complete_tick(3_600_000_000_000);
        assert_eq!(s.counters().ticks, 1);
        assert_eq!(s.counters().missed_deadlines, 1);
        assert_eq!(s.counters().skipped_periods, 431_999);
        assert_eq!(s.next_deadline(), Deadline {
            tick: 432_001,
            at_ns: 3_600_008_333_333,
        });
    }
}

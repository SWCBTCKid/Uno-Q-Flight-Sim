//! Vblank-locked 30 fps pacing on a 60 Hz mode, and frame statistics.
//!
//! The render thread presents every second vblank. After a flip completes at
//! vblank `V`, rendering for the next frame starts immediately. The resulting
//! buffer is not committed until just after the intermediate vblank `V + P`,
//! so both a fast and a slow (but sub-frame-budget) render latch at `V + 2P`.
//! Committing just after a vblank keeps the commit far from the latch boundary
//! where msm produces stale flip stamps (ISS-031; the spike saw one 0.84 ms
//! before a vblank).
//!
//! A flip whose kernel stamp is stale (sequence and time equal to the
//! previous event) has an unknown presentation time; its vblank is
//! estimated as the first vblank after its commit on the fresh-stamp grid.

/// Delay after a completed flip before sampling the next frame. This puts the
/// input sample inside the strict 33.3 ms sample-to-vblank gate even when the
/// physical two-vblank interval is slightly longer than its nominal timing.
pub const SAMPLE_AFTER_VBLANK_NS: u64 = 500_000;

/// Delay after the intermediate vblank before committing the next frame.
pub const WAKE_AFTER_VBLANK_NS: u64 = 1_500_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VblankClock {
    pub period_ns: u64,
    /// Vblanks per presented frame (2 for 30 fps on 60 Hz).
    pub divisor: u64,
    /// Last fresh (trustworthy) flip stamp.
    anchor_ns: Option<u64>,
}

impl VblankClock {
    /// `clock_khz`, `htotal`, `vtotal` from the DRM mode; `fps` the target.
    pub fn from_mode(clock_khz: u32, htotal: u16, vtotal: u16, fps: u32) -> Option<Self> {
        if clock_khz == 0 || htotal == 0 || vtotal == 0 || fps == 0 {
            return None;
        }
        let period_ns = u64::from(htotal) * u64::from(vtotal) * 1_000_000 / u64::from(clock_khz);
        let refresh_mhz = 1_000_000_000_000 / period_ns.max(1);
        let divisor = ((refresh_mhz + u64::from(fps) * 500) / (u64::from(fps) * 1000)).max(1);
        Some(Self { period_ns, divisor, anchor_ns: None })
    }

    pub fn anchor(&self) -> Option<u64> {
        self.anchor_ns
    }

    pub fn on_fresh_flip(&mut self, flip_ns: u64) {
        self.anchor_ns = Some(flip_ns);
    }

    /// First vblank strictly after `t` on the anchored grid.
    pub fn vblank_after(&self, t: u64) -> Option<u64> {
        let a = self.anchor_ns?;
        if t < a {
            return Some(a);
        }
        Some(a + ((t - a) / self.period_ns + 1) * self.period_ns)
    }

    /// When to sample/render the frame after one that flipped at `flip_ns`.
    /// Sampling shortly after the flip preserves almost both refresh periods
    /// for rendering while keeping sample-to-vblank below 33.3 ms on the
    /// measured mode.
    pub fn next_wake(&self, flip_ns: u64) -> u64 {
        flip_ns + SAMPLE_AFTER_VBLANK_NS
    }

    /// Earliest time to submit the rendered buffer. Delaying the commit, not
    /// the rendering, prevents a fast frame from presenting at the mode rate.
    pub fn earliest_commit(&self, flip_ns: u64) -> u64 {
        flip_ns + (self.divisor - 1) * self.period_ns + WAKE_AFTER_VBLANK_NS
    }

    pub fn frame_period_ns(&self) -> u64 {
        self.period_ns * self.divisor
    }
}

/// Bounded sample store for p50/p99/max (no allocation after construction).
pub struct Samples {
    v: Vec<u64>,
    cap: usize,
    pub count: u64,
    pub max: u64,
}

impl Samples {
    pub fn with_capacity(cap: usize) -> Self {
        Self { v: Vec::with_capacity(cap), cap, count: 0, max: 0 }
    }

    pub fn push(&mut self, x: u64) {
        self.count += 1;
        self.max = self.max.max(x);
        if self.v.len() < self.cap {
            self.v.push(x);
        }
    }

    /// Nearest-rank percentile of the stored samples.
    pub fn pct(&self, p: u64) -> Option<u64> {
        if self.v.is_empty() {
            return None;
        }
        let mut s = self.v.clone();
        s.sort_unstable();
        let rank = ((s.len() as u64 * p).div_ceil(100)).clamp(1, s.len() as u64);
        Some(s[rank as usize - 1])
    }
}

/// Fresh-to-fresh presentation intervals between consecutive frames only:
/// an interval spanning a stale-stamped or missing frame is not a frame
/// interval and is not counted.
#[derive(Default)]
pub struct IntervalTracker {
    last: Option<(u64, u64)>, // (frame, fresh flip ns)
}

impl IntervalTracker {
    /// Returns the interval if `frame` directly follows the last fresh frame.
    pub fn on_flip(&mut self, frame: u64, flip_ns: Option<u64>) -> Option<u64> {
        let out = match (self.last, flip_ns) {
            (Some((f, t)), Some(now)) if f + 1 == frame && now > t => Some(now - t),
            _ => None,
        };
        self.last = flip_ns.map(|t| (frame, t));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cea_720p60_is_two_vblanks_per_frame() {
        // CEA-861 1280x720@60: 74.25 MHz, htotal 1650, vtotal 750.
        let c = VblankClock::from_mode(74_250, 1650, 750, 30).unwrap();
        assert_eq!(c.period_ns, 16_666_666);
        assert_eq!(c.divisor, 2);
        assert_eq!(c.frame_period_ns(), 33_333_332);
        let flip = 1_000_000_000;
        assert_eq!(c.next_wake(flip), flip + SAMPLE_AFTER_VBLANK_NS);
        assert_eq!(c.earliest_commit(flip), flip + 16_666_666 + WAKE_AFTER_VBLANK_NS);
        // 59.94 Hz still presents every second vblank.
        assert_eq!(VblankClock::from_mode(74_176, 1650, 750, 30).unwrap().divisor, 2);
        assert!(VblankClock::from_mode(0, 1650, 750, 30).is_none());
    }

    #[test]
    fn rendering_gets_the_whole_two_vblank_budget() {
        let c = VblankClock::from_mode(74_250, 1650, 750, 30).unwrap();
        let flip = 1_000_000_000;
        let observed_photo_gpu_max = 25_863_000;
        let target = flip + c.frame_period_ns();

        assert!(c.next_wake(flip) + observed_photo_gpu_max < target);
        assert!(c.earliest_commit(flip) < target);
        assert!(
            flip + c.period_ns + WAKE_AFTER_VBLANK_NS + observed_photo_gpu_max > target,
            "the former render wake predictably missed the 30 fps latch"
        );
    }

    #[test]
    fn sample_offset_clears_strict_latency_gate_on_measured_kms_period() {
        // The first repaired KMS run measured 33.440 ms sample-to-vblank p99
        // when sampling at the preceding flip. Move sampling 0.5 ms later
        // while retaining ample room for the observed 24.506 ms GPU maximum.
        const OBSERVED_UNOFFSET_P99_NS: u64 = 33_440_000;
        const OBSERVED_TWO_VBLANK_P99_NS: u64 = 33_457_000;
        const OBSERVED_GPU_MAX_NS: u64 = 24_506_000;
        const STRICT_GATE_NS: u64 = 33_300_000;
        assert!(OBSERVED_UNOFFSET_P99_NS - SAMPLE_AFTER_VBLANK_NS < STRICT_GATE_NS);
        assert!(SAMPLE_AFTER_VBLANK_NS + OBSERVED_GPU_MAX_NS < OBSERVED_TWO_VBLANK_P99_NS);
    }

    #[test]
    fn stale_flip_is_placed_on_the_fresh_grid() {
        let mut c = VblankClock::from_mode(74_250, 1650, 750, 30).unwrap();
        assert_eq!(c.vblank_after(5), None);
        c.on_fresh_flip(1_000_000_000);
        let commit = 1_000_000_000 + 16_666_666 + 1_500_000;
        assert_eq!(c.vblank_after(commit), Some(1_000_000_000 + 2 * 16_666_666));
    }

    #[test]
    fn intervals_only_between_consecutive_fresh_frames() {
        let mut t = IntervalTracker::default();
        assert_eq!(t.on_flip(1, Some(100)), None);
        assert_eq!(t.on_flip(2, Some(133)), Some(33));
        assert_eq!(t.on_flip(3, None), None, "stale stamp");
        assert_eq!(t.on_flip(4, Some(233)), None, "spans the stale frame");
        assert_eq!(t.on_flip(5, Some(266)), Some(33));
    }

    #[test]
    fn samples_percentiles() {
        let mut s = Samples::with_capacity(3);
        for x in [5, 1, 9, 100] {
            s.push(x);
        }
        assert_eq!((s.count, s.max), (4, 100));
        assert_eq!(s.pct(50), Some(5));
        assert_eq!(s.pct(99), Some(9), "only the stored samples rank");
        assert_eq!(Samples::with_capacity(1).pct(99), None);
    }
}

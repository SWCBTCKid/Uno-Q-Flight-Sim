//! Identity-based DRM page-flip attribution (ISS-031).
//!
//! msm can report a newly completed flip with the preceding vblank's sequence
//! and timestamp. Kernel stamps therefore remain timing diagnostics, while a
//! unique token carried in atomic-commit user data is the presentation
//! identity. Only an accepted event may retire a scanout buffer.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PendingPresent {
    pub token: u64,
    pub framebuffer_id: u32,
    pub buffer_id: u64,
    pub snapshot_sequence: u64,
    pub submit_ns: u64,
    pub commit_flags: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventStatus {
    Presented,
    PresentedWithStaleVblankStamp,
    WrongCrtc,
    NoPendingCommit,
    DuplicateCommitEvent,
    UnknownCommit,
    SequenceRegressed,
    TimestampRegressed,
    StampInconsistent,
    TimestampBeforeCommit,
    RepeatedStaleStamp,
}

impl EventStatus {
    pub fn presented(self) -> bool {
        matches!(self, Self::Presented | Self::PresentedWithStaleVblankStamp)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PresentCompletion {
    pub status: EventStatus,
    pub pending: Option<PendingPresent>,
    /// Zero when no trustworthy interval exists (first or stale-stamped flip).
    pub interval_ns: u64,
}

#[derive(Debug)]
pub struct PresentTracker {
    crtc_id: u32,
    pending: Option<PendingPresent>,
    last_submitted_token: u64,
    last_fresh_stamp: Option<(u32, u64)>,
    last_was_stale: bool,
    presented: u64,
    stale_stamps: u64,
    errors: u64,
}

impl PresentTracker {
    pub fn new(crtc_id: u32) -> Self {
        Self {
            crtc_id,
            pending: None,
            last_submitted_token: 0,
            last_fresh_stamp: None,
            last_was_stale: false,
            presented: 0,
            stale_stamps: 0,
            errors: 0,
        }
    }

    pub fn pending(&self) -> Option<PendingPresent> { self.pending }
    pub fn presented_count(&self) -> u64 { self.presented }
    pub fn stale_stamp_count(&self) -> u64 { self.stale_stamps }
    pub fn error_count(&self) -> u64 { self.errors }

    /// Reserves one never-reused commit token before the atomic ioctl.
    pub fn submit(&mut self, present: PendingPresent) -> Result<(), &'static str> {
        if self.pending.is_some() { return Err("a presentation is already pending"); }
        if present.token == 0 || present.token <= self.last_submitted_token {
            return Err("presentation token must be nonzero and strictly increasing");
        }
        self.last_submitted_token = present.token;
        self.pending = Some(present);
        Ok(())
    }

    /// Removes exactly the reservation whose atomic commit failed. The token
    /// remains consumed and can never be reused.
    pub fn cancel_failed_commit(&mut self, token: u64) -> bool {
        if self.pending.is_some_and(|p| p.token == token) {
            self.pending = None;
            true
        } else {
            false
        }
    }

    /// After [`Self::on_event`] rejected an event *that carried the pending
    /// commit's own token* (a stamp anomaly, not an identity error): the
    /// kernel did complete that commit, so its buffer is on screen. Retires
    /// it without trusting the stamp: no interval, the fresh-stamp history is
    /// left as it was, and the event stays counted as an error. Returns
    /// `None` (and changes nothing) for any other token.
    pub fn retire_rejected(&mut self, token: u64) -> Option<PendingPresent> {
        if self.pending.is_some_and(|p| p.token == token) {
            self.presented += 1;
            self.last_was_stale = false;
            self.pending.take()
        } else {
            None
        }
    }

    pub fn on_event(
        &mut self,
        crtc_id: u32,
        token: u64,
        sequence: u32,
        event_ns: u64,
    ) -> PresentCompletion {
        let pending = self.pending;
        let mut status = EventStatus::Presented;

        if crtc_id != self.crtc_id {
            status = EventStatus::WrongCrtc;
        } else if token != 0
            && token <= self.last_submitted_token
            && !pending.is_some_and(|p| p.token == token)
        {
            status = EventStatus::DuplicateCommitEvent;
        } else if pending.is_none() {
            status = EventStatus::NoPendingCommit;
        } else if pending.is_some_and(|p| p.token != token) {
            status = EventStatus::UnknownCommit;
        } else if let Some((last_sequence, last_event_ns)) = self.last_fresh_stamp {
            let sequence_delta = sequence.wrapping_sub(last_sequence) as i32;
            let same_sequence = sequence_delta == 0;
            let same_time = event_ns == last_event_ns;
            status = if same_sequence && same_time {
                if self.last_was_stale {
                    EventStatus::RepeatedStaleStamp
                } else {
                    EventStatus::PresentedWithStaleVblankStamp
                }
            } else if same_sequence || same_time {
                EventStatus::StampInconsistent
            } else if sequence_delta < 0 {
                EventStatus::SequenceRegressed
            } else if event_ns < last_event_ns {
                EventStatus::TimestampRegressed
            } else {
                EventStatus::Presented
            };
        }

        if status == EventStatus::Presented
            && pending.is_some_and(|p| event_ns <= p.submit_ns)
        {
            status = EventStatus::TimestampBeforeCommit;
        }

        if !status.presented() {
            self.errors += 1;
            return PresentCompletion { status, pending: None, interval_ns: 0 };
        }

        let accepted = self.pending.take();
        self.presented += 1;
        self.last_was_stale = status == EventStatus::PresentedWithStaleVblankStamp;
        if self.last_was_stale {
            self.stale_stamps += 1;
            return PresentCompletion { status, pending: accepted, interval_ns: 0 };
        }

        let interval_ns = self
            .last_fresh_stamp
            .map_or(0, |(_, last_event_ns)| event_ns - last_event_ns);
        self.last_fresh_stamp = Some((sequence, event_ns));
        PresentCompletion { status, pending: accepted, interval_ns }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pending(token: u64, submit_ns: u64) -> PendingPresent {
        PendingPresent {
            token,
            framebuffer_id: token as u32 + 40,
            buffer_id: token + 100,
            snapshot_sequence: token * 4,
            submit_ns,
            commit_flags: 3,
        }
    }

    #[test]
    fn unique_tokens_complete_their_exact_buffers() {
        let mut tracker = PresentTracker::new(47);
        tracker.submit(pending(1, 100)).unwrap();
        let first = tracker.on_event(47, 1, 10, 120);
        assert_eq!(first.status, EventStatus::Presented);
        assert_eq!(first.pending.unwrap().buffer_id, 101);

        tracker.submit(pending(2, 125)).unwrap();
        let second = tracker.on_event(47, 2, 11, 140);
        assert_eq!(second.interval_ns, 20);
        assert_eq!(second.pending.unwrap().framebuffer_id, 42);
    }

    #[test]
    fn one_stale_stamp_is_accepted_but_two_are_rejected() {
        let mut tracker = PresentTracker::new(47);
        tracker.submit(pending(1, 100)).unwrap();
        tracker.on_event(47, 1, 10, 120);
        tracker.submit(pending(2, 121)).unwrap();
        let stale = tracker.on_event(47, 2, 10, 120);
        assert_eq!(stale.status, EventStatus::PresentedWithStaleVblankStamp);
        assert_eq!(tracker.stale_stamp_count(), 1);

        tracker.submit(pending(3, 122)).unwrap();
        let repeated = tracker.on_event(47, 3, 10, 120);
        assert_eq!(repeated.status, EventStatus::RepeatedStaleStamp);
        assert!(tracker.pending().is_some(), "bad event cannot retire the pending buffer");
    }

    #[test]
    fn duplicate_and_unknown_events_never_retire_pending() {
        let mut tracker = PresentTracker::new(47);
        tracker.submit(pending(1, 100)).unwrap();
        tracker.on_event(47, 1, 10, 120);
        tracker.submit(pending(2, 125)).unwrap();

        assert_eq!(tracker.on_event(47, 1, 11, 140).status, EventStatus::DuplicateCommitEvent);
        assert_eq!(tracker.on_event(47, 9, 11, 140).status, EventStatus::UnknownCommit);
        assert_eq!(tracker.pending().unwrap().token, 2);
    }

    #[test]
    fn rejected_stamp_with_own_token_can_retire_without_an_interval() {
        let mut tracker = PresentTracker::new(47);
        tracker.submit(pending(1, 100)).unwrap();
        tracker.on_event(47, 1, 10, 120);
        tracker.submit(pending(2, 121)).unwrap();
        tracker.on_event(47, 2, 10, 120); // stale, accepted
        tracker.submit(pending(3, 122)).unwrap();
        assert_eq!(tracker.on_event(47, 3, 10, 120).status, EventStatus::RepeatedStaleStamp);
        assert!(tracker.retire_rejected(9).is_none(), "other tokens never retire");
        assert_eq!(tracker.retire_rejected(3).unwrap().token, 3);
        assert_eq!((tracker.presented_count(), tracker.error_count()), (3, 1));
        // The next fresh stamp is measured from the last fresh one (frame 1).
        tracker.submit(pending(4, 140)).unwrap();
        let next = tracker.on_event(47, 4, 14, 186);
        assert_eq!((next.status, next.interval_ns), (EventStatus::Presented, 66));
    }

    #[test]
    fn failed_commit_consumes_token_and_only_cancels_exact_pending() {
        let mut tracker = PresentTracker::new(47);
        tracker.submit(pending(1, 100)).unwrap();
        assert!(!tracker.cancel_failed_commit(2));
        assert!(tracker.cancel_failed_commit(1));
        assert!(tracker.submit(pending(1, 110)).is_err());
        tracker.submit(pending(2, 110)).unwrap();
    }
}

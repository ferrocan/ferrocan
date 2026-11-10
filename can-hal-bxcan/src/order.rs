// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Transmissions outstanding, reported in the order they were handed over
//! whatever order they finish in, as can-hal's `ConfirmedTransmit` requires.
//! Also the event queue.

use can_hal::{Abort, Failure, Outcome, TransmitId};

/// bxCAN's transmit mailboxes, and so the transmissions outstanding at once.
pub(crate) const MAILBOXES: usize = 3;

struct Entry<T> {
    id: TransmitId,
    mailbox: u8,
    abort_requested: bool,
    outcome: Option<Outcome<T>>,
}

/// The transmissions handed over and not yet reported, oldest first.
pub(crate) struct Outstanding<T> {
    entries: [Option<Entry<T>>; MAILBOXES],
    head: usize,
    len: usize,
}

impl<T> Outstanding<T> {
    pub(crate) const fn new() -> Self {
        Self {
            entries: [const { None }; MAILBOXES],
            head: 0,
            len: 0,
        }
    }

    pub(crate) const fn len(&self) -> usize {
        self.len
    }

    pub(crate) const fn is_full(&self) -> bool {
        self.len == MAILBOXES
    }

    /// Records transmission `id`, just placed in `mailbox`.
    pub(crate) fn push(&mut self, id: TransmitId, mailbox: u8) {
        assert!(!self.is_full(), "no room for another transmission");
        self.entries[(self.head + self.len) % MAILBOXES] = Some(Entry {
            id,
            mailbox,
            abort_requested: false,
            outcome: None,
        });
        self.len += 1;
    }

    fn iter_mut(&mut self) -> impl Iterator<Item = &mut Entry<T>> {
        let (head, len) = (self.head, self.len);
        let (front, back) = self.entries.split_at_mut(head);
        back.iter_mut().chain(front.iter_mut()).take(len).filter_map(Option::as_mut)
    }

    /// Notes that transmission `id` is to be aborted, returning the mailbox
    /// to abort if it is still unfinished.
    pub(crate) fn request_abort(&mut self, id: TransmitId) -> Option<u8> {
        let entry = self.iter_mut().find(|e| e.id == id && e.outcome.is_none())?;
        entry.abort_requested = true;
        Some(entry.mailbox)
    }

    /// Notes that every unfinished transmission is to be aborted, returning
    /// their mailboxes.
    pub(crate) fn abort_unfinished(&mut self) -> impl Iterator<Item = u8> + '_ {
        self.iter_mut().filter(|e| e.outcome.is_none()).map(|e| {
            e.abort_requested = true;
            e.mailbox
        })
    }

    /// The mailboxes of the transmissions still unfinished.
    pub(crate) fn unfinished(&mut self) -> impl Iterator<Item = u8> + '_ {
        self.iter_mut().filter(|e| e.outcome.is_none()).map(|e| e.mailbox)
    }

    /// The transmission in `mailbox` has finished: `sent` with its time if it
    /// was transmitted. If not, it was aborted when that was asked for, or else
    /// the controller gave up on it (single-shot, or going bus-off).
    pub(crate) fn finish(&mut self, mailbox: u8, sent: Option<T>) {
        // A mailbox is reused once it has finished, so it's the oldest
        // unfinished transmission in it that finished now.
        let Some(entry) = self.iter_mut().find(|e| e.mailbox == mailbox && e.outcome.is_none()) else {
            return;
        };
        entry.outcome = Some(match sent {
            Some(time) => Outcome::Sent(time),
            None if entry.abort_requested => Outcome::Aborted(Abort::Confirmed),
            None => Outcome::Failed(Failure::GaveUp),
        });
    }

    /// The report of the oldest transmission, if it has finished.
    pub(crate) fn pop_report(&mut self) -> Option<(TransmitId, Outcome<T>)> {
        if self.len == 0 {
            return None;
        }
        let slot = &mut self.entries[self.head];
        if slot.as_ref().is_none_or(|e| e.outcome.is_none()) {
            return None;
        }
        let entry = slot.take()?;
        self.head = (self.head + 1) % MAILBOXES;
        self.len -= 1;
        Some((entry.id, entry.outcome?))
    }
}

/// A bounded first-in, first-out queue.
pub(crate) struct Queue<E, const N: usize> {
    items: [Option<E>; N],
    head: usize,
    len: usize,
}

impl<E, const N: usize> Queue<E, N> {
    pub(crate) const fn new() -> Self {
        Self { items: [const { None }; N], head: 0, len: 0 }
    }

    pub(crate) const fn free(&self) -> usize {
        N - self.len
    }

    /// Adds `item` at the back, or hands it back if the queue is full.
    pub(crate) fn push(&mut self, item: E) -> Result<(), E> {
        if self.len == N {
            return Err(item);
        }
        self.items[(self.head + self.len) % N] = Some(item);
        self.len += 1;
        Ok(())
    }

    pub(crate) fn pop(&mut self) -> Option<E> {
        if self.len == 0 {
            return None;
        }
        let item = self.items[self.head].take();
        self.head = (self.head + 1) % N;
        self.len -= 1;
        item
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn reports(outstanding: &mut Outstanding<u32>) -> Vec<(u32, Outcome<u32>)> {
        core::iter::from_fn(|| outstanding.pop_report()).map(|(id, outcome)| (id.0, outcome)).collect()
    }

    #[test]
    fn reports_in_hand_over_order() {
        let mut o = Outstanding::new();
        o.push(TransmitId(1), 0);
        o.push(TransmitId(2), 1);
        o.push(TransmitId(3), 2);
        assert!(o.is_full());
        // B aborted while A is on its way: B's report waits for A's.
        assert_eq!(o.request_abort(TransmitId(2)), Some(1));
        o.finish(1, None);
        assert_eq!(reports(&mut o), []);
        o.finish(0, Some(100));
        o.finish(2, Some(200));
        assert_eq!(reports(&mut o), [(1, Outcome::Sent(100)), (2, Outcome::Aborted(Abort::Confirmed)), (3, Outcome::Sent(200))]);
        assert_eq!(o.len(), 0);
    }

    #[test]
    fn a_reused_mailbox_finishes_its_oldest_transmission() {
        let mut o = Outstanding::new();
        o.push(TransmitId(1), 0);
        o.push(TransmitId(2), 1);
        o.finish(1, None); // B gave up while A waits
        o.push(TransmitId(3), 1); // and C took B's mailbox
        o.finish(1, Some(30));
        o.finish(0, Some(10));
        assert_eq!(reports(&mut o), [(1, Outcome::Sent(10)), (2, Outcome::Failed(Failure::GaveUp)), (3, Outcome::Sent(30))]);
    }

    #[test]
    fn an_abort_too_late_is_reported_sent() {
        let mut o = Outstanding::new();
        o.push(TransmitId(7), 2);
        assert_eq!(o.request_abort(TransmitId(7)), Some(2));
        o.finish(2, Some(5));
        assert_eq!(reports(&mut o), [(7, Outcome::Sent(5))]);
        // Unknown and finished ids are ignored.
        assert_eq!(o.request_abort(TransmitId(7)), None);
        assert_eq!(o.request_abort(TransmitId(8)), None);
    }

    #[test]
    fn wraps_around() {
        let mut o = Outstanding::new();
        for id in 0..10 {
            o.push(TransmitId(id), (id % 3) as u8);
            o.finish((id % 3) as u8, Some(id));
            assert_eq!(reports(&mut o), [(id, Outcome::Sent(id))]);
        }
        let mut unfinished = Outstanding::<u32>::new();
        unfinished.push(TransmitId(0), 0);
        unfinished.push(TransmitId(1), 1);
        unfinished.finish(0, Some(0));
        assert_eq!(unfinished.unfinished().collect::<Vec<_>>(), [1]);
    }

    #[test]
    fn queue_is_fifo_and_bounded() {
        let mut q = Queue::<u8, 2>::new();
        assert_eq!(q.free(), 2);
        assert_eq!(q.push(1), Ok(()));
        assert_eq!(q.push(2), Ok(()));
        assert_eq!(q.push(3), Err(3));
        assert_eq!(q.pop(), Some(1));
        assert_eq!(q.push(4), Ok(()));
        assert_eq!((q.pop(), q.pop(), q.pop()), (Some(2), Some(4), None));
    }
}

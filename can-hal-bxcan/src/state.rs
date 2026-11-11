// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! What the channel and the interrupt handler share: the event queue, the
//! outstanding transmissions and the counts, behind a critical section.

use core::cell::RefCell;
use core::task::Waker;

use can_hal::{BusState, CanFrame, Event, Timestamped, TransmitId};
use critical_section::Mutex;

use crate::frame::{self, Words};
use crate::order::{MAILBOXES, Outstanding, Queue};
use crate::registers::{ESR_BOFF, ESR_EPVF, ESR_LEC_SHIFT, LEC_SOFTWARE, MSR_ERRI, RFR_FMP, RFR_FOVR, RFR_RFOM, RegisterBlock, Regs, tsr_abrq, tsr_rqcp, tsr_txok};
use crate::{BusError, Clock};

/// The state a [`Can`](crate::Can) channel shares with its interrupt handler,
/// with room for `N` events. Declare one per controller as a `static`, give it
/// to [`Can::new`](crate::Can::new), and call [`on_interrupt`](Self::on_interrupt)
/// from the controller's interrupt handlers.
pub struct State<C: Clock, const N: usize> {
    inner: Mutex<RefCell<Inner<C::Instant, N>>>,
}

pub(crate) struct Inner<T, const N: usize> {
    pub(crate) regs: Option<Regs>,
    pub(crate) events: Queue<Event<CanFrame, T>, N>,
    pub(crate) outstanding: Outstanding<T>,
    pub(crate) next_id: u32,
    pub(crate) bus_state: BusState,
    pub(crate) waker: Option<Waker>,
    pub(crate) counts: Counts,
    /// Whether bus errors are reported, and the latest not yet taken.
    pub(crate) report_bus_errors: bool,
    pub(crate) bus_error: Option<BusError>,
    pub(crate) error_waker: Option<Waker>,
}

/// Running counts for [`Statistics`](can_hal::Statistics).
#[derive(Default, Clone, Copy)]
pub(crate) struct Counts {
    pub(crate) rx_frames: u32,
    pub(crate) tx_frames: u32,
    pub(crate) rx_dropped: u32,
    pub(crate) rx_overruns: u32,
    pub(crate) tx_failed: u32,
    pub(crate) bus_errors: u32,
}

/// Something that happened, with the bit-time stamp of its start of frame.
#[derive(Clone)]
enum Happening {
    Finished { mailbox: u8, sent: bool },
    Received(CanFrame),
}

/// At most three transmissions and three frames in each receive FIFO.
const MAX_HAPPENINGS: usize = MAILBOXES + 6;

impl<C: Clock, const N: usize> State<C, N> {
    /// A state with no controller yet.
    ///
    /// # Panics
    ///
    /// When `N` is 3 or less, too few for the transmission reports: at compile
    /// time when the state is a `static`.
    #[must_use]
    pub const fn new() -> Self {
        assert!(N > MAILBOXES, "the queue needs room for more events than there are mailboxes");
        Self {
            inner: Mutex::new(RefCell::new(Inner {
                regs: None,
                events: Queue::new(),
                outstanding: Outstanding::new(),
                next_id: 0,
                bus_state: BusState::ErrorActive,
                waker: None,
                report_bus_errors: false,
                bus_error: None,
                error_waker: None,
                counts: Counts {
                    rx_frames: 0,
                    tx_frames: 0,
                    rx_dropped: 0,
                    rx_overruns: 0,
                    tx_failed: 0,
                    bus_errors: 0,
                },
            })),
        }
    }

    pub(crate) fn with<R>(&self, f: impl FnOnce(&mut Inner<C::Instant, N>) -> R) -> R {
        critical_section::with(|cs| f(&mut self.inner.borrow_ref_mut(cs)))
    }

    /// Handles the controller's interrupts: transmissions finished, frames
    /// received, receive overruns and error state changes. Call it from every
    /// interrupt handler of the controller (on the STM32F0 all four share one
    /// vector).
    pub fn on_interrupt(&self) {
        self.with(|inner| {
            let Some(regs) = inner.regs else { return };
            let now = C::now();
            inner.handle(regs.get(), &now);
            if let Some(waker) = inner.waker.take() {
                waker.wake();
            }
        });
    }
}

impl<C: Clock, const N: usize> Default for State<C, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Clone, const N: usize> Inner<T, N> {
    fn handle(&mut self, r: &RegisterBlock, now: &T) {
        let mut happened: [Option<(u16, Happening)>; MAX_HAPPENINGS] = [const { None }; MAX_HAPPENINGS];
        let mut count = 0;

        // Finished transmissions. Clearing RQCP clears TXOK with it, so TSR is
        // read first.
        let tsr = r.tsr.read();
        for (mailbox, tx) in r.tx.iter().enumerate() {
            if tsr & tsr_rqcp(mailbox) != 0 {
                r.tsr.write(tsr_rqcp(mailbox));
                let time = (tx.tdtr.read() >> 16) as u16;
                #[allow(clippy::cast_possible_truncation)] // three mailboxes
                let finished = Happening::Finished {
                    mailbox: mailbox as u8,
                    sent: tsr & tsr_txok(mailbox) != 0,
                };
                happened[count] = Some((time, finished));
                count += 1;
            }
        }

        // Received frames, three at most in each FIFO.
        for (fifo, rx) in r.rx.iter().enumerate() {
            for _ in 0..3 {
                let rfr = r.rfr[fifo].read();
                if rfr & RFR_FOVR != 0 {
                    r.rfr[fifo].write(RFR_FOVR);
                    self.counts.rx_overruns += 1;
                }
                if rfr & RFR_FMP == 0 {
                    break;
                }
                let words = Words {
                    ir: rx.rir.read(),
                    dtr: rx.rdtr.read(),
                    dlr: rx.rdlr.read(),
                    dhr: rx.rdhr.read(),
                };
                r.rfr[fifo].write(RFR_RFOM);
                happened[count] = Some((frame::time(&words), Happening::Received(frame::decode(words))));
                count += 1;
            }
        }

        // Into the order they had on the bus, by start of frame. The stamps
        // are 16 bits of bit time, so ordering by difference holds while they
        // span less than 32768 bits (65 ms at 500 kbit/s).
        let happened = &mut happened[..count];
        if let Some(&Some((reference, _))) = happened.first() {
            #[allow(clippy::cast_possible_wrap)] // the difference, as a signed offset
            happened.sort_unstable_by_key(|h| h.as_ref().map(|&(time, _)| time.wrapping_sub(reference) as i16));
        }
        for (_, happening) in happened.iter_mut().filter_map(Option::take) {
            match happening {
                Happening::Finished { mailbox, sent } => {
                    if sent {
                        self.counts.tx_frames += 1;
                    }
                    self.outstanding.finish(mailbox, sent.then(|| now.clone()));
                    self.push_reports();
                }
                Happening::Received(frame) => self.push_received(frame, now),
            }
        }

        // Error state. ERRI is set when the controller became error passive or
        // went bus-off; there is no interrupt for recovering, so the state is
        // also checked as events are read.
        if r.msr.read() & MSR_ERRI != 0 {
            r.msr.write(MSR_ERRI);
        }
        // A bus error, when they are reported. The code is then marked as
        // set by software, so that the next error shows as new.
        if self.report_bus_errors
            && let Some(error) = BusError::from_lec((r.esr.read() >> ESR_LEC_SHIFT) & 0b111)
        {
            r.esr.write(LEC_SOFTWARE << ESR_LEC_SHIFT);
            self.bus_error = Some(error);
            self.counts.bus_errors += 1;
            if let Some(waker) = self.error_waker.take() {
                waker.wake();
            }
        }
        self.update_bus_state(r);
    }

    /// Queues the reports of the transmissions that are next in order.
    fn push_reports(&mut self) {
        while let Some((id, outcome)) = self.outstanding.pop_report() {
            if matches!(outcome, can_hal::Outcome::Failed(_)) {
                self.counts.tx_failed += 1;
            }
            // There is always room: received frames leave space for every
            // outstanding report and a change of bus state.
            let _ = self.events.push(Event::Transmitted { id, outcome });
        }
    }

    fn push_received(&mut self, frame: CanFrame, now: &T) {
        self.counts.rx_frames += 1;
        if self.events.free() > self.outstanding.len() + 1 {
            let _ = self.events.push(Event::Received(Timestamped::new(frame, now.clone())));
        } else {
            self.counts.rx_dropped += 1;
        }
    }

    /// Reads the error state, queuing a change. On going bus-off, the
    /// controller would hold its transmissions until it recovers; they are
    /// aborted instead, and so reported as given up.
    pub(crate) fn update_bus_state(&mut self, r: &RegisterBlock) {
        let state = bus_state(r.esr.read());
        if state == self.bus_state {
            return;
        }
        self.bus_state = state;
        if state == BusState::BusOff {
            for mailbox in self.outstanding.unfinished() {
                r.tsr.write(tsr_abrq(usize::from(mailbox)));
            }
        }
        let _ = self.events.push(Event::BusStateChanged(state));
    }

    pub(crate) fn take_id(&mut self) -> TransmitId {
        let id = TransmitId(self.next_id);
        self.next_id = self.next_id.wrapping_add(1);
        id
    }
}

/// The bus state ESR describes.
pub(crate) const fn bus_state(esr: u32) -> BusState {
    if esr & ESR_BOFF != 0 {
        BusState::BusOff
    } else if esr & ESR_EPVF != 0 {
        BusState::ErrorPassive
    } else {
        BusState::ErrorActive
    }
}

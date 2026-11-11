// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! What the channel and the interrupt handler share: the event queue, the
//! outstanding transmissions and the counts, behind a critical section.

use core::cell::RefCell;
use core::task::Waker;

use can_hal::{BusState, Event, Frame, Timestamped, TransmitId};
use critical_section::Mutex;

use crate::frame::{self, Element};
use crate::order::{MAILBOXES, Outstanding, Queue};
use crate::registers::{self, IR_RF0L, PSR_BO, PSR_DLEC_SHIFT, PSR_EP, PSR_LEC_SHIFT, RXFS_FFL_MASK, RXFS_FGI_SHIFT, RegisterBlock, Regs, TXEFS_EFFL_MASK, TXEFS_EFGI_SHIFT};
use crate::{BusError, Clock};

/// The state a [`Can`](crate::Can) channel shares with its interrupt handler,
/// with room for `N` events. Declare one per controller as a `static`, give it
/// to [`Can::new`](crate::Can::new), and call [`on_interrupt`](Self::on_interrupt)
/// from the controller's interrupt line 0 handler.
pub struct State<C: Clock, const N: usize> {
    inner: Mutex<RefCell<Inner<C::Instant, N>>>,
}

pub(crate) struct Inner<T, const N: usize> {
    pub(crate) regs: Option<Regs>,
    pub(crate) events: Queue<Event<Frame, T>, N>,
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
enum Happening {
    Sent { mailbox: u8 },
    Received(Frame),
}

impl Happening {
    /// Sent before received when both started together: a frame received in
    /// loopback is reported after its own transmission.
    const fn rank(&self) -> u8 {
        match self {
            Self::Sent { .. } => 0,
            Self::Received(_) => 1,
        }
    }
}

/// At most three Tx events and three frames in receive FIFO 0.
const MAX_HAPPENINGS: usize = MAILBOXES + 3;

impl<C: Clock, const N: usize> State<C, N> {
    /// A state with no controller yet.
    ///
    /// # Panics
    ///
    /// When `N` is 3 or less, too few for the transmission reports: at compile
    /// time when the state is a `static`.
    #[must_use]
    pub const fn new() -> Self {
        assert!(N > MAILBOXES, "the queue needs room for more events than there are transmit buffers");
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
    /// received, receive overruns, bus errors and error state changes. Call
    /// it from the handler of the controller's interrupt line 0 (on the
    /// STM32G4, `FDCAN1_IT0`), where the channel routes them all.
    pub fn on_interrupt(&self) {
        self.with(|inner| {
            let Some(regs) = inner.regs else { return };
            inner.handle(regs, &C::now());
        });
    }
}

impl<C: Clock, const N: usize> Default for State<C, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Clone, const N: usize> Inner<T, N> {
    /// Takes in everything the controller has finished: Tx events, received
    /// frames and transmissions cancelled or given up, then the error state.
    /// Called from the interrupt handler, and before a transmit buffer is
    /// reused, so that a buffer never holds two unfinished transmissions.
    pub(crate) fn handle(&mut self, regs: Regs, now: &T) {
        let r = regs.get();
        let ram = regs.ram();
        // Clear the flags first: anything that happens from here on raises
        // them again.
        let ir = r.ir.read();
        r.ir.write(ir);
        if ir & IR_RF0L != 0 {
            self.counts.rx_overruns += 1;
        }

        // The transmit buffers' state, before the Tx events are read: a
        // frame sent by now has its event in the FIFO.
        let pending = r.txbrp.read();
        let cancelled = r.txbcf.read();
        let sent = r.txbto.read();

        let mut happened: [Option<(u16, Happening)>; MAX_HAPPENINGS] = [const { None }; MAX_HAPPENINGS];
        let mut count = 0;

        // Transmissions sent, from the Tx event FIFO: the message marker is
        // the buffer the frame was in.
        for _ in 0..MAILBOXES {
            let txefs = r.txefs.read();
            if txefs & TXEFS_EFFL_MASK == 0 {
                break;
            }
            let index = (txefs >> TXEFS_EFGI_SHIFT) & 0b11;
            let e1 = ram.tx_events[index as usize][1].read();
            r.txefa.write(index);
            let mailbox = (e1 >> registers::H1_MM_SHIFT) as u8;
            happened[count] = Some((frame::time(e1), Happening::Sent { mailbox }));
            count += 1;
        }

        // Received frames, from FIFO 0.
        let fifo = &r.rxf[0];
        for _ in 0..3 {
            let rxfs = fifo.status.read();
            if rxfs & RXFS_FFL_MASK == 0 {
                break;
            }
            let index = (rxfs >> RXFS_FGI_SHIFT) & 0b11;
            let element = read_element(&ram.rx_fifo[0][index as usize]);
            fifo.ack.write(index);
            happened[count] = Some((frame::time(element.h1), Happening::Received(frame::decode(&element))));
            count += 1;
        }

        // Into the order they had on the bus, by start of frame. The stamps
        // are 16 bits of bit time, so ordering by difference holds while they
        // span less than 32768 bits (65 ms at 500 kbit/s).
        let happened = &mut happened[..count];
        if let Some(&Some((reference, _))) = happened.first() {
            #[expect(clippy::cast_possible_wrap, reason = "the difference, as a signed offset")]
            happened.sort_unstable_by_key(|h| h.as_ref().map(|(time, happening)| (time.wrapping_sub(reference) as i16, happening.rank())));
        }
        for (_, happening) in happened.iter_mut().filter_map(Option::take) {
            match happening {
                Happening::Sent { mailbox } => {
                    self.counts.tx_frames += 1;
                    self.outstanding.finish(mailbox, Some(now.clone()));
                }
                Happening::Received(frame) => self.push_received(frame, now),
            }
        }

        // Transmissions that ended without being sent: cancelled, or, without
        // automatic retransmission, failed. Their buffer was no longer
        // pending, its cancellation had finished and it hadn't been sent (a
        // frame asked to be cancelled may be sent all the same, when both
        // flags are set).
        for mailbox in self.unfinished_mailboxes() {
            let bit = 1 << mailbox;
            if pending & bit == 0 && sent & bit == 0 && cancelled & bit != 0 {
                self.outstanding.finish(mailbox, None);
            }
        }
        self.push_reports();

        let _ = self.update_bus_state(r);
        if let Some(waker) = self.waker.take() {
            waker.wake();
        }
    }

    fn unfinished_mailboxes(&mut self) -> impl Iterator<Item = u8> + use<T, N> {
        let mut mailboxes = [None; MAILBOXES];
        for (slot, mailbox) in mailboxes.iter_mut().zip(self.outstanding.unfinished()) {
            *slot = Some(mailbox);
        }
        mailboxes.into_iter().flatten()
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

    fn push_received(&mut self, frame: Frame, now: &T) {
        self.counts.rx_frames += 1;
        if self.events.free() > self.outstanding.len() + 1 {
            let _ = self.events.push(Event::Received(Timestamped::new(frame, now.clone())));
        } else {
            self.counts.rx_dropped += 1;
        }
    }

    /// Reads the protocol status, queuing a change of bus state and noting
    /// a bus error. Every read of PSR goes through here: reading it resets
    /// its error codes, which would otherwise be lost.
    ///
    /// On going bus-off the controller enters initialisation and holds its
    /// transmissions until it recovers; they are cancelled instead, and so
    /// reported as given up. Returns what PSR read.
    pub(crate) fn update_bus_state(&mut self, r: &RegisterBlock) -> u32 {
        let psr = r.psr.read();
        if self.report_bus_errors {
            // The nominal phase's error, else the data phase's.
            let error = BusError::from_lec((psr >> PSR_LEC_SHIFT) & 0b111).or_else(|| BusError::from_lec((psr >> PSR_DLEC_SHIFT) & 0b111));
            if let Some(error) = error {
                self.bus_error = Some(error);
                self.counts.bus_errors += 1;
                if let Some(waker) = self.error_waker.take() {
                    waker.wake();
                }
            }
        }
        let state = bus_state(psr);
        if state == self.bus_state {
            return psr;
        }
        self.bus_state = state;
        if state == BusState::BusOff {
            for mailbox in self.unfinished_mailboxes() {
                r.txbcr.write(1 << mailbox);
            }
        }
        let _ = self.events.push(Event::BusStateChanged(state));
        psr
    }

    pub(crate) fn take_id(&mut self) -> TransmitId {
        let id = TransmitId(self.next_id);
        self.next_id = self.next_id.wrapping_add(1);
        id
    }
}

/// An element's header and the data its DLC calls for.
fn read_element(element: &registers::Element) -> Element {
    let h0 = element.header[0].read();
    let h1 = element.header[1].read();
    let mut data = [0u8; 64];
    let words = frame::data_len(h1).div_ceil(4);
    for (chunk, word) in data.as_chunks_mut::<4>().0.iter_mut().zip(&element.data).take(words) {
        *chunk = word.read().to_le_bytes();
    }
    Element { h0, h1, data }
}

/// Writes `contents` to a transmit buffer: its header, then the data its DLC
/// calls for.
pub(crate) fn write_element(element: &registers::Element, contents: &Element) {
    element.header[0].write(contents.h0);
    element.header[1].write(contents.h1);
    let words = frame::data_len(contents.h1).div_ceil(4);
    for (chunk, word) in contents.data.as_chunks::<4>().0.iter().zip(&element.data).take(words) {
        word.write(u32::from_le_bytes(*chunk));
    }
}

/// The bus state PSR describes.
pub(crate) const fn bus_state(psr: u32) -> BusState {
    if psr & PSR_BO != 0 {
        BusState::BusOff
    } else if psr & PSR_EP != 0 {
        BusState::ErrorPassive
    } else {
        BusState::ErrorActive
    }
}

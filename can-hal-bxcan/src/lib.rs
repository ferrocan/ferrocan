// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A [can-hal](can_hal) backend for bxCAN, the classic CAN controller of many
//! STM32 microcontrollers (F0, F1, F3, F4, F7, L4 and others) and of chips
//! compatible with them.
//!
//! The channel confirms its transmissions: it implements can-hal's
//! [`ConfirmedTransmit`] and [`ReceiveEvents`] for [`CanFrame`], reporting each
//! frame [`Sent`](can_hal::Outcome::Sent) once the controller says it was
//! transmitted and acknowledged (bxCAN's TXOK), and interleaving those reports
//! with the frames received in the order they started on the bus. It also
//! implements [`BusStatus`], [`Recover`] and [`Statistics`].
//!
//! The crate drives the controller's registers itself and depends on no HAL:
//! the application enables the controller's clock, sets up its pins, and
//! calls [`State::on_interrupt`] from its interrupt handlers. Frames keep
//! their DLC, 9 to 15 included, and received remote frames carry no data.
//!
//! ```ignore
//! static STATE: State<MyClock, 32> = State::new();
//!
//! // The controller's clock is on, its pins are set to CAN.
//! let peripheral = unsafe { Peripheral::new(0x4000_6400) };
//! let timing = BitTiming::solve(48_000_000, 500_000, SamplePoint::from_per_mille(875))?;
//! let mut can = Can::new(peripheral, &STATE, Config::new(timing))?;
//! // Unmask the controller's interrupt, whose handler calls STATE.on_interrupt().
//! can.start();
//! ```

#![no_std]

#[cfg(test)]
extern crate std;

mod frame;
mod order;
mod registers;
mod state;
mod timing;

use core::fmt;
use core::future::poll_fn;
use core::task::Poll;
use core::time::Duration;

use can_hal::{AbortError, BusState, BusStatistics, BusStatus, CanFrame, ConfirmedTransmit, ErrorCounters, Event, ReceiveEvents, Recover, Statistics, TransmitId, TryTransmitError};

pub use state::State;
pub use timing::BitTiming;

use order::MAILBOXES;
use registers::{
    BTR_LBKM, BTR_SILM, FMR_FINIT, IER_BOFIE, IER_EPVIE, IER_ERRIE, IER_FMPIE0, IER_FMPIE1, IER_FOVIE0, IER_FOVIE1, IER_LECIE, IER_TMEIE, IR_TXRQ, MCR_INRQ, MCR_NART, MCR_SLEEP, MCR_TTCM, MCR_TXFP, MSR_INAK, MSR_SLAK, RegisterBlock, Regs, tsr_abrq,
    tsr_tme,
};

/// A source of time for timestamps and timeouts.
///
/// Its [`now`](Self::now) is read in the interrupt handler, to stamp the
/// frames received and the transmissions sent.
pub trait Clock {
    /// A point in time.
    type Instant: Clone + Send + 'static;
    /// The time now.
    fn now() -> Self::Instant;
    /// The time `timeout` from now.
    fn deadline(timeout: Duration) -> Self::Instant;
    /// Whether `deadline` has passed.
    fn has_passed(deadline: &Self::Instant) -> bool;
}

/// A bxCAN controller's registers, not yet configured.
pub struct Peripheral(Regs);

impl Peripheral {
    /// The controller whose register block is at `address`: on the STM32F0,
    /// `0x4000_6400`.
    ///
    /// # Safety
    ///
    /// `address` must be a bxCAN controller's, and nothing else may use that
    /// controller while this, or the [`Can`] made from it, exists.
    #[must_use]
    pub const unsafe fn new(address: usize) -> Self {
        // SAFETY: passed on from the caller.
        Self(unsafe { Regs::new(address) })
    }
}

/// What the controller does with the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Takes part in the bus.
    #[default]
    Normal,
    /// Listens and never transmits or acknowledges: transmit requests wait.
    Silent,
    /// Receives its own frames and drives them on the bus too, but ignores
    /// what the bus carries.
    Loopback,
    /// Receives its own frames and neither drives the bus nor listens to it:
    /// a self-test, safe whatever the pins are wired to.
    SilentLoopback,
}

/// How to set up the controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The nominal bit timing.
    pub timing: BitTiming,
    /// The mode.
    pub mode: Mode,
    /// Whether the controller retries a frame until it is sent (the CAN
    /// default), or tries once and gives up (single-shot).
    pub retransmit: bool,
    /// Whether to report each bus error, through [`Can::next_bus_error`].
    /// Off by default: on a faulty bus there can be thousands a second.
    pub bus_errors: bool,
}

impl Config {
    /// A normal channel with the given timing, retrying frames until sent.
    #[must_use]
    pub const fn new(timing: BitTiming) -> Self {
        Self {
            timing,
            mode: Mode::Normal,
            retransmit: true,
            bus_errors: false,
        }
    }

    /// The same, in `mode`.
    #[must_use]
    pub const fn with_mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }

    /// The same, retrying frames or not.
    #[must_use]
    pub const fn with_retransmit(mut self, retransmit: bool) -> Self {
        self.retransmit = retransmit;
        self
    }

    /// The same, reporting bus errors or not.
    #[must_use]
    pub const fn with_bus_errors(mut self, bus_errors: bool) -> Self {
        self.bus_errors = bus_errors;
        self
    }
}

/// A bus error the controller saw, from its last error code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum BusError {
    /// More than five equal bits in a row.
    Stuff,
    /// A fixed-form field held an illegal bit.
    Form,
    /// The frame it sent wasn't acknowledged.
    Acknowledgement,
    /// It sent a recessive bit and read back a dominant one.
    BitRecessive,
    /// It sent a dominant bit and read back a recessive one.
    BitDominant,
    /// A received frame's CRC didn't match.
    Crc,
}

impl BusError {
    /// The error a last error code (ESR's LEC) stands for, if any.
    pub(crate) const fn from_lec(lec: u32) -> Option<Self> {
        Some(match lec {
            1 => Self::Stuff,
            2 => Self::Form,
            3 => Self::Acknowledgement,
            4 => Self::BitRecessive,
            5 => Self::BitDominant,
            6 => Self::Crc,
            _ => return None,
        })
    }
}

/// The errors of this backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A bit timing outside bxCAN's ranges, or one that gives no bit rate.
    InvalidTiming,
    /// The controller didn't enter or leave initialisation mode: its clock may
    /// be off, or, leaving it, the bus not idle.
    InitTimeout,
    /// The [`State`] already serves a channel.
    StateInUse,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::InvalidTiming => "bit timing out of bxCAN's range",
            Self::InitTimeout => "the controller didn't change mode",
            Self::StateInUse => "the state already serves a channel",
        })
    }
}

impl core::error::Error for Error {}

/// Polls of the status register before giving up on a mode change: far
/// longer than the few bit times entering initialisation takes.
const MODE_CHANGE_POLLS: u32 = 1_000_000;

/// A bxCAN channel with confirmed transmission.
///
/// Its events are read with [`ReceiveEvents`], or asynchronously with
/// [`next_event`](Self::next_event).
pub struct Can<C: Clock + 'static, const N: usize> {
    regs: Regs,
    state: &'static State<C, N>,
}

impl<C: Clock, const N: usize> Can<C, N> {
    /// Configures the controller and binds it to `state`, leaving it in
    /// initialisation mode: unmask its interrupts, then [`start`](Self::start)
    /// it. Every identifier is received, into FIFO 0.
    ///
    /// Frames go on the bus in the order they are handed over (bxCAN's
    /// chronological mode, not by identifier).
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't enter initialisation
    /// mode, and [`Error::StateInUse`] if `state` already serves a channel.
    #[expect(clippy::needless_pass_by_value, reason = "taking the Peripheral is what makes the channel the controller's only user")]
    pub fn new(peripheral: Peripheral, state: &'static State<C, N>, config: Config) -> Result<Self, Error> {
        let Peripheral(regs) = peripheral;
        let r = regs.get();

        // Out of sleep, into initialisation.
        r.mcr.modify(|mcr| (mcr & !MCR_SLEEP) | MCR_INRQ);
        wait_for(r, MSR_INAK | MSR_SLAK, MSR_INAK)?;
        configure(r, config);

        // Filter bank 0: a 32-bit mask of 0, passing everything to FIFO 0.
        r.fmr.modify(|fmr| fmr | FMR_FINIT);
        r.fa1r.modify(|v| v & !1);
        r.fm1r.modify(|v| v & !1);
        r.fs1r.modify(|v| v | 1);
        r.ffa1r.modify(|v| v & !1);
        r.filter[0].fr1.write(0);
        r.filter[0].fr2.write(0);
        r.fa1r.modify(|v| v | 1);
        r.fmr.modify(|fmr| fmr & !FMR_FINIT);

        state.with(|inner| {
            if inner.regs.is_some() {
                return Err(Error::StateInUse);
            }
            inner.regs = Some(regs);
            inner.report_bus_errors = config.bus_errors;
            Ok(())
        })?;
        r.ier.write(interrupts(config));
        Ok(Self { regs, state })
    }

    fn registers(&self) -> &RegisterBlock {
        self.regs.get()
    }

    /// Stops the controller: aborts the transmissions outstanding, whose
    /// reports follow (`Aborted`, or `Sent` if a frame was already on its
    /// way), and enters initialisation mode, leaving the bus. [`start`](Self::start)
    /// rejoins it.
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't enter initialisation
    /// mode.
    pub fn stop(&mut self) -> Result<(), Error> {
        let regs = self.regs;
        self.state.with(|inner| {
            for mailbox in inner.outstanding.abort_unfinished() {
                regs.get().tsr.write(tsr_abrq(usize::from(mailbox)));
            }
        });
        // A mailbox empties once its frame is aborted, or sent.
        let r = self.registers();
        let empty = tsr_tme(0) | tsr_tme(1) | tsr_tme(2);
        for _ in 0..MODE_CHANGE_POLLS {
            if r.tsr.read() & empty == empty {
                break;
            }
            core::hint::spin_loop();
        }
        r.mcr.modify(|mcr| mcr | MCR_INRQ);
        wait_for(r, MSR_INAK, MSR_INAK)
    }

    /// Stops the controller if it is running and sets it up afresh with
    /// `config`, leaving it stopped: then [`start`](Self::start) it.
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't enter initialisation
    /// mode.
    pub fn reconfigure(&mut self, config: Config) -> Result<(), Error> {
        self.stop()?;
        configure(self.registers(), config);
        self.registers().ier.write(interrupts(config));
        self.state.with(|inner| {
            inner.report_bus_errors = config.bus_errors;
            inner.bus_error = None;
        });
        Ok(())
    }

    /// Leaves initialisation mode. The controller joins the bus once it has
    /// seen it idle (11 recessive bits); in silent loopback, at once.
    pub fn start(&mut self) {
        self.registers().mcr.modify(|mcr| mcr & !MCR_INRQ);
    }

    /// A read-only view of the channel's error state, to read it without
    /// the channel.
    #[must_use]
    pub const fn status(&self) -> Status {
        Status { regs: self.regs }
    }

    /// Whether the controller has joined the bus since [`start`](Self::start).
    #[must_use]
    pub fn is_synchronized(&self) -> bool {
        self.registers().msr.read() & MSR_INAK == 0
    }

    /// The next bus error, waiting for one, when the channel was configured
    /// to report them ([`Config::bus_errors`]); otherwise it waits for ever.
    /// Errors that come faster than they are read are counted
    /// ([`Statistics`]) but only the latest is kept.
    pub async fn next_bus_error(&self) -> BusError {
        poll_fn(|cx| {
            self.state.with(|inner| {
                if let Some(error) = inner.bus_error.take() {
                    Poll::Ready(error)
                } else {
                    inner.error_waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await
    }

    /// The next event, waiting for one.
    pub async fn next_event(&self) -> Event<CanFrame, C::Instant> {
        let regs = self.regs;
        poll_fn(|cx| {
            self.state.with(|inner| {
                inner.update_bus_state(regs.get());
                if let Some(event) = inner.events.pop() {
                    Poll::Ready(event)
                } else {
                    inner.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await
    }
}

/// The interrupts `config` needs: transmissions finished, frames received,
/// overruns, error passive and bus-off, and each bus error if reported.
const fn interrupts(config: Config) -> u32 {
    let base = IER_TMEIE | IER_FMPIE0 | IER_FMPIE1 | IER_FOVIE0 | IER_FOVIE1 | IER_EPVIE | IER_BOFIE | IER_ERRIE;
    if config.bus_errors { base | IER_LECIE } else { base }
}

/// Writes `config` to the controller, which must be in initialisation mode.
fn configure(r: &RegisterBlock, config: Config) {
    // Keep the debug freeze bit (16) as it is; transmit in request order;
    // stamp frames with the bit-time counter, to put them in bus order.
    let keep = r.mcr.read() & (1 << 16);
    let nart = if config.retransmit { 0 } else { MCR_NART };
    r.mcr.write(keep | MCR_INRQ | MCR_TXFP | MCR_TTCM | nart);

    let mode = match config.mode {
        Mode::Normal => 0,
        Mode::Silent => BTR_SILM,
        Mode::Loopback => BTR_LBKM,
        Mode::SilentLoopback => BTR_SILM | BTR_LBKM,
    };
    r.btr.write(config.timing.btr() | mode);
}

/// Polls MSR until its `mask` bits are `value`.
fn wait_for(r: &RegisterBlock, mask: u32, value: u32) -> Result<(), Error> {
    for _ in 0..MODE_CHANGE_POLLS {
        if r.msr.read() & mask == value {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(Error::InitTimeout)
}

impl<C: Clock, const N: usize> ConfirmedTransmit<CanFrame> for Can<C, N> {
    type Error = Error;

    fn capacity(&self) -> usize {
        MAILBOXES
    }

    fn try_transmit(&mut self, frame: &CanFrame) -> Result<TransmitId, TryTransmitError<Error>> {
        let regs = self.regs;
        self.state.with(|inner| {
            let r = regs.get();
            let tsr = r.tsr.read();
            if inner.outstanding.is_full() || tsr & (tsr_tme(0) | tsr_tme(1) | tsr_tme(2)) == 0 {
                return Err(TryTransmitError::Full);
            }
            // CODE: the number of an empty mailbox.
            let mailbox = ((tsr >> 24) & 0b11) as u8;
            let words = frame::encode(frame);
            let tx = &r.tx[usize::from(mailbox)];
            tx.tdtr.write(words.dtr);
            tx.tdlr.write(words.dlr);
            tx.tdhr.write(words.dhr);
            let id = inner.take_id();
            inner.outstanding.push(id, mailbox);
            tx.tir.write(words.ir | IR_TXRQ);
            Ok(id)
        })
    }

    fn abort(&mut self, id: TransmitId) -> Result<(), AbortError<Error>> {
        let regs = self.regs;
        self.state.with(|inner| {
            if let Some(mailbox) = inner.outstanding.request_abort(id) {
                regs.get().tsr.write(tsr_abrq(usize::from(mailbox)));
            }
        });
        Ok(())
    }
}

impl<C: Clock, const N: usize> ReceiveEvents<CanFrame> for Can<C, N> {
    type Error = Error;
    type Timestamp = C::Instant;

    fn receive_event(&mut self) -> Result<Event<CanFrame, C::Instant>, Error> {
        loop {
            if let Some(event) = self.try_receive_event()? {
                return Ok(event);
            }
            core::hint::spin_loop();
        }
    }

    fn try_receive_event(&mut self) -> Result<Option<Event<CanFrame, C::Instant>>, Error> {
        let regs = self.regs;
        Ok(self.state.with(|inner| {
            inner.update_bus_state(regs.get());
            inner.events.pop()
        }))
    }

    fn receive_event_timeout(&mut self, timeout: Duration) -> Result<Option<Event<CanFrame, C::Instant>>, Error> {
        let deadline = C::deadline(timeout);
        loop {
            if let Some(event) = self.try_receive_event()? {
                return Ok(Some(event));
            }
            if C::has_passed(&deadline) {
                return Ok(None);
            }
            core::hint::spin_loop();
        }
    }
}

impl<C: Clock, const N: usize> BusStatus for Can<C, N> {
    type Error = Error;

    fn bus_state(&self) -> Result<BusState, Error> {
        self.status().bus_state()
    }

    fn error_counters(&self) -> Result<ErrorCounters, Error> {
        self.status().error_counters()
    }
}

/// A read-only view of a channel's error state, for code that doesn't hold
/// the channel: a USB control handler answering a request for it, say. It
/// only reads the error status register.
#[derive(Clone, Copy)]
pub struct Status {
    regs: Regs,
}

impl BusStatus for Status {
    type Error = Error;

    fn bus_state(&self) -> Result<BusState, Error> {
        Ok(state::bus_state(self.regs.get().esr.read()))
    }

    fn error_counters(&self) -> Result<ErrorCounters, Error> {
        let esr = self.regs.get().esr.read();
        #[expect(clippy::cast_possible_truncation, reason = "the counters are eight-bit fields")]
        Ok(ErrorCounters {
            transmit: (esr >> 16) as u8,
            receive: (esr >> 24) as u8,
        })
    }
}

impl<C: Clock, const N: usize> Recover for Can<C, N> {
    type Error = Error;

    /// bxCAN recovers when software enters and leaves initialisation mode
    /// (with automatic bus-off management off, as here).
    fn recover(&mut self) -> Result<(), Error> {
        let r = self.registers();
        r.mcr.modify(|mcr| mcr | MCR_INRQ);
        wait_for(r, MSR_INAK, MSR_INAK)?;
        r.mcr.modify(|mcr| mcr & !MCR_INRQ);
        Ok(())
    }
}

impl<C: Clock, const N: usize> Statistics for Can<C, N> {
    type Error = Error;

    fn statistics(&self) -> Result<BusStatistics, Error> {
        let counts = self.state.with(|inner| inner.counts);
        let mut statistics = BusStatistics::default();
        statistics.rx_frames = Some(u64::from(counts.rx_frames));
        statistics.tx_frames = Some(u64::from(counts.tx_frames));
        statistics.rx_dropped = Some(u64::from(counts.rx_dropped));
        statistics.rx_overruns = Some(u64::from(counts.rx_overruns));
        statistics.tx_failed = Some(u64::from(counts.tx_failed));
        statistics.bus_errors = Some(u64::from(counts.bus_errors));
        Ok(statistics)
    }
}

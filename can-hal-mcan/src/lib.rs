// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! A [can-hal](can_hal) backend for Bosch's `M_CAN` CAN FD controller, starting
//! with ST's FDCAN: the cut-down `M_CAN` of the STM32G0, G4, L5, U5 and H5, whose
//! message RAM has a fixed layout.
//!
//! The channel confirms its transmissions: it implements can-hal's
//! [`ConfirmedTransmit`] (for [`Frame`] and [`CanFrame`]) and
//! [`ReceiveEvents`] (for [`Frame`]), reporting each frame
//! [`Sent`](can_hal::Outcome::Sent) from the controller's Tx event FIFO, and
//! interleaving those reports with the frames received in the order they
//! started on the bus. It also implements [`BusStatus`], [`Recover`] and
//! [`Statistics`].
//!
//! The crate drives the controller's registers itself and depends on no HAL:
//! the application enables the controller's kernel clock, sets up its pins,
//! and calls [`State::on_interrupt`] from the handler of its interrupt line 0.
//! Classic frames keep their DLC, 9 to 15 included, and received remote
//! frames carry no data.
//!
//! ```ignore
//! static STATE: State<MyClock, 32> = State::new();
//!
//! // The controller's kernel clock is on, its pins are set to FDCAN.
//! let peripheral = unsafe { Peripheral::new(0x4000_6400, 0x4000_A400) };
//! let nominal = BitTiming::solve_nominal(80_000_000, 500_000, SamplePoint::from_per_mille(875))?;
//! let data = BitTiming::solve_data(80_000_000, 2_000_000, SamplePoint::from_per_mille(800))?;
//! let mut can = Can::new(peripheral, &STATE, Config::new(nominal).with_data(data))?;
//! // Unmask FDCAN1_IT0, whose handler calls STATE.on_interrupt().
//! can.start()?;
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

use can_hal::{AbortError, BusState, BusStatistics, BusStatus, CanFrame, ConfirmedTransmit, ErrorCounters, Event, Frame, ReceiveEvents, Recover, Statistics, TransmitId, TryTransmitError};

pub use state::State;
pub use timing::BitTiming;

use order::MAILBOXES;
use registers::{
    CCCR_BRSE, CCCR_CCE, CCCR_DAR, CCCR_FDOE, CCCR_INIT, CCCR_MON, CCCR_TEST, DBTP_TDC, H1_MM_SHIFT, ILE_EINT0, IR_BO, IR_EP, IR_PEA, IR_PED, IR_RF0L, IR_RF0N, IR_TCF, IR_TEFN, MessageRam, PSR_ACT_MASK, RegisterBlock, Regs, T1_EFC, TEST_LBCK,
    TOCC_ETOC, TOCC_TOP_SHIFT, TSCC_TSS_INTERNAL, TXFQS_TFQF, TXFQS_TFQPI_SHIFT,
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

/// An FDCAN controller's registers and message RAM, not yet configured.
pub struct Peripheral(Regs);

impl Peripheral {
    /// The controller whose register block is at `registers` and whose
    /// message RAM is at `message_ram`: on the STM32G4, FDCAN1's are
    /// `0x4000_6400` and `0x4000_A400`, FDCAN2's `0x4000_6800` and
    /// `0x4000_A750`, FDCAN3's `0x4000_6C00` and `0x4000_AAA0`.
    ///
    /// # Safety
    ///
    /// The addresses must be one FDCAN controller's, and nothing else may use
    /// that controller while this, or the [`Can`] made from it, exists.
    #[must_use]
    pub const unsafe fn new(registers: usize, message_ram: usize) -> Self {
        // SAFETY: passed on from the caller.
        Self(unsafe { Regs::new(registers, message_ram) })
    }
}

/// What the controller does with the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// Takes part in the bus.
    #[default]
    Normal,
    /// Bus monitoring: listens, and never transmits or acknowledges.
    /// Transmit requests wait.
    Silent,
    /// External loopback: receives its own frames and drives them on the bus
    /// too, but ignores the bus's acknowledgement.
    Loopback,
    /// Internal loopback: receives its own frames and neither drives the bus
    /// nor listens to it; a self-test, safe whatever the pins are wired to,
    /// and one that needs no pins at all.
    SilentLoopback,
}

/// How to set up the controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// The nominal bit timing, of classic frames and of FD frames' arbitration.
    pub nominal: BitTiming,
    /// The data phase's timing, for FD frames with bit-rate switching. With
    /// none, the channel sends and receives classic frames only.
    pub data: Option<BitTiming>,
    /// The mode.
    pub mode: Mode,
    /// Whether the controller retries a frame until it is sent (the CAN
    /// default), or tries once and gives up (single-shot).
    ///
    /// Single-shot, the channel takes the bus error interrupts whether or
    /// not it reports them, and the timeout counter: a frame that fails
    /// raises no interrupt of its own.
    pub retransmit: bool,
    /// Whether to report each bus error, through [`Can::next_bus_error`].
    /// Off by default: on a faulty bus there can be thousands a second.
    pub bus_errors: bool,
}

impl Config {
    /// A normal classic channel with the given timing, retrying frames until
    /// sent.
    #[must_use]
    pub const fn new(nominal: BitTiming) -> Self {
        Self {
            nominal,
            data: None,
            mode: Mode::Normal,
            retransmit: true,
            bus_errors: false,
        }
    }

    /// The same, taking FD frames too, with `data` the timing of their data
    /// phase when they switch bit rate.
    #[must_use]
    pub const fn with_data(mut self, data: BitTiming) -> Self {
        self.data = Some(data);
        self
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

/// A bus error the controller saw, from its last error code, in the nominal
/// phase or, failing that, the data phase.
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
    /// The error a last error code (PSR's LEC or DLEC) stands for, if any.
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
    /// A bit timing outside the controller's ranges, or one that gives no bit
    /// rate.
    InvalidTiming,
    /// The controller didn't enter or leave initialisation: its kernel clock
    /// may be off.
    InitTimeout,
    /// The [`State`] already serves a channel.
    StateInUse,
    /// An FD frame, on a channel configured without a data phase.
    FdDisabled,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::InvalidTiming => "bit timing out of the controller's range",
            Self::InitTimeout => "the controller didn't change mode",
            Self::StateInUse => "the state already serves a channel",
            Self::FdDisabled => "an FD frame on a classic channel",
        })
    }
}

impl core::error::Error for Error {}

/// Polls of the control register before giving up on a mode change: far
/// longer than synchronising the two clock domains takes.
const MODE_CHANGE_POLLS: u32 = 1_000_000;

/// The FDCAN's transmitter delay compensation works with a data prescaler of
/// one or two only, which fast data phases need anyway.
const TDC_MAX_PRESCALER: u16 = 2;

/// Bit times from a bus error to the end of its error frame, and so to a
/// single-shot frame it failed being marked cancelled: up to 12 bits of error
/// flags, 8 of delimiter and 3 of intermission, with room to spare.
pub(crate) const ERROR_FRAME_BITS: u32 = 32;

/// An FDCAN channel with confirmed transmission.
///
/// Its events are read with [`ReceiveEvents`], or asynchronously with
/// [`next_event`](Self::next_event).
pub struct Can<C: Clock + 'static, const N: usize> {
    regs: Regs,
    state: &'static State<C, N>,
}

impl<C: Clock, const N: usize> Can<C, N> {
    /// Configures the controller and binds it to `state`, leaving it in
    /// initialisation: unmask its interrupt line 0, then [`start`](Self::start)
    /// it. Every identifier is received, into FIFO 0.
    ///
    /// Frames go on the bus in the order they are handed over (the transmit
    /// FIFO, not the identifier-ordered queue).
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't enter initialisation,
    /// and [`Error::StateInUse`] if `state` already serves a channel.
    #[expect(clippy::needless_pass_by_value, reason = "taking the Peripheral is what makes the channel the controller's only user")]
    pub fn new(peripheral: Peripheral, state: &'static State<C, N>, config: Config) -> Result<Self, Error> {
        let Peripheral(regs) = peripheral;
        let r = regs.get();
        enter_init(r)?;
        state.with(|inner| {
            if inner.regs.is_some() {
                return Err(Error::StateInUse);
            }
            inner.regs = Some(regs);
            inner.report_bus_errors = config.bus_errors;
            inner.single_shot = !config.retransmit;
            Ok(())
        })?;
        configure(r, regs.ram(), config);
        Ok(Self { regs, state })
    }

    fn registers(&self) -> &RegisterBlock {
        self.regs.get()
    }

    /// Stops the controller: cancels the transmissions outstanding, whose
    /// reports follow (`Aborted`, or `Sent` if a frame was already on its
    /// way; one that had already ended keeps its outcome), and enters
    /// initialisation, leaving the bus. [`start`](Self::start) rejoins it.
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't enter initialisation.
    pub fn stop(&mut self) -> Result<(), Error> {
        let regs = self.regs;
        self.state.with(|inner| {
            // What has finished already is reported as it ended, not as
            // aborted: a single-shot frame may have failed unreported.
            inner.handle(regs, &C::now());
            for mailbox in inner.outstanding.abort_unfinished() {
                regs.get().txbcr.write(1 << mailbox);
            }
        });
        // A buffer stops pending once its frame is cancelled, or sent.
        let r = self.registers();
        for _ in 0..MODE_CHANGE_POLLS {
            if r.txbrp.read() == 0 {
                break;
            }
            core::hint::spin_loop();
        }
        enter_init(r)?;
        self.state.with(|inner| inner.handle(regs, &C::now()));
        Ok(())
    }

    /// Stops the controller if it is running and sets it up afresh with
    /// `config`, leaving it stopped: then [`start`](Self::start) it.
    ///
    /// A channel set up afresh counts as error active. The error counters
    /// carry over, though, so if they still say otherwise, the first read of
    /// the state after this reports the change.
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't enter initialisation.
    pub fn reconfigure(&mut self, config: Config) -> Result<(), Error> {
        self.stop()?;
        configure(self.registers(), self.regs.ram(), config);
        self.state.with(|inner| {
            inner.report_bus_errors = config.bus_errors;
            inner.single_shot = !config.retransmit;
            inner.watching = false;
            inner.bus_error = None;
            inner.bus_state = BusState::ErrorActive;
        });
        Ok(())
    }

    /// Leaves initialisation. The controller joins the bus once it has seen
    /// it idle (11 recessive bits); in internal loopback, at once.
    ///
    /// # Errors
    ///
    /// [`Error::InitTimeout`] if the controller doesn't leave initialisation.
    pub fn start(&mut self) -> Result<(), Error> {
        leave_init(self.registers())
    }

    /// A read-only view of the channel's error state, to read it without
    /// the channel.
    #[must_use]
    pub const fn status(&self) -> Status<C, N> {
        Status { regs: self.regs, state: self.state }
    }

    /// Whether the controller has joined the bus since [`start`](Self::start):
    /// it is out of initialisation and no longer synchronising.
    #[must_use]
    pub fn is_synchronized(&self) -> bool {
        let regs = self.regs;
        regs.get().cccr.read() & CCCR_INIT == 0 && self.state.with(|inner| inner.update_bus_state(regs.get())) & PSR_ACT_MASK != 0
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
    pub async fn next_event(&self) -> Event<Frame, C::Instant> {
        let regs = self.regs;
        poll_fn(|cx| {
            self.state.with(|inner| {
                let _ = inner.update_bus_state(regs.get());
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

/// Sets CCCR.INIT and waits for it to take.
fn enter_init(r: &RegisterBlock) -> Result<(), Error> {
    r.cccr.modify(|cccr| cccr | CCCR_INIT);
    wait_for_init(r, CCCR_INIT)
}

/// Clears CCCR.CCE then CCCR.INIT, and waits for it to take.
fn leave_init(r: &RegisterBlock) -> Result<(), Error> {
    r.cccr.modify(|cccr| cccr & !CCCR_CCE);
    r.cccr.modify(|cccr| cccr & !CCCR_INIT);
    wait_for_init(r, 0)
}

/// Polls CCCR until its INIT bit is `value`.
fn wait_for_init(r: &RegisterBlock, value: u32) -> Result<(), Error> {
    for _ in 0..MODE_CHANGE_POLLS {
        if r.cccr.read() & CCCR_INIT == value {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(Error::InitTimeout)
}

/// The interrupts `config` needs: Tx events, frames received, overruns,
/// cancellations finished, error passive and bus-off changes, and each bus
/// error if reported or if frames are single-shot (see
/// [`Inner::watch_failures`](state::Inner::watch_failures)). The timeout
/// counter's is enabled only while it is needed.
const fn interrupts(config: Config) -> u32 {
    let base = IR_TEFN | IR_RF0N | IR_RF0L | IR_TCF | IR_EP | IR_BO;
    if config.bus_errors || !config.retransmit { base | IR_PEA | IR_PED } else { base }
}

/// Writes `config` to the controller, which must be in initialisation, and
/// clears its message RAM.
fn configure(r: &RegisterBlock, ram: &MessageRam, config: Config) {
    r.cccr.modify(|cccr| cccr | CCCR_CCE);
    clear(ram);

    let fd = if config.data.is_some() { CCCR_FDOE | CCCR_BRSE } else { 0 };
    let dar = if config.retransmit { 0 } else { CCCR_DAR };
    let (mode, test) = match config.mode {
        Mode::Normal => (0, 0),
        Mode::Silent => (CCCR_MON, 0),
        Mode::Loopback => (CCCR_TEST, TEST_LBCK),
        Mode::SilentLoopback => (CCCR_TEST | CCCR_MON, TEST_LBCK),
    };
    r.cccr.write(CCCR_INIT | CCCR_CCE | fd | dar | mode);
    r.test.write(test);

    r.nbtp.write(config.nominal.nbtp());
    // The data phase, with transmitter delay compensation when it is fast
    // enough to need it: the secondary sample point is then the data sample
    // point after the measured delay.
    if let Some(data) = config.data {
        if data.prescaler() <= TDC_MAX_PRESCALER {
            r.tdcr.write((data.sample_offset() & 0x7F) << 8);
            r.dbtp.write(data.dbtp() | DBTP_TDC);
        } else {
            r.tdcr.write(0);
            r.dbtp.write(data.dbtp());
        }
    }

    // No filters: everything into FIFO 0, which keeps the oldest when full.
    r.rxgfc.write(0);
    // Stamps count bit times, and so does the timeout counter, which
    // single-shot frames need.
    r.tscc.write(TSCC_TSS_INTERNAL);
    r.tocc.write(if config.retransmit { 0 } else { (ERROR_FRAME_BITS << TOCC_TOP_SHIFT) | TOCC_ETOC });
    // The transmit FIFO, in hand-over order.
    r.txbc.write(0);

    r.ie.write(interrupts(config));
    r.ils.write(0);
    r.ile.write(ILE_EINT0);
    r.txbtie.write(0);
    r.txbcie.write((1 << MAILBOXES) - 1);
    r.ir.write(u32::MAX);
}

/// Zeroes the message RAM, whose contents are undefined after power-on.
fn clear(ram: &MessageRam) {
    for word in &ram.standard_filters {
        word.write(0);
    }
    for word in &ram.extended_filters {
        word.write(0);
    }
    for element in ram.rx_fifo.iter().flatten().chain(&ram.tx_buffers) {
        for word in element.header.iter().chain(&element.data) {
            word.write(0);
        }
    }
    for word in ram.tx_events.iter().flatten() {
        word.write(0);
    }
}

impl<C: Clock, const N: usize> ConfirmedTransmit<Frame> for Can<C, N> {
    type Error = Error;

    fn capacity(&self) -> usize {
        MAILBOXES
    }

    fn try_transmit(&mut self, frame: &Frame) -> Result<TransmitId, TryTransmitError<Error>> {
        let regs = self.regs;
        let r = regs.get();
        if matches!(frame, Frame::Fd(_)) && r.cccr.read() & CCCR_FDOE == 0 {
            return Err(TryTransmitError::Backend(Error::FdDisabled));
        }
        let now = C::now();
        self.state.with(|inner| {
            // Take in what has finished, so that a buffer the controller has
            // freed has no transmission left unfinished in it.
            inner.handle(regs, &now);
            let txfqs = r.txfqs.read();
            if inner.outstanding.is_full() || txfqs & TXFQS_TFQF != 0 {
                return Err(TryTransmitError::Full);
            }
            let mailbox = ((txfqs >> TXFQS_TFQPI_SHIFT) & 0b11) as u8;
            let mut element = frame::encode(frame);
            element.h1 |= T1_EFC | (u32::from(mailbox) << H1_MM_SHIFT);
            state::write_element(&regs.ram().tx_buffers[usize::from(mailbox)], &element);
            let id = inner.take_id();
            inner.outstanding.push(id, mailbox);
            r.txbar.write(1 << mailbox);
            Ok(id)
        })
    }

    fn abort(&mut self, id: TransmitId) -> Result<(), AbortError<Error>> {
        let regs = self.regs;
        self.state.with(|inner| {
            if let Some(mailbox) = inner.outstanding.request_abort(id) {
                regs.get().txbcr.write(1 << mailbox);
            }
        });
        Ok(())
    }
}

impl<C: Clock, const N: usize> ConfirmedTransmit<CanFrame> for Can<C, N> {
    type Error = Error;

    fn capacity(&self) -> usize {
        MAILBOXES
    }

    fn try_transmit(&mut self, frame: &CanFrame) -> Result<TransmitId, TryTransmitError<Error>> {
        ConfirmedTransmit::<Frame>::try_transmit(self, &Frame::Can(frame.clone()))
    }

    fn abort(&mut self, id: TransmitId) -> Result<(), AbortError<Error>> {
        ConfirmedTransmit::<Frame>::abort(self, id)
    }
}

impl<C: Clock, const N: usize> ReceiveEvents<Frame> for Can<C, N> {
    type Error = Error;
    type Timestamp = C::Instant;

    fn receive_event(&mut self) -> Result<Event<Frame, C::Instant>, Error> {
        loop {
            if let Some(event) = self.try_receive_event()? {
                return Ok(event);
            }
            core::hint::spin_loop();
        }
    }

    fn try_receive_event(&mut self) -> Result<Option<Event<Frame, C::Instant>>, Error> {
        let regs = self.regs;
        Ok(self.state.with(|inner| {
            let _ = inner.update_bus_state(regs.get());
            inner.events.pop()
        }))
    }

    fn receive_event_timeout(&mut self, timeout: Duration) -> Result<Option<Event<Frame, C::Instant>>, Error> {
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
/// reads the protocol status through the channel's state, so that a bus
/// error the read clears is still reported.
pub struct Status<C: Clock + 'static, const N: usize> {
    regs: Regs,
    state: &'static State<C, N>,
}

impl<C: Clock, const N: usize> Clone for Status<C, N> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<C: Clock, const N: usize> Copy for Status<C, N> {}

impl<C: Clock, const N: usize> BusStatus for Status<C, N> {
    type Error = Error;

    fn bus_state(&self) -> Result<BusState, Error> {
        let regs = self.regs;
        Ok(self.state.with(|inner| {
            let _ = inner.update_bus_state(regs.get());
            inner.bus_state
        }))
    }

    fn error_counters(&self) -> Result<ErrorCounters, Error> {
        let ecr = self.regs.get().ecr.read();
        #[expect(clippy::cast_possible_truncation, reason = "the counters are seven- and eight-bit fields")]
        Ok(ErrorCounters {
            transmit: ecr as u8,
            receive: ((ecr >> 8) & 0x7F) as u8,
        })
    }
}

impl<C: Clock, const N: usize> Recover for Can<C, N> {
    type Error = Error;

    /// Going bus-off puts the controller in initialisation; leaving it starts
    /// recovery, which ends after 129 runs of 11 recessive bits. If the
    /// controller isn't bus-off, it re-enters and leaves initialisation.
    fn recover(&mut self) -> Result<(), Error> {
        let r = self.registers();
        enter_init(r)?;
        leave_init(r)
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

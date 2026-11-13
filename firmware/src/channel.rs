// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! What the `gs_usb` bridge needs of a CAN channel, whichever controller
//! drives it: [`Channel`], implemented for `can-hal-bxcan`'s channel and
//! `can-hal-fdcan`'s.
//!
//! The bridge works with can-hal's [`Frame`], classic or FD, and its own
//! [`Settings`], which each implementation turns into its backend's.

use core::future::Future;

use can_hal::{BusState, BusStatus, ErrorCounters, Event, Frame, TransmitId};
use embassy_time::Instant;
use gs_usb_protocol::DeviceBittiming;

#[cfg(feature = "bxcan")]
mod bxcan;
#[cfg(feature = "fdcan")]
mod fdcan;

/// A bit timing as the host gives it, checked against a controller's
/// [`Limits`]: `seg1` is the propagation and phase 1 segments together.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct Timing {
    pub prescaler: u16,
    pub seg1: u16,
    pub seg2: u8,
    pub sjw: u8,
}

/// The ranges of a controller's bit timing fields, as `gs_usb`'s `BT_CONST`
/// reports them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Limits {
    pub tseg1_max: u16,
    pub tseg2_max: u8,
    pub sjw_max: u8,
    pub brp_max: u16,
}

impl Limits {
    /// The host's timing, if it is within these limits: every field at
    /// least one, and the jump width no more than phase 2.
    #[must_use]
    pub fn timing(&self, bt: &DeviceBittiming) -> Option<Timing> {
        let timing = Timing {
            prescaler: u16::try_from(bt.brp).ok()?,
            seg1: u16::try_from(bt.prop_seg.checked_add(bt.phase_seg1)?).ok()?,
            seg2: u8::try_from(bt.phase_seg2).ok()?,
            sjw: u8::try_from(bt.sjw).ok()?,
        };
        let within = (1..=self.brp_max).contains(&timing.prescaler) && (1..=self.tseg1_max).contains(&timing.seg1) && (1..=self.tseg2_max).contains(&timing.seg2) && (1..=self.sjw_max.min(timing.seg2)).contains(&timing.sjw);
        within.then_some(timing)
    }
}

/// What the controller does with the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Mode {
    /// Takes part in the bus.
    Normal,
    /// Listens, and never transmits or acknowledges.
    Silent,
    /// Receives its own frames, and drives them on the bus too.
    Loopback,
    /// Receives its own frames, and neither drives the bus nor listens to it.
    SilentLoopback,
}

/// How the host asked the channel to run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct Settings {
    /// The nominal bit timing.
    pub nominal: Timing,
    /// The data phase's, for CAN FD; none for a classic channel.
    pub data: Option<Timing>,
    pub mode: Mode,
    /// Retry frames until they are sent, rather than trying each once.
    pub retransmit: bool,
    /// Report each bus error.
    pub bus_errors: bool,
}

/// A bus error the controller saw.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BusError {
    Stuff,
    Form,
    Acknowledgement,
    BitRecessive,
    BitDominant,
    Crc,
    /// One the backend knows and this firmware doesn't.
    Other,
}

/// Why the controller gave up on a frame, when it says.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FailureCause {
    /// Its last attempt lost arbitration.
    LostArbitration,
    /// Its last attempt failed on an error.
    Error,
}

/// What went wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Fault {
    /// No room for another transmission.
    Full,
    /// The controller can't take this timing.
    InvalidTiming,
    /// An FD frame, on a classic channel or controller.
    FdUnsupported,
    /// The controller didn't do as asked.
    Controller,
}

/// A CAN channel, as the bridge drives it.
pub trait Channel {
    /// A read-only view of the error state, for the control handler.
    type Status: BusStatus + 'static;

    /// The kernel clock the host's bit timings divide.
    const CLOCK_HZ: u32;
    /// The nominal bit timing's limits.
    const NOMINAL: Limits;
    /// The data phase's limits, if the controller does CAN FD.
    const DATA: Option<Limits>;

    /// A view of the channel's error state.
    fn status(&self) -> Self::Status;

    /// Hands `frame` over to be sent, to be reported by an event.
    ///
    /// # Errors
    ///
    /// [`Fault::Full`] when there is no room, [`Fault::FdUnsupported`] for an
    /// FD frame the channel can't send.
    fn transmit(&mut self, frame: &Frame) -> Result<TransmitId, Fault>;

    /// The next event, waiting for one.
    fn next_event(&self) -> impl Future<Output = Event<Frame, Instant>>;

    /// The next bus error, when they are reported; otherwise never.
    fn next_bus_error(&self) -> impl Future<Output = BusError>;

    /// Asks for transmission `id` to be withdrawn if its transmission hasn't
    /// started; its report says whether it was.
    ///
    /// # Errors
    ///
    /// [`Fault::Controller`] if the controller refuses.
    fn abort(&mut self, id: TransmitId) -> Result<(), Fault>;

    /// Why the controller gave up on transmission `id`, reported failed, if
    /// it says (bxCAN does; FDCAN, without retransmission, doesn't tell lost
    /// arbitration from an error). Asked once, as the report is read.
    fn failure_cause(&self, id: TransmitId) -> Option<FailureCause>;

    /// Whether an error counter is at the warning level (96), once that has
    /// changed, waiting for a change. It follows the counters, so it stays
    /// set while error passive or bus-off.
    fn next_warning_change(&self) -> impl Future<Output = bool>;

    /// Leaves the bus, cancelling the transmissions outstanding.
    ///
    /// # Errors
    ///
    /// [`Fault::Controller`] if the controller doesn't stop.
    fn stop(&mut self) -> Result<(), Fault>;

    /// Sets the channel up with `settings` and joins the bus, as from
    /// power-on: the controller is reset, so nothing from before carries
    /// over (frames, events, the error counters).
    ///
    /// # Errors
    ///
    /// [`Fault::InvalidTiming`] if the controller can't take the timing,
    /// [`Fault::Controller`] if it doesn't start.
    fn start(&mut self, settings: &Settings) -> Result<(), Fault>;

    /// Starts recovery from bus-off.
    ///
    /// # Errors
    ///
    /// [`Fault::Controller`] if the controller doesn't respond.
    fn recover(&mut self) -> Result<(), Fault>;

    /// The bus state.
    fn bus_state(&self) -> BusState;

    /// The error counters.
    fn error_counters(&self) -> ErrorCounters;

    /// The frames lost so far: dropped from the event queue, or overrun in
    /// the controller.
    fn lost_frames(&self) -> u64;
}

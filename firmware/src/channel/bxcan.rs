// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! [`Channel`] for `can-hal-bxcan`: classic CAN only.

use can_hal::{BusState, BusStatus, ConfirmedTransmit, ErrorCounters, Event, Frame, Recover, Statistics, Timestamped, TransmitId, TryTransmitError};
use can_hal_bxcan::{BitTiming, Can, Config};
use embassy_time::Instant;

use super::{BusError, Channel, FailureCause, Fault, Limits, Mode, Settings};
use crate::board::{CAN_CLOCK_HZ, EmbassyClock};

impl<const N: usize> Channel for Can<EmbassyClock, N> {
    type Status = can_hal_bxcan::Status;

    const CLOCK_HZ: u32 = CAN_CLOCK_HZ;
    /// BTR: prescaler 1-1024, seg1 1-16, seg2 1-8, SJW 1-4.
    const NOMINAL: Limits = Limits {
        tseg1_max: 16,
        tseg2_max: 8,
        sjw_max: 4,
        brp_max: 1024,
    };
    const DATA: Option<Limits> = None;

    fn status(&self) -> Self::Status {
        Can::status(self)
    }

    fn transmit(&mut self, frame: &Frame) -> Result<TransmitId, Fault> {
        let Frame::Can(frame) = frame else { return Err(Fault::FdUnsupported) };
        self.try_transmit(frame).map_err(|e| match e {
            TryTransmitError::Full => Fault::Full,
            TryTransmitError::Backend(_) => Fault::Controller,
        })
    }

    async fn next_event(&self) -> Event<Frame, Instant> {
        match Can::next_event(self).await {
            Event::Received(received) => {
                let at = *received.timestamp();
                Event::Received(Timestamped::new(Frame::Can(received.into_frame()), at))
            }
            Event::Transmitted { id, outcome } => Event::Transmitted { id, outcome },
            Event::BusStateChanged(state) => Event::BusStateChanged(state),
        }
    }

    fn failure_cause(&self, id: TransmitId) -> Option<FailureCause> {
        match Can::failure_cause(self, id)? {
            can_hal_bxcan::FailureCause::LostArbitration => Some(FailureCause::LostArbitration),
            can_hal_bxcan::FailureCause::Error => Some(FailureCause::Error),
            _ => None,
        }
    }

    async fn next_warning_change(&self) -> bool {
        Can::next_warning_change(self).await
    }

    async fn next_bus_error(&self) -> BusError {
        match Can::next_bus_error(self).await {
            can_hal_bxcan::BusError::Stuff => BusError::Stuff,
            can_hal_bxcan::BusError::Form => BusError::Form,
            can_hal_bxcan::BusError::Acknowledgement => BusError::Acknowledgement,
            can_hal_bxcan::BusError::BitRecessive => BusError::BitRecessive,
            can_hal_bxcan::BusError::BitDominant => BusError::BitDominant,
            can_hal_bxcan::BusError::Crc => BusError::Crc,
            _ => BusError::Other,
        }
    }

    fn stop(&mut self) -> Result<(), Fault> {
        Can::stop(self).map_err(|_| Fault::Controller)
    }

    fn start(&mut self, settings: &Settings) -> Result<(), Fault> {
        if settings.data.is_some() {
            return Err(Fault::FdUnsupported);
        }
        let t = settings.nominal;
        let timing = BitTiming::new(t.prescaler, u8::try_from(t.seg1).map_err(|_| Fault::InvalidTiming)?, t.seg2, t.sjw).map_err(|_| Fault::InvalidTiming)?;
        let mode = match settings.mode {
            Mode::Normal => can_hal_bxcan::Mode::Normal,
            Mode::Silent => can_hal_bxcan::Mode::Silent,
            Mode::Loopback => can_hal_bxcan::Mode::Loopback,
            Mode::SilentLoopback => can_hal_bxcan::Mode::SilentLoopback,
        };
        let config = Config::new(timing).with_mode(mode).with_retransmit(settings.retransmit).with_bus_errors(settings.bus_errors);
        // Every start as from power-on: nothing from before, the error
        // counters at 0.
        crate::board::reset_can();
        self.reinitialize(config).map_err(|_| Fault::Controller)?;
        Can::start(self);
        Ok(())
    }

    fn recover(&mut self) -> Result<(), Fault> {
        Recover::recover(self).map_err(|_| Fault::Controller)
    }

    fn bus_state(&self) -> BusState {
        BusStatus::bus_state(self).unwrap_or(BusState::ErrorActive)
    }

    fn error_counters(&self) -> ErrorCounters {
        BusStatus::error_counters(self).unwrap_or(ErrorCounters { transmit: 0, receive: 0 })
    }

    fn lost_frames(&self) -> u64 {
        self.statistics().map_or(0, |s| s.rx_dropped.unwrap_or(0) + s.rx_overruns.unwrap_or(0))
    }
}

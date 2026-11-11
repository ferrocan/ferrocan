// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! [`Channel`] for `can-hal-mcan`: classic CAN, and CAN FD when the host
//! gives a data phase timing.

use can_hal::{BusState, BusStatus, ConfirmedTransmit, ErrorCounters, Event, Frame, Recover, Statistics, TransmitId, TryTransmitError};
use can_hal_mcan::{BitTiming, Can, Config, Error};
use embassy_time::Instant;

use super::{BusError, Channel, Fault, Limits, Mode, Settings, Timing};
use crate::board::{CAN_CLOCK_HZ, EmbassyClock};

impl<const N: usize> Channel for Can<EmbassyClock, N> {
    type Status = can_hal_mcan::Status<EmbassyClock, N>;

    const CLOCK_HZ: u32 = CAN_CLOCK_HZ;
    /// NBTP: prescaler 1-512, seg1 1-256, seg2 1-128, SJW 1-128.
    const NOMINAL: Limits = Limits {
        tseg1_max: 256,
        tseg2_max: 128,
        sjw_max: 128,
        brp_max: 512,
    };
    /// DBTP: prescaler 1-32, seg1 1-32, seg2 1-16, SJW 1-16.
    const DATA: Option<Limits> = Some(Limits {
        tseg1_max: 32,
        tseg2_max: 16,
        sjw_max: 16,
        brp_max: 32,
    });

    fn status(&self) -> Self::Status {
        Can::status(self)
    }

    fn transmit(&mut self, frame: &Frame) -> Result<TransmitId, Fault> {
        ConfirmedTransmit::<Frame>::try_transmit(self, frame).map_err(|e| match e {
            TryTransmitError::Full => Fault::Full,
            TryTransmitError::Backend(Error::FdDisabled) => Fault::FdUnsupported,
            TryTransmitError::Backend(_) => Fault::Controller,
        })
    }

    async fn next_event(&self) -> Event<Frame, Instant> {
        Can::next_event(self).await
    }

    async fn next_bus_error(&self) -> BusError {
        match Can::next_bus_error(self).await {
            can_hal_mcan::BusError::Stuff => BusError::Stuff,
            can_hal_mcan::BusError::Form => BusError::Form,
            can_hal_mcan::BusError::Acknowledgement => BusError::Acknowledgement,
            can_hal_mcan::BusError::BitRecessive => BusError::BitRecessive,
            can_hal_mcan::BusError::BitDominant => BusError::BitDominant,
            can_hal_mcan::BusError::Crc => BusError::Crc,
            _ => BusError::Other,
        }
    }

    fn stop(&mut self) -> Result<(), Fault> {
        Can::stop(self).map_err(|_| Fault::Controller)
    }

    fn start(&mut self, settings: &Settings) -> Result<(), Fault> {
        let timing = |t: Timing, make: fn(u16, u16, u8, u8) -> Result<BitTiming, Error>| make(t.prescaler, t.seg1, t.seg2, t.sjw).map_err(|_| Fault::InvalidTiming);
        let mode = match settings.mode {
            Mode::Normal => can_hal_mcan::Mode::Normal,
            Mode::Silent => can_hal_mcan::Mode::Silent,
            Mode::Loopback => can_hal_mcan::Mode::Loopback,
            Mode::SilentLoopback => can_hal_mcan::Mode::SilentLoopback,
        };
        let mut config = Config::new(timing(settings.nominal, BitTiming::nominal)?).with_mode(mode).with_retransmit(settings.retransmit).with_bus_errors(settings.bus_errors);
        if let Some(data) = settings.data {
            config = config.with_data(timing(data, BitTiming::data)?);
        }
        self.reconfigure(config).map_err(|_| Fault::Controller)?;
        Can::start(self).map_err(|_| Fault::Controller)
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

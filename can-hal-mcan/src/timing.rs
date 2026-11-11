// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Bit timing: the nominal phase, and the data phase of CAN FD frames sent
//! with bit-rate switching.

use can_hal::SamplePoint;

use crate::Error;

/// A bit's timing in time quanta: one quantum of sync, then `seg1` before the
/// sample point and `seg2` after it, each quantum `prescaler` cycles of the
/// controller's kernel clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BitTiming {
    prescaler: u16,
    seg1: u16,
    seg2: u8,
    sjw: u8,
}

/// The ranges of a phase's fields.
struct Limits {
    prescaler: u16,
    seg1: u16,
    seg2: u8,
    sjw: u8,
}

/// NBTP: prescaler 1-512, seg1 1-256, seg2 1-128, SJW 1-128.
const NOMINAL: Limits = Limits {
    prescaler: 512,
    seg1: 256,
    seg2: 128,
    sjw: 128,
};

/// DBTP: prescaler 1-32, seg1 1-32, seg2 1-16, SJW 1-16.
const DATA: Limits = Limits { prescaler: 32, seg1: 32, seg2: 16, sjw: 16 };

impl BitTiming {
    const fn checked(limits: &Limits, prescaler: u16, seg1: u16, seg2: u8, sjw: u8) -> Result<Self, Error> {
        if prescaler < 1 || prescaler > limits.prescaler || seg1 < 1 || seg1 > limits.seg1 || seg2 < 1 || seg2 > limits.seg2 || sjw < 1 || sjw > limits.sjw || sjw > seg2 {
            return Err(Error::InvalidTiming);
        }
        Ok(Self { prescaler, seg1, seg2, sjw })
    }

    /// Nominal timing from its fields, checked against the controller's ranges:
    /// `prescaler` 1 to 512, `seg1` 1 to 256 (propagation and phase 1
    /// together), `seg2` 1 to 128, `sjw` 1 to 128 and at most `seg2`.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidTiming`] when a field is out of range.
    pub const fn nominal(prescaler: u16, seg1: u16, seg2: u8, sjw: u8) -> Result<Self, Error> {
        Self::checked(&NOMINAL, prescaler, seg1, seg2, sjw)
    }

    /// Data-phase timing from its fields: `prescaler` 1 to 32, `seg1` 1 to
    /// 32, `seg2` 1 to 16, `sjw` 1 to 16 and at most `seg2`.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidTiming`] when a field is out of range.
    pub const fn data(prescaler: u16, seg1: u16, seg2: u8, sjw: u8) -> Result<Self, Error> {
        Self::checked(&DATA, prescaler, seg1, seg2, sjw)
    }

    fn solve(limits: &Limits, clock_hz: u32, bitrate: u32, sample_point: SamplePoint) -> Result<Self, Error> {
        let target = u32::from(sample_point.per_mille());
        let most = 1 + u32::from(limits.seg1) + u32::from(limits.seg2);
        let mut best: Option<(u32, Self)> = None;
        // More quanta first: the smallest prescaler, as CiA 601-3 advises for
        // CAN FD, and the finest resynchronisation.
        for quanta in (4..=most).rev() {
            let Some(cycles) = bitrate.checked_mul(quanta) else { continue };
            if cycles == 0 || !clock_hz.is_multiple_of(cycles) {
                continue;
            }
            let Ok(prescaler) = u16::try_from(clock_hz / cycles) else { continue };
            let sample = (quanta * target + 500) / 1000;
            let (Ok(seg1), Ok(seg2)) = (u16::try_from(sample.saturating_sub(1)), u8::try_from(quanta.saturating_sub(sample))) else {
                continue;
            };
            let Ok(timing) = Self::checked(limits, prescaler, seg1, seg2, seg2.min(limits.sjw)) else { continue };
            let error = (sample * 1000 / quanta).abs_diff(target);
            if best.is_none_or(|(best_error, _)| error < best_error) {
                best = Some((error, timing));
            }
        }
        best.map(|(_, timing)| timing).ok_or(Error::InvalidTiming)
    }

    /// The nominal timing for `bitrate` from a `clock_hz` kernel clock whose
    /// sample point is nearest `sample_point`.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidTiming`] when no whole number of quanta divides the
    /// clock into `bitrate`.
    pub fn solve_nominal(clock_hz: u32, bitrate: u32, sample_point: SamplePoint) -> Result<Self, Error> {
        Self::solve(&NOMINAL, clock_hz, bitrate, sample_point)
    }

    /// The data-phase timing for `bitrate`, as [`solve_nominal`](Self::solve_nominal).
    ///
    /// # Errors
    ///
    /// [`Error::InvalidTiming`] as for [`solve_nominal`](Self::solve_nominal).
    pub fn solve_data(clock_hz: u32, bitrate: u32, sample_point: SamplePoint) -> Result<Self, Error> {
        Self::solve(&DATA, clock_hz, bitrate, sample_point)
    }

    /// Cycles of the kernel clock per quantum.
    #[must_use]
    pub const fn prescaler(self) -> u16 {
        self.prescaler
    }

    /// Quanta before the sample point, after the sync quantum.
    #[must_use]
    pub const fn seg1(self) -> u16 {
        self.seg1
    }

    /// Quanta after the sample point.
    #[must_use]
    pub const fn seg2(self) -> u8 {
        self.seg2
    }

    /// The synchronisation jump width, in quanta.
    #[must_use]
    pub const fn sjw(self) -> u8 {
        self.sjw
    }

    /// The bit rate from a `clock_hz` kernel clock.
    #[must_use]
    pub const fn bitrate(self, clock_hz: u32) -> u32 {
        clock_hz / (self.prescaler as u32 * (1 + self.seg1 as u32 + self.seg2 as u32))
    }

    /// NBTP: SJW, prescaler, seg1 and seg2, each less one.
    pub(crate) const fn nbtp(self) -> u32 {
        ((self.sjw as u32 - 1) << 25) | ((self.prescaler as u32 - 1) << 16) | ((self.seg1 as u32 - 1) << 8) | (self.seg2 as u32 - 1)
    }

    /// DBTP's timing fields, each less one; transmitter delay compensation
    /// is the caller's.
    pub(crate) const fn dbtp(self) -> u32 {
        ((self.prescaler as u32 - 1) << 16) | ((self.seg1 as u32 - 1) << 8) | ((self.seg2 as u32 - 1) << 4) | (self.sjw as u32 - 1)
    }

    /// Where the secondary sample point falls, in kernel clock cycles from the
    /// start of the bit: the data sample point, the transmitter delay
    /// compensation offset (TDCO) for fast data phases.
    pub(crate) const fn sample_offset(self) -> u32 {
        self.prescaler as u32 * (1 + self.seg1 as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SP_80: SamplePoint = SamplePoint::from_per_mille(800);
    const SP_875: SamplePoint = SamplePoint::from_per_mille(875);

    #[test]
    fn nominal_rates_from_80_mhz() {
        for bitrate in [125_000, 250_000, 500_000, 1_000_000] {
            let timing = BitTiming::solve_nominal(80_000_000, bitrate, SP_875).unwrap();
            assert_eq!(timing.bitrate(80_000_000), bitrate);
            let quanta = 1 + u32::from(timing.seg1()) + u32::from(timing.seg2());
            assert_eq!((1 + u32::from(timing.seg1())) * 1000 / quanta, 875, "{bitrate}");
        }
        // 500 kbit/s: 160 quanta of one cycle each.
        assert_eq!(BitTiming::solve_nominal(80_000_000, 500_000, SP_875).unwrap(), BitTiming::nominal(1, 139, 20, 20).unwrap());
    }

    #[test]
    fn data_rates_from_80_mhz() {
        for bitrate in [1_000_000, 2_000_000, 4_000_000, 5_000_000, 8_000_000] {
            let timing = BitTiming::solve_data(80_000_000, bitrate, SP_80).unwrap();
            assert_eq!(timing.bitrate(80_000_000), bitrate, "{bitrate}");
        }
        // 2 Mbit/s: 40 quanta would need seg1 31 and seg2 8, which fit.
        assert_eq!(BitTiming::solve_data(80_000_000, 2_000_000, SP_80).unwrap(), BitTiming::data(1, 31, 8, 8).unwrap());
        assert_eq!(BitTiming::solve_data(80_000_000, 2_000_000, SP_80).unwrap().sample_offset(), 32);
    }

    #[test]
    fn checks_fields() {
        assert!(BitTiming::nominal(0, 10, 2, 1).is_err());
        assert!(BitTiming::nominal(513, 10, 2, 1).is_err());
        assert!(BitTiming::nominal(1, 257, 2, 1).is_err());
        assert!(BitTiming::data(1, 33, 2, 1).is_err());
        assert!(BitTiming::data(1, 10, 17, 1).is_err());
        assert!(BitTiming::data(1, 10, 2, 3).is_err());
        assert_eq!(BitTiming::nominal(2, 13, 2, 1).unwrap().nbtp(), (1 << 16) | (12 << 8) | 1);
        assert_eq!(BitTiming::data(2, 13, 2, 1).unwrap().dbtp(), (1 << 16) | (12 << 8) | (1 << 4));
    }
}

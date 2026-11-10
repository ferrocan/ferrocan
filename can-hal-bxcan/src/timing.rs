// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Nominal bit timing: the time quanta of a bit and where it is sampled.

use can_hal::SamplePoint;

use crate::Error;

/// A bit's timing in time quanta: one quantum of sync, then `seg1` before the
/// sample point and `seg2` after it, each quantum `prescaler` cycles of the
/// controller's clock (APB1 on the STM32).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BitTiming {
    prescaler: u16,
    seg1: u8,
    seg2: u8,
    sjw: u8,
}

impl BitTiming {
    /// The fewest and most quanta in a bit that bxCAN allows: 1 + 1 + 1 and
    /// 1 + 16 + 8.
    const QUANTA: core::ops::RangeInclusive<u32> = 3..=25;

    /// Timing from its fields, checked against bxCAN's ranges: `prescaler`
    /// 1 to 1024, `seg1` 1 to 16 (propagation and phase 1 together), `seg2` 1
    /// to 8, `sjw` 1 to 4 and at most `seg2`.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidTiming`] when a field is out of range.
    pub const fn new(prescaler: u16, seg1: u8, seg2: u8, sjw: u8) -> Result<Self, Error> {
        if prescaler < 1 || prescaler > 1024 || seg1 < 1 || seg1 > 16 || seg2 < 1 || seg2 > 8 || sjw < 1 || sjw > 4 || sjw > seg2 {
            return Err(Error::InvalidTiming);
        }
        Ok(Self { prescaler, seg1, seg2, sjw })
    }

    /// The timing for `bitrate` from a `clock_hz` clock whose sample point is
    /// nearest `sample_point`, with as many quanta as allow it (finer
    /// resynchronisation), and the widest jump width bxCAN allows.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidTiming`] when no whole number of quanta divides the
    /// clock into `bitrate`.
    pub fn solve(clock_hz: u32, bitrate: u32, sample_point: SamplePoint) -> Result<Self, Error> {
        let target = u32::from(sample_point.per_mille());
        let mut best: Option<(u32, Self)> = None;
        for quanta in Self::QUANTA.rev() {
            let Some(cycles) = bitrate.checked_mul(quanta) else { continue };
            if cycles == 0 || !clock_hz.is_multiple_of(cycles) {
                continue;
            }
            let Ok(prescaler) = u16::try_from(clock_hz / cycles) else { continue };
            // The sample point is at the end of seg1, `1 + seg1` quanta in.
            let sample = (quanta * target + 500) / 1000;
            let (Ok(seg1), Ok(seg2)) = (u8::try_from(sample.saturating_sub(1)), u8::try_from(quanta.saturating_sub(sample))) else {
                continue;
            };
            let Ok(timing) = Self::new(prescaler, seg1, seg2, seg2.min(4)) else { continue };
            let error = (sample * 1000 / quanta).abs_diff(target);
            if best.is_none_or(|(best_error, _)| error < best_error) {
                best = Some((error, timing));
            }
        }
        best.map(|(_, timing)| timing).ok_or(Error::InvalidTiming)
    }

    /// Cycles of the clock per quantum.
    #[must_use]
    pub const fn prescaler(self) -> u16 {
        self.prescaler
    }

    /// Quanta before the sample point, after the sync quantum.
    #[must_use]
    pub const fn seg1(self) -> u8 {
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

    /// The bit rate this timing gives from a `clock_hz` clock.
    #[must_use]
    pub const fn bitrate(self, clock_hz: u32) -> u32 {
        clock_hz / (self.prescaler as u32 * (1 + self.seg1 as u32 + self.seg2 as u32))
    }

    /// The BTR register's timing fields (its mode bits clear).
    pub(crate) const fn btr(self) -> u32 {
        (self.prescaler as u32 - 1) | ((self.seg1 as u32 - 1) << 16) | ((self.seg2 as u32 - 1) << 20) | ((self.sjw as u32 - 1) << 24)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SP_875: SamplePoint = SamplePoint::from_per_mille(875);

    #[test]
    fn solves_common_rates_from_48_mhz() {
        for (bitrate, prescaler) in [(1_000_000, 3), (500_000, 6), (250_000, 12), (125_000, 24)] {
            let timing = BitTiming::solve(48_000_000, bitrate, SP_875).unwrap();
            assert_eq!(timing, BitTiming::new(prescaler, 13, 2, 2).unwrap(), "{bitrate}");
            assert_eq!(timing.bitrate(48_000_000), bitrate);
        }
    }

    #[test]
    fn prefers_more_quanta_at_equal_error() {
        // 8 MHz, 250 kbit/s: 16 quanta at 87.5% (prescaler 2), not 8 (prescaler 4).
        let timing = BitTiming::solve(8_000_000, 250_000, SP_875).unwrap();
        assert_eq!((timing.prescaler(), timing.seg1(), timing.seg2()), (2, 13, 2));
    }

    #[test]
    fn refuses_a_rate_no_prescaler_reaches() {
        assert_eq!(BitTiming::solve(48_000_000, 33_333_333, SP_875), Err(Error::InvalidTiming));
        assert_eq!(BitTiming::solve(48_000_000, 0, SP_875), Err(Error::InvalidTiming));
    }

    #[test]
    fn checks_fields() {
        assert!(BitTiming::new(0, 13, 2, 1).is_err());
        assert!(BitTiming::new(1025, 13, 2, 1).is_err());
        assert!(BitTiming::new(1, 17, 2, 1).is_err());
        assert!(BitTiming::new(1, 13, 9, 1).is_err());
        assert!(BitTiming::new(1, 13, 2, 3).is_err());
        assert_eq!(BitTiming::new(6, 13, 2, 2).unwrap().btr(), 5 | (12 << 16) | (1 << 20) | (1 << 24));
    }
}

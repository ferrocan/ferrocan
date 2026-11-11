// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The board: its clocks, pins and CAN controller. One board is built at a
//! time, chosen by cargo feature.
//!
//! The CAN controller is driven by a can-hal backend, `can-hal-bxcan` on the
//! STM32F0 and `can-hal-fdcan` on the STM32G4; the chip's module does what is
//! chip-specific: its clocks, its pins, its interrupts and its bootloader.

use embassy_stm32::gpio::{Level, Output};

#[cfg(feature = "bxcan")]
mod f072;
#[cfg(feature = "bxcan")]
pub use f072::*;
#[cfg(feature = "fdcan")]
mod g431;
#[cfg(feature = "fdcan")]
pub use g431::*;

/// The boards, of which a build takes exactly one.
const BOARDS: [bool; 5] = [cfg!(feature = "nucleo-f072rb"), cfg!(feature = "disco-f072b"), cfg!(feature = "sh-c30a"), cfg!(feature = "nucleo-g431kb"), cfg!(feature = "sh-c31a")];
const _: () = {
    let mut count = 0;
    let mut i = 0;
    while i < BOARDS.len() {
        if BOARDS[i] {
            count += 1;
        }
        i += 1;
    }
    assert!(count == 1, "build for one board, through its cargo alias, such as `cargo build-disco-f072b`");
};

/// Room for this many CAN events between reads.
pub const CAN_EVENTS: usize = 32;

/// Embassy's time, for the CAN driver's timestamps and timeouts.
pub struct EmbassyClock;

impl EmbassyClock {
    fn deadline_after(timeout: core::time::Duration) -> embassy_time::Instant {
        let micros = u64::try_from(timeout.as_micros()).unwrap_or(u64::MAX);
        embassy_time::Instant::now().saturating_add(embassy_time::Duration::from_micros(micros))
    }
}

/// Implements a backend's `Clock` for [`EmbassyClock`]: each backend has its
/// own trait, of the same shape.
macro_rules! embassy_clock {
    ($backend:ident) => {
        impl $backend::Clock for EmbassyClock {
            type Instant = embassy_time::Instant;

            fn now() -> Self::Instant {
                embassy_time::Instant::now()
            }

            fn deadline(timeout: core::time::Duration) -> Self::Instant {
                Self::deadline_after(timeout)
            }

            fn has_passed(deadline: &Self::Instant) -> bool {
                embassy_time::Instant::now() >= *deadline
            }
        }
    };
}

#[cfg(feature = "bxcan")]
embassy_clock!(can_hal_bxcan);
#[cfg(feature = "fdcan")]
embassy_clock!(can_hal_fdcan);

/// An LED, and the pin level that lights it.
pub struct Led {
    pin: Output<'static>,
    lit: Level,
}

impl Led {
    /// An LED on `pin`, lit by `lit`, starting dark.
    fn new(pin: Output<'static>, lit: Level) -> Self {
        let mut led = Self { pin, lit };
        led.set(false);
        led
    }

    /// Lights the LED, or darkens it.
    pub fn set(&mut self, on: bool) {
        let level = if on { self.lit } else { invert(self.lit) };
        self.pin.set_level(level);
    }

    /// Changes the LED over: lit to dark, or dark to lit.
    pub fn toggle(&mut self) {
        self.pin.toggle();
    }
}

const fn invert(level: Level) -> Level {
    match level {
        Level::Low => Level::High,
        Level::High => Level::Low,
    }
}

/// The board's LEDs: one for frames received, and one for frames sent where
/// the board has two (the one-LED boards show both on `rx`).
pub struct Leds {
    /// Frames received, and the heartbeat.
    pub rx: Led,
    /// Frames sent, if the board has a second LED.
    pub tx: Option<Led>,
}

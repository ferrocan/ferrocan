// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The board: its clocks and pins. One board is built at a time, chosen by
//! cargo feature.

use embassy_stm32::Config;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{Hsi48Config, Sysclk};

#[cfg(not(any(feature = "nucleo-f072rb")))]
compile_error!("Build for a board, through its cargo alias, such as `cargo build-nucleo-f072rb`.");

/// The board's peripherals, as the firmware uses them.
pub struct Board {
    /// The activity LED.
    pub led: Output<'static>,
}

impl Board {
    /// Set up the clocks and take the pins.
    ///
    /// The system runs from the 48 MHz internal oscillator, trimmed from
    /// USB's start-of-frame packets once USB is up, as on crystal-less
    /// adapters such as the SH-C30A.
    #[must_use]
    pub fn init() -> Self {
        let mut config = Config::default();
        let mut hsi48 = Hsi48Config::new();
        hsi48.sync_from_usb = true;
        config.rcc.hsi48 = Some(hsi48);
        config.rcc.sys = Sysclk::HSI48;
        let p = embassy_stm32::init(config);

        Self::take(p)
    }

    /// NUCLEO-F072RB (MB1136): user LED LD2 on PA5.
    #[cfg(feature = "nucleo-f072rb")]
    fn take(p: embassy_stm32::Peripherals) -> Self {
        Self {
            led: Output::new(p.PA5, Level::Low, Speed::Low),
        }
    }
}

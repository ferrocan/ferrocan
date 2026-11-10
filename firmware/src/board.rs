// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The board: its clocks, pins and CAN controller. One board is built at a
//! time, chosen by cargo feature.
//!
//! The CAN controller is driven by `can-hal-bxcan`; the board does what is
//! chip-specific: its clock, its pins and its interrupt.

use can_hal_bxcan::{Clock, Peripheral, State};
use embassy_stm32::Config;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{AfType, Flex, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::interrupt::typelevel::{Binding, CEC_CAN, Handler, Interrupt};
use embassy_stm32::peripherals::CAN;
use embassy_stm32::rcc::{self, Hsi48Config, Sysclk};

#[cfg(not(any(feature = "nucleo-f072rb", feature = "disco-f072b")))]
compile_error!("Build for a board, through its cargo alias, such as `cargo build-disco-f072b`.");
#[cfg(all(feature = "nucleo-f072rb", feature = "disco-f072b"))]
compile_error!("Build for one board at a time.");

/// The CAN controller's clock, APB1: the 48 MHz system clock undivided.
pub const CAN_CLOCK_HZ: u32 = 48_000_000;

/// Room for this many CAN events between reads.
pub const CAN_EVENTS: usize = 32;

/// Embassy's time, for the CAN driver's timestamps and timeouts.
pub struct EmbassyClock;

impl Clock for EmbassyClock {
    type Instant = embassy_time::Instant;

    fn now() -> Self::Instant {
        embassy_time::Instant::now()
    }

    fn deadline(timeout: core::time::Duration) -> Self::Instant {
        let micros = u64::try_from(timeout.as_micros()).unwrap_or(u64::MAX);
        embassy_time::Instant::now().saturating_add(embassy_time::Duration::from_micros(micros))
    }

    fn has_passed(deadline: &Self::Instant) -> bool {
        embassy_time::Instant::now() >= *deadline
    }
}

/// The CAN channel's shared state.
pub static CAN_STATE: State<EmbassyClock, CAN_EVENTS> = State::new();

/// The CAN interrupt, shared on the F0 with HDMI-CEC, which isn't used.
pub struct CanInterrupt;

impl Handler<CEC_CAN> for CanInterrupt {
    unsafe fn on_interrupt() {
        CAN_STATE.on_interrupt();
    }
}

embassy_stm32::bind_interrupts!(struct Irqs {
    CEC_CAN => CanInterrupt;
});

/// The board's peripherals, as the firmware uses them.
pub struct Board {
    /// The activity LED.
    pub led: Output<'static>,
    /// The CAN controller, its clock on and its pins set: give it to
    /// `can_hal_bxcan::Can::new` with [`CAN_STATE`], then
    /// [`unmask_can_interrupt`].
    pub can: Peripheral,
    /// Held so that nothing else takes the controller or its pins.
    _can_parts: (Peri<'static, CAN>, Flex<'static>, Flex<'static>),
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

    /// NUCLEO-F072RB (MB1136): user LED LD2 on PA5; CAN RX on PB8 and TX on
    /// PB9 (Arduino D15 and D14), as CANable-style adapters wire them, which
    /// leaves PA11 and PA12 for USB.
    #[cfg(feature = "nucleo-f072rb")]
    fn take(p: embassy_stm32::Peripherals) -> Self {
        let (can, rx, tx) = can_parts(p.CAN, Flex::new(p.PB8), Flex::new(p.PB9), Irqs);
        Self {
            led: Output::new(p.PA5, Level::Low, Speed::Low),
            // SAFETY: the board holds the controller, so nothing else uses it.
            can: unsafe { Peripheral::new(embassy_stm32::pac::CAN.as_ptr() as usize) },
            _can_parts: (can, rx, tx),
        }
    }

    /// 32F072BDISCOVERY (MB1076): LED LD3 (red) on PC6, its USB USER connector on
    /// PA11 and PA12, no crystal; CAN RX on PB8 and TX on PB9, as on the
    /// Nucleo and CANable-style adapters.
    #[cfg(feature = "disco-f072b")]
    fn take(p: embassy_stm32::Peripherals) -> Self {
        let (can, rx, tx) = can_parts(p.CAN, Flex::new(p.PB8), Flex::new(p.PB9), Irqs);
        Self {
            led: Output::new(p.PC6, Level::Low, Speed::Low),
            // SAFETY: the board holds the controller, so nothing else uses it.
            can: unsafe { Peripheral::new(embassy_stm32::pac::CAN.as_ptr() as usize) },
            _can_parts: (can, rx, tx),
        }
    }
}

/// Turns the controller's clock on and gives it its pins (alternate function
/// 4 on PB8 and PB9). The RX pin is pulled up, recessive, so the controller
/// can join an idle bus even with no transceiver attached.
fn can_parts(can: Peri<'static, CAN>, mut rx: Flex<'static>, mut tx: Flex<'static>, _irqs: impl Binding<CEC_CAN, CanInterrupt>) -> (Peri<'static, CAN>, Flex<'static>, Flex<'static>) {
    rcc::enable_and_reset::<CAN>();
    rx.set_as_af_unchecked(4, AfType::input(Pull::Up));
    tx.set_as_af_unchecked(4, AfType::output(OutputType::PushPull, Speed::VeryHigh));
    (can, rx, tx)
}

/// Lets the CAN controller's interrupt through, once its channel is made.
pub fn unmask_can_interrupt() {
    CEC_CAN::unpend();
    // SAFETY: the handler only touches CAN_STATE, which is ready once the
    // channel is made.
    unsafe { CEC_CAN::enable() };
}

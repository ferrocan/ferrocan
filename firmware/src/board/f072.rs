// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The STM32F072 boards: bxCAN, driven by `can-hal-bxcan`.

use can_hal::SamplePoint;
use can_hal_bxcan::{BitTiming, Can, Config, Peripheral, State};
use embassy_stm32::Peri;
use embassy_stm32::gpio::{AfType, Flex, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::interrupt::typelevel::{Binding, CEC_CAN, Handler, Interrupt};
use embassy_stm32::peripherals::{CAN, USB};
use embassy_stm32::rcc::{self, Hsi48Config, Sysclk};
use embassy_stm32::usb;

use super::{CAN_EVENTS, EmbassyClock, Led, Leds};

/// The ROM bootloader: the `STM32F07x`'s system memory, its vector table
/// first (ST's `AN2606`).
pub const BOOTLOADER: usize = 0x1FFF_C800;

/// The CAN controller's clock, APB1: the 48 MHz system clock undivided.
pub const CAN_CLOCK_HZ: u32 = 48_000_000;

/// The CAN channel, as this firmware makes it.
pub type CanChannel = Can<EmbassyClock, CAN_EVENTS>;

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
    USB => usb::InterruptHandler<USB>;
});

/// The board's peripherals, as the firmware uses them.
pub struct Board {
    /// The LEDs.
    pub leds: Leds,
    /// The CAN channel, stopped, its interrupt unmasked.
    pub can: CanChannel,
    /// The USB device controller, on PA11 and PA12.
    pub usb: usb::Driver<'static, USB>,
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
        let mut config = embassy_stm32::Config::default();
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
            leds: Leds {
                rx: Led::new(Output::new(p.PA5, Level::Low, Speed::Low), Level::High),
                tx: None,
            },
            can: channel(),
            usb: usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11),
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
            leds: Leds {
                rx: Led::new(Output::new(p.PC6, Level::Low, Speed::Low), Level::High),
                tx: None,
            },
            can: channel(),
            usb: usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11),
            _can_parts: (can, rx, tx),
        }
    }

    /// DSD TECH SH-C30A, a `CANable` V1.1 (its schematic, 2022-09-26): an
    /// `STM32F072CB` with a crystal footprint (X1) that is likely unfitted, so
    /// the internal oscillator; LEDs on PB0 (TXD) and PB1 (RXD), lit low
    /// through 1 kΩ from 3.3 V, as upstream candleLight's `DSD_TECH_SH_C30A`
    /// board has them; CAN on PB8 (RXD) and PB9 (TXD) to its transceiver,
    /// whose standby pin is tied low. SW1 is two switches: the 120 Ω
    /// termination, and BOOT0 for the ROM bootloader. A 5-pin header (H1)
    /// has SWD. The RX LED shows frames received, the TX LED frames sent.
    #[cfg(feature = "sh-c30a")]
    fn take(p: embassy_stm32::Peripherals) -> Self {
        let (can, rx, tx) = can_parts(p.CAN, Flex::new(p.PB8), Flex::new(p.PB9), Irqs);
        Self {
            leds: Leds {
                rx: Led::new(Output::new(p.PB1, Level::High, Speed::Low), Level::Low),
                tx: Some(Led::new(Output::new(p.PB0, Level::High, Speed::Low), Level::Low)),
            },
            can: channel(),
            usb: usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11),
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

/// The channel, stopped until the host starts it, its interrupt unmasked.
/// Its clock must be on.
fn channel() -> CanChannel {
    // SAFETY: the board holds the controller, so nothing else uses it.
    let peripheral = unsafe { Peripheral::new(embassy_stm32::pac::CAN.as_ptr() as usize) };
    let timing = BitTiming::solve(CAN_CLOCK_HZ, 500_000, SamplePoint::from_per_mille(875)).expect("500 kbit/s from 48 MHz");
    let can = Can::new(peripheral, &CAN_STATE, Config::new(timing)).expect("the controller enters initialisation");
    CEC_CAN::unpend();
    // SAFETY: the handler only touches CAN_STATE, which is ready now that
    // the channel is made.
    unsafe { CEC_CAN::enable() };
    can
}

/// The registers that tell whether the firmware started straight out of a
/// reset, as it finds them before it sets anything up. The ROM bootloader
/// starts new firmware (`dfu-util`'s `:leave`) by jumping to it, leaving the
/// chip as it had it.
#[derive(Clone, Copy)]
pub struct StartState {
    pub rcc_cr: u32,
    pub rcc_cfgr: u32,
    pub rcc_csr: u32,
    pub syscfg_cfgr1: u32,
}

/// `RCC_CSR`'s reset flags: low-power, window and independent watchdog,
/// software, power-on, pin and option byte loader.
const RESET_FLAGS: u32 = 0x7F << 25;
/// `RCC_CSR`'s remove-flags bit.
const RMVF: u32 = 1 << 24;
/// HSITRIM's value out of reset.
const HSITRIM_DEFAULT: u32 = 16;

impl StartState {
    /// The registers as they are. A register whose peripheral has no clock
    /// reads 0, as after a reset.
    #[must_use]
    pub fn read() -> Self {
        use embassy_stm32::pac::{RCC, SYSCFG};
        let csr = (RCC.as_ptr() as usize + 0x24) as *const u32;
        Self {
            rcc_cr: RCC.cr().read().0,
            rcc_cfgr: RCC.cfgr().read().0,
            // SAFETY: RCC_CSR; reading it has no effect.
            rcc_csr: unsafe { csr.read_volatile() },
            syscfg_cfgr1: SYSCFG.cfgr1().read().0,
        }
    }

    /// Whether the firmware didn't start out of a reset: no reset flag set
    /// (the firmware clears them as it starts, see [`clear_reset_flags`]),
    /// or the chip not as a reset leaves it, for when the bootloader was
    /// itself entered from a reset: the PLL running, the system clock on
    /// something other than HSI, system memory mapped at 0 (where the
    /// Cortex-M0, with no vector table offset register, would take the
    /// bootloader's interrupt vectors), or HSI trimmed.
    #[must_use]
    pub const fn not_out_of_reset(&self) -> bool {
        let no_reset = self.rcc_csr & RESET_FLAGS == 0;
        let pll_on = self.rcc_cr & (1 << 24) != 0;
        let not_hsi = (self.rcc_cfgr >> 2) & 0b11 != 0b00;
        let system_memory = self.syscfg_cfgr1 & 0b11 != 0;
        let trimmed = (self.rcc_cr >> 3) & 0x1F != HSITRIM_DEFAULT;
        no_reset || pll_on || not_hsi || system_memory || trimmed
    }
}

/// Clears the reset flags, so that a start without a reset finds none.
pub fn clear_reset_flags() {
    let csr = (embassy_stm32::pac::RCC.as_ptr() as usize + 0x24) as *mut u32;
    // SAFETY: RCC_CSR; RMVF clears the reset flags and nothing else.
    unsafe { csr.write_volatile(csr.read_volatile() | RMVF) };
}

impl defmt::Format for StartState {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RCC_CR {=u32:#010x} RCC_CFGR {=u32:#010x} RCC_CSR {=u32:#010x} SYSCFG_CFGR1 {=u32:#010x}",
            self.rcc_cr,
            self.rcc_cfgr,
            self.rcc_csr,
            self.syscfg_cfgr1
        );
    }
}

/// Maps system memory at address 0, for the ROM bootloader: the Cortex-M0
/// has no vector table offset register, so its vectors must be there, or the
/// bootloader's interrupts would run this firmware's handlers.
pub fn map_system_memory() {
    embassy_stm32::pac::RCC.apb2enr().modify(|w| w.set_syscfgen(true));
    embassy_stm32::pac::SYSCFG.cfgr1().modify(|w| w.set_mem_mode(embassy_stm32::pac::syscfg::vals::MemMode::SYSTEM_FLASH));
}

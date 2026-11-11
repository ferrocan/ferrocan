// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The STM32G431 boards: FDCAN, driven by `can-hal-mcan`.

use can_hal::SamplePoint;
use can_hal_mcan::{BitTiming, Can, Config, Peripheral, State};
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Flex, Level, Output, Speed};
use embassy_stm32::interrupt::typelevel::{FDCAN1_IT0, Handler, Interrupt};
use embassy_stm32::peripherals::{FDCAN1, USB};
use embassy_stm32::rcc::{self, Hsi48Config, Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::usb;

use super::{CAN_EVENTS, EmbassyClock, Led, Leds};

/// The ROM bootloader: the `STM32G431`'s system memory (ST's `AN2606`).
pub const BOOTLOADER: usize = 0x1FFF_0000;

/// FDCAN1's kernel clock: PLL Q, 80 MHz.
pub const CAN_CLOCK_HZ: u32 = 80_000_000;

/// The CAN channel, as this firmware makes it.
pub type CanChannel = Can<EmbassyClock, CAN_EVENTS>;

/// The CAN channel's shared state.
pub static CAN_STATE: State<EmbassyClock, CAN_EVENTS> = State::new();

/// FDCAN1's interrupt line 0, where the channel routes all its interrupts.
pub struct CanInterrupt;

impl Handler<FDCAN1_IT0> for CanInterrupt {
    unsafe fn on_interrupt() {
        CAN_STATE.on_interrupt();
    }
}

embassy_stm32::bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => CanInterrupt;
    USB_LP => usb::InterruptHandler<USB>;
});

/// The board's peripherals, as the firmware uses them.
pub struct Board {
    /// The LEDs.
    pub leds: Leds,
    /// The CAN channel, stopped, its interrupt unmasked.
    pub can: CanChannel,
    /// The USB device controller, on PA11 and PA12.
    pub usb: usb::Driver<'static, USB>,
    /// Held so that nothing else takes the controller or its pins, which
    /// return to analog when dropped. A board with no CAN pins has none.
    _can_parts: (Peri<'static, FDCAN1>, Option<(Flex<'static>, Flex<'static>)>),
}

impl Board {
    /// Set up the clocks and take the pins.
    ///
    /// The 16 MHz internal oscillator drives the PLL: 160 MHz for the system,
    /// 80 MHz for FDCAN, whose bit timings then divide evenly from 125 kbit/s
    /// to 8 Mbit/s. USB runs from the 48 MHz internal oscillator, trimmed from
    /// its start-of-frame packets, as on crystal-less adapters such as the
    /// SH-C31A.
    #[must_use]
    pub fn init() -> Self {
        let mut config = embassy_stm32::Config::default();
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL80,
            divp: None,
            divq: Some(PllQDiv::DIV4),
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.boost = true;
        let mut hsi48 = Hsi48Config::new();
        hsi48.sync_from_usb = true;
        config.rcc.hsi48 = Some(hsi48);
        config.rcc.mux.clk48sel = rcc::mux::Clk48sel::HSI48;
        config.rcc.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q;
        let p = embassy_stm32::init(config);

        Self::take(p)
    }

    /// NUCLEO-G431KB (MB1430): user LED LD2 on PB8. It has no USB connector
    /// (PA11 and PA12 go to the headers) and FDCAN1's pins aren't brought out
    /// apart from PA11 and PA12, so the channel can only run in internal
    /// loopback: the board is for running the firmware under the probe.
    #[cfg(feature = "nucleo-g431kb")]
    fn take(p: embassy_stm32::Peripherals) -> Self {
        Self {
            leds: Leds {
                rx: Led::new(Output::new(p.PB8, Level::Low, Speed::Low), Level::High),
                tx: None,
            },
            can: channel(),
            usb: usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11),
            _can_parts: (p.FDCAN1, None),
        }
    }

    /// DSD TECH SH-C31A V1.1, a `CANable` 2.0 (its schematic, in
    /// dsdtech-official/can-adapters): an `STM32G431C8T6` with no crystal;
    /// LEDs on PA15 (TXD) and PA0 (RXD), lit low through 1 kΩ from 3.3 V;
    /// CAN on PB8 (RXD) and PB9 (TXD) to an SIT1051T transceiver (TJA1051T/3
    /// in the schematic), its standby pin tied low and its I/O supply
    /// switched on a moment after power-up. SW1 is two switches: the 120 Ω
    /// termination, and BOOT, which ties PB8 (BOOT0, pulled down by 10 kΩ) to
    /// 3.3 V for the ROM bootloader. A 5-pin header (H1) has SWD.
    #[cfg(feature = "sh-c31a")]
    fn take(p: embassy_stm32::Peripherals) -> Self {
        use embassy_stm32::gpio::{AfType, OutputType, Pull};

        // FDCAN1 on alternate function 9. RX has no pull: the transceiver
        // drives it, against the BOOT0 pull-down.
        let mut rx = Flex::new(p.PB8);
        let mut tx = Flex::new(p.PB9);
        rx.set_as_af_unchecked(9, AfType::input(Pull::None));
        tx.set_as_af_unchecked(9, AfType::output(OutputType::PushPull, Speed::VeryHigh));
        Self {
            leds: Leds {
                rx: Led::new(Output::new(p.PA0, Level::High, Speed::Low), Level::Low),
                tx: Some(Led::new(Output::new(p.PA15, Level::High, Speed::Low), Level::Low)),
            },
            can: channel(),
            usb: usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11),
            _can_parts: (p.FDCAN1, Some((rx, tx))),
        }
    }
}

/// Turns FDCAN1's clock on and makes the channel, stopped until the host
/// starts it, its interrupt unmasked.
fn channel() -> CanChannel {
    rcc::enable_and_reset::<FDCAN1>();
    // SAFETY: FDCAN1's registers and message RAM; the board holds the
    // controller, so nothing else uses it.
    let peripheral = unsafe { Peripheral::new(embassy_stm32::pac::FDCAN1.as_ptr() as usize, embassy_stm32::pac::FDCANRAM1.as_ptr() as usize) };
    let timing = BitTiming::solve_nominal(CAN_CLOCK_HZ, 500_000, SamplePoint::from_per_mille(875)).expect("500 kbit/s from 80 MHz");
    let can = Can::new(peripheral, &CAN_STATE, Config::new(timing)).expect("the controller enters initialisation");
    FDCAN1_IT0::unpend();
    // SAFETY: the handler only touches CAN_STATE, which is ready now that
    // the channel is made.
    unsafe { FDCAN1_IT0::enable() };
    can
}

/// The registers that tell whether the firmware started straight out of a
/// reset, as it finds them before it sets anything up. The ROM bootloader
/// starts new firmware (`dfu-util`'s `:leave`) by jumping to it, leaving the
/// chip as it had it: HSI16 trimmed away from its default, for one, which
/// puts every clock off, CAN's bit rate with them.
#[derive(Clone, Copy)]
pub struct StartState {
    pub rcc_cr: u32,
    pub rcc_icscr: u32,
    pub rcc_cfgr: u32,
    pub rcc_csr: u32,
    pub syscfg_memrmp: u32,
}

/// `RCC_CSR`'s reset flags: low-power, window and independent watchdog,
/// software, brown-out, pin and option byte loader.
const RESET_FLAGS: u32 = 0x7F << 25;
/// `RCC_CSR`'s remove-flags bit.
const RMVF: u32 = 1 << 23;
/// HSITRIM's value out of reset.
const HSITRIM_DEFAULT: u32 = 64;

impl StartState {
    /// The registers as they are. A register whose peripheral has no clock
    /// reads 0, as after a reset.
    #[must_use]
    pub fn read() -> Self {
        use embassy_stm32::pac::{RCC, SYSCFG};
        let rcc = RCC.as_ptr() as usize;
        // SAFETY: RCC_ICSCR and RCC_CSR; reading them has no effect.
        let (icscr, csr) = unsafe { (((rcc + 0x04) as *const u32).read_volatile(), ((rcc + 0x94) as *const u32).read_volatile()) };
        Self {
            rcc_cr: RCC.cr().read().0,
            rcc_icscr: icscr,
            rcc_cfgr: RCC.cfgr().read().0,
            rcc_csr: csr,
            syscfg_memrmp: SYSCFG.memrmp().read().0,
        }
    }

    /// Whether the firmware didn't start out of a reset: no reset flag set
    /// (the firmware clears them as it starts, see [`clear_reset_flags`]),
    /// or the chip not as a reset leaves it, for when the bootloader was
    /// itself entered from a reset: the PLL running, the system clock on
    /// something other than HSI16, system memory mapped at 0, or HSI16
    /// trimmed.
    #[must_use]
    pub const fn not_out_of_reset(&self) -> bool {
        let no_reset = self.rcc_csr & RESET_FLAGS == 0;
        let pll_on = self.rcc_cr & (1 << 24) != 0;
        let not_hsi16 = (self.rcc_cfgr >> 2) & 0b11 != 0b01;
        let system_memory = self.syscfg_memrmp & 0b111 != 0;
        let trimmed = (self.rcc_icscr >> 24) & 0x7F != HSITRIM_DEFAULT;
        no_reset || pll_on || not_hsi16 || system_memory || trimmed
    }
}

/// Clears the reset flags, so that a start without a reset finds none.
pub fn clear_reset_flags() {
    let csr = (embassy_stm32::pac::RCC.as_ptr() as usize + 0x94) as *mut u32;
    // SAFETY: RCC_CSR; RMVF clears the reset flags and nothing else.
    unsafe { csr.write_volatile(csr.read_volatile() | RMVF) };
}

impl defmt::Format for StartState {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RCC_CR {=u32:#010x} RCC_ICSCR {=u32:#010x} RCC_CFGR {=u32:#010x} RCC_CSR {=u32:#010x} SYSCFG_MEMRMP {=u32:#010x}",
            self.rcc_cr,
            self.rcc_icscr,
            self.rcc_cfgr,
            self.rcc_csr,
            self.syscfg_memrmp
        );
    }
}

/// Maps system memory at address 0, for the ROM bootloader, as ST advises
/// before jumping to it.
pub fn map_system_memory() {
    embassy_stm32::pac::RCC.apb2enr().modify(|w| w.set_syscfgen(true));
    // MEM_MODE 1: system flash.
    embassy_stm32::pac::SYSCFG.memrmp().modify(|w| w.set_mem_mode(1));
}

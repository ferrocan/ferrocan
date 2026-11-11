// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! DFU detach: a DFU runtime interface beside the `gs_usb` one, so that
//! `dfu-util -e` restarts the device in the chip's ROM bootloader (ST's DFU,
//! `0483:df11`), which then takes new firmware. This is how candleLight
//! reflashes without its boot switch.
//!
//! The request to the bootloader survives a reset in RAM that the runtime
//! leaves uninitialised; [`enter_bootloader_if_requested`], first thing in
//! `main`, jumps there before any peripheral is set up.
//!
//! The other way, the bootloader starts new firmware (`dfu-util`'s `:leave`)
//! by jumping to it, not by a reset, so the firmware inherits whatever the
//! bootloader's de-initialisation leaves; `AN2606` says it should reset all
//! it used, but "occasionally, not all reset values are set due to errors".
//! The STM32G431's leaves HSI16 trimmed, which the board's init puts right
//! (`restore_hsi_trim`); the rest is trusted to the bootloader.
//!
//! That trust is the maintainer's deliberate choice (2026-11-11, keeping it
//! simple), over restarting with a system reset whenever the firmware finds
//! it didn't start out of one (no reset flag set, or the PLL, system clock,
//! memory map or trim not as a reset leaves them), which commit bde038c did
//! and the next commit took out. If a bootloader version is found to leave
//! something else, bring that back.

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use cortex_m::peripheral::SCB;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_usb::Handler;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::types::InterfaceNumber;

use crate::board::BOOTLOADER;

/// The DFU class (application specific), subclass (DFU) and protocol
/// (runtime) of the interface.
pub const CLASS: u8 = 0xFE;
pub const SUBCLASS: u8 = 0x01;
pub const PROTOCOL: u8 = 0x01;

/// The DFU functional descriptor's type, and its contents after the length
/// and type: will detach, can upload and download (`0x0B`); a 255 ms detach
/// timeout; 2048-byte transfers; DFU 1.1a. As candleLight has it.
pub const FUNCTIONAL_DESCRIPTOR_TYPE: u8 = 0x21;
pub const FUNCTIONAL_DESCRIPTOR: [u8; 7] = [0x0B, 0xFF, 0x00, 0x00, 0x08, 0x1A, 0x01];

/// The DFU requests a runtime interface answers.
const DFU_DETACH: u8 = 0;
const DFU_GETSTATUS: u8 = 3;
const DFU_GETSTATE: u8 = 5;

/// DFU's appIDLE state.
const APP_IDLE: u8 = 0;

/// Set by the host's `DFU_DETACH`; the firmware then restarts in the
/// bootloader, once the request has been acknowledged.
pub static DETACH: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The value in [`REQUEST`] that asks for the bootloader; anything else, as
/// RAM holds after power-on, doesn't.
const BOOTLOADER_REQUESTED: u32 = 0x4446_5521;

#[unsafe(link_section = ".uninit.FERROCAN_DFU")]
static mut REQUEST: MaybeUninit<u32> = MaybeUninit::uninit();

/// Restarts the device in the ROM bootloader.
pub fn reboot_into_bootloader() -> ! {
    // SAFETY: nothing else touches REQUEST, and it is plain RAM.
    unsafe { addr_of_mut!(REQUEST).cast::<u32>().write_volatile(BOOTLOADER_REQUESTED) };
    SCB::sys_reset()
}

/// On a panic, restarts in the ROM bootloader rather than halting: on a board
/// with no debug probe, a firmware that panics at start would otherwise
/// leave nothing on USB, and only the boot switch would bring it back. In
/// the bootloader it shows as ST's DFU device (`0483:df11`), ready for
/// `dfu-util`. A power cycle runs the firmware again.
#[cfg(feature = "panic-dfu")]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic: {}, restarting in the bootloader", defmt::Display2Format(info));
    reboot_into_bootloader()
}

/// If the last reset was [`reboot_into_bootloader`]'s, jumps to the ROM
/// bootloader. Call it first in `main`, before any peripheral is set up, so
/// that the bootloader starts from the chip's reset state.
pub fn enter_bootloader_if_requested() {
    let request = addr_of_mut!(REQUEST).cast::<u32>();
    // SAFETY: REQUEST is RAM that the runtime leaves as the last reset found
    // it: after power-on, arbitrary bits, which are a valid u32 that is very
    // unlikely to be the request.
    if unsafe { request.read_volatile() } == BOOTLOADER_REQUESTED {
        // SAFETY: as above; cleared so that the next reset runs the firmware.
        unsafe { request.write_volatile(0) };
        crate::board::map_system_memory();
        // SAFETY: BOOTLOADER is the chip's system memory, whose first words
        // are the bootloader's stack pointer and reset vector; nothing has
        // been set up that it would mind.
        unsafe { cortex_m::asm::bootload(BOOTLOADER as *const u32) }
    }
}

/// Answers the DFU runtime requests to its interface.
pub struct Dfu {
    interface: InterfaceNumber,
}

impl Dfu {
    /// A handler for interface 1, until [`set_interface`](Self::set_interface)
    /// says which it is.
    #[must_use]
    pub const fn new() -> Self {
        Self { interface: InterfaceNumber(1) }
    }

    /// The interface's number, as the USB builder gave it.
    pub const fn set_interface(&mut self, interface: InterfaceNumber) {
        self.interface = interface;
    }

    fn is_mine(&self, req: &Request) -> bool {
        req.request_type == RequestType::Class && req.recipient == Recipient::Interface && req.index == u16::from(self.interface.0)
    }
}

impl Default for Dfu {
    fn default() -> Self {
        Self::new()
    }
}

impl Handler for Dfu {
    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if !self.is_mine(&req) {
            return None;
        }
        Some(if req.request == DFU_DETACH {
            defmt::info!("dfu: detach, restarting in the bootloader");
            DETACH.signal(());
            OutResponse::Accepted
        } else {
            OutResponse::Rejected
        })
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if !self.is_mine(&req) {
            return None;
        }
        Some(match req.request {
            DFU_GETSTATUS => {
                // OK, no poll timeout, appIDLE, no status string.
                buf[..6].copy_from_slice(&[0, 0, 0, 0, APP_IDLE, 0]);
                InResponse::Accepted(&buf[..6])
            }
            DFU_GETSTATE => {
                buf[0] = APP_IDLE;
                InResponse::Accepted(&buf[..1])
            }
            _ => InResponse::Rejected,
        })
    }
}

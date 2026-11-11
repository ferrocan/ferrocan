// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! ST's FDCAN, the `M_CAN` variant of the STM32G0, G4, L5, U5 and H5: its
//! registers (RM0440 for the G4; checked against stm32-metapac's
//! `can_fdcan_v1`, generated from ST's SVD) and its message RAM, whose
//! layout is fixed. The full Bosch `M_CAN` (the STM32H7, Microchip's parts)
//! places its message RAM by register instead and numbers some interrupts
//! differently; it would be a second variant.

use core::cell::UnsafeCell;
use core::ptr::NonNull;

/// One 32-bit memory-mapped register, or message RAM word.
#[repr(transparent)]
pub(crate) struct Reg(UnsafeCell<u32>);

impl Reg {
    pub(crate) fn read(&self) -> u32 {
        // SAFETY: memory-mapped I/O, valid for as long as its block is.
        unsafe { self.0.get().read_volatile() }
    }

    pub(crate) fn write(&self, value: u32) {
        // SAFETY: as for `read`.
        unsafe { self.0.get().write_volatile(value) }
    }

    pub(crate) fn modify(&self, f: impl FnOnce(u32) -> u32) {
        self.write(f(self.read()));
    }
}

/// The FDCAN register block.
#[repr(C)]
pub(crate) struct RegisterBlock {
    pub(crate) crel: Reg,
    pub(crate) endn: Reg,
    _reserved0: Reg,
    pub(crate) dbtp: Reg,
    pub(crate) test: Reg,
    pub(crate) rwd: Reg,
    pub(crate) cccr: Reg,
    pub(crate) nbtp: Reg,
    pub(crate) tscc: Reg,
    pub(crate) tscv: Reg,
    pub(crate) tocc: Reg,
    pub(crate) tocv: Reg,
    _reserved1: [Reg; 4],
    pub(crate) ecr: Reg,
    pub(crate) psr: Reg,
    pub(crate) tdcr: Reg,
    _reserved2: Reg,
    pub(crate) ir: Reg,
    pub(crate) ie: Reg,
    pub(crate) ils: Reg,
    pub(crate) ile: Reg,
    _reserved3: [Reg; 8],
    pub(crate) rxgfc: Reg,
    pub(crate) xidam: Reg,
    pub(crate) hpms: Reg,
    _reserved4: Reg,
    /// RXF0S and RXF0A, then RXF1S and RXF1A.
    pub(crate) rxf: [RxFifoRegs; 2],
    _reserved5: [Reg; 8],
    pub(crate) txbc: Reg,
    pub(crate) txfqs: Reg,
    pub(crate) txbrp: Reg,
    pub(crate) txbar: Reg,
    pub(crate) txbcr: Reg,
    pub(crate) txbto: Reg,
    pub(crate) txbcf: Reg,
    pub(crate) txbtie: Reg,
    pub(crate) txbcie: Reg,
    pub(crate) txefs: Reg,
    pub(crate) txefa: Reg,
}

/// A receive FIFO's status and acknowledge registers.
#[repr(C)]
pub(crate) struct RxFifoRegs {
    pub(crate) status: Reg,
    pub(crate) ack: Reg,
}

const _: () = {
    assert!(core::mem::offset_of!(RegisterBlock, dbtp) == 0x0C);
    assert!(core::mem::offset_of!(RegisterBlock, cccr) == 0x18);
    assert!(core::mem::offset_of!(RegisterBlock, ecr) == 0x40);
    assert!(core::mem::offset_of!(RegisterBlock, ir) == 0x50);
    assert!(core::mem::offset_of!(RegisterBlock, ile) == 0x5C);
    assert!(core::mem::offset_of!(RegisterBlock, rxgfc) == 0x80);
    assert!(core::mem::offset_of!(RegisterBlock, rxf) == 0x90);
    assert!(core::mem::offset_of!(RegisterBlock, txbc) == 0xC0);
    assert!(core::mem::offset_of!(RegisterBlock, txefa) == 0xE8);
};

/// The message RAM, in words: 28 standard and 8 extended filters, two
/// receive FIFOs of three 18-word elements, three Tx event elements of two
/// words, and three 18-word transmit buffers.
#[repr(C)]
pub(crate) struct MessageRam {
    pub(crate) standard_filters: [Reg; 28],
    pub(crate) extended_filters: [Reg; 16],
    pub(crate) rx_fifo: [[Element; 3]; 2],
    pub(crate) tx_events: [[Reg; 2]; 3],
    pub(crate) tx_buffers: [Element; 3],
}

/// A receive or transmit element: two header words and 64 bytes of data.
#[repr(C)]
pub(crate) struct Element {
    pub(crate) header: [Reg; 2],
    pub(crate) data: [Reg; 16],
}

const _: () = assert!(core::mem::size_of::<MessageRam>() == 212 * 4);

/// Pointers to a controller's registers and message RAM.
#[derive(Clone, Copy)]
pub(crate) struct Regs {
    regs: NonNull<RegisterBlock>,
    ram: NonNull<MessageRam>,
}

// SAFETY: the registers are only touched with the critical section held, or
// by the one channel that owns them.
unsafe impl Send for Regs {}

impl Regs {
    /// # Safety
    ///
    /// `regs` and `ram` must be one FDCAN's register block and message RAM.
    pub(crate) const unsafe fn new(regs: usize, ram: usize) -> Self {
        // SAFETY: the caller gives their addresses, which are never null.
        unsafe {
            Self {
                regs: NonNull::new_unchecked(regs as *mut RegisterBlock),
                ram: NonNull::new_unchecked(ram as *mut MessageRam),
            }
        }
    }

    pub(crate) fn get(&self) -> &RegisterBlock {
        // SAFETY: a register block (see `new`), which lives for as long as the
        // program.
        unsafe { self.regs.as_ref() }
    }

    pub(crate) fn ram(&self) -> &MessageRam {
        // SAFETY: as for `get`.
        unsafe { self.ram.as_ref() }
    }
}

// CCCR: control.
pub(crate) const CCCR_INIT: u32 = 1 << 0;
pub(crate) const CCCR_CCE: u32 = 1 << 1;
pub(crate) const CCCR_MON: u32 = 1 << 5;
pub(crate) const CCCR_DAR: u32 = 1 << 6;
pub(crate) const CCCR_TEST: u32 = 1 << 7;
pub(crate) const CCCR_FDOE: u32 = 1 << 8;
pub(crate) const CCCR_BRSE: u32 = 1 << 9;

// TEST.
pub(crate) const TEST_LBCK: u32 = 1 << 4;

// DBTP: data bit timing.
pub(crate) const DBTP_TDC: u32 = 1 << 23;

// PSR: protocol status.
pub(crate) const PSR_EP: u32 = 1 << 5;
pub(crate) const PSR_BO: u32 = 1 << 7;
/// ACT: 0 while synchronising, then idle, receiving or transmitting.
pub(crate) const PSR_ACT_MASK: u32 = 0b11 << 3;
pub(crate) const PSR_LEC_SHIFT: u32 = 0;
pub(crate) const PSR_DLEC_SHIFT: u32 = 8;

// TSCC: timestamp counter, counting bit times.
pub(crate) const TSCC_TSS_INTERNAL: u32 = 0b01;

// IR / IE: interrupts (ST's numbering).
pub(crate) const IR_RF0N: u32 = 1 << 0;
pub(crate) const IR_RF0L: u32 = 1 << 2;
pub(crate) const IR_TCF: u32 = 1 << 8;
pub(crate) const IR_TEFN: u32 = 1 << 10;
pub(crate) const IR_EP: u32 = 1 << 17;
pub(crate) const IR_BO: u32 = 1 << 19;
pub(crate) const IR_PEA: u32 = 1 << 21;
pub(crate) const IR_PED: u32 = 1 << 22;

// ILE: interrupt line enable.
pub(crate) const ILE_EINT0: u32 = 1 << 0;

// RXFnS: receive FIFO status.
pub(crate) const RXFS_FFL_MASK: u32 = 0xF;
pub(crate) const RXFS_FGI_SHIFT: u32 = 8;

// TXFQS: transmit FIFO status.
pub(crate) const TXFQS_TFQPI_SHIFT: u32 = 16;
pub(crate) const TXFQS_TFQF: u32 = 1 << 21;

// TXEFS: Tx event FIFO status.
pub(crate) const TXEFS_EFFL_MASK: u32 = 0x7;
pub(crate) const TXEFS_EFGI_SHIFT: u32 = 8;

// Element header bits: word 0 (T0, R0, E0).
pub(crate) const H0_RTR: u32 = 1 << 29;
pub(crate) const H0_XTD: u32 = 1 << 30;
pub(crate) const H0_ESI: u32 = 1 << 31;
// Word 1 (T1, R1, E1).
pub(crate) const H1_DLC_SHIFT: u32 = 16;
pub(crate) const H1_BRS: u32 = 1 << 20;
pub(crate) const H1_FDF: u32 = 1 << 21;
pub(crate) const T1_EFC: u32 = 1 << 23;
pub(crate) const H1_MM_SHIFT: u32 = 24;

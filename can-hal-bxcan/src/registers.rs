// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The bxCAN register block and its bits, from ST's reference manuals (RM0091
//! for the STM32F0); the layout is the same on every STM32 with bxCAN.

use core::cell::UnsafeCell;
use core::ptr::NonNull;

/// One 32-bit memory-mapped register.
#[repr(transparent)]
pub(crate) struct Reg(UnsafeCell<u32>);

impl Reg {
    pub(crate) fn read(&self) -> u32 {
        // SAFETY: the register is memory-mapped I/O, valid for the lifetime of
        // the register block it is part of.
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

/// A transmit mailbox.
#[repr(C)]
pub(crate) struct TxMailbox {
    pub(crate) tir: Reg,
    pub(crate) tdtr: Reg,
    pub(crate) tdlr: Reg,
    pub(crate) tdhr: Reg,
}

/// The output mailbox of a receive FIFO.
#[repr(C)]
pub(crate) struct RxMailbox {
    pub(crate) rir: Reg,
    pub(crate) rdtr: Reg,
    pub(crate) rdlr: Reg,
    pub(crate) rdhr: Reg,
}

/// A filter bank's two registers.
#[repr(C)]
pub(crate) struct FilterBank {
    pub(crate) fr1: Reg,
    pub(crate) fr2: Reg,
}

/// The bxCAN register block.
#[repr(C)]
pub(crate) struct RegisterBlock {
    pub(crate) mcr: Reg,
    pub(crate) msr: Reg,
    pub(crate) tsr: Reg,
    pub(crate) rfr: [Reg; 2],
    pub(crate) ier: Reg,
    pub(crate) esr: Reg,
    pub(crate) btr: Reg,
    _reserved0: [Reg; 88],
    pub(crate) tx: [TxMailbox; 3],
    pub(crate) rx: [RxMailbox; 2],
    _reserved1: [Reg; 12],
    pub(crate) fmr: Reg,
    pub(crate) fm1r: Reg,
    _reserved2: Reg,
    pub(crate) fs1r: Reg,
    _reserved3: Reg,
    pub(crate) ffa1r: Reg,
    _reserved4: Reg,
    pub(crate) fa1r: Reg,
    _reserved5: [Reg; 8],
    pub(crate) filter: [FilterBank; 28],
}

const _: () = {
    assert!(core::mem::offset_of!(RegisterBlock, btr) == 0x1C);
    assert!(core::mem::offset_of!(RegisterBlock, tx) == 0x180);
    assert!(core::mem::offset_of!(RegisterBlock, rx) == 0x1B0);
    assert!(core::mem::offset_of!(RegisterBlock, fmr) == 0x200);
    assert!(core::mem::offset_of!(RegisterBlock, fa1r) == 0x21C);
    assert!(core::mem::offset_of!(RegisterBlock, filter) == 0x240);
};

/// A pointer to a bxCAN register block, from [`Peripheral`](crate::Peripheral).
#[derive(Clone, Copy)]
pub(crate) struct Regs(NonNull<RegisterBlock>);

// SAFETY: the registers are only touched with the critical section held, or
// by the one channel that owns them.
unsafe impl Send for Regs {}

impl Regs {
    /// # Safety
    ///
    /// `address` must be a bxCAN register block's.
    pub(crate) const unsafe fn new(address: usize) -> Self {
        // SAFETY: the caller gives the address of a register block, which is
        // never null.
        Self(unsafe { NonNull::new_unchecked(address as *mut RegisterBlock) })
    }

    pub(crate) fn get(&self) -> &RegisterBlock {
        // SAFETY: the pointer is to a register block (see `new`), which lives
        // for as long as the program.
        unsafe { self.0.as_ref() }
    }
}

// MCR: master control.
pub(crate) const MCR_INRQ: u32 = 1 << 0;
pub(crate) const MCR_SLEEP: u32 = 1 << 1;
pub(crate) const MCR_TXFP: u32 = 1 << 2;
pub(crate) const MCR_NART: u32 = 1 << 4;
pub(crate) const MCR_TTCM: u32 = 1 << 7;

// MSR: master status.
pub(crate) const MSR_INAK: u32 = 1 << 0;
pub(crate) const MSR_SLAK: u32 = 1 << 1;
pub(crate) const MSR_ERRI: u32 = 1 << 2;

// TSR: transmit status, per mailbox `n`.
pub(crate) const fn tsr_rqcp(n: usize) -> u32 {
    1 << (8 * n)
}
pub(crate) const fn tsr_txok(n: usize) -> u32 {
    1 << (8 * n + 1)
}
pub(crate) const fn tsr_abrq(n: usize) -> u32 {
    1 << (8 * n + 7)
}
pub(crate) const fn tsr_tme(n: usize) -> u32 {
    1 << (26 + n)
}

// RFxR: receive FIFO.
pub(crate) const RFR_FMP: u32 = 0b11;
pub(crate) const RFR_FOVR: u32 = 1 << 4;
pub(crate) const RFR_RFOM: u32 = 1 << 5;

// IER: interrupt enable.
pub(crate) const IER_TMEIE: u32 = 1 << 0;
pub(crate) const IER_FMPIE0: u32 = 1 << 1;
pub(crate) const IER_FOVIE0: u32 = 1 << 3;
pub(crate) const IER_FMPIE1: u32 = 1 << 4;
pub(crate) const IER_FOVIE1: u32 = 1 << 6;
pub(crate) const IER_EPVIE: u32 = 1 << 9;
pub(crate) const IER_BOFIE: u32 = 1 << 10;
pub(crate) const IER_LECIE: u32 = 1 << 11;
pub(crate) const IER_ERRIE: u32 = 1 << 15;

// ESR: error status.
pub(crate) const ESR_EPVF: u32 = 1 << 1;
pub(crate) const ESR_BOFF: u32 = 1 << 2;
/// The last error code: 0 none, 1 to 6 an error, 7 set by software.
pub(crate) const ESR_LEC_SHIFT: u32 = 4;
pub(crate) const LEC_SOFTWARE: u32 = 7;

// BTR: bit timing and test modes.
pub(crate) const BTR_LBKM: u32 = 1 << 30;
pub(crate) const BTR_SILM: u32 = 1 << 31;

// TIxR / RIxR: identifier.
pub(crate) const IR_TXRQ: u32 = 1 << 0;
pub(crate) const IR_RTR: u32 = 1 << 1;
pub(crate) const IR_IDE: u32 = 1 << 2;

// FMR: filter master.
pub(crate) const FMR_FINIT: u32 = 1 << 0;

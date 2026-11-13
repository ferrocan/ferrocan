// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! ferrocan: `gs_usb` (candleLight-compatible) USB to CAN adapter firmware.
//!
//! The library holds what the firmware and its examples share: the board,
//! the channel the bridge drives, the `gs_usb` device and DFU detach.

#![no_std]

pub mod board;
pub mod channel;
pub mod dfu;
pub mod gs_usb;
pub mod version;

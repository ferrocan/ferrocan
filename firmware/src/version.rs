// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The firmware's version, from the package's, in the forms the host sees.

/// Parses one component of the package version.
const fn parse(digits: &str) -> u32 {
    let bytes = digits.as_bytes();
    let mut value = 0;
    let mut i = 0;
    while i < bytes.len() {
        assert!(bytes[i].is_ascii_digit(), "a version component is a number");
        value = value * 10 + (bytes[i] - b'0') as u32;
        i += 1;
    }
    value
}

pub const MAJOR: u32 = parse(env!("CARGO_PKG_VERSION_MAJOR"));
pub const MINOR: u32 = parse(env!("CARGO_PKG_VERSION_MINOR"));
pub const PATCH: u32 = parse(env!("CARGO_PKG_VERSION_PATCH"));

/// USB `bcdDevice`: the version in binary-coded decimal, `0xJJMN` for
/// JJ.M.N.
pub const BCD_DEVICE: u16 = {
    assert!(MAJOR < 100 && MINOR < 10 && PATCH < 10, "bcdDevice holds JJ.M.N");
    #[expect(clippy::cast_possible_truncation, reason = "below 0x10000, by the assertion")]
    let bcd = ((MAJOR / 10) << 12 | (MAJOR % 10) << 8 | MINOR << 4 | PATCH) as u16;
    bcd
};

/// `gs_usb`'s `sw_version` in `DEVICE_CONFIG`: `0xMMmmpp`. At least 2:
/// Linux's `gs_usb` ignores the identify feature of firmware older than
/// that, as candleLight's first releases had it broken.
pub const SW_VERSION: u32 = {
    assert!(MINOR < 0x100 && PATCH < 0x100, "sw_version holds a byte each");
    let version = MAJOR << 16 | MINOR << 8 | PATCH;
    assert!(version >= 2, "Linux needs sw_version 2 or more for identify");
    version
};

// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! The `gs_usb` protocol: Geschwister Schneider's USB to CAN protocol, spoken
//! by candleLight firmware and by Linux's in-kernel `gs_usb` driver.
//!
//! A device has one vendor-specific interface with a bulk IN and a bulk OUT
//! endpoint, which carry [`HostFrame`]s, and answers vendor [`Request`]s to
//! that interface on the control endpoint, `wValue` naming the channel. All
//! values are little-endian (the byte order request of the original
//! firmware is accepted and ignored, as candleLight does).
//!
//! This crate is the wire format only, for both sides: it encodes and decodes
//! the structures, and has no dependencies.

#![no_std]

#[cfg(test)]
extern crate std;

/// candleLight's USB vendor ID, which Linux's `gs_usb` binds to.
pub const VENDOR_ID: u16 = 0x1D50;
/// candleLight's USB product ID.
pub const PRODUCT_ID: u16 = 0x606F;

/// The vendor requests, in a SETUP packet's `bRequest`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Request {
    /// OUT, a `u32`: the host's byte order (`0x0000_BEEF`), ignored.
    HostFormat,
    /// OUT, a [`DeviceBittiming`]: the nominal bit timing.
    Bittiming,
    /// OUT, a [`DeviceMode`]: start or reset a channel.
    Mode,
    /// Bus error reporting; unused.
    Berr,
    /// IN, a [`BtConst`]: the features and timing limits.
    BtConst,
    /// IN, a [`DeviceConfig`]: the channel count and versions.
    DeviceConfig,
    /// IN, a `u32`: the device's microsecond timer.
    Timestamp,
    /// OUT, a `u32`: blink to be identified, or stop.
    Identify,
    /// IN, a `u32`: a user-set identifier.
    GetUserId,
    /// OUT, a `u32`: set the user identifier.
    SetUserId,
    /// OUT, a [`DeviceBittiming`]: the CAN FD data bit timing.
    DataBittiming,
    /// IN: the features and timing limits including CAN FD's data phase.
    BtConstExt,
    /// OUT, a `u32`: switch the termination resistor.
    SetTermination,
    /// IN, a `u32`: whether the termination resistor is on.
    GetTermination,
    /// IN, a [`DeviceState`]: the bus state and error counters.
    GetState,
}

impl Request {
    /// The request with this `bRequest`, if known.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            0 => Self::HostFormat,
            1 => Self::Bittiming,
            2 => Self::Mode,
            3 => Self::Berr,
            4 => Self::BtConst,
            5 => Self::DeviceConfig,
            6 => Self::Timestamp,
            7 => Self::Identify,
            8 => Self::GetUserId,
            9 => Self::SetUserId,
            10 => Self::DataBittiming,
            11 => Self::BtConstExt,
            12 => Self::SetTermination,
            13 => Self::GetTermination,
            14 => Self::GetState,
            _ => return None,
        })
    }
}

/// Feature bits, in [`BtConst::feature`]; the same bits ask for them in
/// [`DeviceMode::flags`].
pub mod feature {
    /// Listen only: never transmit or acknowledge.
    pub const LISTEN_ONLY: u32 = 1 << 0;
    /// Loopback: receive the frames sent.
    pub const LOOP_BACK: u32 = 1 << 1;
    /// Three samples per bit.
    pub const TRIPLE_SAMPLE: u32 = 1 << 2;
    /// One attempt per frame, no retransmission.
    pub const ONE_SHOT: u32 = 1 << 3;
    /// A microsecond timestamp after each frame's data.
    pub const HW_TIMESTAMP: u32 = 1 << 4;
    /// The identify request.
    pub const IDENTIFY: u32 = 1 << 5;
    /// The user ID requests.
    pub const USER_ID: u32 = 1 << 6;
    /// Pad IN transfers to the endpoint's maximum packet size.
    pub const PAD_PKTS_TO_MAX_PKT_SIZE: u32 = 1 << 7;
    /// CAN FD.
    pub const FD: u32 = 1 << 8;
    /// The host adds a byte to each OUT transfer (an `LPC546xx` erratum).
    pub const REQ_USB_QUIRK_LPC546XX: u32 = 1 << 9;
    /// The extended timing limits request.
    pub const BT_CONST_EXT: u32 = 1 << 10;
    /// The termination requests.
    pub const TERMINATION: u32 = 1 << 11;
    /// Bus error frames.
    pub const BERR_REPORTING: u32 = 1 << 12;
    /// The state request.
    pub const GET_STATE: u32 = 1 << 13;
}

/// Flags in [`HostFrame::flags`].
pub mod frame_flag {
    /// The device's receive queue overflowed before this frame.
    pub const OVERFLOW: u8 = 1 << 0;
    /// A CAN FD frame.
    pub const FD: u8 = 1 << 1;
    /// Bit rate switch (CAN FD).
    pub const BRS: u8 = 1 << 2;
    /// Error state indicator (CAN FD).
    pub const ESI: u8 = 1 << 3;
}

/// Flags in [`HostFrame::can_id`], as in Linux's `struct can_frame`.
pub mod can_id {
    /// An extended (29-bit) identifier.
    pub const EFF: u32 = 1 << 31;
    /// A remote frame.
    pub const RTR: u32 = 1 << 30;
    /// An error frame: the identifier's bits say which errors, as in Linux's
    /// `linux/can/error.h`.
    pub const ERR: u32 = 1 << 29;
    /// A standard identifier's bits.
    pub const SFF_MASK: u32 = 0x7FF;
    /// An extended identifier's bits.
    pub const EFF_MASK: u32 = 0x1FFF_FFFF;
}

/// An error frame's contents: [`can_id::ERR`] with these bits in the
/// identifier, and these values in the data, as in Linux's
/// `linux/can/error.h`, which `gs_usb` passes on unchanged.
pub mod can_err {
    /// An error frame's DLC.
    pub const DLC: u8 = 8;

    /// Identifier: lost arbitration, the bit in `data[0]`.
    pub const LOSTARB: u32 = 0x0000_0002;
    /// Identifier: controller problems, in `data[1]` ([`crtl`]).
    pub const CRTL: u32 = 0x0000_0004;
    /// Identifier: no acknowledge on transmission.
    pub const ACK: u32 = 0x0000_0020;
    /// Identifier: bus-off.
    pub const BUSOFF: u32 = 0x0000_0040;
    /// Identifier: the controller restarted, back to error active.
    pub const RESTARTED: u32 = 0x0000_0100;
    /// Identifier: the error counters are in `data[6]` (transmit) and
    /// `data[7]` (receive).
    pub const CNT: u32 = 0x0000_0200;

    /// `data[1]` of a [`CRTL`] error frame.
    pub mod crtl {
        /// The receive buffer overflowed.
        pub const RX_OVERFLOW: u8 = 0x01;
        /// The receive error counter reached the warning level (96).
        pub const RX_WARNING: u8 = 0x04;
        /// The transmit error counter reached the warning level.
        pub const TX_WARNING: u8 = 0x08;
        /// The receive error counter reached error passive (128).
        pub const RX_PASSIVE: u8 = 0x10;
        /// The transmit error counter reached error passive.
        pub const TX_PASSIVE: u8 = 0x20;
        /// Back to error active.
        pub const ACTIVE: u8 = 0x40;
    }
}

/// Why a structure could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DecodeError {
    /// Too few bytes.
    Short,
    /// A CAN FD frame, where only classic frames are supported.
    Fd,
    /// A DLC over 15.
    Dlc,
}

fn u32_at(bytes: &[u8], index: usize) -> Result<u32, DecodeError> {
    let word = bytes.get(4 * index..4 * index + 4).ok_or(DecodeError::Short)?;
    Ok(u32::from_le_bytes([word[0], word[1], word[2], word[3]]))
}

fn put_u32s<const N: usize>(words: [u32; N], out: &mut [u8]) -> &[u8] {
    for (chunk, word) in out.as_chunks_mut::<4>().0.iter_mut().zip(words) {
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    &out[..4 * N]
}

/// [`Request::Mode`]'s `mode`.
pub mod mode {
    /// Stop the channel.
    pub const RESET: u32 = 0;
    /// Start the channel.
    pub const START: u32 = 1;
}

/// [`Request::Mode`]: start or reset a channel, with [`feature`] flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceMode {
    /// [`mode::RESET`] or [`mode::START`].
    pub mode: u32,
    /// The [`feature`]s asked for.
    pub flags: u32,
}

impl DeviceMode {
    /// From the request's data.
    ///
    /// # Errors
    ///
    /// [`DecodeError::Short`] for fewer than 8 bytes.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        Ok(Self {
            mode: u32_at(bytes, 0)?,
            flags: u32_at(bytes, 1)?,
        })
    }
}

/// [`Request::Bittiming`]: a bit's timing in quanta, as Linux computes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceBittiming {
    /// Propagation segment.
    pub prop_seg: u32,
    /// Phase segment 1; with `prop_seg`, the quanta before the sample point.
    pub phase_seg1: u32,
    /// Phase segment 2: the quanta after it.
    pub phase_seg2: u32,
    /// Synchronisation jump width.
    pub sjw: u32,
    /// Bit rate prescaler.
    pub brp: u32,
}

impl DeviceBittiming {
    /// From the request's data.
    ///
    /// # Errors
    ///
    /// [`DecodeError::Short`] for fewer than 20 bytes.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        Ok(Self {
            prop_seg: u32_at(bytes, 0)?,
            phase_seg1: u32_at(bytes, 1)?,
            phase_seg2: u32_at(bytes, 2)?,
            sjw: u32_at(bytes, 3)?,
            brp: u32_at(bytes, 4)?,
        })
    }
}

/// [`Request::DeviceConfig`]: how many channels, and versions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceConfig {
    /// The number of channels less one.
    pub icount: u8,
    /// The firmware's version.
    pub sw_version: u32,
    /// The hardware's version.
    pub hw_version: u32,
}

impl DeviceConfig {
    /// Its 12 bytes, written into `out` (at least 12 long).
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> &'a [u8] {
        out[..4].copy_from_slice(&[0, 0, 0, self.icount]);
        out[4..8].copy_from_slice(&self.sw_version.to_le_bytes());
        out[8..12].copy_from_slice(&self.hw_version.to_le_bytes());
        &out[..12]
    }
}

/// [`Request::BtConst`]: the features, the controller's clock and the ranges
/// of the bit timing's fields (`tseg1` being propagation and phase 1
/// together).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs, reason = "named as in Linux's struct gs_device_bt_const")]
pub struct BtConst {
    pub feature: u32,
    pub fclk_can: u32,
    pub tseg1_min: u32,
    pub tseg1_max: u32,
    pub tseg2_min: u32,
    pub tseg2_max: u32,
    pub sjw_max: u32,
    pub brp_min: u32,
    pub brp_max: u32,
    pub brp_inc: u32,
}

impl BtConst {
    /// Its 40 bytes, written into `out` (at least 40 long).
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> &'a [u8] {
        put_u32s(
            [self.feature, self.fclk_can, self.tseg1_min, self.tseg1_max, self.tseg2_min, self.tseg2_max, self.sjw_max, self.brp_min, self.brp_max, self.brp_inc],
            out,
        )
    }
}

/// [`Request::Identify`]'s mode.
pub mod identify {
    /// Stop identifying.
    pub const OFF: u32 = 0;
    /// Blink, or otherwise show which device this is.
    pub const ON: u32 = 1;
}

/// A request's single `u32`, as [`Request::Identify`] and
/// [`Request::HostFormat`] carry.
///
/// # Errors
///
/// [`DecodeError::Short`] for fewer than 4 bytes.
pub fn decode_u32(bytes: &[u8]) -> Result<u32, DecodeError> {
    u32_at(bytes, 0)
}

/// [`DeviceState::state`]'s values.
pub mod state {
    /// Error active.
    pub const ERROR_ACTIVE: u32 = 0;
    /// Error warning.
    pub const ERROR_WARNING: u32 = 1;
    /// Error passive.
    pub const ERROR_PASSIVE: u32 = 2;
    /// Bus-off.
    pub const BUS_OFF: u32 = 3;
    /// Stopped.
    pub const STOPPED: u32 = 4;
    /// Sleeping.
    pub const SLEEPING: u32 = 5;
}

/// [`Request::GetState`]: the bus state and error counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceState {
    /// One of [`state`].
    pub state: u32,
    /// The receive error counter.
    pub rxerr: u32,
    /// The transmit error counter.
    pub txerr: u32,
}

impl DeviceState {
    /// Its 12 bytes, written into `out` (at least 12 long).
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> &'a [u8] {
        put_u32s([self.state, self.rxerr, self.txerr], out)
    }
}

/// [`HostFrame::echo_id`] of a frame received from the bus, as opposed to the
/// echo of one the host sent.
pub const ECHO_ID_RX: u32 = 0xFFFF_FFFF;

/// The header's length: echo ID, CAN ID, DLC, channel, flags, reserved.
pub const HEADER_LEN: usize = 12;
/// A classic frame's length on the wire, without a timestamp.
pub const CLASSIC_LEN: usize = HEADER_LEN + 8;
/// A classic frame's length with a timestamp.
pub const CLASSIC_TS_LEN: usize = CLASSIC_LEN + 4;

/// A classic CAN frame between host and device: one the host sends (OUT),
/// one received from the bus (IN, [`ECHO_ID_RX`]), or the echo of one the host
/// sent (IN, its `echo_id`), with the eight data bytes always present.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HostFrame {
    /// [`ECHO_ID_RX`], or the host's identifier for a frame it sent.
    pub echo_id: u32,
    /// The identifier, with [`can_id`] flags.
    pub can_id: u32,
    /// The DLC, 0 to 15.
    pub can_dlc: u8,
    /// The channel.
    pub channel: u8,
    /// [`frame_flag`]s.
    pub flags: u8,
    /// The data; bytes past the DLC's length are zero.
    pub data: [u8; 8],
    /// Microseconds on the device's timer, when [`feature::HW_TIMESTAMP`]
    /// was asked for (IN only).
    pub timestamp_us: Option<u32>,
}

impl HostFrame {
    /// Decodes a classic frame, the host's (no timestamp).
    ///
    /// # Errors
    ///
    /// [`DecodeError::Short`] for fewer than [`CLASSIC_LEN`] bytes,
    /// [`DecodeError::Fd`] for a CAN FD frame and [`DecodeError::Dlc`] for a
    /// DLC over 15.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let header = bytes.get(..CLASSIC_LEN).ok_or(DecodeError::Short)?;
        let flags = header[10];
        if flags & frame_flag::FD != 0 {
            return Err(DecodeError::Fd);
        }
        let can_dlc = header[8];
        if can_dlc > 15 {
            return Err(DecodeError::Dlc);
        }
        let mut data = [0; 8];
        data.copy_from_slice(&header[HEADER_LEN..CLASSIC_LEN]);
        Ok(Self {
            echo_id: u32_at(header, 0)?,
            can_id: u32_at(header, 1)?,
            can_dlc,
            channel: header[9],
            flags,
            data,
            timestamp_us: None,
        })
    }

    /// Encodes the frame into `out` (at least [`CLASSIC_TS_LEN`] long),
    /// returning the bytes to send: [`CLASSIC_LEN`] of them, or
    /// [`CLASSIC_TS_LEN`] with a timestamp.
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> &'a [u8] {
        out[..4].copy_from_slice(&self.echo_id.to_le_bytes());
        out[4..8].copy_from_slice(&self.can_id.to_le_bytes());
        out[8..HEADER_LEN].copy_from_slice(&[self.can_dlc, self.channel, self.flags, 0]);
        out[HEADER_LEN..CLASSIC_LEN].copy_from_slice(&self.data);
        match self.timestamp_us {
            Some(timestamp) => {
                out[CLASSIC_LEN..CLASSIC_TS_LEN].copy_from_slice(&timestamp.to_le_bytes());
                &out[..CLASSIC_TS_LEN]
            }
            None => &out[..CLASSIC_LEN],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requests_by_number() {
        assert_eq!(Request::from_u8(0), Some(Request::HostFormat));
        assert_eq!(Request::from_u8(4), Some(Request::BtConst));
        assert_eq!(Request::from_u8(14), Some(Request::GetState));
        assert_eq!(Request::from_u8(15), None);
    }

    #[test]
    fn host_frame_round_trip() {
        let frame = HostFrame {
            echo_id: 3,
            can_id: 0x1234_5678 | can_id::EFF,
            can_dlc: 12,
            channel: 0,
            flags: 0,
            data: [1, 2, 3, 4, 5, 6, 7, 8],
            timestamp_us: None,
        };
        let mut buf = [0; CLASSIC_TS_LEN];
        let bytes = frame.encode(&mut buf);
        assert_eq!(bytes.len(), CLASSIC_LEN);
        assert_eq!(&bytes[..12], &[3, 0, 0, 0, 0x78, 0x56, 0x34, 0x92, 12, 0, 0, 0]);
        assert_eq!(HostFrame::decode(bytes), Ok(frame));
    }

    #[test]
    fn host_frame_with_timestamp() {
        let frame = HostFrame {
            echo_id: ECHO_ID_RX,
            can_id: 0x123,
            can_dlc: 1,
            channel: 0,
            flags: 0,
            data: [0xAA, 0, 0, 0, 0, 0, 0, 0],
            timestamp_us: Some(0x0102_0304),
        };
        let mut buf = [0; CLASSIC_TS_LEN];
        let bytes = frame.encode(&mut buf);
        assert_eq!(bytes.len(), CLASSIC_TS_LEN);
        assert_eq!(&bytes[20..], &[4, 3, 2, 1]);
    }

    #[test]
    fn refuses_short_fd_and_bad_dlc() {
        assert_eq!(HostFrame::decode(&[0; 19]), Err(DecodeError::Short));
        let mut fd = [0; CLASSIC_LEN];
        fd[10] = frame_flag::FD;
        assert_eq!(HostFrame::decode(&fd), Err(DecodeError::Fd));
        let mut dlc = [0; CLASSIC_LEN];
        dlc[8] = 16;
        assert_eq!(HostFrame::decode(&dlc), Err(DecodeError::Dlc));
    }

    #[test]
    fn control_structures() {
        let bt = DeviceBittiming::decode(&[1, 0, 0, 0, 12, 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0, 6, 0, 0, 0]).unwrap();
        assert_eq!((bt.prop_seg, bt.phase_seg1, bt.phase_seg2, bt.sjw, bt.brp), (1, 12, 2, 1, 6));
        assert_eq!(DeviceBittiming::decode(&[0; 19]), Err(DecodeError::Short));
        let dm = DeviceMode::decode(&[1, 0, 0, 0, 0x10, 0, 0, 0]).unwrap();
        assert_eq!(
            dm,
            DeviceMode {
                mode: mode::START,
                flags: feature::HW_TIMESTAMP
            }
        );
        let mut buf = [0; 40];
        assert_eq!(DeviceConfig { icount: 0, sw_version: 2, hw_version: 1 }.encode(&mut buf), &[0, 0, 0, 0, 2, 0, 0, 0, 1, 0, 0, 0]);
        let bt_const = BtConst {
            feature: 0x13,
            fclk_can: 48_000_000,
            tseg1_min: 1,
            tseg1_max: 16,
            tseg2_min: 1,
            tseg2_max: 8,
            sjw_max: 4,
            brp_min: 1,
            brp_max: 1024,
            brp_inc: 1,
        };
        let bytes = bt_const.encode(&mut buf);
        assert_eq!(bytes.len(), 40);
        assert_eq!(&bytes[4..8], &48_000_000u32.to_le_bytes());
        assert_eq!(DeviceState { state: state::BUS_OFF, rxerr: 0, txerr: 255 }.encode(&mut buf), &[3, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0]);
    }
}

// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Frames to and from a mailbox's four registers.

use can_hal::{CanFrame, CanId};

use crate::registers::{IR_IDE, IR_RTR};

/// A mailbox's contents: identifier, data length and time, low and high data
/// words, as in TIxR/TDTxR/TDLxR/TDHxR or RIxR/RDTxR/RDLxR/RDHxR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Words {
    pub(crate) ir: u32,
    pub(crate) dtr: u32,
    pub(crate) dlr: u32,
    pub(crate) dhr: u32,
}

/// The words to send `frame`, its transmit request bit clear. The DLC is the
/// frame's own, 9 to 15 included; a remote frame's data words are zero.
pub(crate) fn encode(frame: &CanFrame) -> Words {
    let mut ir = match frame.id() {
        CanId::Standard(id) => u32::from(id) << 21,
        CanId::Extended(id) => (id << 3) | IR_IDE,
    };
    let mut data = [0u8; 8];
    if frame.is_remote_frame() {
        ir |= IR_RTR;
    } else {
        data[..frame.len()].copy_from_slice(frame.data());
    }
    let [d0, d1, d2, d3, d4, d5, d6, d7] = data;
    #[expect(clippy::cast_possible_truncation, reason = "a DLC is at most 15")]
    let dlc = frame.dlc() as u32;
    Words {
        ir,
        dtr: dlc & 0xF,
        dlr: u32::from_le_bytes([d0, d1, d2, d3]),
        dhr: u32::from_le_bytes([d4, d5, d6, d7]),
    }
}

/// The frame in a receive mailbox. A remote frame's data registers are not
/// read, as they hold whatever the FIFO last carried.
pub(crate) fn decode(words: Words) -> CanFrame {
    let id = if words.ir & IR_IDE == 0 {
        CanId::Standard(((words.ir >> 21) & 0x7FF) as u16)
    } else {
        CanId::Extended((words.ir >> 3) & 0x1FFF_FFFF)
    };
    let dlc = (words.dtr & 0xF) as usize;
    let frame = if words.ir & IR_RTR != 0 {
        CanFrame::new_remote(id, dlc)
    } else {
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&words.dlr.to_le_bytes());
        data[4..].copy_from_slice(&words.dhr.to_le_bytes());
        CanFrame::new_with_dlc(id, &data[..dlc.min(8)], dlc)
    };
    // The masks keep the ID and the DLC in range.
    frame.expect("a mailbox holds a valid frame")
}

/// The 16-bit bit-time stamp of a mailbox's frame (TTCM), taken at its start
/// of frame.
pub(crate) const fn time(words: &Words) -> u16 {
    (words.dtr >> 16) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(frame: &CanFrame) {
        let mut words = encode(frame);
        // The controller adds the time; it doesn't change the frame.
        words.dtr |= 0xBEEF << 16;
        assert_eq!(&decode(words), frame);
        assert_eq!(time(&words), 0xBEEF);
    }

    #[test]
    fn data_and_remote_frames_of_every_dlc() {
        let data = [1, 2, 3, 4, 5, 6, 7, 8];
        for id in [
            CanId::new_standard(0x7FF).unwrap(),
            CanId::new_standard(0).unwrap(),
            CanId::new_extended(0x1FFF_FFFF).unwrap(),
            CanId::new_extended(0x1234_5678).unwrap(),
        ] {
            for dlc in 0..=15 {
                round_trip(&CanFrame::new_with_dlc(id, &data[..dlc.min(8)], dlc).unwrap());
                round_trip(&CanFrame::new_remote(id, dlc).unwrap());
            }
        }
    }

    #[test]
    fn identifier_layout() {
        assert_eq!(encode(&CanFrame::new(CanId::new_standard(0x123).unwrap(), &[]).unwrap()).ir, 0x123 << 21);
        assert_eq!(encode(&CanFrame::new(CanId::new_extended(0x1234_5678).unwrap(), &[]).unwrap()).ir, (0x1234_5678 << 3) | IR_IDE);
        assert_eq!(encode(&CanFrame::new_remote(CanId::new_standard(1).unwrap(), 2).unwrap()).ir, (1 << 21) | IR_RTR);
    }

    #[test]
    fn stale_data_of_a_received_remote_frame_is_dropped() {
        let words = Words {
            ir: (0x123 << 21) | IR_RTR,
            dtr: 3,
            dlr: 0x6745_2301,
            dhr: 0xEFCD_AB89,
        };
        let frame = decode(words);
        assert!(frame.is_remote_frame());
        assert_eq!(frame.dlc(), 3);
        assert_eq!(frame, CanFrame::new_remote(CanId::new_standard(0x123).unwrap(), 3).unwrap());
    }
}

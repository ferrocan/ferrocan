// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Frames to and from message RAM elements: two header words and the data,
//! the same on every `M_CAN`.

use can_hal::{CanFdFrame, CanFrame, CanId, Frame};

use crate::registers::{H0_ESI, H0_RTR, H0_XTD, H1_BRS, H1_DLC_SHIFT, H1_FDF};

/// An element's contents: its two header words and up to 64 data bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Element {
    pub(crate) h0: u32,
    pub(crate) h1: u32,
    pub(crate) data: [u8; 64],
}

/// The data length an FD DLC stands for.
pub(crate) const fn fd_len(dlc: usize) -> usize {
    match dlc {
        0..=8 => dlc,
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        _ => 64,
    }
}

/// The element to send `frame`, its marker and Tx event bits (word 1's top
/// byte) left for the caller. A classic frame keeps its DLC, 9 to 15
/// included; a remote frame's data is zero; an FD frame is sent with bit-rate
/// switching if it asks for it and the controller allows it.
pub(crate) fn encode(frame: &Frame) -> Element {
    let mut h0 = match frame.id() {
        CanId::Standard(id) => u32::from(id) << 18,
        CanId::Extended(id) => id | H0_XTD,
    };
    let mut data = [0u8; 64];
    #[expect(clippy::cast_possible_truncation, reason = "a DLC is at most 15")]
    let h1 = match frame {
        Frame::Can(can) => {
            if can.is_remote_frame() {
                h0 |= H0_RTR;
            } else {
                data[..can.len()].copy_from_slice(can.data());
            }
            ((can.dlc() as u32) & 0xF) << H1_DLC_SHIFT
        }
        Frame::Fd(fd) => {
            if fd.esi() {
                h0 |= H0_ESI;
            }
            data[..fd.len()].copy_from_slice(fd.data());
            let brs = if fd.brs() { H1_BRS } else { 0 };
            (((fd.dlc() as u32) & 0xF) << H1_DLC_SHIFT) | H1_FDF | brs
        }
    };
    Element { h0, h1, data }
}

/// The frame in a received element.
pub(crate) fn decode(element: &Element) -> Frame {
    let id = if element.h0 & H0_XTD == 0 {
        CanId::Standard(((element.h0 >> 18) & 0x7FF) as u16)
    } else {
        CanId::Extended(element.h0 & 0x1FFF_FFFF)
    };
    let dlc = ((element.h1 >> H1_DLC_SHIFT) & 0xF) as usize;
    let frame = if element.h1 & H1_FDF != 0 {
        CanFdFrame::new(id, &element.data[..fd_len(dlc)], element.h1 & H1_BRS != 0, element.h0 & H0_ESI != 0).map(Frame::Fd)
    } else if element.h0 & H0_RTR != 0 {
        CanFrame::new_remote(id, dlc).map(Frame::Can)
    } else {
        CanFrame::new_with_dlc(id, &element.data[..dlc.min(8)], dlc).map(Frame::Can)
    };
    // The masks keep the ID, the DLC and the length in range.
    frame.expect("an element holds a valid frame")
}

/// How many data bytes an element's DLC puts in it (for a remote frame,
/// the bytes it would have).
pub(crate) const fn data_len(h1: u32) -> usize {
    let dlc = ((h1 >> H1_DLC_SHIFT) & 0xF) as usize;
    if h1 & H1_FDF != 0 {
        fd_len(dlc)
    } else if dlc > 8 {
        8
    } else {
        dlc
    }
}

/// The 16-bit timestamp of a received element or Tx event (word 1).
pub(crate) const fn time(h1: u32) -> u16 {
    (h1 & 0xFFFF) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn round_trip(frame: &Frame) {
        let mut element = encode(frame);
        // The controller adds the timestamp and filter index; they don't
        // change the frame.
        element.h1 |= 0x1234 | (5 << 24);
        assert_eq!(&decode(&element), frame);
        assert_eq!(time(element.h1), 0x1234);
        // A remote frame has no data, but the words its DLC would fill are
        // read all the same.
        if frame.is_data_frame() {
            assert_eq!(data_len(element.h1), frame.len());
        }
    }

    fn ids() -> [CanId; 4] {
        [
            CanId::new_standard(0).unwrap(),
            CanId::new_standard(0x7FF).unwrap(),
            CanId::new_extended(0x1234_5678).unwrap(),
            CanId::new_extended(0x1FFF_FFFF).unwrap(),
        ]
    }

    #[test]
    fn classic_frames_of_every_dlc() {
        let data = [1, 2, 3, 4, 5, 6, 7, 8];
        for id in ids() {
            for dlc in 0..=15 {
                round_trip(&Frame::Can(CanFrame::new_with_dlc(id, &data[..dlc.min(8)], dlc).unwrap()));
                round_trip(&Frame::Can(CanFrame::new_remote(id, dlc).unwrap()));
            }
        }
    }

    #[test]
    fn fd_frames_of_every_length() {
        let data: Vec<u8> = (0..64).collect();
        for id in ids() {
            for len in [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64] {
                for (brs, esi) in [(false, false), (true, false), (false, true), (true, true)] {
                    round_trip(&Frame::Fd(CanFdFrame::new(id, &data[..len], brs, esi).unwrap()));
                }
            }
        }
    }

    #[test]
    fn header_layout() {
        let standard = encode(&Frame::Can(CanFrame::new(CanId::new_standard(0x123).unwrap(), &[0xAA]).unwrap()));
        assert_eq!((standard.h0, standard.h1), (0x123 << 18, 1 << 16));
        let extended = encode(&Frame::Fd(CanFdFrame::new(CanId::new_extended(0x1234_5678).unwrap(), &[0; 64], true, false).unwrap()));
        assert_eq!((extended.h0, extended.h1), (0x1234_5678 | H0_XTD, (15 << 16) | H1_FDF | H1_BRS));
        let remote = encode(&Frame::Can(CanFrame::new_remote(CanId::new_standard(1).unwrap(), 3).unwrap()));
        assert_eq!(remote.h0, (1 << 18) | H0_RTR);
    }

    #[test]
    fn stale_data_of_a_received_remote_frame_is_dropped() {
        let mut element = Element {
            h0: (0x123 << 18) | H0_RTR,
            h1: 3 << H1_DLC_SHIFT,
            data: [0xEE; 64],
        };
        element.h1 |= 0x55;
        assert_eq!(decode(&element), Frame::Can(CanFrame::new_remote(CanId::new_standard(0x123).unwrap(), 3).unwrap()));
    }
}

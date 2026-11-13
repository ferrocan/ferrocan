ferrocan
========

USB to CAN adapter firmware for STM32, in Rust on [Embassy](https://embassy.dev).
It speaks gs_usb, the protocol of candleLight and Linux's in-kernel `gs_usb`
driver, so it works with SocketCAN out of the box.

It runs on DSD TECH's SH-C30A and SH-C30G (STM32F072, bxCAN, classic CAN)
and SH-C31A (STM32G431, FDCAN, CAN FD). Unlike candleLight, it echoes a
frame to the host when the frame has been acknowledged on the bus, not when
it has been queued.

# Crates

- `firmware` (package `ferrocan`): the firmware, one cargo feature per board.

The gs_usb wire format, `gs_usb_protocol`, lives in
[can_usb_rs](https://github.com/aeolionics-rs/can_usb_rs) beside the host
driver, `can_usb_gs`.

The CAN drivers, `can-hal-bxcan` (bxCAN) and `can-hal-fdcan` (FDCAN), are
[can-hal](https://github.com/can-hal-rs-contributors/can-hal-rs) backends in
[can-hal-embedded](https://github.com/aeolionics-rs/can-hal-embedded), built
on the confirmed-transmission traits of our
[can-hal fork](https://github.com/aeolionics-rs/can-hal-rs)'s `aeolionics`
branch. Their channels confirm transmissions: a frame is reported sent when
the controller says it was acknowledged, in bus order with the frames
received.

All three are git dependencies, which `Cargo.lock` pins to commits.

## Building against local checkouts

For changes to the libraries not yet pushed, check them out beside this
repository and patch them in `.cargo/local.toml`, which `.cargo/config.toml`
includes if it exists and git ignores:

```toml
[patch."https://github.com/aeolionics-rs/can-hal-embedded"]
can-hal-bxcan = { path = "../can-hal-embedded/can-hal-bxcan" }
can-hal-fdcan = { path = "../can-hal-embedded/can-hal-fdcan" }

[patch."https://github.com/aeolionics-rs/can_usb_rs"]
gs_usb_protocol = { path = "../can_usb_rs/gs_usb_protocol" }

[patch."https://github.com/aeolionics-rs/can-hal-rs"]
can-hal-rs = { path = "../can-hal-rs/can-hal" }
```

A patched build rewrites the patched entries in `Cargo.lock` as local paths.
That does no harm: an unpatched build puts the git entries back, and the
release workflow regenerates the lockfile before it builds. Merge a library's
change before the ferrocan change that needs it, so that CI builds against
it.

# Status

All three adapters work as gs_usb devices under Linux's `gs_usb` and with
[`can_usb_gs`](https://github.com/aeolionics-rs/can_usb_rs), tested on real
buses against a Kvaser Leaf Light, candleLight-based adapters and an M_CAN
controller under Linux.

- **Frames:** every kind of classic frame (standard and extended, data and
  remote, every DLC) goes both ways unchanged, paced and with the bus
  saturated (about 2000 frames a second at 250 kbit/s); 2000-frame bursts
  arrive whole. On the SH-C31A, CAN FD at every length, with and without
  bit rate switching and the error state indicator (tested at 500 kbit/s
  and 2 Mbit/s).
- **Echoes** reach the host once the frame is acknowledged, in bus order with
  the frames received. Single-shot frames are sent once or given up after
  one attempt.
- **Errors** reach the host as error frames, as candleLight sends them:
  error warning (at 96), error passive, bus-off and the restart, with the
  error counters; lost frames set the overflow flag. Bus-off recovers by
  itself after 100 ms, as candleLight and Kvaser's adapters do. With bus
  error reporting on (`ip link … berr-reporting on`), each bus error is
  reported. `GET_STATE` gives the live counters (`ip -details link show`).
- **Each start is as from power-on:** the controller is reset, so a new
  session begins with its error counters at zero and no frames left over.
- **Identify** (`ethtool -p`) blinks the LEDs fast. Otherwise the LEDs are
  dark on the bus and flash for each frame received (RX) and sent (TX), with
  a brief heartbeat each second while stopped.
- **DFU:** a DFU runtime interface beside the gs_usb one restarts the
  adapter in ST's ROM bootloader, so it can be reflashed over USB without
  its boot switch (see below). A firmware panic restarts it there too.
- **Transmit confirmation**, an extension Linux doesn't use: a host that
  turns it on is told when a frame wasn't sent, and why (given up, bus-off,
  aborted; on bxCAN also lost arbitration or an error), and can withdraw a
  pending frame. `can_usb_gs` supports it.

Limits:

- CAN FD is bounded by USB full speed: gs_usb takes two USB packets per FD
  frame, so the SH-C31A receives about 6000 short FD frames a second (stock
  candleLight-based firmware about 7000); past that, frames are dropped and
  the host is told with the overflow flag. Classic CAN and longer FD frames
  are unaffected.
- Not implemented: filters, the `BUS_OFF_RECOVERY` and `PAD_PKTS` requests,
  more than one channel.
- Windows: Microsoft OS 2.0 descriptors ask for WinUSB on both interfaces,
  with candleLight's device interface GUID on the gs_usb one, so Windows
  should need no driver and candleLight tools should find it. The
  descriptors read correctly, but this hasn't been tried on Windows yet.

ST's NUCLEO-F072RB, 32F072BDISCOVERY (through its USB USER connector) and
NUCLEO-G431KB (no USB or CAN pins: a start-up check under a probe) are
supported for development.

# Building

Each board has cargo aliases, run from the repository's root:

```sh
cargo build-nucleo-f072rb
cargo run-nucleo-f072rb      # flash with probe-rs through the Nucleo's ST-LINK, then show the log
cargo clippy-nucleo-f072rb
cargo build-sh-c30a           # release image; flash it over USB as below
```

# Flashing over USB

On a board without a debug probe, such as an adapter, flash over USB with
`dfu-util` and `arm-none-eabi-objcopy`:

```sh
arm-none-eabi-objcopy -O binary target/thumbv6m-none-eabi/release/ferrocan ferrocan.bin
dfu-util -d 1d50:606f -e                                     # restart in the ROM bootloader
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D ferrocan.bin  # flash, then run it
```

A board still running other firmware can be put in the bootloader with its
boot switch instead.

# USB identity

ferrocan presents USB ID `1d50:606f`, the ID OpenMoko allocated to
candleLight, so that Linux's `gs_usb` driver, which matches on it, binds
the adapter with no configuration. Other gs_usb-compatible firmware does the
same. The manufacturer and product strings name ferrocan and the board, so a
host can tell it from candleLight.

# License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

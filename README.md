ferrocan
========

USB to CAN adapter firmware for STM32, in Rust on [Embassy](https://embassy.dev).
It speaks gs_usb, the protocol of candleLight and Linux's in-kernel `gs_usb`
driver, so it works with SocketCAN out of the box.

Planned: classic CAN on the STM32F072 (bxCAN; the DSD Tech SH-C30A), then
CAN FD on the STM32G431 (FDCAN; the SH-C31A), with a transmit echo sent when
the frame is on the bus rather than when it is queued.

# Crates

- `firmware` (package `ferrocan`): the firmware, one cargo feature per board.

The gs_usb wire format, `gs_usb_protocol`, lives in
[can_usb_rs](https://github.com/aeolionics-rs/can_usb_rs) beside the host
driver, `can_usb_gs`, and is checked out beside this repository.

The CAN drivers, `can-hal-bxcan` (bxCAN) and `can-hal-fdcan` (FDCAN), are
[can-hal](https://github.com/can-hal-rs-contributors/can-hal-rs) backends in
[can-hal-embedded](https://github.com/aeolionics-rs/can-hal-embedded), checked
out beside this repository, as is our can-hal fork (`../can-hal-rs`, on its
`aeolionics` branch) for the confirmed-transmission traits they use. Their
channels confirm transmissions: a frame is reported sent when the controller
says it was acknowledged, in bus order with the frames received.

# Status

On the NUCLEO-F072RB and the 32F072BDISCOVERY, `can-hal-bxcan` passes its
loopback example: every kind of classic frame (standard and extended, data and
remote, DLC 0 to 15) is received back unchanged and reported sent, and reports
keep their order through an abort.

On the Discovery, whose USB USER connector is the F072's own USB, the firmware
is a gs_usb device that Linux's `gs_usb` binds to. In loopback, `cansend`
frames of every kind come back echoed and received, and `cangen` bursts of
2000 frames at 0.3 ms intervals arrive complete, each echo sent when the
controller reports the frame acknowledged.

Changes of bus state reach the host as error frames (error passive, bus-off,
restarted, with the error counters), and lost frames as the overflow flag:
with no bus attached, one frame takes the channel to error passive and
bus-off, which Linux follows. After 100 ms bus-off the channel recovers by
itself, as candleLight and Kvaser's adapters do, and Linux counts the restart.
The host can read the bus state and error counters (`GET_STATE`, shown by
`ip -details link show` as `berr-counter`), and ask the device to identify
itself (`ethtool -p`), which blinks its LED fast.

A DFU runtime interface beside the gs_usb one restarts the device in ST's ROM
bootloader, so it can be reflashed over USB without its boot switch (see
below).

It runs on the DSD TECH SH-C30A (a CANable V1.1), flashed over USB from its
stock candleLight firmware without the boot switch, keeping its serial number.

On a real bus at 250 kbit/s, the SH-C30A against a Kvaser Leaf Light v2 (both
through Linux), frames of every kind go both ways unchanged, with each echo
sent once the Leaf acknowledged the frame; 2000-frame bursts at 1 ms arrive
complete in each direction; and with the Leaf saturating the bus (about 1700
eight-byte extended frames a second) ferrocan receives every frame, with no
overruns or errors.

Against the Leaf it also passed single-shot (sent once when acknowledged, given
up after one attempt when not) and a real fault: with the Leaf at the wrong
bit rate, ferrocan went error passive on transmit and receive, bus-off,
restarted by itself, and returned to error active once the Leaf was put right,
with Linux following each step from its error frames.

When the host asks for bus error reporting (`ip link … berr-reporting on`),
each bus error reaches it as an error frame, as candleLight writes them: with
nobody to acknowledge, every retry is reported, about 3400 a second at
250 kbit/s, until a node acknowledges. On USB suspend the channel leaves the
bus.

The LEDs show what the adapter is doing: a brief heartbeat each second while
stopped; dark on the bus, flashing for each frame received (RX) and sent (TX);
blinking fast to identify. A firmware that panics on a board with no debug
probe restarts in the ROM bootloader, so it can still be reflashed over USB.

For Windows, Microsoft OS 2.0 descriptors ask for WinUSB on both interfaces,
with candleLight's device interface GUID on the gs_usb one, so Windows should
need no driver and candleLight tools should find it. The descriptors read
correctly, but this hasn't been tried on Windows yet.

Not yet: CAN FD on the G431.

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

# License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

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
- `can-hal-bxcan`: a [can-hal](https://github.com/can-hal-rs-contributors/can-hal-rs)
  backend for STM32's bxCAN, with no HAL dependency. Its channel confirms
  transmissions (`ConfirmedTransmit` and `ReceiveEvents`): a frame is reported
  sent when the controller says it was acknowledged, in bus order with the
  frames received. It uses the confirmed-transmission traits on our fork's
  `aeolionics` branch, checked out beside this repository.
- `gs_usb_protocol`: the gs_usb wire format (requests, structures, host
  frames), for both device and host, with no dependencies.

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

A DFU runtime interface beside the gs_usb one restarts the device in ST's ROM
bootloader, so it can be reflashed over USB without its boot switch (see
below).

Not yet: a real bus.

# Building

Each board has cargo aliases, run from the repository's root:

```sh
cargo build-nucleo-f072rb
cargo run-nucleo-f072rb      # flash with probe-rs through the Nucleo's ST-LINK, then show the log
cargo clippy-nucleo-f072rb
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

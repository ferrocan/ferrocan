ferrocan
========

USB to CAN adapter firmware for STM32, in Rust on [Embassy](https://embassy.dev).
It speaks gs_usb, the protocol of candleLight and Linux's in-kernel `gs_usb`
driver, so it works with SocketCAN out of the box.

Planned: classic CAN on the STM32F072 (bxCAN; the DSD Tech SH-C30A), then
CAN FD on the STM32G431 (FDCAN; the SH-C31A), with a transmit echo sent when
the frame is on the bus rather than when it is queued.

# Status

Milestone 1: the NUCLEO-F072RB boots on its 48 MHz internal oscillator,
blinks LD2 and logs over RTT.

# Building

Each board has cargo aliases, run from the repository's root:

```sh
cargo build-nucleo-f072rb
cargo run-nucleo-f072rb      # flash with probe-rs through the Nucleo's ST-LINK, then show the log
cargo clippy-nucleo-f072rb
```

# License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

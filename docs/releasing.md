# Releasing ferrocan

A release is a tag, `vX.Y.Z`, on `main`. Pushing it runs the release
workflow, which builds the images with `scripts/release.sh` and drafts a
GitHub release with them; it's published by hand once the images have
passed the bench checklist below.

## What a release holds

For each adapter family, `sh-c30` (SH-C30A, SH-C30G: STM32F072, bxCAN) and
`sh-c31` (SH-C31A: STM32G431, FDCAN):

- `ferrocan-X.Y.Z-<family>.bin`, the image to flash, and `.elf`, the same
  with its symbols, for a debugger and for decoding defmt;
- `ferrocan-X.Y.Z-<family>.cdx.json`, its software bill of materials
  (CycloneDX 1.5);
- `ferrocan-X.Y.Z-<family>.THIRD-PARTY-LICENSES.txt`, the licences of the
  crates in it;

and `SHA256SUMS` over them all. On a public repository the images also get
signed build provenance (`gh attestation verify <file> -R ferrocan/ferrocan`).

The images are reproducible: `scripts/release.sh` with the toolchain that
`rust-toolchain.toml` pins and the commits that `Cargo.lock` pins makes the
same bytes on any machine. It maps the checkout, Cargo's home and the
toolchain to fixed paths, and fails if any local path is left in what it
writes. The SBOMs carry a timestamp and a serial number, so they differ from
run to run.

## Steps

1. Raise the version in `Cargo.toml` (`workspace.package.version`) in a
   pull request. The firmware reports it as USB `bcdDevice` and as
   `gs_usb`'s `sw_version`.
2. Once that's merged, take the images from the `release-images` artifact
   of `main`'s CI run (or run `scripts/release.sh`, which makes the same
   bytes) and go through the bench checklist.
3. Tag the merge commit and push the tag:
   `git tag -a vX.Y.Z -m "ferrocan X.Y.Z" && git push origin vX.Y.Z`.
4. Check the draft release (assets, notes), then publish it.

## Bench checklist

On a bus with a second gs_usb adapter (ferrocan or candleLight) and Linux's
SocketCAN, each image on its board:

- Frames of every kind both ways, unchanged: standard and extended, data
  and remote, every DLC; on the SH-C31, CAN FD at every length, with and
  without bit rate switching.
- Echoes only once a frame is on the bus; a 2000-frame burst arrives whole.
- Error warning and error passive, reported as error frames, with the
  error counters (`ip -details link show`).
- Bus-off with CAN-H shorted to CAN-L, then the restart once the short is
  gone.
- Single-shot: the reason a frame wasn't sent (lost arbitration, an error).
- Transmit confirmation and abort, with a host that turns them on
  (`can_usb_gs`'s examples).
- Identify (`ethtool -p`), and DFU detach: `scripts/flash.sh` with the new
  image.

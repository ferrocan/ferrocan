#!/bin/sh
# SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
#
# SPDX-License-Identifier: MIT OR Apache-2.0

# Flashes a ferrocan image (the .bin of a release) onto an adapter over USB
# with dfu-util, through the STM32's ROM bootloader: restarts a board that
# runs ferrocan in the bootloader, writes the image, reads it back to
# compare, and starts it.
#
# Usage: scripts/flash.sh IMAGE.bin [SERIAL]
#
# SERIAL picks one of several gs_usb adapters by its USB serial number. A
# board running other firmware has to be put in the bootloader by hand
# first, with its BOOT switch or jumper; only one board may be in the
# bootloader at a time.

set -eu

ROM=0483:df11
FERROCAN=1d50:606f
ADDRESS=0x08000000

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "usage: $0 IMAGE.bin [SERIAL]" >&2
    exit 2
fi
image=$1
serial=${2:-}

case $image in
*.bin) ;;
*)
    echo "error: $image: flash the .bin image, not the .elf" >&2
    exit 2
    ;;
esac
[ -r "$image" ] || { echo "error: can't read $image" >&2; exit 1; }
command -v dfu-util >/dev/null || { echo "error: dfu-util isn't installed" >&2; exit 1; }

# Check the image against the release's SHA256SUMS, if it's beside it.
sums=$(dirname "$image")/SHA256SUMS
if [ -r "$sums" ] && command -v sha256sum >/dev/null; then
    (cd "$(dirname "$image")" && grep " $(basename "$image")\$" SHA256SUMS | sha256sum -c -)
fi

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

# The number of USB devices with this ID that dfu-util sees.
devices() {
    dfu-util -l 2>/dev/null | grep "\[$1\]" | grep -o 'path="[^"]*"' | sort -u | wc -l | tr -d ' '
}

# dfu-util may hang on the request that starts the firmware; give up after
# a while where the system has `timeout`.
bounded() {
    if command -v timeout >/dev/null; then timeout 10 "$@"; else "$@"; fi
}

# A board running ferrocan (or candleLight, which shares its USB ID) restarts
# in the bootloader when asked.
running=$(devices "$FERROCAN")
if [ -n "$serial" ]; then
    echo "Restarting $serial in the bootloader"
    dfu-util -d "$FERROCAN" -S "$serial" -e >/dev/null
elif [ "$running" = 1 ]; then
    echo "Restarting the adapter in the bootloader"
    dfu-util -d "$FERROCAN" -e >/dev/null
elif [ "$running" -gt 1 ]; then
    echo "error: $running gs_usb adapters ($FERROCAN): name one by its serial number" >&2
    exit 1
fi

# Wait for the bootloader.
tries=0
while [ "$(devices "$ROM")" = 0 ]; do
    tries=$((tries + 1))
    if [ $tries -gt 20 ]; then
        echo "error: no board in the bootloader ($ROM); put it there with its BOOT switch" >&2
        exit 1
    fi
    sleep 0.5
done
if [ "$(devices "$ROM")" != 1 ]; then
    echo "error: more than one board in the bootloader; leave only one" >&2
    exit 1
fi

echo "Writing $image"
dfu-util -d "$ROM" -a 0 -s "$ADDRESS" -D "$image"

echo "Reading it back"
size=$(wc -c < "$image" | tr -d ' ')
dfu-util -d "$ROM" -a 0 -s "$ADDRESS:$size" -U "$tmp/readback.bin" >/dev/null
if ! cmp -s "$image" "$tmp/readback.bin"; then
    echo "error: the flash doesn't hold the image; try again" >&2
    exit 1
fi

echo "Starting it"
bounded dfu-util -d "$ROM" -a 0 -s "$ADDRESS:4:leave" -U "$tmp/leave.bin" >/dev/null 2>&1 || true

tries=0
while [ "$(devices "$FERROCAN")" -le "$running" ] && [ $tries -lt 10 ]; do
    tries=$((tries + 1))
    sleep 0.5
done
if [ "$(devices "$FERROCAN")" -gt "$running" ] || [ -n "$serial" ]; then
    echo "Done: ferrocan is running"
else
    echo "Written and verified, but the adapter hasn't come back: if its BOOT switch is on, switch it off and replug it"
fi

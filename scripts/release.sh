#!/bin/sh
# SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
#
# SPDX-License-Identifier: MIT OR Apache-2.0

# Builds the release images, the same on any machine with the pinned
# toolchain: for each adapter family an ELF and a raw binary, its SBOM
# (CycloneDX) and its third-party notices, and SHA256SUMS over them all.
#
# Usage: scripts/release.sh [output directory, default dist]
#
# Needs cargo-cyclonedx and cargo-about (with its `cli` feature). Builds
# with --locked against the libraries from GitHub, so it refuses to run
# while .cargo/local.toml patches in local checkouts.

set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
out=${1:-dist}

if [ -e .cargo/local.toml ]; then
    echo "error: .cargo/local.toml patches in local checkouts; move it aside to build a release" >&2
    exit 1
fi

# Our package's version, from `ferrocan@X.Y.Z` at the end of its id.
version=$(cargo pkgid --locked -p ferrocan | sed 's/.*[@#]//')

# The images carry source paths in their panic messages and debug info:
# map the checkout, Cargo's home and the toolchain to fixed names, so that
# no local path ends up in them and every machine builds the same bytes.
cargo_home=${CARGO_HOME:-$HOME/.cargo}
sysroot=$(rustc --print sysroot)
sep=$(printf '\037')
CARGO_ENCODED_RUSTFLAGS="--remap-path-prefix=$root=/ferrocan$sep--remap-path-prefix=$cargo_home=/cargo$sep--remap-path-prefix=$sysroot=/rustc"
export CARGO_ENCODED_RUSTFLAGS

host=$(rustc -vV | sed -n 's/^host: //p')
tools=$sysroot/lib/rustlib/$host/bin

rm -rf "$out"
mkdir -p "$out"

# One image per adapter family: the family's name, the board feature it is
# built with, and its target.
build() {
    family=$1 feature=$2 target=$3
    name=ferrocan-$version-$family
    elf=target/$target/release/ferrocan

    cargo build --locked --release -p ferrocan --target "$target" --features "$feature"
    cp "$elf" "$out/$name.elf"
    "$tools/llvm-objcopy" -O binary "$elf" "$out/$name.bin"
    "$tools/llvm-size" "$out/$name.elf"

    # The SBOM names our package by Cargo's id, which holds the checkout's
    # path: map it as the images do.
    cargo cyclonedx --manifest-path firmware/Cargo.toml --features "$feature" --target "$target" \
        --no-build-deps --format json --spec-version 1.5 --override-filename "$name.cdx"
    sed "s#path+file://$root/#path+file:///ferrocan/#g" "firmware/$name.cdx.json" > "$out/$name.cdx.json"
    rm "firmware/$name.cdx.json"

    cargo about generate --locked -m firmware/Cargo.toml --features "$feature" --target "$target" \
        about.hbs -o "$out/$name.THIRD-PARTY-LICENSES.txt"

    # No local path may be left in what we publish.
    for file in "$out/$name".*; do
        if grep -a -q -e "$HOME" -e /home/ -e /Users/ "$file"; then
            echo "error: $file still holds a local path:" >&2
            grep -a -o -e "$HOME[^[:space:]\"]*" -e '/home/[^[:space:]"]*' -e '/Users/[^[:space:]"]*' "$file" | sort -u | head >&2
            exit 1
        fi
    done
}

# The SH-C30A and SH-C30G (STM32F072, bxCAN).
build sh-c30 sh-c30a thumbv6m-none-eabi
# The SH-C31A (STM32G431, FDCAN).
build sh-c31 sh-c31a thumbv7em-none-eabihf

(cd "$out" && sha256sum -- * > SHA256SUMS)
echo "ferrocan $version: $out/"
cat "$out/SHA256SUMS"

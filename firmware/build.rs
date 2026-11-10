// SPDX-FileCopyrightText: 2026-2026. Aeolionics, LLC
//
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Linker scripts from cortex-m-rt (`link.x`, with embassy-stm32's
//! `memory.x`) and defmt (`defmt.x`), for the firmware and its examples.

fn main() {
    for target in ["bins", "examples"] {
        println!("cargo:rustc-link-arg-{target}=--nmagic");
        println!("cargo:rustc-link-arg-{target}=-Tlink.x");
        println!("cargo:rustc-link-arg-{target}=-Tdefmt.x");
    }
}

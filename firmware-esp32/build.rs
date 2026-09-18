//! Build script: hand the linker script to the Xtensa linker.
//!
//! Unlike the other bare-metal targets, Xtensa has no usable `lld`: the esp-rs
//! fork's `rust-lld` refuses `-m elf32xtensa` ("unknown emulation"), so the
//! `.cargo/config.toml` for this target points at Espressif's `xtensa-esp32-elf-gcc`
//! as the linker driver. `-Tlink.x` is therefore passed on to GNU ld by GCC.

use std::env;
use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rustc-link-search={}", manifest.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=build.rs");
}

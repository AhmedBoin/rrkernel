//! Build script: hand the linker script to `rust-lld`.
//!
//! `rustc` already defaults to the bundled `rust-lld` for `*-none-elf` targets,
//! so no external RISC-V toolchain is required to build or link this firmware.

use std::env;
use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rustc-link-search={}", manifest.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=build.rs");
}

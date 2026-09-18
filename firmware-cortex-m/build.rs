//! Build script: hand the linker script to `rust-lld` and check the memory
//! budget the kernel needs on this part.
//!
//! `rustc` already defaults to the bundled `rust-lld` for `*-none-eabi` targets
//! (verified against the target spec: `"linker": "rust-lld"`), so no external
//! ARM toolchain is required to build this firmware.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // rust-lld searches the link search path for `-Tlink.x`.
    println!("cargo:rustc-link-search={}", manifest.display());
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Keep a copy in OUT_DIR too, so `-L OUT_DIR` also resolves it.
    let _ = fs::copy(manifest.join("link.x"), out.join("link.x"));

    println!("cargo:rerun-if-changed=link.x");
    println!("cargo:rerun-if-changed=build.rs");
}

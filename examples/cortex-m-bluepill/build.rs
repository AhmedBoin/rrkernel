//! Just enough build script to let `cortex-m-rt`'s linker script find `memory.x`
//! (it does `INCLUDE memory.x`), which is the only board-specific file in this project.

use std::env;

fn main() {
    println!(
        "cargo:rustc-link-search={}",
        env::var("CARGO_MANIFEST_DIR").unwrap()
    );
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}

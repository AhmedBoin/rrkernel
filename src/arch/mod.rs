//! Architecture / platform ports.
//!
//! The kernel core (`tcb`, `ring`, `arena`, `closure`, `scheduler`,
//! `trampoline`) is 100% neutral `core` code. Everything that is genuinely
//! platform-specific is reduced to the small set of functions below, and each
//! backend is a single file that implements exactly this list:
//!
//! | Function | What it must do |
//! |---|---|
//! | [`plan_timer`] | validate the requested slice against the real timer, return the achieved values |
//! | [`start_timer`] / [`retune_timer`] | arm the periodic tick source |
//! | [`platform_limits`] | report min/max achievable slice |
//! | [`cycles_to_ns`] | convert the achieved timer ticks back to time |
//! | [`critical_enter`] / [`critical_exit`] | make ring mutation atomic |
//! | [`adopt_current_task`] | turn the calling context into task 0 |
//! | [`create_task`] | build a fresh task's stack frame / thread / fiber |
//! | [`request_switch`] | ask for an immediate context switch + full-slice restart |
//! | [`exit_current_task_forever`] | what a finished task does instead of returning |
//! | [`on_task_reclaimed`] | release backend resources (e.g. the Win32 thread handle) when a dead task's memory is recycled |
//! | [`is_pinned`] | tell the reclaimer to keep a dead TCB around (the port still needs it to switch away from it) |
//! | [`idle_forever`] | what the CPU does with nothing runnable |
//! | [`shutdown`] | stop the kernel |
//!
//! That is the entire porting surface — see `docs/PORTING.md`.

/// The slice as actually programmed into the timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerPlan {
    /// Achieved slice length in nanoseconds.
    pub slice_ns: u64,
    /// Achieved slice length in timer ticks ("cycles").
    pub slice_cycles: u32,
    /// Timer frequency in Hz (0 when the platform's timer is time-based and
    /// the "cycles" field carries a time unit instead: 100 ns on Win32, 1 µs
    /// on POSIX).
    pub timer_hz: u32,
}

// ---------------------------------------------------------------------------
// Backend selection
// ---------------------------------------------------------------------------
//
// The kernel core never sees any of this: a port is chosen purely by the target
// triple, so `cargo build --target <triple>` "just works" with no feature flags.
// Adding a port means adding one `mod`/`pub use` pair plus one arm here.

// --- hosted ----------------------------------------------------------------
#[cfg(all(feature = "std", target_os = "windows"))]
mod win32;
#[cfg(all(feature = "std", target_os = "windows"))]
pub use win32::*;

#[cfg(all(feature = "std", target_os = "linux"))]
mod posix;
#[cfg(all(feature = "std", target_os = "linux"))]
pub use posix::*;

#[cfg(all(feature = "std", unix, not(target_os = "linux")))]
compile_error!(
    "rrkernel: the POSIX backend is currently implemented and verified for \
     Linux/glibc only. `sigaction`, `sigset_t` and `sigevent` have different \
     layouts on macOS/BSD, and guessing them would corrupt memory instead of \
     failing loudly. See docs/PORTING.md (about 30 lines to adapt)."
);

#[cfg(all(feature = "std", not(any(target_os = "windows", unix))))]
compile_error!(
    "rrkernel: no `std` backend for this operating system. Implement the port \
     described in docs/PORTING.md and add it to src/arch/mod.rs."
);

// --- bare metal ------------------------------------------------------------

// ARM Cortex-M (M-profile: thumbv6m, thumbv7m, thumbv8m.main, ...).
#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    target_feature = "mclass"
))]
mod cortex_m;
#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    target_feature = "mclass"
))]
pub use cortex_m::*;

// ARM A/R profile (armv7a-none-eabi, armv7r-none-eabi, armv8r-none-eabihf, ...):
// mode-based exception model with banked stack pointers, not the M-profile
// NVIC/SysTick model.
#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    not(target_feature = "mclass")
))]
mod arm_ar;
#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    not(target_feature = "mclass")
))]
pub use arm_ar::*;

// RISC-V 32-bit machine mode (riscv32imac, riscv32imc, riscv32i, ...).
#[cfg(all(not(feature = "std"), target_arch = "riscv32"))]
mod riscv;
#[cfg(all(not(feature = "std"), target_arch = "riscv32"))]
pub use riscv::*;

// AVR (`avr-none`) and Xtensa (`xtensa-esp32-none-elf`) are *not* ported yet: see
// the `compile_error!` messages below, which state exactly what each needs.

#[cfg(all(
    not(feature = "std"),
    not(any(
        target_arch = "arm",
        target_arch = "riscv32",
        target_arch = "avr",
        target_arch = "xtensa"
    ))
))]
compile_error!(
    "rrkernel: this bare-metal architecture has no port yet. Verified ports: ARM \
     Cortex-M (thumbv6m/v7m/v8m.main) and ARM A/R profile (armv7a/armv7r, QEMU- \
     tested) and RISC-V 32 (riscv32imac, QEMU-tested). See docs/PORTING.md for the \
     port surface: it is 14 functions, most of them one line."
);

// AVR is *not* ported yet, and the reason is toolchain rather than design: the
// kernel core is already AVR-compatible (no atomics anywhere, 16-bit `usize`
// throughout), but
//   * `avr-none` needs `-C target-cpu=atmega328p` (rustc: "target requires
//     explicitly specifying a cpu") and `-Z build-std=core`,
//   * its linker is `avr-gcc` — `lld` has no AVR backend — so `avr-gcc` (or the
//     `binutils-avr` package) must be installed to produce a runnable image for
//     QEMU's `-M arduino-uno`.
//   * AVR has no atomic read-modify-write at all (`max-atomic-width = 16`), which
//     is why this kernel's core deliberately avoids atomics: the counter/flag
//     helpers in `firmware-common` use a single-writer volatile cell for exactly
//     this reason.
#[cfg(all(not(feature = "std"), target_arch = "avr"))]
compile_error!(
    "rrkernel: the AVR port is not implemented yet. What it needs, concretely: \
     (1) `rustup component add rust-src`, then build with \
     `RUSTFLAGS=\"-C target-cpu=atmega328p\" cargo build -Z build-std=core \
     --target avr-none`; (2) `avr-gcc` (binutils-avr) installed for linking, \
     because `lld` cannot link AVR (QEMU can then run the image with \
     `qemu-system-avr -M arduino-uno`); (3) a port in the shape documented in \
     docs/PORTING.md: a 35-byte frame (r0-r31 + SREG + the 2-byte return address, \
     low byte first), Timer1 in CTC mode with OCR1A, `cli`/`sei` with SREG.I as \
     the critical-section token, and an ISR that swaps SPL/SPH and `reti`s into \
     the next task."
);

// Xtensa: the ESP32 (LX6) and ESP32-S3 (LX7) cores. A windowed register file, a
// fixed `VECBASE` vector table, and `CCOMPARE0` as the slice timer.
//
// This is the port that is *not* just a frame: with 64 physical address registers
// and four windows, a context switch is only half the work — the other half is the
// window overflow/underflow vectors, which `xtensa.rs` implements.
#[cfg(all(not(feature = "std"), target_arch = "xtensa"))]
mod xtensa;
#[cfg(all(not(feature = "std"), target_arch = "xtensa"))]
pub use xtensa::*;




#[cfg(all(feature = "std", not(any(target_os = "windows", unix))))]
compile_error!(
    "rrkernel: no `std` backend for this operating system. Implement the port \
     described in docs/PORTING.md (14 functions) and add it to src/arch/mod.rs."
);

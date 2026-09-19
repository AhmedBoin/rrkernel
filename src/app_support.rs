//! The `app!` facade: everything a Cortex-M application should not have to write.
//!
//! [`app!`](crate::app) hides the four things every bare-metal rrkernel program needs and
//! no *application* should own:
//!
//! * `#[cortex_m_rt::entry]` and the call into `scheduler::main_body` that makes the
//!   program's `main` be **task 0**;
//! * `SchedulerConfig` construction (slice, core clock, per-task stack) and its error
//!   path, which must never be silent;
//! * `HardFault` / `DefaultHandler` that *say what happened* instead of parking quietly —
//!   the whole reason this file exists: a fault that prints nothing looks exactly like a
//!   hang, and that cost hours to chase;
//! * a `#[panic_handler]`, so the application does not need `panic-halt` either.
//!
//! Logging is one optional line — `log: |args| ...` — so the kernel keeps its
//! zero-dependency promise: the sink is supplied by the application (RTT, a UART, a RAM
//! buffer) and the kernel only ever calls it with `core::fmt::Arguments`.

use crate::config::{ConfigError, SchedulerConfig, Slice};
use core::cell::UnsafeCell;

/// The application's log sink, installed by `#[rrkernel]` / `log_with`.
///
/// Guarded by the kernel's own critical section rather than an atomic, so this module
/// compiles on targets with no atomic read-modify-write (Cortex-M0, AVR, RV32IMC) like
/// everything else in the kernel. The section is a handful of instructions and never blocks.
struct LogCell(UnsafeCell<Option<fn(core::fmt::Arguments)>>);

// SAFETY: every access goes through `crate::critical::enter()`.
unsafe impl Sync for LogCell {}

static LOG: LogCell = LogCell(UnsafeCell::new(None));

/// Install the log sink. Called by `#[rrkernel]` before anything can fail.
pub fn install_log(f: fn(core::fmt::Arguments)) {
    let g = crate::critical::enter();
    unsafe { *LOG.0.get() = Some(f) };
    drop(g);
}

/// Format one line to the installed sink, and — always — to the built-in RAM buffer.
///
/// The buffer is the black box: it survives a hang, a hard fault, or a console that never
/// comes up (an RTT terminal that is not initialized yet, a UART with no wire), and a probe
/// can read it back. The alternative is silence, which in this project has cost more time
/// than every other class of bug combined.
pub fn log_fmt(args: core::fmt::Arguments) {
    // Record first, so the reason survives even if the sink itself misbehaves.
    log_to_buffer(args);

    let g = crate::critical::enter();
    let sink = unsafe { *LOG.0.get() };
    drop(g);
    if let Some(f) = sink {
        f(args);
    }
}

// ---------------------------------------------------------------------------
// The default sink: a fixed RAM buffer, readable by a debugger
// ---------------------------------------------------------------------------

/// Capacity of the built-in log buffer, including the trailing NUL.
const LOG_CAP: usize = 512;

struct LogBuf(core::cell::UnsafeCell<[u8; LOG_CAP]>);

// SAFETY: bytes are only written through the guarded counter below, with volatile stores,
// and no task ever holds a reference into the buffer.
unsafe impl Sync for LogBuf {}

struct LenCell(core::cell::UnsafeCell<usize>);

// SAFETY: every access goes through `crate::critical::enter()`.
unsafe impl Sync for LenCell {}

static BUFFER: LogBuf = LogBuf(core::cell::UnsafeCell::new([0; LOG_CAP]));
static BUFFER_LEN: LenCell = LenCell(core::cell::UnsafeCell::new(0));

/// The default log sink: append to a fixed target-RAM buffer.
///
/// This is the default because it needs no hardware, no pins and no dependency — and a
/// debugger can always read it (`probe-rs gdb` → `x/s &BUFFER`). Point `log_with` (or
/// `#[rrkernel(log = rtt)]`) at a live console when you have one; until then, panics and
/// faults still leave a readable trace instead of silence.
pub fn log_to_buffer(args: core::fmt::Arguments) {
    struct W;
    impl core::fmt::Write for W {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            for b in s.bytes() {
                push(b);
            }
            Ok(())
        }
    }
    let _ = core::fmt::Write::write_fmt(&mut W, args);
}

fn push(b: u8) {
    let g = crate::critical::enter();
    let i = unsafe { *BUFFER_LEN.0.get() };
    unsafe { *BUFFER_LEN.0.get() = i.wrapping_add(1) };
    drop(g);
    if i + 1 < LOG_CAP {
        unsafe {
            let p = BUFFER.0.get() as *mut u8;
            core::ptr::write_volatile(p.add(i), b);
            // Always a valid C string: a debugger may read it at any moment.
            core::ptr::write_volatile(p.add(i + 1), 0);
        }
    }
}

/// What the default sink has written so far, without the terminator.
pub fn log_bytes() -> &'static [u8] {
    let g = crate::critical::enter();
    let n = unsafe { *BUFFER_LEN.0.get() }.min(LOG_CAP - 1);
    drop(g);
    // SAFETY: `n` is bounded by the buffer, and the bytes are only ever appended.
    unsafe { core::slice::from_raw_parts(BUFFER.0.get() as *const u8, n) }
}

/// Configure and start the kernel: slice, core clock, per-task stack size.
pub fn init(clock: u32, slice: Slice, stack: usize) -> Result<(), ConfigError> {
    crate::scheduler::init_with(SchedulerConfig::embedded(slice, clock).stack_size(stack))
}

/// Park with interrupts masked: the board is left quiescent and inspectable instead of
/// resetting into the same fault.
pub fn park() -> ! {
    crate::scheduler::shutdown(0)
}

/// Where panics and faults are reported. Optional, one line, and worth it: without a sink
/// the handlers still park (and a hard fault still halts a debugger), but the *reason* is
/// lost, and a silent fault is the single most expensive thing to debug on a target with
/// no console.
pub fn log_with(f: fn(core::fmt::Arguments)) {
    install_log(f);
}

/// Configure and start the kernel — core clock, slice, per-task stack. Called once, from
/// `main`.
///
/// Panics on a rejected configuration, loudly (through the log sink and the handlers
/// `#[rrkernel]` installs). A `Result` would be ignored by most callers, and a silently
/// unconfigured kernel is indistinguishable from a hang.
pub fn configure(clock: u32, slice: Slice, stack: usize) {
    // Mask interrupts across the whole configuration: the tick must not fire while the
    // kernel is half-built.
    let g = crate::critical::enter();
    let started = init(clock, slice, stack);
    drop(g);
    if let Err(e) = started {
        log_fmt(format_args!("rrkernel: configure failed: {:?}\r\n", e));
        panic!("rrkernel: configure failed: {:?}", e);
    }
}

/// The tail of `main`: task 0's body has returned, so unlink it and switch away forever.
///
/// Appended automatically by `#[rrkernel]`; an application never calls this.
pub fn exit_main() -> ! {
    let me = {
        let g = crate::critical::enter();
        let me = unsafe { crate::tcb::KERNEL.current() };
        drop(g);
        me
    };
    assert!(
        !me.is_null(),
        "rrkernel: `configure(...)` was never called before `main` returned"
    );
    // SAFETY: we are task 0 — the context `configure` adopted — and its body has returned.
    unsafe { crate::trampoline::exit_task(me) }
}

/// The body of the `HardFault` that `app!` installs.
///
/// # Why this is gated on the profile, not just on `target_arch`
/// `target_arch = "arm"` is **also true for the A/R-profile port** (`armv7a-none-eabi`,
/// `armv7r-none-eabi`), but `CFSR`/`HFSR`/`BFAR`/`MMFAR` are M-profile `SCB` registers and
/// `mrs r0, psp` is an M-profile instruction. Assembling this body for A/R failed:
///
/// ```text
/// error: invalid operand for instruction
///   --> src\app_support.rs:188:32
/// note: instantiated into assembly here
///   |     mrs r0, psp
/// ```
///
/// Only the **dev** profile reported it: the release profile sets `lto = "fat"`, so the
/// library is built as bitcode and this function — which no A/R firmware calls — is dropped
/// before codegen, leaving the assembler nothing to reject. Gating on
/// `target_feature = "mclass"` makes the two profiles agree, so a stale `VERDICT : PASS`
/// can no longer hide a build failure.
#[cfg(all(target_arch = "arm", target_feature = "mclass"))]
pub fn hard_fault() -> ! {
    let cfsr = unsafe { core::ptr::read_volatile(0xE000_ED28 as *const u32) };
    let hfsr = unsafe { core::ptr::read_volatile(0xE000_ED2C as *const u32) };
    let bfar = unsafe { core::ptr::read_volatile(0xE000_ED38 as *const u32) };
    let mmfar = unsafe { core::ptr::read_volatile(0xE000_ED34 as *const u32) };
    // Every task runs on PSP, so a fault from a task has its exception frame at the top
    // of that stack, with the faulting PC 24 bytes in (R0,R1,R2,R3,R12,LR,PC).
    let psp: u32;
    unsafe { core::arch::asm!("mrs {}, psp", out(reg) psp, options(nomem, nostack)) };
    let pc = unsafe { core::ptr::read_volatile((psp as usize + 24) as *const u32) };
    log_fmt(format_args!(
        "!!! HardFault  CFSR {:#010x} HFSR {:#010x}\r\n",
        cfsr, hfsr
    ));
    log_fmt(format_args!(
        "    BFAR {:#010x} MMFAR {:#010x} PSP {:#010x} faulting PC {:#010x}\r\n",
        bfar, mmfar, psp, pc
    ));
    park()
}

/// A/R-profile fault report — the counterpart of the M-profile body above.
///
/// There is no `SCB` on A/R, so there is no `CFSR`/`HFSR`/`BFAR`/`MMFAR` to read and no `PSP`
/// to sample; what the profile does have is the mode register. `mrs <rd>, cpsr` is the same
/// idiom [`crate::arch`]'s A/R port uses in `current_cpsr()`, so it is known to assemble for
/// this target rather than merely expected to.
///
/// In practice an A/R application never reaches this: the A/R port reports faults through its
/// own trap entry. It exists so that `app!` still links on such a target, and so that the
/// failure mode is a printed reason instead of a silent park.
#[cfg(all(target_arch = "arm", not(target_feature = "mclass")))]
pub fn hard_fault() -> ! {
    let cpsr: u32;
    let sp: u32;
    let lr: u32;
    unsafe {
        core::arch::asm!(
            "mrs {}, cpsr",
            out(reg) cpsr,
            options(nomem, nostack, preserves_flags)
        );
        core::arch::asm!("mov {}, sp", out(reg) sp, options(nomem, nostack));
        core::arch::asm!("mov {}, lr", out(reg) lr, options(nomem, nostack));
    }
    log_fmt(format_args!(
        "!!! fault (ARM A/R profile)  CPSR {:#010x} SP {:#010x} LR {:#010x}\r\n",
        cpsr, sp, lr
    ));
    log_fmt(format_args!(
        "    no CFSR/HFSR on this profile (those are M-profile SCB registers); \
         the A/R port reports through its own trap entry\r\n"
    ));
    park()
}

/// The body of the `DefaultHandler` that `app!` installs: an exception vector fired that
/// nothing handles. In a `cortex-m-rt` application that is what you see when the kernel's
/// trap entries are not wired into the vector table, so it is worth saying out loud.
pub fn unexpected_exception() -> ! {
    log_fmt(format_args!(
        "!!! unexpected exception: an unhandled vector fired\r\n"
    ));
    park()
}

/// The body of the `#[panic_handler]` that `app!` installs.
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    log_fmt(format_args!("!!! panic: {}\r\n", info.message()));
    park()
}

/// Declare the program: configuration, then `main` as **task 0**.
///
/// ```ignore
/// #![no_std]
/// #![no_main]
///
/// use rrkernel::{scheduler, thread, Slice};
///
/// rrkernel::app! {
///     clock: 8_000_000,            // core clock: turns the slice into a SysTick reload
///     slice: Slice::Millis(1),     // the timing contract, chosen once
///     stack: 1024,                 // per-task stack, carved from the kernel arena
///     log: |a| { let _ = rtt_target::rprintln!("{}", a); },    // optional, one line
///     main: {
///         thread::spawn(worker);
///         // `main` IS task 0: it runs here, and spawning or returning behave normally.
///         loop { scheduler::sleep_ticks(1000); }
///     }
/// }
///
/// fn worker() { /* ... */ }
/// ```
///
/// The application never writes `interrupt::free`, `main_body`, a vector table, a
/// `HardFault` or a `#[panic_handler]` — all of it is emitted here. What is *not* hidden
/// is anything you need to see: the fault handlers log the fault registers and the
/// faulting PC through `log:` before parking.
///
/// From the application's `Cargo.toml`: `cortex-m-rt` (for the entry point) and a
/// `panic = "abort"` profile. Do **not** also depend on `panic-halt` — this macro installs
/// the only panic handler.
#[cfg(target_arch = "arm")]
#[macro_export]
macro_rules! app {
    (
        clock: $clock:expr,
        slice: $slice:expr,
        stack: $stack:expr,
        log: $log:expr,
        main: $main:block
        $(,)?
    ) => {
        #[cortex_m_rt::entry]
        fn main() -> ! {
            $crate::app_support::install_log($log);

            // Mask interrupts across init — the tick must not fire halfway through
            // configuring the kernel — and, not incidentally, reference the `cortex-m`
            // crate: its `critical-section` implementation is what `rtt-target` and most
            // of the ecosystem link against, and a crate nothing references is a crate
            // the linker never pulls in. (Omitting this showed up as `undefined symbol:
            // _critical_section_1_0_acquire`.)
            let started =
                cortex_m::interrupt::free(|_cs| $crate::app_support::init($clock, $slice, $stack));
            if let Err(e) = started {
                $crate::app_support::log_fmt(format_args!("rrkernel: init failed: {:?}\r\n", e));
                // Also halt the debugger: if the application's own log sink is not up
                // yet, this is still unmissable with a probe attached — and without one
                // it faults into the handler below rather than pretending all is well.
                unsafe { core::arch::asm!("bkpt 0", options(nomem, nostack)) };
                $crate::app_support::park();
            }

            // `main` is task 0: the scheduler inherited this context at init time. When
            // this body returns, task 0 ends, exactly like any other task.
            $crate::scheduler::main_body(|| $main)
        }

        #[no_mangle]
        pub extern "C" fn HardFault() -> ! {
            $crate::app_support::hard_fault()
        }

        #[no_mangle]
        pub extern "C" fn DefaultHandler() -> ! {
            $crate::app_support::unexpected_exception()
        }

        #[panic_handler]
        fn __rrkernel_panic(info: &core::panic::PanicInfo) -> ! {
            $crate::app_support::panic(info)
        }
    };
}

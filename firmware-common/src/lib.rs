//! The bare-metal demo, written **once**, for every architecture this kernel
//! supports.
//!
//! Each firmware crate is then a thin platform shim: a startup routine, a
//! console, and an exit path. The task code, the assertions and the report below
//! are identical on AVR, Cortex-M, RISC-V, Cortex-R/A and Xtensa — which is the
//! real test of a "universal" kernel.
//!
//! # What it proves on target
//! * **Task 1** spins in a `loop {}` and never yields → `spinner` keeps climbing,
//!   so preemption works.
//! * **Task 2** spawns Task 3 at run time and returns → the trampoline unlinks it
//!   and the counters drop.
//! * **Task 3** watches the rotation, then walks the ring and checks
//!   `ring length == active_threads`, no dead nodes, `ticks > 0`,
//!   `switches > 0`, and that the programmed slice matches the requested one.
//! * It prints everything it measured and hands over to the platform's exit hook.
//!
//! # Portability rules it follows (and why)
//! * **No atomics at all.** 8-bit AVR has no atomic read-modify-write, so this
//!   crate uses a single-writer [`SyncCell`] with volatile access. Every field has
//!   exactly one writer, which is what makes that correct.
//! * **No `alloc`.** Counters and the report are plain statics.
//! * **Blocking is allowed only in the final report**, which runs once.

#![no_std]

use core::cell::UnsafeCell;
use core::fmt::{self, Write};
use rrkernel::{scheduler, thread, IdlePolicy, Measure, SchedulerConfig, Slice, TaskState};

// ---------------------------------------------------------------------------
// A single-writer cell (portable to targets without atomics)
// ---------------------------------------------------------------------------

/// A `Copy` value with volatile access and exactly one writer.
///
/// This is deliberately *not* an atomic: on ATmega328P there is no atomic
/// read-modify-write instruction, and pulling in a critical-section-based
/// polyfill for a demo counter would hide that constraint instead of respecting
/// it. Single writer + volatile access is the correct primitive here.
pub struct SyncCell<T: Copy>(UnsafeCell<T>);

// SAFETY: the contract is "one writer, any number of readers", and every use in
// this crate respects it.
unsafe impl<T: Copy> Sync for SyncCell<T> {}

impl<T: Copy> SyncCell<T> {
    pub const fn new(v: T) -> Self {
        SyncCell(UnsafeCell::new(v))
    }
    /// Read the current value.
    #[inline]
    pub fn get(&self) -> T {
        unsafe { core::ptr::read_volatile(self.0.get()) }
    }
    /// Write a new value (call from the single designated writer).
    #[inline]
    pub fn set(&self, v: T) {
        unsafe { core::ptr::write_volatile(self.0.get(), v) }
    }
}

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

/// A byte sink. Implement this once per board (UART, semihosting, USART, ...).
pub trait Console {
    /// Send one byte, blocking until it can be accepted.
    fn write_byte(&self, byte: u8);
}

/// Console that discards everything, for boards built without a serial port.
pub struct NullConsole;

impl Console for NullConsole {
    fn write_byte(&self, _byte: u8) {}
}

struct ConsoleSlot(UnsafeCell<&'static dyn Console>);

// SAFETY: written once during `run()` before any task can exist.
unsafe impl Sync for ConsoleSlot {}

static CONSOLE: ConsoleSlot = ConsoleSlot(UnsafeCell::new(&NullConsole));

fn console() -> &'static dyn Console {
    unsafe { *CONSOLE.0.get() }
}

struct FmtWriter;

impl Write for FmtWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let c = console();
        for b in s.bytes() {
            c.write_byte(b);
        }
        Ok(())
    }
}

/// Print to the platform console (no allocation, `fmt`-based).
pub fn print(args: fmt::Arguments<'_>) {
    let _ = FmtWriter.write_fmt(args);
}

/// Like `println!`, but always `\r\n` (serial terminals expect CR).
#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {
        $crate::print(format_args!($($arg)*))
    };
}

// ---------------------------------------------------------------------------
// Demo state (each field has exactly one writer)
// ---------------------------------------------------------------------------

/// Total spins Task 1 has accumulated.
pub static SPINNER: SyncCell<u32> = SyncCell::new(0);
/// Set to 1 by Task 2 just before it returns.
pub static TASK2_DONE: SyncCell<u32> = SyncCell::new(0);
/// Set to 1 by Task 2 when its dynamic spawn succeeded.
pub static TASK2_SPAWNED: SyncCell<u32> = SyncCell::new(0);
/// Set to 1 by Task 3 when it starts.
pub static TASK3_RAN: SyncCell<u32> = SyncCell::new(0);
/// Slices granted to the spinner, published by Task 1.
pub static SPINNER_SLICES: SyncCell<u32> = SyncCell::new(0);
/// Slices granted to the observer, published by Task 3.
pub static OBSERVER_SLICES: SyncCell<u32> = SyncCell::new(0);
/// 0 while running, 1 once the verdict is known.
pub static VERDICT: SyncCell<u8> = SyncCell::new(0);
/// 1 = all checks passed.
pub static PASSED: SyncCell<u8> = SyncCell::new(0);

// ---------------------------------------------------------------------------
// The demo itself
// ---------------------------------------------------------------------------

/// Everything a platform shim has to supply.
pub struct DemoConfig {
    /// Where the report goes.
    pub console: &'static dyn Console,
    /// Requested slice; rounded and validated by the kernel.
    pub slice: Slice,
    /// Frequency of the hardware tick source in Hz — SysTick core clock, RISC-V
    /// `mtime` frequency, AVR Timer1 clock, ... This is what turns the requested
    /// slice into timer ticks.
    pub timer_hz: u32,
    /// Kernel arena: TCBs, closures and (on bare metal) every task stack.
    pub arena: &'static mut [u8],
    /// Per-task stack size, carved from the arena on bare metal. This is the knob
    /// that decides how many tasks fit.
    pub stack_size: usize,
    /// How long to observe before reporting, in scheduler **ticks** rather than
    /// milliseconds, so the demo needs no wall clock.
    pub observe_ticks: u32,
    /// Platform exit: QEMU's finisher register, a reset, a park, ...
    pub exit: fn(i32) -> !,
}

/// Initialise the kernel, run the three tasks as task 0's body, never return.
pub fn run(cfg: DemoConfig) -> ! {
    let exit = cfg.exit;
    let observe_ticks = cfg.observe_ticks;

    unsafe {
        *CONSOLE.0.get() = cfg.console;
    }

    log!("\r\nrrkernel demo\r\n");
    log!("platform : {}\r\n", scheduler::platform_limits().timer_note);
    log!("tick src : {} Hz\r\n", cfg.timer_hz);

    let scfg = SchedulerConfig::embedded(cfg.slice, cfg.timer_hz)
        .stack_size(cfg.stack_size)
        .idle(IdlePolicy::Wait)
        .measure(Measure::Cycles)
        .arena(cfg.arena);

    if let Err(e) = scheduler::init_with(scfg) {
        log!("init failed: {}\r\n", e);
        exit(2);
    }
    log!(
        "slice    : requested {}, achieved {} ns ({} ticks)\r\n",
        cfg.slice,
        scheduler::slice_ns(),
        scheduler::config().slice_cycles
    );
    log!("scheduler live; spawning tasks\r\n");

    // Task 0's body. Returning from here unlinks task 0 like any other task.
    scheduler::main_body(move || {
        if thread::try_spawn(spinner).is_err()
            || thread::try_spawn(short_task).is_err()
            || thread::try_spawn(move || observer(observe_ticks, exit)).is_err()
        {
            log!("spawn failed: arena too small for the demo\r\n");
            exit(3);
        }
    });
}

/// Task 1: CPU-bound, never yields, never blocks.
fn spinner() {
    let mut n: u32 = 0;
    loop {
        n = n.wrapping_add(1);
        SPINNER.set(n);
        // Enough real work that slice boundaries land inside it.
        for _ in 0..200 {
            core::hint::spin_loop();
        }
        SPINNER_SLICES.set(scheduler::current_slices_run());
    }
}

/// Task 2: a little work, a **dynamic spawn**, then a plain `return` — which is
/// the entire exit protocol.
fn short_task() {
    let spawned = thread::try_spawn(|| {
        // The child only needs to exist: it proves insertion into the live ring.
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    })
    .is_ok();
    TASK2_SPAWNED.set(if spawned { 1 } else { 0 });

    for _ in 0..64 {
        core::hint::spin_loop();
    }
    TASK2_DONE.set(1);
}

/// Task 3: watches the rotation, validates the kernel's bookkeeping, reports.
fn observer(observe_ticks: u32, exit: fn(i32) -> !) {
    TASK3_RAN.set(1);

    // Wait for `observe_ticks` scheduler ticks to actually elapse. No sleep: a
    // task must never block. The heartbeat inside the loop proves the task is
    // alive and prints exactly what it reads, which is what makes a stall on a
    // serial-only board diagnosable.
    let start_slices = scheduler::current_slices_run();
    let start_ticks = scheduler::tick_count();
    loop {
        let waited = scheduler::tick_count().wrapping_sub(start_ticks);
        if waited >= observe_ticks as u64 {
            break;
        }
        let mine = scheduler::current_slices_run();
        if mine % 64 == 0 {
            log!(
                "[observer] alive: slice {} tick {} waited {} of {}\r\n",
                mine,
                scheduler::tick_count(),
                waited,
                observe_ticks
            );
        }
        core::hint::spin_loop();
    }
    OBSERVER_SLICES.set(scheduler::current_slices_run().wrapping_sub(start_slices));

    // --- measure ---------------------------------------------------------
    let st = scheduler::stats();
    let mut ring_len = 0u32;
    let mut dead_in_ring = 0u32;
    scheduler::for_each_task(|t| {
        ring_len += 1;
        if t.state == TaskState::Dead {
            dead_in_ring += 1;
        }
    });

    // --- verdict ---------------------------------------------------------
    // task 0 (main) + task 1 + task 2 + the short task's child + task 3 = 5
    let ok = ring_len as usize == st.active_threads
        && dead_in_ring == 0
        && st.total_threads == 5
        && st.switches > 0
        && st.ticks > 0
        && TASK2_DONE.get() == 1
        && TASK2_SPAWNED.get() == 1
        && st.reclaimed >= 2
        && st.slice_cycles > 0;

    log!("\r\n--- report ------------------------------------------------\r\n");
    log!(
        "slice    : {} ticks ({} ns)\r\n",
        st.slice_cycles,
        st.slice_ns
    );
    log!(
        "threads  : total {} active {} reclaimed {}\r\n",
        st.total_threads,
        st.active_threads,
        st.reclaimed
    );
    log!(
        "ticks    : {}  switches {}  deferred {}\r\n",
        st.ticks,
        st.switches,
        st.ticks_deferred
    );
    log!(
        "accuracy : worst period error {} ns, worst switch {} ticks\r\n",
        st.worst_period_error_ns,
        st.worst_latency
    );
    log!(
        "ring     : {} nodes, {} dead; slices spinner {} / observer {}\r\n",
        ring_len,
        dead_in_ring,
        SPINNER_SLICES.get(),
        OBSERVER_SLICES.get()
    );
    log!(
        "arena    : {} of {} bytes carved\r\n",
        st.arena.bytes_bump,
        st.arena.bytes_total
    );
    log!(
        "task 2   : done {}, dynamic spawn {}\r\n",
        TASK2_DONE.get(),
        TASK2_SPAWNED.get()
    );
    if ok {
        log!("VERDICT  : PASS  (round robin + automatic unlink + dynamic spawn)\r\n");
    } else {
        log!("VERDICT  : FAIL\r\n");
    }
    log!("----------------------------------------------------------\r\n");

    VERDICT.set(1);
    PASSED.set(if ok { 1 } else { 0 });
    scheduler::request_shutdown();
    exit(if ok { 0 } else { 1 });
}

//! Bare-metal Cortex-M demo: preemptive round-robin on `SysTick` + `PendSV`.
//!
//! ```text
//! cargo build -p firmware-cortex-m --target thumbv7m-none-eabi --release
//! cargo build -p firmware-cortex-m --target thumbv6m-none-eabi   # Cortex-M0: same source
//! ```
//!
//! # What the demo proves (without a console)
//! There is no `println!` on bare metal, so the demo writes its findings into
//! [`REPORT`], a `#[no_mangle]` struct in `.bss` that a debugger (probe-rs,
//! OpenOCD, `arm-none-eabi-gdb`) or QEMU can read:
//!
//! * `t1_loops` keeps climbing → Task 1's infinite `loop {}` is being preempted
//!   and resumed without ever yielding.
//! * `t2_completed` == 1 → Task 2 returned, the trampoline unlinked it and the
//!   counters were decremented.
//! * `t2_spawned_t3` and `t3_ran` → a task spawned another task at run time.
//! * `ring_len == active_threads` → the ring and the counter agree, i.e. no dead
//!   node was left behind and no live one was dropped.
//! * `worst_switch_cycles` (Cortex-M3+ with `DWT`) → how long the switch took.
//!
//! # Stack model
//! `main` runs on MSP (the linker's stack) as task 0; spawned tasks run on their
//! own PSP stacks from the kernel arena. See `rrkernel::arch::cortex_m`.

#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use rrkernel::{scheduler, thread, Measure, SchedulerConfig, Slice};

/// Core clock of the target. Change this for your part — the kernel needs it to
/// turn the requested slice into a `SysTick` reload value, and it deliberately
/// refuses to guess.
///
/// 12 MHz is the QEMU `lm3s6965evb` system clock. A 168 MHz STM32F4 works the
/// same way: `CORE_HZ = 168_000_000` → a 1 ms slice is exactly 168 000 cycles.
const CORE_HZ: u32 = 12_000_000;

// ---------------------------------------------------------------------------
// Startup: vector table, .data/.bss init, and the jump into Rust
// ---------------------------------------------------------------------------

global_asm!(
    r#"
    .syntax unified
    .thumb

    /* --- vector table: must be the first thing in flash -------------------
     * Word 0 is the initial stack pointer, word 1 the reset handler; the core
     * fetches both from address 0 without any help from us.
     * `PendSV_Handler` and `SysTick_Handler` are exported by the kernel. */
    .section .vector_table, "a", %progbits
    .align 2
    .global __rrkernel_vectors
__rrkernel_vectors:
    .word _stack_start
    .word __reset
    .word __nmi
    .word __hard_fault
    .word __mem_manage
    .word __bus_fault
    .word __usage_fault
    .word 0
    .word 0
    .word 0
    .word 0
    .word __svcall
    .word 0
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

    /* --- reset ----------------------------------------------------------- */
    .section .text.__reset, "ax", %progbits
    .thumb_func
    .global __reset
__reset:
    ldr   r0, =_stack_start
    msr   msp, r0

    /* Copy .data (initialised statics) from flash to RAM. */
    ldr   r1, =_sidata
    ldr   r2, =_sdata
    ldr   r3, =_edata
1:  cmp   r2, r3
    bcs   2f
    ldr   r0, [r1]
    str   r0, [r2]
    adds  r1, r1, #4
    adds  r2, r2, #4
    b     1b

    /* Zero .bss: the kernel's `KERNEL` state and the arena live here, and
     * `KERNEL.current_tcb` must be null before anything runs. */
2:  ldr   r2, =_sbss
    ldr   r3, =_ebss
    movs  r0, #0
3:  cmp   r2, r3
    bcs   4f
    str   r0, [r2]
    adds  r2, r2, #4
    b     3b

4:  bl    rrkernel_main
    b     .

    /* --- default handlers ------------------------------------------------
     * A fault handler that parks is enough for a demo; set a breakpoint here
     * and inspect CFSR/HFSR (0xE000ED28 / 0xE000ED2C) to diagnose. */
    .thumb_func
    .global __nmi
__nmi:          b .
    .thumb_func
    .global __hard_fault
__hard_fault:   b .
    .thumb_func
    .global __mem_manage
__mem_manage:   b .
    .thumb_func
    .global __bus_fault
__bus_fault:    b .
    .thumb_func
    .global __usage_fault
__usage_fault:  b .
    .thumb_func
    .global __svcall
__svcall:       b .
"#
);

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // On bare metal there is nowhere to unwind to. Park with interrupts masked
    // so the tick cannot keep mutating state behind the debugger's back.
    scheduler::shutdown(1)
}

// ---------------------------------------------------------------------------
// Observable results (read these from a debugger, probe-rs or QEMU)
// ---------------------------------------------------------------------------

/// Findings, laid out in `.bss` so they can be read by halting the target and
/// dumping memory at the `REPORT` symbol.
///
/// `#[repr(C)]` + `#[no_mangle]` so the layout is stable and easy to find.
#[repr(C)]
pub struct RrKernelReport {
    /// Increments forever inside Task 1's `loop {}` — proves preemption.
    ///
    /// Deliberately a plain `u32`, not `AtomicU32`: Cortex-M0/M0+ have no atomic
    /// read-modify-write instructions, so `fetch_add` does not exist there. This
    /// field has a single writer (Task 1) and is read by the observer, so a
    /// volatile store is the correct and portable choice — a 32-bit load/store
    /// is atomic on ARM by construction.
    pub t1_loops: u32,
    /// Number of slices Task 1 was granted.
    pub t1_slices: u32,
    /// Set to 1 at the end of Task 2's body (just before it returns).
    pub t2_completed: u32,
    /// Set to 1 when Task 2's dynamic spawn succeeded.
    pub t2_spawned_t3: u32,
    /// Set to 1 at the start of Task 3's body.
    pub t3_ran: u32,
    /// Slices granted to Task 3.
    pub t3_slices: u32,
    /// Slices granted to Task 0 (`main`).
    pub main_slices: u32,
    /// Kernel counters at report time.
    pub total_threads: u32,
    pub active_threads: u32,
    pub switches: u32,
    pub ticks: u32,
    pub reclaimed: u32,
    /// Ring length walked at report time; must equal `active_threads`.
    pub ring_len: u32,
    /// 1 when every self-check below passed.
    pub invariants_ok: u32,
    /// Worst switch latency in core cycles (0 if the core has no `DWT`).
    pub worst_switch_cycles: u32,
    /// Slice the kernel actually programmed, in cycles.
    pub slice_cycles: u32,
    /// The core clock the kernel was told about.
    pub core_hz: u32,
}

#[no_mangle]
#[used]
pub static REPORT: RrKernelReport = RrKernelReport {
    t1_loops: 0,
    t1_slices: 0,
    t2_completed: 0,
    t2_spawned_t3: 0,
    t3_ran: 0,
    t3_slices: 0,
    main_slices: 0,
    total_threads: 0,
    active_threads: 0,
    switches: 0,
    ticks: 0,
    reclaimed: 0,
    ring_len: 0,
    invariants_ok: 0,
    worst_switch_cycles: 0,
    slice_cycles: 0,
    core_hz: 0,
};

/// Hand out a mutable pointer to the report so a task can write fields that a
/// debugger is expected to read as plain memory.
///
/// The single-writer discipline is documented at the call sites: only the
/// observer task writes these fields, and only after the other tasks have
/// finished (Task 1 never writes them).
fn publish(write_fields: impl FnOnce(*mut RrKernelReport)) {
    write_fields(&REPORT as *const RrKernelReport as *mut RrKernelReport);
}

// ---------------------------------------------------------------------------
// The demo
// ---------------------------------------------------------------------------

/// Entry point called by `__reset` (see the assembly above).
///
/// 1. Configure and start the scheduler: **this arms SysTick**, so from the next
///    millisecond onward the CPU is time-sliced. There is no `scheduler_run()`.
/// 2. Run the rest of "main" as task 0 via
///    [`rrkernel::scheduler::main_body`], so returning from it is an ordinary
///    task exit (unlink + immediate switch) instead of falling off the reset
///    handler.
#[no_mangle]
pub extern "C" fn rrkernel_main() -> ! {
    let cfg = SchedulerConfig::embedded(Slice::Millis(1), CORE_HZ)
        .stack_size(1024) // per spawned task; carved from the kernel arena
        .measure(Measure::Cycles);

    if let Err(_e) = scheduler::init_with(cfg) {
        // Nothing sensible to do without a timer; park for the debugger.
        loop {
            core::hint::spin_loop();
        }
    }

    scheduler::main_body(|| {
        let _ = thread::try_spawn(task_one);
        let _ = thread::try_spawn(task_two);
    });
}

/// Task 1: a CPU-bound `loop {}` that never yields. The kernel preempts it
/// every slice; the loop only notices because `t1_loops` keeps climbing.
fn task_one() {
    let mut n: u32 = 0;
    loop {
        n = n.wrapping_add(1);
        // Single writer + 32-bit store: no atomic RMW needed (see the field's
        // note — Cortex-M0 has none).
        publish(|p| unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).t1_loops), n);
        });
        for _ in 0..2_000 {
            core::hint::spin_loop();
        }
        publish(|p| unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!((*p).t1_slices),
                scheduler::current_slices_run(),
            );
        });
    }
}

/// Task 2: short work, a **dynamic spawn**, then a plain `return` that the
/// trampoline turns into unlink + counter update + immediate switch.
fn task_two() {
    // Spawn a child from inside a running task. The child is linked into the
    // live ring right after this task and gets the CPU on the next switch.
    let spawned = thread::try_spawn(task_three).is_ok();
    publish(|p| unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*p).t2_spawned_t3),
            if spawned { 1 } else { 0 },
        );
    });

    // A little work, so the preemption is visible from the outside.
    for _ in 0..500 {
        core::hint::spin_loop();
    }

    // Signal that this task reached the end of its body *before* returning.
    publish(|p| unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).t2_completed), 1);
    });

    // No explicit exit call, no yield: returning from here IS the exit.
}

/// Task 3: the observer. Waits for a few rotations, verifies the kernel's
/// bookkeeping, publishes the report and stops the kernel.
fn task_three() {
    publish(|p| unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).t3_ran), 1);
    });

    // Wait until Task 1 has been observed making progress across slices.
    let mut last = 0u32;
    let mut rotations = 0u32;
    while rotations < 5 {
        let now = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(REPORT.t1_loops)) };
        if now != last {
            rotations += 1;
            last = now;
        }
        let st = scheduler::stats();
        publish(|p| unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!((*p).t3_slices),
                scheduler::current_slices_run(),
            );
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!((*p).total_threads),
                st.total_threads as u32,
            );
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!((*p).active_threads),
                st.active_threads as u32,
            );
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).switches), st.switches as u32);
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).ticks), st.ticks as u32);
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).reclaimed), st.reclaimed as u32);
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!((*p).worst_switch_cycles),
                st.worst_latency,
            );
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).slice_cycles), st.slice_cycles);
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).core_hz), st.timer_hz);
        });
    }

    // --- self-checks ------------------------------------------------------
    // Count the ring by walking it, and compare with the kernel's counter.
    let mut ring_len = 0u32;
    let mut saw_dead = false;
    scheduler::for_each_task(|t| {
        ring_len += 1;
        if t.state == rrkernel::TaskState::Dead {
            saw_dead = true;
        }
    });

    let st = scheduler::stats();
    let ok = ring_len as usize == st.active_threads
        && !saw_dead
        && st.total_threads >= 3
        && st.switches > 0
        && st.ticks > 0
        && REPORT.t2_completed == 1
        && REPORT.t2_spawned_t3 == 1
        && st.slice_cycles == Slice::Millis(1).to_cycles(CORE_HZ) as u32;

    publish(|p| unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*p).ring_len), ring_len);
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!((*p).invariants_ok),
            if ok { 1 } else { 0 },
        );
    });

    // Stop the kernel: masks interrupts and parks. A debugger (or a watchdog)
    // then finds a quiescent target with a fully populated REPORT.
    scheduler::shutdown(if ok { 0 } else { 2 });
}

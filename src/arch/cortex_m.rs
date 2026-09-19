//! Cortex-M port: `SysTick` as the cycle-exact slice timer, `PendSV` as the
//! context switch, and hand-written Thumb assembly for the register frame.
//!
//! # Why this is the "real" backend
//! Everything else in this kernel is `core` code; on Cortex-M the timing is
//! *hardware* timing:
//!
//! * the slice is `SysTick->LOAD + 1` core cycles — 1 ms at 168 MHz is exactly
//!   168 000 cycles, not "about a millisecond";
//! * `PendSV` restores the next task and the handler then **writes
//!   `SysTick->VAL = 0`**, which clears `COUNTFLAG` and restarts the counter, so
//!   every task gets a full slice and the period cannot drift;
//! * the switch itself is `stmdb`/`ldmia` of R4–R11 plus `bx 0xFFFFFFFD`, i.e.
//!   a few dozen cycles (~0.2 µs at 168 MHz), with jitter of a handful of cycles
//!   unless a higher-priority IRQ is in flight.
//!
//! # Stack model (documented because it is unusual but deliberate)
//! * **Task 0 (`main`)** runs on **MSP** — the stack the linker set up. The
//!   kernel marks its TCB with [`crate::tcb::TCB_FLAG_USE_MSP`] and `PendSV`
//!   returns with `0xFFFFFFF9` for it, `0xFFFFFFFD` (PSP) for everything else.
//!   That is what lets `scheduler_init()` adopt the running `main` without
//!   copying or relocating a live C stack (which cannot be done safely).
//!   Consequence, and it is the standard small-RTOS arrangement: MSP doubles as
//!   the kernel/ISR stack, so keep `main` light or make it the idle loop and put
//!   heavy work in spawned tasks.
//! * **Every spawned task** gets its own stack from the kernel arena and runs on
//!   PSP.
//!
//! # Frame layout built for a new task (ascending addresses)
//! ```text
//!   [R4..R11] [R0=tcb] [R1] [R2] [R3] [R12] [LR] [PC=trampoline] [xPSR]
//!    ^ sp points here                 (hardware-pushed exception frame)   ^ stack top
//! ```
//! `R0` carries the trampoline's argument because `PendSV` only ever touches
//! R4–R11 (plus the hardware frame restore), and `xPSR` has the Thumb bit set,
//! so the very first `bx 0xFFFFFFFD` lands in Rust at
//! [`crate::trampoline::task_trampoline`].
//!
//! # FPU
//! Lazy FPU stacking is **disabled** here (`FPCCR.ASPEN = LSPEN = 0`) and no
//! S-registers are saved, so a task that uses hardware floating point must not
//! run on this backend unless you extend `PendSV` with `vstmdb`/`vldmia` for
//! S16–S31. `thumbv7m-none-eabi` (soft-float) is the recommended demo target.
//! This is called out rather than hidden: a lazy-stacked FPU frame combined with
//! a stack switch is the classic source of silent corruption.

use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::tcb::{TaskControlBlock, KERNEL, TCB_FLAG_USE_MSP, TCB_SP_OFFSET};
use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

// ---------------------------------------------------------------------------
// Memory-mapped registers
// ---------------------------------------------------------------------------

const SYST_CSR: u32 = 0xE000_E010; // control/status
const SYST_RVR: u32 = 0xE000_E014; // reload value
const SYST_CVR: u32 = 0xE000_E018; // current value (writing any value clears it)
const SHPR3: u32 = 0xE000_ED20; // SysTick + PendSV priorities
const ICSR: u32 = 0xE000_ED04; // interrupt control/state (PENDSVSET)
const FPCCR: u32 = 0xE000_EF34; // floating-point context control
const DEMCR: u32 = 0xE000_EDFC; // debug exception/monitor control (TRCENA)
const DWT_CTRL: u32 = 0xE000_1000;
const DWT_CYCCNT: u32 = 0xE000_1004;

const ICSR_PENDSVSET: u32 = 1 << 28;
const SYST_CSR_ENABLE: u32 = 1 << 0;
const SYST_CSR_TICKINT: u32 = 1 << 1;
const SYST_CSR_CLKSOURCE: u32 = 1 << 2; // processor clock (1) vs external ref (0)

/// `SysTick` has a 24-bit reload register.
const MAX_SLICE_CYCLES: u64 = 0x00FF_FFFF;
/// Below this the slice is shorter than the switch itself.
///
/// The switch costs roughly: PendSV entry ~12–16 cycles + `stmdb`/`ldmia` ~28
/// cycles + the Rust `schedule_next` (a few tens of cycles) ≈ 60–100 cycles, so
/// 100 cycles is the smallest slice that still leaves most of the slice to the
/// task. Asking for less is rejected rather than silently producing a kernel
/// that spends all its time switching.
const MIN_SLICE_CYCLES: u64 = 100;

/// Timer frequency used for cycle↔time conversion *before* init.
///
/// The core clock is a configuration input on bare metal — there is no portable
/// way for a kernel to discover it. Once `scheduler_init*` has run,
/// `KERNEL.config.timer_hz` holds the real value and this nominal one is unused.
const NOMINAL_HZ: u32 = 16_000_000;

#[inline]
unsafe fn rd(addr: u32) -> u32 {
    read_volatile(addr as *const u32)
}
#[inline]
unsafe fn wr(addr: u32, value: u32) {
    write_volatile(addr as *mut u32, value);
}

/// The core clock the kernel was configured with (0 before init).
fn configured_hz() -> u32 {
    unsafe { *KERNEL.config.get() }.timer_hz
}

/// 0 = unknown, 1 = this core has a usable `DWT->CYCCNT`, 2 = it does not.
static DWT_OK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// `DWT->CYCCNT` at the previous timer tick, for period-accuracy measurement.
static LAST_TICK_CYCLES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn cycles_to_ns_with(hz: u32, cycles: u32) -> u64 {
    if hz == 0 {
        return 0;
    }
    ((cycles as u128) * 1_000_000_000u128 / hz as u128) as u64
}

pub fn cycles_to_ns(cycles: u32) -> u64 {
    let hz = configured_hz();
    cycles_to_ns_with(if hz == 0 { NOMINAL_HZ } else { hz }, cycles)
}

// ---------------------------------------------------------------------------
// Critical sections: PRIMASK
// ---------------------------------------------------------------------------

/// Snapshot of `PRIMASK` taken when the critical section was entered.
pub type CriticalToken = u32;

/// Mask interrupts, returning the previous `PRIMASK`.
///
/// `PRIMASK` is used rather than `BASEPRI` because `BASEPRI` does not exist on
/// ARMv6-M (Cortex-M0/M0+), and this kernel stays single-source across
/// `thumbv6m`, `thumbv7m` and `thumbv8m.main`. Masking all interrupts rather
/// than only those below a level is also what keeps `spawn` safe when called
/// from a device ISR.
///
/// # Safety
/// Pair with exactly one [`critical_exit`].
pub unsafe fn critical_enter() -> CriticalToken {
    let primask: u32;
    asm!("mrs {}, PRIMASK", out(reg) primask, options(nomem, nostack, preserves_flags));
    asm!("cpsid i", options(nomem, nostack, preserves_flags));
    primask
}

/// Restore the interrupt state saved by [`critical_enter`].
///
/// # Safety
/// `token` must come from the matching [`critical_enter`].
pub unsafe fn critical_exit(token: CriticalToken) {
    if token & 1 == 0 {
        asm!("cpsie i", options(nomem, nostack, preserves_flags));
    }
}

// ---------------------------------------------------------------------------
// The context switch: SysTick_Handler (pends) + PendSV_Handler (switches)
// ---------------------------------------------------------------------------

/// `SysTick` handler.
///
/// Deliberately trivial: it only sets `PENDSVSET` so the actual switch happens
/// in `PendSV`, which has the lowest priority and therefore never preempts
/// another exception handler. Doing the switch in the tick handler instead would
/// make it re-entrant with respect to every other ISR.
///
/// When `Measure::Cycles` is configured *and* the core has a cycle counter, the
/// handler also measures how far this tick landed from its ideal boundary. That
/// number — not a claim — is the kernel's timing accuracy on this part, and it
/// is visible through `scheduler::stats().worst_period_error_ns`.
#[no_mangle]
pub unsafe extern "C" fn SysTick_Handler() {
    if DWT_OK.load(core::sync::atomic::Ordering::Relaxed) == 1
        && (*KERNEL.config.get()).measure == crate::tcb::Measure::Cycles
    {
        let now = rd(DWT_CYCCNT);
        let last = LAST_TICK_CYCLES.load(core::sync::atomic::Ordering::Relaxed);
        if last != 0 {
            let period = now.wrapping_sub(last);
            let want = (*KERNEL.config.get()).slice_cycles;
            let err = period.abs_diff(want);
            crate::scheduler::record_period_error(err);
        }
        LAST_TICK_CYCLES.store(now, core::sync::atomic::Ordering::Relaxed);
    }
    // Count the tick and wake anything whose deadline has passed. Without this call the
    // kernel's clock never advances on Cortex-M: `scheduler::tick_count()` stays 0
    // forever, `sleep_ticks()` can never wake (its deadline sweep lives in `on_tick`),
    // `try_lock_for` never times out, and every `ticks > 0` invariant fails — while the
    // machine looks perfectly healthy, because the switches still happen.
    unsafe { crate::scheduler::on_tick() };
    wr(ICSR, ICSR_PENDSVSET);
}

/// Kernel half of the switch, called from `PendSV_Handler` (symbol referenced by
/// the assembly via `sym`).
///
/// Interrupts are masked across the ring walk so a device ISR that calls
/// `thread::spawn` can never observe a half-updated ring. The mask window is a
/// few dozen cycles and does not delay the *next* tick: `SysTick` counts in
/// hardware, so only this switch is postponed, never the period.
///
/// It also reclaims finished tasks here, because `PendSV` runs on the kernel
/// stack (MSP) and can therefore safely release a dead task's PSP stack.
#[no_mangle]
unsafe extern "C" fn rrkernel_schedule_next() -> *mut TaskControlBlock {
    let guard = crate::critical::enter();
    crate::scheduler::reclaim_finished_tasks();
    let t0 = if DWT_OK.load(core::sync::atomic::Ordering::Relaxed) == 1 {
        rd(DWT_CYCCNT)
    } else {
        0
    };
    let next = crate::scheduler::schedule_next();
    if t0 != 0 && (*KERNEL.config.get()).measure == crate::tcb::Measure::Cycles {
        let dt = rd(DWT_CYCCNT).wrapping_sub(t0);
        crate::scheduler::record_latency(dt);
    }
    drop(guard);
    next
}

/// `PendSV` handler: the entire context switch.
///
/// Register discipline: `PendSV` is a naked handler with no prologue. It saves
/// the interrupted task's R4–R11 (the caller-saved registers are already in the
/// hardware exception frame on that task's stack), stores the resulting SP into
/// `[current_tcb + sp]`, asks Rust for the next task, restores that task's
/// R4–R11, zeroes `SysTick->VAL` so the successor gets a **full** slice, and
/// returns with the right `EXC_RETURN` for the successor's stack (PSP for
/// spawned tasks, MSP for the adopted `main`).
///
/// `thumbv6m`-safe subset only: `mrs/msr`, `stmdb/ldmia`, `cmp`+`beq`, `lsls`,
/// `wfi`, `bx` — no `it` blocks, no `cbz` (which is Thumb-2/ARMv7-M only), no
/// Thumb-2-only forms.
#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn PendSV_Handler() {
    core::arch::naked_asm!(
        "isb",
        // r1 = current_tcb (KERNEL.current_tcb is at offset 0 of KERNEL).
        "ldr   r3, ={kernel}",
        "ldr   r1, [r3]",
        "cmp   r1, #0",
        "beq   2f",
        // r0 = the interrupted task's stack pointer. PSP for spawned tasks, but **MSP
        // for task 0**: the context `adopt_current_task` inherited runs on MSP, and the
        // hardware pushed its exception frame there. Reading PSP for task 0 publishes 0
        // as its saved SP, and the restore below then loads MSP = 0 — an exception
        // return from address 0. That is precisely the failure both targets produced:
        // "misaligned PC is UNPREDICTABLE" on QEMU and an imprecise BusFault inside
        // PendSV on an STM32F103.
        "mrs   r0, psp",
        "ldrb  r2, [r1, #{flags_off}]",
        "lsls  r2, r2, #31",
        "beq   1f",
        "mrs   r0, msp",
        "1:",
        // Save the callee-saved registers and publish the new SP.
        //
        // R4-R11 are saved as two 16-bit `stmia`s rather than one Thumb-2
        // `stmdb {r4-r11}`: the 16-bit LDM/STM encoding can only address r0-r7,
        // so the high half is moved through r4-r7 after those have been stored.
        // This costs ~4 extra cycles on Cortex-M3+ and is what makes the same
        // source valid on Cortex-M0/M0+ as well.
        "subs  r0, r0, #32",
        "stmia r0!, {{r4-r7}}",
        "mov   r4, r8",
        "mov   r5, r9",
        "mov   r6, r10",
        "mov   r7, r11",
        "stmia r0!, {{r4-r7}}",
        "subs  r0, r0, #32",
        "str   r0, [r1, #{sp_off}]",
        // Task 0's stack *is* MSP, and the `bl {sched}` below pushes on MSP: left
        // alone it lands on top of the frame just saved and destroys it. Move MSP
        // below that frame first. This is not a permanent change — the restore path
        // sets the final MSP/PSP from the successor's saved SP before the return.
        "ldrb  r2, [r1, #{flags_off}]",
        "lsls  r2, r2, #31",
        "beq   2f",
        "msr   msp, r0",
        // 2: pick the next task in Rust (skips Dead nodes, updates the
        //    counters and the current-task register).
        "2:",
        "bl    {sched}",
        "cmp   r0, #0",
        "beq   4f",
        // r0 = next TCB (r0 is kept intact through the restore so `flags` can
        // be read afterwards), r1 = its saved SP.
        "ldr   r1, [r0, #{sp_off}]",
        "ldmia r1!, {{r4-r7}}",
        "ldr   r2, [r1, #0]",
        "mov   r8, r2",
        "ldr   r3, [r1, #4]",
        "mov   r9, r3",
        "ldr   r2, [r1, #8]",
        "mov   r10, r2",
        "ldr   r3, [r1, #12]",
        "mov   r11, r3",
        "adds  r1, r1, #16",
        // Bit 0 of `flags` selects the MSP (adopted main) vs PSP path.
        // `lsls` is used instead of `tst` because `tst reg, reg` does not exist
        // on ARMv6-M.
        "ldrb  r2, [r0, #{flags_off}]",
        "lsls  r2, r2, #31",
        "beq   3f",
        "msr   msp, r1",
        // Restart the slice counter: zeroing VAL clears COUNTFLAG and reloads
        // the countdown, so the successor always gets a complete slice and the
        // period never accumulates drift.
        "movs  r2, #0",
        "ldr   r3, ={syst_cvr}",
        "str   r2, [r3]",
        // Exception return, thread mode, MSP (0xFFFFFFF9).
        "ldr   r0, =0xFFFFFFF9",
        "bx    r0",
        // 3: PSP task: same slice reset, PSP exception return (0xFFFFFFFD).
        "3:",
        "msr   psp, r1",
        "movs  r2, #0",
        "ldr   r3, ={syst_cvr}",
        "str   r2, [r3]",
        "ldr   r0, =0xFFFFFFFD",
        "bx    r0",
        // 4: nothing is runnable. Stay in the handler and wait for an interrupt
        //    (a new task being spawned re-pends PendSV); re-check after each
        //    wake-up instead of spinning.
        "4:",
        "wfi",
        "b     2b",
        kernel = sym KERNEL,
        sched = sym rrkernel_schedule_next,
        sp_off = const TCB_SP_OFFSET,
        flags_off = const crate::tcb::TCB_FLAGS_OFFSET,
        syst_cvr = const SYST_CVR,
    );
}

// The vector table of a `cortex-m-rt` application (and of most device crates) names the
// core exceptions *without* the `_Handler` suffix and installs a table of its own.
// Without these two thunks such an application routes `SysTick` and `PendSV` to
// `DefaultHandler`, so the kernel's tick and switch entry points are never entered —
// and the symptom is merely "nothing happens", which is a miserable thing to debug.
//
// They are single branches, so the handler is entered exactly as if the vector had
// pointed at it directly: no extra stack frame, no ABI difference, nothing to unwind.
// Both conventions therefore work:
//
//   * a hand-written table points at `SysTick_Handler` / `PendSV_Handler`
//     (`firmware-cortex-m` does this);
//   * `cortex-m-rt` points at `SysTick` / `PendSV` and lands in the same code.
//
// `.thumb_func` is load-bearing: it sets bit 0 in the symbol's value, which is what a
// vector table entry needs in order to enter Thumb state rather than taking a
// UsageFault on the first instruction.
core::arch::global_asm!(
    r#"
    .syntax unified
    .thumb

    .section .text.SysTick_linkname, "ax", %progbits
    .thumb_func
    .global SysTick
    .type SysTick, %function
SysTick:
    b     SysTick_Handler

    .section .text.PendSV_linkname, "ax", %progbits
    .thumb_func
    .global PendSV
    .type PendSV, %function
PendSV:
    b     PendSV_Handler
"#
);

/// Ask for an immediate context switch. The pending `PendSV` runs as soon as
/// this critical section (if any) exits — not on the next tick.
pub fn request_switch() {
    unsafe {
        wr(ICSR, ICSR_PENDSVSET);
        // Ensure the store lands before the next instruction that could depend
        // on it (documented requirement for PENDSVSET).
        asm!("dsb", options(nomem, nostack, preserves_flags));
    }
}

// ---------------------------------------------------------------------------
// Timer configuration
// ---------------------------------------------------------------------------

pub fn platform_limits() -> PlatformLimits {
    let hz = configured_hz();
    let hz = if hz == 0 { NOMINAL_HZ } else { hz };
    PlatformLimits {
        min_slice_ns: cycles_to_ns_with(hz, MIN_SLICE_CYCLES as u32),
        max_slice_ns: Some(cycles_to_ns_with(hz, MAX_SLICE_CYCLES as u32)),
        timer_hz: hz,
        timer_note: "SysTick, processor clock, 24-bit reload (cycle-exact slice)",
    }
}

pub fn plan_timer(cfg: &SchedulerConfig) -> Result<crate::arch::TimerPlan, ConfigError> {
    let hz = cfg.timer_hz;
    if hz == 0 {
        // Without the core clock the reload value cannot be computed, and
        // guessing would silently give the wrong slice.
        return Err(ConfigError::TimerClockRequired);
    }
    let cycles = match cfg.slice {
        Slice::Cycles(c) => c as u64,
        other => other.to_cycles(hz),
    };
    if cycles == 0 {
        return Err(ConfigError::ZeroSlice);
    }
    if cycles < MIN_SLICE_CYCLES {
        return Err(ConfigError::SliceBelowPlatformMinimum {
            requested_ns: cycles_to_ns_with(hz, cycles as u32),
            minimum_ns: cycles_to_ns_with(hz, MIN_SLICE_CYCLES as u32),
        });
    }
    if cycles > MAX_SLICE_CYCLES {
        return Err(ConfigError::SliceAboveTimerRange {
            requested_ns: cycles_to_ns_with(hz, cycles.min(u32::MAX as u64) as u32),
            maximum_ns: cycles_to_ns_with(hz, MAX_SLICE_CYCLES as u32),
        });
    }
    Ok(crate::arch::TimerPlan {
        slice_ns: cycles_to_ns_with(hz, cycles as u32),
        slice_cycles: cycles as u32,
        timer_hz: hz,
    })
}

/// Start `SysTick` and hand the tick path to it. Called once, from
/// `scheduler::init_with`, before any task has been spawned.
pub fn start_timer(plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        // Disable the FPU's lazy stacking *before* any switch can happen: with
        // ASPEN/LSPEN set, exception entry/exit would push/pop an FPU frame on
        // whichever stack PSP happens to point at, which is exactly the wrong
        // stack once PendSV starts switching stacks.
        wr(FPCCR, rd(FPCCR) & !((1 << 31) | (1 << 30)));

        wr(SYST_CSR, 0); // stop while we reprogram it
        wr(SYST_RVR, plan.slice_cycles - 1); // period = RVR + 1 cycles
        wr(SYST_CVR, 0); // clear counter + COUNTFLAG

        // Priorities: PendSV lowest (so it never preempts another handler),
        // SysTick just above it (so device IRQs still preempt the tick).
        let mut shpr3 = rd(SHPR3);
        shpr3 |= 0x00FF_0000; // PendSV  = 0xFF
        shpr3 &= !0x00C0_0000; // SysTick = 0x40 (keeps the low bits clear)
        shpr3 |= 0x0040_0000;
        wr(SHPR3, shpr3);

        wr(
            SYST_CSR,
            SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE,
        );

        // Enable the cycle counter for latency measurement, if this core has
        // one (Cortex-M3+; M0/M0+ silently ignore the write).
        let ok = dwt_enable();
        DWT_OK.store(
            if ok { 1 } else { 2 },
            core::sync::atomic::Ordering::Relaxed,
        );

        asm!("dsb", options(nomem, nostack, preserves_flags));
    }
    Ok(())
}

/// Change the slice at run time: the counter is restarted, so the first slice
/// after a retune is a full one.
pub fn retune_timer(plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        let was_enabled = rd(SYST_CSR) & SYST_CSR_ENABLE != 0;
        wr(SYST_CSR, 0);
        wr(SYST_RVR, plan.slice_cycles - 1);
        wr(SYST_CVR, 0);
        if was_enabled {
            wr(
                SYST_CSR,
                SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE,
            );
        }
    }
    Ok(())
}

/// Enable `DWT->CYCCNT` if the core provides it. Returns whether it worked.
///
/// Cortex-M0/M0+ have no cycle counter; the write is ignored and the read-back
/// check catches that instead of producing bogus latency numbers.
pub fn dwt_enable() -> bool {
    unsafe {
        wr(DEMCR, rd(DEMCR) | (1 << 24)); // TRCENA
        wr(DWT_CTRL, rd(DWT_CTRL) | 1); // CYCCNTENA
        rd(DWT_CTRL) & 1 != 0
    }
}

/// Free-running core cycle counter, or 0 if the core has none.
pub fn cycles_now() -> u32 {
    unsafe { rd(DWT_CYCCNT) }
}

// ---------------------------------------------------------------------------
// Task construction
// ---------------------------------------------------------------------------

/// Push one word onto a downward-growing stack.
#[inline]
unsafe fn push(sp: &mut *mut u32, value: u32) {
    *sp = (*sp).sub(1);
    write_volatile(*sp, value);
}

/// Turn the **calling** context into task 0.
///
/// Task 0 keeps using MSP, so nothing is relocated and no live C stack is
/// copied. `sp` is deliberately left null: `PendSV` saves it the first time this
/// task is switched away from, and by the time the scheduler can select it again
/// that save has already happened. (A switch *to* a task that was never switched
/// away from is impossible: the ring is circular, so returning to a task
/// requires having left it.)
pub unsafe fn adopt_current_task(tcb: *mut TaskControlBlock) -> Result<(), ConfigError> {
    unsafe {
        (*tcb).flags = TCB_FLAG_USE_MSP;
        (*tcb).stack_base = core::ptr::null_mut();
        (*tcb).stack_size = 0;
        (*tcb).sp = core::ptr::null_mut();
    }
    Ok(())
}

/// Build a fresh task: stack from the kernel arena plus an initial exception
/// frame whose program counter is [`crate::trampoline::task_trampoline`].
pub unsafe fn create_task(
    tcb: *mut TaskControlBlock,
    stack_size: usize,
) -> Result<(), crate::SpawnError> {
    // Worst-case alignment padding, so the 8-byte-aligned top is always inside
    // the block.
    let size = stack_size.max(256) + 8;
    let block = {
        let guard = crate::critical::enter();
        let p = unsafe { (*crate::scheduler::arena()).alloc(size, 8) };
        drop(guard);
        p
    };
    let block = match block {
        Some(p) => p,
        None => return Err(crate::SpawnError::ArenaExhausted),
    };

    unsafe {
        (*tcb).stack_base = block;
        (*tcb).stack_size = size - 8;
        (*tcb).flags = 0; // PSP task

        // The hardware exception frame must be 8-byte aligned; ARM stacks grow
        // down, so the top is the high address.
        let top = ((block as usize + size) & !7) as *mut u32;
        let mut sp = top;

        // --- hardware-pushed frame (highest address first) -----------------
        push(&mut sp, 0x0100_0000); // xPSR: Thumb bit set, no flags
        let entry = crate::trampoline::task_trampoline as *const () as usize as u32;
        push(&mut sp, entry | 1); // PC (Thumb bit required)
        push(&mut sp, 0xFFFF_FFF9); // LR: sentinel, the trampoline never returns
        push(&mut sp, 0); // R12
        push(&mut sp, 0); // R3
        push(&mut sp, 0); // R2
        push(&mut sp, 0); // R1
        push(&mut sp, tcb as u32); // R0 = trampoline's argument (the TCB)

        // --- software-saved frame (R4-R11), where `sp` ends up --------------
        for _ in 0..8 {
            push(&mut sp, 0);
        }
        (*tcb).sp = sp as *mut u8;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Port plumbing
// ---------------------------------------------------------------------------

pub fn mark_running() {}

/// A finished task, on bare metal: the next `PendSV` is already pending (the
/// trampoline asked for it), so just idle until it takes the CPU away. This
/// stack is recycled by the kernel's deferred-free path *after* the switch, so
/// control never returns here.
pub fn exit_current_task_forever() -> ! {
    idle_forever()
}

/// Nothing is runnable: sleep until an interrupt arrives (`wfi`). The tick keeps
/// running, so a task spawned from an ISR is picked up immediately.
pub fn idle_forever() -> ! {
    loop {
        unsafe {
            asm!("wfi", options(nomem, nostack, preserves_flags));
        }
    }
}

/// Stop the kernel: mask interrupts and park. There is nothing to "exit" on bare
/// metal, so `code` is ignored — a firmware that wants a reset should issue one
/// itself (`SCB->AIRCR`).
pub fn shutdown(_code: i32) -> ! {
    unsafe {
        let _ = critical_enter(); // mask, forever
    }
    loop {
        core::hint::spin_loop();
    }
}

/// No per-task OS resources to release on bare metal: the stack and TCB are both
/// arena blocks and are freed by the same deferred-free pass.
pub unsafe fn on_task_reclaimed(_tcb: *mut TaskControlBlock) {}

/// Bare metal has no second thread, so nothing needs pinning: the reclaimer only
/// ever runs from `PendSV`, on the kernel stack, after the switch.
pub unsafe fn is_pinned(_tcb: *mut TaskControlBlock) -> bool {
    false
}

/// Whether a blocking call (`sleep`, `lock`, `park`) is guaranteed to have taken the caller off
/// the CPU **before it returns**.
///
/// `true` on this portrue here: the pending `PendSV` is taken the moment the blocking critical section ends.
///
/// This matters because a task measures its own sleep around the call: if the switch is
/// asynchronous, the task can observe `elapsed == 0` and think it woke up early, even though
/// the kernel booked the wait correctly. `examples/sleep_fidelity.rs` reports this property
/// and gates its strict assertions on it.
pub fn parks_synchronously() -> bool {
    true
}

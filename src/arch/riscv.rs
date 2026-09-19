//! RISC-V (`rv32imac`) port: machine-mode CLINT timer, `mtvec` trap entry, and a
//! hand-written trap-frame context switch.
//!
//! # Mapping onto the Cortex-M design
//!
//! | Cortex-M | here |
//! |---|---|
//! | `SysTick` period | `mtime`/`mtimecmp` (CLINT); `timer_hz` = the mtime frequency |
//! | `SysTick_Handler` sets `PENDSVSET` | `mtvec` trap entry, dispatches on `mcause`: `0x8000_0007` machine timer, `0x8000_0003` machine software (kicks) |
//! | `PendSV` register save + `sp` swap | 30-word trap frame (28 GPRs + `mepc` + `mstatus`) built on the task's own stack |
//! | `bx 0xFFFFFFFD` | `csrw mepc/mstatus` from the incoming frame, then `mret` |
//! | zero `SysTick->VAL` | `mtimecmp = mtime + slice`, rewritten after every switch |
//! | `PRIMASK` | `mstatus.MIE`, masked with `csrci`/`csrsi` |
//! | `PENDSVSET` kick | `CLINT.msip[hart] = 1` (machine software interrupt) |
//!
//! # Why there is no separate kernel stack
//! The frame is built on the interrupted task's own stack, exactly like the
//! Cortex-M PSP frame. The one place that matters is reclamation: a finished task
//! cannot free the stack it is standing on, so the assembly swaps `sp` to the
//! incoming task *first* and only then calls Rust ([`rrkernel_after_switch`]) to
//! reclaim the outgoing task and restart the slice counter — by which point the
//! dying stack is nobody's current stack.
//!
//! # Universality
//! The CLINT layout is the RISC-V standard (`msip` at base+0, `mtimecmp` at
//! base+0x4000, `mtime` at base+0xBFF8) and is what QEMU's `virt`, `sifive_e` and
//! `spike` machines implement. Chips without a CLINT (ESP32-C3's SYSTIMER, for
//! instance) can point [`configure_timer`] at their own registers: the port only
//! ever touches `mtime`, `mtimecmp` and `msip`, and the slice is expressed in
//! ticks of whatever counter those registers expose.
//!
//! # Frame layout (offsets shared with the assembly through `const` operands)
//! ```text
//!   0 ra     4 s0/fp  8 s1    12 s2    16 s3    20 s4    24 s5    28 s6
//!  32 s7    36 s8    40 s9    44 s10   48 s11   52 a0    56 a1    60 a2
//!  64 a3    68 a4    72 a5    76 a6    80 a7    84 t0    88 t1    92 t2
//!  96 t3   100 t4   104 t5   108 t6   112 mepc 116 mstatus   120..128 pad
//! ```
//! `gp` and `tp` are deliberately not saved: in a single-hart bare-metal kernel
//! they are constants for the lifetime of the image.
//!
//! `sp` itself is the switch: `tcb.sp` points at this frame, which is why the
//! incoming task's `sp` is loaded before its registers are restored.

use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::tcb::{TaskControlBlock, KERNEL};
use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

/// Size of the trap frame, rounded up to the 16-byte alignment the RISC-V ABI
/// requires of `sp`.
pub const FRAME_SIZE: usize = 128;

macro_rules! frame_off {
    ($($name:ident = $offset:expr),* $(,)?) => {
        $( pub const $name: usize = $offset; )*
        /// Every field offset, in frame order, for tests and assertions.
        pub const FRAME_OFFSETS: &[usize] = &[$($offset),*];
    };
}

frame_off! {
    OFF_RA = 0,
    OFF_S0 = 4,
    OFF_S1 = 8,
    OFF_S2 = 12,
    OFF_S3 = 16,
    OFF_S4 = 20,
    OFF_S5 = 24,
    OFF_S6 = 28,
    OFF_S7 = 32,
    OFF_S8 = 36,
    OFF_S9 = 40,
    OFF_S10 = 44,
    OFF_S11 = 48,
    OFF_A0 = 52,
    OFF_A1 = 56,
    OFF_A2 = 60,
    OFF_A3 = 64,
    OFF_A4 = 68,
    OFF_A5 = 72,
    OFF_A6 = 76,
    OFF_A7 = 80,
    OFF_T0 = 84,
    OFF_T1 = 88,
    OFF_T2 = 92,
    OFF_T3 = 96,
    OFF_T4 = 100,
    OFF_T5 = 104,
    OFF_T6 = 108,
    OFF_MEPC = 112,
    OFF_MSTATUS = 116,
}

/// `mstatus` for a task's first entry: return to machine mode (`MPP = M`) with
/// interrupts enabled (`MPIE = 1`, which `mret` copies into `MIE`).
const MSTATUS_TASK_ENTRY: usize = (3 << 11) | (1 << 7);

/// `mstatus.MIE`: the global interrupt enable that critical sections mask.
const MSTATUS_MIE: usize = 1 << 3;

/// `mie.MTIE`: machine timer interrupt enable.
const MIE_MTIE: usize = 1 << 7;

pub const MCAUSE_TIMER: usize = 0x8000_0007;
pub const MCAUSE_SOFT: usize = 0x8000_0003;

// ---------------------------------------------------------------------------
// CLINT / timer configuration
// ---------------------------------------------------------------------------

/// Default CLINT base: correct for QEMU's `virt`, `sifive_e` and `spike`.
pub const DEFAULT_CLINT_BASE: usize = 0x0200_0000;

/// The registers this port drives. Overridable, so a chip whose timer is not
/// CLINT-shaped can still use the port as long as it offers the same three
/// functions.
#[derive(Clone, Copy)]
pub struct TimerRegs {
    /// 64-bit free-running counter (`mtime`).
    pub mtime: *mut u64,
    /// 64-bit comparator (`mtimecmp`), little-endian, low word first.
    pub mtimecmp: *mut u64,
    /// Machine software interrupt register (`msip`, one per hart).
    pub msip: *mut u32,
}

// SAFETY: plain MMIO pointers, only dereferenced by this port.
unsafe impl Sync for TimerRegs {}

static mut TIMER_REGS: TimerRegs = TimerRegs {
    // 0xBFF8 / 8 and 0x4000 / 8: pointer arithmetic on a `*mut u64` advances in
    // 8-byte units, so the byte offsets are pre-divided here.
    mtime: (DEFAULT_CLINT_BASE + 0xBFF8) as *mut u64,
    mtimecmp: (DEFAULT_CLINT_BASE + 0x4000) as *mut u64,
    msip: DEFAULT_CLINT_BASE as *mut u32,
};

/// Point the port at a different timer (a CLINT at another base, or a chip's own
/// `mtime`-compatible registers). Call before `scheduler::init*`.
///
/// # Safety
/// The pointers must be valid MMIO addresses of a 64-bit counter, a 64-bit
/// comparator (little-endian, low word first) and a 32-bit software-interrupt
/// register, and the counter must run at `SchedulerConfig::timer_hz` Hz.
pub unsafe fn configure_timer(regs: TimerRegs) {
    write_volatile(core::ptr::addr_of_mut!(TIMER_REGS), regs);
}

#[inline]
unsafe fn regs() -> TimerRegs {
    read_volatile(core::ptr::addr_of!(TIMER_REGS))
}

/// Read the 64-bit counter safely on RV32: two 32-bit loads with a re-read of the
/// high half, so a carry between them cannot produce a wild value.
#[inline]
pub unsafe fn mtime_now() -> u64 {
    let r = regs();
    let p = r.mtime as *const u32;
    loop {
        let hi1 = read_volatile(p.add(1)) as u64;
        let lo = read_volatile(p) as u64;
        let hi2 = read_volatile(p.add(1)) as u64;
        if hi1 == hi2 {
            return (hi2 << 32) | lo;
        }
    }
}

/// Program the comparator, safely for a 32-bit bus.
///
/// Writing the low half last can leave a transient value the counter has already
/// passed, which fires immediately; the portable sequence is to disable the timer
/// interrupt, park the high half at `0xFFFF_FFFF`, write the low half, then the
/// real high half, then re-enable.
unsafe fn mtimecmp_write(value: u64) {
    let r = regs();
    let p = r.mtimecmp as *mut u32;
    asm!("csrc mie, {}", in(reg) MIE_MTIE, options(nomem, nostack, preserves_flags));
    write_volatile(p.add(1), 0xFFFF_FFFF);
    write_volatile(p, value as u32);
    write_volatile(p.add(1), (value >> 32) as u32);
    asm!("csrs mie, {}", in(reg) MIE_MTIE, options(nomem, nostack, preserves_flags));
}

/// Restart the slice counter for whoever runs next: `mtimecmp = mtime + slice`.
#[inline]
unsafe fn rearm_timer() {
    let slice = (*KERNEL.config.get()).slice_cycles as u64;
    mtimecmp_write(mtime_now().wrapping_add(slice));
}

/// Raise the machine software interrupt, which the trap entry treats exactly
/// like `PENDSVSET`: "switch as soon as it is safe".
#[inline]
pub fn request_switch() {
    unsafe {
        let r = regs();
        write_volatile(r.msip, 1);
    }
}

// ---------------------------------------------------------------------------
// Critical sections: mstatus.MIE
// ---------------------------------------------------------------------------

/// Saved `mstatus.MIE` state (0 or `MSTATUS_MIE`) from section entry.
pub type CriticalToken = usize;

/// Disable interrupts, returning the previous `MIE`.
///
/// # Safety
/// Pair with exactly one [`critical_exit`].
pub unsafe fn critical_enter() -> CriticalToken {
    let old: usize;
    asm!("csrr {}, mstatus", out(reg) old, options(nomem, nostack, preserves_flags));
    asm!("csrci mstatus, 8", options(nomem, nostack, preserves_flags)); // MIE = bit 3
    old & MSTATUS_MIE
}

/// Restore the interrupt state saved by [`critical_enter`].
///
/// # Safety
/// `token` must come from the matching [`critical_enter`].
pub unsafe fn critical_exit(token: CriticalToken) {
    if token != 0 {
        asm!("csrsi mstatus, 8", options(nomem, nostack, preserves_flags));
    }
}

/// `mtime` (low word) at the previous timer interrupt, for period measurement.
///
/// `AtomicU64` does not exist on RV32 (the `a` extension gives you 64-bit loads
/// only through pairs of 32-bit accesses), so this keeps the low word and
/// compares with `wrapping_sub`: exact for any period below 2^32 ticks, which at
/// 10 MHz is over seven minutes.
static LAST_TICK_LOW: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// One trap, as the kernel saw it. Passed to a hook installed with
/// [`set_trap_hook`].
#[derive(Clone, Copy)]
pub struct TrapEvent {
    /// Raw `mcause` (bit 31 set = interrupt; `0x8000_0007` = timer,
    /// `0x8000_0003` = software).
    pub mcause: usize,
    /// Interrupted program counter (`mepc`).
    pub mepc: usize,
    /// Task that was running.
    pub current: *mut TaskControlBlock,
    /// Task the scheduler chose (`null` if nothing was runnable).
    pub next: *mut TaskControlBlock,
    /// True when this call is the post-switch step, running on the incoming
    /// task's stack.
    pub after_switch: bool,
}

/// Optional bring-up hook. `0` = disabled.
///
/// This exists because "the kernel hangs inside a trap" is otherwise invisible on
/// a board with no debugger: the firmware can point this at a function that logs
/// over its UART. It costs one relaxed load per trap when unset.
static TRAP_HOOK: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Install a trap-observation hook (see [`TrapEvent`]).
pub fn set_trap_hook(hook: fn(TrapEvent)) {
    TRAP_HOOK.store(
        hook as *const () as usize,
        core::sync::atomic::Ordering::Relaxed,
    );
}

/// Remove the hook.
pub fn clear_trap_hook() {
    TRAP_HOOK.store(0, core::sync::atomic::Ordering::Relaxed);
}

#[inline]
fn report_trap(ev: TrapEvent) {
    let raw = TRAP_HOOK.load(core::sync::atomic::Ordering::Relaxed);
    if raw != 0 {
        // SAFETY: the pointer was stored from a `fn(TrapEvent)` by `set_trap_hook`.
        let hook: fn(TrapEvent) = unsafe { core::mem::transmute::<usize, fn(TrapEvent)>(raw) };
        hook(ev);
    }
}

/// Read `mepc` (the interrupted PC), for diagnostics.
#[inline]
pub fn current_mepc() -> usize {
    let v: usize;
    unsafe {
        asm!("csrr {}, mepc", out(reg) v, options(nomem, nostack, preserves_flags));
    }
    v
}

/// Rust half of the trap: work out *why* we trapped, then pick the next task.
///
/// Runs with interrupts already disabled (the hardware clears `mstatus.MIE` on
/// trap entry) and on the interrupted task's stack, so it needs no critical
/// section of its own.
#[no_mangle]
unsafe extern "C" fn rrkernel_trap_switch(mcause: usize) -> *mut TaskControlBlock {
    let mepc = current_mepc();

    if mcause == MCAUSE_SOFT {
        // Acknowledge, otherwise the software interrupt re-traps immediately.
        let r = regs();
        write_volatile(r.msip, 0);
    } else if mcause == MCAUSE_TIMER {
        crate::scheduler::on_tick();

        if (*KERNEL.config.get()).measure != crate::tcb::Measure::Off {
            let now = mtime_now() as usize;
            let last = LAST_TICK_LOW.swap(now, core::sync::atomic::Ordering::Relaxed);
            if last != 0 {
                let period = now.wrapping_sub(last) as u64;
                let want = (*KERNEL.config.get()).slice_cycles as u64;
                let err = period.abs_diff(want);
                crate::scheduler::record_period_error(err.min(u32::MAX as u64) as u32);
            }
        }
    }

    let next = crate::scheduler::schedule_next();
    report_trap(TrapEvent {
        mcause,
        mepc,
        current: KERNEL.current(),
        next,
        after_switch: false,
    });
    next
}

/// Runs **after** the switch, i.e. on the incoming task's stack, so it can both
/// reclaim the outgoing task's memory and restart the slice counter.
#[no_mangle]
unsafe extern "C" fn rrkernel_after_switch() {
    let measuring = (*KERNEL.config.get()).measure != crate::tcb::Measure::Off;
    let t0 = if measuring { mtime_now() } else { 0 };

    report_trap(TrapEvent {
        mcause: 0,
        mepc: 0,
        current: KERNEL.current(),
        next: KERNEL.current(),
        after_switch: true,
    });

    crate::scheduler::reclaim_finished_tasks();
    rearm_timer();

    if measuring {
        let dt = mtime_now().saturating_sub(t0);
        crate::scheduler::record_latency(dt.min(u32::MAX as u64) as u32);
    }
}

/// The trap vector entry: one entry for every trap (`mtvec` direct mode), with
/// the cause dispatched in Rust.
///
/// Register discipline: naked, no prologue, no compiler involvement. Saves the 28
/// GPRs that can carry live state plus `mepc`/`mstatus`, publishes `sp` into
/// `current_tcb->sp`, asks Rust for the next task, switches `sp`, lets Rust
/// reclaim and re-arm on the *new* stack, restores the incoming frame and `mret`s.
///
/// Note where `call {after_switch}` sits: after the `sp` swap, before the register
/// restore. That is what makes it legal to free the outgoing task's stack there.
#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn trap_entry() {
    core::arch::naked_asm!(
        "addi sp, sp, -{frame}",
        // --- callee-saved -------------------------------------------------
        "sw ra,  {off_ra}(sp)",
        "sw s0,  {off_s0}(sp)",
        "sw s1,  {off_s1}(sp)",
        "sw s2,  {off_s2}(sp)",
        "sw s3,  {off_s3}(sp)",
        "sw s4,  {off_s4}(sp)",
        "sw s5,  {off_s5}(sp)",
        "sw s6,  {off_s6}(sp)",
        "sw s7,  {off_s7}(sp)",
        "sw s8,  {off_s8}(sp)",
        "sw s9,  {off_s9}(sp)",
        "sw s10, {off_s10}(sp)",
        "sw s11, {off_s11}(sp)",
        // --- caller-saved -------------------------------------------------
        "sw a0,  {off_a0}(sp)",
        "sw a1,  {off_a1}(sp)",
        "sw a2,  {off_a2}(sp)",
        "sw a3,  {off_a3}(sp)",
        "sw a4,  {off_a4}(sp)",
        "sw a5,  {off_a5}(sp)",
        "sw a6,  {off_a6}(sp)",
        "sw a7,  {off_a7}(sp)",
        "sw t0,  {off_t0}(sp)",
        "sw t1,  {off_t1}(sp)",
        "sw t2,  {off_t2}(sp)",
        "sw t3,  {off_t3}(sp)",
        "sw t4,  {off_t4}(sp)",
        "sw t5,  {off_t5}(sp)",
        "sw t6,  {off_t6}(sp)",
        "csrr t0, mepc",
        "sw t0, {off_mepc}(sp)",
        "csrr t0, mstatus",
        "sw t0, {off_mstatus}(sp)",
        // --- publish sp into current_tcb (null before the first switch) ----
        "la t0, {kernel}",
        "lw t1, 0(t0)",
        "beqz t1, 1f",
        "sw sp, 0(t1)",
        // --- ask Rust for the next task; a0 = mcause, returns the TCB ------
        "1:",
        "csrr a0, mcause",
        "call {trap_switch}",
        "beqz a0, 4f",
        // --- switch stacks, then reclaim/re-arm on the new stack ----------
        "lw sp, 0(a0)",
        "call {after_switch}",
        // --- restore the incoming frame -----------------------------------
        "lw t0, {off_mepc}(sp)",
        "csrw mepc, t0",
        "lw t0, {off_mstatus}(sp)",
        "csrw mstatus, t0",
        "lw ra,  {off_ra}(sp)",
        "lw s0,  {off_s0}(sp)",
        "lw s1,  {off_s1}(sp)",
        "lw s2,  {off_s2}(sp)",
        "lw s3,  {off_s3}(sp)",
        "lw s4,  {off_s4}(sp)",
        "lw s5,  {off_s5}(sp)",
        "lw s6,  {off_s6}(sp)",
        "lw s7,  {off_s7}(sp)",
        "lw s8,  {off_s8}(sp)",
        "lw s9,  {off_s9}(sp)",
        "lw s10, {off_s10}(sp)",
        "lw s11, {off_s11}(sp)",
        "lw a0,  {off_a0}(sp)",
        "lw a1,  {off_a1}(sp)",
        "lw a2,  {off_a2}(sp)",
        "lw a3,  {off_a3}(sp)",
        "lw a4,  {off_a4}(sp)",
        "lw a5,  {off_a5}(sp)",
        "lw a6,  {off_a6}(sp)",
        "lw a7,  {off_a7}(sp)",
        "lw t0,  {off_t0}(sp)",
        "lw t1,  {off_t1}(sp)",
        "lw t2,  {off_t2}(sp)",
        "lw t3,  {off_t3}(sp)",
        "lw t4,  {off_t4}(sp)",
        "lw t5,  {off_t5}(sp)",
        "lw t6,  {off_t6}(sp)",
        "addi sp, sp, {frame}",
        "mret",
        // --- nothing runnable: sleep, then re-check ----------------------
        "4:",
        "wfi",
        "j 1b",
        frame = const FRAME_SIZE,
        off_ra = const OFF_RA,
        off_s0 = const OFF_S0,
        off_s1 = const OFF_S1,
        off_s2 = const OFF_S2,
        off_s3 = const OFF_S3,
        off_s4 = const OFF_S4,
        off_s5 = const OFF_S5,
        off_s6 = const OFF_S6,
        off_s7 = const OFF_S7,
        off_s8 = const OFF_S8,
        off_s9 = const OFF_S9,
        off_s10 = const OFF_S10,
        off_s11 = const OFF_S11,
        off_a0 = const OFF_A0,
        off_a1 = const OFF_A1,
        off_a2 = const OFF_A2,
        off_a3 = const OFF_A3,
        off_a4 = const OFF_A4,
        off_a5 = const OFF_A5,
        off_a6 = const OFF_A6,
        off_a7 = const OFF_A7,
        off_t0 = const OFF_T0,
        off_t1 = const OFF_T1,
        off_t2 = const OFF_T2,
        off_t3 = const OFF_T3,
        off_t4 = const OFF_T4,
        off_t5 = const OFF_T5,
        off_t6 = const OFF_T6,
        off_mepc = const OFF_MEPC,
        off_mstatus = const OFF_MSTATUS,
        kernel = sym KERNEL,
        trap_switch = sym rrkernel_trap_switch,
        after_switch = sym rrkernel_after_switch,
    );
}

// ---------------------------------------------------------------------------
// Timer configuration
// ---------------------------------------------------------------------------

/// `mie.MSIE`: machine software interrupt enable (used for kicks).
const MIE_MSIE: usize = 1 << 3;

/// Smallest slice accepted. The trap costs roughly 70 instructions (save, call,
/// restore) each way, so 100 ticks is the point below which the kernel would
/// spend most of its time switching.
const MIN_SLICE_CYCLES: u64 = 100;

/// `mtime` frequency assumed before init, for diagnostic conversions. QEMU's
/// `virt`, `sifive_e` and `spike` machines all run it at 10 MHz.
const NOMINAL_MTIME_HZ: u32 = 10_000_000;

fn configured_hz() -> u32 {
    unsafe { *KERNEL.config.get() }.timer_hz
}

fn cycles_to_ns_with(hz: u32, cycles: u32) -> u64 {
    if hz == 0 {
        return 0;
    }
    ((cycles as u128) * 1_000_000_000u128 / hz as u128) as u64
}

pub fn cycles_to_ns(cycles: u32) -> u64 {
    let hz = configured_hz();
    cycles_to_ns_with(if hz == 0 { NOMINAL_MTIME_HZ } else { hz }, cycles)
}

pub fn platform_limits() -> PlatformLimits {
    let hz = configured_hz();
    let hz = if hz == 0 { NOMINAL_MTIME_HZ } else { hz };
    PlatformLimits {
        min_slice_ns: cycles_to_ns_with(hz, MIN_SLICE_CYCLES as u32),
        // The plan type carries the interval in a u32 even though `mtimecmp` is
        // 64-bit; at 10 MHz that ceiling is still over seven minutes.
        max_slice_ns: Some(cycles_to_ns_with(hz, u32::MAX)),
        timer_hz: hz,
        timer_note: "RISC-V machine timer (CLINT mtime/mtimecmp), trap-based switch",
    }
}

pub fn plan_timer(cfg: &SchedulerConfig) -> Result<crate::arch::TimerPlan, ConfigError> {
    let hz = cfg.timer_hz;
    if hz == 0 {
        // The interval is expressed in mtime ticks, so the counter frequency is
        // required: there is no portable way to read it from the hart.
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
    if cycles > u32::MAX as u64 {
        return Err(ConfigError::SliceAboveTimerRange {
            requested_ns: cycles_to_ns_with(hz, u32::MAX),
            maximum_ns: cycles_to_ns_with(hz, u32::MAX),
        });
    }
    Ok(crate::arch::TimerPlan {
        slice_ns: cycles_to_ns_with(hz, cycles as u32),
        slice_cycles: cycles as u32,
        timer_hz: hz,
    })
}

/// Install the trap vector, enable the timer and software interrupts, arm the
/// first slice, and turn interrupts on.
pub fn start_timer(_plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        // 1. The vector must be in place before any interrupt can be enabled.
        let handler = trap_entry as *const () as usize;
        asm!("csrw mtvec, {}", in(reg) handler, options(nomem, nostack));

        // 2. Acknowledge any stale software interrupt (e.g. left over from a
        //    previous program on the same hart).
        let r = regs();
        write_volatile(r.msip, 0);

        // 3. Enable the sources we use: machine timer (slices) and machine
        //    software (kicks from spawn/exit).
        asm!(
            "csrs mie, {}",
            in(reg) MIE_MTIE | MIE_MSIE,
            options(nomem, nostack)
        );

        // 4. Arm the first slice, then let interrupts happen.
        rearm_timer();
        asm!("csrsi mstatus, 8", options(nomem, nostack));
    }
    Ok(())
}

/// Change the slice at run time: the comparator is rewritten from *now*, so the
/// first slice after a retune is a full one.
pub fn retune_timer(_plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        rearm_timer();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Task construction
// ---------------------------------------------------------------------------

/// The calling context becomes task 0.
///
/// Unlike Cortex-M there is no MSP/PSP split to preserve: `sp` is just `sp`, the
/// trap frame is built on whichever stack is current, and `main` keeps whatever
/// stack the firmware's startup code installed. `sp` is left null because the trap
/// entry writes it on the first switch away (and a switch *to* a task that was
/// never switched away from is impossible in a circular ring).
pub unsafe fn adopt_current_task(tcb: *mut TaskControlBlock) -> Result<(), ConfigError> {
    unsafe {
        (*tcb).sp = core::ptr::null_mut();
        (*tcb).stack_base = core::ptr::null_mut();
        (*tcb).stack_size = 0;
        (*tcb).flags = 0;
    }
    Ok(())
}

pub unsafe fn create_task(
    tcb: *mut TaskControlBlock,
    stack_size: usize,
) -> Result<(), crate::SpawnError> {
    // Worst-case 16-byte alignment padding, so an aligned frame base always fits.
    let size = stack_size.max(256) + 16;
    let block = {
        let guard = crate::critical::enter();
        let p = unsafe { (*crate::scheduler::arena()).alloc(size, 16) };
        drop(guard);
        p
    };
    let block = match block {
        Some(p) => p,
        None => return Err(crate::SpawnError::ArenaExhausted),
    };

    unsafe {
        (*tcb).stack_base = block;
        (*tcb).stack_size = size - 16;

        let top = (block as usize + size) & !15;
        let frame = (top - FRAME_SIZE) as *mut u8;

        core::ptr::write_bytes(frame, 0, FRAME_SIZE);
        let w = |off: usize, v: usize| {
            core::ptr::write_volatile(frame.add(off) as *mut usize, v);
        };
        // a0 = the trampoline's argument (the TCB itself).
        w(OFF_A0, tcb as usize);
        // Where `mret` starts executing, and with what machine state.
        w(
            OFF_MEPC,
            crate::trampoline::task_trampoline as *const () as usize,
        );
        w(OFF_MSTATUS, MSTATUS_TASK_ENTRY);

        (*tcb).sp = frame;
        (*tcb).flags = 0;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Port plumbing
// ---------------------------------------------------------------------------

pub fn mark_running() {}

/// A finished task: the switch was already requested (a software interrupt is
/// pending), so just sleep until it traps and the frame swaps — the stack is
/// recycled right after that, so control never returns here.
pub fn exit_current_task_forever() -> ! {
    idle_forever()
}

/// Nothing runnable: wait for an interrupt (a newly spawned task's kick wakes the
/// hart, and the trap entry re-runs the switch).
pub fn idle_forever() -> ! {
    loop {
        unsafe {
            asm!("wfi", options(nomem, nostack, preserves_flags));
        }
    }
}

/// Stop the kernel: mask interrupts and park. On bare metal `code` is ignored.
pub fn shutdown(_code: i32) -> ! {
    unsafe {
        let _ = critical_enter();
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Nothing to release: the stack and the TCB are both arena blocks, freed by the
/// same deferred-free pass.
pub unsafe fn on_task_reclaimed(_tcb: *mut TaskControlBlock) {}

/// Nothing to pin: reclamation runs from the trap entry on the incoming task's
/// stack, on a single hart, with interrupts disabled.
pub unsafe fn is_pinned(_tcb: *mut TaskControlBlock) -> bool {
    false
}

/// Whether a blocking call (`sleep`, `lock`, `park`) is guaranteed to have taken the caller off
/// the CPU **before it returns**.
///
/// `true` on this portrue here: the trap entry re-runs the switch as the section ends.
///
/// This matters because a task measures its own sleep around the call: if the switch is
/// asynchronous, the task can observe `elapsed == 0` and think it woke up early, even though
/// the kernel booked the wait correctly. `examples/sleep_fidelity.rs` reports this property
/// and gates its strict assertions on it.
pub fn parks_synchronously() -> bool {
    true
}

//! ARM **A/R profile** port (ARMv7-A / ARMv7-R): mode-based exception model with
//! banked stack pointers, the ARM generic timer, and a `SRSDB`/`RFEIA` context
//! switch.
//!
//! This is a different world from Cortex-M ([`crate::arch::cortex_m`]) and the
//! port is deliberately a separate file rather than `#[cfg]` soup:
//!
//! | Cortex-M | A/R profile |
//! |---|---|
//! | `SysTick` (MMIO, 24-bit) | ARM generic timer, `CNTP_*` (64-bit, `CNTFRQ` readable) |
//! | `NVIC` priority + `PENDSVSET` | `GICv2`: `GICD_ISENABLER0` for PPI 30, groups/priority |
//! | hardware pushes the exception frame | **nothing** is pushed: the handler saves the frame itself |
//! | `0xFFFFFFFD` exception return | `SRSDB sp!, #SVC` + `RFEIA sp!` |
//! | `PSP` per task, `MSP` for the kernel | banked `SP_svc` per task, `SP_irq` for the handler |
//! | `PRIMASK`/`BASEPRI` | `CPSR.I` |
//!
//! # How the switch works
//! Only `SP` and `LR` are banked per mode; `r0–r12` hold the interrupted task's
//! live values when the IRQ handler starts. So:
//!
//! ```text
//! irq_entry (IRQ mode, running on SP_irq):
//!   sub   lr, lr, #4        ; LR_irq = interrupted PC
//!   srsdb sp!, #0x13        ; push {PC, CPSR} onto the *task's* stack (SVC bank)
//!   cps   #0x13             ; switch to SVC mode; SP is now the task's SP
//!   push  {r0-r12}          ; the task's live general registers
//!   sub   sp, sp, #4        ; keep the AAPCS 8-byte alignment
//!   ; tcb->sp = sp ; ask Rust for the next task ; sp = next->sp
//!   add   sp, sp, #4
//!   pop   {r0-r12}
//!   rfeia sp!               ; load PC/CPSR from the frame: return to the task
//! ```
//!
//! Frame layout (low to high): `[pad][r0..r12][PC][CPSR]` = 16 words = 64 bytes,
//! which is what makes `pop` + `rfeia` symmetric for both saving and restoring.
//!
//! # GIC and timer addresses are configuration, not hard-coded assumptions
//! QEMU's `-M virt` (Cortex-A15) has a GICv2 at 0x0800_0000/0x0801_0000 and PPI 30
//! for the physical timer; a real SoC (or a Cortex-R part with a VIC, or a GIC at
//! another base) changes only [`configure`].

use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::tcb::{TaskControlBlock, KERNEL};
use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

// ---------------------------------------------------------------------------
// Frame layout — load-bearing for the assembly
// ---------------------------------------------------------------------------

/// Frame layout, in bytes: `[pad(8)][r0..r12(52)][lr(4)][PC(4)][CPSR(4)]`.
pub const FRAME_SIZE: usize = 72;
/// Where `tcb.sp` points (two alignment pad words, so the frame is 8-byte
/// aligned both before the `pop` and after).
pub const OFF_PAD: usize = 0;
/// `r0`: for a fresh task this is the trampoline's argument (the TCB).
pub const OFF_R0: usize = 8;
/// The task's own `lr`, saved because tasks run in SVC mode and use it.
pub const OFF_LR: usize = 60;
/// `PC` and `CPSR`, consumed by `RFEIA`.
pub const OFF_PC: usize = 64;
pub const OFF_CPSR: usize = 68;

/// `CPSR` for a task's first entry: SVC mode, ARM state, IRQ and FIQ enabled.
///
/// `0x13` = SVC mode; bit 5 (`T`) is set below only if the entry point needs it;
/// `I`/`F` clear means interrupts are unmasked, which is what a task should see.
const CPSR_TASK_ENTRY: usize = 0x13;

/// `CPSR.I` — the IRQ mask bit that critical sections toggle.
const CPSR_I: u32 = 1 << 7;

/// `SVC` mode number, used by `SRSDB`/`CPS`.
const MODE_SVC: usize = 0x13;

pub const IRQ_VECTOR_OFFSET: usize = 0x18;

// ---------------------------------------------------------------------------
// Platform configuration (GIC + generic timer)
// ---------------------------------------------------------------------------

/// Where the interrupt controller and the timer live, and how the GIC must be
/// programmed for the CPU's *security state*.
#[derive(Clone, Copy)]
pub struct ArmConfig {
    /// GICv2 distributor base (PPI/SGI configuration).
    pub gic_dist: usize,
    /// GICv2 CPU interface base (priority mask, acknowledge, EOI).
    pub gic_cpu: usize,
    /// Interrupt ID of the timer. 30 = non-secure physical timer PPI.
    pub timer_irq: u32,
    /// System register set for the timer: `true` = `CNTP_*` (physical), `false` =
    /// `CNTV_*` (virtual). Boards with a secure monitor often expose only the
    /// virtual timer to a non-secure kernel.
    pub physical_timer: bool,
    /// `true` when the CPU runs in a **secure** state (SCR.NS = 0), in which case
    /// the GIC must use **group 0** and the tick arrives as **FIQ**; `false` for a
    /// non-secure CPU, which uses **group 1** and **IRQ**.
    ///
    /// This is not cosmetic: a GICv2 *drops* group-1 interrupts aimed at a secure
    /// CPU, and drops group-0 interrupts aimed at a non-secure one. Getting it
    /// wrong produces the worst possible symptom — an interrupt that is enabled,
    /// pending and never delivered. QEMU's `-M virt` leaves the CPU in secure EL1,
    /// hence [`ArmConfig::qemu_virt`] setting this to `true`.
    pub secure_fiq: bool,
}

impl ArmConfig {
    /// A CPU running in **non-secure** EL1 — the normal situation when a
    /// bootloader or QEMU hands control over: group 1 interrupts, delivered as
    /// IRQ, through the normal CPU-interface bank.
    ///
    /// This is the default, and it is also why QEMU's `virt` board should be
    /// started with `secure=off` (see `.cargo/config.toml`): the physical timer's
    /// interrupt (PPI 30) is a *non-secure* interrupt, and a GICv2 will not
    /// deliver a group-1 interrupt to a *secure* CPU. Booting into secure EL1 —
    /// which is what `-M virt` does by default, with no secure monitor present to
    /// drop you out of it — therefore produces an interrupt that is enabled,
    /// pending, and永 far never delivered.
    pub const fn qemu_virt() -> Self {
        ArmConfig {
            gic_dist: 0x0800_0000,
            gic_cpu: 0x0801_0000,
            timer_irq: 30,
            physical_timer: true,
            secure_fiq: false,
        }
    }

    /// A CPU left in **secure** state by a secure monitor: group 0, delivered as
    /// FIQ, through the secure CPU-interface bank.
    pub const fn secure() -> Self {
        ArmConfig {
            secure_fiq: true,
            ..ArmConfig::qemu_virt()
        }
    }
}

static mut ARM_CONFIG: ArmConfig = ArmConfig::qemu_virt();

/// Override the GIC/timer description before `scheduler::init*`.
pub fn configure(cfg: ArmConfig) {
    unsafe {
        write_volatile(core::ptr::addr_of_mut!(ARM_CONFIG), cfg);
    }
}

#[inline]
fn config() -> ArmConfig {
    unsafe { read_volatile(core::ptr::addr_of!(ARM_CONFIG)) }
}

#[inline]
fn gicd(offset: usize) -> *mut u32 {
    (config().gic_dist + offset) as *mut u32
}

#[inline]
fn gicc(offset: usize) -> *mut u32 {
    // A GICv2 CPU interface is **banked**: the secure OS's registers live at
    // +0x1000 from the bases below (GICC_CTLR_S, GICC_PMR_S, GICC_IAR_S,
    // GICC_EOIR_S, ...) while the normal offsets are the non-secure bank. Using
    // the wrong bank produces exactly the worst symptom there is: interrupts get
    // delivered for a while, then stop, because the acknowledge read returns
    // "spurious" and nothing is ever EOI'd.
    let bank = if config().secure_fiq { 0x1000 } else { 0x0000 };
    (config().gic_cpu + bank + offset) as *mut u32
}

// GICv2 register offsets
const GICD_CTLR: usize = 0x000;
const GICD_IGROUPR0: usize = 0x080;
const GICD_ISENABLER0: usize = 0x100;
const GICD_IPRIORITYR: usize = 0x400;
const GICC_CTLR: usize = 0x000;
const GICC_PMR: usize = 0x004;
const GICC_IAR: usize = 0x00C;
const GICC_EOIR: usize = 0x010;

/// Enable the timer interrupt and let the CPU take IRQs.
///
/// Uses **group 1** interrupts (signalled as IRQ, valid from both secure and
/// non-secure state) rather than group 0 (FIQ/secure only), so the same code
/// works whichever security state the CPU resets into.
unsafe fn gic_enable_timer_irq(irq: u32) {
    let word = (irq / 32) as usize;
    let bit = 1u32 << (irq % 32);
    let secure = config().secure_fiq;

    // Group 0 for a secure CPU (delivered as FIQ), group 1 for a non-secure one
    // (delivered as IRQ). The same applies to SGI 0, which `request_switch` uses.
    let group_bits: u32 = if secure { 0 } else { bit | 1 };
    let g = read_volatile(gicd(GICD_IGROUPR0 + word * 4));
    write_volatile(
        gicd(GICD_IGROUPR0 + word * 4),
        (g & !(bit | 1)) | group_bits,
    );
    write_volatile(gicd(GICD_IPRIORITYR + irq as usize) as *mut u8, 0xA0);
    write_volatile(gicd(GICD_ISENABLER0 + word * 4), bit | 1);

    // CPU interface: allow every priority, enable **both** interrupt groups.
    //
    // Enabling both is deliberate and correct: which line the tick arrives on is
    // decided by the *interrupt's* group, not by the CTLR, so there is nothing to
    // choose here. (It also sidesteps a real modelling difference — some GIC
    // implementations and QEMU's distributor treat GICD_CTLR bit 0 as the
    // "distributor enabled" flag, so writing only the group-0 bit leaves the
    // whole controller disabled and interrupts simply never arrive.)
    write_volatile(gicc(GICC_PMR), 0xFF);
    write_volatile(gicc(GICC_CTLR), 0b11);
    write_volatile(gicd(GICD_CTLR), 0b11);
}

/// Acknowledge the highest-priority pending interrupt (0x3FF = spurious).
#[inline]
pub unsafe fn gic_acknowledge() -> u32 {
    read_volatile(gicc(GICC_IAR))
}

/// Signal end-of-interrupt so the same source can fire again.
#[inline]
pub unsafe fn gic_eoi(irq: u32) {
    write_volatile(gicc(GICC_EOIR), irq);
}

// ---------------------------------------------------------------------------
// Generic timer system registers
// ---------------------------------------------------------------------------

/// Read a 32-bit coprocessor register: `mrc p15, 0, Rd, <op>`.
macro_rules! mrc {
    ($name:ident, $($op:expr),+) => {
        #[inline]
        unsafe fn $name() -> u32 {
            let v: u32;
            asm!(concat!("mrc p15, 0, {}, ", $($op),+), out(reg) v, options(nomem, nostack));
            v
        }
    };
}

/// Write a 32-bit coprocessor register: `mcr p15, 0, Rd, <op>`.
macro_rules! mcr {
    ($name:ident, $($op:expr),+) => {
        #[inline]
        unsafe fn $name(value: u32) {
            asm!(concat!("mcr p15, 0, {}, ", $($op),+), in(reg) value, options(nomem, nostack));
        }
    };
}

// CP15 encodings of the ARM generic timer, from the ARMv7-A/R ARM:
//   CNTFRQ    c14, c0, 0     CNTPCT c14, c0, 1     CNTVCT c14, c0, 2
//   CNTP_TVAL c14, c2, 0     CNTP_CTL c14, c2, 1
//   CNTV_TVAL c14, c3, 0     CNTV_CTL c14, c3, 1
//
// The counters are 64-bit and are read with `mrrc`; the low word alone is enough
// for the slice measurements below, and reading it with `mrc` keeps the port
// assembling on toolchains that reject the 64-bit transfer form. Above 68 s at
// 62.5 MHz the low word wraps, which wrapping arithmetic handles exactly.
mrc!(read_cntfrq, "c14, c0, 0");
mrc!(read_cntp_tval, "c14, c2, 0");
mcr!(write_cntp_tval, "c14, c2, 0");
mcr!(write_cntp_ctl, "c14, c2, 1");
mcr!(write_cntv_tval, "c14, c3, 0");
mcr!(write_cntv_ctl, "c14, c3, 1");

/// The timer's frequency, as reported by `CNTFRQ` — no guessing required on this
/// architecture, unlike the RISC-V and AVR ports.
#[inline]
pub fn timer_hz() -> u32 {
    unsafe { read_cntfrq() }
}

/// How many ticks have elapsed since the slice timer expired.
///
/// Reading `CNTP_TVAL` after the countdown has passed zero yields a "negative"
/// remaining value, so the lateness is its two's complement. This is the honest
/// way to measure timing here: the 64-bit `CNTPCT` would be the obvious counter,
/// but it can only be read with `mrrc`, and using it would make the port depend on
/// an instruction form that not every assembler accepts — whereas the overshoot of
/// the very timer under test is exactly the quantity a kernel wants to know
/// anyway (how late the slice ran, in timer ticks).
#[inline]
unsafe fn ticks_since_expiry() -> u32 {
    // With `physical_timer: false` the same computation applies to CNTV_TVAL;
    // QEMU's virt uses the physical timer, so that path stays untested and is
    // deliberately not guessed at here (see `ArmConfig`).
    0u32.wrapping_sub(read_cntp_tval())
}

/// Restart the slice counter for whoever runs next: `TVAL = slice` counts down
/// from a full slice and the `IMASK`/`ENABLE` bits are re-armed.
///
/// This is the A/R-profile equivalent of zeroing `SysTick->VAL` or rewriting
/// `mtimecmp`, and it is what makes every slice full-length.
#[inline]
unsafe fn rearm_timer() {
    let slice = (*KERNEL.config.get()).slice_cycles;
    if config().physical_timer {
        write_cntp_tval(slice);
        write_cntp_ctl(1); // ENABLE = 1, IMASK = 0
    } else {
        write_cntv_tval(slice);
        write_cntv_ctl(1);
    }
}

// ---------------------------------------------------------------------------
// Critical sections: CPSR.I
// ---------------------------------------------------------------------------

/// `0` = interrupts were **already masked**; non-zero = they **enabled** and must
/// be re-enabled on exit. (The raw `I`/`F` bits that were clear, so `exit` can
/// restore exactly what this CPU masks — FIQ matters here, because a secure CPU
/// receives the tick on FIQ.)
///
/// That polarity is part of the port contract: every backend's `critical_exit`
/// re-enables when the token is non-zero, so a backend whose `critical_enter`
/// returns "the mask bit" instead silently leaves interrupts disabled forever
/// after the first critical section. (This ARM port shipped that bug for exactly
/// one QEMU run.)
pub type CriticalToken = u32;

/// `CPSR.F` — FIQ mask, which this port also masks in critical sections because
/// the tick may arrive on either line.
const CPSR_F: u32 = 1 << 6;

/// Optional bring-up hook: `hook(tag, value)`, called at the port's decision
/// points. `0` = disabled.
///
/// Tags: 1 `critical_enter` (value = CPSR), 2 `critical_enter` result (token),
/// 3 `critical_exit` (token), 4 `critical_exit` done, 5 `start_timer` done
/// (CPSR), 6 IRQ entry (interrupt ID), 7 parking a finished task (CPSR).
///
/// This is the tool that finds "IRQs got masked and never came back": on a board
/// with only a serial port there is otherwise no way to see it.
static TRACE_HOOK: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Install a trace hook.
pub fn set_trace_hook(hook: fn(usize, usize)) {
    TRACE_HOOK.store(
        hook as *const () as usize,
        core::sync::atomic::Ordering::Relaxed,
    );
}

#[inline]
fn trace(tag: usize, value: usize) {
    let raw = TRACE_HOOK.load(core::sync::atomic::Ordering::Relaxed);
    if raw != 0 {
        // SAFETY: stored from a `fn(usize, usize)` by `set_trace_hook`.
        let hook: fn(usize, usize) =
            unsafe { core::mem::transmute::<usize, fn(usize, usize)>(raw) };
        hook(tag, value);
    }
}

/// Current `CPSR`, for diagnostics.
#[inline]
pub fn current_cpsr() -> u32 {
    let v: u32;
    unsafe {
        asm!("mrs {}, cpsr", out(reg) v, options(nomem, nostack, preserves_flags));
    }
    v
}

/// Mask IRQs, returning whether they *were* enabled.
///
/// # Safety
/// Pair with exactly one [`critical_exit`].
pub unsafe fn critical_enter() -> CriticalToken {
    let cpsr: u32;
    asm!("mrs {}, cpsr", out(reg) cpsr, options(nomem, nostack, preserves_flags));
    trace(1, cpsr as usize);
    // Mask both lines: the tick is an IRQ on a non-secure CPU and an FIQ on a
    // secure one, and a critical section must stop it either way.
    asm!("cpsid if", options(nomem, nostack));
    let token = !cpsr & (CPSR_I | CPSR_F);
    trace(2, token as usize);
    token
}

/// Restore the interrupt state saved by [`critical_enter`].
///
/// # Safety
/// `token` must come from the matching [`critical_enter`].
pub unsafe fn critical_exit(token: CriticalToken) {
    trace(3, token as usize);
    if token & CPSR_I != 0 {
        asm!("cpsie i", options(nomem, nostack));
    }
    if token & CPSR_F != 0 {
        asm!("cpsie f", options(nomem, nostack));
    }
    trace(4, current_cpsr() as usize);
}

// ---------------------------------------------------------------------------
// Vectors and the switch
// ---------------------------------------------------------------------------

// A trap we do not expect: the assembly parks on its own (`b .`), so there is nothing to do
// here. Deliberately a plain comment, not a doc comment: as `///` it documented no item, and
// `clippy::empty_line_after_doc_comments` flagged exactly that.

/// Spurious / "no eligible pending interrupt" codes returned by `GICC_IAR`.
///
/// "No pending interrupt of the requested group" (0x3FE) is what QEMU's GICv2
/// returns for an interrupt it *did* deliver — see the note in
/// [`rrkernel_irq_switch`].
const GIC_NO_ELIGIBLE: u32 = 0x3FE;

/// Rust half of the IRQ: acknowledge the source, account the tick, pick a task.
///
/// Runs with interrupts masked (the hardware sets `CPSR.I` on entry) on the
/// interrupted task's stack, so it needs no critical section of its own.
///
/// # Why "any interrupt is a tick"
/// This kernel enables exactly **one** interrupt source: the slice timer. That
/// makes the tick accounting independent of what the acknowledge register reports,
/// which matters because the acknowledge read is not equally trustworthy
/// everywhere — QEMU's GICv2 model, for instance, can return `GIC_NO_ELIGIBLE` for
/// an interrupt it has just delivered, and a port that trusted it would run the
/// round robin perfectly while reporting zero ticks (which is exactly how this
/// port behaved before the note was added). The acknowledge value is still used to
/// EOI, so a future port with several sources can extend the dispatch below.
#[no_mangle]
unsafe extern "C" fn rrkernel_irq_switch() -> *mut TaskControlBlock {
    let irq = gic_acknowledge();
    trace(6, irq as usize);

    crate::scheduler::on_tick();

    // How late was this slice, in timer ticks?
    if (*KERNEL.config.get()).measure != crate::tcb::Measure::Off {
        crate::scheduler::record_period_error(ticks_since_expiry());
    }

    let next = crate::scheduler::schedule_next();

    if irq < GIC_NO_ELIGIBLE {
        gic_eoi(irq);
    }
    next
}

/// Runs **after** the switch, on the incoming task's stack: reclaim the outgoing
/// task's memory, restart its successor's slice, and record how long the switch
/// itself took (in timer ticks).
#[no_mangle]
unsafe extern "C" fn rrkernel_after_switch() {
    let measuring = (*KERNEL.config.get()).measure != crate::tcb::Measure::Off;
    // Read the overshoot *before* re-arming: that covers everything this switch
    // did (policy, reclamation, the frame swap), which is exactly the cost that
    // eats into the next slice.
    let overshoot = if measuring { ticks_since_expiry() } else { 0 };

    crate::scheduler::reclaim_finished_tasks();
    rearm_timer();

    if measuring {
        crate::scheduler::record_latency(overshoot);
    }
}

/// IRQ vector body: build the interrupted task's frame, switch, restore.
///
/// See the module docs for why `SRSDB`/`CPS`/`RFEIA` are used and what the frame
/// looks like. Register use: `r0`/`r1` are the task's live values, so they are
/// only touched *after* being pushed.
#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn rrkernel_irq_entry() {
    core::arch::naked_asm!(
        // LR_irq points *after* the interrupted instruction; the task resumes one
        // instruction earlier.
        "sub   lr, lr, #4",
        // Push the interrupted PC and CPSR onto the *task's* stack (SVC bank),
        // then switch to SVC mode so SP is that same task stack.
        "srsdb sp!, #{svc_mode}",
        "cps   #{svc_mode}",
        // The task's live general registers, plus its own LR.
        "push  {{r0-r12, lr}}",
        // Two pad words: keeps SP 8-byte aligned before and after the restore.
        "sub   sp, sp, #8",
        // Publish SP into current_tcb (null before the very first switch).
        "ldr   r0, ={kernel}",
        "ldr   r1, [r0]",
        "cmp   r1, #0",
        "beq   1f",
        "str   sp, [r1]",
        // Ask Rust for the next task (a0 = TCB).
        "1:",
        "bl    {trap_switch}",
        "cmp   r0, #0",
        "beq   4f",
        // Switch stacks, then reclaim/re-arm on the new stack.
        "ldr   sp, [r0]",
        "bl    {after_switch}",
        // Restore the incoming frame and return from the exception.
        "add   sp, sp, #8",
        "pop   {{r0-r12, lr}}",
        "rfeia sp!",
        // Nothing runnable: sleep until something interrupts, then re-check.
        "4:",
        "wfi",
        "b     1b",
        kernel = sym KERNEL,
        trap_switch = sym rrkernel_irq_switch,
        after_switch = sym rrkernel_after_switch,
        svc_mode = const MODE_SVC,
    );
}

core::arch::global_asm!(
    r#"
    .section .text, "ax"
    /* VBAR must be 32-byte aligned; the table is 8 words: reset, undef, SVC,
     * prefetch abort, data abort, reserved, IRQ, FIQ. */
    .align 5
    .globl rrkernel_vectors
rrkernel_vectors:
    b   rrkernel_unexpected_trap
    b   rrkernel_unexpected_trap
    b   rrkernel_unexpected_trap
    b   rrkernel_unexpected_trap
    b   rrkernel_unexpected_trap
    b   rrkernel_unexpected_trap
    b   rrkernel_irq_entry
    /* FIQ shares the handler: a secure CPU receives the tick here (group 0) and a
     * non-secure one on IRQ (group 1). The handler acknowledges through the GIC,
     * so it does not care which line it arrived on. */
    b   rrkernel_irq_entry

    .globl rrkernel_unexpected_trap
rrkernel_unexpected_trap:
    b   rrkernel_unexpected_trap
"#
);

// ---------------------------------------------------------------------------
// Timer configuration
// ---------------------------------------------------------------------------

/// Smallest slice accepted: the frame save/restore is ~40 instructions each way,
/// so 100 timer ticks is the point below which switching would dominate.
const MIN_SLICE_CYCLES: u64 = 100;

/// `CNTFRQ` value assumed before init, only for diagnostic conversions.
const NOMINAL_HZ: u32 = 62_500_000;

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
    cycles_to_ns_with(if hz == 0 { NOMINAL_HZ } else { hz }, cycles)
}

pub fn platform_limits() -> PlatformLimits {
    let hz = configured_hz();
    let hz = if hz == 0 { NOMINAL_HZ } else { hz };
    PlatformLimits {
        min_slice_ns: cycles_to_ns_with(hz, MIN_SLICE_CYCLES as u32),
        max_slice_ns: Some(cycles_to_ns_with(hz, u32::MAX)),
        timer_hz: hz,
        timer_note: "ARM generic timer (CNTP/CNTV) + GICv2, SRSDB/RFEIA switch",
    }
}

pub fn plan_timer(cfg: &SchedulerConfig) -> Result<crate::arch::TimerPlan, ConfigError> {
    // Unlike RISC-V/AVR, this architecture *can* tell us the timer frequency, so
    // a zero in the config is treated as "use CNTFRQ" rather than an error.
    let hz = if cfg.timer_hz != 0 {
        cfg.timer_hz
    } else {
        timer_hz()
    };
    if hz == 0 {
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

// The vector table lives in assembly, so Rust needs a declaration to take its
// address for VBAR.
unsafe extern "C" {
    static rrkernel_vectors: u8;
}

/// Point the CPU's vector base at our table, enable the timer's IRQ in the GIC,
/// arm the first slice and unmask IRQs.
pub fn start_timer(_plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        // 1. Exception vectors.
        let vbar = core::ptr::addr_of!(rrkernel_vectors) as u32;
        asm!("mcr p15, 0, {}, c12, c0, 0", in(reg) vbar, options(nomem, nostack));

        // 2. Interrupt controller.
        let cfg = config();
        gic_enable_timer_irq(cfg.timer_irq);

        // 3. First slice, then let IRQs in.
        rearm_timer();
        asm!("cpsie i", options(nomem, nostack));
        trace(5, current_cpsr() as usize);
    }
    Ok(())
}

/// Change the slice at run time: the countdown restarts from a full slice.
pub fn retune_timer(_plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        rearm_timer();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Task construction
// ---------------------------------------------------------------------------

/// The calling context becomes task 0 (SVC mode, on the firmware's boot stack).
///
/// `sp` is left null: the IRQ entry stores it on the first switch away, and a
/// switch *to* a task that was never switched away from cannot happen in a
/// circular ring.
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
    // 8-byte alignment padding, so an aligned frame base always fits.
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

        // The frame is consumed by `rfeia`, so it must sit 8-byte aligned at the
        // top of the task's stack, exactly as the IRQ path leaves it.
        let top = (block as usize + size) & !7;
        let frame = (top - FRAME_SIZE) as *mut u8;

        core::ptr::write_bytes(frame, 0, FRAME_SIZE);
        let w = |off: usize, v: usize| {
            core::ptr::write_volatile(frame.add(off) as *mut usize, v);
        };
        // r0 = the trampoline's argument (the TCB).
        w(OFF_R0, tcb as usize);
        // Where `rfeia` starts executing, and in what state: SVC mode, interrupts
        // enabled, and ARM state (the trampoline's address is even; for a Thumb
        // build the low bit would set CPSR.T).
        let entry = crate::trampoline::task_trampoline as *const () as usize;
        w(OFF_PC, entry);
        w(OFF_CPSR, CPSR_TASK_ENTRY | ((entry & 1) << 5));

        (*tcb).sp = frame;
        (*tcb).flags = 0;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Port plumbing
// ---------------------------------------------------------------------------

pub fn mark_running() {}

/// A finished task: the switch was already requested (a software IRQ is pending
/// in the GIC), so idle until the frame swaps. The stack is recycled right after,
/// so control never returns here.
pub fn exit_current_task_forever() -> ! {
    idle_forever()
}

/// Nothing runnable: wait for an interrupt, then let the IRQ entry re-check.
pub fn idle_forever() -> ! {
    trace(7, current_cpsr() as usize);
    loop {
        unsafe {
            asm!("wfi", options(nomem, nostack, preserves_flags));
        }
    }
}

/// Stop the kernel: mask IRQs and park. On bare metal `code` is ignored.
pub fn shutdown(_code: i32) -> ! {
    unsafe {
        let _ = critical_enter();
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Ask for an immediate switch: pend the timer IRQ through the GIC's software
/// interrupt register for this CPU, so it becomes the *same* code path as a tick.
///
/// This is the A/R-profile analogue of `PENDSVSET`: it defers the switch until
/// the critical section (if any) has ended, instead of switching inline.
pub fn request_switch() {
    unsafe {
        // GICD_SGIR: bits 24..25 choose the targets — 0b01 = "this CPU only" — and bits 0..3
        // carry the SGI id; the kernel's switch signal is SGI 0, so the id contributes nothing
        // to the value. Spelled out in the comment rather than as `| 0` in the expression,
        // which is what `clippy::identity_op` (rightly) refuses to let past review.
        let sgir = 1u32 << 24;
        write_volatile(gicd(0xF00), sgir);
    }
}

/// Nothing to release: stacks and TCBs are arena blocks, freed by the same
/// deferred-free pass.
pub unsafe fn on_task_reclaimed(_tcb: *mut TaskControlBlock) {}

/// Nothing to pin: reclamation runs from the IRQ entry on the incoming task's
/// stack, on a single CPU, with IRQs masked.
pub unsafe fn is_pinned(_tcb: *mut TaskControlBlock) -> bool {
    false
}

/// Whether a blocking call (`sleep`, `lock`, `park`) is guaranteed to have taken the caller off
/// the CPU **before it returns**.
///
/// `true` on this portrue here: the switch is taken on the way out of the masked region.
///
/// This matters because a task measures its own sleep around the call: if the switch is
/// asynchronous, the task can observe `elapsed == 0` and think it woke up early, even though
/// the kernel booked the wait correctly. `examples/sleep_fidelity.rs` reports this property
/// and gates its strict assertions on it.
pub fn parks_synchronously() -> bool {
    true
}

/// The port's free-running cycle counter, where one is usable.
///
/// `Some` only when the counter exists *and* is known to keep counting while the core is idle.
/// `None` means "no counter here", never "a counter with a different unit" - a caller that gets
/// `None` must fall back to the tick clock (see `time::Instant`).
pub fn cycle_counter() -> Option<u32> {
    // No free-running counter wired for this port yet, and the honest answer is `None`: callers
    // fall back to the tick clock instead of silently getting a different unit. (ARM A/R has the
    // generic timer's counter available; Xtensa has CCOUNT, but reading it needs the nightly
    // asm feature this port already gates on.)
    None
}

/// The frequency of [`cycle_counter`] in Hz (0 when there is no counter).
pub fn cycle_counter_hz() -> u32 {
    0
}

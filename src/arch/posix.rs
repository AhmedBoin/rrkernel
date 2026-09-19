//! POSIX (Linux) port: `setitimer`-style POSIX timers plus `SIGALRM`, with
//! hand-written x86-64 **fiber** switching. This is the closest thing to the
//! Cortex-M design that a hosted OS allows:
//!
//! | Cortex-M | here |
//! |---|---|
//! | `SysTick` period | `timer_create`/`timer_settime` (`CLOCK_MONOTONIC`, ns resolution) |
//! | `PendSV` exception | `SIGALRM` signal handler (`SA_ONSTACK` off, see below) |
//! | save R4–R11, swap SP, restore | save rbx/rbp/r12–r15 + MXCSR/x87-CW, swap SP, restore |
//! | `PRIMASK` mask | `sigprocmask` blocking `SIGALRM` |
//! | zero `SysTick->VAL` | `timer_settime` with a full relative slice in the past → wait, in the future |
//!
//! All tasks are fibers on the one calling OS thread, so a task switch costs a
//! register save and a stack swap — a few tens of nanoseconds — rather than an
//! OS context switch.
//!
//! # Deliberate design points
//! * **No `sigaltstack`.** With an alt stack, several fibers' interrupted
//!   handler frames would share the same stack region and overwrite each other
//!   when the scheduler switches between them mid-handler. Instead the handler
//!   runs on the interrupted fiber's own stack, and finished-task reclamation is
//!   therefore deferred until *after* the switch (the next handler invocation
//!   runs on a different fiber's stack), which is exactly the ordering
//!   [`crate::scheduler::reclaim_finished_tasks`] documents.
//! * **No `libc` crate.** The ~10 declarations needed are written out below,
//!   with size assertions, so this backend is dependency-free like the rest.
//! * Linux/glibc only: `struct sigaction`, `sigset_t` and `sigevent` have
//!   different layouts on macOS/BSD, and shipping unverifiable layouts is worse
//!   than a clear `compile_error!`. See `docs/PORTING.md` for what to change.
//!
//! # Timing expectations
//! POSIX timers are hrtimer-backed, so sub-millisecond slices do work, but the
//! signal delivery path (handler entry, `sigreturn`, scheduler wake-up) adds
//! microseconds of jitter and the OS can always delay a signal. Treat this
//! backend as "identical semantics, soft timing"; use Cortex-M when the timing
//! has to be hard.

use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::tcb::{TaskControlBlock, KERNEL};
use core::ffi::{c_int, c_void};
use core::ptr;
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Minimal libc surface (x86_64 Linux / glibc)
// ---------------------------------------------------------------------------

/// `sigset_t` on Linux is an array of unsigned longs; 128 bytes total.
pub type SigSet = [u64; 16];

#[repr(C)]
pub struct SigAction {
    pub sa_handler: usize,
    pub sa_mask: SigSet,
    pub sa_flags: c_int,
    pub sa_restorer: usize,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TimeSpec {
    pub tv_sec: i64,
    pub tv_nsec: i64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct TimeVal {
    pub tv_sec: i64,
    pub tv_usec: i64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ITimerVal {
    pub it_interval: TimeVal,
    pub it_value: TimeVal,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct ITimerSpec {
    pub it_interval: TimeSpec,
    pub it_value: TimeSpec,
}

/// `sigval` is a union of a pointer and an int; 8 bytes on x86_64.
#[repr(C)]
#[derive(Clone, Copy)]
pub union SigVal {
    pub sival_ptr: *mut c_void,
    pub sival_int: c_int,
}

/// `struct sigevent` as glibc lays it out for `timer_create`.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SigEvent {
    pub sigev_value: SigVal,
    pub sigev_signo: c_int,
    pub sigev_notify: c_int,
    /// Union of `sigev_notify_function`/`sigev_notify_attributes` (glibc) or
    /// `sigev_notify_thread_id` (Linux). Only zeroed here, so the variant does
    /// not matter, but the 16-byte size does.
    pub sigev_un: [usize; 2],
}

/// Opaque POSIX timer handle (`timer_t` is a pointer on glibc).
pub type TimerId = *mut c_void;

unsafe extern "C" {
    fn sigaction(signum: c_int, act: *const SigAction, old: *mut SigAction) -> c_int;
    fn sigemptyset(set: *mut SigSet) -> c_int;
    fn sigaddset(set: *mut SigSet, signum: c_int) -> c_int;
    fn sigprocmask(how: c_int, set: *const SigSet, old: *mut SigSet) -> c_int;
    fn raise(sig: c_int) -> c_int;
    fn timer_create(clockid: c_int, evp: *mut SigEvent, timer: *mut TimerId) -> c_int;
    fn timer_settime(
        timer: TimerId,
        flags: c_int,
        new: *const ITimerSpec,
        old: *mut ITimerSpec,
    ) -> c_int;
    fn clock_gettime(clockid: c_int, tp: *mut TimeSpec) -> c_int;
}

const SIGALRM: c_int = 14;
const SIG_BLOCK: c_int = 0;
const SIG_UNBLOCK: c_int = 1;
const SIG_SETMASK: c_int = 2;
const SA_RESTART: c_int = 0x1000_0000;
const CLOCK_MONOTONIC: c_int = 1;
const SIGEV_SIGNAL: c_int = 0;

/// Smallest slice this backend will accept.
///
/// POSIX timers ride on hrtimers, so the floor is not the 1 ms timer tick, but
/// signal delivery plus `sigreturn` plus the OS scheduler still costs tens of
/// microseconds. Asking for 1 µs would "work" and then behave badly; this is
/// rejected up front instead.
const MIN_SLICE_NS: u64 = 50_000;
/// 100 ns units, as in the Win32 backend, so `Slice::Cycles` stays portable.
const TIMER_UNITS_PER_SEC: u32 = 10_000_000;

// Compile-time ABI checks: if any of these layouts is wrong, the kernel would
// corrupt memory rather than fail loudly, so they are asserted here.
const _: () = assert!(core::mem::size_of::<SigSet>() == 128);
const _: () = assert!(core::mem::size_of::<SigAction>() == 152);
const _: () = assert!(core::mem::size_of::<SigEvent>() == 32);
const _: () = assert!(core::mem::size_of::<ITimerSpec>() == 32);
const _: () = assert!(core::mem::align_of::<SigAction>() == 8);

// ---------------------------------------------------------------------------
// Critical sections: block SIGALRM (the fibre equivalent of masking the tick)
// ---------------------------------------------------------------------------

/// The signal mask to restore on exit from the critical section.
pub type CriticalToken = SigSet;

/// Block `SIGALRM` and return the previous mask.
///
/// Blocking the tick signal is stronger than a lock here: a signal raised while
/// blocked stays *pending* and is delivered the moment the section ends, so no
/// tick can ever be lost, and no lock can be contended by the handler (there is
/// only one thread).
///
/// # Safety
/// Pair with exactly one [`critical_exit`].
pub unsafe fn critical_enter() -> CriticalToken {
    let mut set: SigSet = [0; 16];
    let mut old: SigSet = [0; 16];
    sigemptyset(&mut set);
    sigaddset(&mut set, SIGALRM);
    sigprocmask(SIG_BLOCK, &set, &mut old);
    old
}

/// Restore the mask saved by [`critical_enter`].
///
/// # Safety
/// `token` must come from the matching [`critical_enter`].
pub unsafe fn critical_exit(token: CriticalToken) {
    sigprocmask(SIG_SETMASK, &token, ptr::null_mut());
}

// ---------------------------------------------------------------------------
// The fibre context switch (x86-64 SysV)
// ---------------------------------------------------------------------------

core::arch::global_asm!(
    r#"
    .text

    /* void rrkernel_switch(void **from_sp, void *to_sp)
     *   rdi = where to store our SP, rsi = the SP to resume.
     *
     * Saved frame, ascending from the stored SP:
     *   [ 0..16) MXCSR (4) , x87 control word (2), padding
     *   [16..64) r15, r14, r13, r12, rbx, rbp
     *   [64..72) return address
     * SysV callee-saved state is exactly rbx/rbp/r12-r15 plus the two control
     * registers; anything else is caller-saved and already dead at a call site.
     */
    .globl rrkernel_switch
    .type  rrkernel_switch, @function
rrkernel_switch:
    push  rbp
    push  rbx
    push  r12
    push  r13
    push  r14
    push  r15
    sub   rsp, 16
    stmxcsr [rsp]
    fnstcw  [rsp+4]
    mov   [rdi], rsp
    mov   rsp, rsi
    ldmxcsr [rsp]
    fldcw   [rsp+4]
    add   rsp, 16
    pop   r15
    pop   r14
    pop   r13
    pop   r12
    pop   rbx
    pop   rbp
    ret

    /* A new fibre lands here after its first switch: r12 holds the TCB pointer
     * that create_task planted in the frame.
     *
     * First it unblocks the tick signal: this fibre was born inside a signal
     * handler's callee chain (that is where the switch happened), so the
     * thread's SIGALRM mask is still "blocked" and no tick would ever reach it
     * again. Then `jmp` (not `call`) into the trampoline keeps the stack
     * alignment the ABI expects at a function entry; the trampoline never
     * returns. */
    .globl rrkernel_task_start
    .type  rrkernel_task_start, @function
rrkernel_task_start:
    sub   rsp, 8
    call  rrkernel_fibre_entry
    add   rsp, 8
    mov   rdi, r12
    jmp   task_trampoline
"#
);

unsafe extern "C" {
    /// `extern "C" fn rrkernel_switch(from_sp: *mut *mut u8, to_sp: *mut u8)`
    fn rrkernel_switch(from_sp: *mut *mut u8, to_sp: *mut u8);
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static TIMER: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
/// Slice in nanoseconds; the timer is re-armed with this on every switch, which
/// is the host equivalent of zeroing `SysTick->VAL`.
static SLICE_NS: AtomicU64 = AtomicU64::new(1_000_000);
static LAST_TICK_NS: AtomicU64 = AtomicU64::new(0);
/// Absolute time the armed timer should fire, so the handler can tell a *timer*
/// tick from a kick (`raise` from a `spawn`/task-exit path). Without this, every
/// kick would be counted as a tick and the rotation would advance twice as fast
/// as the statistics claim.
static NEXT_EXPIRY_NS: AtomicU64 = AtomicU64::new(0);
/// 0 = not initialised, 1 = ready.
static READY: AtomicU32 = AtomicU32::new(0);

fn now_ns() -> u64 {
    let mut ts = TimeSpec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe {
        if clock_gettime(CLOCK_MONOTONIC, &mut ts) != 0 {
            return 0;
        }
    }
    (ts.tv_sec as u64)
        .saturating_mul(1_000_000_000)
        .saturating_add(ts.tv_nsec as u64)
}

/// Arm the timer for a **full** slice starting now (one-shot, re-armed on every
/// switch).
unsafe fn rearm() {
    let ns = SLICE_NS.load(Ordering::Relaxed);
    NEXT_EXPIRY_NS.store(now_ns().saturating_add(ns), Ordering::Relaxed);
    let spec = ITimerSpec {
        it_interval: TimeSpec {
            tv_sec: 0,
            tv_nsec: 0,
        },
        it_value: TimeSpec {
            tv_sec: (ns / 1_000_000_000) as i64,
            tv_nsec: (ns % 1_000_000_000) as i64,
        },
    };
    timer_settime(
        TIMER.load(Ordering::Relaxed) as TimerId,
        0,
        &spec,
        ptr::null_mut(),
    );
}

/// First Rust code a brand new fibre executes (see the assembly entry).
///
/// A fibre created by [`create_task`] cannot inherit the thread's signal mask
/// from whatever was running before it: the switch that started it happened
/// *inside* the `SIGALRM` handler, where the kernel has the tick signal blocked
/// for the handler's duration. If the mask were left blocked, this fibre would
/// run exactly once and never be preempted again — the whole kernel would stop
/// at the first task that blocks the signal at the wrong moment. So the entry
/// explicitly unblocks the tick.
///
/// (Fibres that resume *inside* a handler do not need this: their `sigreturn`
/// restores their own mask when the handler returns.)
#[no_mangle]
extern "C" fn rrkernel_fibre_entry() {
    unsafe {
        let mut set: SigSet = [0; 16];
        sigemptyset(&mut set);
        sigaddset(&mut set, SIGALRM);
        sigprocmask(SIG_UNBLOCK, &set, ptr::null_mut());
    }
}

// ---------------------------------------------------------------------------
// The tick: the userspace `PendSV`
// ---------------------------------------------------------------------------

/// Where a *finishing* fibre's final context is dumped. It is never resumed —
/// that is deliberate: there is no live context left to save into, because the
/// task that owned the current context has already unlinked itself.
static SCRATCH_SP: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// `SIGALRM` handler.
///
/// Runs on the interrupted fibre's stack (no `sigaltstack` — see the module
/// docs for why), with `SIGALRM` blocked for its whole duration, so the ring can
/// never be entered re-entrantly and no lock is needed.
extern "C" fn alarm_handler(_sig: c_int) {
    unsafe {
        let t0 = now_ns();

        // Did the *timer* expire, or is this a kick from `spawn`/task-exit?
        // Only a real expiry counts as a tick, and only then is the achieved
        // period meaningful.
        let from_timer = t0 >= NEXT_EXPIRY_NS.load(Ordering::Relaxed);
        if from_timer {
            crate::scheduler::on_tick();

            let cfg = *KERNEL.config.get();
            if cfg.measure != crate::tcb::Measure::Off {
                let last = LAST_TICK_NS.swap(t0, Ordering::Relaxed);
                if last != 0 && t0 > last {
                    let want = SLICE_NS.load(Ordering::Relaxed);
                    let err = (t0 - last).abs_diff(want);
                    crate::scheduler::record_period_error(err.min(u32::MAX as u64) as u32);
                }
            }
        }

        let cur = KERNEL.current();
        let next = crate::scheduler::schedule_next();

        // `next == cur` means "one runnable task", and null means "none": both
        // are no-ops. Note that `cur` may legitimately be null while `next` is
        // not — that is the case where the task that was running has just
        // unlinked itself. Returning early there would strand the kernel, which
        // is exactly the bug this comment exists to prevent.
        if next.is_null() || next == cur {
            rearm();
            return;
        }

        // Full slice for whoever runs next, measured from the moment it starts.
        rearm();

        // Service time: everything the kernel did up to the actual switch. The
        // register-swap itself is a handful of instructions either side of this
        // measurement and cannot be observed from Rust.
        if (*KERNEL.config.get()).measure != crate::tcb::Measure::Off {
            let dt = now_ns().saturating_sub(t0);
            crate::scheduler::record_latency(dt.min(u32::MAX as u64) as u32);
        }

        // The switch itself. Saves the *interrupted* context and resumes `next`;
        // control returns here only when this fibre is switched back in, after
        // which the handler returns normally and `sigreturn` resumes the
        // interrupted instruction.
        let from: *mut *mut u8 = if cur.is_null() {
            // Nobody to save: the previous task already finished and unlinked
            // itself. Dumping its dying context to a scratch slot is correct
            // because that fibre is never resumed; its stack is released by the
            // reclamation below.
            SCRATCH_SP.as_ptr() as *mut *mut u8
        } else {
            &mut (*cur).sp as *mut *mut u8
        };
        rrkernel_switch(from, (*next).sp);

        // Reached on the way *back in*: we are running on this fibre's own stack
        // again, and the fibre we switched away from (if it finished) is
        // untouched by anybody, so its stack is finally safe to recycle.
        crate::scheduler::reclaim_finished_tasks();
    }
}

// ---------------------------------------------------------------------------
// Port interface
// ---------------------------------------------------------------------------

pub fn cycles_to_ns(cycles: u32) -> u64 {
    cycles as u64 * (1_000_000_000 / TIMER_UNITS_PER_SEC as u64)
}

pub fn platform_limits() -> PlatformLimits {
    PlatformLimits {
        min_slice_ns: MIN_SLICE_NS,
        max_slice_ns: Some(u32::MAX as u64 * 100),
        timer_hz: TIMER_UNITS_PER_SEC,
        timer_note: "POSIX hrtimer (CLOCK_MONOTONIC) + SIGALRM, x86-64 fibre switch",
    }
}

pub fn plan_timer(cfg: &SchedulerConfig) -> Result<crate::arch::TimerPlan, ConfigError> {
    let requested = match cfg.slice {
        Slice::Cycles(c) => c as u64 * 100,
        other => other.as_nanos(cfg.timer_hz),
    };
    if requested == 0 {
        return Err(ConfigError::ZeroSlice);
    }
    if requested < MIN_SLICE_NS {
        return Err(ConfigError::SliceBelowPlatformMinimum {
            requested_ns: requested,
            minimum_ns: MIN_SLICE_NS,
        });
    }
    let units = requested.div_ceil(100);
    if units > u32::MAX as u64 {
        return Err(ConfigError::SliceAboveTimerRange {
            requested_ns: requested,
            maximum_ns: u32::MAX as u64 * 100,
        });
    }
    Ok(crate::arch::TimerPlan {
        slice_ns: units * 100,
        slice_cycles: units as u32,
        timer_hz: TIMER_UNITS_PER_SEC,
    })
}

pub fn start_timer(plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        SLICE_NS.store(plan.slice_ns, Ordering::Release);

        // 1. Handler. `sa_mask` is empty because SIGALRM is blocked for the
        //    handler's duration by default (SA_NODEFER is deliberately unset).
        let mut act = SigAction {
            // Function-to-integer casts go through a pointer type on purpose:
            // a direct `fn as usize` is not portable.
            sa_handler: alarm_handler as *const () as usize,
            sa_mask: [0; 16],
            sa_flags: SA_RESTART,
            sa_restorer: 0,
        };
        sigemptyset(&mut act.sa_mask);
        if sigaction(SIGALRM, &act, ptr::null_mut()) != 0 {
            return Err(ConfigError::Platform("sigaction(SIGALRM) failed"));
        }

        // 2. A nanosecond-resolution timer delivering SIGALRM.
        let mut ev = SigEvent {
            sigev_value: SigVal { sival_int: 0 },
            sigev_signo: SIGALRM,
            sigev_notify: SIGEV_SIGNAL,
            sigev_un: [0; 2],
        };
        let mut id: TimerId = ptr::null_mut();
        if timer_create(CLOCK_MONOTONIC, &mut ev, &mut id) != 0 {
            return Err(ConfigError::Platform("timer_create failed"));
        }
        TIMER.store(id as usize, Ordering::Release);

        rearm();
        READY.store(1, Ordering::Release);
    }
    Ok(())
}

pub fn retune_timer(plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        SLICE_NS.store(plan.slice_ns, Ordering::Release);
        rearm();
    }
    Ok(())
}

/// The calling thread becomes task 0. Fibres share the thread, so there is no
/// stack to allocate: the handler saves task 0's context on its first switch
/// away, exactly as `PendSV` does on Cortex-M.
pub unsafe fn adopt_current_task(tcb: *mut TaskControlBlock) -> Result<(), ConfigError> {
    unsafe {
        (*tcb).sp = ptr::null_mut();
        (*tcb).stack_base = ptr::null_mut();
        (*tcb).stack_size = 0;
    }
    Ok(())
}

/// Address of the assembly entry point for a brand new fibre.
#[inline]
fn rrkernel_task_start_addr() -> usize {
    unsafe extern "C" {
        fn rrkernel_task_start();
    }
    rrkernel_task_start as *const () as usize
}

pub unsafe fn create_task(
    tcb: *mut TaskControlBlock,
    stack_size: usize,
) -> Result<(), crate::SpawnError> {
    // 16-byte alignment for the ABI, plus 16 so the frame can be padded to keep
    // `rsp % 16 == 8` at the task entry (the state the ABI expects after a
    // `call`). The floor is 8 KiB: a fibre stack also has to host the signal
    // handler that preempts it.
    let size = stack_size.max(8 * 1024) + 16;
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

        // Frame layout (see the assembly): 72 bytes, placed so that
        // `frame + 72 ≡ 8 (mod 16)` — the ABI state at a function entry.
        let top = (block as usize + size) & !15;
        let frame = ((top - 72) & !15) as *mut u8;

        ptr::write_bytes(frame, 0, 72);
        // [0..16) FP control state. This must be the *architectural default*,
        // not zero: MXCSR 0x1F80 masks all SSE exceptions and FCW 0x037F masks
        // all x87 exceptions. Loading zeroes here unmasks everything, and the
        // first inexact floating-point result in the task would then trap as
        // SIGFPE — a genuinely confusing failure that only shows up in tasks
        // doing any FP arithmetic.
        ptr::write_volatile(frame as *mut u32, 0x0000_1F80); // MXCSR
        ptr::write_volatile(frame.add(4) as *mut u16, 0x037F); // x87 control word
                                                               // [16..64) r15, r14, r13, r12, rbx, rbp — only r12 matters here.
        ptr::write_volatile(frame.add(40) as *mut usize, tcb as usize);
        // [64..72) return address: what the first switch "returns" into.
        ptr::write_volatile(frame.add(64) as *mut usize, rrkernel_task_start_addr());

        (*tcb).sp = frame;
        (*tcb).flags = 0;
    }
    Ok(())
}

pub fn mark_running() {}

/// Finished task: the timer interrupts us within a slice and the handler
/// switches away for good — the stack is recycled after that switch, so control
/// never comes back here.
pub fn exit_current_task_forever() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// Nothing runnable: stay responsive with a cheap spin. The signal still
/// arrives, and a task spawned from anywhere is picked up on the next tick.
pub fn idle_forever() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// Ask for an immediate switch. `raise` targets the *calling thread*, which is
/// where every fibre lives; if SIGALRM is blocked (inside a critical section) it
/// is delivered the moment the section exits.
pub fn request_switch() {
    unsafe {
        if READY.load(Ordering::Acquire) == 1 {
            raise(SIGALRM);
        }
    }
}

/// A single-threaded process can exit the normal way: no other thread can hold a
/// lock the C runtime needs.
pub fn shutdown(code: i32) -> ! {
    std::process::exit(code)
}

pub unsafe fn on_task_reclaimed(_tcb: *mut TaskControlBlock) {}

/// Nothing to pin: reclamation runs after the switch, on the *incoming* fibre's
/// stack, so a finishing task's stack is never in use when it is freed.
pub unsafe fn is_pinned(_tcb: *mut TaskControlBlock) -> bool {
    false
}

/// Whether a blocking call (`sleep`, `lock`, `park`) is guaranteed to have taken the caller off
/// the CPU **before it returns**.
///
/// `true` on this portrue here: the SIGALRM handler cannot switch when nothing is runnable (no idle context), so it returns and the blocked fibre resumes instead of parking.
///
/// This matters because a task measures its own sleep around the call: if the switch is
/// asynchronous, the task can observe `elapsed == 0` and think it woke up early, even though
/// the kernel booked the wait correctly. `examples/sleep_fidelity.rs` reports this property
/// and gates its strict assertions on it.
pub fn parks_synchronously() -> bool {
    false
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

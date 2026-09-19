//! Windows (`std`) backend: ring-driven preemptive round robin on top of real
//! OS threads, with a dedicated tick thread holding the baton.
//!
//! # Why not a signal / fiber hijack
//! Windows has no per-thread asynchronous timer signal (no `SIGALRM`), and
//! `SwitchToFiber` is documented as thread-affine, so a timer callback running
//! on a thread-pool thread can never switch the fiber of the thread that is
//! actually burning CPU. The primitives that *can* force a running thread off
//! the CPU from another thread are `SuspendThread`/`ResumeThread`, so that is
//! what this backend uses:
//!
//! * each task is a Win32 thread, created **suspended**, whose entry point is
//!   [`crate::trampoline::task_trampoline`];
//! * a dedicated tick thread (`THREAD_PRIORITY_TIME_CRITICAL`) wakes every
//!   slice on a **one-shot high-resolution waitable timer**, calls
//!   [`crate::scheduler::schedule_next`], then `SuspendThread`s the task that
//!   was running and `ResumeThread`s the one the ring selected;
//! * because the tick thread *re-arms the timer on every switch*, every task
//!   gets a full slice from the moment it was switched in — the same
//!   "restart the count" behaviour as zeroing `SysTick->VAL` on Cortex-M. A
//!   task that finishes early therefore does not shorten anybody's slice.
//!
//! # Known, documented limits (host-only)
//! * Timer granularity: 0.5 ms with
//!   `CREATE_WAITABLE_TIMER_HIGH_RESOLUTION`, otherwise 1 ms
//!   (`timeBeginPeriod`); a smaller slice is **rejected**, never clamped.
//! * `SuspendThread` on a thread inside a syscall takes effect when it returns
//!   to user mode, so a task blocked in the kernel (I/O, page fault) can
//!   overrun its slice. Pure CPU-bound work — the case this kernel exists for
//!   — is preempted immediately.
//! * Jitter is bounded by the Windows scheduler and DPC latency (tens of µs),
//!   not by the kernel. `jitter_bench` measures it rather than hiding it.
//! * Each task costs an OS thread, so the thread count is the real limit
//!   (unlike bare metal, where tasks are pure stacks). The kernel's own
//!   semantics — ring, counters, automatic unlink — are identical.

use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::scheduler;
use crate::tcb::{IdlePolicy, TaskControlBlock, TaskState, KERNEL};
use core::ffi::c_void;
use core::ptr;
use core::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, AtomicUsize, Ordering};
use std::sync::Once;

// ---------------------------------------------------------------------------
// Raw Win32 / WinMM declarations. No `windows-sys`, no `libc`: seven
// functions' worth of FFI is not worth a dependency on every target.
// ---------------------------------------------------------------------------

pub type Handle = *mut c_void;
type Bool = i32;
type Dword = u32;
type Lpvoid = *mut c_void;
type ThreadStart = unsafe extern "system" fn(Lpvoid) -> Dword;

#[link(name = "kernel32")]
unsafe extern "system" {
    fn CreateThread(
        attrs: Lpvoid,
        stack_size: usize,
        start: ThreadStart,
        param: Lpvoid,
        flags: Dword,
        thread_id: *mut Dword,
    ) -> Handle;
    fn ResumeThread(thread: Handle) -> Dword;
    fn SuspendThread(thread: Handle) -> Dword;
    fn OpenThread(access: Dword, inherit: Bool, thread_id: Dword) -> Handle;
    fn GetCurrentThreadId() -> Dword;
    fn CloseHandle(handle: Handle) -> Bool;
    fn CreateEventW(attrs: Lpvoid, manual_reset: Bool, initial: Bool, name: *const u16) -> Handle;
    fn SetEvent(handle: Handle) -> Bool;
    fn WaitForMultipleObjects(
        count: Dword,
        handles: *const Handle,
        wait_all: Bool,
        timeout_ms: Dword,
    ) -> Dword;
    fn CreateWaitableTimerExW(
        attrs: Lpvoid,
        name: *const u16,
        flags: Dword,
        access: Dword,
    ) -> Handle;
    fn CancelWaitableTimer(timer: Handle) -> Bool;
    fn SetWaitableTimer(
        timer: Handle,
        due_time: *const i64,
        period_ms: i32,
        completion: Lpvoid,
        arg: Lpvoid,
        resume: Bool,
    ) -> Bool;
    fn SwitchToThread() -> Bool;
    fn Sleep(ms: Dword);
    fn SetThreadPriority(thread: Handle, priority: i32) -> Bool;
    fn ExitThread(code: Dword) -> !;
    fn GetCurrentProcess() -> Handle;
    fn TerminateProcess(process: Handle, code: Dword) -> Bool;
    fn QueryPerformanceCounter(out: *mut i64) -> Bool;
    fn QueryPerformanceFrequency(out: *mut i64) -> Bool;
}

#[link(name = "winmm")]
unsafe extern "system" {
    fn timeBeginPeriod(period: u32) -> u32;
    fn timeEndPeriod(period: u32) -> u32;
}

const CREATE_SUSPENDED: Dword = 0x0000_0004;
const CREATE_WAITABLE_TIMER_HIGH_RESOLUTION: Dword = 0x0000_0002;
const TIMER_ALL_ACCESS: Dword = 0x001F_0003;
const THREAD_SUSPEND_RESUME: Dword = 0x0002;
const THREAD_TERMINATE: Dword = 0x0001;
const THREAD_QUERY_INFORMATION: Dword = 0x0040;
const THREAD_PRIORITY_TIME_CRITICAL: i32 = 15;
const INFINITE: Dword = 0xFFFF_FFFF;
const WAIT_OBJECT_0: Dword = 0;
/// One "timer tick" is 100 ns on this backend (the Win32 timer unit), so
/// `TimerPlan::timer_hz` is 10 MHz and `cycles_to_ns(c) == c * 100`.
const TIMER_UNITS_PER_SEC: u32 = 10_000_000;
/// 0.5 ms is the granularity of a high-resolution waitable timer.
const MIN_SLICE_NS_HIGH_RES: u64 = 500_000;
/// Without high-resolution timers, `timeBeginPeriod(1)` gives 1 ms.
const MIN_SLICE_NS_FALLBACK: u64 = 1_000_000;

// ---------------------------------------------------------------------------
// Kernel lock (the host double of Cortex-M's PRIMASK)
// ---------------------------------------------------------------------------

/// Owner thread id + 1 (0 = free) and recursion depth.
static LOCK_OWNER: AtomicU32 = AtomicU32::new(0);
static LOCK_DEPTH: AtomicU32 = AtomicU32::new(0);

/// Token returned by [`critical_enter`].
///
/// On this backend the critical section is a **global recursive spinlock**, so unlike the
/// interrupt-masking ports there is no interrupt state to save and no state to restore:
/// leaving the section is a depth decrement. The token is therefore the **nesting depth this
/// section added**, which is the one piece of per-entry state that really exists here, rather
/// than `()`.
///
/// That keeps the port surface uniform (`docs/PORTING.md` documents each backend's token
/// meaning) without pretending a saved `PRIMASK` exists, and it removes a unit value that
/// otherwise had to travel through `critical::CriticalGuard`, `smp::IrqToken` and every
/// `SpinLock` guard in the kernel.
pub type CriticalToken = u32;

/// Enter the kernel critical section.
///
/// # Safety
/// Every `enter` must be paired with exactly one `exit`. Sections must be
/// short and must never block: the tick thread waits for this lock.
pub unsafe fn critical_enter() -> CriticalToken {
    let me = GetCurrentThreadId().wrapping_add(1);
    if LOCK_OWNER.load(Ordering::Acquire) == me {
        return LOCK_DEPTH.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    }
    let mut spins: u64 = 0;
    loop {
        if LOCK_OWNER
            .compare_exchange_weak(0, me, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
        {
            LOCK_DEPTH.store(1, Ordering::Relaxed);
            return 1;
        }
        spins += 1;
        if spins & 0xFF == 0 {
            // Yield so the lock owner is guaranteed CPU time. This is what
            // makes a *blocking* acquire safe here: the only way to reach the
            // kernel lock is through a bounded, non-blocking critical section,
            // so the owner always makes progress.
            SwitchToThread();
        }
        if spins > 200_000_000 {
            // Unreachable by construction. If it ever happens, a critical
            // section is blocked or a task was suspended while holding the
            // lock — a kernel bug that must not be papered over.
            std::process::abort();
        }
        core::hint::spin_loop();
    }
}

/// Leave the kernel critical section.
///
/// The token is not consulted: see [`CriticalToken`] — releasing is a depth decrement, and
/// the ownership word is cleared only when the outermost section on this thread ends.
///
/// # Safety
/// Must be called exactly once per [`critical_enter`] on the same thread.
pub unsafe fn critical_exit(_token: CriticalToken) {
    if LOCK_DEPTH.fetch_sub(1, Ordering::AcqRel) == 1 {
        LOCK_OWNER.store(0, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// Platform objects
// ---------------------------------------------------------------------------

static TIMER: AtomicUsize = AtomicUsize::new(0);
static KICK: AtomicUsize = AtomicUsize::new(0);
static TICK_THREAD: AtomicUsize = AtomicUsize::new(0);
/// Exit code requested by `scheduler::shutdown`, consumed by the tick thread.
static EXIT_CODE: AtomicU32 = AtomicU32::new(0);
/// **TCB** of the task whose thread currently owns the CPU (the "baton").
///
/// Kept as a TCB pointer rather than a thread handle on purpose: a handle value
/// can be recycled by a later `CreateThread`, so a stale handle could make the
/// tick thread suspend `Thread B` thinking it was suspending `Thread A`. A TCB
/// pointer cannot alias a live task because the kernel never reclaims the TCB
/// the tick thread is still holding — see `is_pinned` — and it is also
/// deliberately *not* the same thing as `KERNEL.current_tcb`, because those two
/// legitimately differ for the few instructions between a task unlinking itself
/// and the tick thread switching away from it.
static RUNNING_TCB: AtomicUsize = AtomicUsize::new(0);
/// Slice in 100 ns timer units, read by the tick thread when re-arming.
static SLICE_UNITS: AtomicU64 = AtomicU64::new(10_000);
/// Absolute deadline (performance-counter nanoseconds) of the next timer
/// expiry, used to measure how exact the achieved period is.
static DEADLINE: AtomicU64 = AtomicU64::new(0);
/// Performance-counter nanoseconds of the previous timer-driven wake, used to
/// measure the achieved tick period.
static LAST_TICK_NS: AtomicU64 = AtomicU64::new(0);
/// 0 = unknown, 1 = high-resolution timer available, 2 = fallback.
static HIGH_RES: AtomicU8 = AtomicU8::new(0);
static OBJECTS: Once = Once::new();

unsafe fn ensure_objects() {
    OBJECTS.call_once(|| {
        // Raise the system timer resolution (also needed for plain Sleep(1)
        // loops) and prefer a high-resolution waitable timer (Win10 1803+).
        timeBeginPeriod(1);

        let mut timer = CreateWaitableTimerExW(
            ptr::null_mut(),
            ptr::null(),
            CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
            TIMER_ALL_ACCESS,
        );
        if timer.is_null() {
            HIGH_RES.store(2, Ordering::Release);
            timer = CreateWaitableTimerExW(ptr::null_mut(), ptr::null(), 0, TIMER_ALL_ACCESS);
        } else {
            HIGH_RES.store(1, Ordering::Release);
        }
        TIMER.store(timer as usize, Ordering::Release);

        let kick = CreateEventW(ptr::null_mut(), 0, 0, ptr::null());
        KICK.store(kick as usize, Ordering::Release);
    });
}

/// Arm the one-shot timer for a **full** slice starting now.
///
/// This is the host equivalent of writing `0` to `SysTick->VAL`: whoever runs
/// next gets the whole slice, no matter how much of the previous slice was
/// left unused because a task exited or a spawn kicked the scheduler.
unsafe fn rearm_timer() {
    let units = SLICE_UNITS.load(Ordering::Relaxed) as i64;
    let due: i64 = -units; // negative = relative
    DEADLINE.store(now_ns() + (units as u64) * 100, Ordering::Relaxed);
    SetWaitableTimer(
        TIMER.load(Ordering::Relaxed) as Handle,
        &due,
        0,
        ptr::null_mut(),
        ptr::null_mut(),
        0,
    );
}

static QPC_FREQ: AtomicU64 = AtomicU64::new(0);

/// Monotonic nanoseconds from the performance counter.
///
/// The multiply is done in `u128`: a QPC value is already ~1e11–1e12 ticks on a
/// machine that has been up for a while, and `value * 1e9` overflows `u64`
/// (silently, with a saturating multiply, which yields a *constant* and hides
/// every timing difference). That bug is why this function carries a comment.
unsafe fn now_ns() -> u64 {
    let mut freq = QPC_FREQ.load(Ordering::Relaxed);
    if freq == 0 {
        let mut f: i64 = 0;
        if QueryPerformanceFrequency(&mut f) == 0 || f <= 0 {
            return 0; // no usable clock: callers treat 0 as "unmeasured"
        }
        freq = f as u64;
        QPC_FREQ.store(freq, Ordering::Relaxed);
    }
    let mut c: i64 = 0;
    if QueryPerformanceCounter(&mut c) == 0 {
        return 0;
    }
    ((c as u128) * 1_000_000_000u128 / freq as u128) as u64
}

// ---------------------------------------------------------------------------
// The tick thread: the one and only place a context switch happens on Win32
// ---------------------------------------------------------------------------

/// Take the CPU away from `prev` and, if that task has finished, close its
/// thread handle at the same moment.
///
/// Closing here (rather than at reclaim time) is deliberate: this is the only
/// point where we know the handle value cannot be aliased by a newly created
/// thread, because the TCB is pinned until the switch completes.
unsafe fn suspend_and_maybe_close(prev: *mut TaskControlBlock, _next: Handle) {
    let h = (*prev).backend as Handle;
    if h.is_null() {
        return;
    }
    SuspendThread(h);
    if (*prev).state == TaskState::Dead {
        CloseHandle(h);
        (*prev).backend = ptr::null_mut();
    }
}

/// Terminate the whole process, **without** unwinding the C runtime.
///
/// # Why not `std::process::exit`
/// `exit` runs CRT/loader tear-down (`ExitProcess`), which waits for every
/// other thread to reach a safe point. This kernel deliberately keeps task
/// threads suspended between slices, so that wait can deadlock — observed in
/// practice, not theorised. A kernel that owns the process for its entire
/// lifetime has nothing meaningful to unwind, so we flush the standard streams
/// ourselves (so `println!` output is not lost) and then `TerminateProcess`.
/// The OS reclaims every thread, stack, fiber and arena byte.
pub fn exit_process(code: i32) -> ! {
    use std::io::Write;
    // Best effort: if another task holds the stdout lock right now, don't let
    // that stop us from terminating.
    let _ = std::io::stdout().flush();
    let _ = std::io::stderr().flush();
    unsafe {
        CancelWaitableTimer(TIMER.load(Ordering::Relaxed) as Handle);
        TerminateProcess(GetCurrentProcess(), code as Dword);
    }
    // TerminateProcess does not return for the current process.
    loop {
        core::hint::spin_loop();
    }
}

/// Tear the process down from the **tick thread**.
///
/// Never `exit` from a task: `ExitProcess` can deadlock while other threads are
/// suspended (the CRT/loader wait for all threads to reach a safe point), and a
/// task thread is exactly the sort of thread we keep suspended. So: resume
/// every task first, cancel the timer, then exit from here.
unsafe fn finish_shutdown(code: u32) -> ! {
    let mut start = *KERNEL.ring_head.get();
    if start.is_null() {
        start = KERNEL.current();
    }
    if !start.is_null() {
        let mut p = start;
        loop {
            let h = (*p).backend as Handle;
            if !h.is_null() {
                ResumeThread(h);
            }
            p = (*p).next;
            if p.is_null() || p == start {
                break;
            }
        }
    }
    timeEndPeriod(1);
    exit_process(code as i32)
}

unsafe extern "system" fn tick_thread_main(_param: Lpvoid) -> Dword {
    SetThreadPriority(
        OpenThread(THREAD_SUSPEND_RESUME, 0, GetCurrentThreadId()),
        THREAD_PRIORITY_TIME_CRITICAL,
    );

    loop {
        let handles = [
            TIMER.load(Ordering::Relaxed) as Handle,
            KICK.load(Ordering::Relaxed) as Handle,
        ];
        let woke = WaitForMultipleObjects(2, handles.as_ptr(), 0, INFINITE);
        let from_timer = woke == WAIT_OBJECT_0;
        let t0 = now_ns();

        if from_timer {
            // Timing accuracy: measured period between two consecutive
            // timer-driven wakes versus the requested slice. Unambiguous and
            // independent of when exactly the timer was re-armed.
            let last = LAST_TICK_NS.swap(t0, Ordering::Relaxed);
            if last != 0 && t0 > last {
                let period = t0 - last;
                let want = SLICE_UNITS.load(Ordering::Relaxed) * 100;
                let err = period.abs_diff(want);
                scheduler::record_period_error(err.min(u32::MAX as u64) as u32);
            }
        }

        let token = critical_enter();

        if *KERNEL.shutdown.get() {
            finish_shutdown(EXIT_CODE.load(Ordering::Relaxed));
        }

        if from_timer {
            scheduler::on_tick();
        }

        // Release the memory of tasks that finished since the last tick. The
        // tick thread is not running on any task's stack, so this is safe here;
        // `is_pinned` keeps the TCB we still need out of the way.
        scheduler::reclaim_finished_tasks();

        let next = scheduler::schedule_next();
        if next.is_null() {
            // Nothing runnable: either the last task just finished, or no task
            // has been spawned yet.
            let policy = (*KERNEL.config.get()).idle;
            if policy == IdlePolicy::ExitWhenAllDead {
                finish_shutdown(0);
            }
            rearm_timer();
        } else {
            let next_handle = (*next).backend as Handle;
            let prev_ptr = RUNNING_TCB.swap(next as usize, Ordering::AcqRel);
            if prev_ptr != 0 && prev_ptr != next as usize {
                // Take the CPU away from the previous task *before* giving it to
                // the next one: exactly one task runs at a time, which is what
                // makes the ring's shared state safe without per-field
                // synchronization.
                //
                // `prev_ptr` is guaranteed to still point at a live,
                // not-yet-reclaimed TCB: `is_pinned` keeps it out of the
                // reclaimer's hands, and `schedule_next` above only reclaims
                // *other* finished tasks.
                suspend_and_maybe_close(prev_ptr as *mut TaskControlBlock, next_handle);
            }
            ResumeThread(next_handle);
            // Full slice for the task just switched in, regardless of how the
            // previous slice ended (tick, spawn kick, task exit).
            rearm_timer();
        }

        critical_exit(token);

        // Service time of this switch: how long the kernel (plus any lock wait
        // caused by a task holding the kernel lock) delayed the boundary.
        let dt = now_ns().saturating_sub(t0);
        scheduler::record_latency(dt.min(u32::MAX as u64) as u32);
    }
}

// ---------------------------------------------------------------------------
// Port interface
// ---------------------------------------------------------------------------

use crate::arch::TimerPlan;

pub fn cycles_to_ns(cycles: u32) -> u64 {
    // "Cycles" on this backend are 100 ns timer units.
    cycles as u64 * (1_000_000_000 / TIMER_UNITS_PER_SEC as u64)
}

pub fn platform_limits() -> PlatformLimits {
    unsafe { ensure_objects() };
    let high_res = HIGH_RES.load(Ordering::Acquire) == 1;
    PlatformLimits {
        min_slice_ns: if high_res {
            MIN_SLICE_NS_HIGH_RES
        } else {
            MIN_SLICE_NS_FALLBACK
        },
        // 100 ns units stored in a u32: ~429 s ceiling.
        max_slice_ns: Some(u32::MAX as u64 * 100),
        timer_hz: TIMER_UNITS_PER_SEC,
        timer_note: if high_res {
            "high-resolution waitable timer (0.5 ms granularity)"
        } else {
            "system waitable timer with timeBeginPeriod(1) (1 ms granularity)"
        },
    }
}

pub fn plan_timer(cfg: &SchedulerConfig) -> Result<TimerPlan, ConfigError> {
    let limits = platform_limits();
    let requested = match cfg.slice {
        // A raw "cycles" request is interpreted in this backend's own unit
        // (100 ns) so `Slice::Cycles` still means something portable.
        Slice::Cycles(c) => c as u64 * 100,
        other => other.as_nanos(cfg.timer_hz),
    };
    if requested == 0 {
        return Err(ConfigError::ZeroSlice);
    }
    if requested < limits.min_slice_ns {
        return Err(ConfigError::SliceBelowPlatformMinimum {
            requested_ns: requested,
            minimum_ns: limits.min_slice_ns,
        });
    }
    let units = requested.div_ceil(100);
    if units > u32::MAX as u64 {
        return Err(ConfigError::SliceAboveTimerRange {
            requested_ns: requested,
            maximum_ns: u32::MAX as u64 * 100,
        });
    }
    Ok(TimerPlan {
        slice_ns: units * 100,
        slice_cycles: units as u32,
        timer_hz: TIMER_UNITS_PER_SEC,
    })
}

pub fn start_timer(plan: TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        ensure_objects();
        if TIMER.load(Ordering::Acquire) == 0 || KICK.load(Ordering::Acquire) == 0 {
            return Err(ConfigError::Platform("could not create the kernel timer"));
        }
        SLICE_UNITS.store(plan.slice_cycles as u64, Ordering::Release);
        rearm_timer();

        let handle = CreateThread(
            ptr::null_mut(),
            64 * 1024,
            tick_thread_main,
            ptr::null_mut(),
            0,
            ptr::null_mut(),
        );
        if handle.is_null() {
            return Err(ConfigError::Platform("could not create the tick thread"));
        }
        TICK_THREAD.store(handle as usize, Ordering::Release);
        SetThreadPriority(handle, THREAD_PRIORITY_TIME_CRITICAL);
    }
    Ok(())
}

pub fn retune_timer(plan: TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        SLICE_UNITS.store(plan.slice_cycles as u64, Ordering::Release);
        rearm_timer();
    }
    Ok(())
}

// `unsafe fn` is the honest signature for the whole port surface: the pointers arrive from the
// kernel, and the contract — a live, arena-allocated TCB, in kernel context — is stated once in
// `docs/PORTING.md`. Keeping every port identical here is the point: a per-target variation
// would be invisible to whoever writes the next port. Clippy's `not_unsafe_ptr_arg_deref` was
// right to complain, by the way — `rrkernel::arch` is re-exported, so before this change
// `arch::create_task(wild_pointer, 0)` was callable from *safe* code and would dereference it.
pub unsafe fn adopt_current_task(tcb: *mut TaskControlBlock) -> Result<(), ConfigError> {
    unsafe {
        // A real handle (not the GetCurrentThread pseudo-handle) so the tick
        // thread can suspend/resume it like any other task.
        let handle = OpenThread(
            THREAD_SUSPEND_RESUME | THREAD_QUERY_INFORMATION | THREAD_TERMINATE,
            0,
            GetCurrentThreadId(),
        );
        if handle.is_null() {
            return Err(ConfigError::Platform(
                "OpenThread on the calling thread failed",
            ));
        }
        (*tcb).backend = handle as *mut u8;
        (*tcb).stack_base = ptr::null_mut();
        (*tcb).stack_size = 0;
        RUNNING_TCB.store(tcb as usize, Ordering::Release);
    }
    Ok(())
}

pub unsafe fn create_task(
    tcb: *mut TaskControlBlock,
    stack_size: usize,
) -> Result<(), crate::SpawnError> {
    unsafe {
        // Windows rounds thread stacks up to its own granularity and 4 KiB is
        // not a workable real thread stack, so small requests are raised here
        // rather than failing mysteriously later. The OS owns this memory; the
        // arena is not involved.
        let size = stack_size.max(64 * 1024);
        let mut tid: Dword = 0;
        let handle = CreateThread(
            ptr::null_mut(),
            size,
            task_thread_main,
            tcb as Lpvoid,
            CREATE_SUSPENDED,
            &mut tid,
        );
        if handle.is_null() {
            return Err(crate::SpawnError::Backend(ConfigError::Platform(
                "CreateThread failed for a new task",
            )));
        }
        // The OS owns the stack; the arena holds only the TCB and the closure.
        (*tcb).backend = handle as *mut u8;
        (*tcb).stack_base = ptr::null_mut();
        (*tcb).stack_size = stack_size;
    }
    Ok(())
}

/// Task thread entry: a task needs nothing but the TCB it was created with.
unsafe extern "system" fn task_thread_main(param: Lpvoid) -> Dword {
    crate::trampoline::task_trampoline(param as *mut TaskControlBlock)
}

pub fn request_switch() {
    unsafe {
        let k = KICK.load(Ordering::Acquire);
        if k != 0 {
            SetEvent(k as Handle);
        }
    }
}

pub fn mark_running() {}

/// Release the OS thread handle of a finished task.
///
/// Normally the tick thread has already closed it in [`suspend_and_maybe_close`]
/// when it switched away from the task; this is the safety net for a task that
/// died without ever being switched away from (or on an error path), so a
/// long-running host program that spawns tasks in a loop cannot leak handles.
pub unsafe fn on_task_reclaimed(tcb: *mut TaskControlBlock) {
    unsafe {
        let h = (*tcb).backend as Handle;
        (*tcb).backend = ptr::null_mut();
        if h.is_null() {
            return;
        }
        if h as usize != RUNNING_TCB.load(Ordering::Acquire) {
            CloseHandle(h);
        }
    }
}

/// See [`crate::scheduler::drain_pending_free`]: the TCB the tick thread is
/// still holding as "the task I last resumed" must not be recycled, because the
/// next switch needs its `backend` handle to suspend it and, if it is finished,
/// to close that handle.
pub unsafe fn is_pinned(tcb: *mut TaskControlBlock) -> bool {
    let v = RUNNING_TCB.load(Ordering::Acquire);
    v != 0 && v == tcb as usize
}

pub fn exit_current_task_forever() -> ! {
    unsafe { ExitThread(0) }
}

pub fn idle_forever() -> ! {
    unsafe {
        loop {
            Sleep(1);
        }
    }
}

pub fn shutdown(code: i32) -> ! {
    unsafe {
        if TICK_THREAD.load(Ordering::Acquire) == 0 {
            // No tick thread yet (e.g. shutting down after a failed init):
            // there are no task threads, so a direct exit cannot deadlock.
            exit_process(code);
        }
        EXIT_CODE.store(code as u32, Ordering::Relaxed);
        // Ask the tick thread to do the handover politely (it resumes every
        // task first), then park this task forever: it must never continue into
        // user code after asking to stop.
        request_switch();
        let spins = std::sync::atomic::AtomicU32::new(0);
        loop {
            Sleep(1);
            // Safety valve: if the tick thread cannot run (e.g. it is blocked
            // inside SuspendThread on a thread wedged in the kernel), stop
            // waiting and terminate directly. Terminating from here is safe
            // because TerminateProcess is not a cooperative operation.
            if spins.fetch_add(1, Ordering::Relaxed) > 200 {
                exit_process(code);
            }
        }
    }
}

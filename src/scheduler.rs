//! Scheduler policy: who runs next, what the counters say, and how a finished
//! task's memory gets reclaimed.
//!
//! Everything in this module is backend-neutral. The port (see [`crate::arch`])
//! only ever calls into [`schedule_next`], [`on_tick`] and [`record_latency`],
//! and the ring surgery happens here — in Rust, under a critical section —
//! rather than in hand-written assembly. That split is deliberate: assembly
//! does register/stack mechanics, Rust does policy.

use crate::arena::{Arena, ArenaStats, DEFAULT_ARENA};
use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::critical;
use crate::ring;
use crate::tcb::{KernelConfig, TaskControlBlock, TaskState, KERNEL};
use core::cell::UnsafeCell;
use core::ptr;

/// Kernel arena. A private wrapper (rather than `static mut Arena`) so the
/// `UnsafeCell` contract is explicit.
struct ArenaCell(UnsafeCell<Arena>);

// SAFETY: all access happens under `critical::enter`.
unsafe impl Sync for ArenaCell {}

static ARENA: ArenaCell = ArenaCell(UnsafeCell::new(Arena::empty()));

/// Raw access to the kernel arena. Caller must hold a critical section.
#[inline]
pub(crate) unsafe fn arena() -> *mut Arena {
    ARENA.0.get()
}

/// Snapshot of everything the kernel knows about itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SchedulerStats {
    /// Tasks ever created (monotonic).
    pub total_threads: usize,
    /// Tasks currently linked in the ring.
    pub active_threads: usize,
    /// Slices elapsed since init.
    pub ticks: u64,
    /// Context switches performed since init.
    pub switches: u64,
    /// Timer ticks that could not be serviced in time (host contention).
    pub ticks_deferred: u64,
    /// Worst switch latency observed (timer cycles or ns; see `Measure`).
    pub worst_latency: u32,
    /// Most recent switch latency.
    pub last_latency: u32,
    /// Worst deviation of the achieved tick period from the requested slice
    /// (nanoseconds). The headline timing number: how exact the slice is.
    pub worst_period_error_ns: u32,
    /// Most recent tick-period deviation (nanoseconds).
    pub last_period_error_ns: u32,
    /// Achieved slice length in nanoseconds.
    pub slice_ns: u64,
    /// Achieved slice length in timer cycles (0 on time-based host backends).
    pub slice_cycles: u32,
    /// Timer frequency used for the conversion (0 on host time-based timers).
    pub timer_hz: u32,
    /// Dead tasks whose memory has been reclaimed by deferred-free.
    pub reclaimed: u64,
    /// Times a task tried to block with no current task. Must stay zero: see
    /// [`crate::tcb::Kernel::blocks_without_current`]. Non-zero means a block came from
    /// interrupt or idle context and therefore did *not* wait at all.
    pub blocks_without_current: u64,
    /// Arena usage.
    pub arena: ArenaStats,
    /// Whether the tick source is live.
    pub running: bool,
}

/// Initialise the scheduler with the default configuration (1 ms slices).
///
/// The scheduler starts running **immediately**: the tick source is armed
/// before this returns, and the calling context is adopted as task 0 and
/// linked into the ring. There is no `scheduler_run()`.
///
/// # Panics
/// Panics if the default 1 ms slice is not achievable on this platform. Use
/// [`init_with`] if you want to handle that as a value.
pub fn init() {
    if let Err(e) = init_with(SchedulerConfig::default()) {
        panic!("rrkernel: scheduler_init() failed: {}", e);
    }
}

/// Initialise the scheduler with an explicit [`SchedulerConfig`].
///
/// `cfg.slice` is **the** tuning knob for timing-sensitive applications:
/// `scheduler::init_with(SchedulerConfig::embedded(Slice::Micros(100),
/// 168_000_000))` gives 100 µs slices, cycle-exact, on a 168 MHz core.
pub fn init_with(cfg: SchedulerConfig) -> Result<(), ConfigError> {
    // --- validate the time slice against the real platform -----------------
    let plan = crate::arch::plan_timer(&cfg)?;
    if plan.slice_ns == 0 || plan.slice_cycles == 0 {
        return Err(ConfigError::ZeroSlice);
    }

    // --- arena -------------------------------------------------------------
    let (base, len) = match cfg.arena {
        Some(region) => (region.as_mut_ptr(), region.len()),
        None => (DEFAULT_ARENA.as_ptr(), crate::arena::DEFAULT_ARENA_SIZE),
    };
    // The arena must at least hold task 0's TCB plus a few tasks' worth of
    // bookkeeping. Bare metal also carves each task's *stack* from here (see
    // `arch::cortex_m`), so the recommended size there is
    // `stack_size * max_tasks`; OS-backed backends only need the TCBs and the
    // closure blobs.
    let min_arena = core::mem::size_of::<TaskControlBlock>() * 4 + 512;
    if len < min_arena {
        return Err(ConfigError::ArenaTooSmall {
            provided: len,
            minimum: min_arena,
        });
    }

    let g = critical::enter();
    unsafe {
        if *KERNEL.running.get() {
            return Err(ConfigError::AlreadyInitialized);
        }

        (*arena()).init(base, len);
        *KERNEL.config.get() = KernelConfig {
            stack_size: cfg.stack_size,
            idle: cfg.idle,
            measure: cfg.measure,
            slice_cycles: plan.slice_cycles,
            timer_hz: plan.timer_hz,
            // Preserved from whatever `set_active_cores` chose before init, so the
            // multi-core decision survives this snapshot.
            active_cores: (*KERNEL.config.get()).active_cores.max(1),
        };

        // --- adopt the calling context as task 0 ---------------------------
        let tcb = alloc_tcb().ok_or(ConfigError::ArenaTooSmall {
            provided: len,
            minimum: min_arena,
        })?;
        (*tcb).state = TaskState::Running;
        (*tcb).id = take_id();
        crate::arch::adopt_current_task(tcb)?;
        ring::insert_after(ptr::null_mut(), tcb);
        KERNEL.set_current(tcb);
        *KERNEL.ring_head.get() = tcb;
        *KERNEL.total_threads.get() = 1;
        *KERNEL.active_threads.get() = 1;
    }
    drop(g);

    // Arm the periodic tick source. From here on the CPU is time-sliced.
    crate::arch::start_timer(plan)?;
    let g = critical::enter();
    unsafe { *KERNEL.running.get() = true };
    drop(g);
    Ok(())
}

/// Retune the slice at run time (reloads the timer; the *first* slice after
/// the change is a full slice).
pub fn set_slice(slice: Slice) -> Result<(), ConfigError> {
    let g = critical::enter();
    let timer_hz = unsafe {
        if !*KERNEL.running.get() {
            return Err(ConfigError::NotInitialized);
        }
        (*KERNEL.config.get()).timer_hz
    };
    drop(g);

    let cfg = SchedulerConfig {
        slice,
        timer_hz,
        ..SchedulerConfig::default()
    };
    let plan = crate::arch::plan_timer(&cfg)?;
    crate::arch::retune_timer(plan)?;

    let g = critical::enter();
    unsafe {
        (*KERNEL.config.get()).slice_cycles = plan.slice_cycles;
        (*KERNEL.config.get()).timer_hz = plan.timer_hz;
    }
    drop(g);
    Ok(())
}

/// What this platform can actually do. Call before choosing a slice.
pub fn platform_limits() -> PlatformLimits {
    crate::arch::platform_limits()
}

/// Current configuration snapshot (achieved slice, not the request).
pub fn config() -> KernelConfig {
    let g = critical::enter();
    let c = unsafe { *KERNEL.config.get() };
    drop(g);
    c
}

/// Achieved slice length in nanoseconds.
pub fn slice_ns() -> u64 {
    let cycles = config().slice_cycles;
    crate::arch::cycles_to_ns(cycles)
}

/// Full kernel statistics. Takes a critical section internally.
pub fn stats() -> SchedulerStats {
    let g = critical::enter();
    let out = unsafe {
        let cfg = *KERNEL.config.get();
        SchedulerStats {
            total_threads: *KERNEL.total_threads.get(),
            active_threads: *KERNEL.active_threads.get(),
            ticks: *KERNEL.ticks.get(),
            switches: *KERNEL.switches.get(),
            ticks_deferred: *KERNEL.ticks_deferred.get(),
            worst_latency: *KERNEL.worst_latency.get(),
            last_latency: *KERNEL.last_latency.get(),
            worst_period_error_ns: *KERNEL.worst_period_error_ns.get(),
            last_period_error_ns: *KERNEL.last_period_error_ns.get(),
            slice_ns: crate::arch::cycles_to_ns(cfg.slice_cycles),
            slice_cycles: cfg.slice_cycles,
            timer_hz: cfg.timer_hz,
            reclaimed: *KERNEL.reclaimed.get(),
            blocks_without_current: *KERNEL.blocks_without_current.get(),
            arena: (*arena()).stats(),
            running: *KERNEL.running.get(),
        }
    };
    drop(g);
    out
}

/// One row of [`for_each_task`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskInfo {
    pub id: u32,
    pub state: TaskState,
    pub slices_run: u32,
    pub switches: u32,
    pub stack_size: usize,
    /// True for the task currently owning the CPU.
    pub is_current: bool,
}

/// Visit every task currently linked in the ring, in ring order.
///
/// Safe to call from a task (takes a critical section) but *not* from inside
/// the switch path. The callback runs with preemption masked, so it must be
/// short and must not spawn or sleep.
pub fn for_each_task(mut f: impl FnMut(TaskInfo)) {
    let g = critical::enter();
    unsafe {
        let cur = KERNEL.current();
        // Iterate from the ring head so this also works in the window where a
        // task has unlinked itself and no switch has happened yet.
        let start = if cur.is_null() || !(*cur).is_linked() {
            *KERNEL.ring_head.get()
        } else {
            cur
        };
        if !start.is_null() && (*start).is_linked() {
            let mut p = start;
            loop {
                f(TaskInfo {
                    id: (*p).id,
                    state: (*p).state,
                    slices_run: (*p).slices_run,
                    switches: (*p).switches,
                    stack_size: (*p).stack_size,
                    is_current: p == cur,
                });
                p = (*p).next;
                if p == start || p.is_null() {
                    break;
                }
            }
        }
    }
    drop(g);
}

/// Number of tasks linked in the ring.
pub fn active_threads() -> usize {
    let g = critical::enter();
    let n = unsafe { *KERNEL.active_threads.get() };
    drop(g);
    n
}

/// Total tasks ever created.
pub fn total_threads() -> usize {
    let g = critical::enter();
    let n = unsafe { *KERNEL.total_threads.get() };
    drop(g);
    n
}

/// Id of the currently running task (0 if none).
pub fn current_task_id() -> u32 {
    let g = critical::enter();
    let id = unsafe {
        let c = KERNEL.current();
        if c.is_null() {
            0
        } else {
            (*c).id
        }
    };
    drop(g);
    id
}

/// Slices handed to the **calling task** so far. Cheap way for a task to see
/// its own share and, together with its siblings, to demonstrate that pure
/// round-robin fairness holds.
pub fn current_slices_run() -> u32 {
    let g = critical::enter();
    let n = unsafe {
        let c = KERNEL.current();
        if c.is_null() {
            0
        } else {
            (*c).slices_run
        }
    };
    drop(g);
    n
}

/// Number of context switches performed so far (cheap, lock-free read of the
/// 64-bit counter). Useful for correlating a task's timeline with the
/// scheduler's own rotation.
pub fn switch_count() -> u64 {
    let g = critical::enter();
    let n = unsafe { *KERNEL.switches.get() };
    drop(g);
    n
}

/// Number of slices elapsed so far.
pub fn tick_count() -> u64 {
    let g = critical::enter();
    let n = unsafe { *KERNEL.ticks.get() };
    drop(g);
    n
}

/// Ask the kernel to stop: on the host this exits the process, on bare metal
/// it masks interrupts and parks forever. Never returns.
pub fn shutdown(code: i32) -> ! {
    let g = critical::enter();
    unsafe { *KERNEL.shutdown.get() = true };
    drop(g);
    crate::arch::shutdown(code)
}

/// Set the shutdown flag without exiting (the host tick thread will notice and
/// terminate the process; useful for cooperative tear-down in tests).
pub fn request_shutdown() {
    let g = critical::enter();
    unsafe { *KERNEL.shutdown.get() = true };
    drop(g);
    crate::arch::request_switch();
}

/// True once [`request_shutdown`] / [`shutdown`] has been called.
pub fn shutdown_requested() -> bool {
    let g = critical::enter();
    let b = unsafe { *KERNEL.shutdown.get() };
    drop(g);
    b
}

/// Run the rest of `main` as **task 0's body**, then treat its return as a task
/// exit (unlink, `active_threads -= 1`, immediate switch) instead of falling
/// back into the C runtime.
///
/// # Why this exists
/// `scheduler_init()` adopts the calling context as task 0 — but on a hosted
/// process, *returning from `main`* is not something the kernel can intercept:
/// the C runtime takes over and calls `exit()`, which would tear the whole
/// process (and every task with it) down. `main_body` closes that hole without
/// any stack surgery: it calls `f`, and on return it runs the very same
/// teardown a task trampoline runs, ending the calling thread through the
/// kernel's own exit path (`ExitThread` on Windows, the idle loop on bare
/// metal) instead of the CRT's `exit()`.
///
/// Usage — spec semantics, host and metal alike:
///
/// ```no_run
/// # use rrkernel::{scheduler, thread, Slice, SchedulerConfig};
/// fn main() {
///     scheduler::init_with(SchedulerConfig::embedded(Slice::Millis(1), 168_000_000)).unwrap();
///     scheduler::main_body(|| {
///         thread::spawn(|| loop { /* CPU-bound, preempted every slice */ });
///         thread::spawn(|| { /* short work, then returns -> auto unlink */ });
///     });
/// }
/// ```
///
/// # Panics
/// Panics if the scheduler has not been initialised.
pub fn main_body<F: FnOnce()>(f: F) -> ! {
    // The scheduler adopted the calling context as task 0 at init time, so the
    // current TCB *is* our task (which stays true across preemptions: a task is
    // only ever running while it is the current one).
    let me = {
        let g = critical::enter();
        let me = unsafe { KERNEL.current() };
        drop(g);
        assert!(
            !me.is_null(),
            "scheduler::main_body() requires scheduler::init*() to have run first"
        );
        me
    };

    f();

    // SAFETY: we are this task, and its body has returned.
    unsafe { crate::trampoline::exit_task(me) }
}

// ---------------------------------------------------------------------------
// Called by the ports
// ---------------------------------------------------------------------------

/// Pick the next runnable task and hand it the CPU. **Runs in kernel context**
/// (Cortex-M: inside PendSV, on the kernel/main stack; host: on the tick
/// thread's stack), which is what makes it safe to reclaim the memory of the
/// task we just switched away from.
///
/// Returns the TCB to switch to, or null when nothing is runnable.
///
/// # Safety
/// Must be called from the port's switch path with the ring quiescent.
pub unsafe fn schedule_next() -> *mut TaskControlBlock {
    let mut cur = KERNEL.current();
    if !cur.is_null() && !(*cur).is_linked() {
        // The current task finished and unlinked itself; it is only waiting
        // for the port to switch away from it.
        cur = ptr::null_mut();
        KERNEL.set_current(ptr::null_mut());
    }

    let next = if cur.is_null() {
        // No current task (the previous one finished and unlinked itself). Take the first
        // *runnable* node from the head — **not** the head itself.
        //
        // Taking the head blindly is how a sleeping task came to resume early. `ring_head` is
        // moved on every spawn, so a blocked task can be at the head; if it is, this branch used
        // to hand it the CPU in the middle of its sleep. The symptom was the reported "about 1 in
        // 15, right after a task is created or destroyed", because a task exit is exactly when
        // this branch runs. Measured on an STM32F103: a 200-tick sleep in an otherwise idle ring
        // returned after **6** ticks, reproduced with the cycle counter agreeing (so it was real
        // time, not a counter artifact).
        crate::ring::first_runnable_from(*KERNEL.ring_head.get())
    } else {
        ring::next_runnable(cur)
    };

    if next.is_null() || !(*next).is_linked() {
        // Nothing runnable at all: leave `current` alone, the port's idle path
        // decides what to do (wfi / sleep / exit).
        return ptr::null_mut();
    }

    if next != cur {
        if !cur.is_null() && (*cur).state == TaskState::Running {
            (*cur).state = TaskState::Ready;
        }
        (*next).state = TaskState::Running;
        KERNEL.set_current(next);
        *KERNEL.switches.get() += 1;
    }
    (*next).switches = (*next).switches.wrapping_add(1);
    (*next).slices_run = (*next).slices_run.wrapping_add(1);
    next
}

/// A timer tick fired. The port calls this before asking for a switch.
#[inline]
pub unsafe fn on_tick() {
    *KERNEL.ticks.get() += 1;
    // Wake anything whose bounded wait has expired. This is also what makes
    // `Mutex::try_lock_for` time out: the waiter is asleep on a deadline and the tick
    // is the only clock that can notice it passing.
    wake_expired();
}

/// A tick could not be serviced (host lock contention). Visible in
/// [`SchedulerStats::ticks_deferred`]; a non-zero and growing value means the
/// slice is too short for the host scheduler to honour it.
#[inline]
pub unsafe fn on_tick_deferred() {
    *KERNEL.ticks_deferred.get() += 1;
}

/// Record switch latency (`Measure::Cycles` / `Measure::Nanos`, whichever the
/// port produced).
#[inline]
pub unsafe fn record_latency(value: u32) {
    *KERNEL.last_latency.get() = value;
    if value > *KERNEL.worst_latency.get() {
        *KERNEL.worst_latency.get() = value;
    }
}

/// Record how far the achieved tick period deviated from the requested slice
/// (nanoseconds, absolute value). This is the kernel's timing-accuracy metric:
/// on bare metal it is a handful of cycles, on a desktop OS it is what the host
/// timer allows.
#[inline]
pub unsafe fn record_period_error(value: u32) {
    *KERNEL.last_period_error_ns.get() = value;
    if value > *KERNEL.worst_period_error_ns.get() {
        *KERNEL.worst_period_error_ns.get() = value;
    }
}

// ---------------------------------------------------------------------------
// Allocation, reclamation and spawn (internal)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Blocking: sleeping `Mutex` waits and bounded sleeps
// ---------------------------------------------------------------------------
//
// A task that blocks stays **linked** in the ring and is simply skipped by
// `ring::next_runnable`, which is what makes both blocking and waking O(1) — nothing has
// to be searched to put it back, and its place in the rotation is preserved.

/// What a blocking call did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockOutcome {
    /// The caller's own condition was already satisfied, so nothing was blocked. The caller keeps
    /// the CPU and should carry on (this is what makes a wake-up impossible to lose).
    Ready,
    /// The caller is switched away and will resume when woken or its deadline passes.
    Blocked,
    /// The deadline had already passed, so nothing was waited for. Not an error.
    AlreadyPast,
    /// **No current task**: called from interrupt or idle context, where there is nothing to
    /// switch away from. The wait did *not* happen; counted in `stats().blocks_without_current`.
    NoCurrentTask,
}

/// Mark the current task blocked — **the caller must already hold the kernel critical section**.
///
/// This is the primitive every wait in the kernel builds on, and its contract is the reason
/// sleeps stopped returning early:
///
/// * the deadline, the state change and the switch-pend happen inside the caller's *one*
///   critical section, so no tick sweep can observe "marked blocked" without the switch already
///   being pended;
/// * `blocked_on`/`block_deadline` are written *before* `state`, and the state store is last,
///   so a tick that runs immediately afterwards sees a fully formed waiter.
///
/// On [`BlockOutcome::Blocked`] the caller must leave the critical section for the switch to
/// happen: on bare metal the pending `PendSV` is taken the moment the mask lifts.
///
/// # Safety
/// Caller holds a critical section, and is running on the current task (or is willing to be
/// told that it is not).
pub unsafe fn mark_blocked_locked(resource: u32, deadline_tick: u64) -> BlockOutcome {
    let me = KERNEL.current();
    if me.is_null() {
        // Interrupt or idle context. Counted and asserted rather than silently ignored: a
        // silent return here makes the caller's wait vanish, which is exactly how a sleep came
        // to "wake up early" — it never blocked at all.
        *KERNEL.blocks_without_current.get() += 1;
        debug_assert!(
            false,
            "rrkernel: blocking wait attempted with no current task (interrupt or idle context)"
        );
        return BlockOutcome::NoCurrentTask;
    }
    // Already past the deadline (or a zero-length wait): do not pretend to have waited.
    if deadline_tick != 0 && *KERNEL.ticks.get() >= deadline_tick {
        return BlockOutcome::AlreadyPast;
    }
    (*me).blocked_on = resource;
    (*me).block_deadline = deadline_tick;
    (*me).state = TaskState::Blocked;
    crate::arch::request_switch();
    BlockOutcome::Blocked
}

/// Block the calling task until `resource` is woken or the absolute `deadline_tick` passes.
///
/// `resource` is a [`crate::sync::LockId`], or 0 for a plain sleep. `deadline_tick == 0` means
/// "no deadline" — an unbounded wait, which callers must opt into explicitly.
pub fn block_until_tick(resource: u32, deadline_tick: u64) -> BlockOutcome {
    let g = critical::enter();
    let outcome = unsafe { mark_blocked_locked(resource, deadline_tick) };
    drop(g);
    outcome
}

/// Block the current task **unless** `ready()` says the wait is already over.
///
/// `ready()` runs inside the *same* critical section that would mark the task blocked. That is the
/// whole point: a waker either arrives before the section — the condition is then already true and
/// nothing blocks — or after it, in which case the task is already `Blocked` and visible to the
/// waker. A wake-up cannot fall into the gap.
///
/// A closure rather than a "check, then block" pair, deliberately: that two-step form is exactly
/// how `sync::Mutex::lock` came to lose a wake-up and block forever on a lock that was already
/// free. Everything built on top of this — `event::Signal`, `event::WaitQueue`, `event::park`, the
/// async executor — inherits the property instead of re-deriving it.
///
/// `deadline_tick == 0` means "no deadline".
pub fn block_until_tick_if(
    resource: u32,
    deadline_tick: u64,
    ready: impl FnOnce() -> bool,
) -> BlockOutcome {
    let g = critical::enter();
    let outcome = if ready() {
        BlockOutcome::Ready
    } else {
        unsafe { mark_blocked_locked(resource, deadline_tick) }
    };
    drop(g);
    outcome
}

/// Wake a specific task: set its wake flag, and make it runnable if it is blocked.
///
/// ISR-safe (short critical section, no allocation) and idempotent. It deliberately does **not**
/// preempt: a woken task waits for its normal turn in the ring (design rule 1).
///
/// Task ids are never reused — `take_id` is monotonic — so a stale id either names the task it came
/// from or names nothing at all. A waker that outlived its task therefore does nothing, which is
/// why no generation counter is needed on this path.
pub fn wake_task(id: u32) {
    if id == 0 {
        return;
    }
    let g = critical::enter();
    unsafe {
        let head = *KERNEL.ring_head.get();
        if !head.is_null() {
            let mut p = head;
            loop {
                if (*p).id == id {
                    (*p).flags |= crate::tcb::TCB_FLAG_WOKEN;
                    if (*p).state == TaskState::Blocked {
                        (*p).state = TaskState::Ready;
                        (*p).blocked_on = 0;
                        (*p).block_deadline = 0;
                    }
                    break;
                }
                p = (*p).next;
                if p.is_null() || p == head {
                    break;
                }
            }
        }
    }
    drop(g);
}

/// Consume the calling task's wake flag: `true` if a `wake_task` arrived and has not been taken.
///
/// # Safety
/// Caller holds the kernel critical section. The flag read belongs to the *running* task, which is
/// exactly what a blocking primitive wants to check before it blocks.
pub unsafe fn take_wake_flag_locked() -> bool {
    let me = KERNEL.current();
    if me.is_null() {
        return false;
    }
    let set = (*me).flags & crate::tcb::TCB_FLAG_WOKEN != 0;
    (*me).flags &= !crate::tcb::TCB_FLAG_WOKEN;
    set
}

/// Clear the calling task's timer deadline. Used by an executor before each poll, so only the
/// deadlines *this* poll armed are honoured.
pub fn clear_timer_deadline() {
    let g = critical::enter();
    unsafe {
        let me = KERNEL.current();
        if !me.is_null() {
            (*me).block_deadline = 0;
        }
    }
    drop(g);
}

/// Arm the calling task's timer deadline, keeping the **earliest** of the ones requested.
///
/// A task can have several pending timers at once (two futures under `join!`, say), and the tick
/// sweep can only know about one — so it gets the earliest, and each future re-arms on its next
/// poll. Whichever fires first wakes the task, which then polls all of them.
///
/// The field is the same `block_deadline` the blocking path uses: while the task is `Ready` it
/// carries "when this task wants to run again", and when the task blocks it becomes the block's own
/// deadline (the executor passes this value straight to `park`, so it is not clobbered).
pub fn set_timer_deadline(deadline_tick: u64) {
    if deadline_tick == 0 {
        return;
    }
    let g = critical::enter();
    unsafe {
        let me = KERNEL.current();
        if !me.is_null() {
            let cur = (*me).block_deadline;
            if cur == 0 || deadline_tick < cur {
                (*me).block_deadline = deadline_tick;
            }
        }
    }
    drop(g);
}

/// The calling task's timer deadline (0 = none).
pub fn task_timer_deadline() -> u64 {
    let g = critical::enter();
    let d = unsafe {
        let me = KERNEL.current();
        if me.is_null() {
            0
        } else {
            (*me).block_deadline
        }
    };
    drop(g);
    d
}

/// Drop an unconsumed wake on the floor.
///
/// An executor calls this before polling: a wake that arrives *during* the poll sets the flag
/// again, `park` then returns immediately, and the future is polled again — which is the property
/// that makes a lost wake-up impossible without any extra bookkeeping.
pub fn clear_wake_flag() {
    let g = critical::enter();
    unsafe {
        let me = KERNEL.current();
        if !me.is_null() {
            (*me).flags &= !crate::tcb::TCB_FLAG_WOKEN;
        }
    }
    drop(g);
}

/// Make the **first** task blocked on `resource` runnable, and say whether one was found. O(ring).
///
/// This is the single-waiter hand-off a `WaitQueue` needs. `wake_blocked_on` wakes everyone, which
/// is right for a lock (they contend and one wins) and wrong for a queue (it would wake the whole
/// line and hand every waiter a "signalled" that is not theirs).
pub fn wake_first_blocked_on(resource: u32) -> bool {
    let g = critical::enter();
    let mut found = false;
    unsafe {
        let head = *KERNEL.ring_head.get();
        if !head.is_null() {
            let mut p = head;
            loop {
                if (*p).state == TaskState::Blocked && (*p).blocked_on == resource {
                    (*p).flags |= crate::tcb::TCB_FLAG_WOKEN;
                    (*p).state = TaskState::Ready;
                    (*p).blocked_on = 0;
                    (*p).block_deadline = 0;
                    found = true;
                    break;
                }
                p = (*p).next;
                if p.is_null() || p == head {
                    break;
                }
            }
        }
    }
    drop(g);
    found
}

/// How many tasks are blocked on `resource` right now. O(ring).
pub fn blocked_on_count(resource: u32) -> usize {
    let g = critical::enter();
    let mut n = 0usize;
    unsafe {
        let head = *KERNEL.ring_head.get();
        if !head.is_null() {
            let mut p = head;
            loop {
                if (*p).state == TaskState::Blocked && (*p).blocked_on == resource {
                    n += 1;
                }
                p = (*p).next;
                if p.is_null() || p == head {
                    break;
                }
            }
        }
    }
    drop(g);
    n
}

/// Give up the rest of the slice but stay `Ready`: the caller goes to the back of the round.
///
/// This is the "skip to the next task immediately" primitive. An `nb::WouldBlock`, a `Pending`
/// future, an application polling a peripheral with no interrupt, or code that simply has nothing
/// useful to do until its next turn all reduce to this call. It costs one switch and never blocks,
/// so the caller stays runnable and cannot starve.
///
/// With nothing else runnable there is nowhere to yield to and the call is a no-op: the caller
/// keeps its slice (the tick still fires) rather than spinning it away.
pub fn yield_now() {
    crate::arch::request_switch();
}

/// A task identity that stays meaningful after the task is gone.
///
/// Ids are monotonic and never reused (`take_id`), so a `TaskId` held by a waker either refers to
/// the task it was taken from or to nothing at all. That is the whole safety story for stale
/// wakers: no generation counter, no lookup table, and no way to wake a recycled TCB by accident.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskId(u32);

impl TaskId {
    /// The raw id, as stored in the TCB. `0` means "no task".
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// True for the null id, which `current_id` returns outside a task.
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }
}

/// The calling task's id, or the null [`TaskId`] when called outside a task (interrupt or idle).
pub fn current_id() -> TaskId {
    TaskId(current_task_id())
}

/// Id allocator for `event` resources (`WaitQueue`), kept out of the task-id space.
///
/// Task ids start at 1 and grow with every spawn; resource ids start at `0x4000_0000`, so the two
/// spaces cannot collide in any realistic lifetime. A critical-section counter rather than an
/// `AtomicU32`, so the same code compiles on Cortex-M0 and AVR, which have no atomic
/// read-modify-write at all.
struct ResourceIds(UnsafeCell<u32>);

// SAFETY: only touched inside `critical::enter`.
unsafe impl Sync for ResourceIds {}

static NEXT_RESOURCE_ID: ResourceIds = ResourceIds(UnsafeCell::new(0x4000_0000));

/// Take a resource id for a new `event::WaitQueue`.
pub fn alloc_resource_id() -> u32 {
    let g = critical::enter();
    let id = unsafe { *NEXT_RESOURCE_ID.0.get() };
    unsafe { *NEXT_RESOURCE_ID.0.get() = id.wrapping_add(1) };
    drop(g);
    id
}

/// Block the calling task for **at least** `ticks` slice ticks.
///
/// The deadline is computed *inside* the critical section. Reading the tick first and blocking
/// afterwards — which is what this used to do — leaves a window: a tick landing in it makes the
/// deadline one tick short, so the sleep returns early. The symptom is intermittent and shows up
/// most often right after a spawn or an exit, because that is when a switch and a just-latched
/// tick most often coincide.
pub fn block_for_ticks(resource: u32, ticks: u64) -> BlockOutcome {
    let g = critical::enter();
    let deadline = unsafe { *KERNEL.ticks.get() }.wrapping_add(ticks);
    let outcome = unsafe { mark_blocked_locked(resource, deadline) };
    drop(g);
    outcome
}

/// Absolute wake-up tick `ticks` from now, captured atomically with the counter.
///
/// For callers that need **one** deadline across several waits (a retry loop that times out).
/// Documented tolerance: a tick landing between this call and the first block shifts the
/// deadline by one tick, so a timeout can fire up to one slice early. Sleeps deliberately do not
/// have that tolerance — they compute their deadline inside the blocking section.
pub fn deadline_after_ticks(ticks: u64) -> u64 {
    let g = critical::enter();
    let d = unsafe { *KERNEL.ticks.get() }.wrapping_add(ticks);
    drop(g);
    d
}

/// Sleep for at least `ticks` slice ticks (the only clock the kernel has).
pub fn sleep_ticks(ticks: u64) {
    let _ = block_for_ticks(0, ticks);
}

/// Sleep until an absolute tick, tolerating spurious wake-ups: a wake that arrives early simply
/// blocks again, because the loop re-checks the deadline.
///
/// What this **cannot** fix: a clock that jumps forward. A double-counted tick moves `now()`
/// past the deadline and the loop exits immediately — that is a time-base fault, not a wake-up
/// fault, and it is why the deadline clock moves off the slice counter in the next phase.
pub fn sleep_until_tick(deadline_tick: u64) {
    while tick_count() < deadline_tick {
        match block_until_tick(0, deadline_tick) {
            BlockOutcome::Blocked => {}
            // `block_until_tick` has no condition, so it never reports this; the shared enum simply
            // has to name it.
            BlockOutcome::Ready => {}
            // Deadline passed while we were deciding: done.
            BlockOutcome::AlreadyPast => break,
            // Cannot block here at all; stop rather than spin.
            BlockOutcome::NoCurrentTask => break,
        }
    }
}

/// Make every task blocked on `resource` runnable again. O(ring).
///
/// Called by `Mutex::unlock`. A per-lock waiter list would make this O(waiters) at the
/// cost of another intrusive link in every TCB; at the task counts this kernel targets
/// the ring walk is the cheaper trade, and the *context switch* it triggers is O(1).
pub fn wake_blocked_on(resource: u32) {
    let g = critical::enter();
    unsafe {
        let head = *KERNEL.ring_head.get();
        if !head.is_null() {
            let mut p = head;
            loop {
                if (*p).state == TaskState::Blocked && (*p).blocked_on == resource {
                    (*p).state = TaskState::Ready;
                    (*p).blocked_on = 0;
                    (*p).block_deadline = 0;
                }
                p = (*p).next;
                if p.is_null() || p == head {
                    break;
                }
            }
        }
    }
    drop(g);
}

/// Wake tasks whose bounded wait has expired.
///
/// # Safety
/// Called from the tick path, with preemption masked.
unsafe fn wake_expired() {
    let now = *KERNEL.ticks.get();
    let head = *KERNEL.ring_head.get();
    if head.is_null() {
        return;
    }
    let mut p = head;
    loop {
        if (*p).state == TaskState::Blocked
            && (*p).block_deadline != 0
            && now >= (*p).block_deadline
        {
            (*p).state = TaskState::Ready;
            (*p).blocked_on = 0;
            (*p).block_deadline = 0;
        }
        p = (*p).next;
        if p.is_null() || p == head {
            break;
        }
    }
}

/// Times a blocking wait was attempted with no current task (interrupt or idle context).
///
/// Must stay zero: a non-zero value means some path called `sleep`/`lock`/`park` from a context
/// that has nothing to switch away from, so that wait did not happen at all. It is a counter as
/// well as a debug assertion because the assertion is compiled out in release, and this is
/// exactly the class of bug that looks like "the sleep woke up early".
pub fn blocks_without_current() -> u64 {
    let g = critical::enter();
    let n = unsafe { *KERNEL.blocks_without_current.get() };
    drop(g);
    n
}

/// Number of tasks currently blocked (waiting for a lock, or sleeping).
pub fn blocked_threads() -> usize {
    let g = critical::enter();
    let mut n = 0usize;
    unsafe {
        let head = *KERNEL.ring_head.get();
        if !head.is_null() {
            let mut p = head;
            loop {
                if (*p).state == TaskState::Blocked {
                    n += 1;
                }
                p = (*p).next;
                if p.is_null() || p == head {
                    break;
                }
            }
        }
    }
    drop(g);
    n
}

/// Set how many cores run the scheduler (validated against the target's maximum).
///
/// Cores at or above `n` park in a low-power wait loop instead of pulling tasks.
pub fn set_active_cores(n: usize) -> Result<(), ConfigError> {
    let max = crate::smp::max_cores().max(1);
    if n == 0 || n > max {
        return Err(ConfigError::Platform(
            "active_cores outside 1..=smp::max_cores() for this target",
        ));
    }
    if n > 1 && !crate::smp::supports_smp() {
        return Err(ConfigError::Platform(
            "this target has no cross-core interlock, so it cannot run more than one core",
        ));
    }
    let g = critical::enter();
    unsafe { (*KERNEL.config.get()).active_cores = n };
    drop(g);
    Ok(())
}

/// How many cores are running the scheduler.
pub fn active_cores() -> usize {
    let g = critical::enter();
    let n = unsafe { (*KERNEL.config.get()).active_cores };
    drop(g);
    n
}

/// Allocate and zero a TCB from the kernel arena. Caller holds a critical
/// section.
pub(crate) unsafe fn alloc_tcb() -> Option<*mut TaskControlBlock> {
    let p = (*arena()).alloc(
        core::mem::size_of::<TaskControlBlock>(),
        core::mem::align_of::<TaskControlBlock>(),
    )? as *mut TaskControlBlock;
    ptr::write_bytes(p as *mut u8, 0, core::mem::size_of::<TaskControlBlock>());
    Some(p)
}

/// Hand out the next task id.
pub(crate) unsafe fn take_id() -> u32 {
    let id = *KERNEL.next_id.get();
    *KERNEL.next_id.get() = id.wrapping_add(1);
    id
}

/// Kernel-context reclamation of finished tasks.
///
/// A returning task cannot unmap the stack it is still standing on, so
/// `task_exit` only *unlinks* and pushes itself onto `pending_free`. This
/// function — which must run in a context that is **not** on the finished task's
/// stack — performs the actual `Arena::free` calls. That is what keeps
/// `active_threads`, the ring and the arena consistent with each other: dead
/// nodes leave the ring in O(1), their memory is recycled soon after, and
/// repeated spawn/exit is steady-state.
///
/// # Where each port calls it
/// * **Cortex-M** — from `PendSV`, which runs on MSP (the kernel stack): called
///   *before* the switch, which is safe because the dying task's PSP stack is
///   not where the handler is running.
/// * **Win32** — from the tick thread, before the switch. `is_pinned` keeps the
///   TCB the tick thread still needs from being recycled underneath it.
/// * **POSIX fibres** — *after* the switch (the handler shares the interrupted
///   fibre's stack, so the dying stack must not be released from inside it; the
///   next handler invocation runs on a different fibre's stack).
///
/// # Safety
/// Kernel context, with the kernel lock/critical section held.
pub unsafe fn reclaim_finished_tasks() {
    let mut node = *KERNEL.pending_free.get();
    *KERNEL.pending_free.get() = ptr::null_mut();

    let ar = &mut *arena();
    while !node.is_null() {
        // `prev` doubles as the deferred-free list link for a dead TCB.
        let next = (*node).prev;

        debug_assert!(!(*node).is_linked(), "dead task still linked in the ring");
        debug_assert_eq!((*node).state, TaskState::Dead);

        if crate::arch::is_pinned(node) {
            // The port still needs this TCB — typically because it is the task
            // the tick thread has just taken the CPU away from, and its backend
            // resources (Win32 thread handle, fiber stack) must stay valid until
            // that switch has fully completed. Re-queue it for the next drain;
            // this pins at most one node, so the list stays bounded.
            (*node).prev = *KERNEL.pending_free.get();
            *KERNEL.pending_free.get() = node;
            node = next;
            continue;
        }

        // Let the backend release its own resources (Win32 thread handle, ...)
        // before the TCB itself is recycled.
        crate::arch::on_task_reclaimed(node);

        ar.free((*node).closure_block);
        ar.free((*node).stack_base);
        let n = node;
        node = next;
        ar.free(n as *mut u8);

        *KERNEL.reclaimed.get() += 1;
    }
}

/// Push a finished task onto the deferred-free list. Called by the trampoline
/// from the dying task's own context, with the ring already unlinked and the
/// counters already decremented.
///
/// # Safety
/// Critical section held; `tcb` must be unlinked and marked `Dead`.
pub(crate) unsafe fn push_pending_free(tcb: *mut TaskControlBlock) {
    (*tcb).prev = *KERNEL.pending_free.get();
    *KERNEL.pending_free.get() = tcb;
}

/// Bodies of `thread::spawn*`. Takes the critical section itself.
pub(crate) unsafe fn spawn_internal<F>(f: F, stack_size: usize) -> Result<(), crate::SpawnError>
where
    F: FnOnce() + Send + 'static,
{
    let g = critical::enter();

    let ar = arena();
    if !(*ar).is_ready() {
        drop(g);
        return Err(crate::SpawnError::NotInitialized);
    }

    // 1. TCB.
    let tcb = match alloc_tcb() {
        Some(t) => t,
        None => {
            drop(g);
            return Err(crate::SpawnError::ArenaExhausted);
        }
    };

    // 2. Closure blob (the *task body*), placed without any heap.
    let csize = crate::closure::block_size_for::<F>();
    let calign = crate::closure::block_align_for::<F>();
    let blob = match (*arena()).alloc(csize, calign) {
        Some(b) => b,
        None => {
            (*arena()).free(tcb as *mut u8);
            drop(g);
            return Err(crate::SpawnError::ArenaExhausted);
        }
    };
    crate::closure::place::<F>(blob, f);
    (*tcb).closure_block = blob;

    (*tcb).id = take_id();
    (*tcb).state = TaskState::Ready;
    (*tcb).slices_run = 0;
    (*tcb).switches = 0;

    // 3. Backend-specific: stack, initial register frame, thread/fiber.
    if let Err(e) = crate::arch::create_task(tcb, stack_size) {
        crate::closure::drop_blob::<F>(blob);
        (*arena()).free(blob);
        (*arena()).free(tcb as *mut u8);
        drop(g);
        return Err(e);
    }

    // 4. Link into the live ring, *after* the current task: the newcomer runs
    //    immediately next, instead of waiting a whole lap.
    let cur = KERNEL.current();
    crate::ring::insert_after(cur, tcb);
    if cur.is_null() {
        (*tcb).state = TaskState::Running;
        KERNEL.set_current(tcb);
    }
    *KERNEL.ring_head.get() = tcb;
    *KERNEL.total_threads.get() += 1;
    *KERNEL.active_threads.get() += 1;

    debug_assert!(crate::ring::check(KERNEL.current()).is_ok());

    // 5. Register the newcomer instantly: kick the switch path and restart the
    //    slice counter, so the new task gets a full, unreserved slice.
    if *KERNEL.running.get() {
        crate::arch::request_switch();
        // On bare metal the pending PendSV fires as soon as this critical
        // section exits; on the host the tick thread wakes (blocking on the
        // kernel lock until we drop it) and performs the switch plus the tick
        // reset.
    }
    drop(g);
    Ok(())
}

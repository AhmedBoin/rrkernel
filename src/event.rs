//! Events: `park`, `Signal`, `WaitQueue`, and the poll-yield fallback.
//!
//! Everything here is a thin layer over [`crate::scheduler::block_until_tick_if`], and that is the
//! point: "check the condition and block" happens inside **one** critical section, so a wake-up from
//! an ISR cannot fall between the two steps. Each type below therefore inherits race-freedom rather
//! than re-deriving it — the mistake that let `sync::Mutex::lock` block forever on a free lock.
//!
//! None of this needs `alloc`, atomics, or a TCB field: waiters block on a resource id (the task's
//! own id for `Signal`, an allocated resource id for `WaitQueue`), which is the same keyed-blocking
//! mechanism the mutex already uses.

/// Re-exported so an event user names the same type the scheduler does.
pub use crate::scheduler::TaskId;
use crate::scheduler::{self, BlockOutcome};
use core::cell::UnsafeCell;

/// A wait that ran out of time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timeout;

impl core::fmt::Display for Timeout {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "wait timed out")
    }
}

/// Why [`park`] returned.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeReason {
    /// [`wake`] reached this task.
    Woken,
    /// The deadline passed first.
    Timeout,
}

/// Park the calling task until [`wake`] reaches it, or the deadline passes.
///
/// Spurious returns are allowed and this loop tolerates them: it re-checks its own wake flag every
/// time it resumes, so a wake that arrived *before* the park is not lost, and a wake during it ends
/// the park. That flag lives in the TCB's `flags` byte (`TCB_FLAG_WOKEN`), so parking costs no
/// allocation and no extra field.
///
/// `deadline` is an absolute tick count, or `None` for an unbounded wait — which callers must opt
/// into explicitly (design rule 7: no API blocks forever by accident).
pub fn park(deadline: Option<u64>) -> WakeReason {
    // Note: no early return for "no current task". The blocking primitive below is the one place
    // that decides what that means, and it counts the attempt and asserts in debug builds — an
    // event that fails quietly here would hide exactly the class of bug this layer exists to keep
    // visible.
    let me = scheduler::current_task_id();
    let d = deadline.unwrap_or(0);
    loop {
        match scheduler::block_until_tick_if(me, d, || unsafe {
            scheduler::take_wake_flag_locked()
        }) {
            // A wake arrived before we blocked: consume it, never park.
            BlockOutcome::Ready => return WakeReason::Woken,
            // Resumed: woken, or the deadline passed. The next iteration decides.
            BlockOutcome::Blocked => {}
            BlockOutcome::AlreadyPast => return WakeReason::Timeout,
            BlockOutcome::NoCurrentTask => return WakeReason::Timeout,
        }
    }
}

/// Wake a parked (or sleeping) task by id.
///
/// ISR-safe, allocation-free, idempotent. Ids are never reused, so a stale id is harmless: see
/// [`crate::scheduler::TaskId`].
pub fn wake(id: TaskId) {
    scheduler::wake_task(id.raw());
}

/// Poll-yield fallback, for peripherals with neither interrupt nor DMA.
///
/// Stays `Ready` and gives up the rest of each slice, so it costs one switch per poll and cannot
/// starve anyone. It is also the only wait in this crate that burns CPU, which is why the deadline
/// is mandatory rather than optional.
pub fn wait_until(mut ready: impl FnMut() -> bool, deadline_tick: u64) -> Result<(), Timeout> {
    while !ready() {
        if scheduler::tick_count() >= deadline_tick {
            return Err(Timeout);
        }
        scheduler::yield_now();
    }
    Ok(())
}

/// A one-shot signal that an ISR can set and a task can wait on.
///
/// Critical sections rather than atomics, so this exists on *every* target including Cortex-M0 and
/// AVR (the same reason `app_support`'s log cell does). The sections are a handful of instructions
/// and never block, so `set` is safe to call from an interrupt handler.
///
/// Single-waiter by design: `set` hands the signal to whichever task registered as the waiter. For
/// several waiters use [`WaitQueue`].
pub struct Signal {
    set: UnsafeCell<bool>,
    waiter: UnsafeCell<u32>,
}

// SAFETY: every access goes through `crate::critical::enter`.
unsafe impl Sync for Signal {}

impl Default for Signal {
    fn default() -> Self {
        Self::new()
    }
}

impl Signal {
    /// A new, unset signal. `const`, so it can be a `static`.
    pub const fn new() -> Self {
        Signal {
            set: UnsafeCell::new(false),
            waiter: UnsafeCell::new(0),
        }
    }

    /// Set the signal and wake the registered waiter, if any. **ISR-safe.**
    ///
    /// The flag is stored *before* the waiter is read, and both happen in one section, so a task
    /// inside `wait`'s section either sees `set == true` or is already registered as the waiter —
    /// never neither. That is the entire lost-wakeup argument, and it is one paragraph long because
    /// the primitive underneath it does the hard part.
    pub fn set(&self) {
        let waiter = {
            let g = crate::critical::enter();
            unsafe { *self.set.get() = true };
            let id = unsafe { *self.waiter.get() };
            unsafe { *self.waiter.get() = 0 };
            drop(g);
            id
        };
        if waiter != 0 {
            scheduler::wake_task(waiter);
        }
    }

    /// Take the signal without blocking: `true` if it was set (and now is not). ISR-safe.
    pub fn try_take(&self) -> bool {
        let g = crate::critical::enter();
        let was = unsafe { *self.set.get() };
        unsafe { *self.set.get() = false };
        drop(g);
        was
    }

    /// Is the signal set, without consuming it?
    pub fn is_set(&self) -> bool {
        let g = crate::critical::enter();
        let v = unsafe { *self.set.get() };
        drop(g);
        v
    }

    /// Wait for the signal, or give up at `deadline` (absolute ticks; `None` = wait forever).
    ///
    /// Returns `Ok(())` once the signal has been consumed. A signal that is already set succeeds
    /// even outside a task (consuming it blocks nothing); otherwise a wait from interrupt or idle
    /// context cannot block, and the scheduler reports that — loudly in debug builds, as [`Timeout`]
    /// in release.
    pub fn wait(&self, deadline: Option<u64>) -> Result<(), Timeout> {
        // No early return for "no current task": see the note in `park`.
        let me = scheduler::current_task_id();
        let d = deadline.unwrap_or(0);
        loop {
            let outcome = scheduler::block_until_tick_if(me, d, || unsafe {
                if *self.set.get() {
                    *self.set.get() = false;
                    true
                } else {
                    *self.waiter.get() = me;
                    false
                }
            });
            match outcome {
                BlockOutcome::Ready => return Ok(()),
                // Woken (or a spurious resume): loop, so the next turn consumes the flag.
                BlockOutcome::Blocked => {}
                BlockOutcome::AlreadyPast => return Err(Timeout),
                BlockOutcome::NoCurrentTask => return Err(Timeout),
            }
        }
    }
}

/// A wait queue: several tasks can wait on it, with no allocation and no TCB field.
///
/// Waiters block on a resource id from [`crate::scheduler::alloc_resource_id`], so a queue costs one
/// `u32`. Waking is O(ring) rather than O(waiters) — the same trade the mutex makes, and the reason
/// the design refuses to add an intrusive waiter link to every TCB.
///
/// Unlike a lock, a queue hands its wake to **one** waiter at a time ([`WaitQueue::wake_one`]):
/// `wake_blocked_on` would wake the whole line, and every waiter would be told "signalled" when the
/// signal belonged to one of them.
pub struct WaitQueue {
    id: u32,
}

impl Default for WaitQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl WaitQueue {
    /// A queue with an id from the shared resource allocator.
    pub fn new() -> Self {
        WaitQueue {
            id: scheduler::alloc_resource_id(),
        }
    }

    /// A queue with an explicit id, usable in a `const`/`static`.
    ///
    /// The id must be unique across *all* resources in the system (locks and other queues included),
    /// so a `static` queue should pick from the resource range, e.g. `0x4000_0000 + n`.
    pub const fn with_id(id: u32) -> Self {
        WaitQueue { id }
    }

    /// The resource id this queue waits on.
    pub const fn id(&self) -> u32 {
        self.id
    }

    /// Block until [`WaitQueue::wake_one`] or [`WaitQueue::wake_all`] reaches this task, or the
    /// deadline passes.
    pub fn wait(&self, deadline: Option<u64>) -> Result<(), Timeout> {
        // No early return for "no current task": the blocking primitive refuses, counts and asserts.
        // (No `me` binding either: this queue's resource id is its own, not the caller's task id.)
        let d = deadline.unwrap_or(0);
        loop {
            // The condition ("has this task been woken?") is the wake flag, checked inside the same
            // section that blocks, so a wake that arrives first is remembered rather than lost.
            match scheduler::block_until_tick_if(self.id, d, || unsafe {
                scheduler::take_wake_flag_locked()
            }) {
                BlockOutcome::Ready => return Ok(()),
                BlockOutcome::Blocked => {}
                BlockOutcome::AlreadyPast => return Err(Timeout),
                BlockOutcome::NoCurrentTask => return Err(Timeout),
            }
        }
    }

    /// Wake the first waiter, if any. **ISR-safe.** Returns whether one was waiting.
    pub fn wake_one(&self) -> bool {
        scheduler::wake_first_blocked_on(self.id)
    }

    /// Wake every waiter. ISR-safe. Returns how many were waiting.
    pub fn wake_all(&self) -> usize {
        let n = self.waiters();
        scheduler::wake_blocked_on(self.id);
        n
    }

    /// How many tasks are blocked on this queue right now.
    pub fn waiters(&self) -> usize {
        scheduler::blocked_on_count(self.id)
    }
}

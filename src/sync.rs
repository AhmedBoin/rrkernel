//! Sleeping locks, static lock-order enforcement, and timeout/back-off recovery.
//!
//! The scheduler is pure round robin with no priorities, so the classic answers are
//! unavailable: there is no priority inheritance, and a task that spins on a held lock
//! burns a slice that could have made progress. Instead:
//!
//! * **[`Mutex`] sleeps, it does not spin.** A failed acquisition marks the caller
//!   `Blocked` on the lock's [`LockId`] and asks the port for an immediate switch — O(1),
//!   so the caller leaves the CPU at once. [`Mutex::unlock`] wakes that id's waiters.
//! * **Lock ordering is enforced, not documented.** Each task records the ids of the
//!   locks it holds; acquiring a lock whose id is not *greater* than the last one held is
//!   refused with [`LockError::OrderViolation`]. A program that creates locks in the
//!   order it takes them therefore cannot form a circular wait.
//! * **[`Mutex::try_lock_for`] bounds the wait**, and [`Mutex::lock_with_backoff`]
//!   implements the runtime recovery: release everything held, back off, retry.
//!
//! Blocking is safe here because a task that blocks never holds a *kernel* lock: kernel
//! spinlocks ([`crate::smp::SpinLock`]) are only held across straight-line critical
//! sections that cannot block, and [`Mutex::lock`] is only reachable from task code.

use crate::smp::SpinLock;
use crate::tcb::KERNEL;
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

/// A unique, monotonically increasing lock identity.
///
/// The ordering discipline is expressed over these ids: handed out in creation order,
/// "acquire in increasing id order" is a total order over every lock in the system, which
/// is what makes circular wait impossible.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct LockId(pub u32);

impl LockId {
    /// A hand-assigned id, for locks that must be `static` (see [`Mutex::with_id`]).
    pub const fn new(raw: u32) -> LockId {
        LockId(raw)
    }
}

static NEXT_LOCK_ID: AtomicU32 = AtomicU32::new(1);

fn alloc_lock_id() -> LockId {
    LockId(NEXT_LOCK_ID.fetch_add(1, Ordering::Relaxed))
}

/// Why an acquisition failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LockError {
    /// `try_lock_for` gave up: the wait exceeded the timeout.
    Timeout,
    /// The caller holds a lock with a higher id, so taking this one would create a
    /// lock-order inversion — the precondition for a circular wait.
    OrderViolation {
        /// The lock already held with the higher id.
        held: LockId,
        /// The lock that was refused.
        requested: LockId,
    },
    /// The caller already owns this lock.
    AlreadyOwnedBySelf,
    /// The scheduler is not running, so no task can block (called before init).
    NotRunning,
}

impl core::fmt::Display for LockError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            LockError::Timeout => write!(f, "lock acquisition timed out"),
            LockError::OrderViolation { held, requested } => write!(
                f,
                "lock order violation: holding {} while acquiring {} \
                 (acquire in increasing LockId order)",
                held.0, requested.0
            ),
            LockError::AlreadyOwnedBySelf => write!(f, "lock already held by this task"),
            LockError::NotRunning => write!(f, "scheduler not running"),
        }
    }
}

/// Depth of the per-task held-lock stack (defined with the TCB, re-exported here).
pub use crate::tcb::MAX_HELD_LOCKS;

/// The type-erased part of a [`Mutex`], so a registry can release locks without
/// knowing `T` (see [`release_all_held_locks`]).
pub struct MutexHeader {
    id: LockId,
    /// 0 when free, otherwise the owning task id.
    owner: AtomicU32,
    /// Guards `owner` and the held-lock registry, with local interrupts masked.
    pub(crate) state: SpinLock,
    /// Releases this mutex: the type-erased [`release_shim`].
    pub(crate) release: fn(&MutexHeader),
}

/// A lock whose waiters sleep rather than spin.
pub struct Mutex<T> {
    pub(crate) header: MutexHeader,
    data: UnsafeCell<T>,
}

// SAFETY: `T: Send` is what may cross a core boundary inside the lock, and the header's
// spinlock makes the ownership bookkeeping race-free.
unsafe impl<T: Send> Sync for Mutex<T> {}
unsafe impl<T: Send> Send for Mutex<T> {}

impl<T> Mutex<T> {
    /// A new lock, taking the next [`LockId`] from the monotonic allocator.
    ///
    /// Runtime rather than `const` for exactly that reason; for a `static` lock use
    /// [`Mutex::with_id`].
    pub fn new(data: T) -> Self {
        Mutex {
            header: MutexHeader {
                id: alloc_lock_id(),
                owner: AtomicU32::new(0),
                state: SpinLock::new(),
                release: release_shim,
            },
            data: UnsafeCell::new(data),
        }
    }

    /// A lock with an explicitly chosen id, usable in a `const`/`static`.
    ///
    /// The id is the lock's place in the ordering hierarchy: assign them in the order you
    /// intend to acquire, e.g. `LockId::new(1)` outer and `LockId::new(2)` inner.
    pub const fn with_id(id: LockId, data: T) -> Self {
        Mutex {
            header: MutexHeader {
                id,
                owner: AtomicU32::new(0),
                state: SpinLock::new(),
                release: release_shim,
            },
            data: UnsafeCell::new(data),
        }
    }

    /// This lock's identity and its position in the ordering hierarchy.
    #[inline]
    pub fn id(&self) -> LockId {
        self.header.id
    }

    /// The owning task id, or 0 when free.
    #[inline]
    pub fn owner(&self) -> u32 {
        self.header.owner.load(Ordering::Relaxed)
    }

    /// Whether this lock is free right now.
    #[inline]
    pub fn is_free(&self) -> bool {
        self.owner() == 0
    }

    /// One non-blocking attempt, enforcing lock order on the way in.
    fn try_acquire(&self) -> Result<bool, LockError> {
        let me = crate::scheduler::current_task_id();
        if me == 0 {
            return Err(LockError::NotRunning);
        }
        let _g = self.header.state.lock();
        let owner = self.header.owner.load(Ordering::Relaxed);
        if owner == me {
            return Err(LockError::AlreadyOwnedBySelf);
        }
        if owner != 0 {
            return Ok(false);
        }
        // Free: check the ordering rule *before* taking it, so a refused acquisition
        // leaves nothing behind.
        unsafe { held_check_order(self.header.id)? };
        self.header.owner.store(me, Ordering::Relaxed);
        unsafe { held_push(self.header.id) };
        Ok(true)
    }

    /// Take the lock, or fail immediately if it is held.
    pub fn try_lock(&self) -> Result<MutexGuard<'_, T>, LockError> {
        if self.try_acquire()? {
            return Ok(MutexGuard { lock: self });
        }
        Err(LockError::Timeout)
    }

    /// Take the lock, **sleeping** until it is free.
    ///
    /// On contention the caller's TCB is marked `Blocked` on this lock's id and the port
    /// is asked for an immediate switch, so the waiter stops consuming CPU instead of
    /// spinning through its slice.
    pub fn lock(&self) -> Result<MutexGuard<'_, T>, LockError> {
        loop {
            if self.try_acquire()? {
                return Ok(MutexGuard { lock: self });
            }
            crate::scheduler::block_current(self.header.id.0, 0);
        }
    }

    /// Take the lock, giving up after `timeout_ms` (measured in scheduler ticks, the
    /// only clock this kernel has).
    pub fn try_lock_for(&self, timeout_ms: u64) -> Result<MutexGuard<'_, T>, LockError> {
        if self.try_acquire()? {
            return Ok(MutexGuard { lock: self });
        }
        let deadline = crate::scheduler::tick_count() + ms_to_ticks(timeout_ms);
        loop {
            crate::scheduler::block_current(self.header.id.0, deadline);
            // Re-try before checking the clock: if the lock was released *and* the
            // deadline passed in the same tick, succeeding is the better answer.
            if self.try_acquire()? {
                return Ok(MutexGuard { lock: self });
            }
            if crate::scheduler::tick_count() >= deadline {
                return Err(LockError::Timeout);
            }
        }
    }

    /// Acquire with the runtime deadlock-avoidance protocol.
    ///
    /// On timeout the recovery is: **release every lock this task still holds**, wait a
    /// bounded back-off, then try again. Releasing is what breaks the cycle — a task
    /// waiting for a lock while holding one its partner needs *is* the circular-wait
    /// condition, and dropping its holdings dissolves it. `attempt` is a closure so the
    /// caller's whole acquisition sequence is retried, not just this one lock.
    pub fn lock_with_backoff<F, R>(
        &self,
        timeout_ms: u64,
        attempts: u32,
        mut attempt: F,
    ) -> Result<R, LockError>
    where
        F: FnMut(&Mutex<T>) -> Result<R, LockError>,
    {
        let mut tries = 0u32;
        loop {
            match attempt(self) {
                Ok(ok) => return Ok(ok),
                Err(LockError::Timeout) if tries + 1 < attempts => {
                    let released = release_all_held_locks();
                    backoff(tries, released + (timeout_ms as usize % 3));
                    tries += 1;
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// Release the lock and wake whoever is blocked on it.
    pub fn unlock(&self) {
        self.release_lock();
    }

    /// The release half of [`Mutex::unlock`], named apart from the header's function
    /// pointer field.
    pub(crate) fn release_lock(&self) {
        (self.header.release)(&self.header);
    }
}

/// The guard returned by every successful acquisition.
#[must_use = "the lock is held only for the lifetime of this guard"]
pub struct MutexGuard<'a, T> {
    lock: &'a Mutex<T>,
}

impl<T> core::ops::Deref for MutexGuard<'_, T> {
    type Target = T;
    #[inline]
    fn deref(&self) -> &T {
        unsafe { &*self.lock.data.get() }
    }
}

impl<T> core::ops::DerefMut for MutexGuard<'_, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.lock.data.get() }
    }
}

impl<T> Drop for MutexGuard<'_, T> {
    #[inline]
    fn drop(&mut self) {
        self.lock.release_lock();
    }
}

/// The type-erased release stored in [`MutexHeader::release`].
///
/// Deliberately **not** generic: releasing touches only the header — the owner word, the
/// held-lock registry and the wake-up walk — so there is nothing to monomorphise over `T`.
/// It used to be `release_shim::<T>`, which clippy flagged as an unused type parameter and
/// which implied per-type behaviour that does not exist.
fn release_shim(header: &MutexHeader) {
    {
        let _g = header.state.lock();
        header.owner.store(0, Ordering::Relaxed);
    }
    unsafe { held_remove(header.id) };
    // Wake every task sleeping on this lock. The context switch that follows is O(1);
    // this walk is over the ring, which the scheduler traverses anyway.
    crate::scheduler::wake_blocked_on(header.id.0);
}

/// Milliseconds to slice ticks.
fn ms_to_ticks(ms: u64) -> u64 {
    let slice_ns = crate::scheduler::slice_ns();
    if slice_ns == 0 {
        return ms;
    }
    ms.saturating_mul(1_000_000).div_ceil(slice_ns)
}

/// A deterministic pseudo-random back-off, in ticks.
///
/// Deterministic on purpose: a `no_std` kernel has no RNG, and what the protocol needs
/// is a bounded, attempt-dependent delay. `jitter` lets the caller mix in something
/// task-specific (e.g. how many locks were just released) so two tasks do not back off
/// in lockstep forever.
pub fn backoff(attempt: u32, jitter: usize) {
    let ticks = (1u64 << attempt.min(6)) + (jitter as u64 % 3);
    crate::scheduler::sleep_ticks(ticks);
}

// ---------------------------------------------------------------------------
// Held-lock bookkeeping (the ordering rule) and the recovery registry
// ---------------------------------------------------------------------------

/// Locks registered for [`release_all_held_locks`], so a task can drop everything it
/// holds without the kernel having to know which locks exist.
///
/// `AtomicPtr` rather than `static mut` slots: registration and lookup both happen under
/// `REGISTRY`, but atomics keep that fact out of the type system's way (and out of the
/// `static_mut_refs` lint, which exists precisely because a `&mut` into a static is a
/// footgun).
static REGISTRY: SpinLock = SpinLock::new();
static REGISTRY_SLOTS: [AtomicPtr<MutexHeader>; 8] =
    [const { AtomicPtr::new(core::ptr::null_mut()) }; 8];

impl<T: Send> Mutex<T> {
    /// Register this lock so [`release_all_held_locks`] can release it.
    ///
    /// Only meaningful for `static` locks (the reference has to outlive the kernel), and
    /// only needed by tasks that use the back-off recovery path.
    pub fn register(&'static self) {
        let _g = REGISTRY.lock();
        let slot_ptr = &self.header as *const MutexHeader as *mut MutexHeader;
        for slot in REGISTRY_SLOTS.iter() {
            if slot.load(Ordering::Relaxed).is_null() {
                let _ = slot.compare_exchange(
                    core::ptr::null_mut(),
                    slot_ptr,
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                );
                return;
            }
        }
    }
}

fn registered(id: LockId) -> Option<&'static MutexHeader> {
    let _g = REGISTRY.lock();
    for slot in REGISTRY_SLOTS.iter() {
        let p = slot.load(Ordering::Relaxed);
        if !p.is_null() {
            // SAFETY: a slot is only ever filled from a `&'static Mutex<T>`, so the
            // header is valid for the life of the program.
            let h = unsafe { &*p };
            if h.id == id {
                return Some(h);
            }
        }
    }
    None
}

/// Refuse an acquisition that would invert the lock order.
///
/// This is the *static* half of the deadlock story: because ids are handed out in
/// creation order, "always acquire with increasing id" is a total order over every lock,
/// and a cycle cannot exist in a total order. Callers that respect it can never produce
/// the circular-wait condition — which is why it is enforced here rather than documented.
///
/// # Safety
/// Reads the current TCB, so it must run with preemption masked (it is called from
/// `Mutex::try_acquire`, which holds the lock's own spinlock).
unsafe fn held_check_order(id: LockId) -> Result<(), LockError> {
    let me = KERNEL.current();
    if me.is_null() {
        return Ok(());
    }
    let n = (*me).held_count as usize;
    if n == 0 || n > MAX_HELD_LOCKS {
        return Ok(());
    }
    let last = LockId((*me).held_locks[n - 1]);
    if id.0 <= last.0 {
        return Err(LockError::OrderViolation {
            held: last,
            requested: id,
        });
    }
    Ok(())
}

unsafe fn held_push(id: LockId) {
    let me = KERNEL.current();
    if me.is_null() {
        return;
    }
    let n = (*me).held_count as usize;
    if n < MAX_HELD_LOCKS {
        (*me).held_locks[n] = id.0;
        (*me).held_count = (n + 1) as u8;
    }
}

unsafe fn held_remove(id: LockId) {
    let me = KERNEL.current();
    if me.is_null() {
        return;
    }
    let n = ((*me).held_count as usize).min(MAX_HELD_LOCKS);
    let mut k = 0;
    while k < n {
        if (*me).held_locks[k] == id.0 {
            let mut j = k;
            while j + 1 < n {
                (*me).held_locks[j] = (*me).held_locks[j + 1];
                j += 1;
            }
            (*me).held_count -= 1;
            return;
        }
        k += 1;
    }
}

unsafe fn held_clear() {
    let me = KERNEL.current();
    if !me.is_null() {
        (*me).held_count = 0;
    }
}

/// How many locks the calling task currently holds.
pub fn held_count() -> usize {
    let g = crate::critical::enter();
    let n = unsafe {
        let me = KERNEL.current();
        if me.is_null() {
            0
        } else {
            (*me).held_count as usize
        }
    };
    drop(g);
    n
}

/// Release every lock the calling task holds, returning how many were released.
///
/// This is the recovery half of [`Mutex::lock_with_backoff`]: a task that cannot make
/// progress releases its holdings so somebody else can, which is what actually breaks a
/// potential cycle (delay alone merely postpones it).
pub fn release_all_held_locks() -> usize {
    let mut ids = [LockId(0); MAX_HELD_LOCKS];
    let mut n = 0usize;
    {
        let g = crate::critical::enter();
        unsafe {
            let me = KERNEL.current();
            if !me.is_null() {
                n = ((*me).held_count as usize).min(MAX_HELD_LOCKS);
                let mut k = 0;
                while k < n {
                    ids[k] = LockId((*me).held_locks[k]);
                    k += 1;
                }
                held_clear();
            }
        }
        drop(g);
    }
    let mut released = 0;
    for id in ids.iter().take(n) {
        if let Some(h) = registered(*id) {
            (h.release)(h);
            released += 1;
        }
    }
    released
}

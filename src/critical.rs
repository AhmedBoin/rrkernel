//! Critical sections: the one primitive every ring mutation depends on.
//!
//! Masking is deliberately *not* uniform across backends, because the right
//! mechanism genuinely differs:
//!
//! | Backend    | `critical_enter`                                        |
//! |------------|---------------------------------------------------------|
//! | Cortex-M   | `cpsid i` + save/restore `PRIMASK`                      |
//! | POSIX      | `sigprocmask` blocking `SIGALRM` (a *pending* SIGALRM is delivered as soon as the section exits, so ticks are never lost) |
//! | Win32      | recursive kernel spinlock (owner thread id + depth), because the tick thread really is a second thread |
//!
//! Critical sections must be **short and non-blocking**: on the host the tick
//! thread waits for the lock, so a task that blocked while holding it would
//! stall the scheduler.

use crate::arch::CriticalToken;

/// RAII guard for a critical section. Restores the previous interrupt state /
/// drops the lock on scope exit (including on early return).
#[must_use = "a critical section is only held for the lifetime of this guard"]
pub struct CriticalGuard {
    token: CriticalToken,
}

impl Drop for CriticalGuard {
    #[inline]
    fn drop(&mut self) {
        // Fence *before* re-enabling interrupts: everything written inside the section must
        // have landed before the ISR can run again.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        // SAFETY: paired with the `enter` below, exactly once.
        unsafe { crate::arch::critical_exit(self.token) }
    }
}

/// Enter a critical section. Nestable on every backend.
#[inline]
pub fn enter() -> CriticalGuard {
    // A compiler barrier *before* the mask, and one *after* it in `Drop`.
    //
    // The per-backend entry assembly carries `nomem`, which is accurate - `cpsid i` really does
    // not touch memory - and that accuracy is exactly what removes the barrier this section
    // exists to provide. Without these fences the optimiser may sink a store past the unmask,
    // so a task can publish its state and enable interrupts in the wrong order; the tick ISR
    // then sees a stale state, misses the wake, and the system wedges the first time a task
    // blocks. That is a release-only, timing-dependent failure, which is what it looked like.
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    // SAFETY: the token is opaque and only consumed by the matching exit.
    let token = unsafe { crate::arch::critical_enter() };
    CriticalGuard { token }
}

/// Run `f` with preemption disabled, returning its result.
#[inline]
pub fn with<R>(f: impl FnOnce() -> R) -> R {
    let _g = enter();
    f()
}

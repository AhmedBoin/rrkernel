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
        // SAFETY: paired with the `enter` below, exactly once.
        unsafe { crate::arch::critical_exit(self.token) }
    }
}

/// Enter a critical section. Nestable on every backend.
#[inline]
pub fn enter() -> CriticalGuard {
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

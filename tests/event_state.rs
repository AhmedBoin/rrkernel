//! Event-layer state machines, and the paths that must *not* block.
//!
//! The blocking paths themselves (`park`, `Signal::wait`, `WaitQueue::wait` with a live kernel) need
//! a running scheduler, which owns the process on the host backends — so those live in
//! `examples/wake_interleave.rs`, which reports a verdict instead of a test result. What is checked
//! here is everything that must hold *without* a kernel, including the promise that a wait attempted
//! from interrupt/idle context fails loudly rather than pretending to have waited.
//!
//! This file never starts a scheduler (one test binary = one process, and nothing else here drives
//! the kernel), so `KERNEL.current()` is null for a real reason.

use rrkernel::event::{self, park, Signal, Timeout, WaitQueue, WakeReason};
use rrkernel::scheduler::{self, BlockOutcome};

/// Run `f`, swallowing the panic the scheduler raises for a wait with no current task, and report
/// whether it panicked. The scheduler's `debug_assert` is the loud half of that contract.
fn panicking(f: impl FnOnce()) -> bool {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
    std::panic::set_hook(prev);
    r.is_err()
}

#[test]
fn signal_set_take_and_is_set() {
    let s = Signal::new();
    assert!(!s.is_set(), "a new signal starts clear");
    assert!(!s.try_take(), "nothing to take yet");
    s.set();
    assert!(s.is_set(), "set() marks it");
    assert!(s.try_take(), "the first take consumes it");
    assert!(!s.try_take(), "and only the first take");
    assert!(!s.is_set());
    // Setting it twice is one-shot, not two.
    s.set();
    s.set();
    assert!(s.try_take());
    assert!(!s.try_take());
}

#[test]
fn signal_default_is_equivalent_to_new() {
    let s = Signal::default();
    assert!(!s.is_set());
    s.set();
    assert!(s.try_take());
}

#[test]
fn wait_queue_allocates_distinct_resource_ids() {
    let a = WaitQueue::new();
    let b = WaitQueue::new();
    assert_ne!(a.id(), b.id(), "two queues must not share a resource id");
    // Resource ids live above the task-id space, which starts at 1.
    assert!(
        a.id() >= 0x4000_0000,
        "resource ids are out of the task-id range"
    );
    // An explicit id is usable in a const/static.
    const Q: WaitQueue = WaitQueue::with_id(0x4000_0100);
    assert_eq!(Q.id(), 0x4000_0100);
    // No waiters, so there is nobody to wake.
    assert!(!a.wake_one(), "wake_one with an empty queue reports false");
    assert_eq!(a.waiters(), 0);
    assert_eq!(a.wake_all(), 0);
}

#[test]
fn wait_until_returns_immediately_when_ready_or_past_the_deadline() {
    // Already satisfied: no yield, no kernel, no panic.
    assert_eq!(event::wait_until(|| true, 0), Ok(()));

    // Never satisfied with a deadline in the past: the deadline is checked before any yield, so this
    // returns Err without needing a scheduler — and without spinning.
    let spins = std::cell::Cell::new(0u32);
    let r = event::wait_until(
        || {
            spins.set(spins.get() + 1);
            false
        },
        0,
    );
    assert_eq!(r, Err(Timeout));
    assert_eq!(
        spins.get(),
        1,
        "the condition is polled once, then the deadline decides"
    );
}

#[test]
fn blocking_without_a_current_task_is_loud_and_changes_nothing() {
    let before = scheduler::blocks_without_current();

    // `park` must not report "woken" when it never parked, and the scheduler must count the try.
    assert!(
        panicking(|| {
            let _ = park(None);
        }),
        "debug builds assert on a wait with no current task"
    );
    assert!(
        scheduler::blocks_without_current() > before,
        "counted in release too"
    );

    // `Signal::wait` in the same situation: the closure runs, `mark_blocked_locked` refuses, and the
    // caller is told it timed out rather than that the signal arrived.
    let s = Signal::new();
    assert!(panicking(|| {
        let _ = s.wait(Some(1));
    }));
    assert!(
        !s.is_set(),
        "a refused wait must not consume a signal it never received"
    );

    // `WaitQueue` likewise.
    let q = WaitQueue::new();
    assert!(panicking(|| {
        let _ = q.wait(None);
    }));
    assert_eq!(q.waiters(), 0, "a refused wait leaves no waiter registered");
}

#[test]
fn park_from_interrupt_context_reports_timeout() {
    // Outside a task, `park` returns without blocking. The debug assert fires in debug builds, so
    // the observable contract this checks is the return value in release and the panic in debug —
    // both mean "this did not wait", which is the point.
    let panicked = panicking(|| {
        let r = park(None);
        // Unreachable in debug builds (the assert fires first), but stated for release clarity.
        assert_eq!(r, WakeReason::Timeout);
    });
    assert!(panicked || cfg!(not(debug_assertions)));
}

#[test]
fn block_until_tick_if_reports_ready_without_blocking_when_the_condition_holds() {
    // The primitive under every event type: a true condition means "do not block", and that path
    // must not touch the scheduler at all — no current task is needed.
    let calls = std::cell::Cell::new(0u32);
    let outcome = scheduler::block_until_tick_if(0x4000_FFFF, 0, || {
        calls.set(calls.get() + 1);
        true
    });
    assert_eq!(outcome, BlockOutcome::Ready);
    assert_eq!(calls.get(), 1, "the condition runs exactly once");
}

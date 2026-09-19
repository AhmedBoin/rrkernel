//! Blocking from the wrong context: the deterministic half of the Phase 1 fix.
//!
//! `scheduler::block_current` used to `return` silently when there was no current task. That is
//! the *cheapest* way for a sleep to "wake up early": it never blocked at all, so the deadline
//! was never waited for. It is a genuinely reachable path — the switch path runs with
//! `current == null` while it is on the idle branch — which is why the fix counts it and asserts
//! on it instead of returning.
//!
//! This file deliberately **never starts a kernel**. Test binaries are one process per file, so
//! nothing else here can be driving the scheduler: `KERNEL.current()` is null for a reason, and
//! the assertion under test is the real one rather than a simulated one.
//!
//! The live-kernel half of Phase 1 (sleeps under spawn/exit churn, all-sleeping windows) is
//! `examples/sleep_fidelity.rs`: a live kernel owns the process (the host backend runs a tick
//! thread, and `scheduler::shutdown` *is* a process exit), which is why the repository reports
//! that kind of check as an example with a verdict instead of a `#[test]`.

use rrkernel::scheduler;

#[test]
fn blocking_with_no_current_task_is_counted_and_loud_in_debug() {
    let before = scheduler::blocks_without_current();

    // Silence the harness's panic output: the panic is the behaviour under test, not a failure.
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let result = std::panic::catch_unwind(|| scheduler::sleep_ticks(1));
    std::panic::set_hook(prev);

    let after = scheduler::blocks_without_current();
    // `> before`, not `== before + 1`: this binary's tests share one process and run in parallel,
    // so both of them increment the same global counter. (Merging them into one test would be
    // tidier; the threading here is the harness's, not the kernel's — no kernel is running.)
    assert!(
        after > before,
        "a wait that cannot happen must still be counted (it is the release-build signal)"
    );
    if cfg!(debug_assertions) {
        assert!(
            result.is_err(),
            "debug builds must assert rather than silently not waiting: this is the bug"
        );
    }
}

#[test]
fn blocking_with_no_current_task_does_not_touch_any_task_state() {
    // A second attempt, with the count observed from both sides, to confirm the failed wait is
    // inert: it must not create a task, wake anything, or move the tick counter.
    let t0 = scheduler::tick_count();
    let a0 = scheduler::active_threads();
    let c0 = scheduler::blocks_without_current();

    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let _ = std::panic::catch_unwind(|| {
        // An unbounded wait is the dangerous one: no deadline, so nothing would ever wake it.
        let _ = scheduler::block_until_tick(0, 0);
    });
    std::panic::set_hook(prev);

    assert_eq!(scheduler::active_threads(), a0);
    assert_eq!(scheduler::tick_count(), t0);
    assert!(
        scheduler::blocks_without_current() > c0,
        "the failed unbounded wait must be counted too"
    );
}

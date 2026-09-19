//! Lock nesting depth: the lock past the limit is **refused**, not silently untracked.
//!
//! Regression test for the second half of the lock-order guarantee. `held_check_order` used to
//! return `Ok(())` once a task already held `MAX_HELD_LOCKS` locks, and `held_push` then dropped
//! the record on the floor — while the lock itself had *already been taken*. Three consequences,
//! all silent:
//!
//! * the extra lock was invisible to [`rrkernel::sync::release_all_held_locks`], so the
//!   back-off recovery path could not release it (it could never break a cycle it caused);
//! * every later ordering check compared against the wrong "last held" lock;
//! * so the deadlock-order rule — the thing that makes circular wait impossible — simply
//!   stopped being enforced past the fourth lock.
//!
//! This file runs **no scheduler**: `held_check_order`/`held_push` only read `KERNEL.current()`,
//! which is installed here by hand, so the test is deterministic. It must be its own binary,
//! because it *does* set a current task and other test files assert that it is null.

use rrkernel::sync::{self, LockError, Mutex, MAX_HELD_LOCKS};
use rrkernel::tcb::{TaskControlBlock, TaskState, KERNEL};
use std::ptr;

/// A TCB that lives for the rest of the process, installed as the current task.
///
/// The fields mirror what the scheduler would set up; only `id` and the held-lock fields
/// matter to the lock layer, and the rest are null/zero exactly as
/// `tests/ring_invariants.rs` builds them.
fn install_fake_current(id: u32) {
    let tcb = Box::new(TaskControlBlock {
        sp: ptr::null_mut(),
        stack_base: ptr::null_mut(),
        stack_size: 0,
        closure_block: ptr::null_mut(),
        state: TaskState::Running,
        flags: 0,
        next: ptr::null_mut(),
        prev: ptr::null_mut(),
        id,
        slice_cycles: 0,
        slices_run: 0,
        switches: 0,
        backend: ptr::null_mut(),
        blocked_on: 0,
        block_deadline: 0,
        held_locks: [0; MAX_HELD_LOCKS],
        held_count: 0,
    });
    let leaked: *mut TaskControlBlock = Box::leak(tcb);
    unsafe { KERNEL.set_current(leaked) };
}

#[test]
fn acquiring_past_the_limit_is_an_error_not_a_silent_grant() {
    install_fake_current(4242);

    // Ids come from a monotonic allocator, and these are created (and taken) in that order, so
    // the ordering rule is satisfied throughout: anything refused below is about *depth*.
    let locks: Vec<Mutex<u32>> = (0..=MAX_HELD_LOCKS).map(|i| Mutex::new(i as u32)).collect();

    let mut guards = Vec::new();
    for (i, l) in locks.iter().take(MAX_HELD_LOCKS).enumerate() {
        guards.push(
            l.try_lock()
                .unwrap_or_else(|e| panic!("lock {i} within the limit must be granted: {e}")),
        );
    }
    assert_eq!(
        sync::held_count(),
        MAX_HELD_LOCKS,
        "every granted lock is recorded, up to the limit"
    );

    // The one past the limit: the lock must NOT be taken, and the error must say why.
    let past = &locks[MAX_HELD_LOCKS];
    match past.try_lock() {
        Err(LockError::TooManyHeldLocks) => {}
        Err(e) => panic!("expected TooManyHeldLocks past the limit, got {e:?}"),
        // Not `{:?}`-able: the guard is not `Debug`, and it should not exist here at all.
        Ok(_) => panic!("granted a lock that cannot be recorded: the lock past the limit is a bug"),
    }
    assert!(
        past.is_free(),
        "a refused acquisition must leave the lock free: granting it untracked is the bug"
    );
    assert_eq!(
        sync::held_count(),
        MAX_HELD_LOCKS,
        "a refusal adds nothing to the held stack"
    );

    // The error is legible, and names the limit rather than the ordering rule.
    let msg = LockError::TooManyHeldLocks.to_string();
    assert!(
        msg.contains(&MAX_HELD_LOCKS.to_string()) && !msg.contains("order"),
        "the message must describe the depth limit, not a lock order: {msg}"
    );

    // Releasing one slot must make the previously refused lock acquirable again — i.e. the
    // refusal left no residue behind.
    drop(guards.pop());
    assert_eq!(sync::held_count(), MAX_HELD_LOCKS - 1);
    let recovered = past
        .try_lock()
        .expect("with a slot free, the same lock must now be granted");
    assert_eq!(sync::held_count(), MAX_HELD_LOCKS, "and it is recorded");

    // Dropping everything empties the held stack, so the bookkeeping is symmetric.
    drop(recovered);
    drop(guards);
    assert_eq!(
        sync::held_count(),
        0,
        "releasing every guard empties the stack"
    );
    assert!(past.is_free());
}

//! The automated doubly-linked circular task ring.
//!
//! Every ring operation here is O(1) (except [`next_runnable`], which is
//! O(number of *consecutive dead* nodes — at most 1 in practice, since the
//! trampoline unlinks in O(1) and dead nodes therefore never accumulate)).
//!
//! All functions in this module must be called with preemption masked
//! ([`crate::critical::enter`]) — the tick path can run at any moment on every
//! backend.

use crate::tcb::{TaskControlBlock, TaskState};
use core::ptr;

/// Insert `node` immediately **after** `current` in the ring.
///
/// If `current` is null, `node` becomes a self-linked ring of one.
///
/// # Safety
/// * `node` must not already be linked anywhere.
/// * Ring mutation requires a critical section.
pub unsafe fn insert_after(current: *mut TaskControlBlock, node: *mut TaskControlBlock) {
    debug_assert!(!node.is_null());
    if current.is_null() {
        (*node).next = node;
        (*node).prev = node;
        return;
    }
    let after = (*current).next;
    debug_assert!(!after.is_null(), "ring invariant: linked node has a successor");
    (*node).prev = current;
    (*node).next = after;
    (*current).next = node;
    (*after).prev = node;
}

/// Unlink `node` from the ring in O(1) — exactly the pointer surgery the task
/// requires on completion:
///
/// ```text
/// (*node.prev).next = node.next
/// (*node.next).prev = node.prev
/// ```
///
/// Returns the node that should become `current` if `node` was current, i.e.
/// its old successor, or null if the ring is now empty.
///
/// # Safety
/// * `node` must currently be linked into exactly one ring.
/// * Ring mutation requires a critical section.
pub unsafe fn unlink(node: *mut TaskControlBlock) -> *mut TaskControlBlock {
    debug_assert!(!node.is_null());
    let next = (*node).next;
    let prev = (*node).prev;

    if next == node {
        // Sole member of the ring.
        (*node).next = ptr::null_mut();
        (*node).prev = ptr::null_mut();
        return ptr::null_mut();
    }

    debug_assert!(!next.is_null() && !prev.is_null());
    (*prev).next = next;
    (*next).prev = prev;

    // Poison the removed node's links: any later attempt to walk through it
    // (or to unlink it twice) trips a debug assertion instead of corrupting
    // the live ring.
    (*node).next = ptr::null_mut();
    (*node).prev = ptr::null_mut();
    next
}

/// True when a linked task must be skipped: finished, or waiting for a resource.
///
/// A blocked task stays linked (so its place in the rotation is preserved and waking it
/// needs no re-insertion), which is why the scheduler has to filter it out here rather
/// than relying on it being absent from the ring.
#[inline]
fn not_runnable(state: TaskState) -> bool {
    matches!(state, TaskState::Dead | TaskState::Blocked)
}

/// First runnable successor of `from`, skipping any node marked
/// [`TaskState::Dead`] or [`TaskState::Blocked`]. Returns null when nothing is runnable
/// (including the case where `from` itself is not runnable and is the only node).
///
/// # Safety
/// `from` must be a valid linked TCB pointer, called with preemption masked.
pub unsafe fn next_runnable(from: *mut TaskControlBlock) -> *mut TaskControlBlock {
    if from.is_null() {
        return ptr::null_mut();
    }
    let mut p = (*from).next;
    while !p.is_null() && not_runnable((*p).state) {
        p = (*p).next;
        if p == from {
            // Went all the way around without finding a runnable node.
            return if not_runnable((*from).state) {
                ptr::null_mut()
            } else {
                from
            };
        }
    }
    if p.is_null() {
        return ptr::null_mut();
    }
    p
}

/// Number of nodes currently linked in the ring.
///
/// # Safety
/// Requires a critical section (walks the whole ring).
pub unsafe fn len(any: *mut TaskControlBlock) -> usize {
    if any.is_null() {
        return 0;
    }
    let mut n = 1usize;
    let mut p = (*any).next;
    while p != any && !p.is_null() {
        n += 1;
        p = (*p).next;
    }
    n
}

/// Walk the ring and verify its invariants. Used by tests and by
/// `debug_assertions` builds after every mutation.
///
/// Returns the number of nodes, or `Err` describing the first broken
/// invariant found.
///
/// # Safety
/// Requires a critical section.
pub unsafe fn check(any: *mut TaskControlBlock) -> Result<usize, &'static str> {
    if any.is_null() {
        return Ok(0);
    }
    if (*any).next.is_null() && (*any).prev.is_null() {
        // Not linked at all (e.g. a task that has finished and been unlinked):
        // an empty ring, not a broken one.
        return Ok(0);
    }
    if (*any).prev.is_null() || (*any).next.is_null() {
        return Err("half-linked node: exactly one of next/prev is null");
    }
    let mut n = 1usize;
    let mut p = (*any).next;
    let mut guard = 0usize;
    while p != any {
        if p.is_null() {
            return Err("ring not circular (hit null)");
        }
        if (*p).prev != (if n == 0 { any } else { prev_of(p, any) }) {
            return Err("ring not doubly linked (prev mismatch)");
        }
        n += 1;
        p = (*p).next;
        guard += 1;
        if guard > 1_000_000 {
            return Err("ring walk did not terminate");
        }
    }
    if (*any).prev != prev_of(any, any) {
        return Err("head prev does not point at tail");
    }
    Ok(n)
}

/// Helper for [`check`]: walk backwards to find `p`'s predecessor. Only used
/// by the invariant checker, so O(n) is fine.
unsafe fn prev_of(p: *mut TaskControlBlock, any: *mut TaskControlBlock) -> *mut TaskControlBlock {
    let mut q = p;
    let mut guard = 0usize;
    loop {
        let prev = (*q).prev;
        if prev.is_null() {
            return ptr::null_mut();
        }
        if (*prev).next == p {
            return prev;
        }
        q = prev;
        guard += 1;
        if guard > 1_000_000 || q == any {
            return (*p).prev;
        }
    }
}

//! Task life cycle: run the body, then tear the task down automatically.
//!
//! Every backend points a new task's initial program counter at
//! [`task_trampoline`], so **the developer never writes scheduling code**: no
//! `yield`, no `exit`, no `scheduler_run()`. Returning from the task body is
//! the exit.

use crate::critical;
use crate::ring;
use crate::scheduler;
use crate::tcb::{TaskControlBlock, TaskState, KERNEL};

/// Backend-agnostic task entry point.
///
/// The port arranges for the task's **first** instruction to be here, with the
/// TCB pointer passed in the platform's first argument register (Cortex-M: R0
/// of the initial exception frame; Win32: the thread parameter; POSIX fibers:
/// a callee-saved register restored by the fiber switch).
///
/// # Safety
/// Only the port may call this, exactly once per spawned task, at the bottom
/// of that task's own stack.
#[no_mangle]
pub unsafe extern "C" fn task_trampoline(tcb: *mut TaskControlBlock) -> ! {
    // Identity check: we are the running task, so the kernel must agree. This
    // catches a wrong frame/parameter wiring at the earliest possible moment
    // instead of corrupting some other task's bookkeeping later.
    debug_assert_eq!(tcb, KERNEL.current(), "trampoline got the wrong TCB");

    // The body. Takes ownership of the closure; its captures are dropped when
    // it returns.
    //
    // NOTE: `closure_block` is deliberately left intact — the deferred-free
    // path needs it to recycle the block after we have switched away.
    let blob = (*tcb).closure_block;
    crate::closure::run(blob);

    exit_task(tcb)
}

/// Automatic task teardown — the kernel half of "just return from your
/// function".
///
/// Steps, in this exact order (all but the switch inside one critical
/// section, so no tick can observe an inconsistent ring):
///
/// 1. `state = Dead`
/// 2. O(1) unlink: `(*prev).next = next`, `(*next).prev = prev`
/// 3. `active_threads -= 1`, and publish the successor as the ring head
/// 4. queue this TCB for deferred-free (its stack cannot be unmapped while we
///    are still standing on it — [`scheduler::drain_pending_free`] does that
///    from kernel context on the next switch)
/// 5. request an immediate context switch **and restart the slice counter**,
///    so the successor gets a full slice instead of the remainder of this one
///
/// # Safety
/// Callable only from the task's own context, after its body has returned.
pub unsafe fn exit_task(tcb: *mut TaskControlBlock) -> ! {
    let me = if tcb.is_null() { KERNEL.current() } else { tcb };
    if me.is_null() {
        // Nothing to run and nothing to unlink: park.
        crate::arch::idle_forever();
    }

    let g = critical::enter();
    (*me).state = TaskState::Dead;

    let succ = ring::unlink(me);
    // Advance the parent's entry point and cursor past the node that just left, so neither
    // is left pointing at an unlinked — and soon to be recycled — node. This is every
    // level, not just the root: a group's cursor can be the child that just died, and a
    // dangling cursor would either stall that subtree or, after reclaim, point into freed
    // arena memory that the next spawn may hand to a different task.
    //
    // Nothing other than the dying node's *own* parent ring may be touched here.
    let parent = (*me).parent;
    if !parent.is_null() {
        if (*parent).children_head == me {
            (*parent).children_head = succ;
        }
        if (*parent).current_child == me {
            (*parent).current_child = succ;
        }
    }
    // `ring_head` describes the level-1 ring, so only a level-1 task may move it. A nested
    // exit leaving it alone is what keeps the root's ring intact; before init (no root
    // yet) the old unconditional behaviour is kept.
    let root = *KERNEL.root.get();
    if root.is_null() || parent == root {
        *KERNEL.ring_head.get() = succ;
    }
    KERNEL.set_current(core::ptr::null_mut());

    let active = KERNEL.active_threads.get();
    *active = (*active).saturating_sub(1);

    scheduler::push_pending_free(me);
    drop(g);

    // Immediate handover: do not wait for the next tick. The successor runs
    // *now* and its slice counter is reset to zero.
    crate::arch::request_switch();
    crate::arch::exit_current_task_forever()
}

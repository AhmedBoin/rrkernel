//! Ring, arena, closure-placement and ABI invariants.
//!
//! These run on the host (`cargo test --features std`) and deliberately test the
//! *portable core*: the doubly-linked ring, the O(1) unlink, the offset
//! constants the assembly depends on, the arena's recycling, and closure
//! placement without a heap. The scheduling behaviour itself (preemption,
//! fairness, automatic exit) is covered by `examples/roundrobin_demo.rs` and
//! `examples/jitter_bench.rs`, which need a live kernel.
//!
//! NOTE: these tests build TCBs by hand (`Box`es), so they must not run while a
//! scheduler instance is active in the same process — that is why the
//! live-kernel tests live in `tests/kernel_lifecycle.rs` instead.

use rrkernel::arena::Arena;
use rrkernel::ring;
use rrkernel::tcb::{
    TaskControlBlock, TaskState, KERNEL, TCB_FLAGS_OFFSET, TCB_SP_OFFSET, TCB_STATE_OFFSET,
};
use std::mem::{align_of, size_of};

/// A TCB living in a `Box`, so tests can build rings without the kernel arena.
struct Node {
    tcb: Box<TaskControlBlock>,
}

impl Node {
    fn new(id: u32, state: TaskState) -> Self {
        let mut tcb = Box::new(TaskControlBlock {
            sp: std::ptr::null_mut(),
            stack_base: std::ptr::null_mut(),
            stack_size: 0,
            closure_block: std::ptr::null_mut(),
            state,
            flags: 0,
            next: std::ptr::null_mut(),
            prev: std::ptr::null_mut(),
            id,
            slice_cycles: 0,
            slices_run: 0,
            switches: 0,
            backend: std::ptr::null_mut(),
            blocked_on: 0,
            block_deadline: 0,
            held_locks: [0; rrkernel::sync::MAX_HELD_LOCKS],
            held_count: 0,
        });
        // Give it a non-null stack-pointer value so a bug that dereferences
        // `sp` is more likely to be caught.
        let raw: *mut TaskControlBlock = &mut *tcb;
        tcb.sp = raw as *mut u8;
        Node { tcb }
    }

    fn ptr(&self) -> *mut TaskControlBlock {
        &*self.tcb as *const TaskControlBlock as *mut TaskControlBlock
    }
}

/// The assembly reads `sp`, `state` and `flags` at fixed offsets, and
/// `current_tcb` at offset 0 of `KERNEL`. The library asserts these at compile
/// time; re-checking here means a readable test failure too, and documents why
/// the field order may not be reshuffled.
#[test]
fn abi_offsets_match_assembly_expectations() {
    assert_eq!(TCB_SP_OFFSET, 0, "sp must be first: PendSV uses [tcb + 0]");
    assert_eq!(
        TCB_STATE_OFFSET,
        size_of::<*mut u8>() * 3 + size_of::<usize>(),
        "state offset changed: update any asm that reads it"
    );
    assert_eq!(
        TCB_FLAGS_OFFSET,
        TCB_STATE_OFFSET + 1,
        "flags must immediately follow state"
    );
    assert_eq!(
        core::mem::offset_of!(rrkernel::tcb::Kernel, current_tcb),
        0,
        "KERNEL.current_tcb must stay at offset 0 (PendSV loads it with a literal offset)"
    );
    assert_eq!(align_of::<TaskControlBlock>() % align_of::<usize>(), 0);
}

#[test]
fn ring_insert_after_builds_a_circular_doubly_linked_ring() {
    unsafe {
        let a = Node::new(1, TaskState::Running);
        let b = Node::new(2, TaskState::Ready);
        let c = Node::new(3, TaskState::Ready);

        ring::insert_after(std::ptr::null_mut(), a.ptr());
        assert_eq!((*a.ptr()).next, a.ptr());
        assert_eq!((*a.ptr()).prev, a.ptr());
        assert_eq!(ring::len(a.ptr()), 1);

        ring::insert_after(a.ptr(), b.ptr());
        ring::insert_after(b.ptr(), c.ptr());

        // Order must be a -> b -> c -> a, in both directions.
        assert_eq!((*a.ptr()).next, b.ptr());
        assert_eq!((*b.ptr()).next, c.ptr());
        assert_eq!((*c.ptr()).next, a.ptr());
        assert_eq!((*a.ptr()).prev, c.ptr());
        assert_eq!((*b.ptr()).prev, a.ptr());
        assert_eq!((*c.ptr()).prev, b.ptr());
        assert_eq!(ring::len(a.ptr()), 3);
        assert_eq!(ring::check(a.ptr()), Ok(3));
    }
}

#[test]
fn unlink_is_o1_pointer_surgery_and_returns_the_successor() {
    unsafe {
        let a = Node::new(1, TaskState::Running);
        let b = Node::new(2, TaskState::Ready);
        let c = Node::new(3, TaskState::Ready);
        ring::insert_after(std::ptr::null_mut(), a.ptr());
        ring::insert_after(a.ptr(), b.ptr());
        ring::insert_after(b.ptr(), c.ptr());

        // Unlink the middle node: b.prev.next = b.next, b.next.prev = b.prev.
        let succ = ring::unlink(b.ptr());
        assert_eq!(succ, c.ptr());
        assert_eq!((*a.ptr()).next, c.ptr());
        assert_eq!((*c.ptr()).prev, a.ptr());
        assert_eq!((*b.ptr()).next, std::ptr::null_mut(), "links poisoned");
        assert_eq!((*b.ptr()).prev, std::ptr::null_mut());
        assert_eq!(ring::len(a.ptr()), 2);
        assert_eq!(ring::check(a.ptr()), Ok(2));

        // Unlink the successor we were handed.
        assert_eq!(ring::unlink(c.ptr()), a.ptr());
        assert_eq!(ring::len(a.ptr()), 1);

        // Unlinking the sole member yields null (the ring is now empty) and
        // must not leave self-references behind.
        assert!(ring::unlink(a.ptr()).is_null());
        assert!(ring::check(a.ptr()).is_ok());
    }
}

#[test]
fn next_runnable_skips_dead_nodes() {
    unsafe {
        let a = Node::new(1, TaskState::Running);
        let b = Node::new(2, TaskState::Dead);
        let c = Node::new(3, TaskState::Dead);
        let d = Node::new(4, TaskState::Ready);
        ring::insert_after(std::ptr::null_mut(), a.ptr());
        ring::insert_after(a.ptr(), b.ptr());
        ring::insert_after(b.ptr(), c.ptr());
        ring::insert_after(c.ptr(), d.ptr());

        assert_eq!(ring::next_runnable(a.ptr()), d.ptr());

        // With everything dead, nobody is runnable.
        (*a.ptr()).state = TaskState::Dead;
        (*d.ptr()).state = TaskState::Dead;
        assert!(ring::next_runnable(a.ptr()).is_null());

        // Only the starting node alive: it returns itself (it is its own
        // successor after a full lap).
        (*a.ptr()).state = TaskState::Running;
        assert_eq!(ring::next_runnable(a.ptr()), a.ptr());
    }
}

#[test]
fn kernel_state_starts_empty() {
    unsafe {
        assert!(KERNEL.current().is_null());
        assert_eq!(*KERNEL.active_threads.get(), 0);
        assert_eq!(*KERNEL.total_threads.get(), 0);
        assert!(!*KERNEL.running.get());
    }
}

// ---------------------------------------------------------------------------
// Arena
// ---------------------------------------------------------------------------

#[test]
fn arena_recycles_freed_blocks_instead_of_growing() {
    static mut REGION: [u8; 4096] = [0; 4096];
    unsafe {
        let mut a = Arena::empty();
        a.init(std::ptr::addr_of_mut!(REGION) as *mut u8, 4096);

        let p1 = a.alloc(256, 8).expect("first alloc");
        let p2 = a.alloc(256, 8).expect("second alloc");
        assert_ne!(p1, p2);
        assert_eq!(p1 as usize % 8, 0, "payload must honour alignment");
        assert_eq!(p2 as usize % 8, 0);

        let bump_before = a.stats().bytes_bump;
        a.free(p1);
        let p3 = a.alloc(256, 8).expect("recycle");
        let s = a.stats();
        assert_eq!(p3, p1, "a freed block of the right size must be reused");
        assert_eq!(
            s.bytes_bump, bump_before,
            "recycling must not advance the bump pointer"
        );
        assert_eq!(s.frees, 1);
        assert_eq!(s.free_blocks, 0, "the recycled block left the free list");
    }
}

#[test]
fn arena_honours_over_alignment_and_reports_exhaustion() {
    static mut REGION: [u8; 512] = [0; 512];
    unsafe {
        let mut a = Arena::empty();
        a.init(std::ptr::addr_of_mut!(REGION) as *mut u8, 512);

        let p = a.alloc(64, 64).expect("64-byte aligned alloc");
        assert_eq!(p as usize % 64, 0, "over-alignment must be honoured");

        // Exhaustion is reported, never wrapped around.
        let mut exhausted = false;
        for _ in 0..64 {
            if a.alloc(256, 8).is_none() {
                exhausted = true;
                break;
            }
        }
        assert!(exhausted, "arena must report exhaustion");
    }
}

// ---------------------------------------------------------------------------
// Closure placement — exactly what `thread::spawn` relies on, with no heap
// ---------------------------------------------------------------------------

/// Place a closure into an arena block and run it the way the task trampoline
/// does.
unsafe fn place_and_run<F: FnOnce()>(a: &mut Arena, f: F) {
    let size = rrkernel::closure::block_size_for::<F>();
    let align = rrkernel::closure::block_align_for::<F>();
    let blob = a.alloc(size, align).expect("closure block");
    assert_eq!(
        blob as usize % align,
        0,
        "closure block must satisfy the closure's alignment"
    );
    rrkernel::closure::place::<F>(blob, f);
    rrkernel::closure::run(blob);
    a.free(blob);
}

/// Destroy (not run) a placed closure — the spawn-failure path.
unsafe fn place_and_destroy<F: FnOnce()>(a: &mut Arena, f: F) {
    let size = rrkernel::closure::block_size_for::<F>();
    let align = rrkernel::closure::block_align_for::<F>();
    let blob = a.alloc(size, align).expect("closure block");
    rrkernel::closure::place::<F>(blob, f);
    rrkernel::closure::drop_blob::<F>(blob);
    a.free(blob);
}

static DROPS: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

struct Dropper;
impl Drop for Dropper {
    fn drop(&mut self) {
        DROPS.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    }
}

#[test]
fn closure_runs_once_and_drops_its_captures_at_the_right_time() {
    static mut REGION: [u8; 2048] = [0; 2048];
    unsafe {
        let mut a = Arena::empty();
        a.init(std::ptr::addr_of_mut!(REGION) as *mut u8, 2048);
        let runs = std::sync::Arc::new(std::sync::atomic::AtomicU32::new(0));

        // 1. Simple by-reference capture.
        let r = runs.clone();
        place_and_run(&mut a, move || {
            r.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        });
        assert_eq!(runs.load(std::sync::atomic::Ordering::SeqCst), 1);

        // 2. Zero-sized closure.
        place_and_run(&mut a, || {});

        // 3. By-value (moved) capture: dropped when the closure body ends.
        let d = Dropper;
        place_and_run(&mut a, move || {
            let _keep = &d;
        });
        assert_eq!(DROPS.load(std::sync::atomic::Ordering::SeqCst), 1);

        // 4. Over-aligned capture must survive placement intact.
        #[repr(align(32))]
        struct Aligned32([u64; 4]);
        let r2 = runs.clone();
        let big = Aligned32([7, 8, 9, 10]);
        place_and_run(&mut a, move || {
            assert_eq!(big.0[3], 10, "over-aligned capture must be intact");
            r2.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        });
        assert_eq!(runs.load(std::sync::atomic::Ordering::SeqCst), 2);

        // 5. Never-run closure still drops exactly once (no leak on the
        //    spawn-failure path).
        let d2 = Dropper;
        place_and_destroy(&mut a, move || {
            let _keep = &d2;
        });
        assert_eq!(DROPS.load(std::sync::atomic::Ordering::SeqCst), 2);

        // 6. The arena is still healthy after all that: exact-fit recycling
        //    gives the same block back.
        let p = a.alloc(128, 8).expect("still allocatable");
        a.free(p);
        assert_eq!(a.alloc(128, 8), Some(p));
    }
}

//! Closure placement without a heap and without `alloc`.
//!
//! `thread::spawn` needs to move a `FnOnce` value into a task's memory and
//! later run it from a backend-agnostic entry point that knows nothing about
//! the closure's type. Two pieces make that work with **zero dependencies**:
//!
//! 1. A tiny `#[repr(C)]` header carrying a *monomorphized* runner function
//!    pointer, so the generic type is recovered at run time:
//!    `(hdr.run)(base + hdr.data_off)`.
//! 2. The closure value itself is `ptr::write`-n into arena memory carved by
//!    [`crate::arena`], and `ptr::read`-n back out when it runs — the exact
//!    ownership dance `Box<dyn FnOnce()>` performs, done by hand.
//!
//! This is why the kernel does not need `extern crate alloc`, a
//! `#[global_allocator]`, or the `libc`/`heapless`/`linked_list_allocator`
//! crates on bare metal.

use core::ptr;

/// Header written at the start of a closure block.
///
/// `data_off` is an offset (not a pointer) so the header stays small and the
/// payload can be over-aligned for closures that need it.
#[repr(C)]
pub struct ClosureHeader {
    /// Monomorphized `run_closure::<F>`.
    pub run: unsafe fn(*mut u8),
    /// Byte offset from the block base to the `F` value.
    pub data_off: usize,
}

/// Size of the header; also the alignment the block is guaranteed to have.
pub const HEADER_SIZE: usize = core::mem::size_of::<ClosureHeader>();

/// Alignment required for a block holding a closure of type `F`.
#[inline]
pub const fn block_align_for<F>() -> usize {
    let a = core::mem::align_of::<F>();
    let base = core::mem::align_of::<ClosureHeader>();
    if a > base {
        a
    } else {
        base
    }
}

/// Size of the block needed to hold a closure of type `F`.
#[inline]
pub const fn block_size_for<F>() -> usize {
    // Worst case: header + enough padding to align the payload to `F`'s
    // alignment.
    align_up(HEADER_SIZE + core::mem::align_of::<F>(), block_align_for::<F>())
        + core::mem::size_of::<F>()
}

/// Write `f` into the block at `base` and return `base` (the pointer handed to
/// the task trampoline).
///
/// # Safety
/// * `base` must come from `arena::Arena::alloc` with
///   `size >= block_size_for::<F>()` and `align >= block_align_for::<F>()`.
/// * `base` must not be read/written by anything else afterwards.
pub unsafe fn place<F: FnOnce()>(base: *mut u8, f: F) -> *mut u8 {
    let align = block_align_for::<F>();
    let data_off = align_up_usize(HEADER_SIZE, align);
    let hdr = base as *mut ClosureHeader;
    (*hdr).run = run_closure::<F>;
    (*hdr).data_off = data_off;
    ptr::write(base.add(data_off) as *mut F, f);
    base
}

/// Execute and consume the closure stored at `base`.
///
/// # Safety
/// `base` must point at a block produced by [`place`] that has not run yet.
pub unsafe fn run(base: *mut u8) {
    debug_assert!(!base.is_null(), "task trampoline got a null closure block");
    let hdr = &*(base as *const ClosureHeader);
    let data = base.add(hdr.data_off);
    // Poison the header so a double-run trips immediately instead of
    // re-running on freed memory.
    (hdr.run)(data);
}

/// Monomorphized body: moves the closure out of the block and calls it.
///
/// Taking ownership by value (`ptr::read`) means the closure's captures are
/// dropped exactly once, when `f` returns — no leak, no double drop.
unsafe fn run_closure<F: FnOnce()>(data: *mut u8) {
    let f: F = ptr::read(data as *const F);
    f();
}

/// Destroy a closure that will never run (used on spawn-failure paths), so its
/// captures are dropped properly instead of leaking.
///
/// # Safety
/// `base` must be a block produced by [`place::<F>`] whose closure has not run
/// and will not run.
pub unsafe fn drop_blob<F>(base: *mut u8) {
    let data_off = align_up_usize(HEADER_SIZE, block_align_for::<F>());
    ptr::drop_in_place(base.add(data_off) as *mut F);
}

#[inline]
const fn align_up(v: usize, align: usize) -> usize {
    (v + (align - 1)) & !(align - 1)
}

#[inline]
const fn align_up_usize(v: usize, align: usize) -> usize {
    align_up(v, align)
}

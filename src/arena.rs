//! Stack / TCB / closure arena with O(1)-ish recycling.
//!
//! # Why not a allocator crate
//! The kernel is meant to be dependency-free on every target, and task
//! lifetimes here follow a very regular pattern: a task's TCB + closure +
//! (bare metal) stack are allocated at `spawn` and all released at once when
//! the task returns. A general-purpose allocator with its own metadata and
//! fragmentation behaviour is the wrong tool for that.
//!
//! # Policy
//! * **Allocation:** a first-fit scan of the free list for a block that is big
//!   enough *and* correctly aligned; otherwise the bump pointer carves fresh
//!   memory low-to-high out of the arena (the arena sits after `.bss`/`.data`,
//!   so task stacks can never collide with static memory).
//! * **Deallocation:** every block carries a fixed header plus a
//!   pointer-sized slot immediately below the payload that records where the
//!   header is; freed blocks are pushed onto a singly-linked free list threaded
//!   *through the free blocks themselves*. Freed stacks are therefore recycled
//!   by the next task that needs them, so a repeated spawn/exit workload is
//!   steady-state rather than a leak.
//! * Exhaustion is a sizing bug, not a runtime condition: allocation returns
//!   `None`, surfaced to the caller as [`crate::SpawnError::ArenaExhausted`].
//!
//! ```text
//!   base+bump ──► [ Block header ][ pad ][ ptr->header ][ payload ... ]
//!                  (HDR_SIZE)             (PTR_SLOT)      ^ returned
//! ```

use core::cell::UnsafeCell;
use core::ptr;

/// Default arena size when [`crate::SchedulerConfig::arena`] is not supplied.
///
/// # Why 16 KiB and not more
/// On bare metal this arena holds every task's TCB *and* its stack, so the size
/// is a real memory-budget decision that must fit whatever part you are on —
/// 16 KiB fits a 64 KiB-RAM Cortex-M0 with room for the linker's stack. It is
/// enough for roughly four 1 KiB-stack tasks, and because finished tasks are
/// recycled it stays enough no matter how many tasks you spawn over time.
///
/// OS-backed backends (Windows/POSIX) only keep TCBs and closure blobs here —
/// the OS owns task stacks — so the default is generous for them.
///
/// Need more? Pass your own region:
/// `SchedulerConfig::with_slice(..).arena(MyRegion::get())`, or use the
/// statically sized alternative below for a bigger compile-time arena.
pub const DEFAULT_ARENA_SIZE: usize = 16 * 1024;

/// Block header. Kept `#[repr(C)]` and small; `total` is the number of bytes
/// reserved from the arena (header + padding + slot + payload).
#[repr(C)]
struct Block {
    total: usize,
    payload_size: usize,
    next: *mut Block,
}

/// Header size rounded up to its own alignment, so a payload aligned to
/// `HDR_ALIGN` always leaves room for the header plus the pointer slot.
const HDR_SIZE: usize = {
    let a = core::mem::align_of::<Block>();
    let s = core::mem::size_of::<Block>();
    (s + a - 1) & !(a - 1)
};

/// Pointer-sized slot stored immediately *below* the payload, holding the
/// block header address. This is what makes `free(payload)` O(1) without
/// knowing the size.
const PTR_SLOT: usize = core::mem::size_of::<*mut Block>();

/// Backing storage for the default arena. `UnsafeCell` because a `static mut`
/// is no longer acceptable, and the serialized-access contract is documented
/// on [`Arena`].
#[repr(C, align(16))]
pub struct ArenaCell(UnsafeCell<[u8; DEFAULT_ARENA_SIZE]>);

// SAFETY: the arena is only ever touched inside `crate::critical` sections
// (or during single-threaded init before the tick source exists).
unsafe impl Sync for ArenaCell {}

impl ArenaCell {
    pub const fn new() -> Self {
        ArenaCell(UnsafeCell::new([0u8; DEFAULT_ARENA_SIZE]))
    }
    #[inline]
    pub fn as_ptr(&self) -> *mut u8 {
        self.0.get() as *mut u8
    }
    /// Length in bytes of the backing storage.
    pub const LEN: usize = DEFAULT_ARENA_SIZE;
}

impl Default for ArenaCell {
    fn default() -> Self {
        Self::new()
    }
}

/// The process-wide default arena.
pub static DEFAULT_ARENA: ArenaCell = ArenaCell::new();

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ArenaStats {
    pub bytes_total: usize,
    pub bytes_bump: usize,
    pub live_bytes: usize,
    pub allocations: u64,
    pub frees: u64,
    pub free_blocks: u32,
    pub peak_live_bytes: usize,
}

/// A bump-forward allocator with first-fit recycling.
///
/// # Safety contract
/// Every method must be called with the kernel critical section held
/// ([`crate::critical::enter`]).
#[derive(Clone, Copy)]
pub struct Arena {
    base: *mut u8,
    len: usize,
    bump: usize,
    free: *mut Block,
    live: usize,
    peak: usize,
    allocations: u64,
    frees: u64,
}

impl Arena {
    pub const fn empty() -> Self {
        Arena {
            base: ptr::null_mut(),
            len: 0,
            bump: 0,
            free: ptr::null_mut(),
            live: 0,
            peak: 0,
            allocations: 0,
            frees: 0,
        }
    }

    /// # Safety
    /// `base`/`len` must describe a writable region that outlives the kernel,
    /// is never written through any other pointer, and is not already in use.
    pub unsafe fn init(&mut self, base: *mut u8, len: usize) {
        self.base = base;
        self.len = len;
        self.bump = 0;
        self.free = ptr::null_mut();
        self.live = 0;
        self.peak = 0;
        self.allocations = 0;
        self.frees = 0;
    }

    #[inline]
    pub fn is_ready(&self) -> bool {
        !self.base.is_null()
    }

    /// Reserve `size` bytes aligned to `align`, returning the payload pointer.
    ///
    /// # Safety
    /// See the type-level contract.
    #[inline]
    pub unsafe fn alloc(&mut self, size: usize, align: usize) -> Option<*mut u8> {
        if !self.is_ready() || size == 0 {
            return None;
        }
        let align = align.max(PTR_SLOT);

        // 1. First-fit recycling from the free list.
        let mut prev: *mut Block = ptr::null_mut();
        let mut cur = self.free;
        while !cur.is_null() {
            let next = (*cur).next;
            let payload = payload_of(cur, align);
            if payload.add(size) <= (cur as *mut u8).add((*cur).total) {
                if prev.is_null() {
                    self.free = next;
                } else {
                    (*prev).next = next;
                }
                (*cur).next = ptr::null_mut();
                *((payload.sub(PTR_SLOT)) as *mut *mut Block) = cur;
                self.account_alloc(size);
                return Some(payload);
            }
            prev = cur;
            cur = next;
        }

        // 2. Fresh memory, carved low-to-high from the bump region.
        let start = self.base.add(self.bump);
        let payload = align_up(start.add(HDR_SIZE + PTR_SLOT), align);
        let end = payload.add(size);
        let used = end.offset_from(self.base) as usize;
        if used > self.len {
            return None; // arena exhausted
        }
        let hdr = payload.sub(PTR_SLOT + HDR_SIZE) as *mut Block;
        (*hdr).total = used - (hdr as usize - self.base as usize);
        (*hdr).payload_size = size;
        (*hdr).next = ptr::null_mut();
        *((payload.sub(PTR_SLOT)) as *mut *mut Block) = hdr;
        self.bump = used;
        self.account_alloc(size);
        Some(payload)
    }

    /// Reserve and zero `size` bytes aligned to `align`.
    ///
    /// # Safety
    /// See the type-level contract.
    pub unsafe fn alloc_zeroed(&mut self, size: usize, align: usize) -> Option<*mut u8> {
        let p = self.alloc(size, align)?;
        ptr::write_bytes(p, 0, size);
        Some(p)
    }

    /// Return a block previously obtained from [`Arena::alloc`].
    ///
    /// # Safety
    /// `payload` must be a live pointer from *this* arena and must be freed
    /// exactly once.
    pub unsafe fn free(&mut self, payload: *mut u8) {
        if payload.is_null() || !self.is_ready() {
            return;
        }
        let hdr = *((payload.sub(PTR_SLOT)) as *mut *mut Block);
        if hdr.is_null() {
            // Already freed (slot poisoned): ignore rather than corrupt the
            // free list, but make the programming error visible in debug.
            debug_assert!(false, "rrkernel: double free of arena block");
            return;
        }
        let size = (*hdr).payload_size;
        (*hdr).next = self.free;
        self.free = hdr;
        *((payload.sub(PTR_SLOT)) as *mut *mut Block) = ptr::null_mut();
        self.live = self.live.saturating_sub(size);
        self.frees += 1;
    }

    #[inline]
    unsafe fn account_alloc(&mut self, size: usize) {
        self.live += size;
        if self.live > self.peak {
            self.peak = self.live;
        }
        self.allocations += 1;
    }

    /// Snapshot for diagnostics / `scheduler::stats()`.
    ///
    /// # Safety
    /// See the type-level contract (walks the free list).
    pub unsafe fn stats(&self) -> ArenaStats {
        let mut n = 0u32;
        let mut p = self.free;
        while !p.is_null() {
            n += 1;
            p = (*p).next;
            if n > 1_000_000 {
                break; // corrupt list: stop rather than hang
            }
        }
        ArenaStats {
            bytes_total: self.len,
            bytes_bump: self.bump,
            live_bytes: self.live,
            allocations: self.allocations,
            frees: self.frees,
            free_blocks: n,
            peak_live_bytes: self.peak,
        }
    }
}

/// Where a block's payload would land, given its alignment requirement.
#[inline]
fn payload_of(hdr: *mut Block, align: usize) -> *mut u8 {
    // SAFETY: only pointer arithmetic; the block header is live and inside the
    // arena, and the caller validates the resulting payload's bounds before use.
    unsafe { align_up((hdr as *mut u8).add(HDR_SIZE + PTR_SLOT), align) }
}

#[inline]
fn align_up(v: *mut u8, align: usize) -> *mut u8 {
    let mask = align - 1;
    ((v as usize + mask) & !mask) as *mut u8
}

/// Compile-time sanity: the header must be usable as a pointer target.
const _: () = assert!(HDR_SIZE.is_multiple_of(core::mem::align_of::<Block>()));
const _: () = assert!(HDR_SIZE >= core::mem::size_of::<Block>());

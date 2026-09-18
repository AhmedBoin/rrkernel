//! Task Control Block and global kernel control state.
//!
//! This module is completely backend-agnostic: the *shape* of the ring
//! (doubly-linked, circular, O(1) unlink) is identical whether the context
//! switch underneath is a Cortex-M `PendSV` trap, a hand-rolled x86-64 fiber
//! switch, or an OS-thread baton on Windows. Only `sp`/`backend` mean
//! different things per backend, and that is documented on the fields.

use core::cell::UnsafeCell;
use core::ptr;

/// Depth of the per-task held-lock stack (see [`TaskControlBlock::held_locks`]).
///
/// Lives here rather than in `sync` so that the TCB (and therefore every target, even
/// those where the lock layer is compiled out for lack of atomics) can size the array.
pub const MAX_HELD_LOCKS: usize = 4;

/// Task lifecycle state.
///
/// `Blocked` is the state a task enters when it is waiting for a resource (a [`Mutex`]
/// or a bounded sleep) rather than for its turn: it stays **linked** in the ring, so its
/// position in the rotation is preserved and waking it is O(1), but the scheduler skips
/// it exactly as it skips a dead task.
///
/// [`Mutex`]: crate::sync::Mutex
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TaskState {
    /// Linked in the ring, waiting for the CPU.
    Ready = 0,
    /// Linked in the ring and currently owning the CPU.
    Running = 1,
    /// Finished. The trampoline unlinks it in O(1) before this state is ever
    /// observable from the tick path (the unlink happens with interrupts /
    /// preemption masked), and deferred-free reclaims its memory.
    Dead = 2,
    /// Linked in the ring but not runnable: waiting for `blocked_on` (a lock id, or 0
    /// for a plain sleep) or for `block_deadline` to pass.
    Blocked = 3,
}

/// `flags` bit: this task's saved context lives on the **main** stack (MSP)
/// instead of the task's own PSP stack. Only ever set for the adopted
/// "main"/supervisor context on Cortex-M; see `arch::cortex_m`.
pub const TCB_FLAG_USE_MSP: u8 = 1 << 0;

/// Task Control Block.
///
/// # ABI warning
/// `#[repr(C)]`, and `sp` **must stay at offset 0**: the Cortex-M `PendSV`
/// handler loads `sp` with a fixed literal offset (see
/// `arch::cortex_m`). The `const _` assertion below turns a layout change into
/// a compile error instead of a silently corrupted ring.
#[repr(C)]
pub struct TaskControlBlock {
    /// Saved stack pointer.
    ///
    /// * Cortex-M: points at the task's saved R4-R11 block (the exception
    ///   frame is implied by the return address in the restored LR/PC).
    /// * POSIX fibers: points at the fiber's saved register frame.
    /// * Windows threads: unused (the OS owns the register file); the thread
    ///   handle lives in `backend` instead.
    pub sp: *mut u8,

    /// Low address of the task's stack (arena payload). Null when the stack
    /// is owned by the OS or when this is the adopted main context.
    pub stack_base: *mut u8,
    /// Reserved stack capacity in bytes.
    pub stack_size: usize,

    /// Heap block holding the task's closure blob (`closure::Blob`), freed by
    /// deferred-free after the task has switched away for good.
    pub closure_block: *mut u8,

    pub state: TaskState,
    pub flags: u8,

    /// Next active task in the circular ring (never null while linked).
    pub next: *mut TaskControlBlock,
    /// Previous active task in the circular ring (never null while linked).
    pub prev: *mut TaskControlBlock,

    /// Monotonic task id (1 = `main`/task 0), handy for logs.
    pub id: u32,
    /// Per-task slice budget in timer cycles. Reserved: today every task uses
    /// `KERNEL.slice_cycles`, but keeping the field here means a future
    /// per-task quantum needs no ABI change.
    pub slice_cycles: u32,
    /// How many slices this task has been given (observability).
    pub slices_run: u32,
    /// How many times this task has been switched in/out (observability).
    pub switches: u32,

    /// Backend-private payload: Win32 thread handle, POSIX fiber context
    /// pointer, ... (null when unused).
    pub backend: *mut u8,

    /// What this task is waiting for while [`TaskState::Blocked`]: a
    /// [`crate::sync::LockId`], or 0 for a plain bounded sleep.
    pub blocked_on: u32,
    /// Slice tick at which a blocked task gives up and becomes runnable again
    /// (0 = wait forever). This is what `try_lock_for` and `sleep_ticks` time out on,
    /// because the slice tick is the only clock the kernel has.
    pub block_deadline: u64,

    /// Ids of the locks this task holds, innermost last. The *order* is what
    /// [`crate::sync`] enforces: acquisitions must be strictly increasing, which makes
    /// a circular wait impossible.
    pub held_locks: [u32; MAX_HELD_LOCKS],
    /// How many entries of `held_locks` are in use.
    pub held_count: u8,
}

/// Byte offset of `sp` inside [`TaskControlBlock`]. Load-bearing for assembly.
pub const TCB_SP_OFFSET: usize = core::mem::offset_of!(TaskControlBlock, sp);
/// Byte offset of `flags` inside [`TaskControlBlock`]. Load-bearing for asm.
pub const TCB_FLAGS_OFFSET: usize = core::mem::offset_of!(TaskControlBlock, flags);
/// Byte offset of `state` inside [`TaskControlBlock`]. Load-bearing for asm.
pub const TCB_STATE_OFFSET: usize = core::mem::offset_of!(TaskControlBlock, state);

const _: () = assert!(
    TCB_SP_OFFSET == 0,
    "TaskControlBlock::sp must stay at offset 0 (Cortex-M PendSV reads it with a fixed offset)"
);

impl TaskControlBlock {
    /// True if this TCB is currently linked into the ring.
    ///
    /// A linked TCB always has non-null `next`/`prev` (a single-element
    /// circular ring points at itself), so this is a cheap identity test.
    #[inline]
    pub fn is_linked(&self) -> bool {
        !self.next.is_null()
    }
}

/// Idle behaviour when no task is runnable (all tasks finished / none spawned
/// yet). Applies to every backend, best-effort.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdlePolicy {
    /// Cortex-M: `wfi` until the next tick. Desktop: sleep until the next
    /// tick. Lowest power, still wakes on every tick.
    Wait,
    /// Busy-spin. Highest determinism (no wake-up latency), burns power.
    Spin,
    /// Terminate the process once `active_threads` reaches zero. This is what
    /// makes a host demo terminate on its own once every task has returned.
    ExitWhenAllDead,
}

/// What the port should measure, if anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Measure {
    /// Record switch latency in timer cycles (Cortex-M `DWT->CYCCNT`).
    Cycles,
    /// Record switch latency in nanoseconds (host high-resolution clock).
    Nanos,
    /// No measurement; the cheapest switch path.
    Off,
}

/// Immutable-after-init configuration snapshot, stored in [`KERNEL`] so the
/// tick path and the measurement paths can read it without a lock.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct KernelConfig {
    pub stack_size: usize,
    pub idle: IdlePolicy,
    pub measure: Measure,
    /// Cycles per slice actually programmed into the hardware timer. This is
    /// the *achieved* value, after rounding/clamping of the user request.
    pub slice_cycles: u32,
    /// Timer frequency in Hz (Cortex-M: core clock; host: timer tick rate).
    pub timer_hz: u32,
    /// How many physical cores are actually running the scheduler (1..=max).
    ///
    /// Cores at or above this index park in a low-power wait loop instead of pulling
    /// tasks, which is how a chip with more cores than you want to use is tamed. The
    /// value is validated against [`crate::smp::max_cores`] at init.
    pub active_cores: usize,
}

impl KernelConfig {
    pub const fn default_const() -> Self {
        KernelConfig {
            stack_size: crate::thread::DEFAULT_STACK_SIZE,
            idle: IdlePolicy::Wait,
            measure: Measure::Off,
            slice_cycles: 0,
            timer_hz: 0,
            active_cores: 1,
        }
    }
}

/// Global kernel control state.
///
/// # Why one `UnsafeCell` per field
/// The Cortex-M `PendSV` handler reads `current_tcb` with a hand-written fixed
/// offset from the `KERNEL` symbol, so `current_tcb` must be at offset 0 of a
/// `#[no_mangle]` static. One `UnsafeCell` per field (rather than a single
/// `UnsafeCell<Inner>`) keeps `offset_of!` usable for exactly that assertion,
/// at zero runtime cost.
///
/// # Safety model
/// Every field is mutated only inside a critical section
/// ([`crate::critical`]) — on bare metal that means interrupts masked, on the
/// host it means the kernel spinlock is held. The type is `Sync` because all
/// access goes through raw pointers obtained from `UnsafeCell::get` and is
/// serialized by that lock.
#[repr(C)]
pub struct Kernel {
    /// Currently running task. Offset 0 — **load-bearing for the PendSV asm**.
    pub current_tcb: UnsafeCell<*mut TaskControlBlock>,
    /// The task to hand the CPU to when `current_tcb` is null (i.e. right
    /// after a task finished and unlinked itself). Kept as an explicit field
    /// so the switch path never has to walk a ring whose "current" node is
    /// already gone — that would be a use-after-free of a recycled TCB.
    pub ring_head: UnsafeCell<*mut TaskControlBlock>,
    /// Total tasks ever created (monotonic).
    pub total_threads: UnsafeCell<usize>,
    /// Tasks currently linked in the ring.
    pub active_threads: UnsafeCell<usize>,
    /// Tick count (slices elapsed) since init.
    pub ticks: UnsafeCell<u64>,
    /// Context switches performed since init.
    pub switches: UnsafeCell<u64>,
    /// Ticks that could not be serviced (host lock contention / lost timer).
    pub ticks_deferred: UnsafeCell<u64>,
    /// Worst observed switch latency (timer cycles or ns, see `Measure`).
    pub worst_latency: UnsafeCell<u32>,
    /// Last observed switch latency.
    pub last_latency: UnsafeCell<u32>,
    /// Worst deviation of the achieved tick period from the requested slice,
    /// in nanoseconds. This is the number that answers "how exact is my time
    /// slice?" — on bare metal it stays within a handful of cycles, on a
    /// desktop OS it is bounded by the host's timer/DPR latency.
    pub worst_period_error_ns: UnsafeCell<u32>,
    /// Most recent tick-period deviation (ns).
    pub last_period_error_ns: UnsafeCell<u32>,
    /// Next task id to hand out.
    pub next_id: UnsafeCell<u32>,
    /// True once `scheduler_init*` has returned (tick source live).
    pub running: UnsafeCell<bool>,
    /// Set by `scheduler::shutdown()`; checked by the tick thread / idle path.
    pub shutdown: UnsafeCell<bool>,
    /// Head of the deferred-free list. Exiting tasks push themselves here
    /// (reusing `prev` as the list link) because a task cannot unmap the stack
    /// it is still standing on; [`crate::scheduler::schedule_next`] drains it in
    /// kernel context, after the switch.
    pub pending_free: UnsafeCell<*mut TaskControlBlock>,
    /// Number of deferred-free reclaims performed.
    pub reclaimed: UnsafeCell<u64>,
    /// Init-time configuration.
    pub config: UnsafeCell<KernelConfig>,
}

// SAFETY: all mutation is funneled through `critical` sections and the raw
// pointers are only dereferenced by such sections or by the port's own
// interrupt handler (which runs with preemption masked on bare metal, or on
// the single tick thread that owns the ring on the host).
unsafe impl Sync for Kernel {}

impl Kernel {
    const fn new() -> Self {
        Kernel {
            current_tcb: UnsafeCell::new(ptr::null_mut()),
            ring_head: UnsafeCell::new(ptr::null_mut()),
            total_threads: UnsafeCell::new(0),
            active_threads: UnsafeCell::new(0),
            ticks: UnsafeCell::new(0),
            switches: UnsafeCell::new(0),
            ticks_deferred: UnsafeCell::new(0),
            worst_latency: UnsafeCell::new(0),
            last_latency: UnsafeCell::new(0),
            worst_period_error_ns: UnsafeCell::new(0),
            last_period_error_ns: UnsafeCell::new(0),
            next_id: UnsafeCell::new(1),
            running: UnsafeCell::new(false),
            shutdown: UnsafeCell::new(false),
            pending_free: UnsafeCell::new(ptr::null_mut()),
            reclaimed: UnsafeCell::new(0),
            config: UnsafeCell::new(KernelConfig::default_const()),
        }
    }

    /// Current task, or null before the first task exists.
    ///
    /// # Safety
    /// Reading is only meaningful with preemption masked (the value can change
    /// at any other moment).
    #[inline]
    pub unsafe fn current(&self) -> *mut TaskControlBlock {
        *self.current_tcb.get()
    }

    /// # Safety
    /// See [`Kernel::current`].
    #[inline]
    pub unsafe fn set_current(&self, tcb: *mut TaskControlBlock) {
        *self.current_tcb.get() = tcb;
    }

    #[inline]
    pub unsafe fn active(&self) -> usize {
        *self.active_threads.get()
    }
    #[inline]
    pub unsafe fn total(&self) -> usize {
        *self.total_threads.get()
    }
    #[inline]
    pub unsafe fn config(&self) -> KernelConfig {
        *self.config.get()
    }
}

/// The one and only kernel control block.
///
/// `#[no_mangle]` so that the Cortex-M assembly can reference it by symbol
/// name (`ldr r3, =KERNEL`).
#[no_mangle]
pub static KERNEL: Kernel = Kernel::new();

/// Compile-time proof that the asm's `KERNEL` offset for `current_tcb` is 0.
const _: () = assert!(core::mem::offset_of!(Kernel, current_tcb) == 0);

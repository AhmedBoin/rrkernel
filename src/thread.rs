//! `thread::spawn` — the entire task-creation surface.
//!
//! There is deliberately no `JoinHandle`, no `Result` from the spec'd entry
//! point, and no way to wait on a task: joining would need a blocking
//! primitive, and this kernel has none. A task runs until its closure returns,
//! then the trampoline unlinks it. (`try_spawn` exists for callers who want
//! allocation failure as a value instead of a panic.)

use crate::config::{ConfigError, Slice};
use crate::tcb::{TaskControlBlock, KERNEL};
use core::fmt;

/// Default per-task stack size when the caller does not specify one.
///
/// 4 KiB is a sane floor for a Cortex-M task and plenty for a host fiber.
/// Override globally with [`crate::SchedulerConfig::stack_size`] or per task
/// with [`spawn_with_stack`].
pub const DEFAULT_STACK_SIZE: usize = 4096;

/// Why a spawn failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpawnError {
    /// `scheduler::init*` has not run yet, or the arena is not set up.
    NotInitialized,
    /// Not enough space left in the kernel arena for the TCB / closure /
    /// stack. Increase the arena (bare metal: `SchedulerConfig::arena`).
    ArenaExhausted,
    /// The OS refused to create the execution context (Win32 thread / POSIX
    /// fiber).
    Backend(ConfigError),
}

impl fmt::Display for SpawnError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            SpawnError::NotInitialized => {
                write!(f, "scheduler not initialized (call scheduler::init first)")
            }
            SpawnError::ArenaExhausted => write!(
                f,
                "kernel arena exhausted: increase the arena or reduce per-task stacks"
            ),
            SpawnError::Backend(e) => write!(f, "backend could not create the task: {e}"),
        }
    }
}

/// Spawn a new task into the live scheduler ring.
///
/// Callable from `main()` (before it returns) or from inside any running task,
/// including recursively from a task that was itself spawned by a task.
///
/// * The new task is linked into the ring immediately, right after the
///   *current* task, so it runs on the very next switch rather than after a
///   full lap.
/// * `KERNEL.total_threads` and `KERNEL.active_threads` are both incremented.
/// * An immediate context switch is requested and the slice counter is
///   restarted, so the newcomer registers instantly with a full slice.
///
/// The closure must be `'static`: a task may outlive the frame that spawned
/// it, so nothing borrowed from the parent's stack is sound here.
///
/// # Panics
/// Panics if the arena is exhausted or the scheduler is not initialised —
/// matching `std::thread::spawn`. Use [`try_spawn`] to handle those as values.
pub fn spawn<F>(f: F)
where
    F: FnOnce() + Send + 'static,
{
    spawn_with_stack(f, crate::scheduler::config().stack_size)
}

/// Like [`spawn`], with an explicit stack size in bytes.
///
/// Bare metal: exactly the reserved PSP stack. POSIX fibers: the fiber stack.
/// Win32: the OS thread's stack reservation.
///
/// # Panics
/// See [`spawn`].
pub fn spawn_with_stack<F>(f: F, stack_size: usize)
where
    F: FnOnce() + Send + 'static,
{
    if let Err(e) = try_spawn_with_stack(f, stack_size) {
        panic!("rrkernel: thread::spawn failed: {}", e);
    }
}

/// [`spawn`] with the failure as a value.
pub fn try_spawn<F>(f: F) -> Result<(), SpawnError>
where
    F: FnOnce() + Send + 'static,
{
    try_spawn_with_stack(f, crate::scheduler::config().stack_size)
}

/// [`spawn_with_stack`] with the failure as a value.
pub fn try_spawn_with_stack<F>(f: F, stack_size: usize) -> Result<(), SpawnError>
where
    F: FnOnce() + Send + 'static,
{
    let stack_size = if stack_size == 0 {
        DEFAULT_STACK_SIZE
    } else {
        stack_size
    };
    // SAFETY: `spawn_internal` takes the kernel critical section itself, so it
    // is safe to call from any task or from `main`. A plain spawn is a level-1 leaf with
    // the configured slice, i.e. a quantum of one tick.
    let root = unsafe { *KERNEL.root.get() };
    unsafe { crate::scheduler::spawn_internal(root, 1, f, stack_size).map(|_| ()) }
}

// ---------------------------------------------------------------------------
// Nested scheduling: the tree-aware API
//
// The simple path above is unchanged on purpose. Everything here is additive, and you
// only need it if you want a task to have its own slice, or a group of tasks to share
// one budget — see `docs/NESTED_SCHEDULING.md`.
// ---------------------------------------------------------------------------

/// Where a spawned node is attached in the scheduling tree.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Parent {
    /// Level 1: a sibling of `main`. What [`spawn`] uses.
    #[default]
    Root,
    /// A child of an existing [`GroupHandle`].
    Group(GroupHandle),
}

/// An opaque handle to a live task. Valid for the life of the program: tasks are never
/// freed while a handle could still be used, because a task leaves the tree only by
/// finishing, and a finished task's handle is simply never a valid `Parent`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskHandle(pub(crate) *mut TaskControlBlock);

/// An opaque handle to a group node — a scheduling node with no stack and no code of its
/// own, which owns a quantum and a ring of children.
///
/// **Groups live until shutdown.** There is deliberately no `close()`: without teardown
/// there is no way to free a TCB that a cursor or a children-head still points at, which
/// is the whole class of dangling-parent bug this design is otherwise exposed to. The
/// cost is one TCB per group (no stack, no closure) held for the life of the program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroupHandle(pub(crate) *mut TaskControlBlock);

// SAFETY: both handles are opaque pointers to kernel-owned nodes. The kernel serializes
// every access to them under its critical section, so moving a handle between tasks (or
// sending it to one) is safe. Using one is not unsafe — the API takes them by value and
// re-validates the node it finds.
unsafe impl Send for TaskHandle {}
unsafe impl Send for GroupHandle {}

/// Resolve a [`Parent`] to a kernel node, or fail if the kernel has no root yet.
fn parent_node(parent: Parent) -> Result<*mut TaskControlBlock, SpawnError> {
    let node = match parent {
        Parent::Root => unsafe { *KERNEL.root.get() },
        Parent::Group(g) => g.0,
    };
    if node.is_null() {
        Err(SpawnError::NotInitialized)
    } else {
        Ok(node)
    }
}

/// Spawn a task as a child of `parent`, running `slice` per turn — the tree-aware entry
/// point. [`spawn`] is exactly this with `Parent::Root` and [`Slice::Default`].
///
/// The quantum is in *ticks*: `Slice::Default` means the configured slice, and any other
/// value must be a whole number of ticks, i.e. an integer multiple of the slice passed to
/// `configure()`. A shorter or non-multiple quantum is refused with a
/// [`crate::config::ConfigError`] rather than rounded, because rounding would silently
/// change the timing you asked for.
///
/// Nothing here constrains the relationship between a group's quantum and the total of
/// its children's quanta: children may finish early (the group just repeats its lap until
/// its own window closes) or run over (the group's window closes mid-child, and the next
/// visit resumes exactly where it left off).
pub fn spawn_in<F>(parent: Parent, slice: Slice, f: F) -> Result<TaskHandle, SpawnError>
where
    F: FnOnce() + Send + 'static,
{
    let ticks = crate::scheduler::quantum_ticks_for(slice)?;
    let stack_size = crate::scheduler::config().stack_size;
    let node =
        unsafe { crate::scheduler::spawn_internal(parent_node(parent)?, ticks, f, stack_size) }?;
    Ok(TaskHandle(node))
}

/// Create a group node under `parent`, with its own quantum, and return a handle that
/// further spawns can attach to (`Parent::Group(handle)`).
///
/// A group has **no stack, no closure and no backend thread or fiber** — it is a
/// scheduling node, not a task, so it costs one TCB and nothing else. It may be created
/// with no children and populated later, from any context, including from inside another
/// task; a child spawned into it later joins the live ring immediately.
///
/// The quantum bounds one *visit* to the group's subtree. It does not constrain the sum
/// of its children's quanta in either direction — see [`spawn_in`].
pub fn spawn_group(parent: Parent, slice: Slice) -> Result<GroupHandle, SpawnError> {
    let ticks = crate::scheduler::quantum_ticks_for(slice)?;
    let g = unsafe { crate::scheduler::spawn_group_node(parent_node(parent)?, ticks) }?;
    Ok(GroupHandle(g))
}

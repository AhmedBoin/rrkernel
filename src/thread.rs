//! `thread::spawn` — the entire task-creation surface.
//!
//! There is deliberately no `JoinHandle`, no `Result` from the spec'd entry
//! point, and no way to wait on a task: joining would need a blocking
//! primitive, and this kernel has none. A task runs until its closure returns,
//! then the trampoline unlinks it. (`try_spawn` exists for callers who want
//! allocation failure as a value instead of a panic.)

use crate::config::ConfigError;
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
    // is safe to call from any task or from `main`.
    unsafe { crate::scheduler::spawn_internal(f, stack_size) }
}

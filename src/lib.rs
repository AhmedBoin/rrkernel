//! # rrkernel — a preemptive round-robin RTOS kernel with a `no_std` core
//!
//! Zero dependencies. One API. Two kinds of machine:
//!
//! * **Bare metal** (`no_std`, Cortex-M): SysTick as the cycle-exact slice
//!   timer, `PendSV` for the context switch, hand-written Thumb assembly for
//!   the register frame, and a user-chosen slice down to ~100 cycles.
//! * **Desktop** (`std`): identical ring, counters and life cycle on top of OS
//!   primitives — a one-shot timer plus real thread suspension on Windows,
//!   `setitimer`/`SIGALRM` plus hand-written x86-64 fiber switching on POSIX.
//!
//! ## The developer API
//!
//! There is no `scheduler_run()`, no explicit yield, and no manual loop/exit
//! handling. Initialising the scheduler starts the timer engine immediately;
//! spawning links a task into the live ring and switches to it; returning from
//! a task's function unlinks it and switches away.
//!
//! ```no_run
//! use rrkernel::{scheduler, thread, Slice, SchedulerConfig};
//!
//! fn main() {
//!     // Choose the slice first: this is the timing contract.
//!     scheduler::init_with(SchedulerConfig::embedded(Slice::Millis(1), 168_000_000)).unwrap();
//!
//!     // Task 1: CPU-bound, preempted automatically every slice.
//!     thread::spawn(|| loop {
//!         core::hint::spin_loop();
//!     });
//!
//!     // Task 2: finishes its work, then the trampoline unlinks it, updates the
//!     // pointers/counters and switches to the next task immediately.
//!     thread::spawn(|| do_work());
//! }
//!
//! fn do_work() {
//!     // Tasks may spawn tasks at any time; the child is inserted into the
//!     // running ring and gets the CPU on the next switch.
//!     thread::spawn(|| { /* child task */ });
//! }
//! ```
//!
//! ## Scheduling contract
//!
//! * **Pure round robin.** Every task has identical priority; the only knob is
//!   the slice, which is set by the user at init (and can be retuned later with
//!   [`scheduler::set_slice`]).
//! * **Fixed, full slices.** Every switch re-arms the timer, so a task that
//!   exits early or a spawn that kicks the scheduler never shortens the next
//!   task's slice and the period never drifts.
//! * **O(1) unlink on completion.** `(*prev).next = next; (*next).prev = prev`,
//!   `active_threads -= 1`, then an immediate context switch. Dead tasks never
//!   accumulate in the ring, and their stacks are recycled by the deferred-free
//!   path on the next switch.
//!
//! ## Honest limits
//!
//! Cycle-exact slices are a **bare-metal property**. On a desktop OS the slice
//! is bounded by the OS timer resolution (0.5–1 ms on Windows) and the jitter
//! by the OS scheduler — [`scheduler::stats`] and the `jitter_bench` example
//! measure it instead of pretending otherwise. On bare metal the switch costs
//! roughly 0.5–1 µs at 168 MHz and the jitter is a handful of cycles. There are
//! no priorities (so no priority inheritance) and no blocking primitives by
//! design, which also means a task's worst-case lateness is `(N-1)` slices.

#![cfg_attr(not(feature = "std"), no_std)]
// Xtensa inline assembly is still an unstable feature in rustc — the esp-rs fork is
// nightly-only for exactly this reason, so the gate is enabled only for that target
// and the host backends keep building on stable.
#![cfg_attr(
    all(not(feature = "std"), target_arch = "xtensa"),
    feature(asm_experimental_arch)
)]
#![allow(clippy::missing_safety_doc)]

pub mod app_support;
pub mod arena;
/// `#[rrkernel]` — the attribute macro that declares a program: `main` becomes task 0, the
/// kernel gets configured by your call, and the fault/panic handlers come with it.
///
/// Import it to use the bare name: `use rrkernel::rrkernel;`
#[cfg(feature = "macros")]
pub use app_support::{configure, log_with};
#[cfg(feature = "macros")]
pub use rrkernel_macros::rrkernel;

/// Sleeping with units: [`sleep`], [`sleep_ms`], [`sleep_secs`], … built on absolute
/// deadlines against the kernel's global tick.
pub mod time;

pub use time::{
    deadline_add, deadline_after, now, sleep, sleep_hours, sleep_minutes, sleep_ms, sleep_ns,
    sleep_secs, sleep_until, sleep_us, tick_ns, ticks_for,
};

/// `core`'s `Duration`, re-exported so an application needs one import for timekeeping:
/// `rrkernel::Duration::from_millis(250)`.
pub use core::time::Duration;
pub mod arch;
pub mod closure;
pub mod config;
pub mod critical;
/// Events and waiting: [`event::park`], [`event::Signal`], [`event::WaitQueue`], and the
/// [`event::wait_until`] poll-yield fallback. Everything an I/O or async layer needs to block a task
/// without burning its slice.
pub mod event;
pub mod ring;
pub mod scheduler;
pub mod smp;
/// Sleeping locks need at least a word-wide atomic read-modify-write: the id allocator
/// and every lock's owner word are atomics. Targets without one (Cortex-M0, AVR,
/// `riscv32imc`) simply do not get this layer — a mutex whose owner field cannot be
/// updated atomically is not a mutex.
#[cfg(target_has_atomic = "32")]
pub mod sync;
pub mod tcb;
pub mod thread;
pub mod trampoline;

pub use config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
pub use event::{park, wait_until, wake, Signal, Timeout, WaitQueue, WakeReason};
pub use scheduler::{
    block_for_ticks, block_until_tick, block_until_tick_if, current_id, deadline_after_ticks,
    main_body, sleep_until_tick, wake_first_blocked_on, wake_task, yield_now, BlockOutcome,
    SchedulerStats, TaskId, TaskInfo,
};
pub use smp::CpuArch;
#[cfg(target_has_atomic = "32")]
pub use sync::{LockError, LockId, Mutex, MutexGuard};
pub use tcb::{IdlePolicy, KernelConfig, Measure, TaskControlBlock, TaskState};
pub use thread::{spawn, spawn_with_stack, try_spawn, SpawnError, DEFAULT_STACK_SIZE};

/// The global kernel control block (ring, counters, tick statistics).
pub use tcb::KERNEL;

/// Start the scheduler with the default 1 ms slice — the convenient form of
/// [`scheduler::init_with`].
///
/// Returns after the tick source is live. The calling context becomes task 0
/// and is linked into the ring.
pub fn scheduler_init() {
    scheduler::init()
}

/// Start the scheduler with an explicit time slice (and other port knobs).
pub fn scheduler_init_with(cfg: SchedulerConfig) -> Result<(), ConfigError> {
    scheduler::init_with(cfg)
}

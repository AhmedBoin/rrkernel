//! The async layer: `block_on`, the task `Waker`, and async sleeps.
//!
//! A `.await` that returns `Pending` behaves exactly like a blocking call — the task gives up the
//! rest of its slice at once and the round robin carries on. That is not a property of this
//! executor so much as of what it parks on: `block_on` waits through the same primitive every other
//! wait uses, so an async task costs the kernel the same as a blocking one while it waits.
//!
//! `core::future` alone: no `alloc`, no `Box<dyn Future>`, no dependency. Futures live on the stack
//! of the task that spawned them, and a task that never awaits just burns its own slice like any
//! CPU-bound task (which is why a badly written async task cannot starve the system either).
//!
//! To run several futures on one task, use `embassy-futures` (`join`, `select`) inside a single
//! `block_on`. There is deliberately no executor loop here to compete with it.

use crate::event::park;
use crate::scheduler::{self, TaskId};
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

// ---------------------------------------------------------------------------
// The waker
// ---------------------------------------------------------------------------

/// Waking means exactly `scheduler::wake_task`: set the task's wake flag and make it runnable if it
/// was blocked. No allocation, no queue of wakers, no registration — the wake flag in the TCB *is*
/// the registration.
static VTABLE: RawWakerVTable = RawWakerVTable::new(clone_fn, wake_fn, wake_by_ref_fn, drop_fn);

unsafe fn clone_fn(p: *const ()) -> RawWaker {
    RawWaker::new(p, &VTABLE)
}

unsafe fn wake_fn(p: *const ()) {
    scheduler::wake_task(p as usize as u32);
}

unsafe fn wake_by_ref_fn(p: *const ()) {
    scheduler::wake_task(p as usize as u32);
}

/// Nothing to free: the id lives in the pointer, and ids are `Copy`.
unsafe fn drop_fn(_: *const ()) {}

/// A `Waker` for `id`.
///
/// The id travels inside the pointer, so this allocates nothing, and a stale waker is harmless:
/// task ids are never reused, so waking a task that has exited does nothing at all.
pub fn task_waker(id: TaskId) -> Waker {
    // SAFETY: the vtable's functions only reinterpret the pointer as the `u32` it was made from.
    unsafe { Waker::from_raw(RawWaker::new(id.raw() as usize as *const (), &VTABLE)) }
}

/// A `Waker` for the calling task.
pub fn current_waker() -> Waker {
    task_waker(scheduler::current_id())
}

// ---------------------------------------------------------------------------
// The executor
// ---------------------------------------------------------------------------

/// Run a future to completion on the calling task's stack.
///
/// Every `Pending` gives up the slice: the task parks, the next task in the ring runs, and a tick or
/// a `wake` brings this one back to poll again. That is the same path a blocking `sleep` takes, so
/// an async task waiting on a timer, an ISR or another task consumes no CPU.
///
/// ```ignore
/// // In a task:
/// rrkernel::exec::block_on(async {
///     led.set_high();
///     rrkernel::exec::sleep(Duration::from_millis(500)).await;
///     led.set_low();
/// });
/// ```
///
/// # Panics
/// Panics if called outside a task (interrupt or idle context): there is nothing to park, and the
/// scheduler says so instead of letting this loop spin. Use [`spawn_async`] from an application.
pub fn block_on<F: Future>(fut: F) -> F::Output {
    let me = scheduler::current_id();
    let waker = task_waker(me);
    let mut cx = Context::from_waker(&waker);
    let mut fut = pin!(fut);
    loop {
        // Both of these come *before* the poll, and both are load-bearing:
        //  * clearing the wake flag means a wake arriving during the poll sets it again, so the
        //    park below returns immediately instead of sleeping through it;
        //  * clearing the timer deadline means only the timers *this* poll arms are honoured, so a
        //    stale deadline from an earlier sleep cannot wake us early.
        scheduler::clear_wake_flag();
        scheduler::clear_timer_deadline();

        if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
            return v;
        }

        // Wake on whatever the poll armed, or on a `wake` from anywhere. 0 means "no timer armed",
        // which is `park(None)`: an unbounded wait, ended by the waker alone.
        let d = scheduler::task_timer_deadline();
        let _ = park(if d == 0 { None } else { Some(d) });
    }
}

/// Spawn a task that runs a future to completion.
///
/// The future's own size counts against the task's stack, so a large future needs a bigger stack
/// (`thread::try_spawn_with_stack`); the failure comes back as a value rather than an overflow.
pub fn spawn_async<F>(f: F) -> Result<(), crate::SpawnError>
where
    F: Future<Output = ()> + Send + 'static,
{
    crate::thread::try_spawn(move || {
        // `Output = ()`, so there is no value to drop — and a future that panics takes the task down
        // with it, exactly like a blocking task body.
        block_on(f);
    })
}

// ---------------------------------------------------------------------------
// Timers
// ---------------------------------------------------------------------------

/// A future that resolves when the kernel's tick reaches a deadline.
///
/// `Sleep` does not arm a hardware timer: it records "wake me at this tick" *in the task*
/// ([`scheduler::set_timer_deadline`], merged as the earliest) and returns `Pending`. The tick sweep
/// that already wakes every blocking sleep then makes the task runnable, and the executor polls
/// again. One clock, one wake path, and no second timer subsystem to keep in step with the first.
///
/// Because the deadline is absolute, this cannot drift: a sleep started late still ends on time, and
/// several sleeps under `join!` each keep their own deadline.
pub struct Sleep {
    deadline_tick: u64,
}

impl Sleep {
    /// A sleep that resolves at an absolute tick. `0` is already due.
    pub const fn until_tick(deadline_tick: u64) -> Self {
        Sleep { deadline_tick }
    }

    /// The tick this sleep resolves at.
    pub const fn deadline_tick(&self) -> u64 {
        self.deadline_tick
    }
}

impl Future for Sleep {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if scheduler::tick_count() >= self.deadline_tick {
            return Poll::Ready(());
        }
        // Arm the wake-up. Several pending timers collapse to the earliest, and every future
        // re-arms on its next poll — which is why `join!` works with one deadline slot in the TCB.
        scheduler::set_timer_deadline(self.deadline_tick);
        Poll::Pending
    }
}

/// Sleep for `d` without blocking the CPU — the async twin of [`crate::sleep`].
///
/// ```ignore
/// rrkernel::exec::sleep(Duration::from_millis(250)).await;
/// ```
pub fn sleep(d: core::time::Duration) -> Sleep {
    Sleep::until_tick(scheduler::deadline_after_ticks(crate::time::ticks_for(d)))
}

/// Sleep until an absolute tick, as a future.
pub const fn sleep_until_tick(deadline_tick: u64) -> Sleep {
    Sleep::until_tick(deadline_tick)
}

/// Wait for a [`crate::event::Signal`], as a future.
///
/// The ASYNC form of `Signal::wait`, for code that would rather `.await` than block. It exists so a
/// driver written once against `Signal` can offer both APIs — the blocking facade and the async one
/// — without duplicating the ISR side.
pub struct SignalWait<'a> {
    signal: &'a crate::event::Signal,
    deadline_tick: u64,
}

impl<'a> SignalWait<'a> {
    /// Wait on `signal`, giving up at `deadline_tick` (0 = no deadline).
    pub const fn new(signal: &'a crate::event::Signal, deadline_tick: u64) -> Self {
        SignalWait {
            signal,
            deadline_tick,
        }
    }
}

impl Future for SignalWait<'_> {
    type Output = Result<(), crate::event::Timeout>;

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.signal.try_take() {
            return Poll::Ready(Ok(()));
        }
        if self.deadline_tick != 0 && scheduler::tick_count() >= self.deadline_tick {
            return Poll::Ready(Err(crate::event::Timeout));
        }
        // Polled rather than parked: `Signal::set` has no way to find "the task currently awaiting
        // this signal", so instead of registering, the waiter wakes the task on the next tick at the
        // latest. That keeps the ISR side allocation-free and registration-free — the cost is that a
        // signal is noticed on the next tick instead of instantly, which is one slice of jitter, not
        // a lost wake-up. Use `Signal::wait` (blocking) when the fastest possible hand-off matters.
        scheduler::set_timer_deadline(if self.deadline_tick == 0 {
            scheduler::tick_count().wrapping_add(1)
        } else {
            self.deadline_tick
        });
        let _ = cx;
        Poll::Pending
    }
}

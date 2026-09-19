//! Sleeping, with units.
//!
//! Every call here turns a [`Duration`](core::time::Duration) into the kernel's only clock
//! — the slice tick — and hands it to the scheduler, which *records the deadline* and
//! switches the caller away until it is reached. Nothing here spins: a sleeping task costs
//! no CPU, and no other task's load can stretch its sleep.
//!
//! The resolution is one slice. Durations are therefore **rounded up**, never down: asking
//! to sleep 1 ns sleeps a tick, because returning early would be a lie about the contract.
//!
//! ```ignore
//! use core::time::Duration;
//!
//! rrkernel::sleep(Duration::from_millis(250));   // explicit
//! rrkernel::sleep_ms(250);                       // shorthand
//! rrkernel::sleep_us(500);
//! rrkernel::sleep_secs(1);
//! rrkernel::sleep_minutes(2);
//! rrkernel::sleep_hours(1);
//!
//! // Periodic work that must not drift: schedule the *next* absolute deadline from the
//! // previous one, not from "now", so a late wake-up is corrected instead of accumulated.
//! let period = Duration::from_millis(100);
//! let mut next = rrkernel::deadline_after(period);
//! loop {
//!     rrkernel::sleep_until(next);
//!     let _ = rrkernel::now();
//!     next = rrkernel::deadline_add(next, period);
//! }
//! ```

use crate::scheduler;
use core::time::Duration;

/// How long one tick lasts, in nanoseconds: the resolution of every sleep in this module.
pub fn tick_ns() -> u64 {
    scheduler::stats().slice_ns
}

/// Ticks needed to cover `d`, rounded **up**. Public because periodic code needs it to
/// decide whether it fell more than a whole period behind.
pub fn ticks_for(d: Duration) -> u64 {
    let slice_ns = tick_ns();
    if slice_ns == 0 {
        return 1;
    }
    let want = d.as_nanos();
    let slice = slice_ns as u128;
    match want.div_ceil(slice) {
        0 => 1,
        n if n > u64::MAX as u128 => u64::MAX,
        n => n as u64,
    }
}

/// The current tick, i.e. the kernel's clock since `configure(...)`.
pub fn now() -> u64 {
    scheduler::tick_count()
}

/// Sleep for `d` (rounded up to a whole tick). Returns once the deadline is reached; the
/// caller is *not* on the CPU in between.
pub fn sleep(d: Duration) {
    scheduler::sleep_ticks(ticks_for(d));
}

/// An absolute deadline `d` from now — the starting point for drift-free periodic work.
pub fn deadline_after(d: Duration) -> u64 {
    now().wrapping_add(ticks_for(d))
}

/// `deadline + d`: the next period from a previous absolute deadline.
pub fn deadline_add(deadline: u64, d: Duration) -> u64 {
    deadline.wrapping_add(ticks_for(d))
}

/// Sleep until an absolute deadline from [`deadline_after`] / [`deadline_add`]. Already
/// being past the deadline is not an error: it returns immediately, and the caller decides
/// whether to skip ahead (see [`ticks_for`]).
pub fn sleep_until(deadline: u64) {
    let n = now();
    if deadline > n {
        scheduler::sleep_ticks(deadline - n);
    }
}

/// Sleep for `ns` nanoseconds (rounded up to a whole tick).
pub fn sleep_ns(ns: u64) {
    sleep(Duration::from_nanos(ns));
}

/// Sleep for `us` microseconds (rounded up to a whole tick).
pub fn sleep_us(us: u64) {
    sleep(Duration::from_micros(us));
}

/// Sleep for `ms` milliseconds (rounded up to a whole tick). The natural shorthand when the
/// slice is 1 ms.
pub fn sleep_ms(ms: u64) {
    sleep(Duration::from_millis(ms));
}

/// Sleep for `secs` seconds.
pub fn sleep_secs(secs: u64) {
    sleep(Duration::from_secs(secs));
}

/// Sleep for `mins` minutes.
pub fn sleep_minutes(mins: u64) {
    sleep(Duration::from_secs(mins * 60));
}

/// Sleep for `hours` hours.
pub fn sleep_hours(hours: u64) {
    sleep(Duration::from_secs(hours * 3600));
}

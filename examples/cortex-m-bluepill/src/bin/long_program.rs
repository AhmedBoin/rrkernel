#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::{configure, thread, Duration, Slice};
use rtt_target::rprintln;

/// The core clock: the one number the kernel cannot work out for itself.
const CORE_HZ: u32 = 8_000_000;

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    // The scheduler: core clock, slice, per-task stack. This is the timing contract.
    configure(CORE_HZ, Slice::Millis(1), 1024);

    rprintln!(
        "rrkernel on Cortex-M @ {} Hz, tick = {} ns",
        CORE_HZ,
        rrkernel::tick_ns()
    );

    // One thread that never yields, so preemption is genuinely under test...
    thread::spawn(|| loop {
        core::hint::spin_loop();
    });

    // ...and three that must still hit their deadlines while it spins, each printing
    // something different on its own period.
    thread::spawn(|| periodic("A", 100, 0));
    thread::spawn(|| periodic("B", 250, 0));
    thread::spawn(|| periodic("C", 500, 4));
}

/// Print `[name] ...` every `period_ms`, `prints` times (`0` = forever).
///
/// The period is scheduled as an **absolute deadline** (`deadline_after` / `deadline_add`)
/// rather than a relative sleep, which is what keeps it exact: a wake-up that lands a tick
/// late is corrected on the next period instead of accumulating, and no other thread's load
/// can stretch it.
fn periodic(name: &'static str, period_ms: u64, prints: u32) {
    let period = Duration::from_millis(period_ms);
    let mut next = rrkernel::deadline_after(period);
    let mut n = 0u32;
    while prints == 0 || n < prints {
        rrkernel::sleep_until(next);
        n += 1;
        let now = rrkernel::now();
        if now >= next {
            rprintln!(
                "[{}] print {} at t={} ms (deadline {}), late by {} ms",
                name,
                n,
                now,
                next,
                now - next
            );
        } else {
            rprintln!(
                "[{}] print {} at t={} ms (deadline {}), EARLY by {} ms",
                name,
                n,
                now,
                next,
                next - now
            );
        }
        // Next absolute deadline. If this thread fell more than a whole period behind — a
        // long stall, or a debugger halt — skip the deadlines already missed rather than
        // bursting output to catch up.
        next = rrkernel::deadline_add(next, period);
        let now = rrkernel::now();
        let period_ticks = rrkernel::ticks_for(period);
        while next <= now && now.wrapping_sub(next) > period_ticks {
            next = rrkernel::deadline_add(next, period);
        }
    }
}

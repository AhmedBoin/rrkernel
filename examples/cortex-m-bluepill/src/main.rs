//! rrkernel on an STM32F103C8 "Blue Pill" — the entire application.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run
//! ```
//!
//! Everything else is handled for you: the entry point, the kernel's configuration, the
//! vector-table wiring, the fault and panic handlers, and — because of `log = rtt` — the
//! RTT terminal, its guard and the log sink.

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

    // `main` IS task 0 — no wrapper, no `main_body`, no shutdown. `sleep` takes a
    // duration in whatever unit reads best; the kernel records the deadline against its
    // global tick and switches this task away until it is reached.
    let start = rrkernel::now();
    rrkernel::sleep(Duration::from_millis(2500));
    let slept = rrkernel::now().wrapping_sub(start);

    let st = rrkernel::scheduler::stats();
    let mut ring_len = 0u32;
    let mut dead = 0u32;
    rrkernel::scheduler::for_each_task(|t| {
        ring_len += 1;
        if t.state == rrkernel::TaskState::Dead {
            dead += 1;
        }
    });

    rprintln!("--- report ---");
    rprintln!("slept    : {} ticks (asked 2500 ms)", slept);
    rprintln!(
        "threads  : total {} active {} reclaimed {}",
        st.total_threads,
        st.active_threads,
        st.reclaimed
    );
    rprintln!("ticks    : {} switches {}", st.ticks, st.switches);
    rprintln!(
        "accuracy : worst switch {} cycles (DWT), worst period error {} ns",
        st.worst_latency,
        st.worst_period_error_ns
    );
    rprintln!("ring     : {} nodes, {} dead", ring_len, dead);

    let ok = st.ticks > 0
        && st.switches > 0
        && ring_len as usize == st.active_threads
        && dead == 0
        && slept >= 2500;
    rprintln!("VERDICT  : {}", if ok { "PASS" } else { "FAIL" });
    // Returning here ends task 0: the macro's `exit_main()` unlinks it, and the periodic
    // threads above keep running.
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

//! Nothing but the tick path: no tasks, no groups, no sleeps.
//!
//! ```text
//! cargo run --release --bin tick_test
//! ```
//!
//! Main spins and prints whenever `ticks` changes, so the last line printed says exactly which
//! tick the CPU stopped on. There is nothing else in this firmware to blame: with a single task
//! the scheduler has nothing to switch to, so a freeze here cannot be a group, a tree walk, or a
//! wake -- it can only be the tick path itself.

#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::{configure, scheduler, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!(
        "tick_test: tick {} ns, one task, no groups",
        rrkernel::tick_ns()
    );

    let mut last: u64 = u64::MAX;
    let mut n: u32 = 0;
    loop {
        let st = scheduler::stats();
        let t = st.ticks;
        if t != last {
            // The first few ticks individually, then one line per hundred, so an early freeze
            // is visible as a number rather than as a gap.
            if t % 100 == 0 || t == 1 || t == 2 || t == 3 || t == 4 || t == 5 {
                rprintln!(
                    "ticks={} switches={} t={} ms active={}",
                    t,
                    st.switches,
                    rrkernel::now(),
                    st.active_threads
                );
            }
            last = t;
        }
        n = n.wrapping_add(1);
        if n == 0 {
            break;
        }
        core::hint::spin_loop();
    }
}

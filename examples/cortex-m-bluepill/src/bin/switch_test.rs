//! One extra task, no group, no sleep: does the switch path work on metal?
//!
//! `tick_test` showed the tick path is healthy with a single task. This adds exactly one flat
//! task that only spins, so the only new machinery is the switch itself -- PendSV, the trampoline
//! that starts a spawned task, and the counter it increments. Main keeps spinning and printing,
//! so a freeze shows as the last line rather than as silence.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::{configure, scheduler, thread, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

static SPIN: AtomicU32 = AtomicU32::new(0);

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!("switch_test: spawning one flat spinning task");

    thread::spawn(|| loop {
        SPIN.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    });

    let mut n: u32 = 0;
    loop {
        let st = scheduler::stats();
        n = n.wrapping_add(1);
        if n % 300_000 == 0 {
            rprintln!(
                "t={} ticks={} switches={} active={} spin={}",
                rrkernel::now(),
                st.ticks,
                st.switches,
                st.active_threads,
                SPIN.load(Ordering::Relaxed)
            );
        }
        if n == 0 {
            break;
        }
        core::hint::spin_loop();
    }
}

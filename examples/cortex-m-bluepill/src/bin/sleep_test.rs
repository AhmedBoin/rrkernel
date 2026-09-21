//! One task that sleeps, one that spins, main spins and prints. No groups, no nesting.
//!
//! Every firmware that hung had a sleeper in it; every firmware that worked had none. This is
//! the smallest one that sleeps, and it prints the tick count alongside the wake count so a
//! failed wake shows up as ticks rising while wakes stay at zero.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::{configure, scheduler, thread, Duration, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

static SPIN: AtomicU32 = AtomicU32::new(0);
static WAKES: AtomicU32 = AtomicU32::new(0);

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!("sleep_test: one spinning task, one sleeping task");

    thread::spawn(|| loop {
        SPIN.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    });

    thread::spawn(|| loop {
        rrkernel::sleep(Duration::from_millis(10));
        WAKES.fetch_add(1, Ordering::Relaxed);
    });

    let mut n: u32 = 0;
    let mut last = u64::MAX;
    loop {
        let st = scheduler::stats();
        if st.ticks != last {
            if st.ticks % 200 == 0 || st.ticks == 1 || st.ticks == 5 {
                rprintln!(
                    "ticks={} switches={} active={} spin={} wakes={}",
                    st.ticks,
                    st.switches,
                    st.active_threads,
                    SPIN.load(Ordering::Relaxed),
                    WAKES.load(Ordering::Relaxed)
                );
            }
            last = st.ticks;
        }
        n = n.wrapping_add(1);
        if n == 0 {
            break;
        }
        core::hint::spin_loop();
    }
}

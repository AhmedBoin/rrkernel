//! The smallest nested tree, with no sleeps anywhere: one group with one child.
//!
//! `sleep_test` and the flat example work after the blocked-task fix, so this checks the group
//! path itself. Every task spins and main prints, so nothing here depends on the wake path, and a
//! failure means dispatch or the tree walk -- while success means any remaining problem is sleep
//! inside a group.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

static FLAT: AtomicU32 = AtomicU32::new(0);
static GS: AtomicU32 = AtomicU32::new(0);

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    rprintln!("nested_min: before configure");
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!(
        "nested_min: configure returned, tick {} ns",
        rrkernel::tick_ns()
    );

    thread::spawn(|| loop {
        FLAT.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    });
    rprintln!(
        "nested_min: flat task spawned, active {}",
        scheduler::active_threads()
    );

    let g = thread::spawn_group(Parent::Root, Slice::Millis(2)).expect("group");
    rprintln!(
        "nested_min: group spawned, active {}",
        scheduler::active_threads()
    );

    thread::spawn_in(Parent::Group(g), Slice::Millis(1), || loop {
        GS.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    })
    .expect("child");
    rprintln!(
        "nested_min: child spawned, active {}",
        scheduler::active_threads()
    );

    let mut n: u32 = 0;
    let mut last = u64::MAX;
    loop {
        let st = scheduler::stats();
        if st.ticks != last {
            if st.ticks % 200 == 0 || st.ticks == 1 || st.ticks == 3 {
                rprintln!(
                    "ticks={} switches={} active={} flat={} grp={}",
                    st.ticks,
                    st.switches,
                    st.active_threads,
                    FLAT.load(Ordering::Relaxed),
                    GS.load(Ordering::Relaxed)
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

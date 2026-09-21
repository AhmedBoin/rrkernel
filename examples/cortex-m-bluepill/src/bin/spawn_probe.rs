//! Does `thread::spawn` leave interrupts masked, or turn the tick timer off?
//!
//! `tick_test` shows ticks run perfectly with no spawned tasks; `switch_test` shows they stop as
//! soon as one plain flat task is spawned. This prints the two things that would explain that --
//! PRIMASK, and SysTick CTRL/LOAD -- on both sides of the spawn, and then ticks again afterwards.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::{configure, scheduler, thread, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;
const SYST_CTRL: *const u32 = 0xE000_E010 as *const u32;
const SYST_LOAD: *const u32 = 0xE000_E014 as *const u32;
const SYST_VAL: *const u32 = 0xE000_E018 as *const u32;

static SPIN: AtomicU32 = AtomicU32::new(0);

fn primask() -> u32 {
    let v: u32;
    unsafe { core::arch::asm!("mrs {}, primask", out(reg) v, options(nomem, nostack)) };
    v
}

fn systick() -> (u32, u32, u32) {
    unsafe {
        (
            core::ptr::read_volatile(SYST_CTRL),
            core::ptr::read_volatile(SYST_LOAD),
            core::ptr::read_volatile(SYST_VAL),
        )
    }
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Millis(1), 1024);
    let (c, l, v) = systick();
    rprintln!(
        "after configure : primask={} systick ctrl={:#x} load={:#x} val={:#x}",
        primask(),
        c,
        l,
        v
    );
    let st = scheduler::stats();
    rprintln!(
        "                  ticks={} switches={} active={}",
        st.ticks,
        st.switches,
        st.active_threads
    );

    thread::spawn(|| loop {
        SPIN.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    });

    let (c, l, v) = systick();
    rprintln!(
        "after one spawn : primask={} systick ctrl={:#x} load={:#x} val={:#x}",
        primask(),
        c,
        l,
        v
    );

    let mut n: u32 = 0;
    let mut last = u64::MAX;
    loop {
        let st = scheduler::stats();
        if st.ticks != last {
            if st.ticks % 200 == 0 || st.ticks == 1 || st.ticks == 2 || st.ticks == 3 {
                rprintln!(
                    "ticks={} switches={} active={} spin={} primask={}",
                    st.ticks,
                    st.switches,
                    st.active_threads,
                    SPIN.load(Ordering::Relaxed),
                    primask()
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

//! How far does startup actually get? Four values at one fixed RAM address, no output channel.
//!
//! 0x20004700 is just above .bss and well below the stack top, and every write is volatile, so the
//! value survives optimisation. Read it with probe-rs read b32 0x20004700 1 (the core must be
//! halted, which reading does):
//!
//! * 0x11 - the reset path ran, but the entry function was never reached
//! * 0x22 - the entry function was reached, but the RTT init did not complete
//! * 0x44 - RTT init returned; the firmware is now spinning here

#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::{configure, Slice};

const CORE_HZ: u32 = 8_000_000;
use rtt_target::rprintln;

const BREADCRUMB: *mut u32 = 0x2000_4700 as *mut u32;

fn crumb(v: u32) {
    unsafe { core::ptr::write_volatile(BREADCRUMB, v) }
}

/// Called by the reset handler before .data is copied and .bss is zeroed.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    core::ptr::write_volatile(BREADCRUMB, 0x11);
}


#[rrkernel]
#[cortex_m_rt::entry]
fn main() -> ! {
    crumb(0x22);
    // The critical-section implementation has to be linked, or rtt-target fails at link time.
    let _ = cortex_m::interrupt::free(|_cs| ());
    rtt_target::rtt_init_print!();
    crumb(0x33);
    configure(CORE_HZ, Slice::Millis(1), 1024);
    crumb(0x55);
    rprintln!("life3: kernel configured");
    rprintln!("life: startup and RTT both work");
    loop {
        crumb(0x44);
    }
}

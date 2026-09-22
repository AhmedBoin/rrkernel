//! One task, and it sleeps. Nothing else is runnable, so every tick has to come from the idle path.
//!
//! No breadcrumbs and no fixed-address writes: the earlier version scribbled on the last eight
//! bytes of .bss, which is where the RTT control block ended up in this image.

#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::{configure, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

#[rrkernel]
#[cortex_m_rt::entry]
fn main() -> ! {
    // The critical-section implementation has to be linked, or rtt-target fails at link time.
    let _ = cortex_m::interrupt::free(|_cs| ());
    rtt_target::rtt_init_print!();
    rprintln!("life6: RTT is up, one task only, and it will sleep");
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!("life6: kernel configured");
    loop {
        rrkernel::sleep_ms(50);
        rprintln!("life6: main woke at {} ticks", rrkernel::now());
    }
}
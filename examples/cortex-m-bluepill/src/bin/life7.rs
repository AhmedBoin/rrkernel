//! A sleeper that wakes and then prints, while nothing else is runnable in between.
//!
//! This is the decisive shape: the tick has to come from the idle path, and whatever prints must
//! print after the CPU has been idle. If the output appears at all, the idle path works and an
//! earlier silence was the terminal losing sight of the target rather than the kernel failing.
//! No fixed-address breadcrumbs: 0x20004700 lands inside rrkernel::scheduler::ARENA in this image.

#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::{configure, thread, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

#[rrkernel]
#[cortex_m_rt::entry]
fn main() -> ! {
    let _ = cortex_m::interrupt::free(|_cs| ());
    rtt_target::rtt_init_print!();
    rprintln!("life7: RTT is up");
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!("life7: kernel configured");
    thread::spawn(|| {
        // Nothing else will be runnable for these 200 ticks: the idle path has to deliver them.
        rrkernel::sleep_ms(200);
        rprintln!("life7: the sleeper woke at {} ticks", rrkernel::now());
        loop {
            rprintln!("life7: awake at {} ticks", rrkernel::now());
            cortex_m::asm::delay(2_000_000);
        }
    });
    rprintln!("life7: child spawned, main now sleeping");
    loop {
        rrkernel::sleep_ms(1000);
    }
}
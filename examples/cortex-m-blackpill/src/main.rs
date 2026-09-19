//! rrkernel on an **STM32F401 "Black Pill"** — the LED driven through `stm32f4xx-hal` in
//! one thread, RTT debug output from another.
//!
//! ```text
//! cd examples/cortex-m-blackpill
//! cargo run
//! ```
//!
//! Same shape as the Blue Pill example. The clock is read *back from the HAL* and handed to
//! the kernel, so `configure` gets the frequency that is actually running.

#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::{configure, thread, Slice};
use rtt_target::rprintln;
use stm32f4xx_hal::{pac, prelude::*, rcc::Config};

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    // --- the HAL: clock and the board's LED ---------------------------------------
    let dp = pac::Peripherals::take().unwrap();
    // HSI, 16 MHz: no dependency on the board's crystal, so this cannot hang waiting for an
    // oscillator. For the full 100 MHz: `Config::hse(25.MHz()).sysclk(100.MHz())`.
    let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(16.MHz()));
    let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led = gpioc.pc13.into_push_pull_output(); // PC13 = the on-board LED, active low
    let sysclk = rcc.clocks.sysclk().to_Hz();

    // --- the kernel: the clock the hardware is actually running at ----------------
    configure(sysclk, Slice::Millis(1), 2048);
    rprintln!(
        "rrkernel on STM32F401 Black Pill @ {} Hz, tick = {} ns",
        sysclk,
        rrkernel::tick_ns()
    );

    // --- thread 1: the LED, one second on and one second off ----------------------
    thread::spawn(move || loop {
        led.set_low(); // active low: pulling it low lights the LED
        rrkernel::sleep_secs(1);
        led.set_high();
        rrkernel::sleep_secs(1);
    });

    // --- thread 2: the debug channel, once a second -------------------------------
    thread::spawn(|| {
        let start = rrkernel::now();
        loop {
            let st = rrkernel::scheduler::stats();
            rprintln!(
                "[dbg ] t={} ms | ticks {} | switches {} | active {} | worst switch {} cycles",
                rrkernel::now().wrapping_sub(start),
                st.ticks,
                st.switches,
                st.active_threads,
                st.worst_latency
            );
            rrkernel::sleep_secs(1);
        }
    });

    // --- thread 3: something fast, on its own period ------------------------------
    thread::spawn(|| {
        let mut n = 0u32;
        loop {
            n += 1;
            rprintln!("[fast] {} at t={} ms", n, rrkernel::now());
            rrkernel::sleep_ms(200);
        }
    });

    // --- task 0: observe for five seconds, report, then let `main` return ---------
    // Returning ends task 0; the three threads above keep running, so the LED keeps
    // blinking after the report.
    let start = rrkernel::now();
    rrkernel::sleep_secs(5);
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
    rprintln!(
        "clock    : {} Hz, tick {} ns, slice {} ticks",
        sysclk,
        rrkernel::tick_ns(),
        st.slice_cycles
    );
    rprintln!(
        "threads  : total {} active {} reclaimed {}",
        st.total_threads,
        st.active_threads,
        st.reclaimed
    );
    rprintln!(
        "ticks    : {} switches {} (slept {} ms)",
        st.ticks,
        st.switches,
        slept
    );
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
        && slept >= 5000;
    rprintln!("VERDICT  : {}", if ok { "PASS" } else { "FAIL" });
}

//! Phase: a window *narrower* than the lap. A 3ms group over children of 1.5ms, 2ms and 4ms.
//!
//! ```text
//! cd examples/cortex-m-blackpill
//! cargo run --release --bin rounds_cut
//! ```
//!
//! At a 500us tick that is a 6-tick window over children of 3, 4 and 8 ticks, so the window is
//! narrower than the 15-tick lap: every window cuts a turn short and the next window resumes it
//! with exactly the time it had left. The runs printed below are that contract, on the board.
//!
//! Every measured task spins -- a task that sleeps is not measuring its own turn -- and logs one
//! entry for each tick it sees change while it holds the CPU. Task 0 sleeps through the
//! measurement, then reads the log back and checks it.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Slice};
use rtt_target::rprintln;
use stm32f4xx_hal::{pac, prelude::*, rcc::Config};


const LOG_MAX: usize = 128;
const MEASURE_TICKS: u64 = 48;
const READ: usize = 36;

const B1: u32 = 201; // 1.5ms = 3 ticks
const B2: u32 = 202; // 2ms   = 4 ticks
const B3: u32 = 203; // 4ms   = 8 ticks

static LOG_ID: [AtomicU32; LOG_MAX] = [const { AtomicU32::new(0) }; LOG_MAX];
static LOG_READY: [AtomicU32; LOG_MAX] = [const { AtomicU32::new(0) }; LOG_MAX];
static LOG_N: AtomicU32 = AtomicU32::new(0);

fn log(id: u32) {
    let i = LOG_N.fetch_add(1, Ordering::Relaxed) as usize;
    if i < LOG_MAX {
        LOG_ID[i].store(id, Ordering::Relaxed);
        LOG_READY[i].store(1, Ordering::Release);
    }
}

fn measured(id: u32) {
    let mut last = u64::MAX;
    loop {
        let t = rrkernel::now();
        if t != last {
            log(id);
            last = t;
        }
        core::hint::spin_loop();
    }
}

fn read_log(into: &mut [u32; READ]) -> usize {
    let total = LOG_N.load(Ordering::Acquire) as usize;
    let mut k = 0usize;
    let mut i = 0usize;
    while i < total && k < READ {
        if LOG_READY[i].load(Ordering::Acquire) == 1 {
            into[k] = LOG_ID[i].load(Ordering::Relaxed);
            k += 1;
        }
        i += 1;
    }
    k
}

fn quantum_of(id: u32) -> u32 {
    match id {
        B1 => 3,
        B2 => 4,
        B3 => 8,
        _ => 1,
    }
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    // The clock comes back from the HAL, so `configure` gets the frequency actually
    // running rather than one this file asserts.
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.freeze(Config::hsi().sysclk(16.MHz()));
    let sysclk = rcc.clocks.sysclk().to_Hz();
    configure(sysclk, Slice::Micros(500), 2048);
    rprintln!("rounds-201/202/203: {} Hz, tick {} ns", sysclk, rrkernel::tick_ns());

    let g = thread::spawn_group(Parent::Root, Slice::Millis(3)).expect("group");
    thread::spawn_in(Parent::Group(g), Slice::Micros(1500), || measured(B1)).expect("b1");
    thread::spawn_in(Parent::Group(g), Slice::Millis(2), || measured(B2)).expect("b2");
    thread::spawn_in(Parent::Group(g), Slice::Millis(4), || measured(B3)).expect("b3");

    let t0 = rrkernel::now();
    rrkernel::sleep_until(scheduler::deadline_after_ticks(MEASURE_TICKS));
    let t1 = rrkernel::now();

    let st = scheduler::stats();
    let mut seq = [0u32; READ];
    let n = read_log(&mut seq);
    rprintln!("--- narrow window: 3ms group over 1.5ms, 2ms and 4ms children ---");
    rprintln!("measured ticks {} to {} (asked {}), {} log entries", t0, t1, MEASURE_TICKS, LOG_N.load(Ordering::Acquire));

    let mut spent = [0u32; 3];
    let mut worst = 0u32;
    let mut i = 0usize;
    while i < n {
        let id = seq[i];
        let mut len = 1usize;
        while i + len < n && seq[i + len] == id {
            len += 1;
        }
        let q = quantum_of(id);
        if len as u32 > q {
            worst = (len as u32) - q;
        }
        if id == B1 {
            spent[0] += len as u32;
        } else if id == B2 {
            spent[1] += len as u32;
        } else if id == B3 {
            spent[2] += len as u32;
        }
        rprintln!("  {} x {} tick(s), quantum {}", id, len, q);
        i += len;
    }
    rprintln!("spent: {} {} ticks, {} {} ticks, {} {} ticks", B1, spent[0], B2, spent[1], B3, spent[2]);
    rprintln!("kernel: ticks {} switches {}", st.ticks, st.switches);

    let ok = spent[0] != 0 && spent[1] != 0 && spent[2] != 0 && worst == 0;
    if worst != 0 {
        rprintln!("FAIL: a turn ran {} tick(s) past its quantum", worst);
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
}

//! One nested round robin measured on the board, tick by tick, and checked.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin rounds
//! ```
//!
//! //! Phase: a window *wider* than the lap. A 2ms group over children of 1ms and 0.5ms, i.e.
//! 4 ticks over 2 and 1. Since the lap (3 ticks) is shorter than the window, nothing is cut
//! short and the children rotate 2:1 -- [101, 101, 102] repeating.
//!
//! The tick is 500us (`Slice::Micros(500)`), which is what makes half-millisecond quanta
//! expressible at all: a quantum must be a whole number of ticks.
//!
//! Every measured task spins -- a task that sleeps is not measuring its own turn -- and logs
//! one entry for each tick it sees change while it holds the CPU. Task 0 sleeps through the
//! measurement, then reads the log back, prints the runs it implies and checks them.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;
const LOG_MAX: usize = 128;
const MEASURE_TICKS: u64 = 48;
const READ: usize = 36;

static LOG_ID: [AtomicU32; LOG_MAX] = [const { AtomicU32::new(0) }; LOG_MAX];
static LOG_READY: [AtomicU32; LOG_MAX] = [const { AtomicU32::new(0) }; LOG_MAX];
static LOG_N: AtomicU32 = AtomicU32::new(0);

/// Claim a slot, then publish an entry. The ready flag is what makes a reserve-then-write slot
/// safe for a reader on another task.
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

const A1: u32 = 101; // 1ms = 2 ticks
const A2: u32 = 102; // 0.5ms = 1 tick

fn quantum_of(id: u32) -> u32 {
    if id == A1 {
        2
    } else {
        1
    }
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Micros(500), 1024);
    rprintln!("rounds-101/102: tick {} ns", rrkernel::tick_ns());

    // A 2ms window (4 ticks) over children of 1ms (2 ticks) and 0.5ms (1 tick). The lap is 3
    // ticks, so the window is wider than the lap: no turn is ever cut short.
    let g = thread::spawn_group(Parent::Root, Slice::Millis(2)).expect("group");
    thread::spawn_in(Parent::Group(g), Slice::Millis(1), || measured(A1)).expect("a1");
    thread::spawn_in(Parent::Group(g), Slice::Micros(500), || measured(A2)).expect("a2");

    let t0 = rrkernel::now();
    rrkernel::sleep_until(scheduler::deadline_after_ticks(MEASURE_TICKS));
    let t1 = rrkernel::now();

    let st = scheduler::stats();
    let mut seq = [0u32; READ];
    let n = read_log(&mut seq);
    rprintln!("--- wide window: 2ms group over 1ms and 0.5ms children ---");
    rprintln!(
        "measured ticks {} to {} (asked {}), {} log entries",
        t0,
        t1,
        MEASURE_TICKS,
        LOG_N.load(Ordering::Acquire)
    );

    let mut c1 = 0u32;
    let mut c2 = 0u32;
    let mut worst = 0u32;
    let mut i = 0usize;
    while i < n {
        let id = seq[i];
        let mut len = 1usize;
        while i + len < n && seq[i + len] == id {
            len += 1;
        }
        if id == A1 {
            c1 += len as u32;
        } else if id == A2 {
            c2 += len as u32;
        }
        let q = quantum_of(id);
        if len as u32 > q {
            worst = (len as u32) - q;
        }
        rprintln!("  {} x {} tick(s), quantum {}", id, len, q);
        i += len;
    }
    rprintln!("counts: {} in {} ticks, {} in {} ticks", A1, c1, A2, c2);
    rprintln!("kernel: ticks {} switches {}", st.ticks, st.switches);

    let ok = c1 != 0 && c2 != 0 && worst == 0 && matches!(c1.abs_diff(2 * c2), 0..=2);
    if worst != 0 {
        rprintln!("FAIL: a turn ran {} tick(s) past its quantum", worst);
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
}

//! Two nested round robins measured on the board: what the children actually get, tick by tick.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin rounds
//! ```
//!
//! The tick is 500us here (`Slice::Micros(500)`), which is what makes half-millisecond quanta
//! expressible at all -- quanta must be whole numbers of ticks.
//!
//! Phase A: a window *wider* than the lap. A 2ms group over children of 1ms and 0.5ms, i.e. 4
//! ticks over 2 and 1. The lap is 3 ticks, so no turn is cut short and the children rotate 2:1.
//!
//! Phase B: a window *narrower* than the lap. A 3ms group over children of 1.5ms, 2ms and 4ms,
//! i.e. 6 ticks over 3, 4 and 8. Every window cuts a turn short and the next window resumes it
//! with exactly the time it had left: th1 3 | th2 3, then th2 1 | th3 5, then th3 3 | th1 3, ...
//!
//! Each measured task spins (a task that sleeps is not measuring its own turn) and logs
//! (id, tick) for every tick it sees itself running. Task 0 reads the log back and checks it.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Duration, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;
const LOG_MAX: usize = 192;
const MEASURE: usize = 24;
const WAIT: u64 = 140;

/// Phase A ids.
const A1: u32 = 101;
const A2: u32 = 102;
/// Phase B ids.
const B1: u32 = 201;
const B2: u32 = 202;
const B3: u32 = 203;

static LOG_ID: [AtomicU32; LOG_MAX] = [const { AtomicU32::new(0) }; LOG_MAX];
static LOG_READY: [AtomicU32; LOG_MAX] = [const { AtomicU32::new(0) }; LOG_MAX];
static LOG_N: AtomicU32 = AtomicU32::new(0);
static STOP_A: AtomicU32 = AtomicU32::new(0);
static STOP_B: AtomicU32 = AtomicU32::new(0);

/// Claim the next log slot and publish an entry. The ready flag is what makes a reserve-then-write
/// slot safe for a reader running on another task.
fn log(id: u32) {
    let i = LOG_N.fetch_add(1, Ordering::Relaxed) as usize;
    if i < LOG_MAX {
        LOG_ID[i].store(id, Ordering::Relaxed);
        LOG_READY[i].store(1, Ordering::Release);
    }
}

/// A measured task: spin, and log every tick it sees change while it has the CPU.
fn measured(id: u32, stop: &'static AtomicU32) {
    let mut last = u64::MAX;
    while stop.load(Ordering::Relaxed) == 0 {
        let t = rrkernel::now();
        if t != last {
            log(id);
            last = t;
        }
    }
}

/// Sleep `ticks` ticks, so task 0 gets out of the way for a phase.
fn wait_ticks(ticks: u64) {
    rrkernel::sleep(Duration::from_nanos(ticks * 500_000));
}

/// Copy the first `n` published entries into `into`, returning how many were copied.
fn read_log(into: &mut [u32; MEASURE], n: usize) -> usize {
    let total = LOG_N.load(Ordering::Acquire) as usize;
    let mut k = 0usize;
    let mut i = 0usize;
    while i < total && k < n {
        if LOG_READY[i].load(Ordering::Acquire) == 1 {
            into[k] = LOG_ID[i].load(Ordering::Relaxed);
            k += 1;
        }
        i += 1;
    }
    k
}

fn reset_log() {
    for i in 0..LOG_MAX {
        LOG_READY[i].store(0, Ordering::Relaxed);
    }
    LOG_N.store(0, Ordering::Relaxed);
}

/// Print the runs a sequence implies and return (run count, longest overrun against `quantum_of`).
fn print_runs(seq: &[u32], n: usize, label: &str) -> (usize, u32) {
    let mut runs = 0usize;
    let mut worst_overrun = 0u32;
    let mut i = 0usize;
    while i < n {
        let id = seq[i];
        let mut len = 1usize;
        while i + len < n && seq[i + len] == id {
            len += 1;
        }
        let q = quantum_of(id);
        if len as u32 > q {
            worst_overrun = (len as u32) - q;
        }
        rprintln!("    {}: {} x {} tick(s), quantum {}", label, id, len, q);
        runs += 1;
        i += len;
    }
    (runs, worst_overrun)
}

fn quantum_of(id: u32) -> u32 {
    match id {
        A1 => 2, // 1ms
        A2 => 1, // 0.5ms
        B1 => 3, // 1.5ms
        B2 => 4, // 2ms
        B3 => 8, // 4ms
        _ => 1,
    }
}

fn contains(seq: &[u32], n: usize, id: u32) -> bool {
    let mut i = 0usize;
    while i < n {
        if seq[i] == id {
            return true;
        }
        i += 1;
    }
    false
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Micros(500), 1024);
    rprintln!("rounds: tick {} ns", rrkernel::tick_ns());

    // ---------------- Phase A: window wider than the lap ----------------
    let a_grp = thread::spawn_group(Parent::Root, Slice::Millis(2)).expect("group A");
    thread::spawn_in(Parent::Group(a_grp), Slice::Millis(1), || measured(A1, &STOP_A))
        .expect("a1");
    thread::spawn_in(Parent::Group(a_grp), Slice::Micros(500), || measured(A2, &STOP_A))
        .expect("a2");
    wait_ticks(WAIT);
    let mut seq = [0u32; MEASURE];
    let n_a = read_log(&mut seq, MEASURE);
    STOP_A.store(1, Ordering::Relaxed);
    wait_ticks(8);
    rprintln!("--- phase A: 2ms group over 1ms and 0.5ms children (4 ticks over 2 and 1) ---");
    rprintln!("  {} ticks logged:", n_a);
    let (runs_a, over_a) = print_runs(&seq, n_a, "A");
    let both_a = contains(&seq, n_a, A1) && contains(&seq, n_a, A2);

    // ---------------- Phase B: window narrower than the lap ----------------
    reset_log();
    let b_grp = thread::spawn_group(Parent::Root, Slice::Millis(3)).expect("group B");
    thread::spawn_in(Parent::Group(b_grp), Slice::Micros(1500), || measured(B1, &STOP_B))
        .expect("b1");
    thread::spawn_in(Parent::Group(b_grp), Slice::Millis(2), || measured(B2, &STOP_B))
        .expect("b2");
    thread::spawn_in(Parent::Group(b_grp), Slice::Millis(4), || measured(B3, &STOP_B))
        .expect("b3");
    wait_ticks(WAIT);
    let n_b = read_log(&mut seq, MEASURE);
    STOP_B.store(1, Ordering::Relaxed);
    rprintln!("--- phase B: 3ms group over 1.5ms, 2ms and 4ms children (6 ticks over 3, 4, 8) ---");
    rprintln!("  {} ticks logged:", n_b);
    let (runs_b, over_b) = print_runs(&seq, n_b, "B");
    let all_b = contains(&seq, n_b, B1) && contains(&seq, n_b, B2) && contains(&seq, n_b, B3);

    let st = scheduler::stats();
    rprintln!("--- rounds report -------------------------------------------");
    rprintln!("phase A     : {} ticks, {} runs, both children seen: {}", n_a, runs_a, both_a);
    rprintln!("phase B     : {} ticks, {} runs, all three seen: {}", n_b, runs_b, all_b);
    rprintln!("overruns    : A {} tick(s), B {} tick(s) (want 0 each)", over_a, over_b);
    rprintln!("kernel      : ticks {} switches {}", st.ticks, st.switches);
    rprintln!("-----------------------------------------------------------");

    let mut ok = both_a && all_b && over_a == 0 && over_b == 0;
    if !both_a {
        rprintln!("FAIL: phase A did not show both children");
    }
    if !all_b {
        rprintln!("FAIL: phase B did not show all three children");
    }
    if over_a != 0 || over_b != 0 {
        rprintln!("FAIL: a turn ran longer than its own quantum");
    }
    if n_a < MEASURE || n_b < MEASURE {
        rprintln!("FAIL: fewer ticks logged than expected");
        ok = false;
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
}
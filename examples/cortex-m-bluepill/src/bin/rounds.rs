//! A nested round robin measured on the board, with no atomics anywhere in this file.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin rounds
//! ```
//!
//! Phase: a window *wider* than the lap. A 2ms group over children of 1ms and 0.5ms, i.e. 4
//! ticks over 2 and 1 at a 500us tick. The lap (3 ticks) is shorter than the window (4), so no
//! turn is cut short and the children rotate 2:1.
//!
//! Each measured task counts only in its own variables: how many ticks it ran, how many turns it
//! had, and the longest turn it took. Every eighth tick it publishes those three numbers through
//! a lock. Nothing is shared in the hot path and nothing is atomic: the lock is taken rarely
//! enough that two tasks never fight over it while running.

#![no_std]
#![no_main]

use rrkernel::rrkernel;
use rrkernel::sync::{LockId, Mutex};
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;
const MEASURE_TICKS: u64 = 48;
const SLOTS: usize = 4;

/// What a measured task says about itself. Ordinary data.
#[derive(Clone, Copy)]
struct Report {
    ticks: u32,
    turns: u32,
    longest: u32,
}

/// The shared table, behind a lock, with no atomics in it either.
struct Table {
    ids: [u32; SLOTS],
    reports: [Report; SLOTS],
    count: usize,
}

static TABLE: Mutex<Table> = Mutex::with_id(
    LockId::new(1),
    Table {
        ids: [0; SLOTS],
        reports: [Report {
            ticks: 0,
            turns: 0,
            longest: 0,
        }; SLOTS],
        count: 0,
    },
);

/// Publish this task numbers, adding a slot the first time it is seen.
fn publish(id: u32, report: Report) {
    if let Ok(mut t) = TABLE.lock() {
        let mut i = 0usize;
        while i < t.count {
            if t.ids[i] == id {
                t.reports[i] = report;
                return;
            }
            i += 1;
        }
        if t.count < SLOTS {
            let c = t.count;
            t.ids[c] = id;
            t.reports[c] = report;
            t.count = c + 1;
        }
    }
}

/// Look up a slot, if this task ever published.
fn lookup(id: u32) -> Option<Report> {
    if let Ok(t) = TABLE.lock() {
        let mut i = 0usize;
        while i < t.count {
            if t.ids[i] == id {
                return Some(t.reports[i]);
            }
            i += 1;
        }
    }
    None
}

/// A measured task: spin, and count in ordinary local variables.
fn measured(id: u32) {
    let mut last = u64::MAX;
    let mut ticks = 0u32;
    let mut turns = 0u32;
    let mut run = 0u32;
    let mut longest = 0u32;
    loop {
        let t = rrkernel::now();
        if t != last {
            if t == last.wrapping_add(1) {
                run += 1;
            } else {
                if run > longest {
                    longest = run;
                }
                run = 1;
                turns += 1;
            }
            ticks += 1;
            last = t;
            if ticks % 8 == 0 {
                publish(id, Report { ticks, turns, longest });
            }
        }
        core::hint::spin_loop();
    }
}

const A1: u32 = 101;
const A2: u32 = 102;

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Micros(500), 1024);
    rprintln!("rounds: tick {} ns, no atomics in this file", rrkernel::tick_ns());

    let g = thread::spawn_group(Parent::Root, Slice::Millis(2)).expect("group");
    thread::spawn_in(Parent::Group(g), Slice::Millis(1), || measured(A1)).expect("a1");
    thread::spawn_in(Parent::Group(g), Slice::Micros(500), || measured(A2)).expect("a2");

    let t0 = rrkernel::now();
    rrkernel::sleep_until(scheduler::deadline_after_ticks(MEASURE_TICKS));
    let t1 = rrkernel::now();

    let st = scheduler::stats();
    let one = lookup(A1);
    let half = lookup(A2);
    rprintln!("--- wide window: 2ms group over 1ms and 0.5ms children ---");
    rprintln!("measured ticks {} to {} (asked {})", t0, t1, MEASURE_TICKS);
    rprintln!("kernel: ticks {} switches {}", st.ticks, st.switches);
    match (one, half) {
        (Some(a), Some(b)) => {
            rprintln!("task {}: {} ticks, {} turns, longest turn {}", A1, a.ticks, a.turns, a.longest);
            rprintln!("task {}: {} ticks, {} turns, longest turn {}", A2, b.ticks, b.turns, b.longest);
            let ok = a.longest <= 2 && b.longest <= 1 && matches!(a.ticks.abs_diff(2 * b.ticks), 0..=4);
            if !ok {
                rprintln!("FAIL: turns longer than the time asked for, or the 2:1 share is off");
            }
            rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
        }
        _ => rprintln!("VERDICT : FAIL (a measured task never published)", ),
    }
}
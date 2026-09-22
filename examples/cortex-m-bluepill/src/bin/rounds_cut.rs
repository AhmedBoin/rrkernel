//! A 3ms group over children of 1.5ms, 2ms and 4ms, measured on the board with no atomics.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin rounds_cut
//! ```
//!
//! At a 500us tick the window is 6 ticks over children of 3, 4 and 8, so it is narrower than the
//! 15-tick lap: every window cuts a turn short and the next window resumes it with exactly the
//! time it had left. Each task counts only in its own variables and publishes every eighth tick
//! through a lock, so nothing is shared in the hot path and nothing is atomic.

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

#[derive(Clone, Copy)]
struct Report {
    ticks: u32,
    turns: u32,
    longest: u32,
}

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
                publish(
                    id,
                    Report {
                        ticks,
                        turns,
                        longest,
                    },
                );
            }
        }
        core::hint::spin_loop();
    }
}

const B1: u32 = 201;
const B2: u32 = 202;
const B3: u32 = 203;

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Micros(500), 1024);
    rprintln!(
        "rounds_cut: tick {} ns, no atomics in this file",
        rrkernel::tick_ns()
    );

    let g = thread::spawn_group(Parent::Root, Slice::Millis(3)).expect("group");
    thread::spawn_in(Parent::Group(g), Slice::Micros(1500), || measured(B1)).expect("b1");
    thread::spawn_in(Parent::Group(g), Slice::Millis(2), || measured(B2)).expect("b2");
    thread::spawn_in(Parent::Group(g), Slice::Millis(4), || measured(B3)).expect("b3");

    rrkernel::sleep_until(scheduler::deadline_after_ticks(MEASURE_TICKS));
    let st = scheduler::stats();
    rprintln!("--- narrow window: 3ms group over 1.5ms, 2ms and 4ms children ---");
    rprintln!("kernel: ticks {} switches {}", st.ticks, st.switches);
    let mut ok = true;
    let quanta = [(B1, 3u32), (B2, 4), (B3, 8)];
    for (id, q) in quanta {
        match lookup(id) {
            Some(r) => {
                rprintln!(
                    "task {}: {} ticks, {} turns, longest turn {} of {}",
                    id,
                    r.ticks,
                    r.turns,
                    r.longest,
                    q
                );
                if r.longest > q || r.ticks == 0 {
                    ok = false;
                }
            }
            None => {
                rprintln!("task {} never published", id);
                ok = false;
            }
        }
    }
    if !ok {
        rprintln!("FAIL: a task was starved, or took longer than the time it asked for");
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
}

//! Nested round-robin on the board: a group inside a group, with the children totals
//! deliberately unequal to their slot in **both** directions.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin nested     # flashes, then streams RTT
//! ```
//!
//! The tree, and what each part is there to prove:
//!
//! ```text
//! root (implicit, unbounded)
//!  +-- main       leaf, 1 ms    the ordinary `configure` slice (task 0)
//!  +-- ctrl       group, 5 ms   one level-1 *slot*
//!  |    +-- fast  group, 1 ms   a slot inside the slot
//!  |    |    +-- a  leaf, 2 ms
//!  |    |    +-- b  leaf, 2 ms
//!  |    +-- log   leaf, 2 ms
//!  +-- bg         leaf, 1 ms    plain `thread::spawn`, API unchanged
//! ```
//!
//! * `ctrl` children total 4 ms against a 5 ms slot: **underflow**. The lap repeats until
//!   the slot is genuinely used up, and no visit is cut short.
//! * `fast` children total 4 ms against a 1 ms slot: **overflow**. The slot closes mid-child;
//!   the next visit resumes that child with exactly the time it had left, so `a` gets its two
//!   ticks consecutively *from its own point of view* although `fast` was visited twice.
//! * Every leaf keeps a 10 ms **absolute** deadline, so the run measures wall-clock sleep
//!   correctness at depth: a leaf deadline must not depend on how often its group ran.
//!
//! `PASS` requires that every leaf ran, nobody woke early, the worst wake lateness stayed
//! within one tick, and the two groups are absent from the thread count.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Duration, Slice};
use rtt_target::rprintln;

/// The core clock: the one number the kernel cannot work out for itself.
const CORE_HZ: u32 = 8_000_000;
const RUN_MS: u64 = 3000;
/// Every leaf period: short enough to be measured hundreds of times in three seconds.
const PERIOD_MS: u64 = 10;
/// Leaves in the tree: `main` plus a, b, log and bg. The two groups must not be counted.
const LEAVES: usize = 5;

static A_RUNS: AtomicU32 = AtomicU32::new(0);
static B_RUNS: AtomicU32 = AtomicU32::new(0);
static LOG_RUNS: AtomicU32 = AtomicU32::new(0);
static BG_RUNS: AtomicU32 = AtomicU32::new(0);
static A_LATE: AtomicU32 = AtomicU32::new(0);
static B_LATE: AtomicU32 = AtomicU32::new(0);
static LOG_LATE: AtomicU32 = AtomicU32::new(0);
static BG_LATE: AtomicU32 = AtomicU32::new(0);
static EARLY: AtomicU32 = AtomicU32::new(0);

/// What every leaf runs: count turns against an absolute deadline, and record the worst
/// lateness and any early wake. The figure is in milliseconds because that is the unit
/// `now()` returns, and one tick is 1 ms here, so `1` means "within one tick".
fn leaf(name: &'static str, runs: &AtomicU32, late: &AtomicU32) {
    let period = Duration::from_millis(PERIOD_MS);
    let mut next = rrkernel::deadline_after(period);
    loop {
        rrkernel::sleep_until(next);
        let now = rrkernel::now();
        if now >= next {
            late.fetch_max((now - next) as u32, Ordering::Relaxed);
        } else {
            EARLY.fetch_add(1, Ordering::Relaxed);
        }
        runs.fetch_add(1, Ordering::Relaxed);
        // A progress line every hundred turns: the report is a single snapshot, and the
        // terminal should show the tree alive for as long as the probe stays attached.
        let n = runs.load(Ordering::Relaxed);
        if n % 100 == 0 {
            rprintln!("[{}] {} turns at t={} ms", name, n, rrkernel::now());
        }
        // The next *absolute* deadline: a wake that lands a tick late is corrected on the
        // next period instead of accumulating, which is what makes the figure meaningful.
        next = rrkernel::deadline_add(next, period);
    }
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    // The first line out, before anything else: if this never appears, nothing after it ran and
    // the fault is in startup or `configure` rather than in the tree.
    rprintln!("nested: starting");
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!(
        "nested: tick {} ns, tree = 2 groups + {} leaves",
        rrkernel::tick_ns(),
        LEAVES
    );

    // The level-1 slot: 5 ms, holding a nested group and a leaf (4 ms of children in total).
    let ctrl = thread::spawn_group(Parent::Root, Slice::Millis(5)).expect("spawn_group ctrl");
    // The slot inside the slot: 1 ms against 4 ms of children, so its visits truncate.
    let fast =
        thread::spawn_group(Parent::Group(ctrl), Slice::Millis(1)).expect("spawn_group fast");

    thread::spawn_in(Parent::Group(fast), Slice::Millis(2), || {
        leaf("a", &A_RUNS, &A_LATE)
    })
    .expect("spawn_in a");
    thread::spawn_in(Parent::Group(fast), Slice::Millis(2), || {
        leaf("b", &B_RUNS, &B_LATE)
    })
    .expect("spawn_in b");
    thread::spawn_in(Parent::Group(ctrl), Slice::Millis(2), || {
        leaf("log", &LOG_RUNS, &LOG_LATE)
    })
    .expect("spawn_in log");

    // The ordinary path, unchanged: level 1 under the root, on the slice `configure` set.
    thread::spawn(|| leaf("bg", &BG_RUNS, &BG_LATE));

    // A quantum shorter than one tick cannot be honoured, so it is refused with the numbers
    // in the error rather than silently rounded up to a tick.
    match thread::spawn_in(Parent::Group(fast), Slice::Micros(500), || {}) {
        Err(e) => rprintln!("sub-tick quantum refused: {}", e),
        Ok(_) => rprintln!("BUG: a sub-tick quantum was accepted"),
    }

    // Task 0 observes for three seconds, then reports. It is a level-1 leaf like any other.
    let start = rrkernel::now();
    while rrkernel::now().wrapping_sub(start) < RUN_MS {
        rrkernel::sleep(Duration::from_millis(50));
    }
    report();
}

fn report() {
    let st = scheduler::stats();
    let leaves = [
        (
            "a  ",
            A_RUNS.load(Ordering::Relaxed),
            A_LATE.load(Ordering::Relaxed),
        ),
        (
            "b  ",
            B_RUNS.load(Ordering::Relaxed),
            B_LATE.load(Ordering::Relaxed),
        ),
        (
            "log",
            LOG_RUNS.load(Ordering::Relaxed),
            LOG_LATE.load(Ordering::Relaxed),
        ),
        (
            "bg ",
            BG_RUNS.load(Ordering::Relaxed),
            BG_LATE.load(Ordering::Relaxed),
        ),
    ];
    let early = EARLY.load(Ordering::Relaxed);
    let active = scheduler::active_threads();
    let total = scheduler::total_threads();

    rprintln!("--- nested report -----------------------------------------");
    for (name, runs, late) in leaves {
        rprintln!("leaf {}: {} runs, worst lateness {} ms", name, runs, late);
    }
    rprintln!("early wakes  : {} (want 0)", early);
    rprintln!(
        "thread count : active {} total {} (want {}, groups are not threads)",
        active,
        total,
        LEAVES
    );
    rprintln!("ticks        : {} switches {}", st.ticks, st.switches);
    rprintln!("switch worst : {} cycles", st.worst_latency);
    rprintln!("-----------------------------------------------------------");

    let mut ok = true;
    for (name, runs, late) in leaves {
        if runs < 20 {
            rprintln!("FAIL: leaf {} ran only {} times", name, runs);
            ok = false;
        }
        if late > 2 {
            rprintln!("FAIL: leaf {} woke up to {} ms late", name, late);
            ok = false;
        }
    }
    if early != 0 {
        rprintln!("FAIL: {} early wakes", early);
        ok = false;
    }
    if active != LEAVES {
        rprintln!("FAIL: active_threads() is {}, expected {}", active, LEAVES);
        ok = false;
    }
    if st.ticks < 2000 {
        rprintln!("FAIL: only {} ticks in three seconds", st.ticks);
        ok = false;
    }
    if st.switches == 0 {
        rprintln!("FAIL: no switches at all");
        ok = false;
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
    // Task 0 then returns. Every worker above is an infinite loop, so the tree keeps running
    // and the RTT stream stays open; `#[rrkernel]` turns this return into "unlink task 0 and
    // switch away forever", exactly as in the flat example. (Returning rather than spinning
    // here is also what keeps the macro-generated tail after `main` reachable: a diverging
    // last call makes it dead code, and the build says so.)
}

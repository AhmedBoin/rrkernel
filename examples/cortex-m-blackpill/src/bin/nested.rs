//! Nested round-robin on the Black Pill: a group inside a group, plus the on-board LED as a
//! leaf *inside* the nested group, so the tree is visible on the board and not just in RTT.
//!
//! ```text
//! cd examples/cortex-m-blackpill
//! cargo run --release --bin nested     # flashes, then streams RTT
//! ```
//!
//! ```text
//! root (implicit, unbounded)
//!  +-- main       leaf, 1 ms    the ordinary `configure` slice (task 0)
//!  +-- ctrl       group, 5 ms   one level-1 *slot*
//!  |    +-- fast  group, 1 ms   a slot inside the slot
//!  |    |    +-- a   leaf, 2 ms
//!  |    |    +-- b   leaf, 2 ms
//!  |    +-- log   leaf, 2 ms
//!  |    +-- led   leaf, 1 ms    the on-board LED, one blink per second
//!  +-- bg         leaf, 1 ms    plain `thread::spawn`, API unchanged
//! ```
//!
//! * `ctrl` children total 5 ms against a 5 ms slot: they add up exactly.
//! * `fast` children total 4 ms against a 1 ms slot: **overflow**. The slot closes mid-child
//!   and the next visit resumes that child with the time it had left, so `a` gets its two
//!   ticks consecutively *from its own point of view* although `fast` was visited twice.
//! * Every leaf holds a 10 ms **absolute** deadline, so the run measures wall-clock sleep
//!   correctness at depth: a leaf deadline must not depend on how often its group ran.
//!
//! The clock is read back from the HAL, so `configure` gets the frequency actually running.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Duration, Slice};
use rtt_target::rprintln;
use stm32f4xx_hal::{pac, prelude::*, rcc::Config};

const RUN_MS: u64 = 3000;
/// Every leaf period: short enough to be measured hundreds of times in three seconds.
const PERIOD_MS: u64 = 10;
/// Leaves in the tree: `main` plus a, b, log, led and bg. The two groups must not be counted.
const LEAVES: usize = 6;

static A_RUNS: AtomicU32 = AtomicU32::new(0);
static B_RUNS: AtomicU32 = AtomicU32::new(0);
static LOG_RUNS: AtomicU32 = AtomicU32::new(0);
static BG_RUNS: AtomicU32 = AtomicU32::new(0);
static A_LATE: AtomicU32 = AtomicU32::new(0);
static B_LATE: AtomicU32 = AtomicU32::new(0);
static LOG_LATE: AtomicU32 = AtomicU32::new(0);
static BG_LATE: AtomicU32 = AtomicU32::new(0);
static BLINKS: AtomicU32 = AtomicU32::new(0);
static EARLY: AtomicU32 = AtomicU32::new(0);

/// What every counter leaf runs: count turns against an absolute deadline, and record the
/// worst lateness and any early wake. The figure is in milliseconds because that is the unit
/// `now()` returns, and one tick is 1 ms here, so `1` means "within one tick".
fn leaf(runs: &AtomicU32, late: &AtomicU32) {
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
        // The next *absolute* deadline: a wake that lands a tick late is corrected on the
        // next period instead of accumulating, which is what makes the figure meaningful.
        next = rrkernel::deadline_add(next, period);
    }
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    // --- the HAL: clock and the board LED -----------------------------------------
    let dp = pac::Peripherals::take().unwrap();
    // HSI, 16 MHz: no dependency on the board crystal, so this cannot hang waiting for an
    // oscillator to start.
    let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(16.MHz()));
    let gpioc = dp.GPIOC.split(&mut rcc);
    let led = gpioc.pc13.into_push_pull_output(); // PC13 = the on-board LED, active low
    let sysclk = rcc.clocks.sysclk().to_Hz();

    // --- the kernel --------------------------------------------------------------
    configure(sysclk, Slice::Millis(1), 2048);
    rprintln!(
        "nested: {} Hz, tick {} ns, tree = 2 groups + {} leaves",
        sysclk,
        rrkernel::tick_ns(),
        LEAVES
    );

    // The level-1 slot: 5 ms, holding a nested group and two leaves (5 ms of children).
    let ctrl = thread::spawn_group(Parent::Root, Slice::Millis(5)).expect("spawn_group ctrl");
    // The slot inside the slot: 1 ms against 4 ms of children, so its visits truncate.
    let fast =
        thread::spawn_group(Parent::Group(ctrl), Slice::Millis(1)).expect("spawn_group fast");

    thread::spawn_in(Parent::Group(fast), Slice::Millis(2), || {
        leaf(&A_RUNS, &A_LATE)
    })
    .expect("spawn_in a");
    thread::spawn_in(Parent::Group(fast), Slice::Millis(2), || {
        leaf(&B_RUNS, &B_LATE)
    })
    .expect("spawn_in b");
    thread::spawn_in(Parent::Group(ctrl), Slice::Millis(2), || {
        leaf(&LOG_RUNS, &LOG_LATE)
    })
    .expect("spawn_in log");

    // The LED, inside the group: if this blinks, a leaf two levels down is getting the CPU.
    thread::spawn_in(Parent::Group(ctrl), Slice::Millis(1), move || {
        let mut led = led;
        loop {
            led.set_low(); // active low: pulling it low lights the LED
            rrkernel::sleep_ms(500);
            led.set_high();
            rrkernel::sleep_ms(500);
            BLINKS.fetch_add(1, Ordering::Relaxed);
        }
    })
    .expect("spawn_in led");

    // The ordinary path, unchanged: level 1 under the root, on the slice `configure` set.
    thread::spawn(|| leaf(&BG_RUNS, &BG_LATE));

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
    let blinks = BLINKS.load(Ordering::Relaxed);
    let active = scheduler::active_threads();
    let total = scheduler::total_threads();

    rprintln!("--- nested report -----------------------------------------");
    for (name, runs, late) in leaves {
        rprintln!("leaf {}: {} runs, worst lateness {} ms", name, runs, late);
    }
    rprintln!("leaf led: {} blinks (watch PC13)", blinks);
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
    if blinks < 2 {
        rprintln!(
            "FAIL: the LED leaf inside the group only blinked {} times",
            blinks
        );
        ok = false;
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
    // Task 0 then returns; the workers above are infinite loops, so the tree keeps running
    // and the LED keeps blinking while you read the report.
}

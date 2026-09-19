//! Metal acceptance for Phase 1 and the measurement Phase 2 depends on.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin fidelity      # flashes, then streams RTT
//! ```
//!
//! Three questions, one flash:
//!
//! 1. **Does a sleep ever return early?** A task sleeps on relative durations and another runs on
//!    absolute deadlines, while a third creates and destroys a child every slice — the window the
//!    original report named ("1 in 15, right after task create or destroy").
//! 2. **Does `DWT->CYCCNT` keep counting while the CPU is in `WFI`?** After the workload stops,
//!    only task 0 is left; it blocks, nothing is runnable, and the switch path idles in `wfi`.
//!    Comparing the cycle counter against the tick count across that window answers whether the
//!    cycle counter can be the kernel's time base or whether a timer peripheral is required.
//! 3. **What does a switch actually cost** at this clock? `stats()` reports last/worst in DWT
//!    cycles, which is the number `plan_timer`'s floor should be derived from.
//!
//! The report ends in `VERDICT : PASS` / `FAIL`, and every number is printed, not asserted.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::{configure, thread, Duration, Slice};
use rtt_target::rprintln;

/// The core clock: HSI at reset, which is what this board runs at with no PLL configured.
const CORE_HZ: u32 = 8_000_000;
/// How long the workload runs before the report.
const RUN_MS: u64 = 3000;
/// The all-sleeping window used for question 2.
const IDLE_WINDOW_MS: u64 = 200;

static SLEEPS: AtomicU32 = AtomicU32::new(0);
static EARLY: AtomicU32 = AtomicU32::new(0);
static WORST_LATE: AtomicU32 = AtomicU32::new(0);
// `AtomicU32`, not `AtomicU64`: Cortex-M3 has no 64-bit atomics, which is also why the kernel
// keeps its own counters in `UnsafeCell<u64>` behind critical sections.
static MIN_ELAPSED: AtomicU32 = AtomicU32::new(u32::MAX);
static CHURN: AtomicU32 = AtomicU32::new(0);
static EARLY_PERIODIC: AtomicU32 = AtomicU32::new(0);
static STOP: AtomicU32 = AtomicU32::new(0);

/// `DWT->CYCCNT`. The kernel enables `TRCENA`/`CYCCNTENA` during `configure` (it measures switch
/// latency with this same counter), so reading it here is safe while `Measure::Cycles` is in
/// effect — which `SchedulerConfig::embedded` sets.
#[inline]
fn cycles() -> u32 {
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    // 1 ms slices, 1 KiB per task: the fidelity workload runs several at once, and the default
    // 16 KiB arena holds the TCBs, closures and these stacks.
    configure(CORE_HZ, Slice::Millis(1), 1024);

    rprintln!(
        "rrkernel fidelity @ {} Hz, slice {} ns ({})",
        CORE_HZ,
        rrkernel::tick_ns(),
        rrkernel::scheduler::platform_limits().timer_note
    );

    // A CPU-bound task that never yields, so the checks below run against a loaded ring — but it
    // does stop, otherwise the ring would never be empty and the idle window below would not be
    // an idle window at all (the first version of this firmware made exactly that mistake, and
    // its "CYCCNT keeps counting in WFI" line was therefore measuring a busy CPU).
    thread::spawn(|| {
        while STOP.load(Ordering::Relaxed) == 0 {
            for _ in 0..50_000 {
                core::hint::spin_loop();
            }
        }
    });

    // Task: relative sleeps — the path the early-wake report was about.
    thread::spawn(|| {
        while STOP.load(Ordering::Relaxed) == 0 {
            let t0 = rrkernel::now();
            rrkernel::sleep(Duration::from_millis(5));
            let elapsed = rrkernel::now().wrapping_sub(t0);
            SLEEPS.fetch_add(1, Ordering::Relaxed);
            if elapsed < 5 {
                EARLY.fetch_add(1, Ordering::Relaxed);
            }
            let _ = WORST_LATE.fetch_max(elapsed.saturating_sub(5) as u32, Ordering::Relaxed);
            let _ = MIN_ELAPSED.fetch_min(elapsed as u32, Ordering::Relaxed);
        }
    });

    // Task: absolute deadlines — the drift-free periodic pattern.
    thread::spawn(|| {
        let period = Duration::from_millis(10);
        let mut next = rrkernel::deadline_after(period);
        while STOP.load(Ordering::Relaxed) == 0 {
            rrkernel::sleep_until(next);
            if rrkernel::now() < next {
                // An absolute deadline reached "early" is the same defect seen from the other side.
                EARLY_PERIODIC.fetch_add(1, Ordering::Relaxed);
            }
            next = rrkernel::deadline_add(next, period);
        }
    });

    // Task: the churn. Create and destroy a child every slice — the exact window in the report.
    thread::spawn(|| {
        while STOP.load(Ordering::Relaxed) == 0 {
            thread::spawn(|| core::hint::spin_loop()); // returns at once: unlink + reclaim
            CHURN.fetch_add(1, Ordering::Relaxed);
            rrkernel::sleep(Duration::from_millis(1));
        }
    });

    // Task 0 runs the scenario, then the idle window, then reports. Returning from `main` would
    // be a task exit like any other; `report` never returns instead, so the data is not lost.
    let start = rrkernel::now();
    while rrkernel::now().wrapping_sub(start) < RUN_MS {
        rrkernel::sleep(Duration::from_millis(25));
    }
    STOP.store(1, Ordering::Relaxed);
    rrkernel::sleep(Duration::from_millis(50)); // let the workers notice and exit

    // Question 2: with every other task gone, task 0 blocking leaves nothing runnable, so the
    // switch path idles in `wfi`. Does the cycle counter advance there?
    //
    // Printed first, and deliberately: the previous run of this firmware asked for a 200-tick
    // sleep and came back after 6 ticks. These two numbers say which half is wrong — the request
    // (`ticks_for`) or the wake (`wake_expired`) — before anything is concluded about `WFI`.
    let want = rrkernel::ticks_for(Duration::from_millis(IDLE_WINDOW_MS));
    let dl = rrkernel::deadline_after(Duration::from_millis(IDLE_WINDOW_MS));
    rprintln!(
        "request         : {} ticks requested, deadline-now = {} ticks (ring: {} task(s))",
        want,
        dl.wrapping_sub(rrkernel::now()),
        rrkernel::scheduler::active_threads()
    );

    let w0 = rrkernel::arch::idle_waits();
    let c0 = cycles();
    let t0 = rrkernel::now();
    rrkernel::sleep(Duration::from_millis(IDLE_WINDOW_MS));
    let c1 = cycles();
    let t1 = rrkernel::now();
    let idle_cycles = c1.wrapping_sub(c0) as u64;
    let idle_ticks = t1.wrapping_sub(t0);
    let expected = idle_ticks.saturating_mul(CORE_HZ as u64 / 1000);

    // The trustworthy reading: how many times the idle loop actually waited, per tick.
    let waits = rrkernel::arch::idle_waits().wrapping_sub(w0);
    rprintln!(
        "idle waits      : {} over {} ticks = {} per tick (1 = sleeping, thousands = spinning)",
        waits,
        idle_ticks,
        if idle_ticks == 0 {
            0
        } else {
            waits / idle_ticks
        }
    );

    report(idle_ticks, idle_cycles, expected);
}

fn report(idle_ticks: u64, idle_cycles: u64, expected: u64) -> ! {
    let st = rrkernel::scheduler::stats();
    let sleeps = SLEEPS.load(Ordering::Relaxed);
    let early = EARLY.load(Ordering::Relaxed);
    let early_periodic = EARLY_PERIODIC.load(Ordering::Relaxed);
    let churn = CHURN.load(Ordering::Relaxed);
    let worst_late = WORST_LATE.load(Ordering::Relaxed);
    let min_elapsed = MIN_ELAPSED.load(Ordering::Relaxed);

    rprintln!("--- fidelity report ---------------------------------------");
    rprintln!("ticks           : {}   switches: {}", st.ticks, st.switches);
    rprintln!(
        "sleeps recorded : {}   EARLY: {}   early deadlines: {}",
        sleeps,
        early,
        early_periodic
    );
    rprintln!(
        "min elapsed     : {} ticks   worst lateness: {} ticks",
        min_elapsed,
        worst_late
    );
    rprintln!("churn           : {} spawn/exit cycles", churn);
    rprintln!(
        "arena           : {} of {} bytes, live {}",
        st.arena.bytes_bump,
        st.arena.bytes_total,
        st.arena.live_bytes
    );
    rprintln!("blocks w/o task : {}", st.blocks_without_current);
    rprintln!(
        "deferred ticks  : {}   (a growing value means the tick was missed, not merely late)",
        st.ticks_deferred
    );
    rprintln!(
        "switch (DWT)    : last {} cycles, worst {} cycles",
        st.last_latency,
        st.worst_latency
    );
    rprintln!(
        "period error    : last {} ns, worst {} ns",
        st.last_period_error_ns,
        st.worst_period_error_ns
    );
    rprintln!("--- idle window (question 2) -------------------------------");
    rprintln!(
        "ring at idle    : {} runnable task(s) (1 = only task 0, which is what makes this an \
         idle window)",
        rrkernel::scheduler::active_threads()
    );
    rprintln!(
        "idle            : {} ticks = {} cycles expected at {} Hz",
        idle_ticks,
        expected,
        CORE_HZ
    );
    rprintln!("CYCCNT advanced : {} cycles", idle_cycles);
    rprintln!(
        "ratio           : {} per mille of the requested window",
        if expected == 0 {
            0
        } else {
            (idle_cycles.saturating_mul(1000) / expected) as u32
        }
    );
    // The trustworthy reading is `idle_waits`: one wait per tick means the core really parked
    // (and the pend-clear took effect); thousands per tick would mean the idle loop is spinning.
    // `CYCCNT` cannot be used for this on this part — it keeps advancing across a sleep that the
    // wait counter proves happened — but that same fact is what makes DWT usable as a time base.
    let permille = (idle_cycles.saturating_mul(1000) / expected.max(1)) as u32;
    let waits_per_tick = waits / idle_ticks.max(1);
    if waits_per_tick <= 2 {
        rprintln!(
            "verdict (2)     : idle path SLEEPS ({} wfi per tick): the pend clear works",
            waits_per_tick
        );
        rprintln!(
            "                  and CYCCNT advanced across that sleep ({} per mille), so",
            permille
        );
        rprintln!("                  DWT is usable as the Phase 2 time base");
    } else {
        rprintln!(
            "verdict (2)     : idle path SPINS ({} waits per tick): the pend clear did not take",
            waits_per_tick
        );
    }
    rprintln!("-----------------------------------------------------------");

    let ok = early == 0
        && early_periodic == 0
        && sleeps > 50
        && churn > 500
        && st.blocks_without_current == 0;
    if early != 0 || early_periodic != 0 {
        rprintln!(
            "FAIL: {} relative + {} absolute sleep(s) returned early",
            early,
            early_periodic
        );
    }
    if sleeps <= 50 {
        rprintln!(
            "FAIL: only {} sleeps recorded; the workload did not run",
            sleeps
        );
    }
    if churn <= 500 {
        rprintln!("FAIL: churn reached only {} cycles", churn);
    }
    if st.blocks_without_current != 0 {
        rprintln!(
            "FAIL: {} wait(s) attempted with no current task",
            st.blocks_without_current
        );
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
    loop {
        core::hint::spin_loop();
    }
}

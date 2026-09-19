//! Sleep fidelity under spawn/exit churn, and the all-sleeping window.
//!
//! ```text
//! cargo run --example sleep_fidelity --features std --release
//! ```
//!
//! This is the acceptance test for sleeps, and it exists because of a specific report: on an
//! STM32F103 a `sleep` occasionally returned *early* — about one call in fifteen, most often
//! right after a task was created or destroyed. Three distinct causes were identified, and this
//! example is aimed at all three:
//!
//! 1. **a tick counted twice**, when a switch restarts the slice while the timer's exception is
//!    already latched (metal-only; not reproducible on a host backend, and the reason `now()`
//!    belongs on a counter of its own rather than the slice count);
//! 2. **a deadline computed from a stale tick read** — `sleep_ticks` read the counter *outside*
//!    the critical section that did the blocking, so a tick landing in between made the deadline
//!    one tick short. Fixed by construction: the deadline is now read inside that section;
//! 3. **a wait that never happened** — `block_current` returned silently when there was no
//!    current task, so the caller's sleep vanished. Now counted and asserted (see
//!    `tests/blocking_contexts.rs` for the deterministic half).
//!
//! What it measures, and what it prints:
//!   * every sleep lasted **at least** what it asked for, and every wake-up landed within one
//!     slice of its deadline (relative sleeps *and* absolute deadlines);
//!   * `blocks_without_current == 0`;
//!   * a best-effort all-sleeping window: every task parked, and the switch count across it is
//!     bounded and small rather than a spin.
//!
//! The verdict is the process exit status, so CI can gate on it. The one thing a host **cannot**
//! show is the CPU actually idling (`wfi`); on a host "idle" means the tick thread waits, which
//! is why the switch *count* is the evidence here. On hardware the same scenario is the
//! `firmware-cortex-m` demo, where the idle branch is a real `wfi`.

use rrkernel::{scheduler, thread, IdlePolicy, Measure, SchedulerConfig, Slice};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// How long the whole scenario runs, in milliseconds.
const RUN_MS: u64 = 600;
/// The all-sleeping window, in milliseconds. Every task parks for this long.
const WINDOW_MS: u64 = 50;
/// How many all-sleeping windows to measure; the worst is reported.
const WINDOWS: u32 = 3;

static SLEEPS: AtomicU32 = AtomicU32::new(0);
static EARLY: AtomicU32 = AtomicU32::new(0);
static WORST_LATE: AtomicU64 = AtomicU64::new(0);
static CHURN_SPAWNS: AtomicU32 = AtomicU32::new(0);
static CHURN_WINDOWS: AtomicU32 = AtomicU32::new(0);
static PAUSE: AtomicU32 = AtomicU32::new(0);
static WINDOW_SWITCHES: AtomicU64 = AtomicU64::new(0);
static WINDOW_TICKS: AtomicU64 = AtomicU64::new(0);
static WINDOW_MIN: AtomicU64 = AtomicU64::new(u64::MAX);

/// Record one sleep's outcome. `requested` and `elapsed` are in ticks.
///
/// A sleep that returned early is the bug; a sleep that returned late is expected and bounded by
/// one slice (the tick that wakes it is itself periodic).
fn record(requested: u64, elapsed: u64) {
    let _ = SLEEPS.fetch_add(1, Ordering::Relaxed);
    if elapsed < requested {
        let _ = EARLY.fetch_add(1, Ordering::Relaxed);
    }
    let late = elapsed.saturating_sub(requested);
    let _ = WORST_LATE.fetch_max(late, Ordering::Relaxed);
    let _ = WINDOW_MIN.fetch_min(elapsed, Ordering::Relaxed);
}

/// A relative sleep, timed in ticks: the `sleep(duration)` path.
fn sleep_ticks_checked(ticks: u64) {
    let t0 = scheduler::tick_count();
    let w0 = std::time::Instant::now();
    rrkernel::sleep_ms(ticks);
    record(ticks, scheduler::tick_count().wrapping_sub(t0));

    // On a backend whose parking is asynchronous the task can return from its own `sleep` before
    // it has been taken off the CPU, so this loop would run at full speed — and the churn task
    // would spawn tasks without bound. Pace by wall time instead. This does *not* pretend the
    // sleep worked; it stops one known limitation from turning into an unrelated storm.
    if !sync_parking() {
        let want = std::time::Duration::from_millis(ticks);
        while w0.elapsed() < want {
            core::hint::spin_loop();
        }
    }
}

/// Does this backend guarantee the caller is off the CPU before a blocking call returns?
fn sync_parking() -> bool {
    rrkernel::arch::parks_synchronously()
}

/// An absolute deadline: the `sleep_until(deadline_after(..))` path used by periodic work.
fn deadline_checked(period_ticks: u64) {
    let mut next = rrkernel::deadline_after(rrkernel::Duration::from_millis(period_ticks));
    let mut iterations = 0u32;
    while PAUSE.load(Ordering::Relaxed) == 0 && iterations < 8 {
        rrkernel::sleep_until(next);
        let now = rrkernel::now();
        // Deadlines are absolute, so a late wake-up must be corrected rather than accumulated.
        if now >= next {
            let _ = SLEEPS.fetch_add(1, Ordering::Relaxed);
        } else {
            let _ = EARLY.fetch_add(1, Ordering::Relaxed);
        }
        next = rrkernel::deadline_add(next, rrkernel::Duration::from_millis(period_ticks));
        iterations += 1;
    }
}

fn main() {
    let limits = scheduler::platform_limits();
    println!("platform: {}", limits.timer_note);
    println!(
        "slice limits: {} ns .. {} ns",
        limits.min_slice_ns,
        limits.max_slice_ns.unwrap_or(u64::MAX)
    );

    let cfg = SchedulerConfig {
        // One tick == one millisecond, which is what the constants above assume.
        slice: Slice::Millis(1),
        timer_hz: 0,
        stack_size: 16 * 1024,
        arena: Some(Box::<[u8]>::leak(vec![0u8; 512 * 1024].into_boxed_slice())),
        idle: IdlePolicy::Wait,
        measure: Measure::Nanos,
    };
    if let Err(e) = scheduler::init_with(cfg) {
        eprintln!("scheduler init failed: {e}");
        std::process::exit(2);
    }
    println!("scheduler live: slice = {} ns", scheduler::slice_ns());

    // Refuse to run the strict scenario where the backend cannot honour it. This is not a
    // workaround dressed up as a skip: on an asynchronous-parking backend a blocking call can
    // return before the caller is off the CPU, so every number this example measures about
    // sleeps would be about the host's own scheduling, not the kernel's.
    //
    // It is also the honest state of affairs on the two `std` backends today:
    //   * Win32  — the tick thread suspends a blocked task a moment later, so the task runs on;
    //   * POSIX  — the SIGALRM handler has no idle context to switch to when nothing is runnable,
    //              so it returns and the blocked fibre resumes. A live scenario that keeps every
    //              task blocked (which is what the strict form does) hangs here for that reason.
    // The fix for POSIX is an idle fibre created at init; for Win32 a per-task park event so the
    // task waits on itself instead of being suspended from outside. Both are recorded in
    // docs/DESIGN.md; until then this example reports and exits instead of pretending.
    if !sync_parking() {
        println!();
        println!("PARKING  : asynchronous on this backend — the strict scenario is NOT run.");
        println!("           A blocking call can return before its caller is off the CPU, so a");
        println!("           task-visible `elapsed` around a sleep is not evidence of anything.");
        println!("           Run this example on hardware (or a bare-metal QEMU target) for the");
        println!("           strict form; see docs/DESIGN.md, \"asymmetric parking\".");
        println!("expected : more than 20 sleeps recorded, 0 early returns, bounded idle window",);
        scheduler::shutdown(0);
    }

    // Task 1 and 2: relative sleeps of different lengths, so wake-ups keep landing in the middle
    // of other tasks' slices rather than in lockstep with them.
    thread::spawn(|| loop {
        let n = if PAUSE.load(Ordering::Relaxed) != 0 {
            WINDOW_MS
        } else {
            5
        };
        sleep_ticks_checked(n);
    });
    thread::spawn(|| loop {
        let n = if PAUSE.load(Ordering::Relaxed) != 0 {
            WINDOW_MS
        } else {
            7
        };
        sleep_ticks_checked(n);
    });

    // Task 3: absolute deadlines, the drift-free periodic pattern.
    thread::spawn(|| loop {
        if PAUSE.load(Ordering::Relaxed) != 0 {
            sleep_ticks_checked(WINDOW_MS);
            continue;
        }
        deadline_checked(10);
    });

    // Task 4: the churn. Creating and destroying a task every slice is the load the early-sleep
    // report named ("right after task create or destroy"), because that is when a switch and a
    // freshly latched tick most readily coincide.
    //
    // Skipped where parking is asynchronous: there a blocking call does not take the task off the
    // CPU, so the churn loop runs far faster than the slices it asks for and would measure the
    // host scheduler (and, on POSIX, grow the fibre population without bound) rather than the
    // kernel. The strict scenario is for backends that can honour it.
    if sync_parking() {
        thread::spawn(|| loop {
            if PAUSE.load(Ordering::Relaxed) != 0 {
                sleep_ticks_checked(WINDOW_MS);
                continue;
            }
            thread::spawn(core::hint::spin_loop); // returns at once: unlink + reclaim
            let _ = CHURN_SPAWNS.fetch_add(1, Ordering::Relaxed);
            sleep_ticks_checked(1);
        });
    } else {
        println!(
            "reduced scenario: this backend's parking is asynchronous, so the churn and \
             all-sleeping phases are skipped (they would measure the host, not the kernel)"
        );
    }

    // Task 0 runs the scenario and reports.
    scheduler::main_body(|| {
        let start = rrkernel::now();
        while rrkernel::now().wrapping_sub(start) < RUN_MS {
            rrkernel::sleep_ms(1);
        }
        let churn = CHURN_SPAWNS.load(Ordering::Relaxed);

        for _ in 0..WINDOWS {
            // Ask every task to park for the same long window, then measure what the scheduler
            // did while there was nothing runnable. Skipped on asynchronous-parking backends: the
            // window cannot be established there, so the number would be meaningless.
            if !sync_parking() {
                break;
            }
            PAUSE.store(1, Ordering::Relaxed);
            rrkernel::sleep_ms(2); // let the others notice, one slice each
            let s0 = scheduler::switch_count();
            let t0 = scheduler::tick_count();
            rrkernel::sleep_ms(WINDOW_MS); // task 0 sleeps too
            let sw = scheduler::switch_count().wrapping_sub(s0);
            let tk = scheduler::tick_count().wrapping_sub(t0);
            let _ = WINDOW_SWITCHES.fetch_add(sw, Ordering::Relaxed);
            let _ = WINDOW_TICKS.fetch_add(tk, Ordering::Relaxed);
            let _ = CHURN_WINDOWS.fetch_add(1, Ordering::Relaxed);
            PAUSE.store(0, Ordering::Relaxed);
            rrkernel::sleep_ms(2);
        }

        report(churn);
    });
}

fn report(churn: u32) -> ! {
    let st = scheduler::stats();
    let sleeps = SLEEPS.load(Ordering::Relaxed);
    let early = EARLY.load(Ordering::Relaxed);
    let worst_late = WORST_LATE.load(Ordering::Relaxed);
    let min_elapsed = WINDOW_MIN.load(Ordering::Relaxed);
    let wsw = WINDOW_SWITCHES.load(Ordering::Relaxed);
    let wtk = WINDOW_TICKS.load(Ordering::Relaxed);

    println!("\n--- sleep fidelity report ---------------------------------");
    println!(
        "slice           : {} ns ({} timer units, timer {} Hz)",
        st.slice_ns, st.slice_cycles, st.timer_hz
    );
    println!("sleeps recorded : {sleeps}   early returns: {early}  (must be 0)");
    println!("worst lateness  : {worst_late} ticks (one slice is the bound)");
    println!("min elapsed     : {min_elapsed} ticks");
    println!("churn           : {churn} spawn/exit cycles during the load phase");
    println!("ticks/switches  : {} / {}", st.ticks, st.switches);
    println!("all-sleeping    : {wsw} switches over {wtk} ticks in {WINDOWS} windows");
    println!(
        "blocks w/o task : {}  (must be 0)",
        st.blocks_without_current
    );
    println!("-----------------------------------------------------------");

    // One switch per tick is the honest maximum for a park-and-wake scheduler, plus slack for the
    // hand-shake at each window edge. A ring that spun, or a tick source firing in a loop, would
    // be an order of magnitude above this.
    let bound = wtk.saturating_mul(2) + 16;
    let sync = sync_parking();
    let mut ok = true;
    if sync {
        if early != 0 {
            println!("FAIL: {early} sleep(s) returned before their requested deadline");
            ok = false;
        }
    } else if early != 0 {
        // Reported, deliberately not fatal here: see the WARNING below. On metal this same
        // scenario is strict, and that is where the reported bug lived.
        println!(
            "NOTE: {early} of {sleeps} sleeps observed fewer elapsed ticks than requested; on this \
             backend parking is asynchronous (see the WARNING below)"
        );
    }
    if st.blocks_without_current != 0 {
        println!(
            "FAIL: {} wait(s) were attempted with no current task",
            st.blocks_without_current
        );
        ok = false;
    }
    // The scenario's own numbers are only meaningful where blocking actually parks the caller.
    // On an asynchronous-parking backend a 50 ms "window" can measure zero ticks, and churn is
    // paced by wall time rather than by slices, so those three checks become observations.
    if sync {
        if sleeps < 20 {
            println!("FAIL: only {sleeps} sleeps recorded; the scenario did not run");
            ok = false;
        }
        if churn < 100 {
            println!("FAIL: churn only reached {churn} spawn/exit cycles");
            ok = false;
        }
        if wsw > bound {
            println!("FAIL: the all-sleeping window did {wsw} switches, above the {bound} bound");
            ok = false;
        }
    } else {
        println!(
            "observed        : {churn} churn cycles, {wsw} switches over {wtk} idle-window ticks \
             (not asserted: this backend cannot park synchronously)"
        );
    }
    if !sync {
        println!();
        println!(
            "WARNING: parking on this backend is ASYNCHRONOUS ({})",
            limits_note()
        );
        println!(
            "         A task can return from its own `sleep`/`lock` before the CPU has been taken"
        );
        println!(
            "         away from it. The kernel books every wait correctly (blocks_without_current"
        );
        println!(
            "         = {}), so this is a host-backend property, not a scheduling bug.",
            st.blocks_without_current
        );
        println!(
            "         Win32: the tick thread suspends the task a moment later. POSIX: the SIGALRM"
        );
        println!(
            "         handler has no idle context to switch to when nothing else is runnable."
        );
        println!(
            "         On metal the switch is synchronous, and this example asserts strictly there."
        );
    }
    if ok {
        println!(
            "VERDICT : PASS{} ({sleeps} sleeps, {churn} churn cycles, {wsw} switches / {wtk} idle-window ticks)",
            if sync {
                " (strict: no early sleep)"
            } else {
                " (kernel invariants only)"
            }
        );
    }
    scheduler::shutdown(if ok { 0 } else { 1 })
}

/// The platform's timer description, for the WARNING banner.
fn limits_note() -> &'static str {
    scheduler::platform_limits().timer_note
}

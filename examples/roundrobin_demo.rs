//! Round-robin preemption demo — the exact shape from the design brief, plus
//! one observer task so the run terminates with a report.
//!
//! ```text
//! cargo run --example roundrobin_demo --features std
//! ```
//!
//! What it demonstrates:
//!   * **Task 1** — CPU-bound `loop {}`, never yields, preempted every slice.
//!   * **Task 2** — a short function that spawns **Task 3** dynamically and
//!     then returns; the trampoline unlinks it, decrements the counters and
//!     switches away immediately (no explicit exit, no yield).
//!   * **Task 3** — prints the report and shuts the kernel down.
//!   * `main` itself is task 0 and takes part in the rotation.

use rrkernel::{scheduler, thread, IdlePolicy, SchedulerConfig, Slice};
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

static T1_SPINS: AtomicU64 = AtomicU64::new(0);
static T1_SLICES: AtomicU32 = AtomicU32::new(0);
static T2_RAN: AtomicU32 = AtomicU32::new(0);
static T3_RAN: AtomicU32 = AtomicU32::new(0);
static T3_SLICES: AtomicU32 = AtomicU32::new(0);

fn main() {
    // 1. Choose the slice *before* anything runs. On this platform the
    //    achievable range is reported by `scheduler::platform_limits()`.
    let limits = scheduler::platform_limits();
    println!("platform: {}", limits.timer_note);
    println!(
        "slice limits: {} ns .. {} ns",
        limits.min_slice_ns,
        limits.max_slice_ns.unwrap_or(u64::MAX)
    );

    let cfg = SchedulerConfig {
        slice: Slice::Millis(1),
        timer_hz: 0, // host backends are time-based
        // Host-only convenience: OS-backed backends put task *stacks* on the OS
        // heap, but their TCBs and closure blobs come from the kernel arena, and
        // this demo spawns a handful of tasks. Bare metal instead uses the
        // built-in 16 KiB static arena (or passes its own region), because there
        // the arena holds the stacks too.
        stack_size: 16 * 1024,
        arena: Some(leak_arena(256 * 1024)),
        idle: IdlePolicy::Wait,
        measure: rrkernel::Measure::Nanos,
    };
    scheduler::init_with(cfg).expect("scheduler init");

    println!(
        "scheduler live: requested {} , achieved slice = {} ns",
        Slice::Millis(1),
        scheduler::slice_ns()
    );

    // 2..4 — the rest of `main` is task 0's body. Returning from this closure
    // is a task exit (unlink + immediate switch), exactly like any spawned
    // task; `main_body` keeps it from falling back into the C runtime.
    scheduler::main_body(|| {
        // Task 1: an infinite CPU-bound loop. It never yields; the kernel
        // preempts it every slice.
        thread::spawn(|| {
            let mut spins: u64 = 0;
            loop {
                // Busy for a while, then record progress. `spins` is a local,
                // so this also demonstrates that a preempted task's registers
                // and stack survive the switch untouched.
                for _ in 0..200_000 {
                    spins = spins.wrapping_add(1);
                    core::hint::spin_loop();
                }
                T1_SPINS.store(spins, Ordering::Relaxed);
                T1_SLICES.fetch_add(1, Ordering::Relaxed);
            }
        });

        // Task 2: finishes quickly and exits automatically.
        thread::spawn(task_two);

        println!("main: tasks spawned; main's body now ends and task 0 is unlinked like any other");
    });
}

fn task_two() {
    do_work();
    T2_RAN.store(1, Ordering::Relaxed);
    // Returning from here is the exit: the trampoline unlinks this task in
    // O(1), decrements `active_threads` and triggers an immediate switch.
}

fn do_work() {
    // Dynamic spawn from inside a running task.
    thread::spawn(|| {
        T3_RAN.store(1, Ordering::Relaxed);
        let t0 = Instant::now();
        let mut last = t0;
        let mut worst_slice = Duration::ZERO;

        // Observe the rotation for ~120 ms of wall clock.
        while t0.elapsed() < Duration::from_millis(120) {
            // Each pass through this loop is one slice handed to this task.
            let now = Instant::now();
            let delta = now.duration_since(last);
            if delta > worst_slice && delta < Duration::from_millis(50) {
                worst_slice = delta;
            }
            last = now;
            T3_SLICES.fetch_add(1, Ordering::Relaxed);

            // Busy-wait the rest of the slice: even with no explicit yield, the
            // kernel takes the CPU away.
            let s = Instant::now();
            while s.elapsed() < Duration::from_micros(300) {
                core::hint::spin_loop();
            }
        }

        report(worst_slice);
        // An infinite round-robin kernel has no natural end: stop it
        // explicitly. (`IdlePolicy::ExitWhenAllDead` would do it automatically
        // once every task had returned.)
        scheduler::shutdown(0);
    });
}

fn report(worst_slice: Duration) {
    let st = scheduler::stats();
    println!("\n--- report ------------------------------------------------");
    println!(
        "slice: achieved {} ns ({} timer units, timer {} Hz)",
        st.slice_ns, st.slice_cycles, st.timer_hz
    );
    println!(
        "threads: total = {}, active = {} (finished tasks unlinked + reclaimed = {})",
        st.total_threads, st.active_threads, st.reclaimed
    );
    println!(
        "ticks = {}, switches = {}, deferred ticks = {}",
        st.ticks, st.switches, st.ticks_deferred
    );
    println!(
        "observed gap between this task's slices: worst {} us (a full lap of the ring)",
        worst_slice.as_micros()
    );
    println!(
        "slice accuracy: last tick error {} ns, worst {} ns",
        st.last_period_error_ns, st.worst_period_error_ns
    );
    println!(
        "task 1: {} slices / {} spins    task 2 ran = {}    task 3 ran = {} ({} slices)",
        T1_SLICES.load(Ordering::Relaxed),
        T1_SPINS.load(Ordering::Relaxed),
        T2_RAN.load(Ordering::Relaxed),
        T3_RAN.load(Ordering::Relaxed),
        T3_SLICES.load(Ordering::Relaxed)
    );
    println!(
        "arena: {} of {} bytes carved, {} live bytes, {} allocs / {} frees",
        st.arena.bytes_bump,
        st.arena.bytes_total,
        st.arena.live_bytes,
        st.arena.allocations,
        st.arena.frees
    );
    println!("switch service time: last {} ns, worst {} ns", st.last_latency, st.worst_latency);
    print_ring();
    println!("----------------------------------------------------------\n");

    assert_eq!(T2_RAN.load(Ordering::Relaxed), 1, "task 2 never completed");
    assert_eq!(T3_RAN.load(Ordering::Relaxed), 1, "task 3 never ran");
    assert!(
        T1_SLICES.load(Ordering::Relaxed) > 5,
        "task 1 was not rotated: the tick engine is not preempting"
    );
    // Task 2 finished and unlinked itself; the ring must not still hold it.
    let mut ids = Vec::new();
    scheduler::for_each_task(|t| ids.push(t.id));
    assert_eq!(
        ids.len(),
        st.active_threads,
        "ring length disagrees with active_threads"
    );
    println!("all assertions passed: round robin + automatic unlink verified");
}

fn print_ring() {
    print!("ring: ");
    scheduler::for_each_task(|t| {
        print!(
            "[id {} {:?}{} slices={}] ",
            t.id,
            t.state,
            if t.is_current { "*" } else { "" },
            t.slices_run
        );
    });
    println!();
}

/// Give the kernel an arena that lives for the whole process.
///
/// A leaked box is the simplest way to hand a `&'static mut [u8]` to
/// `SchedulerConfig::arena` from `main`, and this really is process-lifetime
/// memory, so leaking it is the honest description.
fn leak_arena(bytes: usize) -> &'static mut [u8] {
    Box::<[u8]>::leak(vec![0u8; bytes].into_boxed_slice())
}

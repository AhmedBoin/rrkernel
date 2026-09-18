//! Slice-accuracy benchmark: **measures** the achieved round-robin timing
//! instead of asserting a number the host cannot promise.
//!
//! ```text
//! cargo run --example jitter_bench --features std --release
//! ```
//!
//! # Method
//! Each worker records `(Instant::now(), scheduler::switch_count())` as often
//! as it can. Samples taken inside the same slice have an unchanged switch
//! count; when the switch count jumps, the wall-clock delta between those two
//! samples covers exactly that many slices — a **preemption boundary**. That
//! makes the measurement independent of how fast a worker iterates, and valid
//! even with a coarse clock.
//!
//! # Reading the results
//! * **Fairness** (`slices_run` spread) is exact and needs no clock at all: in
//!   pure round robin it must be within one slice across all tasks.
//! * **Timing error** is bounded by the platform: a handful of cycles on bare
//!   metal, milliseconds on a desktop OS. On this backend the *effective*
//!   period is `slice + switch cost`, because the timer is re-armed after the
//!   switch precisely so that every task receives a full slice — that cost is
//!   reported too.

use rrkernel::{scheduler, thread, IdlePolicy, SchedulerConfig, Slice};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

const WORKERS: usize = 4;
const SLICE_MS: u64 = 1;
const OBSERVE: Duration = Duration::from_millis(1500);

static DONE: AtomicBool = AtomicBool::new(false);

/// Per-worker results: samples, worst/mean |error|, worst lap, ring length
/// observed, and slices granted by the kernel (the fairness measure).
static TABLE: [(AtomicU32, AtomicU32, AtomicU32, AtomicU32, AtomicU32, AtomicU32); WORKERS] = [const {
    (
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
        AtomicU32::new(0),
    )
}; WORKERS];

fn main() {
    let limits = scheduler::platform_limits();
    println!("platform: {}", limits.timer_note);
    println!(
        "requested slice: {SLICE_MS} ms   platform minimum: {} ns",
        limits.min_slice_ns
    );
    println!(
        "clock granularity on this machine: {} ns (floor of every measurement here)",
        clock_granularity_ns()
    );

    let cfg = SchedulerConfig {
        slice: Slice::Millis(SLICE_MS),
        timer_hz: 0,
        stack_size: 16 * 1024,
        // Host convenience: see the note in `roundrobin_demo`.
        arena: Some(Box::<[u8]>::leak(vec![0u8; 512 * 1024].into_boxed_slice())),
        idle: IdlePolicy::Wait,
        measure: rrkernel::Measure::Nanos,
    };
    scheduler::init_with(cfg).expect("scheduler init");

    // Task 0's body: spawn the load and the observer, then end — task 0 is
    // unlinked exactly like any other task.
    scheduler::main_body(|| {
        for w in 0..WORKERS {
            thread::spawn(move || worker(w));
        }
        thread::spawn(observer);
    });
}

/// Waits for the load to run, then reports and stops the kernel.
///
/// # Why this busy-waits instead of sleeping
/// On the POSIX fibre backend *all* tasks share one OS thread, so a blocking
/// call (`thread::sleep`, I/O) stalls the entire kernel — the same rule as on
/// bare metal. This observer therefore spins while checking the clock, which is
/// what a timing-critical task should do anyway.
fn observer() {
    let t0 = Instant::now();
    while t0.elapsed() < OBSERVE {
        core::hint::spin_loop();
    }
    DONE.store(true, Ordering::Relaxed);

    // Let the workers notice and finish their last iteration.
    let t1 = Instant::now();
    while t1.elapsed() < Duration::from_millis(50) {
        core::hint::spin_loop();
    }

    let elapsed = t0.elapsed();
    report(elapsed);
    scheduler::shutdown(0);
}

/// Busy worker: measures the wall-clock cost of every preemption boundary it
/// crosses. It never yields — preemption is the only thing that stops it.
fn worker(id: usize) {
    let mut last: Option<(Instant, u64)> = None;
    let mut n: u64 = 0;
    let mut sum_err: u128 = 0;
    let mut worst_err: u64 = 0;
    let mut worst_lap_ns: u64 = 0;
    let mut ring_seen: u32 = 0;

    while !DONE.load(Ordering::Relaxed) {
        let sw = scheduler::switch_count();
        let now = Instant::now();
        if let Some((prev_t, prev_sw)) = last {
            let jumped = sw.wrapping_sub(prev_sw);
            if jumped > 0 {
                let lap_ns = now.duration_since(prev_t).as_nanos() as u64;
                let expected = jumped * SLICE_MS * 1_000_000;
                let err = lap_ns.abs_diff(expected);
                // Skip the first couple of boundaries: start-up kicks the
                // scheduler out of phase on purpose.
                if n > 2 {
                    sum_err += err as u128;
                    if err > worst_err {
                        worst_err = err;
                    }
                    if lap_ns > worst_lap_ns {
                        worst_lap_ns = lap_ns;
                    }
                }
                n += 1;
                ring_seen = ring_seen.max(scheduler::active_threads() as u32);
            }
        }
        last = Some((now, sw));

        // Keep the CPU busy for a fraction of a slice. No yield anywhere.
        let spin = Instant::now();
        while spin.elapsed() < Duration::from_micros(250) {
            core::hint::spin_loop();
        }
    }

    let slot = &TABLE[id];
    slot.0.store(n as u32, Ordering::Relaxed);
    slot.1.store(worst_err.min(u32::MAX as u64) as u32, Ordering::Relaxed);
    slot.2.store(
        (sum_err / n.max(1) as u128).min(u32::MAX as u128) as u32,
        Ordering::Relaxed,
    );
    slot.3.store(worst_lap_ns.min(u32::MAX as u64) as u32, Ordering::Relaxed);
    slot.4.store(ring_seen, Ordering::Relaxed);
    slot.5.store(scheduler::current_slices_run(), Ordering::Relaxed);
}

/// Empirical clock granularity: how long until `Instant::now()` actually
/// changes. On many Windows/VM setups this is ~1 ms even though the counter
/// reports nanoseconds, and it is the floor for every number printed here.
fn clock_granularity_ns() -> u64 {
    let t0 = Instant::now();
    let mut t1 = t0;
    let mut spins = 0u64;
    while t1 == t0 && spins < 200_000_000 {
        t1 = Instant::now();
        spins += 1;
    }
    t1.duration_since(t0).as_nanos() as u64
}

fn report(elapsed: Duration) {
    let st = scheduler::stats();
    let secs = elapsed.as_secs_f64();
    println!("\n=== slice accuracy =========================================");
    println!(
        "slice: requested {} ms, achieved {} ns ({} timer units, timer {} Hz)",
        SLICE_MS, st.slice_ns, st.slice_cycles, st.timer_hz
    );
    println!(
        "run: {:.3} s, {} ticks -> {:.0} us per tick (requested {} us)",
        secs,
        st.ticks,
        1e6 * secs / st.ticks.max(1) as f64,
        SLICE_MS * 1000
    );
    println!(
        "kernel-measured tick-period error: worst {} us, last {} us",
        st.worst_period_error_ns / 1000,
        st.last_period_error_ns / 1000
    );
    println!(
        "switch service time: worst {} us, last {} us",
        st.worst_latency / 1000,
        st.last_latency / 1000
    );
    println!(
        "ticks = {}, switches = {}, deferred ticks = {}",
        st.ticks, st.switches, st.ticks_deferred
    );

    println!("per-worker, across preemption boundaries:");
    for (id, s) in TABLE.iter().enumerate() {
        let n = s.0.load(Ordering::Relaxed);
        let worst = s.1.load(Ordering::Relaxed) as u64;
        let mean = s.2.load(Ordering::Relaxed) as u64;
        let worst_lap = s.3.load(Ordering::Relaxed) as u64 / 1000;
        let ring = s.4.load(Ordering::Relaxed);
        println!(
            "  worker {id}: {n:>6} boundaries, |lap error| mean {:>5} us worst {:>5} us, \
             worst lap {worst_lap} us (ring of {ring})",
            mean / 1000,
            worst / 1000,
        );
    }

    let mut min_slices = u32::MAX;
    let mut max_slices = 0u32;
    for s in TABLE.iter() {
        let v = s.5.load(Ordering::Relaxed);
        min_slices = min_slices.min(v);
        max_slices = max_slices.max(v);
    }
    println!(
        "fairness: slices granted per worker {} .. {} (spread {}) — measured while the workers were alive",
        min_slices,
        max_slices,
        max_slices.saturating_sub(min_slices)
    );
    println!(
        "threads: total {} active {} reclaimed {}   arena {} of {} bytes, live {}",
        st.total_threads,
        st.active_threads,
        st.reclaimed,
        st.arena.bytes_bump,
        st.arena.bytes_total,
        st.arena.live_bytes
    );
    println!("============================================================\n");

    // Tolerance, not equality: the first and last laps of the run are not a
    // whole number of rotations, so a slice or two of skew is expected (and
    // kicks from task exits shift the phase). What must hold is that the spread
    // stays negligible relative to the number of slices granted.
    let spread = max_slices.saturating_sub(min_slices);
    let tolerance = (max_slices / 100).max(4);
    assert!(
        spread <= tolerance,
        "round robin is not fair: spread {spread} slices over {max_slices} (tolerance {tolerance})"
    );
    println!("round-robin fairness assertion passed (spread {spread} <= tolerance {tolerance})");
}

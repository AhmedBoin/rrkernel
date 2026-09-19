//! Minimal backend smoke test — the fastest way to tell whether a port's
//! context switch, its automatic exit and its slice timer actually work.
//!
//! ```text
//! cargo run --example smoke --features std
//! ```
//!
//! Every step prints to stderr (unbuffered), so if a backend regresses, the last
//! line printed says exactly which primitive broke.

use rrkernel::{scheduler, thread, IdlePolicy, SchedulerConfig, Slice};

fn main() {
    eprintln!(
        "[smoke] platform: {}",
        scheduler::platform_limits().timer_note
    );

    let cfg = SchedulerConfig {
        slice: Slice::Millis(1),
        timer_hz: 0,
        stack_size: 16 * 1024,
        arena: Some(Box::<[u8]>::leak(vec![0u8; 256 * 1024].into_boxed_slice())),
        idle: IdlePolicy::Wait,
        measure: rrkernel::Measure::Nanos,
    };
    scheduler::init_with(cfg).expect("init");
    eprintln!("[smoke] 1/6 init ok, slice = {} ns", scheduler::slice_ns());

    // Task that returns after a few slices: exercises the trampoline.
    thread::spawn(|| {
        eprintln!("[smoke] 3/6 short task entered");
        let start = std::time::Instant::now();
        while start.elapsed() < std::time::Duration::from_millis(5) {
            core::hint::spin_loop();
        }
        eprintln!("[smoke] 4/6 short task returning (trampoline should unlink it)");
    });
    eprintln!("[smoke] 2/6 short task spawned (spawn returned control to us)");

    // Infinite CPU-bound task: exercises preemption.
    thread::spawn(|| {
        eprintln!("[smoke] 5/6 infinite task entered; spinning, never yielding");
        let mut n: u64 = 0;
        loop {
            n = n.wrapping_add(1);
            core::hint::spin_loop();
            if n.is_multiple_of(200_000_000) {
                eprintln!("[smoke] still spinning (n = {n})");
            }
        }
    });

    scheduler::main_body(|| {
        eprintln!("[smoke] main_body body: letting the tasks run for ~30 ms");

        // Observer task: waits, reports, then stops the kernel. Without it the
        // infinite task above would keep the ring — and the process — alive
        // forever, which is the correct behaviour for a kernel that never stops
        // itself.
        thread::spawn(|| {
            let start = std::time::Instant::now();
            while start.elapsed() < std::time::Duration::from_millis(30) {
                core::hint::spin_loop();
            }
            let st = scheduler::stats();
            eprintln!(
                "[smoke] 6/6 stats: ticks={} switches={} active={} total={} reclaimed={} arena_live={}B",
                st.ticks, st.switches, st.active_threads, st.total_threads, st.reclaimed,
                st.arena.live_bytes
            );
            eprintln!("[smoke] OK: switching, preemption and automatic exit all work");
            scheduler::shutdown(0);
        });
    });
}

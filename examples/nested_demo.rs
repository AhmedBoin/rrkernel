//! A nested tree, running for real on a host backend.
//!
//! Level 1: `main`. Level 2: a group that owns its own budget. Level 3: two leaves inside
//! that group, each with its own quantum. Level 1 also keeps a plain task, so the ordinary
//! `thread::spawn` path is exercised in the same run.
//!
//! What this checks that the deterministic dispatch tests cannot: that a group can host
//! children on a real port, that its children actually get the CPU, and that
//! `schedule_next` returning "the task already running" (which happens on every tick a
//! quantum still has budget) does not upset the host switch path.
//!
//! Run: `cargo run --example nested_demo --features std`

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::thread::{self, Parent};
use rrkernel::{scheduler, Duration, IdlePolicy, SchedulerConfig, Slice};

static HEARTBEAT: AtomicU32 = AtomicU32::new(0);
static PARAMS: AtomicU32 = AtomicU32::new(0);
static PLAIN: AtomicU32 = AtomicU32::new(0);

fn main() {
    let cfg = SchedulerConfig {
        slice: Slice::Millis(1),
        timer_hz: 0,
        stack_size: 16 * 1024,
        arena: Some(Box::<[u8]>::leak(vec![0u8; 256 * 1024].into_boxed_slice())),
        idle: IdlePolicy::Wait,
        measure: rrkernel::Measure::Nanos,
    };
    scheduler::init_with(cfg).expect("scheduler init");

    let tick_ns = scheduler::slice_ns();
    println!("nested demo: tick = {tick_ns} ns (the configured slice)");

    // A group with a 4-tick visit budget, hosting two children of 2 ticks each. Children's
    // total (4) equals the group's budget here; nothing in the kernel requires that, and
    // the equal case is the easiest one to reason about for a first run.
    let telemetry = thread::spawn_group(Parent::Root, Slice::Millis(4)).expect("spawn_group");
    println!("group spawned: quantum 4 ticks, no stack, no closure");

    thread::spawn_in(Parent::Group(telemetry), Slice::Millis(2), || {
        for _ in 0..10 {
            rrkernel::sleep(Duration::from_millis(2));
            HEARTBEAT.fetch_add(1, Ordering::Relaxed);
        }
    })
    .expect("spawn_in heartbeat");

    thread::spawn_in(Parent::Group(telemetry), Slice::Millis(2), || {
        for _ in 0..10 {
            rrkernel::sleep(Duration::from_millis(3));
            PARAMS.fetch_add(1, Ordering::Relaxed);
        }
    })
    .expect("spawn_in params");

    // A plain level-1 task: unchanged API, unchanged slice.
    thread::spawn(|| {
        for _ in 0..20 {
            rrkernel::sleep(Duration::from_millis(1));
            PLAIN.fetch_add(1, Ordering::Relaxed);
        }
    });

    // A quantum shorter than the tick must be refused, not rounded.
    match thread::spawn_in(Parent::Group(telemetry), Slice::Micros(500), || {}) {
        Err(e) => println!("sub-tick quantum refused as expected: {e}"),
        Ok(_) => println!("BUG: a sub-tick quantum was accepted"),
    }

    scheduler::main_body(|| {
        rrkernel::sleep(Duration::from_millis(400));
        let st = scheduler::stats();
        println!(
            "ticks {} | switches {} | active {} | heartbeats {} | params {} | plain {}",
            st.ticks,
            st.switches,
            st.active_threads,
            HEARTBEAT.load(Ordering::Relaxed),
            PARAMS.load(Ordering::Relaxed),
            PLAIN.load(Ordering::Relaxed),
        );
        let nested_ran =
            HEARTBEAT.load(Ordering::Relaxed) > 0 && PARAMS.load(Ordering::Relaxed) > 0;
        println!(
            "VERDICT: {}",
            if nested_ran && PLAIN.load(Ordering::Relaxed) > 0 {
                "nested children and a plain task both ran"
            } else {
                "INCOMPLETE: something inside the tree did not get the CPU"
            }
        );
        // Leave explicitly: a kernel owns the process, so a returning task does not end it.
        std::process::exit(if nested_ran { 0 } else { 3 });
    });
}

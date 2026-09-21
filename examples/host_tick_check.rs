//! Does the host tick source run at all? A three-mode probe.
//!
//! ```text
//! cargo run --quiet --example host_tick_check --features std -- flat
//! cargo run --quiet --example host_tick_check --features std -- group
//! cargo run --quiet --example host_tick_check --features std -- children
//! ```
//!
//! All three modes use the **same** `SchedulerConfig` (the one `roundrobin_demo` uses, and
//! `roundrobin_demo` ticks correctly) and differ only in what is spawned: a plain task, a
//! group, or a group with two children. Each mode then sleeps three times in 100 ms steps and
//! prints `ticks` as a function of *time*, never of what was spawned.
//!
//! It exists because `nested_demo` reported `ticks 0` on Win32, which looked like a nesting
//! defect. It is not one: `flat` stalls identically, with no group anywhere in the tree, so
//! the stall belongs to the host tick source (or to this workload) rather than to groups.
//! Keeping the probe means the next person gets that answer in three runs instead of an
//! afternoon -- and gets it as data rather than as an argument.
//!
//! What a `ticks 0` result does *not* mean: the sleeps still return and `active` still counts
//! correctly, so a stalled tick source is not a hung kernel.

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::thread::{self, Parent};
use rrkernel::{scheduler, Duration, IdlePolicy, SchedulerConfig, Slice};

static CHILD: AtomicU32 = AtomicU32::new(0);
static PLAIN: AtomicU32 = AtomicU32::new(0);

fn main() {
    let mode = std::env::args()
        .nth(1)
        .unwrap_or_else(|| String::from("flat"));
    let cfg = SchedulerConfig {
        slice: Slice::Millis(1),
        timer_hz: 0,
        stack_size: 16 * 1024,
        arena: Some(Box::<[u8]>::leak(vec![0u8; 256 * 1024].into_boxed_slice())),
        idle: IdlePolicy::Wait,
        measure: rrkernel::Measure::Nanos,
    };
    scheduler::init_with(cfg).expect("init");
    println!("mode {}: tick {} ns", mode, scheduler::slice_ns());

    if mode != "flat" {
        let g = thread::spawn_group(Parent::Root, Slice::Millis(4)).expect("group");
        println!("group spawned");
        if mode != "group" {
            for _ in 0..2 {
                thread::spawn_in(Parent::Group(g), Slice::Millis(2), || loop {
                    rrkernel::sleep(Duration::from_millis(2));
                    CHILD.fetch_add(1, Ordering::Relaxed);
                })
                .expect("child");
            }
            println!("two children spawned");
        }
    }

    thread::spawn(|| loop {
        rrkernel::sleep(Duration::from_millis(1));
        PLAIN.fetch_add(1, Ordering::Relaxed);
    });

    scheduler::main_body(|| {
        for i in 0..3u64 {
            rrkernel::sleep(Duration::from_millis(100));
            let st = scheduler::stats();
            println!(
                "  {} ms: ticks {} switches {} active {} no_task_blocks {} child {} plain {}",
                (i + 1) * 100,
                st.ticks,
                st.switches,
                st.active_threads,
                st.blocks_without_current,
                CHILD.load(Ordering::Relaxed),
                PLAIN.load(Ordering::Relaxed)
            );
        }
        std::process::exit(0);
    });
}

//! One flash that says *which stage* is broken, instead of which stage is suspected.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin diag
//! ```
//!
//! Every earlier attempt to explain the hang depended on `sleep`, which conflates three
//! different failures: a dead tick source, a switch path that never hands out the CPU, and a
//! wake path that never expires a deadline. All three look like "nothing prints".
//!
//! So nothing in the tree under test sleeps. The tasks spin, and their counters can only move
//! if the tick fires *and* the switch path runs them:
//!
//! ```text
//! root
//!  +-- main          spins, prints the timeline (never blocks, so it is always scheduled back)
//!  +-- flat          spins at level 1
//!  +-- sleeper       sleeps 10 ms, counts wakes          (flat wake path)
//!  +-- grp           group, 2 ms window
//!       +-- gspin     spins inside the group
//!       +-- gsleep    sleeps 10 ms, counts wakes          (wake path one level down)
//! ```
//!
//! Reading the output:
//!
//! * `ticks` frozen at 0 -- the tick interrupt is not running at all.
//! * `ticks` rising, `switches` frozen -- ticks fire but nothing is ever switched to.
//! * `switches` rising, `flat` rising, `grp` frozen -- level 1 runs, a group child never does.
//! * `flat` rising, `sleeper` frozen -- the flat wake path is broken.
//! * `flat` rising, `gsleep` frozen -- the wake path at depth is broken.
//!
//! All of them are single numbers printed from `main`, so a hang is visible as a line that
//! simply stops changing rather than as an absence of output.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::rrkernel;
use rrkernel::thread::{self, Parent};
use rrkernel::{configure, scheduler, Duration, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;

static FLAT: AtomicU32 = AtomicU32::new(0);
static GSPIN: AtomicU32 = AtomicU32::new(0);
static SLEEPER: AtomicU32 = AtomicU32::new(0);
static GSLEEP: AtomicU32 = AtomicU32::new(0);

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(CORE_HZ, Slice::Millis(1), 1024);

    // Level 1, spinning: preemption at the top level.
    thread::spawn(|| loop {
        FLAT.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    });

    // Level 1, sleeping: the flat wake path.
    thread::spawn(|| loop {
        rrkernel::sleep(Duration::from_millis(10));
        SLEEPER.fetch_add(1, Ordering::Relaxed);
    });

    // A group with a spinning child and a sleeping child: the same two questions one level
    // down, where the dispatch has to descend through a node that owns no stack.
    let grp = thread::spawn_group(Parent::Root, Slice::Millis(2)).expect("group");
    thread::spawn_in(Parent::Group(grp), Slice::Millis(1), || loop {
        GSPIN.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    })
    .expect("gspin");
    thread::spawn_in(Parent::Group(grp), Slice::Millis(1), || loop {
        rrkernel::sleep(Duration::from_millis(10));
        GSLEEP.fetch_add(1, Ordering::Relaxed);
    })
    .expect("gsleep");

    rprintln!(
        "diag: tick {} ns, tasks spawned, main now spins",
        rrkernel::tick_ns()
    );

    // Main never blocks, so a broken wake path cannot hide here: it is always runnable and
    // always comes back. The line below is the whole experiment.
    let mut n: u32 = 0;
    loop {
        n = n.wrapping_add(1);
        if n % 40_000 == 0 {
            let st = scheduler::stats();
            rprintln!(
                "t={} ticks={} switches={} active={} flat={} grp={} sleeper={} gsleep={}",
                rrkernel::now(),
                st.ticks,
                st.switches,
                st.active_threads,
                FLAT.load(Ordering::Relaxed),
                GSPIN.load(Ordering::Relaxed),
                SLEEPER.load(Ordering::Relaxed),
                GSLEEP.load(Ordering::Relaxed)
            );
        }
        // A visible exit, never reached in practice (the counter wraps): without it the
        // loop is a diverging expression, and the code `#[rrkernel]` appends to `main`
        // becomes unreachable, which the build reports.
        if n == 0 {
            break;
        }
        core::hint::spin_loop();
    }
}

//! The upper layers on the board: park/wake, async sleeps, the ISR-driven pipe.
//!
//! ```text
//! cd examples/cortex-m-bluepill
//! cargo run --release --bin layers     # flashes, then streams RTT
//! ```
//!
//! On metal rather than on a host, this proves:
//!   * a task parked on a pipe consumes **no slices** while it waits, while a pseudo-ISR delivers the
//!     bytes through `Pipe::push_from_isr`;
//!   * `park` is woken by an **id-based** `wake` from another task — the ISR shape, and the same
//!     `wake_task` path `Signal::set` uses;
//!   * `exec::block_on` + `exec::sleep`, i.e. `.await` on a Cortex-M3 with no allocator;
//!   * the idle path still parks with all of this running.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use rrkernel::event::{park, wake, TaskId, WakeReason};
use rrkernel::io::Pipe;
use rrkernel::rrkernel;
use rrkernel::{configure, scheduler, thread, Duration, Slice};
use rtt_target::rprintln;

const CORE_HZ: u32 = 8_000_000;
const RUN_MS: u64 = 2000;

static RX: Pipe = Pipe::new();
/// The parked task publishes its id here, the way a driver stores the id of the task it wakes.
static PARKED_ID: AtomicU32 = AtomicU32::new(0);
static PUSHED: AtomicU32 = AtomicU32::new(0);
static ECHOED: AtomicU32 = AtomicU32::new(0);
static BUSY_SLICES: AtomicU32 = AtomicU32::new(0);
static WAITS: AtomicU32 = AtomicU32::new(0);
static PARK_WAKES: AtomicU32 = AtomicU32::new(0);
static ASYNC_DONE: AtomicU32 = AtomicU32::new(0);
static STOP: AtomicU32 = AtomicU32::new(0);

#[rrkernel]
#[cortex_m_rt::entry]
fn main() {
    // RTT belongs to the example, not to the kernel: the terminal is set up here, and the kernel is
    // only handed a log sink so panics still reach it. Leave these lines out on a board with no probe.
    rtt_target::rtt_init_print!();
    rrkernel::log_with(|args| rprintln!("{}", args));
    configure(CORE_HZ, Slice::Millis(1), 1024);
    rprintln!("layers: slice {} ns", rrkernel::tick_ns());

    // The pseudo-ISR: one byte every few slices, using nothing an interrupt handler could not.
    thread::spawn(|| {
        let mut i = 0u32;
        while STOP.load(Ordering::Relaxed) == 0 {
            rrkernel::sleep(Duration::from_millis(4));
            if RX.push_from_isr(b'a' + (i % 26) as u8) {
                PUSHED.fetch_add(1, Ordering::Relaxed);
            }
            i += 1;
        }
    });

    // The reader: parks on the pipe. While parked, its own slice counter must not move.
    thread::spawn(|| {
        let mut buf = [0u8; 8];
        while STOP.load(Ordering::Relaxed) == 0 {
            let before = scheduler::current_slices_run();
            if let Ok(n) = RX.read_blocking(&mut buf, Some(scheduler::deadline_after_ticks(100))) {
                ECHOED.fetch_add(n as u32, Ordering::Relaxed);
            }
            let after = scheduler::current_slices_run();
            if after > before {
                BUSY_SLICES.fetch_add(after - before, Ordering::Relaxed);
            }
            WAITS.fetch_add(1, Ordering::Relaxed);
        }
    });

    // The parked task: publishes its id, then waits with no deadline, so only a `wake` ends it.
    thread::spawn(|| {
        PARKED_ID.store(scheduler::current_id().raw(), Ordering::Release);
        while STOP.load(Ordering::Relaxed) == 0 {
            if let WakeReason::Woken = park(None) {
                PARK_WAKES.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    // The waker: wakes that task by id, which is what an ISR or a driver would hold.
    thread::spawn(|| {
        while STOP.load(Ordering::Relaxed) == 0 {
            rrkernel::sleep(Duration::from_millis(5));
            let id = PARKED_ID.load(Ordering::Acquire);
            if id != 0 {
                wake(TaskId::from_raw(id));
            }
        }
    });

    // The async task: `.await` on a machine with no allocator.
    thread::spawn(|| {
        let _ = rrkernel::exec::spawn_async(async {
            for _ in 0..5 {
                rrkernel::exec::sleep(Duration::from_millis(10)).await;
            }
            ASYNC_DONE.store(1, Ordering::Release);
        });
        while STOP.load(Ordering::Relaxed) == 0 {
            rrkernel::sleep(Duration::from_millis(50));
        }
    });

    // Task 0 runs the scenario, then measures one idle window with the workers stopped.
    let start = rrkernel::now();
    while rrkernel::now().wrapping_sub(start) < RUN_MS {
        rrkernel::sleep(Duration::from_millis(25));
    }
    STOP.store(1, Ordering::Relaxed);
    rrkernel::sleep(Duration::from_millis(50));
    let w0 = rrkernel::arch::idle_waits();
    let t0 = rrkernel::now();
    rrkernel::sleep(Duration::from_millis(100));
    let waits = rrkernel::arch::idle_waits().wrapping_sub(w0);
    report(waits, rrkernel::now().wrapping_sub(t0));
}

fn report(idle_waits: u64, idle_ticks: u64) -> ! {
    let st = scheduler::stats();
    let pushed = PUSHED.load(Ordering::Relaxed);
    let echoed = ECHOED.load(Ordering::Relaxed);
    let busy = BUSY_SLICES.load(Ordering::Relaxed);
    let w = WAITS.load(Ordering::Relaxed);
    let pw = PARK_WAKES.load(Ordering::Relaxed);
    let async_ok = ASYNC_DONE.load(Ordering::Acquire) == 1;

    rprintln!("--- layers report -----------------------------------------");
    rprintln!("pipe           : {} pushed, {} read back", pushed, echoed);
    rprintln!(
        "reader idle    : {} busy slices over {} waits (want 0)",
        busy,
        w
    );
    rprintln!("park wakes     : {}", pw);
    rprintln!(
        "async          : {}",
        if async_ok {
            "5 async sleeps done"
        } else {
            "NOT finished"
        }
    );
    rprintln!(
        "idle waits     : {}/{} ticks (1 = sleeping)",
        idle_waits,
        idle_ticks
    );
    rprintln!("ticks/switches : {} / {}", st.ticks, st.switches);
    rprintln!("switch worst   : {} cycles", st.worst_latency);
    rprintln!("blocks w/o task: {}", st.blocks_without_current);
    rprintln!("-----------------------------------------------------------");

    let ok = echoed > 50 && busy == 0 && pw > 50 && async_ok && st.blocks_without_current == 0;
    if echoed <= 50 {
        rprintln!("FAIL: only {} bytes came back", echoed);
    }
    if busy != 0 {
        rprintln!("FAIL: the parked reader consumed {} slices", busy);
    }
    if pw <= 50 {
        rprintln!("FAIL: only {} park wakes", pw);
    }
    if !async_ok {
        rprintln!("FAIL: the async task did not finish");
    }
    if st.blocks_without_current != 0 {
        rprintln!(
            "FAIL: {} waits with no current task",
            st.blocks_without_current
        );
    }
    rprintln!("VERDICT : {}", if ok { "PASS" } else { "FAIL" });
    loop {
        core::hint::spin_loop();
    }
}

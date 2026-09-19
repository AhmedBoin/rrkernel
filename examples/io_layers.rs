//! Three claims about I/O, measured rather than asserted.
//!
//! ```text
//! cargo run --example io_layers --features std,async,hal,hal-async,io,io-async
//! ```
//!
//! 1. **A task waiting on I/O consumes ~0 slices** — an echo task blocks on a `Pipe` while a
//!    pseudo-ISR delivers bytes, and its *own* slice counter is sampled while it is parked.
//! 2. **Three tasks share one bus without corrupting transfers** — a fake I2C bus counts how many
//!    transactions are in flight at once; that number must never exceed one.
//! 3. **A timeout arrives at the requested time** — a peripheral that never signals must fail within
//!    a slice of its deadline.
//!
//! Plus the layers: an async driver behind the blocking facade, an async task (`exec::block_on`), and
//! `KernelDelay`. The 1-hour soak and a physically unplugged peripheral need hardware; the bare-metal
//! `fidelity` firmware covers those on the board.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
// The I2C trait has to be in scope to call `transaction` on a device that implements it.
use embedded_hal::i2c::I2c as _;
use rrkernel::exec;
use rrkernel::hal::{KernelDelay, MutexDevice};
use rrkernel::io::Pipe;
use rrkernel::{scheduler, thread, Duration, IdlePolicy, SchedulerConfig, Slice};

// --- the shared-bus check ---------------------------------------------------

static IN_FLIGHT: AtomicU32 = AtomicU32::new(0);
static OVERLAPS: AtomicU32 = AtomicU32::new(0);
static TRANSACTIONS: AtomicU32 = AtomicU32::new(0);
static BUS_OK: AtomicU32 = AtomicU32::new(0);
static BUS_DEVICE: core::sync::atomic::AtomicPtr<MutexDevice<FakeBus, 0x4000_0001>> =
    core::sync::atomic::AtomicPtr::new(core::ptr::null_mut());

/// A fake I2C bus. One "transaction" occupies the bus for a few thousand spins, which is long enough
/// that a second task entering here would be caught by `IN_FLIGHT`.
struct FakeBus;

#[derive(Debug)]
struct FakeError;

impl embedded_hal::i2c::ErrorType for FakeBus {
    type Error = FakeError;
}

impl embedded_hal::i2c::Error for FakeError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::Other
    }
}

impl embedded_hal::i2c::I2c for FakeBus {
    fn transaction(
        &mut self,
        _address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // The corruption detector. The kernel preempts mid-transaction, so anything above one here
        // means two tasks are driving the same wires.
        if IN_FLIGHT.fetch_add(1, Ordering::AcqRel) + 1 > 1 {
            OVERLAPS.fetch_add(1, Ordering::Relaxed);
        }

        let mut n = 0u32;
        while n < 3000 {
            core::hint::spin_loop();
            n += 1;
        }
        for op in operations.iter_mut() {
            if let embedded_hal::i2c::Operation::Read(buf) = op {
                for b in buf.iter_mut() {
                    *b = 0xA5;
                }
            }
        }

        IN_FLIGHT.fetch_sub(1, Ordering::AcqRel);
        TRANSACTIONS.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// One of three tasks sharing the bus: ten transactions, and every payload verified afterwards.
fn bus_worker() -> ! {
    let dev = BUS_DEVICE.load(Ordering::Acquire);
    let mut ok = 0u32;
    if !dev.is_null() {
        let dev = unsafe { &mut *dev };
        for _ in 0..10 {
            let mut buf = [0u8; 4];
            let mut ops = [embedded_hal::i2c::Operation::Read(&mut buf)];
            if dev.transaction(0x40, &mut ops).is_ok() && buf == [0xA5; 4] {
                ok += 1;
            }
            // Yield between transactions, so preemption lands *inside* a transaction rather than only
            // between them — which is where corruption would actually happen.
            scheduler::yield_now();
        }
    }
    BUS_OK.fetch_add(ok, Ordering::Relaxed);
    loop {
        scheduler::yield_now();
    }
}

// --- the UART echo, its idle cost, and its timeout --------------------------

static RX: Pipe = Pipe::new();
/// A second pipe nobody ever writes to: the "unplugged peripheral".
static DEAD_RX: Pipe = Pipe::new();
static RX_PUSHED: AtomicU32 = AtomicU32::new(0);
static ECHO_BYTES: AtomicU32 = AtomicU32::new(0);
static BUSY_SLICES: AtomicU32 = AtomicU32::new(0);
static WAITS: AtomicU32 = AtomicU32::new(0);
static TIMEOUTS: AtomicU32 = AtomicU32::new(0);
static TIMEOUT_LATE: AtomicU64 = AtomicU64::new(0);
static ASYNC_DONE: AtomicU32 = AtomicU32::new(0);

fn main() {
    let cfg = SchedulerConfig {
        slice: Slice::Millis(1),
        timer_hz: 0,
        stack_size: 16 * 1024,
        arena: Some(Box::<[u8]>::leak(vec![0u8; 512 * 1024].into_boxed_slice())),
        idle: IdlePolicy::Wait,
        measure: rrkernel::Measure::Nanos,
    };
    if let Err(e) = scheduler::init_with(cfg) {
        eprintln!("init failed: {e}");
        std::process::exit(2);
    }

    // One bus device, shared by three tasks. Leaked deliberately: it must outlive every task, and a
    // host example has no other way to produce a `'static` device.
    let device: &'static mut MutexDevice<FakeBus, 0x4000_0001> =
        Box::leak(Box::new(MutexDevice::new(FakeBus)));
    BUS_DEVICE.store(device as *mut _, Ordering::Release);

    for _ in 0..3 {
        // A statement, so the closure returns `()` rather than the `!` of the worker.
        thread::spawn(|| {
            bus_worker();
        });
    }

    // The pseudo-ISR: delivers bytes the way an interrupt handler would (`push_from_isr`), from task
    // context so no interrupt controller is involved. Bursts with gaps, so the reader really is idle.
    thread::spawn(|| {
        let mut i = 0u32;
        loop {
            for _ in 0..8 {
                if RX.push_from_isr(b'a' + (i % 26) as u8) {
                    RX_PUSHED.fetch_add(1, Ordering::Relaxed);
                }
                i += 1;
            }
            rrkernel::sleep(Duration::from_millis(20));
        }
    });

    // The echo task: blocks on RX. While parked its slice counter must not move — claim 1, measured
    // from the task that would otherwise be burning CPU to poll.
    thread::spawn(|| {
        let mut buf = [0u8; 16];
        loop {
            let before = scheduler::current_slices_run();
            if let Ok(n) = RX.read_blocking(&mut buf, Some(scheduler::deadline_after_ticks(50))) {
                ECHO_BYTES.fetch_add(n as u32, Ordering::Relaxed);
            }
            let after = scheduler::current_slices_run();
            if after > before {
                BUSY_SLICES.fetch_add(after - before, Ordering::Relaxed);
            }
            WAITS.fetch_add(1, Ordering::Relaxed);
        }
    });

    // Claim 3: a read from a pipe that never gets data must fail at the deadline, within a slice.
    thread::spawn(|| {
        let mut buf = [0u8; 4];
        loop {
            let started = scheduler::tick_count();
            let want = started.wrapping_add(3);
            if DEAD_RX.read_blocking(&mut buf, Some(want)).is_err() {
                let late = scheduler::tick_count().saturating_sub(want);
                let _ = TIMEOUT_LATE.fetch_max(late, Ordering::Relaxed);
                TIMEOUTS.fetch_add(1, Ordering::Relaxed);
            }
            rrkernel::sleep(Duration::from_millis(5));
        }
    });

    // The async task: `exec::sleep` (a `.await` that gives up the slice) plus the delay adapter.
    thread::spawn(|| {
        let _ = exec::spawn_async(async {
            let mut delay = KernelDelay::new(100_000);
            for _ in 0..5 {
                exec::sleep(Duration::from_millis(10)).await;
                // Short delay: spins on the cycle counter, because a switch would cost more.
                embedded_hal_async::delay::DelayNs::delay_us(&mut delay, 50).await;
            }
            ASYNC_DONE.store(1, Ordering::Release);
        });
        loop {
            rrkernel::sleep(Duration::from_millis(50));
        }
    });

    scheduler::main_body(|| {
        // Fail fast with the diagnosis rather than hanging: on the host backends the tick source
        // has been observed not to run at all in a task set like this one (`ticks == 0` from the
        // first moment, on Windows *and* Linux), which stalls every deadline-based path — sleep,
        // timeouts, async timers. The blocking-I/O paths above are unaffected by it, and the metal
        // port ticks normally (the `fidelity` firmware measures thousands of ticks).
        if scheduler::tick_count() == 0 {
            println!(
                "DIAGNOSIS: ticks == 0 with {} tasks live - the host tick source is not running.",
                scheduler::active_threads()
            );
            println!(
                "           Deadline-based waits cannot make progress here (see docs/DESIGN.md 8.6)."
            );
            scheduler::shutdown(3);
        }
        println!(
            "diag: after init ticks={} active={} slice_ns={}",
            scheduler::tick_count(),
            scheduler::active_threads(),
            scheduler::slice_ns()
        );
        rrkernel::sleep(Duration::from_millis(200));
        println!(
            "diag: after a 200 ms sleep ticks={} active={}",
            scheduler::tick_count(),
            scheduler::active_threads()
        );
        rrkernel::sleep(Duration::from_millis(1000));
        println!("diag: after 1000 ms more ticks={}", scheduler::tick_count());
        report();
    });
}

fn report() -> ! {
    let st = scheduler::stats();
    let tx = TRANSACTIONS.load(Ordering::Relaxed);
    let overlaps = OVERLAPS.load(Ordering::Relaxed);
    let bus_ok = BUS_OK.load(Ordering::Relaxed);
    let pushed = RX_PUSHED.load(Ordering::Relaxed);
    let echo = ECHO_BYTES.load(Ordering::Relaxed);
    let busy = BUSY_SLICES.load(Ordering::Relaxed);
    let waits = WAITS.load(Ordering::Relaxed);
    let timeouts = TIMEOUTS.load(Ordering::Relaxed);
    let late = TIMEOUT_LATE.load(Ordering::Relaxed);
    let async_ok = ASYNC_DONE.load(Ordering::Acquire) == 1;

    println!("\n--- io layers report --------------------------------------");
    println!("bus            : {tx} transactions, {overlaps} overlaps, {bus_ok} verified payloads");
    println!("uart           : {pushed} bytes pushed by the 'isr', {echo} echoed back");
    println!("reader idle    : {busy} busy slices over {waits} waits (must be 0)");
    println!("timeouts       : {timeouts}, worst lateness {late} ticks (bound: 1)");
    println!(
        "async          : {}",
        if async_ok {
            "5 async sleeps + 5 async delays"
        } else {
            "NOT finished"
        }
    );
    println!("ticks/switches : {} / {}", st.ticks, st.switches);
    println!("blocks w/o task: {}", st.blocks_without_current);
    println!("-----------------------------------------------------------");

    let mut ok = true;
    if overlaps != 0 {
        println!("FAIL: {overlaps} overlapping transactions - two tasks drove one bus at once");
        ok = false;
    }
    if tx != 30 || bus_ok != 30 {
        println!("FAIL: {tx} transactions, {bus_ok} verified payloads (want 30 and 30)");
        ok = false;
    }
    if echo < 32 {
        println!("FAIL: only {echo} bytes came back through the pipe");
        ok = false;
    }
    if busy != 0 {
        println!("FAIL: the blocked reader consumed {busy} slices (want 0)");
        ok = false;
    }
    if timeouts == 0 || late > 1 {
        println!("FAIL: {timeouts} timeouts, worst lateness {late} ticks");
        ok = false;
    }
    if !async_ok {
        println!("FAIL: the async task did not finish");
        ok = false;
    }
    if st.blocks_without_current != 0 {
        println!(
            "FAIL: {} waits with no current task",
            st.blocks_without_current
        );
        ok = false;
    }
    if ok {
        println!(
            "VERDICT : PASS  ({tx} shared-bus transactions, {echo} bytes echoed, 0 busy slices, {timeouts} timeouts within {late} ticks)"
        );
    }
    scheduler::shutdown(if ok { 0 } else { 1 })
}

# rrkernel

A small preemptive round-robin RTOS kernel, written in Rust. `no_std`, no allocator, no priorities every task gets an equal, cycle-exact time slice, and that's the whole scheduling model.

Same kernel runs on bare metal (Cortex-M, RISC-V32, ARM A/R) and on desktop (Windows/Linux), with basically the same API on both.

```rust
#![no_std]
#![no_main]

use rrkernel::{configure, thread, Slice};

#[rrkernel(log = rtt)]
#[cortex_m_rt::entry]
fn main() {
    configure(8_000_000, Slice::Millis(1), 1024); // clock, slice, stack size

    thread::spawn(blink);
    thread::spawn(report);

    loop {
        rrkernel::sleep_secs(1);
    }
}
```

`thread::spawn` takes a closure, just like `std::thread::spawn`. No `unsafe` in application code.

## Why round robin, no priorities

Most Rust RTOS options today are priority-based (RTIC, Embassy) or bindings to a priority-based C kernel (FreeRTOS, Zephyr). Priorities are great until a low-priority task gets starved by higher-priority ones and once you have more than a couple of tasks, reasoning about "will this actually run in time" means reasoning about the whole priority graph.

rrkernel skips that. Every task has the same priority. The only knob is the slice length, and it's a hardware timer count, not "about a millisecond." That buys two things:

- **No starvation.** A task can't be locked out by "more important" work, because there is no more important work.
- **Timing you can actually reason about.** The slice is exact and doesn't drift, so worst-case latency is just `(number of tasks - 1) × slice`, full stop.

Measured on an STM32F103 @ 8MHz, 1ms slice, with a CPU-bound task spinning forever and never yielding:

```
3 periodic tasks (100ms / 250ms / 500ms) 0ms lateness, every time
switch cost: ~13.5µs
```

That's the whole pitch: a spinning task can't monopolize the CPU, and the tasks that need to run on time, do.

## Nested round robin (splitting one slice)

Sometimes one task wants to share its slice with a few jobs of its own: a control loop that gets
2ms, split between reading a sensor and running a filter. Instead of adding a second timer, make
that task a **group**, which hosts its own round robin. Groups nest as deep as you like, and a
group costs one TCB and nothing else - no stack, no thread, no fiber.

```rust
let control = thread::spawn_group(Parent::Root, Slice::Millis(2))?;  // a 2ms slot

thread::spawn_in(Parent::Group(control), Slice::Millis(1), read_sensor)?;
thread::spawn_in(Parent::Group(control), Slice::Millis(1), run_filter)?;

thread::spawn(telemetry);   // unchanged: a plain task, on the slice configure() set
```

The simple path doesn't move. `thread::spawn` is still the same call, and a plain task is just a
leaf at the top level.

**What it buys.** The group gets a bounded slot - 2ms each time its turn comes round - and inside
it the children share that time on their own terms. Both directions work: children that add up to
less than the slot start another lap until it is used up, and children that add up to more are cut
off mid-child and carry on next visit.

**How the timing works.** Still exactly one hardware timer, at the finest slice used anywhere in
the tree. Every tick decrements the quantum of the running path, and the outermost node whose
quantum ran out decides where the turn goes next.

The part worth internalising: from inside a group, its own slices read as one continuous
timeline. A child given 1ms that got 300us of it and comes back two visits later resumes with the
700us it had left, not with a fresh 1ms. The gaps where the group itself wasn't scheduled don't
exist from the children's point of view, so a nested round robin can be written as if it owned the
CPU - which is the whole point of nesting.

Sleeps are the deliberate exception: `sleep(5ms)` is measured against the one global tick
counter, so it wakes 5ms later in real time however often the group holding it actually ran.

On the F103, a 2ms group sharing the top level with a 1ms flat task splits the CPU close to 2:1 -
456k against 235k loop iterations over the same run.

## What's solid vs. what's not

- Cortex-M (M0/M0+/M3/M4/M7/M33): **verified on real hardware** (STM32F103 Blue Pill)
- RISC-V32 and ARM A/R: **verified in QEMU**
- Windows/Linux: works, used for desktop dev/testing
- ESP32/Xtensa: boots, context switch has a known resume bug not working yet
- Multi-core: the hooks exist, but no port actually starts a second core yet
- Nested round robin: **verified on hardware** for a group of tasks sharing the ring (STM32F103); deeper trees and sleeping leaves inside a group are tested on the host, not on a board yet
- One known bug: `sleep` occasionally returns early (~1 in 15 calls) right after another task is spawned or exits. Documented in the code, not fixed yet.

Full flash demo (kernel + 3 tasks) is under 6KB.

## Try it

```bash
cargo test --features std
cargo run --example roundrobin_demo --features std --release

# on a board
rustup target add thumbv7m-none-eabi
cd examples/cortex-m-bluepill && cargo run
```

## License

MIT. Use it wherever, PRs welcome especially if you can run it on hardware I don't have.

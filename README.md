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

### A group slice is one timeline

Think of a group as a worker who may use the workshop for 2ms and then has to leave. The worker
does not care how long the workshop stays closed. The rule is simple: finish the current job,
take the next one, and when the 2ms are over, stop where you are and continue from that exact
point next time.

A 2ms group holding two tasks, one asking for 1ms and one for 0.5ms (`th1`, `th2`):

```text
   the group turn, 2ms                                the group turn again, 2ms
   +-------------------------------+            +-------------------------------+
   | th1 1ms   | th2 0.5ms | th1   |    ...     | th1 0.5ms  | th2 0.5ms | th1  |
   |           |           | 0.5ms |            | (leftover) |           | 1ms  |
   +-------------------------------+            +-------------------------------+
                                          ^
                        th1 was cut here with 0.5ms still left, so the
                        next turn starts by giving that 0.5ms back to th1
```

Two things to see:

* every task gets exactly the time it asked for: `th1` gets 1ms, `th2` gets 0.5ms;
* a cut task is never restarted. The time it did not use is kept for it, and it runs first next
  time.

The same holds when the tasks inside need more time than one slot. A 3ms group over tasks of
1.5ms, 2ms and 4ms:

```text
   slot 1:  | th1 1.5ms | th2 1.5ms |
   slot 2:              | th2 0.5ms | th3 2.5ms |
   slot 3:                          | th3 1.5ms | th1 1.5ms |
```

Each line starts where the line before it stopped. `th2` was cut with 0.5ms left, so it opens the
next slot. `th3` was cut with 1.5ms left, so it opens the slot after that. Nobody is skipped and
nobody is served twice before the others: the order only moves forward.

### Groups inside groups

From the point of view of the group above it, a group is just another task. So the same rule
applies one level down, and again at any depth:

```text
   outer group turn, 4ms
   +---------------------------------------------------------------------+
   | th1 1ms | inner group 3ms                                          |
   +---------------------------------------------------------------------+
                   |
                   +---  inside the inner group, on its own timeline:
                         th a 2ms | th b 1ms
                         (and its next turn starts from where it stopped)
```

```rust
// 4ms of the round robin goes to this part of the system.
let control = thread::spawn_group(Parent::Root, Slice::Millis(4))?;

// 1ms of it to a plain task...
thread::spawn_in(Parent::Group(control), Slice::Millis(1), read_sensor)?;

// ...and 3ms to a group of its own, which splits that again.
let filters = thread::spawn_group(Parent::Group(control), Slice::Millis(3))?;
thread::spawn_in(Parent::Group(filters), Slice::Millis(2), low_pass)?;
thread::spawn_in(Parent::Group(filters), Slice::Millis(1), notch)?;

// Everything else is untouched.
thread::spawn(telemetry);
```

Each extra level costs one small control block and no stack, and each level only ever sees the
level directly below it.

### Before you choose your times

Every time you ask for must be a whole number of ticks, and the tick is the slice you gave to
`configure`. With a 1ms tick, 0.5ms does not exist. With `configure(..., Slice::Micros(500), ...)`
the tick is 0.5ms, and then 0.5ms, 1ms, 1.5ms and 2ms all work. A time the kernel cannot honour is
refused with the numbers in the message, never rounded, so you never get a slice you did not ask
for.

One honest limit: how much time each task gets is exact, but *when* a group gets its slot depends
on the level above it - a group can wait as long as the slots of its neighbours take. That is the
same rule plain tasks have at the top level.

Measured on the F103 with a 0.5ms tick: a 2ms group over tasks of 1ms and 0.5ms gave exactly
`2 ticks, 1 tick, 2 ticks, 1 tick` for twelve laps, and no turn was ever longer than the time its
task asked for.

## What's solid vs. what's not

- Cortex-M (M0/M0+/M3/M4/M7/M33): **verified on real hardware** (STM32F103 Blue Pill)
- RISC-V32 and ARM A/R: **verified in QEMU**
- Windows/Linux: works, used for desktop dev/testing
- ESP32/Xtensa: boots, context switch has a known resume bug not working yet
- Multi-core: the hooks exist, but no port actually starts a second core yet
- Nested round robin: **verified on hardware** for a group of tasks sharing the ring (STM32F103); deeper trees and sleeping leaves inside a group are tested on the host, not on a board yet
- Sleeps: measured on the F103 - 979 wakes in 9.8 seconds at a 10ms period, one per period, and a periodic task reporting `late by 0 ms` for eleven seconds straight

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

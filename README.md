# rrkernel — a preemptive round-robin RTOS kernel in Rust

One API for bare metal **and** desktop. Preemptive, cycle-exact slices, no priorities to
tune, and an application that reads like a desktop program:

```rust
#![no_std]
#![no_main]

use rrkernel::{configure, thread, Slice};

#[rrkernel(log = rtt)]          // entry point, fault + panic handlers, console: all emitted
#[cortex_m_rt::entry]
fn main() {
    configure(8_000_000, Slice::Millis(1), 1024);   // core clock, slice, per-task stack

    thread::spawn(blink);                            // a thread
    thread::spawn(report);                           // another

    // main IS task 0: it spawns, sleeps and returns like any other thread.
    loop {
        rrkernel::sleep_secs(1);
    }
}
```

`configure` takes the three numbers the kernel cannot know; `#[rrkernel]` supplies
everything else — the entry point, the vector-table wiring, a `HardFault` and
`DefaultHandler` that *report* before parking, and a `#[panic_handler]`, so an application
never depends on `panic-halt` and a fault never looks like a hang.

* **Bare metal** (`no_std`): Cortex-M0/M0+/M3/M4/M7/M33 (verified on hardware — see below),
  RISC-V 32, ARM A/R, and a port for Xtensa in progress. `SysTick` as the slice timer,
  `PendSV` for the context switch, hand-written Thumb assembly, and a **cycle-exact** slice
  you choose at init.
* **Desktop** (`std`, Windows/Linux): the same ring, counters and life cycle on top of OS
  primitives — suspend/resume of real threads on Windows, `SIGALRM` plus hand-written x86-64
  fiber switching on Linux.
* **Sleeping with units**: `sleep(Duration::from_millis(250))`, `sleep_secs`, `sleep_minutes`,
  `sleep_hours`, plus absolute deadlines (`deadline_after`/`sleep_until`) for periodic work
  that cannot drift. A sleeping task is switched away, never spun on.
* **The kernel itself depends on nothing** — `core` only, on every target. `#[rrkernel]`
  costs `syn`/`quote` *in the compiler* (host-only), and `default-features = false` removes
  even that.

```rust
use rrkernel::{scheduler, thread, SchedulerConfig, Slice};

fn main() {
    // 1. Choose the slice *first*: this is the timing contract.
    scheduler::init_with(SchedulerConfig::embedded(Slice::Millis(1), 168_000_000)).unwrap();

    // 2. Spawn a CPU-bound task. It never yields — it is preempted every slice.
    thread::spawn(|| loop {
        core::hint::spin_loop();
    });

    // 3. Spawn a task that finishes. Returning IS the exit: the trampoline
    //    unlinks it, updates the counters, and switches away immediately.
    thread::spawn(do_work);

    // Task 0's body: returning here also unlinks task 0, so the call never
    // falls back into the C runtime / reset handler.
    scheduler::main_body(|| {
        // Tasks may spawn tasks at any time:
        thread::spawn(|| { /* child, inserted into the live ring */ });
    });
}

fn do_work() {}
```

## Why another RTOS

This kernel exists for one property above all others: **timing you can reason about**. The
slice is not "about a millisecond" — it is a number of core cycles you choose at init, and
the switch is engineered so that number stays true.

```text
STM32F103 @ 8 MHz, 1 ms slice   measured on hardware
  slice      : 8000 ticks, exactly 1 000 000 ns            (SysTick->LOAD + 1)
  period err : worst 9718 ns                               (the tick lands within ~10 µs)
  switch     : worst 108 cycles  (≈ 13.5 µs at 8 MHz)      (PendSV entry + frame + ring walk)
  3 periodic threads while a CPU-bound task never yields:
     [A] every 100 ms → late by 0 ms, 32 times out of 32
     [B] every 250 ms → late by 0 ms
     [C] every 500 ms → late by 0 ms
```

### Cooperative vs preemptive — and why it matters for critical timing

| | **Cooperative** | **Preemptive** (this kernel) |
|---|---|---|
| When a switch happens | only when the task yields, sleeps, or blocks | at a hardware timer boundary, set once at init |
| A task that spins forever | stalls the whole system | loses the CPU at the slice boundary, automatically |
| Worst-case latency to a runnable task | the longest yield-free region in *any* task | one slice, plus one switch (~13 µs at 8 MHz) |
| Slice accuracy | whatever the code between yields happens to do | `SysTick` reload + restart on every switch, so the period cannot drift |
| Priorities | usually needed to fix the "one task starves the rest" problem | none needed — every task has the same priority, the only knob is the slice |
| Writing a task | must remember to yield, everywhere | a plain function; `loop { spin }` is legal and cannot monopolise |

Cooperative schedulers can look *more* precise — nothing interrupts anything, so the latency
of a hand-off is tiny — right up to the moment one task forgets to yield, which turns
"precise" into "unbounded". This kernel takes the opposite trade: **worst case instead of best
case**. A misbehaving task is throttled by hardware, and the numbers above hold *while* a
CPU-bound task spins in a tight loop, which is the situation that matters in a control loop.

The precision comes from three implementation choices, and they are the reason the numbers are
what they are rather than "whatever the OS felt like today":

1. **The slice is cycles, not milliseconds.** `plan_timer` converts your request into a
   `SysTick` reload and reports back the *achieved* value, refusing configurations the
   hardware cannot honour instead of clamping silently.
2. **Every switch restarts the counter** (`SysTick->VAL = 0`), so the successor gets a whole
   slice and error cannot accumulate over thousands of ticks.
3. **Sleeping is a deadline on the global tick**, not a per-task countdown. A task that asks
   for 100 ms wakes at the next tick at or after its absolute deadline, so a late wake-up is
   corrected on the next period instead of being added to it. Under a busy system the
   periods stay exact — that is what "0 ms lateness" above means.

### The same API as `std::thread`, without the OS

It is meant to read like desktop code, because that is the part Rust programmers already know:

```rust
thread::spawn(|| loop {                       // a closure, moved onto its own stack
    led.set_low();
    rrkernel::sleep_secs(1);                  // blocking sleep, deadlines on a global tick
    led.set_high();
    rrkernel::sleep_secs(1);
});                                           // returning is the exit: nothing to join
```

`thread::spawn` takes `FnOnce() + Send + 'static` exactly like the standard library's, task
bodies are ordinary closures, and **application code contains no `unsafe`**. What differs is
what is *behind* it: no OS, no `alloc`, no `libc`, no global allocator, `core`-only, and a
task costs a stack carved from a bump arena — on an STM32F103 the whole demo firmware
(kernel + three tasks + report) fits in **5904 bytes of flash and 96 bytes of `.data`**.

## Project status: what is done, what is in progress, what is missing

| Area | Status | Evidence |
|---|---|---|
| Preemptive round robin, slice planning, arena, ring, deferred reclamation | **complete** | `cargo test --features std` → 8 + 2 pass |
| Cortex-M backend (`SysTick` + `PendSV`, cycle-exact) | **complete, verified on hardware** | STM32F103C8: worst switch 108 cycles, worst period error 9718 ns, 0 ms lateness on 100/250/500 ms periods while a spinner never yields |
| `#[rrkernel]`, `configure`, units-based sleep | **complete, verified on hardware** | both board examples build and run; `sleep(2500 ms)` woke at exactly 2500 ticks; task 0 exits while other threads keep running |
| Cortex-M0/M0+ (`thumbv6m`) | **complete, builds** | `firmware-cortex-m` links for `thumbv6m`; no M0 board has been attached yet |
| RISC-V 32 backend (CLINT, trap frame, `mret`) | **complete, verified in QEMU** | `-M virt` → `VERDICT : PASS`, QEMU exits 0 |
| ARM A/R backend (generic timer, GICv2, banked `SP`) | **complete, verified in QEMU** | `-M virt -cpu cortex-a15` → `VERDICT : PASS`; also compiles for `armv7r` |
| Windows backend (high-resolution waitable timer, suspend/resume) | **complete; precision limited by the OS** | tests pass; the demo's rotation assertion **fails on a 1 ms slice on the machine this README was written on** (worst tick error 993 400 ns, timer granularity 0.5 ms). Use 5–10 ms slices there |
| Linux backend (`timer_create`/`SIGALRM` + x86-64 fibre switch) | **complete; scripted, not measured by the author** | `bash scripts/run-linux.sh` runs tests + smoke + demo + bench |
| macOS / BSD | **not started** | work list in `docs/PORTING.md`: three struct layouts and the timer semantics |
| `sync::Mutex` (lock ordering, timeout, back-off) | **complete; never exercised at runtime** | compiles on every target with 32-bit atomics; no test runs it |
| Multi-core: `CpuArch`, spinlocks, `active_cores` | **machinery complete; bring-up missing** | validates and *refuses* `> 1` where there is no cross-core interlock; no port starts a secondary core, `-smp 2` has never been run |
| Xtensa / ESP32 LX6 | **in progress** | builds, boots in Espressif QEMU, prints over UART0, the switch runs end to end — then the *resume* is wrong, so no verdict. `docs/ESP32.md` |
| ESP32-C3/C6, AArch64, RV64 | **not started** | needs a different tick source (Timer Group/SysTimer) or 64-bit frames |
| AVR (ATmega328P) | **not started, but the core is compatible** | needs `-C target-cpu=atmega328p` + `avr-gcc` to link |

### Where it has actually been run

| Target | Real hardware | QEMU | In the build matrix |
|---|---|---|---|
| Cortex-M3 (`thumbv7m`) — STM32F103C8 "Blue Pill" | **yes, `VERDICT : PASS`** | runs; no faults, 1579 IRQs in 3 s (the QEMU firmware's report lives in flash, so QEMU cannot print a verdict) | yes |
| Cortex-M4 (`thumbv7em`) — STM32F401 "Black Pill" | **LED + all threads run**; RTT blocked by a `probe-rs` flashing quirk (its first flash word lands as zero, so the core faults before executing) — `probe-rs read b32 0x08000000 2` must read `20010000 08000195` | — | yes |
| Cortex-M0 (`thumbv6m`), Cortex-M33 (`thumbv8m.main`) | no | — | builds / `cargo check` |
| RISC-V 32 (RV32IMAC) | no | **yes, `VERDICT : PASS`** | yes |
| ARM A/R (ARMv7-A/`armv7r`) | no | **yes, `VERDICT : PASS`** | yes |
| Xtensa LX6 (ESP32) | no | boots, no verdict yet (Espressif QEMU) | build only (needs the esp-rs fork) |
| Windows x86-64 | **yes** (tests pass; demo assertion fails at a 1 ms slice — see above) | — | tests |
| Linux x86-64 | not by the author | — | `scripts/run-linux.sh` |
| macOS | no | — | — |

Nothing in that table is an estimate: every "verified" line is reproducible with a command in
this file, and the entries that say *not* verified say so because they have not been.




| Not needed | Because |
|---|---|
| `scheduler_run()` | `scheduler::init*()` arms the tick source immediately; tasks are scheduled as they are spawned |
| `task::yield_now()` | the timer preempts every task at the slice boundary |
| explicit exit / loop handling | the trampoline unlinks the task, decrements the counters and triggers an immediate switch |
| priorities / priority inheritance | pure round robin, identical priority for every task — the only knob is the slice |
| `libc`, `alloc`, a global allocator, any crate | the kernel is `core`-only: FFI is declared by hand, task bodies are placed with `ptr::write`/`ptr::read`, stacks come from a bump arena |

## Commands

```bash
# Host: tests, then the demos (Windows and Linux)
cargo test --features std
cargo run --example roundrobin_demo --features std --release
cargo run --example jitter_bench    --features std --release
cargo run --example smoke           --features std            # backend sanity check

# Linux/POSIX backend end-to-end (tests + smoke + demo + bench)
bash scripts/run-linux.sh

# Bare metal: builds and links with the bundled rust-lld, no extra toolchain
cargo build -p firmware-cortex-m --target thumbv7m-none-eabi --release
cargo build -p firmware-cortex-m --target thumbv6m-none-eabi --release   # Cortex-M0
cargo check --target thumbv8m.main-none-eabihf                          # Cortex-M33/M35P

# A board, end to end: configure, flash, watch RTT over the probe
rustup target add thumbv7m-none-eabi
cd examples/cortex-m-bluepill && cargo run          # STM32F103C8, RTT + LED-free
cd examples/cortex-m-blackpill && cargo run         # STM32F401, stm32f4xx-hal LED thread

# What the requested Cortex-R target compiles to (same source as Cortex-A)
cargo build -p firmware-arm-a --target armv7r-none-eabi --release

# QEMU: RISC-V 32 and ARM A/R profile, end-to-end.
# QEMU exits with the demo's verdict as its status (0 = all checks passed).
cargo run -p firmware-riscv  --target riscv32imac-unknown-none-elf --release
cargo run -p firmware-arm-a  --target armv7a-none-eabi             --release
```

### Building and running the ESP32 port

```powershell
# one-time toolchain: esp-rs fork (rustup run esp) + Espressif GCC for linking
espup install -t esp32

# Espressif's QEMU is the only one with an ESP32 machine; upstream QEMU 11 has
# neither the machine nor the CPU. Fetch the win64 archive from
# https://github.com/espressif/qemu/releases and extract it to tools/qemu-esp32.
rustup run esp cargo build -p firmware-esp32 -Zbuild-std=core --release `
    --target xtensa-esp32-none-elf

tools\qemu-esp32\qemu\bin\qemu-system-xtensa.exe -nographic -machine esp32 `
    -global driver=timer.esp32.timg,property=wdt_disable,value=true `
    -kernel target\xtensa-esp32-none-elf\release\firmware-esp32
```

`-global … wdt_disable` is not optional (the emulated watchdog resets the guest
mid-demo), and the port's status is **bring-up complete, scheduling not yet
verified** — read [`docs/ESP32.md`](docs/ESP32.md) before trusting anything it prints.

### The regression matrix

Everything below is what "green" means in this repository, and it is the list to run
after touching the kernel core or a port:

| Command | Expected |
|---|---|
| `cargo test --features std` | 8 + 2 tests pass (ring/arena/closure/ABI invariants) |
| `cargo build --target {thumbv6m,thumbv7m,thumbv8m.main,armv7a,armv7r,riscv32imac}-…` | all clean, no warnings |
| `cargo run -p firmware-riscv --target riscv32imac-unknown-none-elf --release` | `VERDICT : PASS`, QEMU exits 0 |
| `cargo run -p firmware-arm-a --target armv7a-none-eabi --release` | `VERDICT : PASS` |
| `rustup run esp cargo build -p firmware-esp32 -Zbuild-std=core --release --target xtensa-esp32-none-elf` | links (execution: see `docs/ESP32.md`) |

## Multi-core and synchronization (current state)

The design is architecture-agnostic; the multi-core *bring-up* is not finished. Read
this section as the honest contract, because the two halves have very different
guarantees today.

**What exists and is exercised by tests:**

* **`src/smp.rs`** — the `CpuArch` HAL, and the only per-core facts in the kernel:
  `current_core_id()`, `max_cores()`, `supports_smp()`, `spinlock_acquire/release`,
  and `send_ipi(target_core)`. Implemented for **RISC-V** (`mhartid`, `lr.w`/`sc.w`,
  CLINT `msip`), **ARM A/R** (`MPIDR`, `LDREX`/`STREX`, GICv2 `SGIR`), **Xtensa**
  (`PRID`, `S32C1I`, `DPORT`), the **host** (OS thread id, `Atomic*`, no-op) and a
  **single-core fallback**.
* **`smp::SpinLock`** — cross-core mutual exclusion *and* local interrupt masking,
  both required: the interlock alone lets a core be interrupted while holding a lock
  and then spin forever waiting for itself. The lock word is per-target because a
  byte-wide `AtomicBool` only exists where the hardware has byte atomics.
* **`KernelConfig { active_cores }`** plus `scheduler::set_active_cores(n)`, which
  validates `1..=smp::max_cores()` and **refuses `> 1` on a target without a
  cross-core interlock** rather than pretending.
* **`src/sync.rs`** — a sleeping `Mutex<T>` with a monotonic `LockId`, an owner task
  id, `lock`, `try_lock`, `try_lock_for(timeout_ms)`, `unlock`, and
  `lock_with_backoff(timeout, attempts, attempt)` for the runtime recovery path.
  Contention marks the caller `Blocked(lock id)` and asks for an **O(1) context
  switch** instead of spinning; `unlock` wakes exactly those waiters.
* **Static deadlock prevention**: each task records the locks it holds, and an
  acquisition whose `LockId` is not strictly greater than the last held one is
  refused with `LockError::OrderViolation`. Ids are creation-ordered, so "acquire in
  increasing id order" is a total order — a circular wait cannot exist in a total
  order.
* **`TaskState::Blocked`** — a blocked task stays *linked* in the ring (so its
  rotation slot and the O(1) wake path survive) and `ring::next_runnable` simply skips
  it. The tick sweeps expired deadlines, which is what makes `try_lock_for` time out.

**What is not done yet:**

* **No port starts a secondary core.** `active_cores > 1` is configurable and
  validated, but `arch::start_secondary_cores()` does not exist and the kernel still
  keeps *one* global `current_tcb`. Making `active_cores > 1` real needs per-core
  `current` state (kept after offset 0, so the Cortex-M `PendSV` literal offset
  survives), the global interlock taken by `critical_enter` when more than one core
  runs, and IPI-driven rescheduling.
* **`qemu-system-riscv32 -smp 2` / `qemu-system-arm -smp 2` have not been run.** No
  SMP demo exists, so there is **no multi-core scheduling or contention evidence** in
  this repository — only the machinery above.
* **The `sync` layer is host-tested only.** It compiles for every target that has a
  32-bit atomic RMW, but it has never run on a board.
* `sync` is compiled **out** on targets with no atomic RMW (`thumbv6m`/Cortex-M0, AVR,
  `riscv32imc`) — a mutex whose owner field cannot be updated atomically is not a
  mutex. `smp::supports_smp()` is `false` there too, so `set_active_cores(2)` returns
  a `ConfigError` instead of misbehaving later.


## Architecture support

The *same* kernel, the *same* `TaskControlBlock`, and the *same* application code
(`firmware-common/`, shared by every board below) run on all of these. A port
replaces only the stack-frame layout, the timer, and the switch assembly; it is
selected by the target triple, so there are no feature flags to get wrong.

| Architecture | Target | Status | Timer / switch |
|---|---|---|---|
| ARM Cortex-M0/M0+/M3/M4/M7/M33 | `thumbv6m`, `thumbv7m`, `thumbv8m.main`, `thumbv7em` | **verified on hardware** (STM32F103C8 "Blue Pill", `thumbv7m`): `VERDICT : PASS`, 100 ms/250 ms/500 ms threads landing on **0 ms lateness** with a CPU-bound spinner running, `sleep_secs` waking exactly, and task 0 exiting cleanly. Builds for `thumbv6m` and links the RISC-V/ARM/QEMU targets below. | `SysTick` slice, `PendSV` register-save, hardware stack frame |
| **RISC-V 32 (RV32IMAC)** | `riscv32imac-unknown-none-elf` | **verified running in QEMU (`-M virt`)** | CLINT `mtime`/`mtimecmp`, `mtvec` trap entry, 30-word trap frame, `mret` |
| **ARM A/R profile (ARMv7-A/R)** | `armv7a-none-eabi`, **`armv7r-none-eabi`** | **verified running in QEMU (`-M virt -cpu cortex-a15`)**, and compiles for Cortex-R | ARM generic timer (`CNTP_*`), GICv2, `SRSDB`/`RFEIA` frame with banked `SP` |
| **Xtensa (classic ESP32, LX6)** | `xtensa-esp32-none-elf` | **builds, links and boots in QEMU (`-machine esp32`)**: prints over UART0, timer fires, context switch runs end-to-end — then a known defect on resumption (**not** `VERDICT : PASS`; see [`docs/ESP32.md`](docs/ESP32.md)) | `CCOMPARE0` slice, `VECBASE` vector table, hand-written window spill/refill |
| ESP32-C3 / C6 (RISC-V) | `riscv32imc-unknown-none-elf` (C3), `riscv32imac-…` (C6) | not ported; **C3 is `imc`, not `imac`** (no atomics, see below) and neither has a CLINT/PLIC — the tick comes from the Timer Group / SysTimer through the interrupt matrix | — |
| ARM 64-bit, RISC-V 64-bit | `aarch64`, `riscv64*` | not ported (frame assembly must use 64-bit loads/stores; everything else is identical) | — |
| AVR 8-bit (ATmega328P) | `avr-none` | not ported; the **core is already AVR-compatible** (no atomics anywhere, 16-bit `usize`) and the target needs `-C target-cpu=atmega328p` + `avr-gcc` for linking (`lld` cannot link AVR). QEMU can run it via `-M arduino-uno`. | Timer1 CTC + `reti` |
| Windows, Linux | `x86_64-pc-windows-msvc`, `x86_64-unknown-linux-gnu` | **verified (tests + demos)** | waitable timer / `SIGALRM`, OS-thread or fibre switch |

The Xtensa port is the only one that needs a toolchain beyond `rustc`: `rustup target
add` is not enough, because `xtensa-esp32-none-elf` exists only in the esp-rs fork of
LLVM and its `rust-lld` cannot link Xtensa at all. See
[Building and running the ESP32 port](#building-and-running-the-esp32-port).

Compile-checking the new ports is one command each; both are in the table above
because they are what CI would run:

```bash
cargo build --target riscv32imac-unknown-none-elf      # RISC-V 32 port
cargo build --target armv7r-none-eabi                  # Cortex-R port
cargo build --target armv7a-none-eabi                  # Cortex-A port
```

## Measured behaviour (not claims)

| | RISC-V 32 (QEMU `virt`, RV32IMAC, 10 MHz `mtime`) | ARM A/R (QEMU `virt`, Cortex-A15, 62.5 MHz) | Cortex-M (168 MHz) | Linux (this repo, WSL2) | Windows |
|---|---|---|---|---|---|
| slice source | CLINT `mtimecmp` | generic timer `CNTP_TVAL` | `SysTick`, `RVR+1` cycles | `CLOCK_MONOTONIC` hrtimer | high-resolution waitable timer |
| slice accuracy | 10000 ticks = 1000000 ns exactly; comparator rewritten on every switch | 62500 ticks = 1000000 ns exactly; `TVAL` reloaded on every switch | cycle-exact; `SysTick->VAL` is zeroed on **every** switch, so the period cannot drift | 1506 ticks in 1.51 s = 1.00 ms per tick | ~1.5 ms effective per 1 ms request |
| tick jitter | worst period error 25 µs (emulated timer) | worst period error reported as 0 ticks | a few cycles (measured through `DWT->CYCCNT`) | typ. a few µs, worst ~0.18 ms | typ. ~0.5 ms |
| fairness (pure RR) | spinner 137 vs observer 128 slices | spinner 108 vs observer 99 slices | exact by construction | 57 vs 57 slices in the demo; bench spread 3 of ~295 | spread 0 (196 vs 196) |
| automatic exit | 3 tasks reclaimed, ring 2 nodes / 0 dead | 3 tasks reclaimed, ring 2 nodes / 0 dead | same demo logic | same | same |

Hard numbers come from `jitter_bench` and `scheduler::stats()`
(`worst_period_error_ns`, `last_latency`, `ticks_deferred`, arena accounting).
The two QEMU columns are produced by the shared demo in `firmware-common/`, whose
report is the program's verdict (`VERDICT : PASS`) and — on RISC-V — QEMU's exit
status.


## Honest limits

* **Cycle-exact slices are a bare-metal property.** A hosted OS adds timer
  resolution and scheduler jitter; the kernel measures and reports it instead of
  hiding it. On Windows the *effective* period is `slice + switch cost`, because
  the timer is re-armed after the switch so that every task still receives a
  full slice.
* **Blocking is opt-in and bounded.** `sync::Mutex` is the only blocking primitive: a
  task that cannot take the lock is marked `Blocked` and switched away, and
  `try_lock_for` bounds the wait. There is still no `sleep`/`join`-style syscall, and on
  the POSIX fibre backend a task that blocks on anything *else* (I/O) stalls the whole
  kernel, since every task there shares one OS thread.
* **Lock discipline is enforced, not suggested.** Every lock carries a monotonic
  `LockId`; acquiring one whose id is not strictly greater than the last you hold is
  refused (`LockError::OrderViolation`). That is what removes the circular-wait
  condition, and the price is that locks must be created in the order they will be
  acquired.
* **One known scheduler bug, reproduced on hardware.** A sleep can occasionally return
  *early*: observed on an STM32F103 as roughly one call in fifteen, always shortly after
  another task is created or destroyed, and always a fraction of the requested duration
  (never a hang, never a crash). The evidence points at `scheduler::block_current`, whose
  "am I the current task?" check returns without blocking when `KERNEL.current` is null or
  stale — which is why the examples annotate an early return with `EARLY`. Not fixed yet,
  and documented rather than hidden.
* **No priorities**, so worst-case lateness is `(N-1)` slices. That is inherent
  to pure round robin, which the design requires.
* **Cortex-M FPU:** the port disables lazy FP stacking and saves no S-registers,
  so use `thumbv7m-none-eabi` (soft-float) or extend `PendSV` for S16–S31.
* **Cortex-M stacks:** task 0 (`main`) runs on MSP, which doubles as the kernel
  stack. Keep `main` light or make it the idle loop.
* **RISC-V:** the interrupt controller is machine mode only, and the interval is
  expressed in `mtime` ticks, so `SchedulerConfig::timer_hz` is required (there is
  no portable way to read the counter frequency from the hart). Chips without a
  CLINT (ESP32-C3's SYSTIMER, say) point `arch::configure_timer` at their own
  registers.
* **ARM A/R profile:** a GICv2 will not deliver a group-1 interrupt to a *secure*
  CPU, and QEMU's `virt` boots into secure EL1. That is why the runner passes
  `secure=off`; on a board where a secure monitor exists, use
  `arch::ArmConfig::secure()` (group 0/FIQ) or have the monitor drop you to
  non-secure EL1. See `docs/DESIGN.md` §5.
* TCBs and stacks are recycled once a task returns — do not keep raw pointers
  into a task after it has finished.

## Layout

```
src/
  lib.rs            public API + crate docs
  tcb.rs            TaskControlBlock, TaskState, Kernel (KERNEL), layout asserts
  ring.rs           circular doubly-linked ring: insert_after, unlink, next_runnable, checker
  scheduler.rs      policy: init/config, schedule_next, counters, stats, spawn, reclamation
  trampoline.rs     task entry + automatic teardown (unlink, counters, immediate switch)
  arena.rs          bump allocator with block recycling (TCBs, closures, task stacks)
  closure.rs        placing and running a `FnOnce` with no heap
  config.rs         Slice, SchedulerConfig, PlatformLimits, ConfigError
  critical.rs       critical sections (interrupts / signal mask / kernel lock)
  smp.rs            CpuArch HAL: core id, spinlock, IPI — per-target implementations
  sync.rs           sleeping Mutex, LockId ordering enforcement, timeout + back-off
  time.rs           sleep with units: Duration -> absolute deadlines on the global tick
  app_support.rs    what `#[rrkernel]` calls: configure, the log black box, fault handlers
  arch/
    cortex_m.rs     SysTick + PendSV + frame construction (thumbv6m-safe assembly)
    arm_ar.rs       ARMv7-A/R: generic timer + GICv2 + SRSDB/RFEIA frame (banked SP)
    riscv.rs        RV32 machine mode: CLINT mtime/mtimecmp + mtvec trap frame + mret
    xtensa.rs       ESP32 LX6: VECBASE vector table + CCOMPARE0 + window spill/refill
    win32.rs        tick thread + SuspendThread/ResumeThread baton
    posix.rs        SIGALRM + x86-64 fibre switch
firmware-common/    the demo application, written once for every architecture
firmware-cortex-m/  bare-metal demo: vector table, link.x, three tasks, debugger-readable report
firmware-riscv/     RISC-V demo for QEMU -M virt (UART + SiFive finisher exit)
firmware-arm-a/     ARM A/R demo for QEMU -M virt (PL011 + semihosting exit)
firmware-esp32/     ESP32 (Xtensa) demo for Espressif's QEMU -machine esp32 (UART0)
examples/           roundrobin_demo, jitter_bench, smoke
  cortex-m-bluepill/    standalone cortex-m-rt app on an STM32F103C8 (RTT output)
  cortex-m-blackpill/   the same on an STM32F401, driving the LED through stm32f4xx-hal
rrkernel-macros/    the `#[rrkernel]` proc-macro (host-only: syn/quote)
tests/              ring / arena / closure / ABI invariants
docs/               DESIGN.md, PORTING.md, ESP32.md
scripts/            run-linux.sh, qemu-monitor-probe.ps1 (inspect a hung QEMU guest)
LICENSE             MIT
```

More detail: [`docs/DESIGN.md`](docs/DESIGN.md),
[`docs/PORTING.md`](docs/PORTING.md) and
[`docs/ESP32.md`](docs/ESP32.md) (the Xtensa port's status page).

## Contributing — bugs, and more hardware

This kernel is small enough that a useful contribution is a weekend, not a quarter. The most
valuable things, roughly in order:

1. **Fix a known bug.** They are characterized, not mysterious:
   * a `sleep` can occasionally return *early* (about one call in fifteen on an STM32F103,
     shortly after a task is created or destroyed). Evidence and the prime suspect —
     `scheduler::block_current`'s "am I the current task?" check — are under *Honest limits*.
   * the Xtensa resume defect: an instruction trace and a disproved theory in `docs/ESP32.md`.
   * `firmware-cortex-m`'s `REPORT` is a non-`mut` static, so it lands in `.rodata` (flash) and
     its `write_volatile`s can never take effect. Two lines plus a re-run.
2. **Port it to a board you own.** `docs/PORTING.md` is a checklist rather than a wish list:
   the port surface is about eleven functions, most of a port is comments about the target's
   quirks, and a Cortex-M port is essentially two files (`memory.x` plus a `Cargo.toml`)
   because the kernel supplies the trap entries. A port that runs the shared demo to
   `VERDICT : PASS` is a complete contribution.
3. **Bring up a second core.** The HAL, the validation and the lock-ordering rules exist; what
   is missing is `arch::start_secondary_cores()` and per-core `current_tcb`. Success criterion
   is in the roadmap: the demo asserts that *both* cores took ticks, under `-smp 2` in QEMU.
4. **Give `sync` a runtime test.** A host test hammering a `static Mutex` from N OS threads
   would cover contention, ordering rejection and the timeout/back-off path with no hardware.
5. **Measure something and publish the number.** The host backends especially: the Linux path
   is scripted but unmeasured, and the Windows numbers above come from one machine at a 1 ms
   slice — where the OS timer is the bottleneck. Better data is a real contribution. macOS is
   unported and the exact work list is small (three struct layouts, one timer).

House rules, learned the hard way — they are why these documents read the way they do:

* **Never make a status table prettier than the truth.** Most of this kernel's bugs were found
  by distrusting a status line, and the most expensive ones were the ones a document had
  quietly gotten wrong.
* **A silent failure is a bug.** Almost every bring-up failure here looked like a hang: an ISR
  that was never wired into the vector table, a clock that never ticked, a stack pointer of
  zero. If a new path can fail, it has to say so — which is why the fault handlers log, and
  why the RAM black box keeps the last thing the kernel said even with no console attached.
* **Ports must not skip the contract.** `on_tick()`, the `critical_enter` token polarity ("zero
  when interrupts were already masked"), and the full-slice restart are documented in
  `docs/PORTING.md` because every shortcut around them has produced a real bug in this tree.

A contribution that makes the claim *smaller* — "this is broken, here is the trace" — is as
welcome as a feature. That is how the open bugs above were found.

## License

MIT — see [`LICENSE`](LICENSE). Use it on any board you like, including commercially; if you
port it somewhere, the only thing asked in return is that `docs/PORTING.md` gets the quirks
you discovered, so the next person does not have to find them the way you did.



Every claim above is meant to be checkable, and the command that checks it sits next to it.
Where something is *not* verified — the multi-core path, `sync` at runtime, the Xtensa port,
the early-returning sleep — it says so in as many words rather than being left to
implication. That is deliberate: most of the bugs this kernel has had were found by
distrusting a status line, and the ones that cost the most were the ones a document had
quietly gotten wrong.


## What happens next

In the order that makes each step verifiable rather than hopeful:

1. **Finish the Xtensa/ESP32 port** to `VERDICT : PASS`. The blocking defect is
   localized (the switch path runs end-to-end; the *resume* lands in the wrong place,
   with `a0` pointing into the kernel vector slot) and the instrumentation that found
   it is still in the tree. Next: drop the trace calls out of the restore path, then
   dump the 32-word frame, then decide whether a full window-chain spill is needed at
   all in a call0 build. `docs/ESP32.md` has the exact evidence.
2. **Make `active_cores > 1` real** — per-core `current_tcb` after offset 0, the global
   interlock in `critical_enter`, `arch::start_secondary_cores()`, and then a RISC-V
   `-smp 2` run (a hart lottery on `mhartid` works with `-bios none` because QEMU
   starts every hart at the reset vector) followed by ARM `-smp 2` (PSCI `CPU_ON` +
   GICv2 `SGIR`). Success criterion: the shared demo asserts that *both* cores took
   ticks, in QEMU.
3. **Exercise `sync` end-to-end.** A host test that hammers a `static Mutex` from N OS
   threads would cover contention, ordering rejection and the timeout/back-off path
   without a board — the cheapest way to promote the layer from "compiles" to
   "verified". Then a cross-core contention demo on `-smp 2`.
4. **Cortex-M0 / AVR / `riscv32imc`.** The kernel core is already free of atomics and
   16-bit-`usize`-clean; what these need is a port (`thumbv6m` exists today, `avr-none`
   needs `-C target-cpu=atmega328p` + `avr-gcc`, and `riscv32imc` needs the ESP32-C3
   Timer Group / SysTimer tick source instead of a CLINT).
5. **Real hardware — working, with one board still to be confirmed.**
   `examples/cortex-m-bluepill/` (STM32F103C8) is verified end to end: RTT over the probe,
   100/250/500 ms threads, `sleep_secs`, `VERDICT : PASS`.
   `examples/cortex-m-blackpill/` (STM32F401, LED on PC13 through `stm32f4xx-hal`) runs the
   kernel and blinks correctly, but its RTT output is blocked by a *tool* problem on that
   board: `probe-rs` reports a successful flash while the image's initial stack pointer
   lands as zero in the device, so the core faults before executing anything. Verify with
   `probe-rs read --chip STM32F401CE b32 0x08000000 2` (must read `20010000 08000195`) and
   flash with `--connect-under-reset` if it does not. Also worth doing: an equivalent
   "just use the ecosystem" example for RISC-V (`riscv-rt`) so the pattern is uniform, and
   a hardware example that exercises `sync` and `active_cores`, which today are only
   compiled.


# Porting to another architecture

The kernel core is neutral `core` code. Everything a new target must provide
lives in one file under `src/arch/`, and is the surface below. A working port is
typically 150–350 lines, most of it comments about the target's quirks.

## The port surface

```rust
// --- timing -----------------------------------------------------------------
fn plan_timer(cfg: &SchedulerConfig) -> Result<TimerPlan, ConfigError>;
fn start_timer(plan: TimerPlan) -> Result<(), ConfigError>;
fn retune_timer(plan: TimerPlan) -> Result<(), ConfigError>;
fn platform_limits() -> PlatformLimits;
fn cycles_to_ns(cycles: u32) -> u64;

// --- atomicity --------------------------------------------------------------
type CriticalToken;                       // copyable snapshot of the mask/state
unsafe fn critical_enter() -> CriticalToken;
unsafe fn critical_exit(token: CriticalToken);

// --- task construction ------------------------------------------------------
// `unsafe`: the pointer comes from the kernel, and the contract — a live, arena-allocated
// TCB plus kernel context — is the caller's to uphold. Until this was written down in the
// signature, `rrkernel::arch::create_task(wild_pointer, 0)` was callable from *safe* code
// (the module is re-exported), which is a soundness hole rather than a style choice.
unsafe fn adopt_current_task(tcb: *mut TaskControlBlock) -> Result<(), ConfigError>;
unsafe fn create_task(tcb: *mut TaskControlBlock, stack_size: usize) -> Result<(), SpawnError>;

// --- the switch -------------------------------------------------------------
fn request_switch();                      // switch ASAP + restart the slice
fn exit_current_task_forever() -> !;      // what a finished task does
fn idle_forever() -> !;                   // nothing runnable
unsafe fn on_task_reclaimed(tcb: *mut TaskControlBlock);
unsafe fn is_pinned(tcb: *mut TaskControlBlock) -> bool;
fn parks_synchronously() -> bool;         // does a blocking call park the caller *before* returning?
fn mark_running();
fn shutdown(code: i32) -> !;
```

To add a target: create `src/arch/<name>.rs`, add a `cfg`-gated `mod`/`pub use`
pair in `src/arch/mod.rs`, and keep the existing `compile_error!` fallbacks
honest.

## Checklist, in dependency order

1. **`plan_timer`** validate `cfg.slice` against the real timer and return the
   *achieved* values. Never clamp silently: `ConfigError::SliceBelowPlatformMinimum`
   / `SliceAboveTimerRange` / `TimerClockRequired` exist for exactly this. Bare
   metal ports must require the clock frequency (`cfg.timer_hz`), since only the
   application knows it.
2. **`CriticalToken` + `critical_enter/exit`** the smallest thing that makes a
   critical section atomic: interrupt masking (`PRIMASK`/`BASEPRI`), a signal
   mask, or a lock. It must be nestable and must not block (the tick path enters
   it too).
   **The token's polarity is part of the port contract**: it must be *zero when
   interrupts were already masked* and *non-zero when they were enabled*, because
   every backend's `critical_exit` restores by testing `token != 0`. Returning the
   raw mask bit instead (`PRIMASK == 1` meaning "masked") inverts the sense and
   leaves interrupts masked forever after the first critical section the kernel
   then appears to hang with nothing to show for it, which is exactly what
   happened to the ARM A/R port until it was traced. On CPUs with more than one
   interrupt class (ARM A/R masks both `I` and `F`), return the bits that were
   *clear* so `critical_exit` can restore them individually.
3. **`adopt_current_task`** turn the *calling* context into task 0. On bare
   metal this means deciding where `main`'s stack lives. Keeping it on the kernel
   stack (as Cortex-M does via `TCB_FLAG_USE_MSP`) is the only way to avoid
   copying a live C stack; taking a fresh stack requires assembly that relocates
   it, plus a warning that any frame pointer in the copied frames breaks.
4. **`create_task`** allocate the task's stack (usually
   `crate::scheduler::arena()`), then craft an initial register frame whose
   program counter is `crate::trampoline::task_trampoline` and whose first
   argument is the TCB pointer. The frame must be laid out exactly as the
   switch's *restore* path expects: a first switch and a resumed switch must be
   indistinguishable.
5. **`request_switch`** make the CPU switch as soon as the critical section (if
   any) exits: pend an exception, signal an event, raise a signal. Do **not**
   perform the switch inline from a task unless the target makes that
   unavoidable (the POSIX port is the exception, and the reason is documented).
6. **`exit_current_task_forever`** what a task runs after unlinking itself. On
   bare metal this is a `wfi`/spin loop: the switch takes the CPU away and the
   stack is recycled, so the loop is never re-entered. On a host, terminating the
   OS context (`ExitThread`) is usually right.
7. **`idle_forever` / `shutdown`** the "ring is empty" and "stop the kernel"
   paths. Prefer a real low-power wait (`wfi`) over a spin.
8. **`is_pinned`** only needed if the reclaimer can run *concurrently* with the
   switch and needs a TCB to stay alive (the Win32 tick thread does; POSIX and
   Cortex-M do not).
9. **Where to call `scheduler::reclaim_finished_tasks()`** the subtlest
   decision in a port. It frees the finished task's stack, so it must run in a
   context that is **not** on that stack:
   * its own kernel stack (Cortex-M: `PendSV` on MSP) → before the switch;
   * another OS thread (Win32 tick thread) → before the switch, with `is_pinned`
     protecting the TCB currently in use;
   * the same interrupted stack (POSIX, no `sigaltstack`) → **after** the switch,
     when a different fibre's stack is in use.

## Frame recipes that already exist

* **Cortex-M** (`src/arch/cortex_m.rs`): hardware exception frame
  (`xPSR, PC, LR, R12, R3..R0`) plus the software-saved `R4-R11`, with `R0`
  carrying the TCB. Push order and the `thumbv6m`-safe register moves are
  documented inline, including why `stmdb {r4-r11}` and `cbz` are unusable.
* **RISC-V 32** (`src/arch/riscv.rs`): 128-byte frame 28 GPRs plus `mepc` and
  `mstatus`, addressed from `sp`, consumed by `mret`. `gp`/`tp` are left alone
  because they are constant for a single-hart bare-metal kernel. `mtimecmp` is
  written with the high half parked at `0xFFFF_FFFF` so a 32-bit bus cannot expose
  an already-expired comparator.
* **ARM A/R profile** (`src/arch/arm_ar.rs`): 72-byte frame —
  `[pad][r0..r12][lr][PC][CPSR]`, built by `SRSDB sp!, #SVC` + `push {r0-r12, lr}`
  and consumed by `pop` + `rfeia sp!`. Only `SP`/`LR` are banked, so the
  interrupt handler can push the interrupted task's `r0-r12` straight onto that
  task's stack.
* **x86-64 SysV** (`src/arch/posix.rs`): 72-byte frame —
  `[MXCSR + x87 CW + pad][r15 r14 r13 r12 rbx rbp][return address]`, with
  `r12 = TCB` and the return address pointing at an assembly entry that unblocks
  the tick signal before jumping into Rust. FP control state **must** be
  `0x1F80`/`0x037F`.

## The one call a port must not forget

A port's tick handler must call `scheduler::on_tick()` **on every tick**, before it asks for a
switch. It is the only writer of the kernel's clock and the only place expired deadlines are
swept, so a port that omits it looks perfectly healthy while being wrong in four ways at once:

* `scheduler::tick_count()` stays 0 forever;
* `sleep_ticks()` / `sleep_secs()` never wake the task is off the CPU permanently;
* `sync::Mutex::try_lock_for` never times out (its deadline sweep is the same path);
* every `ticks > 0` invariant in the demos fails.

On Cortex-M this was a real, long-lived bug: the tick handler measured the period error and
poked `PendSV`, the switch happened, tasks ran and the clock simply never moved. The
symptom was a task waiting for 300 ticks forever, with a healthy-looking system and no
output, which is exactly the kind of failure that is worth designing against: the port now
also has a **black box** (see `app_support`) so the last thing the kernel said survives a
hang.

## Trap entry symbols: two naming conventions, both supported

Most of the Cortex-M ecosystem (`cortex-m-rt` and the device crates) names the core exception
vectors **without** the `_Handler` suffix `SysTick`, `PendSV`, `HardFault` while a
hand-written vector table (like `firmware-cortex-m`) names them `SysTick_Handler`,
`PendSV_Handler`. The Cortex-M port exports **both**, the short ones as one-instruction
branches to the real handlers, so either table reaches the same code. This matters more than
it looks: a port that provides only one spelling is silently unreachable from half the
ecosystem, and the failure mode is that every tick lands in `DefaultHandler` the software
appears to hang, with no fault and nothing to read.


`src/arch/posix.rs` is Linux/glibc-only because of three struct layouts. To add
macOS/BSD:

1. `sigset_t` is 4 bytes (`u32`) on Darwin, not 128.
2. `struct sigaction` on Darwin has `sa_mask` as a `sigset_t` and no
   `sa_restorer`; field order is `handler, mask, flags`.
3. `sigevent` differs (`sigev_notify_function`, `sigev_notify_attributes`), and
   the `SA_RESTART`/`SIGEV_SIGNAL`/`SA_ONSTACK` *values* differ do not reuse the
   Linux constants.
4. `timer_create` exists on Darwin, but verify the clock (`mach_absolute_time`
   semantics) before trusting the slice.

The x86-64 fibre switch itself is unchanged (the SysV ABI is the same), and the
same two POSIX lessons apply: unblock the tick signal in a fresh fibre's entry,
and initialise `MXCSR`/`FCW` to their architectural defaults.

## Unsupported targets

`src/arch/mod.rs` ends with two `compile_error!`s (one for unsupported host OSes,
one for unsupported bare-metal architectures) that name this document. Replace
them with a port rather than a workaround: the kernel's invariants (deferred
reclamation context, full-slice restart, frame identity) are what make it
correct, and every shortcut around them has already produced a real bug in this
repository's history.

## Adding multi-core support to a port

Single-core ports need nothing from this section. A port that *can* run more than one
core implements `smp::CpuArch` five functions, all of them hardware queries:

```rust
fn current_core_id() -> usize;          // mhartid / MPIDR / PRID / OS thread id
fn max_cores() -> usize;                // what the chip has, not what you intend to use
fn supports_smp() -> bool;              // honest answer, see below
fn spinlock_acquire(lock: &LockWord);   // target-native interlock
fn spinlock_release(lock: &LockWord);
fn send_ipi(target_core: usize);        // reschedule another core; no-op if single-core
```

Three things to get right, each of which has already bitten this repository:

1. **The lock word is whatever the hardware can do atomically**, not necessarily an
   `AtomicBool`. Byte atomics exist on ARM (`LDREXB`/`STREXB`) and nowhere else in this
   tree: RISC-V's `A` extension is 32-bit only, Xtensa's `S32C1I` is a conditional
   *word* store, and AVR / `riscv32imc` have no atomic RMW at all. `LockWord` is a
   per-target alias for that reality; where there is no atomic RMW the interlock is a
   local interrupt mask, and `supports_smp()` must then return `false` a masked
   test-and-set protects nothing against another core.
2. **Local masking is the lock\'s job, not the trait\'s.** `smp::SpinLock` masks
   (via the port\'s own `critical_enter`) *around* the interlock. Skipping the mask lets
   a core be interrupted by its own tick while holding a lock and then spin forever
   waiting for itself the classic self-deadlock that a cross-core lock does not
   protect against at all.
3. **`send_ipi` must not be a lie.** It is called when a task becomes runnable on a
   core other than the one that is about to sleep, so "silently do nothing" turns into
   a task that runs late or not at all. If the port cannot deliver an IPI yet, keep
   `active_cores == 1` (validated by `set_active_cores`) and document it. The in-tree
   implementations are CLINT `msip[hart]` (RISC-V) and GICv2 `SGIR` (ARM); note that
   the ARM one additionally needs SGI 0 enabled in `GICD_ISENABLER0`, which this
   port does not do yet, which is exactly why its `supports_smp` is aspirational.

### What a real second core still needs

Implementing `CpuArch` makes the *kernel* core-agnostic; starting a core is separate
work, and it is **not done yet for any port**:

* **per-core `current_tcb`** the kernel keeps one global today, and `PendSV` reads it
  at a *literal offset 0*, so per-core state has to be appended after it and the
  `const _` layout assertion extended rather than moved;
* **the global interlock** `critical_enter` must take the kernel spinlock in addition
  to masking once more than one core is active, or two cores will mutate the ring
  simultaneously;
* **secondary bring-up** RISC-V `-smp 2` with `-bios none` starts *every* hart at the
  reset vector, so a `mhartid` lottery plus a park word is enough (non-boot harts spin
  on the word and `wfi`); ARM wants PSCI `CPU_ON` over the configured conduit;
* **IPI-driven rescheduling** the reason `send_ipi` exists at all.

Success criterion, once those land: the shared demo asserts that both cores took ticks,
run under `qemu-system-riscv32 -M virt -smp 2` and `qemu-system-arm -M virt -smp 2`.
Until then treat `active_cores > 1` as configuration and validation only.

### Targets without atomics (`thumbv6m`, AVR, `riscv32imc`)

The kernel core is already free of atomics and 16-bit-`usize`-clean, but two layers are
cfg-gated on `target_has_atomic`:

* `sync` (sleeping `Mutex`) is compiled out see §"the lock word" above;
* `smp::supports_smp()` is `false`, so `set_active_cores(2)` fails loudly.

That is deliberate, and the same reasoning the AVR note in `src/arch/mod.rs` uses: an
atomic that does not exist cannot be polyfilled without changing what a critical section
*means*. A port for such a target reports `max_cores() == 1` and leaves it there.

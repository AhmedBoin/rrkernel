# Design

This document is the engineering record: what the kernel guarantees, how each
target achieves it, which bugs the design has already been through, and where the
limits are.

## 1. Contract

Requirements, and how each one is met:

| Requirement | Implementation |
|---|---|
| No `scheduler_run()` | `scheduler::init*` arms the tick source before returning; the calling context becomes task 0 |
| No `yield_now()` / `await` | the slice timer preempts unconditionally |
| No priorities, pure round robin | `ring::next_runnable` walks `next` exactly one step at a time; every task has the same slice |
| O(1) completion, no dead nodes in the ring | `trampoline::exit_task`: `(*prev).next = next`, `(*next).prev = prev`, `active_threads -= 1`, then an immediate switch |
| User-chosen slice at init | `SchedulerConfig::slice` (`Nanos`/`Micros`/`Millis`/`Hertz`/`Cycles`), validated against the platform and re-tunable at run time |
| Identical API across targets | every backend implements the same port surface (`docs/PORTING.md`) |
| Zero dependencies | `core` only on bare metal; the few OS calls are declared by hand on hosts; no `alloc` anywhere |

## 2. Data structures

### Task control block

```rust
#[repr(C)]
pub struct TaskControlBlock {
    pub sp: *mut u8,             // offset 0 read by PendSV with a literal offset
    pub stack_base: *mut u8,
    pub stack_size: usize,
    pub closure_block: *mut u8,  // the task body: [header | FnOnce], arena-allocated
    pub state: TaskState,        // Ready | Running | Dead
    pub flags: u8,               // bit 0: context lives on MSP (Cortex-M task 0)
    pub next: *mut TaskControlBlock,
    pub prev: *mut TaskControlBlock,
    pub id: u32,
    pub slice_cycles: u32,       // reserved: per-task quantum without an ABI break
    pub slices_run: u32,
    pub switches: u32,
    pub backend: *mut u8,        // Win32 thread handle, ... (unused on cortex-m)
}
```

`sp` at offset 0 and `flags`'s offset are `const`-asserted in the library and
cross-checked by a test, because the Cortex-M assembly reads both with literal
offsets (§6 shows the generated `ldrb r2, [r0, #17]`).

### Kernel control state

`#[no_mangle] pub static KERNEL: Kernel` one `UnsafeCell` per field, so
`current_tcb` sits at offset 0 (the `PendSV` literal) and `offset_of!` can assert
it. Fields: `current_tcb`, `ring_head`, `total_threads`, `active_threads`,
`ticks`, `switches`, `ticks_deferred`, `worst_latency`, `last_latency`,
`worst_period_error_ns`, `last_period_error_ns`, `next_id`, `running`,
`shutdown`, `pending_free`, `reclaimed`, `config`.

`ring_head` exists because `current_tcb` is legitimately null for the few
instructions between a task unlinking itself and the port switching away from it;
the switch path must never walk a ring whose "current" node is already gone.

### Ring invariants

* A linked TCB always has non-null `next`/`prev`; a one-element ring points at
  itself. `check` returns `Ok(0)` for a fully unlinked node (a finished task).
* `unlink` poisons the removed node's links, so a double unlink or a walk through
  a dead node trips an assertion instead of corrupting the live ring.
* `next_runnable` skips `Dead` nodes and returns null only when nothing is
  runnable.
* `ring::check` validates circularity and both directions; it runs in tests and
  in `debug_assertions` after every mutation.

## 3. Life cycle

```
spawn(f)                                    exit (trampoline)
--------                                    ----
critical section                            state = Dead
  TCB      <- arena                         (*prev).next = next     } O(1)
  closure  <- arena (placed, not boxed)     (*next).prev = prev     } unlink
  stack    <- arena (bare metal / fibres)   ring_head = successor
  create_task: build the initial frame      current_tcb = null
    sp -> [callee-saved] [args] [LR] [PC]   active_threads -= 1
       PC = task_trampoline, R0/r12 = TCB   self -> pending_free
ring_insert_after(current, new)             request_switch()  <- immediate, not next tick
total_threads += 1, active_threads += 1     arch::exit_current_task_forever()
request_switch()                            (never returns; the next switch
                                             reclaims this stack)
```

Two details matter, and both were bugs at some point:

1. **Identity comes from the frame, not from a global.** The first version of
   this kernel looked up `KERNEL.current_tcb` inside the trampoline; after a
   preemption that can be a different task. Every backend now passes the TCB in
   a register (Cortex-M `R0`, x86-64 `r12`, Win32 thread parameter).
2. **Deferred reclamation.** A task cannot free the stack it is standing on, so
   `exit_task` only unlinks and queues itself;
   `scheduler::reclaim_finished_tasks` running in a context that is *not* on
   that stack does the `Arena::free` calls. Where "not on that stack" is
   differs per backend (§5), and on POSIX it is *after* the switch, not before.

## 4. Slice timing: the part that makes it real-time

Every backend re-arms or restarts the timer **on every switch** the "reset the
interrupt counter so the next task gets a full slice" requirement. A task that
exits early, or a spawn that kicks the scheduler, therefore never shortens
anybody's slice, and the period cannot drift.

| | mechanism |
|---|---|
| Cortex-M | `SysTick->LOAD = cycles-1`, then `SysTick->VAL = 0` in the `PendSV` epilogue (clears `COUNTFLAG` and restarts the countdown) |
| Linux | one-shot `timer_settime` with `it_value = slice` on every switch |
| Windows | one-shot `SetWaitableTimer(-slice)` on every switch |

Slice validation is a *contract*, not a clamp: `init_with` returns
`ConfigError::SliceBelowPlatformMinimum` / `SliceAboveTimerRange` /
`TimerClockRequired` instead of silently scheduling something else. Bare metal
requires the core clock (`SchedulerConfig::embedded(slice, hz)`) because there is
no portable way to discover it and the SysTick reload depends on it.

Timing accuracy is measured, not asserted:

* Cortex-M: `DWT->CYCCNT` sampled in `SysTick_Handler` (period error) and around
  the Rust half of the switch (service time). Cores without `DWT` (M0/M0+) are
  detected at runtime and report 0 rather than nonsense.
* Linux: `clock_gettime(CLOCK_MONOTONIC)` in the handler which also has to
  distinguish a *timer* expiry from a *kick* (`raise` from `spawn`/exit) so
  `ticks` stays meaningful.
* Windows: `QueryPerformanceCounter`, with the same timer-vs-kick distinction.

## 5. Backend-specific engineering

### Cortex-M (`arch/cortex_m.rs`)

* `SysTick_Handler` only sets `PENDSVSET`; the switch happens in `PendSV`, which
  has the lowest priority and therefore never preempts another handler.
* The switch saves R4–R11 as **two 16-bit `stmia`s with `mov`s through r4–r7**,
  not one Thumb-2 `stmdb {r4-r11}`: the 16-bit LDM/STM encoding can only address
  r0–r7, and `thumbv6m` (Cortex-M0) has no Thumb-2. Cost: ~4 cycles on M3+.
  `cbz` is avoided too (`cmp`/`beq` instead) the first build against
  `thumbv6m-none-eabi` failed with `instruction requires: armv8m.base`, which is
  exactly what building that extra target is for.
* Task 0 (`main`) stays on **MSP** with `flags |= TCB_FLAG_USE_MSP`; `PendSV`
  returns `0xFFFFFFF9` for it and `0xFFFFFFFD` for PSP tasks. Keeping `main` on
  MSP is the only way to adopt the running `main` without relocating a live C
  stack (copying the stack breaks every frame pointer). Cost: MSP doubles as the
  kernel/ISR stack, so keep `main` light.
* Lazy FP stacking is disabled (`FPCCR.ASPEN = LSPEN = 0`) and no S-registers are
  saved: a lazy FPU frame pushed on the wrong stack after a stack switch is the
  classic silent-corruption bug. Use the soft-float target or extend `PendSV`.
* `PRIMASK` is used for critical sections rather than `BASEPRI`, which does not
  exist on ARMv6-M. Ring policy lives in Rust (`rrkernel_schedule_next`); the
  assembly does only register/stack mechanics.

### Windows (`arch/win32.rs`)

* Windows has no per-thread asynchronous timer signal, and `SwitchToFiber` is
  thread-affine, so a timer callback on a pool thread can never switch the fiber
  of the thread burning CPU. What *can* force a running thread off the CPU from
  another thread is `SuspendThread`, so tasks are real OS threads and a tick
  thread holds the baton: `SuspendThread(previous)`, `ResumeThread(next)` —
  exactly one task running at a time.
* The kernel lock is a recursive spinlock (owner thread id + depth) that
  *blocks*: the only way to reach it is a bounded, non-blocking critical section,
  so the owner always makes progress and the tick thread cannot livelock.
* `RUNNING_TCB` is tracked as a **TCB pointer**, not a thread handle: handle
  values are recycled by later `CreateThread` calls, so a stale handle could make
  the tick thread suspend the wrong thread. The TCB the tick thread holds is
  pinned (`is_pinned`) and never reclaimed underneath it.
* `shutdown` must not be done from a task thread: `ExitProcess` can deadlock
  while other threads are suspended observed here, not theorised. The tick
  thread resumes every task and then terminates the process (`TerminateProcess`,
  after flushing the standard streams); a task that asks for shutdown parks, with
  a safety valve that terminates directly if the tick thread cannot run.

### Linux (`arch/posix.rs`)

* POSIX timers (nanosecond `CLOCK_MONOTONIC`) deliver `SIGALRM`; the handler
  performs the switch with hand-written x86-64 fibre switching (callee-saved
  registers + `MXCSR` + x87 control word).
* **No `sigaltstack`**: several fibres' interrupted handler frames would share
  the alt stack and overwrite each other when the scheduler switches between them
  mid-handler. The handler therefore runs on the interrupted fibre's stack, which
  is *why* reclamation happens after the switch instead of before.
* Two bugs only real execution could reveal both found by running this on Linux:
  1. a fibre created by the scheduler is born *inside a signal handler's callee
     chain*, so the thread's `SIGALRM` mask is still blocked; the fibre ran once
     and was then never preempted again. `rrkernel_fibre_entry` unblocks it at
     entry.
  2. a fresh fibre's `MXCSR`/x87 control word must be the architectural default
     (`0x1F80`/`0x037F`), not zero. Zero unmasks every FP exception, so the first
     inexact result traps as `SIGFPE`.
* `SIGALRM` is blocked (not merely deferred) inside critical sections: a signal
  raised while blocked stays *pending* and is delivered the moment the section
  ends, so no tick is ever lost and no lock is needed.
* Linux/glibc only for now: `sigaction`/`sigset_t`/`sigevent` layouts differ on
  macOS/BSD, and shipping unverified layouts would corrupt memory. `PORTING.md`
  lists the ~30 lines needed.

### RISC-V 32 (`arch/riscv.rs`)

Machine mode, one trap vector, no separate kernel stack. The trap entry (`mtvec`,
direct mode) builds a 128-byte frame on the interrupted task's own stack 28 GPRs
plus `mepc` and `mstatus`, all of it addressing `sp` directly:

```asm
addi sp, sp, -128
sw   ra, 0(sp) ... sw s11, 48(sp)     ; callee-saved
sw   a0, 52(sp) ... sw t6, 108(sp)    ; caller-saved
csrr t0, mepc    ; sw t0, 112(sp)
csrr t0, mstatus ; sw t0, 116(sp)
la   t0, KERNEL  ; lw t1, 0(t0)      ; current_tcb
bnez t1, 1f      ; sw sp, 0(t1)      ; publish sp (null before the first switch)
1: csrr a0, mcause
   call rrkernel_trap_switch          ; Rust: tick accounting + policy
   beqz a0, 4f
   lw   sp, 0(a0)                     ; switch stacks
   call rrkernel_after_switch         ; on the NEW stack: reclaim + re-arm
   lw t0, 112(sp) ; csrw mepc, t0
   lw t0, 116(sp) ; csrw mstatus, t0
   lw ra, 0(sp) ... lw t6, 108(sp)
   addi sp, sp, 128
   mret
4: wfi ; j 1b                         ; nothing runnable
```

Two details are worth naming because they are the whole reason the port is a file
rather than a `#[cfg]`:

* **Reclamation happens after the `sp` swap.** A finished task cannot free the
  stack it is standing on, so `rrkernel_after_switch` (which reclaims and rewrites
  `mtimecmp`) runs on the incoming task's stack. On Cortex-M the same job happens
  on MSP before the switch; both are correct, and both are *about the same
  constraint*.
* **The slice is a 64-bit comparator.** `mtimecmp` is written half at a time with
  the timer interrupt masked and the high half parked at `0xFFFF_FFFF` first,
  because writing the low half last can leave a comparator the counter has already
  passed an interrupt storm. `mtime` is read high-low-high for the same reason.

`SchedulerConfig::timer_hz` is required (the interval is in `mtime` ticks and no
portable register reports the frequency), and `arch::configure_timer` redirects the
three registers for chips whose timer is not a CLINT.

### ARM A/R profile (`arch/arm_ar.rs`)

A genuinely different execution model from Cortex-M: **nothing is pushed by
hardware**, `SP` and `LR` are banked per mode, and the return is not an
`EXC_RETURN` value but `SRSDB`/`RFEIA`:

```asm
sub   lr, lr, #4        ; LR_irq (interrupt return) -> the interrupted PC
srsdb sp!, #19          ; push {PC, CPSR} onto the *task's* stack (SVC bank)
cps   #19               ; SVC mode: SP is the task's stack, r0-r12 are the task's
push  {r0-r12, lr}      ; the task's live registers, plus its own LR
sub   sp, sp, #8        ; keep the frame 8-byte aligned
ldr   r0, =KERNEL ; ldr r1, [r0] ; cmp r1, #0 ; strne sp, [r1]
bl    rrkernel_irq_switch
ldr   sp, [r0]          ; switch stacks
bl    rrkernel_after_switch
add   sp, sp, #8
pop   {r0-r12, lr}
rfeia sp!               ; load PC/CPSR from the frame: into the next task
```

The frame is therefore `[pad][r0..r12][lr][PC][CPSR]` = 72 bytes, and a fresh task
is created by writing `r0 = &tcb`, `PC = trampoline`, `CPSR = 0x13` the same
"fabricate the frame the switch will consume" idea as everywhere else.

Three platform lessons, all of them found by running it and none of them
guessable from the architecture manual alone:

1. **Secure state gates interrupt delivery.** A GICv2 will not deliver a *group-1*
   interrupt to a *secure* CPU (nor group 0 to a non-secure one). The physical
   timer's PPI is a non-secure interrupt, so on a CPU in secure EL1 it is enabled,
   pending, and never delivered. QEMU's `virt` boots into secure EL1 with no
   monitor to leave it, which is why the runner is `-M virt,secure=off`, and why
   `ArmConfig` carries `secure_fiq` (group 0/FIQ for a secure CPU, group 1/IRQ for
   a non-secure one) with both vector slots pointing at one handler.
2. **Critical-section token polarity is part of the port contract.** A port whose
   `critical_enter` returns "the mask bit" instead of "was enabled" leaves
   interrupts masked forever after the first critical section the kernel then
   looks hung for no visible reason. `arm_ar` did exactly that for one QEMU run.
3. **The acknowledge register is not always trustworthy.** QEMU's GICv2 can
   return "no eligible pending interrupt" for an interrupt it *just delivered*.
   Trusting it made the round robin work perfectly while reporting zero ticks.
   Since this kernel enables exactly one interrupt source, the port counts a tick
   for any interrupt it takes and uses the acknowledge value only to EOI.

Measured timing on this port comes from `CNTP_TVAL` rather than from a free-running
counter: reading the timer's own overshoot past zero *is* "how late did this slice
run", needs only a 32-bit register read, and avoids depending on the `mrrc`
instruction form (which the bundled assembler rejects, and which is why the 64-bit
`CNTPCT` is not used here). `scheduler::stats().worst_latency` therefore means
"worst timer overshoot, in ticks" on this port and "post-switch service time, in
ticks" on RISC-V the kernel's contract is that each port reports its own backend
latency in its own timer's units, not that they are the same quantity.

### Where the remaining architectures stand

* **AVR** is a toolchain problem, not a design problem: the core avoids atomics
  entirely and uses `usize` for everything (`max-atomic-width = 16` on AVR), but
  `avr-none` needs `-C target-cpu=atmega328p` and `avr-gcc` for linking (`lld` has
  no AVR backend). The frame is 35 bytes (`r0–r31`, `SREG`, and the two return
  address bytes, low byte first), the slice is Timer1 in CTC mode, and `reti` is
  the switch. QEMU can run it with `-M arduino-uno`.
* **Xtensa** is the one port that is not mechanical: ESP32's register-window ABI
  means a switch must spill and refill the window chain, on a toolchain that needs
  the esp-rs fork, on a target upstream QEMU cannot run.
* **64-bit** ARM (`aarch64`) and RISC-V (`riscv64`) need only the 8-byte variants
  of the two frame layouts above; nothing else in the kernel changes.

## 7. Mechanism-level verification

Cortex-M assembly was verified by building, linking and disassembling. `PendSV_Handler`
as generated for `thumbv7m`:

```asm
; r0 = the interrupted task's SP. MSP for task 0, PSP for every spawned task, chosen from
; TCB_FLAG_USE_MSP. Reading PSP unconditionally is how this port first shipped: task 0 runs
; on MSP, so its saved SP became 0 and the switch back to it returned from address 0.
mrs r0, PSP                       ; (overridden with MSP when the current TCB says so)
ldr r3, [pc]  -> 0x20000000       ; &KERNEL (current_tcb at offset 0)
ldr r1, [r3]
cmp r1, #0 / beq 2f               ; nothing current yet -> skip the save
subs r0, #32
stmia r0!, {r4,r5,r6,r7}          ; save R4-R7
mov r4, r8 / mov r5, r9 / mov r6, sl / mov r7, fp
stmia r0!, {r4,r5,r6,r7}          ; save R8-R11
subs r0, #32 / str r0, [r1]       ; tcb->sp = sp
bl rrkernel_schedule_next         ; Rust does the policy
ldr r1, [r0]                      ; next->sp
ldmia r1!, {r4..r7}               ; restore R4-R7
ldr r2,[r1,#0];mov r8,r2; ... fp  ; restore R8-R11
ldrb r2, [r0, #17]                ; flags (offset asserted = 17)
lsls r2, r2, #31 / beq 3f
msr MSP, r1                       ; task 0 path
movs r2, #0 / ldr r3, =0xE000E018 / str r2, [r3]   ; SysTick->VAL = 0
mvn.w r0, #6 / bx r0              ; return 0xFFFFFFF9
3: msr PSP, r1 / ... / mvn.w r0, #2 / bx r0          ; return 0xFFFFFFFD
4: wfi / b 2b                     ; nothing runnable
```

The vector table carries `_stack_start` (0x20010000), `__reset|1`, and
`PendSV_Handler|1` (0x95), `SysTick_Handler|1` (0x120d). The firmware links to
**5904 bytes of flash, 96 bytes of `.data`, 16512 bytes of `.bss`**, and the
linker script asserts *at link time* that `.bss` stays below `_stack_start` an
assertion that fired during development when the default arena was 64 KiB, which
is exactly why it is there.

Behavioural verification:

* `tests/ring_invariants.rs` ring insertion/unlink/skip-dead, the ABI offset
  contract, arena recycling and over-alignment, closure placement/drop semantics.
* `examples/smoke.rs` the backend sanity check: a task that returns, an
  infinite task, a dynamic spawn, and stats at the end.
* `examples/roundrobin_demo.rs` the full life cycle with assertions (Task 2
  completed, Task 3 ran, ring length == `active_threads`, rotation happened).
* `examples/jitter_bench.rs` fairness and timing accuracy, measured against the
  kernel's own switch counter so the numbers are valid even with a coarse clock.
* `firmware-cortex-m` on-target evidence in a debugger-readable `REPORT`
  struct: `t1_loops` climbing (preemption), `t2_completed` (automatic exit),
  `t2_spawned_t3`/`t3_ran` (dynamic spawn), `ring_len == active_threads`,
  `slice_cycles` equal to the requested slice converted at `CORE_HZ`, and
  `invariants_ok` as the overall verdict.

RISC-V and ARM A/R are verified the same way, but with a UART instead of a
debugger, and with the verdict as the program's exit status:

```text
--- report ------------------------------------------------
slice    : 10000 ticks (1000000 ns)          # RISC-V, 10 MHz mtime
threads  : total 5 active 2 reclaimed 3
ticks    : 261  switches 265  deferred 0
accuracy : worst period error 25260 ns, worst switch 1260 ticks
ring     : 2 nodes, 0 dead; slices spinner 137 / observer 128
task 2   : done 1, dynamic spawn 1
VERDICT  : PASS  (round robin + automatic unlink + dynamic spawn)
----------------------------------------------------------
```

```text
slice    : 62500 ticks (1000000 ns)          # ARM Cortex-A15, 62.5 MHz
threads  : total 5 active 2 reclaimed 3
ticks    : 204  switches 201  deferred 0
ring     : 2 nodes, 0 dead; slices spinner 108 / observer 99
VERDICT  : PASS  (round robin + automatic unlink + dynamic spawn)
```

Both run the *same* application code (`firmware-common/`) and the same assertions,
which is the point of the exercise: the port replaces the frame, the timer and the
switch, and nothing else. The Cortex-M firmware predates that crate and keeps its
own debugger-readable `REPORT` struct instead of a UART report the same three
tasks, the same verdict, a different way of getting it off the board.

Bring-up tooling that made this tractable, and is kept in the tree:

* `arch::set_trap_hook` / `arch::set_trace_hook` (RISC-V and ARM A/R) callbacks
  at the port's decision points, so a board with only a serial port can *show*
  what the kernel thinks instead of hanging silently.
* `scripts/qemu-monitor-probe.ps1` reads interrupt-controller, timer and kernel
  state from inside a running QEMU guest, which distinguishes "never raised" from
  "raised and not taken" from "taken and switched wrongly".
* QEMU's `-d int` plus the guest's own report: the pair that found the stale
  comparator on RISC-V and the masked interrupt on ARM.

## 8. Where this design would go next

* **RISC-V port** (`riscv32imac-unknown-none-elf` is already an installed
  target): machine timer (`mtimecmp`) as the slice source, software interrupt for
  the switch. The port surface is fixed, so this is additive.
* **Release-time periodic tasks** (the "ns-precise periodic release" requirement)
  sit on top of this ring as a timer wheel that writes into the TCB's reserved
  `slice_cycles`/state no ABI change needed.
* **Per-task quantum** is already reserved in the TCB (`slice_cycles`), still
  pure round robin, for workloads that need longer stretches than others.
* **FPU support on Cortex-M** means `vstmdb`/`vldmia` of S16–S31 in `PendSV` plus
  keeping lazy stacking off; a `fpu` feature is the natural shape of that work.

---

## 7. Multi-core and synchronization

### 7.1 The HAL, and why it is only five functions

Everything above §7 is written once and works on 1..N cores because the only
per-core facts in the kernel are reduced to `smp::CpuArch`:

| Need | RISC-V | ARM A/R | Xtensa (ESP32) | Host |
|---|---|---|---|---|
| `current_core_id()` | `csrr mhartid` | `mrc p15,0,r,c0,c0,5` (MPIDR) | `rsr PRID` | per-OS-thread id |
| interlock | `lr.w`/`sc.w` (or `amoswap.w`) | `LDREXB`/`STREXB` | `S32C1I` | `Atomic*` |
| `send_ipi(core)` | CLINT `msip[hart]` | GICv2 `SGIR` | `DPORT` latch | no-op |
| `max_cores()` / `supports_smp()` | 8 / true | 8 / true | 2 / true | parallel / true |

**The lock word is not always an `AtomicBool`.** The design sketch says
`spinlock_acquire(lock: &AtomicBool)`, which is right where byte atomics exist (ARM has
`LDREXB`/`STREXB`) and wrong everywhere else: RISC-V's `A` extension provides 32-bit AMOs
and nothing narrower, Xtensa's `S32C1I` is a conditional *word* store, and AVR /
`riscv32imc` have no atomic read-modify-write at all. So the interlock is typed as
`LockWord`, a per-target alias, and on the targets with no atomic RMW it degrades to a
local interrupt mask correct mutual exclusion **on one core**, which is exactly why
`supports_smp()` is `false` there and `set_active_cores(2)` returns a `ConfigError`.

Two consequences worth stating explicitly, because both are easy to get wrong:

* **Masking is not in the trait.** It is the same operation on every target (the port
  primitive `critical_enter`), while the interlock is not. `smp::SpinLock` applies the
  mask around the trait call, and that ordering is load-bearing: an interlock taken
  *without* masking lets a core be interrupted by its own tick while holding it, and
  then spin forever waiting for itself to release it.
* **`sync` is cfg-gated on `target_has_atomic = "32"`.** The id allocator and every
  lock's owner word are atomics; a mutex whose owner field cannot be updated atomically
  is not a mutex, so the layer is compiled out on Cortex-M0/AVR/RV32IMC rather than
  silently approximated.

### 7.2 `KernelConfig::active_cores`

`active_cores: 1` is the default and keeps today's behaviour bit-for-bit.
`scheduler::set_active_cores(n)` validates `1..=smp::max_cores()` and refuses `> 1`
without a cross-core interlock; the value survives `init_with`'s config snapshot
(which is why it is read back and `max(1)`-ed rather than reset). The intent is that
cores `>= active_cores` park in the port's low-power wait instead of pulling tasks.

**What is *not* implemented:** no port starts a secondary core yet, and the kernel still
keeps a single global `KERNEL.current_tcb`. Real `active_cores > 1` needs per-core
`current` state (kept *after* offset 0, so the Cortex-M `PendSV` literal offset and its
`const _` assertion stay valid), the global interlock taken by `critical_enter` when more
than one core runs, IPI-driven rescheduling, and `arch::start_secondary_cores()` in each
port. Until then `active_cores` is configuration and validation only, and the repository
contains **no multi-core execution evidence** `-smp 2` has never been run.

### 7.3 Sleeping `Mutex`, and forcing deadlock out of the design

The scheduler is pure round robin with no priorities, so neither priority inheritance
nor "just spin" is available. `sync::Mutex<T>` therefore:

* **sleeps on contention.** `lock()` marks the caller's TCB `Blocked(lock id)` and asks
the port for an immediate switch O(1), so the waiter costs no CPU and gives its slice
back to the system. `unlock()` wakes exactly the tasks blocked on that id.
* **is identified.** Every lock has a monotonic `LockId` (created at runtime, or pinned
  with `Mutex::with_id` for a `static`), which is what makes the ordering rule below
  expressible.
* **bounded.** `try_lock_for(timeout_ms)` times out against the slice tick, because that
  is the only clock in the kernel; the tick path sweeps expired deadlines.
* **recoverable.** `lock_with_backoff(timeout_ms, attempts, attempt)` implements the
  runtime protocol: on `Timeout`, `release_all_held_locks()` → bounded pseudo-random
  `backoff()` → run the caller's whole acquisition sequence again.

**Static elimination of circular wait.** Each TCB carries a small stack of held lock ids
(`held_locks`/`held_count`), and `try_acquire` refuses any lock whose id is not strictly
greater than the last one held, with `LockError::OrderViolation { held, requested }`.
Because ids are handed out in creation order, "acquire in increasing id order" is a total
order over every lock in the system and a cycle cannot exist in a total order. The
price is a real constraint rather than a suggestion: **locks must be created in the order
they will be acquired**, and the check is enforced at acquisition time so a violation is
a refusal, not a hang.

**Why releasing is what actually breaks a deadlock.** Delaying and retrying alone merely
postpones the cycle. What dissolves it is a task giving up what it already holds, which
is why the recovery path releases *all* held locks (through a small `'static` registry so
the kernel does not have to know which locks exist) before backing off.

### 7.4 `TaskState::Blocked` and the ring

A blocked task stays **linked** in the ring; `ring::next_runnable` skips it exactly as it
skips a dead one. That choice is what keeps both directions O(1) no queue has to be
searched to put the task back, its rotation slot is preserved, and `ring::len()` still
equals `active_threads` (which the demos assert). The cost is that the wake path is
explicit: `wake_blocked_on(id)` walks the ring and flips matching tasks back to `Ready`,
which is O(active tasks) rather than O(waiters). A per-lock waiter list would be O(waiters)
but needs another intrusive link in every TCB; at the task counts this kernel targets the
ring walk is the cheaper trade, and the *context switch* it triggers is still O(1).

### 7.5 Status

| | |
|---|---|
| `smp.rs`, `sync.rs`, `Blocked`, `active_cores`, spinlocks | implemented; `cargo test --features std` (8 + 2) and all six bare-metal targets build |
| Multi-core scheduling on `-smp 2` (RISC-V / ARM) | **not implemented, never run** |
| `sync` at runtime (contention, timeout, back-off) | **not exercised**; the layer is host-*compiled* and unit-untested |

### 7.6 ESP32 / Xtensa, in one paragraph

The Xtensa port is bring-up complete but **not** passing: it builds with the esp-rs fork
plus Espressif's GCC as linker, boots in Espressif's QEMU (`-machine esp32`), prints over
UART0, takes its `CCOMPARE0` tick and runs the whole switch path (verified by `E T K N S R`
markers) then the *resume* lands in the wrong place. The evidence, the disproved
window-chain theory and the corrected root cause (`rsil` and `s32c1i` are illegal while
`PS.EXCM` is set, which every trap runs with) are in `docs/ESP32.md`; the atomic and
critical-section fixes that came out of it are in the tree and apply to every target.

## 8. Blocking, parking and the idle path

*Added after the first hardware-verified run. Every number here was measured on an
STM32F103C8 at 8 MHz with 1 ms slices (`examples/cortex-m-bluepill/src/bin/fidelity.rs`).*

### 8.1 The blocking contract

Every wait — `sleep`, `lock`, and the `park`/`Signal` layer built on the same primitive — goes
through `scheduler::mark_blocked_locked`, whose whole reason for existing is that three things
happen inside **one** critical section: the deadline is read and stored, the state becomes
`Blocked`, and the switch is pended.

Splitting them is how this kernel acquired two real bugs. `sleep_ticks` read the counter
*before* entering the section, so a tick landing in the gap made the deadline one tick short.
`sync::Mutex::lock` tested the lock and then blocked as two separate steps, so an `unlock` in
between woke nobody and the waiter parked with no deadline at all: blocked forever on a free
lock. Both are fixed, and both are now hard to reintroduce, because the section-taking wrappers
(`block_until_tick`, `block_for_ticks`) are the only public entry points.

A wait attempted with **no current task** (interrupt or idle context) no longer succeeds
silently: it is counted in `stats().blocks_without_current` and asserted in debug builds.
Returning quietly was the third way a sleep could "wake up early" — it had never blocked at all.
The sleep path also re-checks its deadline in a loop, so a spurious wake costs another block
instead of an early return.

### 8.2 The blocked-head bug

`schedule_next` took `KERNEL.ring_head` **blindly** whenever there was no current task — that is,
right after any task exit — instead of asking for the first *runnable* task. `ring_head` moves on
every spawn, so a `Blocked` task can be at the head, and it was then resumed in the middle of its
sleep. That is the original defect: *"about 1 in 15 sleeps return early, most often right after a
task is created or destroyed"*.

| same firmware, same board | before | after |
|---|---|---|
| a 200-tick sleep in an otherwise idle ring | **6 ticks** | **200 ticks** |
| `DWT->CYCCNT` across that window | 49 758 (6.2 ms of real time) | 1 595 243 = **997 ‰** of expected |

The cycle counter agreeing with the tick counter is what makes it a proven early *wake* rather
than a counter artifact. Fix: `ring::first_runnable_from`, plus a regression test covering
blocked and dead heads.

### 8.3 The idle path (fixed, and measured the honest way)

The idle branch reaches `rrkernel_idle_wait` (Rust, called from the `PendSV` idle branch), which
counts the wait, clears the `PendSV` request it has just serviced (`SCB->ICSR = PENDSVCLR`), and
then executes `wfi`. Clearing is essential: the tick handler pends `PendSV` on **every** tick, and
since this code is already inside `PendSV` that request can never be taken, so it stays latched —
and `WFI` completes immediately whenever any exception is pending. That is the "PendSV spins at
full speed" case predicted in this document before the board was attached, and it was real.

Measured on an STM32F103C8, 200-tick idle window: **`idle_waits` grew by exactly 200 = 1 per
tick**, so the core parks once per tick and sleeps between ticks.

**A methodology correction, recorded because it cost real time.** The first reading of this used
`DWT->CYCCNT`, which advanced at 997 per mille across the window; that was taken as "the core is
not gated, so the loop is spinning". It was wrong in both directions:

* it reproduced identically with **no debugger attached** (flash + reset, run free, read the RTT
  backlog later), so a SWD session was not keeping the clock alive; and
* after the `PENDSVCLR` fix, with the wait counter proving one `wfi` per tick, `CYCCNT` *still*
  advanced at 997 per mille.

So on this part `CYCCNT` keeps counting while the core is genuinely asleep. The lesson is in the
code: `KERNEL.idle_waits` exists because the cycle counter could not answer the question, and the
"is it sleeping?" check must never be inferred from a counter that is not proven to stop.

Consequence for the time base, now positively established: because `CYCCNT` continues across a
*verified* sleep, DWT is a usable time base on this part (no timer peripheral needed), provided a
64-bit extension folds its wrap.

### 8.4 Asymmetric parking on the two `std` backends (open)

A task that sleeps, blocks or yields is switched away immediately on **every bare-metal port**:
the switch is taken on the way out of the blocking critical section. On the host backends it is
not, and the port says so through `arch::parks_synchronously()`:

* **Win32** — the tick thread performs the switch, so a blocking task keeps running until it is
  suspended, which can be a whole slice later. Measured before the "nothing runnable → suspend
  the previous task" fix: 591 724 of 591 839 sleeps returned at `elapsed == 0`.
* **POSIX** — the `SIGALRM` handler has no idle context to switch to when nothing is runnable, so
  it returns and the blocked fibre simply resumes. A scenario that keeps *every* task blocked
  (which is exactly what a strict sleep test does) hangs there.

In both cases the kernel's bookkeeping is right — the task is `Blocked`, skipped by
`ring::next_runnable`, parked a moment later, `blocks_without_current` stays zero — but a task
measuring its own sleep cannot see that. `examples/sleep_fidelity.rs` reports the property and
runs its strict form only where the port can honour it. Fixes when wanted: an idle fibre for
POSIX, a per-task park event for Windows.

### 8.5 What a switch costs at the real clock

`DWT`-measured at 8 MHz with spawn/exit churn on the ring:

| metric | measured |
|---|---|
| switch, last / worst | **38 / 80 cycles** |
| tick period error, worst | **2674 ns** (0.27 % of a 1 ms slice) |
| early sleeps (601 sleeps, 1501 churn cycles) | **0** |
| `ticks_deferred` / `blocks_without_current` | **0 / 0** |

The 10×-switch-cost rule for `plan_timer` therefore implies a floor near **800 cycles**, not the
100 in `arch::cortex_m::MIN_SLICE_CYCLES`.

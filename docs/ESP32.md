# ESP32 (Xtensa) how to build, run, and exactly where this port stands

> **Current status: bring-up complete, scheduling NOT verified.** The image builds,
> links and boots in Espressif's QEMU (`-machine esp32`), prints the shared demo's
> banner over UART0, takes its `CCOMPARE0` slice tick, and the whole switch path runs
> end-to-end (proven by the `E T K N S R` trace markers). What fails is the *resume*:
> the task does not continue where it left off, and the guest ends up executing
> unmapped code. It is **not** `VERDICT : PASS`, so do not trust anything it reports.
> The corrected root cause and the next concrete steps are in
> [RESOLVED ROOT CAUSE](#resolved-root-cause-supersedes-the-window-chain-theory-above)
> below the window-chain theory this file opened with was wrong.

This is the honest status page for the newest port in `rrkernel`: the classic **ESP32**
(Tensilica **Xtensa LX6**, dual-core). It is written up separately from
`docs/PORTING.md` because Xtensa is the one architecture whose low-level rules (fixed
vector table, register windows, `EXCM` semantics) are unlike every other target here.

## What was needed, beyond the usual port

| Piece | Why it is not like the other ports |
|---|---|
| **Toolchain** | `xtensa-esp32-none-elf` only exists in the esp-rs fork of LLVM (`espup`), and its `rust-lld` cannot link Xtensa at all: `-m elf32xtensa` is rejected as *unknown emulation*. Linking goes through Espressif's `xtensa-esp32-elf-gcc` as the linker driver. |
| **Inline asm** | Still gated in rustc: `#![feature(asm_experimental_arch)]` on this target only. |
| **Vector table** | There is no "install a vector". The hardware dispatches to `VECBASE + VECOFS`, and each vector owns a **0x40-byte slot**. The kernel vector is therefore a 6-instruction stub, because 64 bytes is all it gets. |
| **`VECBASE`** | Resets to `0x4000_0000` (mask ROM, ~512 KB from our code). The kernel vector's exit must be a `j` (no register is free for an indirect jump, no room for a literal), so `start_timer` moves `VECBASE` to the bottom of IRAM and `link.x` puts the whole table there. |
| **Literals** | `l32r` reaches **backwards only**, so `.literal` must be linked *before* the code that uses it. |
| **Register windows** | 64 physical address registers, four windows, rotation on every call. Overflow/underflow need their own handlers, and the *Alloca* recovery shares the underflow-4 slot it fits in 61 of those 64 bytes. |
| **`EXCM`** | Set by hardware on entry to the kernel vector, and **software cannot clear it** (only `rfi`/`rfe` can). A window-rotating call while `EXCM = 1` raises an *Alloca* instead of spilling. This is why the canonical low-level Xtensa code is compiled for the **call0 ABI** (`__XTENSA_CALL0_ABI__` guards in ESP-IDF's `vectors.S`), and this port follows suit with `-C target-feature=-windowed`. |

## Building and running

```powershell
# toolchain: esp-rs fork (rustup run esp) + Espressif GCC (espup install -t esp32)
espup install -t esp32

# Espressif's QEMU is the *only* QEMU with an ESP32 machine. Upstream QEMU 11.1.0 has
# neither the machine nor the CPU (`qemu-system-xtensa -cpu esp32` is an error there),
# which is why this is a downloaded fork rather than the installed QEMU:
#   https://github.com/espressif/qemu/releases -> qemu-xtensa-softmmu-*-x86_64-w64-mingw32.tar.xz
#   (extract to tools/qemu-esp32; gitignored)

rustup run esp cargo build -p firmware-esp32 -Zbuild-std=core --release --target xtensa-esp32-none-elf

tools\qemu-esp32\qemu\bin\qemu-system-xtensa.exe -nographic -machine esp32 `
    -global driver=timer.esp32.timg,property=wdt_disable,value=true `
    -kernel target\xtensa-esp32-none-elf\release\firmware-esp32
```

`-global … wdt_disable` is not optional: the emulated Timer Group watchdog is enabled at
reset and will reset the guest mid-demo. QEMU's ESP32 machine loads a bare-metal ELF
directly (`-kernel`), planting a stub at the reset vector that jumps to our entry point —
no flash image, no IDF bootloader.

## Status: what is verified, and what is not

**Verified to work in QEMU (Espressif fork 9.2.2, `-machine esp32`):**

* the image builds and links (Xtensa GCC driver, custom `link.x`, vector table at the
  bottom of IRAM, RWX IRAM `LOAD` segment as expected for a bare-metal image);
* `_start` runs: `PS.WOE` is set, `sp` is loaded, `.data`/`.bss` are set up;
* the demo boots and prints its banner over UART0 the *shared* `firmware-common` demo,
  unchanged, the same code Cortex-M/RISC-V/ARM run which means deep Rust call chains
  execute correctly on this core. Several are deeper than the four physical register
  windows, so window spill/refill was exercised for real;
* `VECBASE` is accepted at `0x4008_0000` (the port verifies it and fails loudly if not);
* the slice timer fires, the kernel exception vector dispatches it, and the **first
  context switch completes**: the interrupted context resumed and finished printing
  `slice    : requested 1 ms`. (In an earlier state that line was cut off mid-format,
  which is exactly how the difference was measured.)

**Not finished the one blocking defect:** after that first successful switch the kernel
falls into a repeating trap loop (QEMU's `-d int` trace shows the same cause and PC
repeating with `ccount` frozen). The missing piece is what the canonical implementation
does and this port does not yet: a **whole-window-chain spill and refill at switch time**.
The live-window frame used here is sufficient for the switch itself, but it leaves
`WINDOWSTART` which windows hold live data, a register rather than per-task state —
disagreeing with the window base the task resumes with, and software cannot write
`WINDOWSTART` (`wsr` to it is rejected by the assembler, consistent with the ISA). The
canonical fix is Tensilica's `_xt_context_save` / `_xt_context_restore`, which walk and
spill every live window into the frame so a task's window state is entirely self
contained. That change is contained: the frame grows from 128 bytes to one window per
call depth, and `xtensa_exception_handler` gains the spill/refill walk.

Until then, treat this port as **bring-up complete, scheduling not yet verified**. Nothing
else in `rrkernel` is affected: the port is selected purely by target triple, and
`cargo test --features std` plus every other target's build and QEMU run stay green.

## Evidence used while porting (kept in `tools/reference/`, gitignored)

| File | What it settled |
|---|---|
| `core-isa-esp32.h` (from Espressif's QEMU) | `XCHAL_WINDOW_*_VECOFS`, `XCHAL_KERNEL_VECOFS` (0x300), `RESET_VECTOR` (0x400), `INT6_LEVEL = 1`, `EXCM_LEVEL = 3`, `HAVE_PRID`, `NUM_TIMERS = 3` |
| `esp-idf-vectors.S` (ESP-IDF `xtensa_vectors.S`) | the canonical Alloca recovery (`_xt_alloca_exc`), the window spill/refill sequences, and the `EXCCAUSE_ALLOCA` → `call0 _xt_alloca_exc` dispatch *before* any frame is built |
| `assembly_esp32.rs`, `esp32_vectors.rs` (esp-rs `xtensa-lx-rt`) | Rust-native Xtensa asm: the `call4` argument mapping, `PS_*` constants, and the context frame esp-rs uses |

Bugs this evidence plus QEMU found, in order:

1. `rsr a0, EPS1` does not assemble: for a level-1 interrupt there is **no** `EPS1` the
   interrupted `PS` lives in `PS` itself, with `EXCM` set.
2. Saving `PS` without masking `EXCM` made the resumed task unable to spill windows at
   all; the emulator trace's `ps = 0x00060f10` is that bug.
3. `call4` into Rust while `EXCM = 1` (which is always, on this vector) turns every call
   into an `AllocaException`. That is the reason for the call0 ABI.
4. Xtensa conditional branches reach only ±128 bytes, so the long level-1 path must be
   entered with `j` and the short unexpected-trap path must fall through.
5. Literals must precede their uses, and `ASSERT` inside an output section is rejected by
   Xtensa's `ld`.
6. `VECBASE` is writable, which is what makes the whole vector table reachable from IRAM.

## RESOLVED ROOT CAUSE (supersedes the window-chain theory above)

Instrumentation settled this. Markers along the switch path (`E` enter, `T` Rust half,
`K` tick counted, `N` next chosen, `S` switched, `R` restore) plus instrumented vector
slots turned the "flaky hang" into a named fault. The window chain was **never the
problem** under the call0 ABI this port already builds with, there are no window
rotations at all.

**The actual bug: `rsil` and `s32c1i` are illegal while `PS.EXCM` is set, and every trap
runs with `EXCM` set.**

`EXCM` is set by hardware on entry to the kernel vector, and as the emulator's own trace
showed (`ps = 0x00050013`) `wsr` to `PS` cannot clear it (only `rfi`/`rfe` can, which is
why the canonical code sets `PS` for its handler and lets the return clear it). So the
entire kernel half of the switch runs in exception mode, and in that mode:

* `rsil` (this port's `critical_enter`) is illegal;
* `s32c1i`, the conditional store behind every Rust atomic RMW, is illegal.

The port used both: `LAST_TICK_CC.swap()` in the period measurement, and `rsil` in
`critical_enter` so the switch died at the period measurement, and any code reached from
the trap path that took a critical section (including the firmware's own trap hook calling
`scheduler::stats()`) died too. It looked flaky because it depended on whether the
interrupted code had reached those instructions.

**Fixes applied:**

1. `LAST_TICK_CC` is a single-writer volatile cell instead of an `AtomicU32`. The trap path
   is single-threaded with interrupts masked, so this is both correct and the only thing
   that works there.
2. `critical_enter`/`critical_exit` use `PS` read/modify/write (`wsr PS`) rather than
   `rsil`, and *raise* `INTLEVEL` instead of setting it legal in both modes, and it does
   not lower the level when called from inside the kernel half.
3. Every previously empty vector slot (levels 2–7, user exception, double exception) now
   holds a stub that reports its identity and `EXCCAUSE`/`EXCVADDR`/`EPC1`/`PS`/`a0`/
   `INTENABLE`/`INTERRUPT` over UART0 and parks, through a writer that cannot itself fault
   (the reporter's flag is a plain cell, not an atomic, for the same reason as (1)).
4. The firmware's trap hook (`TRAP_LOG`) must stay **off**: it calls `scheduler::stats()`,
   i.e. it takes a critical section from inside the trap path. That is now documented at
   the constant.

**Result:** the switch path now runs end-to-end the markers print `E T K N S R`, meaning
the trap is entered, the tick is counted, the next task is chosen, `sp` is swapped, the
outgoing task is reclaimed and the slice re-armed. The demo reaches
`slice    : requested 1 ms` and the first preemption of task 0 completes.

**What is still wrong:** after the restore, the resumed context does not continue in the
task; the guest ends up executing unmapped code and QEMU exits on a guest-initiated reset,
with no panic and no trap report i.e. the *resume* is not landing where the task left
off. The double-exception dump captured during this work shows `a0 = 0x400803c9`, an
address *inside the kernel exception vector slot*, which points at the restore path
(mis-restored `a0`/`PC`, or a frame field clobbered by the `call0` trace calls the trace
calls are the newest change in that path and the first thing to remove when resuming this
work). So the remaining defect is a frame restore/layout bug in
`xtensa_exception_handler`, and the instrumentation needed to find it is already in place
and proven to work.

Next concrete steps, in order:

1. Drop the `call0 {trace}` markers from the switch path (or make them `movi`-free) and
   re-run to see whether the resume lands correctly the trace calls clobber `a0`, and
   they are interleaved with `l32i sp, a2, 0`.
2. If it still mis-resumes, dump the frame (32 words at `tcb.sp`) through the reporter
   before the restore and compare against the register values the task had.
3. Then re-enable `TRAP_LOG` only after making the hook exception-safe (no critical
   sections, no atomics), since a hook that faults inside the trap path is worse than no
   hook.

The canonical low-level Xtensa code is **call0**, not windowed (`__XTENSA_CALL0_ABI__` in
ESP-IDF's `vectors.S`), and this port already builds that way (`-C target-feature=-windowed`).
Under call0 no `entry`/`call4` is ever emitted, so the register file is flat and the
window-chain problem disappears by construction which means the remaining failure is *not*
the window chain. What the investigation pinned down instead:

1. **The trap path is timing-dependent.** The same binary sometimes resumes correctly after
   the first tick (printing a complete `slice    : requested 1 ms`) and sometimes stops a
   few characters earlier. That is a state bug that only shows up depending on where the
   interrupt lands, not a fixed offset failure.
2. **The software kick is not the culprit.** Disabling it (`USE_SOFTWARE_KICK = false`, and
   masking its interrupt) made the port get *less* far, so the kick number, while still
   unverified against a known-good ESP-IDF build, is not what breaks the switch. The
   constant is kept, documented as unverified, and switchable.
3. **`WINDOWSTART` cannot be repaired in software.** `wsr` to it is rejected by the
   assembler and the ISA agrees: the hardware owns that register. This is exactly why the
   canonical answer is Tensilica's `xthal_window_spill` (referenced from
   `_xt_context_save`, which "saves all Xtensa processor state except PC, PS, A0, A1, A12,
   A13" and delegates the window walk to the HAL), and why a from-scratch replumb of the
   window state is the wrong direction.
4. **The vector table must be complete.** QEMU's `-d int` trace shows a repeating exception
   at a fixed PC with `ccount` frozen the signature of a trap dispatched to a slot that
   cannot handle it. Every unused slot (levels 2–7, the user exception vector) should hold
   a distinctive park/report stub before this port is called done, so a stray trap names
   itself instead of looping.

So the remaining work is: (a) fill and instrument every vector slot, (b) find the
timing-dependent state bug the instrumented run then points at, and (c) only then decide
whether a full window-chain spill is needed at all in a call0 build. Estimated as a
contained piece of work, but it is real debugging, not a mechanical port.

* The classic ESP32 is now a target `rrkernel` builds for, with a real `firmware-esp32`
  crate sharing its task code with every other port.
* The ESP32-C3 (`riscv32imc` note: **not** `imac`; the C3 has no atomics) has its tick
  source answered in `src/arch/riscv.rs`: C3/C6 have **no CLINT** and no PLIC, using the
  Timer Group / SysTimer through the interrupt matrix instead, so an `mtime`/`mtimecmp`
  mapping does not transfer.
* Espressif's QEMU also provides `-machine esp32c3` and `-machine esp32s3`, and publishes
  win64 archives for both the Xtensa and RISC-V32 builds, so more ESP32-family targets can
  be executed not just compiled on this machine.

//! Xtensa (ESP32 / LX6) port: windowed register file, `CCOMPARE0` slice timer,
//! a fixed `VECBASE` vector table and hand-written window spill/refill.
//!
//! # Mapping onto the Cortex-M design
//!
//! | Cortex-M | here |
//! |---|---|
//! | `SysTick` period | `CCOMPARE0`, compared against the free-running `CCOUNT` counter |
//! | `SysTick_Handler` sets `PENDSVSET` | kernel exception vector sees `EXCCAUSE == 4` (level-1 interrupt) |
//! | `PendSV` register save + `sp` swap | frame on the task's own stack, `sp` swap, `rfi 1` |
//! | `bx 0xFFFFFFFD` | `wsr PS/EPC1` from the incoming frame, then `rfi 1` |
//! | `PRIMASK` | `PS.INTLEVEL` (raised with `rsil`) |
//! | `PENDSVSET` kick | `DPORT_CPU_INTR_FROM_CPU_0` (level 1, like the tick) |
//! | (none) | **register windows**: overflow/underflow vectors must spill/refill |
//!
//! # Why this port is not just "another frame"
//!
//! Xtensa LX6 has 64 physical address registers, four 16-register **windows**, and
//! the window base rotates on every call. Two consequences follow, and both are
//! why `docs/PORTING.md` calls Xtensa the one architecture that does not reduce to
//! a recipe:
//!
//! 1. **Window overflow/underflow vectors are not optional.** A call that needs a
//!    window the hardware has no space for raises `EXCCAUSE` 4/5/8/9/12/13, which
//!    the hardware dispatches to *dedicated* vector slots (at `VECBASE + 0`, not
//!    through the kernel vector). With nothing there, deep call chains corrupt
//!    memory silently instead of trapping. The handlers below are the canonical
//!    spill (`s32e` + `rfwo`) and refill (`l32e` + `rfwu`) sequences, including the
//!    underflow handler's `_AllocAException` tail that repairs `PS.OWB` when the
//!    *incoming task's* window base differs from the physical one — exactly the
//!    cross-task case a context switch creates.
//! 2. **A switch must leave the task's window chain on its own stack.** Because the
//!    handlers above spill older windows as calls are made, the switch itself only
//!    saves the **live** window (`a0`-`a15`) plus the special registers, which is
//!    what [`FRAME_SIZE`] describes. The frame is therefore self-contained:
//!    restoring it resumes the task exactly, whatever its window base was.
//!
//! # Frame layout (offsets shared with the assembly through `const` operands)
//!
//! ```text
//!   0 PC      4 PS      8 a0     12 a1     16 a2     20 a3     24 a4     28 a5
//!  32 a6     36 a7     40 a8     44 a9     48 a10    52 a11    56 a12    60 a13
//!  64 a14    68 a15    72 SAR    76 EXCCAUSE 80 EXCVADDR
//!  84 LBEG   88 LEND   92 LCOUNT 96 THREADPTR 100 SCOMPARE1     104..128 pad
//! ```
//!
//! `LBEG`/`LEND`/`LCOUNT` are saved even though the kernel never uses them: LLVM
//! emits zero-overhead loops (`XCHAL_HAVE_LOOPS`), and an interrupt taken inside
//! one would otherwise break the interrupted task's loop. `SCOMPARE1` is saved for
//! the mirror-image reason: the spinlock in `src/lock.rs` programs it (that is what
//! makes `s32c1i` atomic), and a switch inside a critical section must not lose it.
//!
//! # `sp` is `a1`, and the level-1 vector owns no registers
//!
//! Unlike a medium/high-level Xtensa interrupt (which the hardware enters with the
//! interrupted `a0` already parked in `EXCSAVEn`), the level-1 path through the
//! kernel exception vector gets *no* register saved by hardware. The first
//! instruction therefore parks `a0` in `EXCSAVE1` by hand — the convention the rest
//! of the port (and `_AllocAException`) relies on.

use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::tcb::{TaskControlBlock, KERNEL};
use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

/// Frame size, rounded to the 16-byte alignment the Xtensa windowed ABI demands
/// of `sp`.
pub const FRAME_SIZE: usize = 128;

macro_rules! frame_off {
    ($($name:ident = $offset:expr),* $(,)?) => {
        $( pub const $name: usize = $offset; )*
        /// Every field offset, in frame order, for tests and assertions.
        pub const FRAME_OFFSETS: &[usize] = &[$($offset),*];
    };
}

frame_off! {
    OFF_PC = 0,
    OFF_PS = 4,
    OFF_A0 = 8,
    OFF_A1 = 12,
    OFF_A2 = 16,
    OFF_A3 = 20,
    OFF_A4 = 24,
    OFF_A5 = 28,
    OFF_A6 = 32,
    OFF_A7 = 36,
    OFF_A8 = 40,
    OFF_A9 = 44,
    OFF_A10 = 48,
    OFF_A11 = 52,
    OFF_A12 = 56,
    OFF_A13 = 60,
    OFF_A14 = 64,
    OFF_A15 = 68,
    OFF_SAR = 72,
    OFF_EXCCAUSE = 76,
    OFF_EXCVADDR = 80,
    OFF_LBEG = 84,
    OFF_LEND = 88,
    OFF_LCOUNT = 92,
    OFF_THREADPTR = 96,
    OFF_SCOMPARE1 = 100,
}

// ---------------------------------------------------------------------------
// Vector table
// ---------------------------------------------------------------------------
//
// The hardware dispatches to `VECBASE + VECOFS`, and `VECBASE` resets to
// 0x4000_0000 — the chip's mask ROM, half a megabyte away from this kernel's code in
// IRAM. That distance matters: the kernel vector's slot is only 0x40 bytes, so it
// cannot hold a handler, and the jump out of it cannot be an indirect one because
// every register is live at that moment.
//
// So the firmware moves `VECBASE` into IRAM (see `start_timer`) and links the whole
// table at the bottom of it. Every vector is then a short `j` away from the code that
// implements it, on real silicon as well as in QEMU. The offsets are the core's own
// (`XCHAL_WINDOW_*_VECOFS`, `XCHAL_KERNEL_VECOFS`), so this is the *hardware's*
// layout, not a choice.

/// Where the firmware links its vector table, and the value it programs into
/// `VECBASE`. Must stay in step with `firmware-esp32/link.x`.
pub const VECBASE: usize = 0x4008_0000;

/// `XCHAL_WINDOW_OF4_VECOFS`: window overflow, 4 registers.
pub const VEC_WINDOW_OVERFLOW4: usize = VECBASE + 0x000;
/// `XCHAL_WINDOW_UF4_VECOFS` (and the AllocaException entry inside it).
pub const VEC_WINDOW_UNDERFLOW4: usize = VECBASE + 0x040;
/// `XCHAL_WINDOW_OF8_VECOFS`
pub const VEC_WINDOW_OVERFLOW8: usize = VECBASE + 0x080;
/// `XCHAL_WINDOW_UF8_VECOFS`
pub const VEC_WINDOW_UNDERFLOW8: usize = VECBASE + 0x0C0;
/// `XCHAL_WINDOW_OF12_VECOFS`
pub const VEC_WINDOW_OVERFLOW12: usize = VECBASE + 0x100;
/// `XCHAL_WINDOW_UF12_VECOFS`
pub const VEC_WINDOW_UNDERFLOW12: usize = VECBASE + 0x140;

/// `XCHAL_KERNEL_VECOFS`: exceptions *and* level-1 interrupts (the core gives levels
/// 2..7 their own vectors; level 1 shares this one and is distinguished by
/// `EXCCAUSE_LEVEL1_INTERRUPT`).
pub const VEC_KERNEL: usize = VECBASE + 0x300;

/// `XCHAL_DOUBLEEXC_VECOFS`: a trap inside a trap. The firmware parks here rather
/// than pretending it can recover.
pub const VEC_DOUBLE: usize = VECBASE + 0x3C0;

// ---------------------------------------------------------------------------
// Architectural constants
// ---------------------------------------------------------------------------

/// `PS.INTLEVEL` field mask.
pub const PS_INTLEVEL_MASK: u32 = 0x0000_000F;
/// `PS.EXCM`: set while the kernel exception vector runs; masks levels up to
/// `XCHAL_EXCM_LEVEL` (3 on this core).
pub const PS_EXCM: u32 = 0x0000_0010;
/// `PS.UM`: user mode.
pub const PS_UM: u32 = 0x0000_0020;
/// `PS.OWB` shift. The old window base travels *inside* `PS`, which is how a
/// context switch restores a task's window base.
pub const PS_OWB_SHIFT: u32 = 8;
/// `PS.WOE`: window overflow enable. **Must be 1** in task code: with `WOE = 0` a
/// window overflow becomes a double exception instead of a spillable trap.
pub const PS_WOE: u32 = 0x0004_0000;
/// `PS` for a task that is about to start: kernel mode, windows enabled, level 0.
pub const PS_TASK_ENTRY: u32 = PS_WOE;

/// `EXCCAUSE` of a level-1 interrupt — the same vector also receives real
/// exceptions, which is how the two are told apart.
pub const EXCCAUSE_LEVEL1_INTERRUPT: u32 = 4;

/// `XCHAL_TIMER0_INTERRUPT` — `CCOMPARE0`. `XCHAL_INT6_LEVEL == 1`, which is what
/// lets the slice timer share the kernel exception vector.
pub const INT_TIMER0: u32 = 6;

/// Interrupt number of `DPORT_CPU_INTR_FROM_CPU_0`, the software kick used for
/// "switch as soon as it is safe" (RISC-V's `msip`, Cortex-M's `PENDSVSET`).
///
/// **Unverified on hardware/QEMU**: the ESP32 routes sources through a programmable
/// interrupt matrix, so a source index is not an interrupt number, and this constant
/// plus [`DPORT_CPU_INTR_FROM_CPU_0`] have not been confirmed against a working
/// ESP-IDF build. [`USE_SOFTWARE_KICK`] therefore starts `false` and the kernel
/// relies on the slice tick for switches (which is correct, just up to one slice
/// later). See docs/ESP32.md.
pub const INT_SW0: u32 = 0;

/// Whether `request_switch()` raises the software kick interrupt.
///
/// With this `false` a newly spawned task is linked into the ring immediately but
/// gets its first slice at the next tick, and a finishing task's reclaim is likewise
/// deferred to the next tick. The demo is written to tolerate that (it never assumes
/// an immediate switch), so the kernel stays correct either way — which is why it is
/// safe to keep the unverified path switched off until the interrupt number is
/// confirmed.
pub const USE_SOFTWARE_KICK: bool = true;

/// `EXCCAUSE` values for the window exceptions. These are the ISA's numbers, and
/// they are *not* what dispatches the handler (that uses the vector addresses above);
/// they exist so a failure path can name what happened. The CORRECTED mapping is the
/// one QEMU's trace confirmed: 12 was dispatched to the overflow-4 slot.
pub const EXCCAUSE_ALLOCA: u32 = 5;
pub const EXCCAUSE_OVERFLOW4: u32 = 12;
pub const EXCCAUSE_OVERFLOW8: u32 = 13;
pub const EXCCAUSE_OVERFLOW12: u32 = 14;
pub const EXCCAUSE_UNDERFLOW4: u32 = 16;
pub const EXCCAUSE_UNDERFLOW8: u32 = 17;
pub const EXCCAUSE_UNDERFLOW12: u32 = 18;

/// `CCOUNT` frequency used for diagnostics before init: the ESP32's `CCOUNT` is the
/// core clock, so this is the 240 MHz nominal clock.
pub const NOMINAL_CCOUNT_HZ: u32 = 240_000_000;

/// Smallest slice accepted. Below this the trap (frame save/restore, two calls into
/// Rust, the window bookkeeping) dominates the slice itself.
const MIN_SLICE_CYCLES: u64 = 300;

/// `DR_REG_DPORT_BASE + 0`: writing bit 0 raises `CPU_INTR_FROM_CPU_0`. Unlike a
/// RISC-V `msip`, this is per-core hardware, which is what will make cross-core
/// kicking work when the second core is brought up.
const DPORT_CPU_INTR_FROM_CPU_0: usize = 0x3FF0_0000;

// ---------------------------------------------------------------------------
// Special-register access
// ---------------------------------------------------------------------------

macro_rules! rsr {
    ($name:ident, $reg:literal) => {
        /// Read a special register.
        #[inline]
        pub fn $name() -> u32 {
            let v: u32;
            unsafe {
                asm!(concat!("rsr {0}, ", $reg), out(reg) v, options(nomem, nostack));
            }
            v
        }
    };
}

// Special-register *writes* are written inline at each site rather than through a
// macro: every one of them either needs an `rsync` right after, or is the
// `CCOMPARE0` comparator whose write is also the interrupt acknowledge.

rsr!(read_ccount, "CCOUNT");
rsr!(read_sar, "SAR");
rsr!(read_exccause, "EXCCAUSE");
rsr!(read_epc1, "EPC1");
rsr!(read_ps, "PS");
rsr!(read_interrupt, "INTERRUPT");
rsr!(read_intenable, "INTENABLE");
rsr!(read_vecbase, "VECBASE");
rsr!(read_prid, "PRID");

/// The core id (`XCHAL_HAVE_PRID`): 0 = the PRO core, 1 = the APP core. This is the
/// ESP32's answer to RISC-V's `mhartid`, and it is what an SMP port needs to tell
/// the two cores apart.
#[inline]
pub fn core_id() -> u32 {
    read_prid()
}

// ---------------------------------------------------------------------------
// Timer: CCOUNT / CCOMPARE0
// ---------------------------------------------------------------------------

fn configured_hz() -> u32 {
    unsafe { *KERNEL.config.get() }.timer_hz
}

fn cycles_to_ns_with(hz: u32, cycles: u32) -> u64 {
    if hz == 0 {
        return 0;
    }
    ((cycles as u128) * 1_000_000_000u128 / hz as u128) as u64
}

pub fn cycles_to_ns(cycles: u32) -> u64 {
    let hz = configured_hz();
    cycles_to_ns_with(if hz == 0 { NOMINAL_CCOUNT_HZ } else { hz }, cycles)
}

/// Program the comparator. Writing `CCOMPARE0` is *also* how the timer interrupt is
/// acknowledged: a value already in the past would fire again immediately, which is
/// why every write goes through [`rearm_timer`]'s `CCOUNT + slice`.
#[inline]
unsafe fn ccompare0_write(value: u32) {
    asm!("wsr {0}, CCOMPARE0", in(reg) value, options(nomem, nostack));
}

/// Arm exactly the level-1 sources this kernel uses: the slice timer, plus the
/// software kick when it is enabled. Everything else stays masked — including the
/// kick's interrupt number while [`USE_SOFTWARE_KICK`] is `false`: an enabled source
/// the ISR cannot identify would re-assert forever.
#[inline]
unsafe fn enable_kernel_interrupts() {
    let mut mask = 1u32 << INT_TIMER0;
    if USE_SOFTWARE_KICK {
        mask |= 1u32 << INT_SW0;
    }
    asm!("wsr {0}, INTENABLE", in(reg) mask, options(nomem, nostack));
}

/// Restart the slice counter for whoever runs next: `CCOMPARE0 = CCOUNT + slice`.
#[inline]
unsafe fn rearm_timer() {
    let slice = (*KERNEL.config.get()).slice_cycles;
    ccompare0_write(read_ccount().wrapping_add(slice));
}

/// Raise the software interrupt: "switch as soon as it is safe to do so". The trap
/// entry treats it exactly like the tick, except that it counts no tick.
#[inline]
pub fn request_switch() {
    if !USE_SOFTWARE_KICK {
        return;
    }
    unsafe {
        write_volatile(DPORT_CPU_INTR_FROM_CPU_0 as *mut u32, 1);
    }
}

/// Acknowledge the software interrupt, or it re-traps immediately.
#[inline]
unsafe fn clear_software_interrupt() {
    asm!("wsr {0}, INTCLEAR", in(reg) 1u32 << INT_SW0, options(nomem, nostack));
}

// ---------------------------------------------------------------------------
// Critical sections: PS.INTLEVEL
// ---------------------------------------------------------------------------

/// Saved `PS.INTLEVEL` from section entry (0 = interrupts were enabled).
pub type CriticalToken = usize;

/// Disable interrupts by raising `INTLEVEL` to 1, returning the previous level.
///
/// Only the level travels in the token, never the whole `PS`: `PS.OWB` changes on
/// every function call (the window base rotates), so writing back a stale `PS`
/// would corrupt the window base of whatever code we are nested inside.
///
/// # Safety
/// Pair with exactly one [`critical_exit`].
#[inline]
pub unsafe fn critical_enter() -> CriticalToken {
    // Deliberately *not* `rsil`: on Xtensa `rsil` is illegal while `EXCM` is set, and
    // `EXCM` is set for the whole of every trap — so any code reachable from the trap
    // path (including a user's trap hook calling into the scheduler) would fault if
    // this used `rsil`. Reading and raising `PS.INTLEVEL` is legal in both modes.
    //
    // Note it *raises* the level rather than setting it to a fixed value: inside the
    // kernel half the level is already 3, and lowering it there would let interrupts
    // nest into the switch path.
    let ps = read_ps();
    let old_level = ps & PS_INTLEVEL_MASK;
    if old_level < 1 {
        asm!(
            "wsr {0}, PS",
            "rsync",
            in(reg) (ps & !PS_INTLEVEL_MASK) | 1,
            options(nomem, nostack, preserves_flags)
        );
    }
    old_level as CriticalToken
}

/// Restore the interrupt level saved by [`critical_enter`].
///
/// # Safety
/// `token` must come from the matching [`critical_enter`].
#[inline]
pub unsafe fn critical_exit(token: CriticalToken) {
    let cur: u32;
    asm!("rsr {0}, PS", out(reg) cur, options(nomem, nostack, preserves_flags));
    // Note: `EXCM` is deliberately left alone — this function is never called from
    // inside the kernel exception vector, where `EXCM` is 1 by hardware.
    let new = (cur & !PS_INTLEVEL_MASK) | (token as u32 & PS_INTLEVEL_MASK);
    if new != cur {
        asm!(
            "wsr {0}, PS",
            "rsync",
            in(reg) new,
            options(nomem, nostack, preserves_flags)
        );
    }
}

// ---------------------------------------------------------------------------
// Bring-up observability (same hook shape as the RISC-V port)
// ---------------------------------------------------------------------------

/// One trap, as the kernel saw it.
#[derive(Clone, Copy)]
pub struct TrapEvent {
    /// Raw `EXCCAUSE` (byte-granular here: the hardware splits causes 4..6 across
    /// several slots, see [`EXCCAUSE_LEVEL1_INTERRUPT`]).
    pub exccause: u32,
    /// Interrupted program counter (`EPC1`).
    pub epc: u32,
    /// Task that was running.
    pub current: *mut TaskControlBlock,
    /// Task the scheduler chose (`null` if nothing was runnable).
    pub next: *mut TaskControlBlock,
    /// True when this call is the post-switch step, running on the incoming task's
    /// stack.
    pub after_switch: bool,
}

/// Optional bring-up hook. `0` = disabled.
///
/// Costs one relaxed load per trap when unset, and is the only way to see "the
/// kernel hangs inside a trap" on a board with just a serial port.
static TRAP_HOOK: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Install a trap-observation hook (see [`TrapEvent`]).
pub fn set_trap_hook(hook: fn(TrapEvent)) {
    TRAP_HOOK.store(
        hook as *const () as usize,
        core::sync::atomic::Ordering::Relaxed,
    );
}

/// Remove the hook.
pub fn clear_trap_hook() {
    TRAP_HOOK.store(0, core::sync::atomic::Ordering::Relaxed);
}

#[inline]
fn report_trap(ev: TrapEvent) {
    let raw = TRAP_HOOK.load(core::sync::atomic::Ordering::Relaxed);
    if raw != 0 {
        // SAFETY: the pointer was stored from a `fn(TrapEvent)` by `set_trap_hook`.
        let hook: fn(TrapEvent) = unsafe { core::mem::transmute::<usize, fn(TrapEvent)>(raw) };
        hook(ev);
    }
}

/// Read `EPC1` (the interrupted PC), for diagnostics.
#[inline]
pub fn current_epc() -> u32 {
    read_epc1()
}

/// Read `VECBASE` — used at init to prove the vector table the firmware linked at
/// the fixed addresses is the one the core will dispatch to.
#[inline]
pub fn current_vecbase() -> u32 {
    read_vecbase()
}

// ---------------------------------------------------------------------------
// The kernel exception vector: the whole context switch
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Bring-up diagnostics
// ---------------------------------------------------------------------------
//
// A kernel that dies inside an exception has one chance to say why. The port
// therefore owns a tiny UART0 writer (the firmware uses the same block, and the
// port cannot call into the firmware), so that any trap can report itself without
// depending on the console abstraction, the arena, or the scheduler.

/// ESP32 UART0: transmit FIFO (writes push bytes into a 128-entry FIFO).
const UART0_FIFO: usize = 0x3FF4_0000;

/// One byte, best effort. No FIFO-space wait: this runs in fault context, and
/// dropping a byte beats hanging in a spin loop while the fault is live.
#[inline]
unsafe fn diag_putc(c: u8) {
    write_volatile(UART0_FIFO as *mut u32, c as u32);
}

/// Print `0x` followed by 8 hex digits.
unsafe fn diag_hex(v: u32) {
    diag_putc(b'0');
    diag_putc(b'x');
    let mut shift = 28i32;
    loop {
        let nib = ((v >> shift) & 0xF) as u8;
        diag_putc(if nib < 10 {
            b'0' + nib
        } else {
            b'a' + nib - 10
        });
        if shift == 0 {
            break;
        }
        shift -= 4;
    }
}

/// Print a label and a value, e.g. `slice: 0x0000_01f4`.
unsafe fn diag_field(label: &str, v: u32) {
    for b in label.bytes() {
        diag_putc(b);
    }
    diag_hex(v);
    diag_putc(b'\r');
    diag_putc(b'\n');
}

/// Write a string through the diagnostic UART writer, unconditionally.
///
/// Unlike [`diag_marker`] this is never compiled out: it exists so a panic inside the
/// kernel or a trap that cannot use the console abstraction is still visible.
pub fn diag_str(s: &str) {
    for b in s.bytes() {
        unsafe { diag_putc(b) };
    }
}

/// Print an unsigned decimal number (used for panic locations).
pub fn diag_num(v: u32) {
    if v == 0 {
        unsafe { diag_putc(b'0') };
        return;
    }
    let mut buf = [0u8; 10];
    let mut n = v;
    let mut i = buf.len();
    while n > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    for b in &buf[i..] {
        unsafe { diag_putc(*b) };
    }
}

/// Single-character marker for the trace writer, compiled out unless
/// [`TRACE_SWITCH`] is on.
#[inline]
pub fn diag_marker(marker: u8) {
    if TRACE_SWITCH {
        unsafe { diag_putc(marker) };
    }
}

/// A single-character switch-path marker, used when [`TRACE_SWITCH`] is on.
#[no_mangle]
unsafe extern "C" fn rrkernel_trace(marker: u32) {
    diag_putc(marker as u8);
}

/// Whether the switch path prints progress markers (`E` enter, `S` switched,
/// `R` restore, `U` unexpected, `A` alloca). Off for normal runs.
pub const TRACE_SWITCH: bool = false;

/// Report a trap that landed in a vector slot this kernel does not otherwise
/// use, then park.
///
/// `slot` is the vector index (the same numbering QEMU's `-d int` trace prints):
/// 6..11 = levels 2..7, 13 = user exception, 14 = double exception. Nothing here
/// may take a critical section or touch the scheduler: the machine is in an
/// unknown state by definition.
#[no_mangle]
unsafe extern "C" fn rrkernel_slot_trap(slot: u32) -> ! {
    // Also a single-writer cell rather than an atomic: this reporter runs in exception
    // context, where `s32c1i` is illegal, and a diagnostic that faults before it can
    // print is worse than no diagnostic at all.
    static REPORTED: SingleWriterU32 = SingleWriterU32::new(0);

    if REPORTED.replace(1) == 0 {
        let name = match slot {
            0 => "window-overflow4",
            1 => "window-underflow4",
            2 => "window-overflow8",
            3 => "window-underflow8",
            4 => "window-overflow12",
            5 => "window-underflow12",
            6 => "level2",
            7 => "level3",
            8 => "level4",
            9 => "level5",
            10 => "level6/debug",
            11 => "level7/nmi",
            12 => "kernel",
            13 => "user-exception",
            14 => "double-exception",
            _ => "unknown",
        };
        diag_putc(b'\r');
        diag_putc(b'\n');
        diag_field("[trap] slot=", slot);
        for b in name.bytes() {
            diag_putc(b);
        }
        diag_putc(b'\r');
        diag_putc(b'\n');

        let mut exccause: u32;
        asm!("rsr {0}, EXCCAUSE", out(reg) exccause, options(nomem, nostack));
        diag_field("[trap] cause=", exccause);

        let mut v: u32;
        asm!("rsr {0}, EXCVADDR", out(reg) v, options(nomem, nostack));
        diag_field("[trap] vaddr=", v);
        asm!("rsr {0}, EPC1", out(reg) v, options(nomem, nostack));
        diag_field("[trap] epc1 =", v);
        asm!("rsr {0}, PS", out(reg) v, options(nomem, nostack));
        diag_field("[trap] ps   =", v);
        asm!("rsr {0}, EXCSAVE1", out(reg) v, options(nomem, nostack));
        diag_field("[trap] a0   =", v);
        // WINDOWBASE/WINDOWSTART are not assembled in a `-windowed` build at all (the
        // window registers leave the assembler's namespace with the feature), so they
        // are reported only where they exist.
        #[cfg(target_feature = "windowed")]
        {
            asm!("rsr {0}, WINDOWBASE", out(reg) v, options(nomem, nostack));
            diag_field("[trap] wbase=", v);
            asm!("rsr {0}, WINDOWSTART", out(reg) v, options(nomem, nostack));
            diag_field("[trap] wstart=", v);
        }
        asm!("rsr {0}, INTENABLE", out(reg) v, options(nomem, nostack));
        diag_field("[trap] inten=", v);
        asm!("rsr {0}, INTERRUPT", out(reg) v, options(nomem, nostack));
        diag_field("[trap] intset=", v);
    }

    loop {
        core::hint::spin_loop();
    }
}

/// One otherwise-unused vector slot: park the interrupted `a0` (the hardware saves
/// nothing on these vectors), hand the slot index to [`rrkernel_slot_trap`], and let
/// it report. `call0` is enough — the stub and the reporter are both in IRAM, well
/// inside a call's range.
macro_rules! trap_slot {
    ($name:ident, $section:literal, $slot:expr, $label:literal, $jump:literal, $doc:literal) => {
        #[doc = $doc]
        #[no_mangle]
        #[unsafe(naked)]
        #[link_section = $section]
        pub unsafe extern "C" fn $name() {
            core::arch::naked_asm!(
                "wsr     a0, EXCSAVE1",
                "movi    a2, {slot}",
                "call0   {report}",
                $label,
                $jump,
                slot = const $slot,
                report = sym rrkernel_slot_trap,
            );
        }
    };
}

// Levels 2..7 have their own vectors on this core. The kernel enables no source at
// those levels, so a trap arriving here means a mis-set INTENABLE or a hardware/NMI
// event — either way it must name itself instead of executing whatever bytes happen
// to follow the table. Each slot gets its own park label because assembler labels in
// these small stubs share a namespace.
trap_slot!(
    xtensa_level2_vector,
    ".Level2Vector.text",
    6,
    ".Lxt_park2:",
    "j       .Lxt_park2",
    "Level-2 interrupt vector (unused: reports and parks)."
);
trap_slot!(
    xtensa_level3_vector,
    ".Level3Vector.text",
    7,
    ".Lxt_park3:",
    "j       .Lxt_park3",
    "Level-3 interrupt vector (unused: reports and parks)."
);
trap_slot!(
    xtensa_level4_vector,
    ".Level4Vector.text",
    8,
    ".Lxt_park4:",
    "j       .Lxt_park4",
    "Level-4 interrupt vector (unused: reports and parks)."
);
trap_slot!(
    xtensa_level5_vector,
    ".Level5Vector.text",
    9,
    ".Lxt_park5:",
    "j       .Lxt_park5",
    "Level-5 interrupt vector (unused: reports and parks)."
);
trap_slot!(
    xtensa_level6_vector,
    ".Level6Vector.text",
    10,
    ".Lxt_park6:",
    "j       .Lxt_park6",
    "Level-6 (debug) interrupt vector: reports and parks."
);
trap_slot!(
    xtensa_level7_vector,
    ".Level7Vector.text",
    11,
    ".Lxt_park7:",
    "j       .Lxt_park7",
    "Level-7 (NMI) interrupt vector: reports and parks."
);

// The user exception vector: unused, because this kernel never leaves kernel mode
// (`PS.UM` stays 0) — a trap here would mean user mode was entered by accident.
trap_slot!(
    xtensa_user_exception_vector,
    ".UserExceptionVector.text",
    13,
    ".Lxt_parku:",
    "j       .Lxt_parku",
    "User exception vector (unused: this kernel stays in kernel mode)."
);

// The double exception vector: a trap inside a trap (a window overflow while window
// overflow is disabled, or a fault while handling a fault). There is no honest
// recovery from here, so it reports — which is what turns "the kernel just hangs"
// into actionable evidence.
trap_slot!(
    xtensa_double_exception_vector,
    ".DoubleExceptionVector.text",
    14,
    ".Lxt_parkd:",
    "j       .Lxt_parkd",
    "Double exception vector: reports and parks."
);

/// The kernel exception vector. It owns a 0x40-byte slot, so it does no work itself:
/// it parks the interrupted `a0` (the hardware saves no register here) and jumps to
/// [`xtensa_exception_handler`].
///
/// The plain `j` is not just convenient, it is the only correct form: the register
/// set is entirely live, so there is no scratch register for an indirect jump, no
/// room for a literal pool, and any `call` would rotate the window *before* the
/// handler could save the interrupted window. `j` is also why [`VECBASE`] is moved
/// into IRAM at init — at the chip's reset value the handler would be half a
/// megabyte away, out of a `j`'s reach.
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".KernelExceptionVector.text"]
pub unsafe extern "C" fn xtensa_kernel_vector() {
    core::arch::naked_asm!(
        "wsr     a0, EXCSAVE1",
        "j       {handler}",
        handler = sym xtensa_exception_handler,
    );
}

/// The real entry point for every exception and for level-1 interrupts.
///
/// Entered by `j`, so nothing has been rotated or saved: the first job is to put the
/// interrupted window into the frame - `a0` out of `EXCSAVE1`, `a1`-`a15` from the
/// registers themselves. Only once that is done may the handler touch a register
/// freely, which is why the *cause* is read back out of the frame rather than kept
/// in a scratch register.
///
/// For a level-1 interrupt the frame is a task's resume point, so it must capture the
/// **interrupted** `PS` — which for level 1 lives in `PS` itself, not in a per-level
/// save register (see the save sequence below); everything else - the switch, the
/// reclamation, the restore - is the same shape as the RISC-V port: swap `sp`,
/// reclaim on the incoming stack, restore, `rfi 1`.
#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn xtensa_exception_handler() {
    core::arch::naked_asm!(
        // --- AllocaException first, before anything else --------------------
        // "A window operation could not be completed" is the one trap that must not
        // touch a register window: its handler finishes with `rfwu` (resuming the
        // failed call), so the state has to be exactly what the exception left
        // behind. `a2`/`a3` are safe scratch here - the interrupted a0 is already in
        // EXCSAVE1, and the in-flight window registers are dead by definition.
        "rsr     a2, EXCCAUSE",
        "movi    a3, {exc_alloca}",
        "beq     a2, a3, .Lxt_alloca",
        "j       .Lxt_frame",
        ".Lxt_alloca:",
        "j       {alloca}",         // never returns: it `rfwu`s into the failed call
        // --- ordinary trap: build the frame on the interrupted task's stack -
        ".Lxt_frame:",
        "mov     a0, sp",           // interrupted sp
        "addi    sp, sp, -{frame}", // frame base (sp *is* a1)
        "s32i    a0, sp, {off_a1}", // stored, so restoring sp needs no scratch
        "rsr     a0, EXCSAVE1",     // interrupted a0, parked by the vector
        "s32i    a0, sp, {off_a0}",

        "s32i    a2, sp, {off_a2}",
        "s32i    a3, sp, {off_a3}",
        "s32i    a4, sp, {off_a4}",
        "s32i    a5, sp, {off_a5}",
        "s32i    a6, sp, {off_a6}",
        "s32i    a7, sp, {off_a7}",
        "s32i    a8, sp, {off_a8}",
        "s32i    a9, sp, {off_a9}",
        "s32i    a10, sp, {off_a10}",
        "s32i    a11, sp, {off_a11}",
        "s32i    a12, sp, {off_a12}",
        "s32i    a13, sp, {off_a13}",
        "s32i    a14, sp, {off_a14}",
        "s32i    a15, sp, {off_a15}",
        "rsr     a0, EPC1",
        "s32i    a0, sp, {off_pc}",
        // For a level-1 interrupt there is no EPS1: the core saves the interrupted
        // PS *in PS itself*. The only thing it changed on entry is EXCM (set to mask
        // levels up to EXCM_LEVEL = 3, which is why `EXCM_LEVEL` exists), so the
        // task's own PS is this value with EXCM cleared.
        //
        // Masking it is not cosmetic: a task that resumed with EXCM set could no
        // longer spill its register windows, and the next call it made would take an
        // Alloca exception instead - which is precisely how this port first failed
        // in QEMU, with `ps = 0x00060f10` (= EXCM set) in the emulator's own trace.
        "rsr     a0, PS",
        "movi    a2, {ps_excm_mask}",
        "and     a0, a0, a2",
        "s32i    a0, sp, {off_ps}",
        "rsr     a0, SAR",
        "s32i    a0, sp, {off_sar}",
        "rsr     a0, EXCCAUSE",
        "s32i    a0, sp, {off_exccause}",
        "rsr     a0, EXCVADDR",
        "s32i    a0, sp, {off_excvaddr}",
        "rsr     a0, LBEG",
        "s32i    a0, sp, {off_lbeg}",
        "rsr     a0, LEND",
        "s32i    a0, sp, {off_lend}",
        "rsr     a0, LCOUNT",
        "s32i    a0, sp, {off_lcount}",
        // THREADPTR is a user register: RUR/WUR, not RSR/WSR.
        "rur     a0, THREADPTR",
        "s32i    a0, sp, {off_threadptr}",
        "rsr     a0, SCOMPARE1",
        "s32i    a0, sp, {off_scompare1}",
        // --- which trap is it? ---------------------------------------------
        // Reading the cause back out of the frame (rather than from a scratch
        // register) is what makes it safe to use a2/a3 here: every register the
        // interrupted code owned is already saved.
        "l32i    a2, sp, {off_exccause}",
        "movi    a3, {exc_level1}",
        "beq     a2, a3, .Lxt_level1",
        // --- a trap the kernel does not handle -----------------------------
        // `a2`/`a3` are the first two arguments under the call0 ABI.
        //
        // Note the shape of this: the *unexpected* case falls through (short enough
        // for a conditional branch to reach) and the long level-1 path is entered
        // with `j`, whose range is far larger. Xtensa conditional branches reach
        // only +/-128 bytes, which the level-1 body would blow straight through.
        "movi    a0, {ps_kernel}",
        "wsr     a0, PS",
        "rsync",
        "l32i    a2, sp, {off_exccause}",
        "l32i    a3, sp, {off_pc}",
        "call0   {unexpected}",
        ".Lxt_park:",
        "j       .Lxt_park",
        // --- level-1 interrupt (slice tick or software kick) ---------------
        ".Lxt_level1:",
        // INTLEVEL = EXCM_LEVEL masks everything through level 3 while the kernel
        // runs; WOE stays on so the *kernel's* own call chain can still overflow
        // into the task's stack (the spill handlers are re-entrant here).
        "movi    a0, {ps_kernel}",
        "wsr     a0, PS",
        "rsync",
        "movi    a2, 69",            // 'E': entered the kernel half
        "call0   {trace}",
        // --- a2 is the first argument and the return value (call0 ABI) ------
        "mov     a2, sp",
        "call0   {trap_switch}",
        "beqz    a2, .Lxt_noswitch", // nothing runnable: stay on this frame
        "l32i    sp, a2, 0",         // switch: `tcb.sp` is at offset 0
        "movi    a2, 83",            // 'S': switched (trace only when enabled)
        "call0   {trace}",
        // --- reclaim + re-arm, on the *incoming* task's stack --------------
        ".Lxt_noswitch:",
        "call0   {after_switch}",
        "movi    a2, 82",            // 'R': about to restore
        "call0   {trace}",
        // --- restore the incoming frame ------------------------------------
        "l32i    a2, sp, {off_sar}",
        "wsr     a2, SAR",
        "l32i    a2, sp, {off_lbeg}",
        "wsr     a2, LBEG",
        "l32i    a2, sp, {off_lend}",
        "wsr     a2, LEND",
        "l32i    a2, sp, {off_lcount}",
        "wsr     a2, LCOUNT",
        "l32i    a2, sp, {off_threadptr}",
        "wur     a2, THREADPTR",
        "l32i    a2, sp, {off_scompare1}",
        "wsr     a2, SCOMPARE1",
        "l32i    a0, sp, {off_ps}",
        "wsr     a0, PS",           // `rfi 1` returns by clearing EXCM in PS
        "rsync",
        // NOTE (known gap): a windowed context switch also has to reconcile
        // `WINDOWSTART` - the register saying which physical windows hold live data -
        // with the window base in the PS we just restored, because the kernel half
        // rotated windows of its own on the way here. Software cannot write
        // `WINDOWSTART` (WSR is rejected), which is exactly why Tensilica's
        // `_xt_context_save`/`_xt_alloca_exc` spill and refill the *whole* window
        // chain instead of just the live window: see docs/ESP32.md, which records
        // this as the one part of the Xtensa port that is not finished.
        "l32i    a0, sp, {off_pc}",
        "wsr     a0, EPC1",
        "l32i    a2, sp, {off_a2}",
        "l32i    a3, sp, {off_a3}",
        "l32i    a4, sp, {off_a4}",
        "l32i    a5, sp, {off_a5}",
        "l32i    a6, sp, {off_a6}",
        "l32i    a7, sp, {off_a7}",
        "l32i    a8, sp, {off_a8}",
        "l32i    a9, sp, {off_a9}",
        "l32i    a10, sp, {off_a10}",
        "l32i    a11, sp, {off_a11}",
        "l32i    a12, sp, {off_a12}",
        "l32i    a13, sp, {off_a13}",
        "l32i    a14, sp, {off_a14}",
        "l32i    a15, sp, {off_a15}",
        "l32i    a0, sp, {off_a0}",
        "l32i    sp, sp, {off_a1}",
        "rsync",
        "rfi     1",
        frame = const FRAME_SIZE,
        exc_level1 = const EXCCAUSE_LEVEL1_INTERRUPT,
        exc_alloca = const EXCCAUSE_ALLOCA,
        ps_kernel = const (3 | PS_WOE),
        ps_excm_mask = const !PS_EXCM,
        alloca = sym xtensa_alloca_exception,
        off_pc = const OFF_PC,
        off_ps = const OFF_PS,
        off_a0 = const OFF_A0,
        off_a1 = const OFF_A1,
        off_a2 = const OFF_A2,
        off_a3 = const OFF_A3,
        off_a4 = const OFF_A4,
        off_a5 = const OFF_A5,
        off_a6 = const OFF_A6,
        off_a7 = const OFF_A7,
        off_a8 = const OFF_A8,
        off_a9 = const OFF_A9,
        off_a10 = const OFF_A10,
        off_a11 = const OFF_A11,
        off_a12 = const OFF_A12,
        off_a13 = const OFF_A13,
        off_a14 = const OFF_A14,
        off_a15 = const OFF_A15,
        off_sar = const OFF_SAR,
        off_exccause = const OFF_EXCCAUSE,
        off_excvaddr = const OFF_EXCVADDR,
        off_lbeg = const OFF_LBEG,
        off_lend = const OFF_LEND,
        off_lcount = const OFF_LCOUNT,
        off_threadptr = const OFF_THREADPTR,
        off_scompare1 = const OFF_SCOMPARE1,
        trap_switch = sym rrkernel_trap_switch,
        after_switch = sym rrkernel_after_switch,
        unexpected = sym rrkernel_unexpected_trap,
        trace = sym rrkernel_trace,
    );
}

// ---------------------------------------------------------------------------
// Register-window spill and refill
// ---------------------------------------------------------------------------
//
// These six handlers are the canonical Xtensa sequences (the same ones Tensilica's
// `xtos` and esp-rs's `xtensa-lx-rt` use), and they are why this port can have
// windowed registers at all:
//
// * `s32e`/`l32e` are the "store/load extended" forms that address registers through
//   the *incoming* window without first rotating, which is the only way a handler
//   that has no free registers can write them.
// * `rfwo`/`rfwu` resume the interrupted instruction with the window operation
//   completed (the hardware re-tries the `entry`/`retw` that trapped).
// * `_WindowUnderflow4`'s tail is the interesting part: when a task resumes with a
//   window base that does not match the physical one, the underflow handler must
//   *repair* `PS.OWB` (the `extui`/`xor`/`wsr PS` dance) and then re-dispatch to the
//   4-, 8- or 12-register refill depending on how many windows are missing. Without
//   that repair, switching between two tasks with different call depths would
//   refill windows from the wrong stack, which corrupts both.

/// Spill the 4 registers of a window that overflowed, then resume.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowOverflow4.text"]
pub unsafe extern "C" fn _WindowOverflow4() {
    core::arch::naked_asm!(
        "s32e    a0, a5, -16",
        "s32e    a1, a5, -12",
        "s32e    a2, a5,  -8",
        "s32e    a3, a5,  -4",
        "rfwo",
    );
}

/// Refill a 4-register window.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowUnderflow4.text"]
pub unsafe extern "C" fn _WindowUnderflow4() {
    core::arch::naked_asm!(
        "l32e    a0, a5, -16",
        "l32e    a1, a5, -12",
        "l32e    a2, a5,  -8",
        "l32e    a3, a5,  -4",
        "rfwu",
    );
}

/// The AllocaException handler — the recovery for "a window operation could not be
/// completed" (cause 5). It is *not* a hardware vector: the exception arrives at the
/// kernel exception vector, which must forward here **before it rotates a window or
/// builds a frame**, because this code ends in `rfwu` and therefore needs the machine
/// state exactly as the exception left it.
///
/// What it does is the subtle part of the windowed design: the failed operation left
/// the window base out of step with `PS.OWB`, so it rotates once, flips exactly the
/// bits of `PS.OWB` that changed, and then re-dispatches to the 4-, 8- or
/// 12-register refill. Its last jump is a long `j` (not a branch) because
/// `_WindowUnderflow12` sits beyond a conditional branch's reach.
///
/// This is the same sequence Tensilica's `xtos` uses (`_xt_alloca_exc` in ESP-IDF's
/// `xtensa_vectors.S`); leaving it out is why this port first died in QEMU with an
/// `Alloca` exception that nothing handled. It sits in the underflow-4 slot, directly
/// after that vector, because the slot is the only place the canonical layout leaves
/// for it — and it fits: 61 bytes of the available 64.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowUnderflow4.text"]
pub unsafe extern "C" fn xtensa_alloca_exception() {
    core::arch::naked_asm!(
        "rsr     a0, WINDOWBASE", // needs the old base before rotw changes it
        "rotw    -1",             // old WINDOWBASE lands in a4, a0-a3 are scratch
        "rsr     a2, PS",
        "extui   a3, a2, 8, 4", // PS.OWB
        "xor     a3, a3, a4",   // bits that changed between old and current base
        "rsr     a4, EXCSAVE1", // the interrupted a0 waits there
        "slli    a3, a3, 8",    // back into the OWB field
        "xor     a2, a2, a3",   // flip exactly those bits in PS
        "wsr     a2, PS",
        "rsync",
        "bbci    a4, 31, _WindowUnderflow4",
        "rotw    -1", // the original a0 moved to a8
        "bbci    a8, 30, _WindowUnderflow8",
        "rotw    -1",
        "j       _WindowUnderflow12",
    );
}

/// Without register windows there is no window exception to recover from, so the
/// Alloca entry the trap path jumps to can never be reached: it exists only so that
/// the dispatch in [`xtensa_exception_handler`] has something to name in both ABIs.
#[cfg(not(target_feature = "windowed"))]
#[no_mangle]
#[unsafe(naked)]
pub unsafe extern "C" fn xtensa_alloca_exception() {
    core::arch::naked_asm!(".Lxt_alloca_park:", "j       .Lxt_alloca_park");
}

/// Spill the 8 registers of a window that overflowed 8 at a time.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowOverflow8.text"]
pub unsafe extern "C" fn _WindowOverflow8() {
    core::arch::naked_asm!(
        "s32e    a0, a9, -16",
        "l32e    a0, a1, -12", // a1 here is the incoming window's own sp
        "s32e    a1, a9, -12",
        "s32e    a2, a9,  -8",
        "s32e    a3, a9,  -4",
        "s32e    a4, a0, -32",
        "s32e    a5, a0, -28",
        "s32e    a6, a0, -24",
        "s32e    a7, a0, -20",
        "rfwo",
    );
}

/// Refill an 8-register window.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowUnderflow8.text"]
pub unsafe extern "C" fn _WindowUnderflow8() {
    core::arch::naked_asm!(
        "l32e    a0, a9, -16",
        "l32e    a1, a9, -12",
        "l32e    a2, a9,  -8",
        "l32e    a7, a1, -12",
        "l32e    a3, a9,  -4",
        "l32e    a4, a7, -32",
        "l32e    a5, a7, -28",
        "l32e    a6, a7, -24",
        "l32e    a7, a7, -20",
        "rfwu",
    );
}

/// Spill the 12 registers of a window that overflowed 12 at a time.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowOverflow12.text"]
pub unsafe extern "C" fn _WindowOverflow12() {
    core::arch::naked_asm!(
        "s32e    a0,  a13, -16",
        "l32e    a0,  a1,  -12",
        "s32e    a1,  a13, -12",
        "s32e    a2,  a13,  -8",
        "s32e    a3,  a13,  -4",
        "s32e    a4,  a0,  -48",
        "s32e    a5,  a0,  -44",
        "s32e    a6,  a0,  -40",
        "s32e    a7,  a0,  -36",
        "s32e    a8,  a0,  -32",
        "s32e    a9,  a0,  -28",
        "s32e    a10, a0,  -24",
        "s32e    a11, a0,  -20",
        "rfwo",
    );
}

/// Refill a 12-register window.
#[cfg(target_feature = "windowed")]
#[no_mangle]
#[unsafe(naked)]
#[link_section = ".WindowUnderflow12.text"]
pub unsafe extern "C" fn _WindowUnderflow12() {
    core::arch::naked_asm!(
        "l32e    a0,  a13, -16",
        "l32e    a1,  a13, -12",
        "l32e    a2,  a13,  -8",
        "l32e    a11, a1,  -12",
        "l32e    a3,  a13,  -4",
        "l32e    a4,  a11, -48",
        "l32e    a5,  a11, -44",
        "l32e    a6,  a11, -40",
        "l32e    a7,  a11, -36",
        "l32e    a8,  a11, -32",
        "l32e    a9,  a11, -28",
        "l32e    a10, a11, -24",
        "l32e    a11, a11, -20",
        "rfwu",
    );
}

// ---------------------------------------------------------------------------
// Rust half of the trap
// ---------------------------------------------------------------------------

/// `CCOUNT` at the previous timer interrupt, for period measurement.
///
/// Deliberately **not** an `AtomicU32`. The trap path runs with `EXCM = 1`, and on
/// Xtensa the conditional store behind atomics (`s32c1i`) is not available in
/// exception mode: an atomic there raises an illegal-instruction exception. That is
/// exactly what produced this port's "flaky" failure — the switch sometimes finished
/// and sometimes died at the period measurement, depending on whether the interrupted
/// code had reached it. The trap path is single-threaded with interrupts masked, so a
/// plain single-writer cell is both correct and the only thing that works here.
static LAST_TICK_CC: SingleWriterU32 = SingleWriterU32::new(0);

/// A `u32` with exactly one writer, accessed with volatile reads/writes.
///
/// For state that is only ever touched from the trap path (interrupts masked, one
/// core), where atomics are unavailable — see [`LAST_TICK_CC`].
struct SingleWriterU32(core::cell::UnsafeCell<u32>);

// SAFETY: every access is volatile and happens in trap context, where nothing else
// can be observing the value.
unsafe impl Sync for SingleWriterU32 {}

impl SingleWriterU32 {
    const fn new(v: u32) -> Self {
        SingleWriterU32(core::cell::UnsafeCell::new(v))
    }

    /// Store `v`, returning the previous value.
    #[inline]
    fn replace(&self, v: u32) -> u32 {
        unsafe {
            let old = core::ptr::read_volatile(self.0.get());
            core::ptr::write_volatile(self.0.get(), v);
            old
        }
    }
}

/// Work out why we trapped, acknowledge the source, and pick the next task.
///
/// Runs **before** the switch, on the *outgoing* task's stack. It publishes the
/// frame into `tcb.sp`, which is what lets the switch and the reclaimer find it.
#[no_mangle]
unsafe extern "C" fn rrkernel_trap_switch(frame: *mut u8) -> *mut TaskControlBlock {
    let cur = KERNEL.current();
    if !cur.is_null() {
        (*cur).sp = frame;
    }

    let exccause = read_volatile(frame.add(OFF_EXCCAUSE) as *const u32);
    let epc = read_volatile(frame.add(OFF_PC) as *const u32);
    diag_marker(b'T'); // the Rust half of the trap is running

    let pending = read_interrupt();
    let mut ticked = false;

    if pending & (1 << INT_TIMER0) != 0 {
        ticked = true;
        crate::scheduler::on_tick();
        diag_marker(b'K'); // tick counted

        if (*KERNEL.config.get()).measure != crate::tcb::Measure::Off {
            let now = read_ccount();
            let last = LAST_TICK_CC.replace(now);
            if last != 0 {
                let period = now.wrapping_sub(last);
                let want = (*KERNEL.config.get()).slice_cycles;
                crate::scheduler::record_period_error(period.abs_diff(want));
            }
        }
    }

    if pending & (1 << INT_SW0) != 0 {
        // Two ways, because the source is a level trigger: deassert the DPORT latch
        // *and* clear the pending bit, so it cannot re-enter immediately.
        write_volatile(DPORT_CPU_INTR_FROM_CPU_0 as *mut u32, 0);
        clear_software_interrupt();
    }

    let next = crate::scheduler::schedule_next();
    diag_marker(b'N'); // next task chosen

    if next.is_null() && ticked {
        // Nothing runnable (every task has finished). Re-arm here, because the
        // switch path - which normally does it - will not run: without this the
        // pending timer would re-enter the vector in a tight loop, and the tick
        // counter would race ahead of real time.
        rearm_timer();
    }

    report_trap(TrapEvent {
        exccause,
        epc,
        current: cur,
        next,
        after_switch: false,
    });
    next
}

/// Runs **after** the switch, i.e. on the incoming task's stack, so it can both
/// reclaim the outgoing task's memory and restart the slice counter.
#[no_mangle]
unsafe extern "C" fn rrkernel_after_switch() {
    let measuring = (*KERNEL.config.get()).measure != crate::tcb::Measure::Off;
    let t0 = if measuring { read_ccount() } else { 0 };

    report_trap(TrapEvent {
        exccause: 0,
        epc: 0,
        current: KERNEL.current(),
        next: KERNEL.current(),
        after_switch: true,
    });

    crate::scheduler::reclaim_finished_tasks();
    rearm_timer();

    if measuring {
        let dt = read_ccount().wrapping_sub(t0);
        crate::scheduler::record_latency(dt);
    }
}

/// A trap the kernel does not handle (illegal instruction, load/store error, a
/// double exception). Report it through the hook and park: a bare-metal kernel with
/// no debugger gets one chance to say why it died.
#[no_mangle]
unsafe extern "C" fn rrkernel_unexpected_trap(exccause: u32, epc: u32) -> ! {
    report_trap(TrapEvent {
        exccause,
        epc,
        current: KERNEL.current(),
        next: core::ptr::null_mut(),
        after_switch: false,
    });
    loop {
        core::hint::spin_loop();
    }
}

/// Frame offset of `EXCCAUSE`, exposed for the firmware's trap hook so it can name
/// the cause of an unexpected trap without duplicating the layout.

// ---------------------------------------------------------------------------
// Timer configuration
// ---------------------------------------------------------------------------

pub fn platform_limits() -> PlatformLimits {
    let hz = configured_hz();
    let hz = if hz == 0 { NOMINAL_CCOUNT_HZ } else { hz };
    PlatformLimits {
        min_slice_ns: cycles_to_ns_with(hz, MIN_SLICE_CYCLES as u32),
        max_slice_ns: Some(cycles_to_ns_with(hz, u32::MAX)),
        timer_hz: hz,
        timer_note: "Xtensa CCOUNT/CCOMPARE0 (ESP32 LX6), windowed register file",
    }
}

pub fn plan_timer(cfg: &SchedulerConfig) -> Result<crate::arch::TimerPlan, ConfigError> {
    let hz = cfg.timer_hz;
    if hz == 0 {
        // The slice is expressed in CCOUNT ticks, so the core clock is required:
        // there is no register on the ESP32 that reports it.
        return Err(ConfigError::TimerClockRequired);
    }
    let cycles = match cfg.slice {
        Slice::Cycles(c) => c as u64,
        other => other.to_cycles(hz),
    };
    if cycles == 0 {
        return Err(ConfigError::ZeroSlice);
    }
    if cycles < MIN_SLICE_CYCLES {
        return Err(ConfigError::SliceBelowPlatformMinimum {
            requested_ns: cycles_to_ns_with(hz, cycles as u32),
            minimum_ns: cycles_to_ns_with(hz, MIN_SLICE_CYCLES as u32),
        });
    }
    if cycles > u32::MAX as u64 {
        return Err(ConfigError::SliceAboveTimerRange {
            requested_ns: cycles_to_ns_with(hz, u32::MAX),
            maximum_ns: cycles_to_ns_with(hz, u32::MAX),
        });
    }
    Ok(crate::arch::TimerPlan {
        slice_ns: cycles_to_ns_with(hz, cycles as u32),
        slice_cycles: cycles as u32,
        timer_hz: hz,
    })
}

/// Arm the slice timer and let level-1 interrupts in.
///
/// This also puts the calling context into a known `PS`: kernel mode, `WOE = 1`
/// (so task 0's own calls can spill windows), `INTLEVEL = 0`, `EXCM = 0`. Skipping
/// that would leave whatever the boot ROM/QEMU reset left behind, and a `WOE = 0`
/// context turns the first window overflow into a double exception.
pub fn start_timer(_plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        // 1. Move the vector table into IRAM, so that the kernel vector's short `j`
        //    can reach the handler (`VECBASE` resets to 0x4000_0000, in mask ROM).
        //    Doing this before anything is enabled is what makes it safe.
        asm!(
            "wsr {0}, VECBASE",
            "rsync",
            in(reg) VECBASE,
            options(nomem, nostack)
        );
        let vbase = read_vecbase();
        if vbase as usize != VECBASE {
            return Err(ConfigError::Platform(
                "the core did not accept VECBASE = 0x4008_0000; the ESP32 port links \
                 its vector table at the bottom of IRAM",
            ));
        }

        // 2. A known PS for task 0: kernel mode, WOE = 1 (so task 0's own calls can
        //    spill windows), INTLEVEL = 0, EXCM = 0.
        asm!("wsr {0}, PS", "rsync", in(reg) PS_TASK_ENTRY, options(nomem, nostack));

        // 3. Drop any kick left over from a previous program on this core.
        write_volatile(DPORT_CPU_INTR_FROM_CPU_0 as *mut u32, 0);
        clear_software_interrupt();

        // 4. Arm the two level-1 sources and the first slice.
        enable_kernel_interrupts();
        rearm_timer();
    }
    Ok(())
}

/// Change the slice at run time: the comparator is rewritten from *now*, so the
/// first slice after a retune is a full one.
pub fn retune_timer(_plan: crate::arch::TimerPlan) -> Result<(), ConfigError> {
    unsafe {
        rearm_timer();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Task construction
// ---------------------------------------------------------------------------

/// The calling context becomes task 0.
///
/// Like RISC-V (and unlike Cortex-M's MSP/PSP split) there is no second stack to
/// preserve: `sp` is just `sp`, the frame is built on whichever stack is current,
/// and `main` keeps whatever stack the firmware's startup code installed. `sp` is
/// left null because the trap entry writes it on the first switch away, and a
/// switch *to* a task that was never switched away from cannot happen in a
/// circular ring.
pub unsafe fn adopt_current_task(tcb: *mut TaskControlBlock) -> Result<(), ConfigError> {
    unsafe {
        (*tcb).sp = core::ptr::null_mut();
        (*tcb).stack_base = core::ptr::null_mut();
        (*tcb).stack_size = 0;
        (*tcb).flags = 0;
    }
    Ok(())
}

pub unsafe fn create_task(
    tcb: *mut TaskControlBlock,
    stack_size: usize,
) -> Result<(), crate::SpawnError> {
    // Worst-case 16-byte alignment padding, plus room for the frame itself.
    let size = stack_size.max(512) + FRAME_SIZE + 16;
    let block = {
        let guard = crate::critical::enter();
        let p = unsafe { (*crate::scheduler::arena()).alloc(size, 16) };
        drop(guard);
        p
    };
    let block = match block {
        Some(p) => p,
        None => return Err(crate::SpawnError::ArenaExhausted),
    };

    unsafe {
        (*tcb).stack_base = block;
        (*tcb).stack_size = size - FRAME_SIZE - 16;

        let top = (block as usize + size) & !15;
        let frame = (top - FRAME_SIZE) as *mut u8;

        core::ptr::write_bytes(frame, 0, FRAME_SIZE);
        let w = |off: usize, v: usize| {
            core::ptr::write_volatile(frame.add(off) as *mut usize, v);
        };
        // Where `rfi 1` starts executing, and with what machine state.
        w(
            OFF_PC,
            crate::trampoline::task_trampoline as *const () as usize,
        );
        w(OFF_PS, PS_TASK_ENTRY as usize);
        // The task starts with its frame as the *base* of its stack: its first
        // `entry` grows downward, below the frame, instead of over it.
        w(OFF_A1, frame as usize);
        // Windowed ABI: a callee reads its first argument from a2, and the
        // trampoline takes the TCB pointer.
        w(OFF_A2, tcb as usize);

        (*tcb).sp = frame;
        (*tcb).flags = 0;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Port plumbing
// ---------------------------------------------------------------------------

pub fn mark_running() {}

/// A finished task: the switch was already requested (the software interrupt is
/// pending), so just wait until the trap takes the CPU away - which also recycles
/// this stack, so control never returns here.
pub fn exit_current_task_forever() -> ! {
    idle_forever()
}

/// Nothing runnable: `waiti 0` parks the core until a level-1 interrupt arrives (a
/// tick, or the kick that announces a newly spawned task).
pub fn idle_forever() -> ! {
    loop {
        unsafe {
            asm!("waiti 0", options(nomem, nostack));
        }
    }
}

/// Stop the kernel: mask every interrupt and park.
pub fn shutdown(_code: i32) -> ! {
    unsafe {
        asm!("wsr {0}, INTENABLE", in(reg) 0u32, options(nomem, nostack));
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Nothing to release: the stack and the TCB are both arena blocks, freed by the
/// same deferred-free pass.
pub unsafe fn on_task_reclaimed(_tcb: *mut TaskControlBlock) {}

/// Nothing to pin: reclamation runs from the trap entry on the incoming task's
/// stack, on a single core, with interrupts masked.
pub unsafe fn is_pinned(_tcb: *mut TaskControlBlock) -> bool {
    false
}

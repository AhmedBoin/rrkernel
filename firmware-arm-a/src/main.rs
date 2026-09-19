//! rrkernel on ARM **A/R profile**, machine-facing side: boot, console, exit.
//!
//! ```text
//! # QEMU's virt machine, Cortex-A15 (the same source builds for Cortex-R):
//! cargo run -p firmware-arm-a --target armv7a-none-eabi --release
//!
//! # What the requested Cortex-R target compiles to:
//! cargo build -p firmware-arm-a --target armv7r-none-eabi --release
//! ```
//!
//! # The TrustZone dance
//! QEMU's `virt` board starts an A15 at EL3 in *secure* monitor mode, and a GICv2
//! will not signal a group-1 interrupt to a secure CPU. So `_start` detects
//! monitor mode, flips `SCR.NS` (so EL1 is non-secure), lets non-secure EL1 use
//! the physical timer (`CNTHCTL.PL1PCTEN`), and only then drops into SVC mode —
//! which is the mode the kernel's tasks run in. On a Cortex-R part with no
//! TrustZone the same code takes the "not monitor mode" branch and carries on,
//! which is why the detection is conditional rather than unconditional.

#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use rrkernel::Slice;
use rrkernel_demo::{Console, DemoConfig};

/// QEMU's `virt` generic timer runs at 62.5 MHz. `CNTFRQ` is readable on QEMU,
/// but on a real part it is often only writable from secure state, so the demo
/// states it explicitly and logs what the hardware reports.
const TIMER_HZ: u32 = 62_500_000;

/// Set to `true` to trace the port's critical sections, IRQ entry and parking over
/// the UART. The tool to reach for when the kernel appears to hang on a serial-only
/// board (and what found this port's masked-interrupts bug).
const TRACE: bool = false;

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

global_asm!(
    r#"
    .section .text._start, "ax", %progbits
    .arm
    .globl _start
_start:
    /* Capture the boot mode and SCR into callee-saved registers: they must be
     * stored *after* .bss is zeroed, or the zeroing wipes them. */
    mrs   r6, cpsr
    and   r0, r6, #0x1F
    cmp   r0, #0x16                 @ monitor mode (EL3)?
    beq   0f
    mvn   r7, #0                    @ SCR is EL3-only: mark it unreadable
    b     1f
0:
    /* In monitor mode: make EL1 non-secure and let it use the physical timer,
     * then fall through to SVC (which will be the non-secure SVC). */
    mrc   p15, 0, r7, c1, c1, 0     @ SCR
    orr   r0, r7, #1                @ SCR.NS = 1
    mcr   p15, 0, r0, c1, c1, 0

    mrc   p15, 0, r0, c14, c1, 0    @ CNTHCTL
    orr   r0, r0, #1                @ PL1PCTEN: EL1 may use CNTP_*
    mcr   p15, 0, r0, c14, c1, 0
    isb
1:
    mov   r0, #0x13                 @ SVC mode, IRQ/FIQ unmasked
    msr   cpsr_c, r0
    ldr   sp, =_stack_start

    /* Copy .data and zero .bss, exactly as on the other ports. */
    ldr   r1, =_sidata
    ldr   r2, =_sdata
    ldr   r3, =_edata
2:  cmp   r2, r3
    bcs   3f
    ldr   r0, [r1], #4
    str   r0, [r2], #4
    b     2b
3:  ldr   r2, =_sbss
    ldr   r3, =_ebss
    mov   r0, #0
4:  cmp   r2, r3
    bcs   5f
    str   r0, [r2], #4
    b     4b
5:  /* Now that .bss is zeroed, publish what we booted as. */
    ldr   r2, =BOOT_MODE
    str   r6, [r2]
    ldr   r2, =BOOT_SCR
    str   r7, [r2]

    bl    rrkernel_main
    b     .
"#
);

// ---------------------------------------------------------------------------
// Console: PL011 on QEMU virt
// ---------------------------------------------------------------------------

struct Pl011 {
    base: usize,
}

impl Console for Pl011 {
    fn write_byte(&self, byte: u8) {
        unsafe {
            let fr = (self.base + 0x18) as *const u32;
            let dr = self.base as *mut u32;
            // TXFF (bit 5): wait for room so nothing is lost.
            while core::ptr::read_volatile(fr) & (1 << 5) != 0 {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile(dr, byte as u32);
        }
    }
}

static CONSOLE: Pl011 = Pl011 { base: 0x0900_0000 };

/// Print through this firmware's UART, bypassing the demo's console slot (which
/// does not exist until `rrkernel_demo::run` starts).
fn uart_print(args: core::fmt::Arguments<'_>) {
    struct W;
    impl core::fmt::Write for W {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            for b in s.bytes() {
                CONSOLE.write_byte(b);
            }
            Ok(())
        }
    }
    use core::fmt::Write;
    let _ = W.write_fmt(args);
}

// ---------------------------------------------------------------------------
// Exit: ARM semihosting (needs `-semihosting` in the QEMU runner)
// ---------------------------------------------------------------------------

/// Ask QEMU to terminate, with the verdict as the process exit code.
///
/// Uses the **legacy** 32-bit semihosting call (`SYS_EXIT`, 0x18): the extended
/// variant (0x20) with an `ADP_Stopped_ApplicationExit` block is what 64-bit
/// semihosting expects, while the 32-bit OABI path reads the reason code directly
/// from `r1` — and QEMU exits with 0 for `ADP_Stopped_ApplicationExit` and
/// non-zero otherwise, which is exactly the signal a test run wants. On real
/// hardware this would be a reset instead.
fn semihost_exit(code: i32) -> ! {
    const ADP_STOPPED_APPLICATION_EXIT: usize = 0x20026;
    const ADP_STOPPED_RUNTIME_ERROR: usize = 0x20023;
    let reason = if code == 0 {
        ADP_STOPPED_APPLICATION_EXIT
    } else {
        ADP_STOPPED_RUNTIME_ERROR
    };
    unsafe {
        asm_semihost(0x18, reason);
    }
    loop {
        core::hint::spin_loop();
    }
}

/// `bkpt 0xAB` with r0 = operation, r1 = argument.
#[inline(always)]
unsafe fn asm_semihost(op: usize, arg: usize) {
    core::arch::asm!(
        "mov r0, {op}",
        "mov r1, {arg}",
        "bkpt 0xAB",
        op = in(reg) op,
        arg = in(reg) arg,
        // The debug stub may clobber anything, and it is a memory-visible operation.
        out("r0") _, out("r1") _,
    );
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

struct ArenaCell(core::cell::UnsafeCell<[u8; ARENA_BYTES]>);

// SAFETY: handed out exactly once, at boot, before any task exists.
unsafe impl Sync for ArenaCell {}

impl ArenaCell {
    const fn new() -> Self {
        ArenaCell(core::cell::UnsafeCell::new([0; ARENA_BYTES]))
    }
    fn take(&self) -> &'static mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.0.get() as *mut u8, ARENA_BYTES) }
    }
}

const ARENA_BYTES: usize = 32 * 1024;
static ARENA: ArenaCell = ArenaCell::new();

/// Boot mode (CPSR) captured by `_start`, so the demo can report which GIC
/// configuration this CPU needs. Written from assembly before any Rust runs, hence
/// the atomic.
#[no_mangle]
pub static BOOT_MODE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// `SCR` captured by `_start` (`0xFFFF_FFFF` when it was not readable, i.e. we did
/// not boot in monitor mode).
#[no_mangle]
pub static BOOT_SCR: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[no_mangle]
pub extern "C" fn rrkernel_main() -> ! {
    use core::sync::atomic::Ordering;
    let cpsr = BOOT_MODE.load(Ordering::Relaxed);
    let scr = BOOT_SCR.load(Ordering::Relaxed);
    // Print through our own UART: the demo's console slot is only installed once
    // `rrkernel_demo::run` starts, and this happens before that.
    uart_print(format_args!(
        "boot     : cpsr {cpsr:#010x} (mode {:#04x}), scr {scr:#010x}\r\n",
        cpsr & 0x1F
    ));
    if TRACE {
        rrkernel::arch::set_trace_hook(trace_log);
    }
    rrkernel_demo::run(DemoConfig {
        console: &CONSOLE,
        slice: Slice::Millis(1),
        timer_hz: TIMER_HZ,
        arena: ARENA.take(),
        stack_size: 4096,
        observe_ticks: 200,
        exit: semihost_exit,
    })
}

/// Print the first few trace events. Tags are documented on
/// `rrkernel::arch::set_trace_hook`.
fn trace_log(tag: usize, value: usize) {
    use core::sync::atomic::{AtomicUsize, Ordering};
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    if !TRACE {
        return;
    }
    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    if n >= 120 {
        return;
    }
    let name = match tag {
        1 => "critical_enter  cpsr",
        2 => "  -> token        ",
        3 => "critical_exit   tok",
        4 => "  -> cpsr         ",
        5 => "start_timer done   ",
        6 => "IRQ id             ",
        7 => "parking, cpsr      ",
        _ => "?                  ",
    };
    rrkernel_demo::print(format_args!("[trace {n}] {name} {value:#010x}\r\n"));
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    rrkernel_demo::print(format_args!("\r\nPANIC\r\n"));
    semihost_exit(70)
}

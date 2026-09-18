//! rrkernel on the classic **ESP32** (Xtensa LX6, dual-core), under Espressif's
//! QEMU:
//!
//! ```text
//! rustup run esp cargo build -p firmware-esp32 -Zbuild-std=core --release \
//!     --target xtensa-esp32-none-elf
//! tools\qemu-esp32\qemu\bin\qemu-system-xtensa.exe -nographic -machine esp32 \
//!     -global driver=timer.esp32.timg,property=wdt_disable,value=true \
//!     -kernel target\xtensa-esp32-none-elf\release\firmware-esp32
//! ```
//!
//! Upstream QEMU has no ESP32 machine (`-cpu esp32` there is an error); only
//! Espressif's fork does, and it loads a bare-metal ELF directly through `-kernel`,
//! planting a stub at the reset vector that jumps to our entry point. See
//! `docs/ESP32.md`.
//!
//! The *task code* is `rrkernel-demo`, shared with every other architecture this
//! kernel supports; only the boot path, the console and the exit path below are
//! ESP32-specific.

#![no_std]
#![no_main]
// Xtensa assembly is still gated in rustc; this crate only ever builds for
// `xtensa-esp32-none-elf`, so the feature is unconditional here.
#![feature(asm_experimental_arch)]

use core::arch::global_asm;
use core::panic::PanicInfo;
use rrkernel::Slice;
use rrkernel_demo::{Console, DemoConfig};

/// `CCOUNT` runs at the CPU clock, so this is the slice's clock too: a 1 ms slice
/// is 240 000 comparator ticks.
const CCOUNT_HZ: u32 = 240_000_000;

/// Set to `true` to log traps over the UART — the switch to flip when the kernel
/// appears to hang on the way up. Off by default: a hook that prints from inside the
/// trap path interleaves with what the interrupted task was printing.
const TRAP_LOG: bool = false;

/// `RTC_CNTL_OPTIONS0_REG`: bit 31 is `SW_SYS_RST`, the chip's software reset.
/// QEMU's ESP32 machine turns that into a system reset, and with `-no-reboot` QEMU
/// exits — which is how this firmware makes the demo's verdict the process status.
const RTC_CNTL_OPTIONS0: usize = 0x3FF4_8000;

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

global_asm!(
    r#"
    .section .text._start, "ax", @progbits
    .globl _start
    .type  _start, @function
_start:
    /* PS: kernel mode, WOE = 1 (window overflow must be a spillable trap, not a
     * double exception), INTLEVEL = 0. Whatever QEMU's boot stub left behind, we
     * cannot call anything until this is known. */
    movi    a2, 0x00040000          /* PS_WOE */
    wsr     a2, PS
    rsync

    movi    sp, _stack_start

    /* Copy .data (its load address equals its vaddr here, so the loop is a no-op
     * that keeps the script honest for flash-resident builds). */
    movi    a2, _sidata
    movi    a3, _sdata
    movi    a4, _edata
.Lstart_copy:
    beq     a3, a4, .Lstart_copy_done
    l32i    a5, a2, 0
    s32i    a5, a3, 0
    addi    a2, a2, 4
    addi    a3, a3, 4
    j       .Lstart_copy
.Lstart_copy_done:
    /* Zero .bss: KERNEL's current_tcb must be null before anything runs. */
    movi    a3, _sbss
    movi    a4, _ebss
    movi    a5, 0
.Lstart_bss:
    beq     a3, a4, .Lstart_bss_done
    s32i    a5, a3, 0
    addi    a3, a3, 4
    j       .Lstart_bss
.Lstart_bss_done:
    call0   rrkernel_main
    /* rrkernel_main never returns; park rather than executing garbage. */
.Lstart_park:
    j       .Lstart_park
"#
);

// ---------------------------------------------------------------------------
// Console: UART0
// ---------------------------------------------------------------------------

/// ESP32 UART0, in the classic register layout: a 128-byte TX FIFO at offset 0 and a
/// status register at 0x1C whose bits 16..23 count what is still queued.
struct Uart0 {
    base: usize,
}

impl Console for Uart0 {
    fn write_byte(&self, byte: u8) {
        unsafe {
            let status = (self.base + 0x1C) as *const u32;
            // Never let the FIFO fill: it overflows silently otherwise.
            while core::ptr::read_volatile(status) & 0x00FF_0000 >= (128 << 16) {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile(self.base as *mut u32, byte as u32);
        }
    }
}

static CONSOLE: Uart0 = Uart0 { base: 0x3FF4_0000 };

// ---------------------------------------------------------------------------
// Arena
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

const ARENA_BYTES: usize = 48 * 1024;
static ARENA: ArenaCell = ArenaCell::new();

#[no_mangle]
pub extern "C" fn rrkernel_main() -> ! {
    // Bring-up hook: this firmware has no debugger attached.
    rrkernel::arch::set_trap_hook(trap_log);

    rrkernel_demo::run(DemoConfig {
        console: &CONSOLE,
        slice: Slice::Millis(1),
        timer_hz: CCOUNT_HZ,
        arena: ARENA.take(),
        stack_size: 4096,
        observe_ticks: 200,
        exit: qemu_exit,
    })
}

/// Print the first few traps, then periodic summaries — the fastest way to see *why*
/// an ESP32 hangs, since there is no debugger in the loop.
fn trap_log(ev: rrkernel::arch::TrapEvent) {
    if !TRAP_LOG {
        return;
    }
    use core::sync::atomic::{AtomicU32, Ordering};
    static COUNT: AtomicU32 = AtomicU32::new(0);
    static PARKS: AtomicU32 = AtomicU32::new(0);

    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    if !ev.after_switch && ev.next.is_null() {
        let k = PARKS.fetch_add(1, Ordering::Relaxed);
        if k < 4 {
            let st = rrkernel::scheduler::stats();
            rrkernel_demo::print(format_args!(
                "[{n}] NOTHING RUNNABLE active={} ticks={} switches={} epc={:#010x} cause={}\r\n",
                st.active_threads, st.ticks, st.switches, ev.epc, ev.exccause
            ));
        }
        return;
    }
    if n < 12 {
        rrkernel_demo::print(format_args!(
            "[{}] cause={} epc={:#010x} next={:p} core={}\r\n",
            n,
            ev.exccause,
            ev.epc,
            ev.next,
            rrkernel::arch::core_id()
        ));
    }
}

/// Stop the emulator (or reboot the chip) by asking for a software system reset.
fn qemu_exit(_code: i32) -> ! {
    unsafe {
        core::ptr::write_volatile(RTC_CNTL_OPTIONS0 as *mut u32, 1 << 31);
    }
    // If the reset does not take (a chip without the emulated RTC_CNTL, or a QEMU
    // without `-no-reboot`), park instead of restarting the demo endlessly.
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Unconditional, and through the port's own writer: the console is FIFO-counted and
    // may itself be what failed.
    rrkernel::arch::diag_str("\r\nPANIC");
    if let Some(loc) = _info.location() {
        rrkernel::arch::diag_str(" at ");
        rrkernel::arch::diag_str(loc.file());
        rrkernel::arch::diag_str(":");
        rrkernel::arch::diag_num(loc.line());
    }
    rrkernel::arch::diag_str("\r\n");
    qemu_exit(70)
}

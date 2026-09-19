//! rrkernel on RISC-V 32-bit (RV32IMAC), machine mode, for QEMU.
//!
//! ```text
//! cargo run -p firmware-riscv --target riscv32imac-unknown-none-elf --release
//! ```
//!
//! Runs on QEMU's `virt` machine (an ns16550a UART at 0x1000_0000 and a SiFive
//! test finisher at 0x0010_0000 are all this firmware needs); `sifive_e` works
//! with a different `link.x` memory map.
//!
//! The *task code* is `rrkernel-demo`, shared with every other architecture this
//! kernel supports — only the boot path, the console and the exit path below are
//! RISC-V-specific.

#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use rrkernel::Slice;
use rrkernel_demo::{Console, DemoConfig};

/// Tick source: QEMU's `virt` runs `mtime` at 10 MHz. A real SiFive/CH32V board
/// has a different value in its datasheet — change it here (and the kernel turns
/// the requested slice into ticks with it).
const MTIME_HZ: u32 = 10_000_000;

/// Set to `true` to log traps over the UART — the switch to flip when bringing up
/// a board where the kernel appears to hang. Off by default: a hook that prints
/// from inside the trap path interleaves with whatever the interrupted task was
/// printing, which makes console output confusing (and once fooled this port's
/// author into thinking a working kernel had hung).
const TRAP_LOG: bool = false;

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

global_asm!(
    r#"
    .section .text._start, "ax", %progbits
    .globl _start
    .type  _start, @function
_start:
    /* Machine mode, interrupts still off (mstatus.MIE = 0 after reset). */
    la    sp, _stack_start
    la    t0, _sidata
    la    t1, _sdata
    la    t2, _edata
    beq   t1, t2, 2f
1:  lw    t3, 0(t0)
    sw    t3, 0(t1)
    addi  t0, t0, 4
    addi  t1, t1, 4
    blt   t1, t2, 1b
2:
    /* Zero .bss: KERNEL's current_tcb must be null before anything runs. */
    la    t1, _sbss
    la    t2, _ebss
    beq   t1, t2, 4f
3:  sw    zero, 0(t1)
    addi  t1, t1, 4
    blt   t1, t2, 3b
4:
    call  rrkernel_main
    /* rrkernel_main never returns; park rather than executing garbage. */
5:  wfi
    j     5b
"#
);

// ---------------------------------------------------------------------------
// Console: ns16550a on QEMU virt
// ---------------------------------------------------------------------------

/// 16550-compatible UART, as QEMU's `virt` machine maps it.
struct Uart16550 {
    base: usize,
}

impl Console for Uart16550 {
    fn write_byte(&self, byte: u8) {
        unsafe {
            // Wait for THR to be empty (LSR bit 5) so QEMU never drops a byte.
            while core::ptr::read_volatile((self.base + 5) as *const u8) & 0x20 == 0 {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile(self.base as *mut u8, byte);
        }
    }
}

static CONSOLE: Uart16550 = Uart16550 { base: 0x1000_0000 };

// ---------------------------------------------------------------------------
// Exit: QEMU's SiFive test finisher
// ---------------------------------------------------------------------------

/// Ask QEMU to terminate the VM, so `cargo run` exits with the verdict as its
/// status (0 = all checks passed, 1 = a check failed). On real hardware this
/// would be a reset instead.
fn qemu_exit(code: i32) -> ! {
    const FINISHER: usize = 0x0010_0000;
    let value: u32 = if code == 0 {
        0x5555
    } else {
        ((code as u32) << 16) | 0x3333
    };
    unsafe {
        core::ptr::write_volatile(FINISHER as *mut u32, value);
    }
    // Only reached if there is no finisher (real hardware): park.
    loop {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

/// Kernel arena: TCBs, closure blobs and every task stack come from here, because
/// on bare metal a task's stack is an arena block.
struct ArenaCell(core::cell::UnsafeCell<[u8; ARENA_BYTES]>);

// SAFETY: handed out exactly once, at boot, before any task exists.
unsafe impl Sync for ArenaCell {}

impl ArenaCell {
    const fn new() -> Self {
        ArenaCell(core::cell::UnsafeCell::new([0; ARENA_BYTES]))
    }
    /// The arena as the kernel wants it. Call once, at boot.
    fn take(&self) -> &'static mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.0.get() as *mut u8, ARENA_BYTES) }
    }
}

const ARENA_BYTES: usize = 32 * 1024;
static ARENA: ArenaCell = ArenaCell::new();

#[no_mangle]
pub extern "C" fn rrkernel_main() -> ! {
    // Bring-up hook: log the first traps over the UART, since this firmware has
    // no debugger attached. Harmless when the demo works (fires a handful of
    // times) and decisive when it does not.
    rrkernel::arch::set_trap_hook(trap_log);

    rrkernel_demo::run(DemoConfig {
        console: &CONSOLE,
        slice: Slice::Millis(1),
        timer_hz: MTIME_HZ,
        arena: ARENA.take(),
        stack_size: 4096,
        observe_ticks: 200,
        exit: qemu_exit,
    })
}

/// Print the first few traps, then periodic summaries — and *every* event where
/// the scheduler found nothing to run, which is the state that starves the timer.
fn trap_log(ev: rrkernel::arch::TrapEvent) {
    use core::sync::atomic::{AtomicUsize, Ordering};
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    static PARKS: AtomicUsize = AtomicUsize::new(0);

    if !TRAP_LOG {
        return;
    }
    let n = COUNT.fetch_add(1, Ordering::Relaxed);

    if !ev.after_switch && ev.next.is_null() {
        let k = PARKS.fetch_add(1, Ordering::Relaxed);
        if k < 6 {
            let st = rrkernel::scheduler::stats();
            rrkernel_demo::print(format_args!(
                "[{n}] NOTHING RUNNABLE: active={} total={} ticks={} swaps={} head={:p} mepc={:#010x}\r\n",
                st.active_threads, st.total_threads, st.ticks, st.switches,
                unsafe { *rrkernel::KERNEL.ring_head.get() }, ev.mepc
            ));
        }
        return;
    }

    if n < 6 {
        let phase = if ev.after_switch {
            "post-switch"
        } else {
            "trap       "
        };
        rrkernel_demo::print(format_args!(
            "[{}] {} cause={:#x} mepc={:#010x} next={:p}\r\n",
            n, phase, ev.mcause, ev.mepc, ev.next
        ));
    } else if n % 2048 == 0 {
        let st = rrkernel::scheduler::stats();
        rrkernel_demo::print(format_args!(
            "[{n}] active={} ticks={} switches={} ring:",
            st.active_threads, st.ticks, st.switches
        ));
        unsafe {
            let head = *rrkernel::KERNEL.ring_head.get();
            let mut p = head;
            for _ in 0..8 {
                if p.is_null() {
                    break;
                }
                rrkernel_demo::print(format_args!(
                    " [id {} {:?} slices {}]",
                    (*p).id,
                    (*p).state,
                    (*p).slices_run
                ));
                p = (*p).next;
                if p == head {
                    break;
                }
            }
        }
        rrkernel_demo::print(format_args!("\r\n"));
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Print something useful, then exit QEMU non-zero so a test run fails loudly.
    rrkernel_demo::print(format_args!("\r\nPANIC\r\n"));
    qemu_exit(70)
}

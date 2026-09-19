//! Architecture-agnostic multi-core support: core identity, cross-core interlocks
//! and inter-processor interrupts.
//!
//! Everything above this file — scheduler, ring, arena, `Mutex` — is written once and
//! runs on 1..N cores, because the only genuinely per-core facts are reduced to
//! [`CpuArch`]:
//!
//! | Need | RISC-V | ARM A/R | Xtensa (ESP32) | Host |
//! |---|---|---|---|---|
//! | core identity | `mhartid` | `MPIDR` | `PRID` | OS thread |
//! | interlock | `amoswap.w` / `lr.w`+`sc.w` | `LDREX`/`STREX` | `S32C1I` | `Atomic*` |
//! | kick another core | CLINT `msip[hart]` | GIC `SGIR` | `DPORT` latch | none |
//!
//! # Why `spinlock_acquire` is not handed an `AtomicBool`
//!
//! The design sketch says `spinlock_acquire(lock: &AtomicBool)`, and that is exactly
//! right where byte atomics exist — ARM has `LDREXB`/`STREXB`. It is not universal:
//! RISC-V's `A` extension gives 32-bit AMOs and nothing narrower, Xtensa's `S32C1I`
//! is a conditional *word* store, and AVR / `riscv32imc` have no atomic
//! read-modify-write at all. The interlock is therefore typed as [`LockWord`], a
//! per-target alias for what the hardware can actually do; where there is no atomic
//! RMW the lock degrades to a local interrupt mask — correct on one core, which is
//! why [`CpuArch::supports_smp`] is `false` there.
//!
//! Local interrupt masking is deliberately *not* in the trait: masking is the same
//! operation on every target (the port's `critical_enter`), while the interlock is
//! not. [`SpinLock`] applies the mask around the trait call.

use core::sync::atomic::Ordering;

/// Interrupt state saved by a critical section (the port's own token type).
pub type IrqToken = crate::arch::CriticalToken;

/// Upper bound on cores this kernel accepts, used to size per-core tables.
pub const MAX_ACTIVE_CORES: usize = 8;

/// The per-architecture primitives a multi-core kernel needs — and nothing else.
///
/// Implemented for RISC-V, ARM (A/R profile), Xtensa, the hosted backends, and a
/// single-core fallback for targets that cannot support SMP.
pub trait CpuArch {
    /// Which physical core is executing this code, 0-based.
    fn current_core_id() -> usize;

    /// How many cores the *chip* has. Validates `KernelConfig::active_cores` before
    /// anything is brought up.
    fn max_cores() -> usize;

    /// Take the interlock. Local interrupts are already masked by [`SpinLock`].
    fn spinlock_acquire(lock: &LockWord);

    /// Release the interlock.
    fn spinlock_release(lock: &LockWord);

    /// Ask another core to reschedule (`target_core` is a core id, not a mask).
    fn send_ipi(target_core: usize);

    /// Whether more than one core can run concurrently on this target.
    fn supports_smp() -> bool;
}

// ---------------------------------------------------------------------------
// The lock word: what the hardware can actually do atomically
// ---------------------------------------------------------------------------
//
// These arms must stay mutually exclusive: ARM has 8-, 16-, 32- *and* 64-bit
// atomics, so "has 32-bit atomics" alone would match several of them at once.

/// ARM: byte atomics exist (`LDREXB`/`STREXB`), so the sketch's `AtomicBool` works.
#[cfg(all(target_arch = "arm", target_has_atomic = "8"))]
pub type LockWord = core::sync::atomic::AtomicBool;

/// Xtensa: `S32C1I` is a conditional *word* store, so there is no byte atomic.
#[cfg(all(target_arch = "xtensa", target_has_atomic = "32"))]
pub type LockWord = core::sync::atomic::AtomicU32;

/// RISC-V: the `A` extension provides 32-bit AMOs and nothing narrower.
#[cfg(all(target_arch = "riscv32", target_has_atomic = "32"))]
pub type LockWord = core::sync::atomic::AtomicUsize;

/// Hosted backends.
#[cfg(all(feature = "std", target_has_atomic = "ptr"))]
pub type LockWord = core::sync::atomic::AtomicUsize;

/// Targets with **no atomic read-modify-write** (AVR, `riscv32imc`).
#[cfg(not(any(
    all(target_arch = "arm", target_has_atomic = "8"),
    all(target_arch = "xtensa", target_has_atomic = "32"),
    all(target_arch = "riscv32", target_has_atomic = "32"),
    all(feature = "std", target_has_atomic = "ptr"),
)))]
pub type LockWord = SingleCoreLockWord;

/// A `bool` with no atomics behind it, for targets that cannot do atomic RMW.
///
/// Acquire/release become "the caller masked interrupts, so test and set" — correct
/// mutual exclusion on one core, which is exactly the guarantee this kernel offers
/// there.
#[cfg(not(any(
    all(target_arch = "arm", target_has_atomic = "8"),
    all(target_arch = "xtensa", target_has_atomic = "32"),
    all(target_arch = "riscv32", target_has_atomic = "32"),
    all(feature = "std", target_has_atomic = "ptr"),
)))]
pub struct SingleCoreLockWord(core::cell::UnsafeCell<bool>);

#[cfg(not(any(
    all(target_arch = "arm", target_has_atomic = "8"),
    all(target_arch = "xtensa", target_has_atomic = "32"),
    all(target_arch = "riscv32", target_has_atomic = "32"),
    all(feature = "std", target_has_atomic = "ptr"),
)))]
unsafe impl Sync for SingleCoreLockWord {}

#[cfg(not(any(
    all(target_arch = "arm", target_has_atomic = "8"),
    all(target_arch = "xtensa", target_has_atomic = "32"),
    all(target_arch = "riscv32", target_has_atomic = "32"),
    all(feature = "std", target_has_atomic = "ptr"),
)))]
impl SingleCoreLockWord {
    pub const fn new(v: bool) -> Self {
        SingleCoreLockWord(core::cell::UnsafeCell::new(v))
    }
    #[inline]
    pub fn load(&self, _o: Ordering) -> bool {
        unsafe { core::ptr::read_volatile(self.0.get()) }
    }
    #[inline]
    pub fn store(&self, v: bool, _o: Ordering) {
        unsafe { core::ptr::write_volatile(self.0.get(), v) }
    }
}

/// Construct the lock word for this target (the one place that needs the cfgs).
///
/// Two arms rather than one: the atomic word is a `bool` on ARM and an integer
/// everywhere else, so the initial value has to match the type.
#[cfg(any(
    all(target_arch = "arm", target_has_atomic = "8"),
    not(any(
        all(target_arch = "arm", target_has_atomic = "8"),
        all(target_arch = "xtensa", target_has_atomic = "32"),
        all(target_arch = "riscv32", target_has_atomic = "32"),
        all(feature = "std", target_has_atomic = "ptr"),
    )),
))]
const fn lock_word_new() -> LockWord {
    LockWord::new(false)
}

#[cfg(all(
    not(all(target_arch = "arm", target_has_atomic = "8")),
    any(
        all(target_arch = "arm", target_has_atomic = "8"),
        all(target_arch = "xtensa", target_has_atomic = "32"),
        all(target_arch = "riscv32", target_has_atomic = "32"),
        all(feature = "std", target_has_atomic = "ptr"),
    ),
))]
const fn lock_word_new() -> LockWord {
    LockWord::new(0)
}

// The lock word is `bool`-shaped on ARM (byte atomics) and on the no-atomics fallback,
// and integer-shaped on RISC-V, Xtensa and the host. These two predicates are the only
// place that difference shows.

/// Predicate: the lock word is a `bool`.
#[cfg(any(
    all(target_arch = "arm", target_has_atomic = "8"),
    not(any(
        all(target_arch = "arm", target_has_atomic = "8"),
        all(target_arch = "xtensa", target_has_atomic = "32"),
        all(target_arch = "riscv32", target_has_atomic = "32"),
        all(feature = "std", target_has_atomic = "ptr"),
    )),
))]
mod word_ops {
    use super::{LockWord, Ordering};

    /// Held-state test, for the contended path's back-off.
    #[inline]
    pub fn lock_is_held(word: &LockWord) -> bool {
        word.load(Ordering::Relaxed)
    }

    #[inline]
    pub fn lock_clear(word: &LockWord) {
        word.store(false, Ordering::Relaxed)
    }
}

/// Predicate: the lock word is an integer.
#[cfg(all(
    not(all(target_arch = "arm", target_has_atomic = "8")),
    any(
        all(target_arch = "arm", target_has_atomic = "8"),
        all(target_arch = "xtensa", target_has_atomic = "32"),
        all(target_arch = "riscv32", target_has_atomic = "32"),
        all(feature = "std", target_has_atomic = "ptr"),
    ),
))]
mod word_ops {
    use super::{LockWord, Ordering};

    /// Held-state test, for the contended path's back-off.
    #[inline]
    pub fn lock_is_held(word: &LockWord) -> bool {
        word.load(Ordering::Relaxed) != 0
    }

    #[inline]
    pub fn lock_clear(word: &LockWord) {
        word.store(0, Ordering::Relaxed)
    }
}

#[allow(unused_imports)]
use word_ops::{lock_clear, lock_is_held};

// ---------------------------------------------------------------------------
// SpinLock: interlock + local interrupt masking
// ---------------------------------------------------------------------------

/// Cross-core mutual exclusion, with local interrupts masked while held.
///
/// Both halves are required. The interlock alone lets a core take a lock, be
/// interrupted by its own timer, and then spin forever waiting for itself to release
/// it; the mask alone is useless against another core.
pub struct SpinLock {
    word: LockWord,
}

// SAFETY: the word is the target's atomic (or a cell only ever touched with
// interrupts masked, on a single-core target).
unsafe impl Sync for SpinLock {}

impl SpinLock {
    pub const fn new() -> Self {
        SpinLock {
            word: lock_word_new(),
        }
    }

    /// Acquire, masking local interrupts, and return the guard that releases it.
    #[inline]
    pub fn lock(&self) -> SpinGuard<'_> {
        let irq = unsafe { crate::arch::critical_enter() };
        <Native as CpuArch>::spinlock_acquire(&self.word);
        SpinGuard { lock: self, irq }
    }
}

/// RAII guard: dropping releases the interlock and restores the interrupt state.
#[must_use = "a spinlock is held only for the lifetime of this guard"]
pub struct SpinGuard<'a> {
    lock: &'a SpinLock,
    irq: IrqToken,
}

impl Drop for SpinGuard<'_> {
    #[inline]
    fn drop(&mut self) {
        <Native as CpuArch>::spinlock_release(&self.lock.word);
        unsafe { crate::arch::critical_exit(self.irq) };
    }
}

// ---------------------------------------------------------------------------
// Free functions, so no caller needs `#[cfg]`
// ---------------------------------------------------------------------------

/// This core's id (always 0 on single-core targets).
#[inline]
pub fn core_id() -> usize {
    <Native as CpuArch>::current_core_id()
}

/// How many cores the chip has.
#[inline]
pub fn max_cores() -> usize {
    <Native as CpuArch>::max_cores()
}

/// Whether this target can run more than one core.
#[inline]
pub fn supports_smp() -> bool {
    <Native as CpuArch>::supports_smp()
}

/// Ask another core to reschedule. No-op on single-core targets.
#[inline]
pub fn send_ipi(target_core: usize) {
    <Native as CpuArch>::send_ipi(target_core)
}

// ---------------------------------------------------------------------------
// The implementations
// ---------------------------------------------------------------------------

/// RISC-V: `mhartid`, the `A` extension, and the CLINT's `msip` array for kicks.
#[cfg(all(not(feature = "std"), target_arch = "riscv32"))]
pub struct RiscvArch;

#[cfg(all(not(feature = "std"), target_arch = "riscv32"))]
impl CpuArch for RiscvArch {
    fn current_core_id() -> usize {
        let id: usize;
        unsafe {
            core::arch::asm!("csrr {0}, mhartid", out(reg) id, options(nomem, nostack));
        }
        id
    }

    fn max_cores() -> usize {
        // The standard CLINT layout has one `msip` word per hart, and the firmware's
        // link script decides how many harts actually run.
        MAX_ACTIVE_CORES
    }

    fn spinlock_acquire(lock: &LockWord) {
        // Rust's `AtomicUsize` lowers to `lr.w`/`sc.w` (or `amoswap.w`) here, so the
        // interlock is target-native without a line of assembly.
        while lock
            .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            while lock_is_held(lock) {
                core::hint::spin_loop();
            }
        }
    }

    fn spinlock_release(lock: &LockWord) {
        lock.store(0, Ordering::Release);
    }

    fn send_ipi(target_core: usize) {
        // CLINT `msip[hart]`: one 32-bit word per hart at the CLINT base.
        const CLINT_MSIP: usize = 0x0200_0000;
        unsafe {
            core::ptr::write_volatile((CLINT_MSIP + 4 * target_core) as *mut u32, 1);
        }
    }

    fn supports_smp() -> bool {
        true
    }
}

/// ARM A/R profile: `MPIDR` for identity, `LDREXB`/`STREXB` for the interlock (Rust
/// lowers `AtomicBool` to exactly those), GICv2 `SGIR` for kicks.
#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    not(target_feature = "mclass")
))]
pub struct ArmArch;

#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    not(target_feature = "mclass")
))]
impl CpuArch for ArmArch {
    fn current_core_id() -> usize {
        let mpidr: u32;
        unsafe {
            // MPIDR: Aff0 in bits 0..7 is the core number within the cluster.
            core::arch::asm!("mrc p15, 0, {0}, c0, c0, 5", out(reg) mpidr, options(nomem, nostack));
        }
        (mpidr & 0xFF) as usize
    }

    fn max_cores() -> usize {
        MAX_ACTIVE_CORES
    }

    fn spinlock_acquire(lock: &LockWord) {
        while lock
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            while lock_is_held(lock) {
                core::hint::spin_loop();
            }
        }
    }

    fn spinlock_release(lock: &LockWord) {
        lock.store(false, Ordering::Release);
    }

    fn send_ipi(target_core: usize) {
        // GICv2 software-generated interrupt: `GICD_SGIR` with the target list in
        // bits 16..23 (one bit per core). NOTE: the receiver must also have SGI 0
        // enabled in `GICD_ISENABLER0`, which the ARM port does not yet do — no caller
        // depends on the IPI arriving while `active_cores == 1`.
        const GICD_SGIR: usize = 0x0800_0F00;
        let target_list = 1u32 << target_core;
        unsafe {
            core::ptr::write_volatile(GICD_SGIR as *mut u32, target_list << 16);
        }
    }

    fn supports_smp() -> bool {
        true
    }
}

/// Xtensa (ESP32): `PRID` for identity, `S32C1I` for the interlock, and a write to the
/// other core's `DPORT` latch for kicks.
#[cfg(all(not(feature = "std"), target_arch = "xtensa"))]
pub struct XtensaArch;

#[cfg(all(not(feature = "std"), target_arch = "xtensa"))]
impl CpuArch for XtensaArch {
    fn current_core_id() -> usize {
        crate::arch::core_id() as usize
    }

    fn max_cores() -> usize {
        // Classic ESP32: PRO and APP.
        2
    }

    fn spinlock_acquire(lock: &LockWord) {
        // Rust lowers this to the `S32C1I` sequence (which programs SCOMPARE1).
        while lock
            .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            while lock_is_held(lock) {
                core::hint::spin_loop();
            }
        }
    }

    fn spinlock_release(lock: &LockWord) {
        lock.store(0, Ordering::Release);
    }

    fn send_ipi(target_core: usize) {
        // Each core has its own `CPU_INTR_FROM_CPU_0` latch inside the DPORT block.
        const DPORT_CPU_INTR_FROM_CPU_0: usize = 0x3FF0_0000;
        unsafe {
            core::ptr::write_volatile((DPORT_CPU_INTR_FROM_CPU_0 + 4 * target_core) as *mut u32, 1);
        }
    }

    fn supports_smp() -> bool {
        true
    }
}

/// The fallback for targets with no cross-core interlock: Cortex-M in this kernel's
/// port (and, on target triples with no port at all, nothing).
///
/// M-profile only — ARM A/R has [`ArmArch`], and letting both match would define
/// `Native` twice.
#[cfg(all(not(feature = "std"), target_arch = "arm", target_feature = "mclass"))]
pub struct SingleCoreArch;

#[cfg(all(not(feature = "std"), target_arch = "arm", target_feature = "mclass"))]
impl CpuArch for SingleCoreArch {
    fn current_core_id() -> usize {
        0
    }

    fn max_cores() -> usize {
        1
    }

    fn spinlock_acquire(lock: &LockWord) {
        // The caller has masked interrupts, so this is a plain test-and-set — sound
        // mutual exclusion on one core, which is all this target has.
        while lock_is_held(lock) {
            core::hint::spin_loop();
        }
        lock.store(true, Ordering::Relaxed);
    }

    fn spinlock_release(lock: &LockWord) {
        lock_clear(lock);
    }

    fn send_ipi(_target_core: usize) {}

    fn supports_smp() -> bool {
        false
    }
}

/// Hosted backends: real OS threads, real atomics, and no way (or need) to kick one.
#[cfg(feature = "std")]
pub struct HostArch;

#[cfg(feature = "std")]
impl CpuArch for HostArch {
    fn current_core_id() -> usize {
        // A stable id per OS thread, assigned on first use. This is what lets the SMP
        // and `Mutex` logic be exercised with real threads and real races, no board.
        std::thread_local! {
            static CORE: core::cell::Cell<usize> = const { core::cell::Cell::new(usize::MAX) };
        }
        static NEXT: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
        CORE.with(|c| {
            let id = c.get();
            if id != usize::MAX {
                return id;
            }
            let id = NEXT.fetch_add(1, Ordering::Relaxed);
            c.set(id);
            id
        })
    }

    fn max_cores() -> usize {
        std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
            .min(MAX_ACTIVE_CORES)
    }

    fn spinlock_acquire(lock: &LockWord) {
        while lock
            .compare_exchange_weak(0, 1, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            // Yield rather than burn a core: the holder is a real OS thread that has
            // to be scheduled for the lock to be released.
            std::thread::yield_now();
        }
    }

    fn spinlock_release(lock: &LockWord) {
        lock.store(0, Ordering::Release);
    }

    fn send_ipi(_target_core: usize) {}

    fn supports_smp() -> bool {
        true
    }
}

// ---------------------------------------------------------------------------
// Which implementation this build uses
// ---------------------------------------------------------------------------

/// The selected implementation for the current target.
#[cfg(feature = "std")]
pub type Native = HostArch;
#[cfg(all(not(feature = "std"), target_arch = "riscv32"))]
pub type Native = RiscvArch;
#[cfg(all(
    not(feature = "std"),
    target_arch = "arm",
    not(target_feature = "mclass")
))]
pub type Native = ArmArch;
#[cfg(all(not(feature = "std"), target_arch = "xtensa"))]
pub type Native = XtensaArch;
#[cfg(all(not(feature = "std"), target_arch = "arm", target_feature = "mclass"))]
pub type Native = SingleCoreArch;

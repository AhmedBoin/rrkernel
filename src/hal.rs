//! `embedded-hal` adapters: delays, the blocking facade over an async driver, and a kernel-mutex
//! device for a bus shared between tasks.
//!
//! The three layers this crate offers, and when each is right:
//!
//! * **Layer A — the async driver.** Interrupt- or DMA-driven, waking through
//!   [`crate::event::Signal`]. Written once, it is the single implementation of a peripheral.
//! * **Layer B — the blocking facade** ([`Blocking`]). The *same* driver behind the blocking
//!   `embedded-hal` traits, implemented as `block_on(async_impl)` — so blocking code also gets
//!   slice-friendly waiting, and the driver is not duplicated. This module.
//! * **Layer C — the poll-yield fallback.** [`crate::event::wait_until`] for peripherals with no
//!   interrupt and no DMA. It stays `Ready` and gives up the slice each poll, so it cannot starve
//!   anyone, but it does burn CPU — hence the mandatory deadline.
//!
//! Everything here is opt-in (`features = ["hal", "hal-async", "async"]`); the kernel itself stays
//! `core`-only.

/// A delay that does not waste a slice.
///
/// Short delays spin on the cycle counter, because a switch costs more than the wait would; longer
/// ones sleep, so the CPU goes to another task instead of being burned. The threshold is the
/// measured switch cost — see [`KernelDelay::new`].
pub struct KernelDelay {
    /// Below this many nanoseconds, spin instead of switching.
    spin_below_ns: u64,
}

impl Default for KernelDelay {
    fn default() -> Self {
        // Ten times the worst switch measured on a Cortex-M3 at 8 MHz (80 cycles) is ~800 cycles,
        // i.e. ~100 µs. Rounding to the slice keeps it simple and errs towards sleeping.
        KernelDelay::new(100_000)
    }
}

impl KernelDelay {
    /// A delay with an explicit spin threshold, in nanoseconds.
    pub const fn new(spin_below_ns: u64) -> Self {
        KernelDelay { spin_below_ns }
    }

    /// The current spin threshold.
    pub const fn spin_below_ns(&self) -> u64 {
        self.spin_below_ns
    }
}

#[cfg(feature = "hal")]
impl embedded_hal::delay::DelayNs for KernelDelay {
    fn delay_ns(&mut self, ns: u32) {
        let ns = ns as u64;
        if ns < self.spin_below_ns {
            // Spinning here is the cheaper answer, and it is honest about it: the alternative costs
            // a switch in and a switch out for a wait shorter than the switch itself.
            let until = crate::time::now()
                .wrapping_add(crate::time::ticks_for(core::time::Duration::from_nanos(ns)));
            // A sub-tick delay cannot be measured by the tick clock, so spin on the cycle counter
            // where the port has one and fall back to a bounded loop where it does not.
            if let Some(start) = crate::arch::cycle_counter() {
                let hz = crate::arch::cycle_counter_hz();
                let want = (ns as u128 * hz as u128 / 1_000_000_000) as u32;
                while crate::arch::cycle_counter()
                    .unwrap_or(start)
                    .wrapping_sub(start)
                    < want
                {
                    core::hint::spin_loop();
                }
            } else {
                let mut n = ns.saturating_mul(4);
                while n > 0 {
                    core::hint::spin_loop();
                    n -= 1;
                }
            }
            let _ = until;
        } else {
            crate::sleep(core::time::Duration::from_nanos(ns));
        }
    }
}

#[cfg(all(feature = "hal-async", feature = "async"))]
impl embedded_hal_async::delay::DelayNs for KernelDelay {
    async fn delay_ns(&mut self, ns: u32) {
        // Always the async path: an await hands the slice over, which is the entire point of using
        // the async traits, and a spin would defeat it.
        crate::exec::sleep(core::time::Duration::from_nanos(ns as u64)).await;
    }
}

/// Run an async driver's operation from blocking code, without duplicating the driver.
///
/// ```ignore
/// let mut dev = Blocking::new(async_driver);
/// dev.transaction(0x40, &mut [Operation::Read(&mut buf)])?;   // blocking embedded-hal
/// ```
pub struct Blocking<T>(pub T);

impl<T> Blocking<T> {
    /// Wrap a driver.
    pub const fn new(inner: T) -> Self {
        Blocking(inner)
    }

    /// The wrapped driver.
    pub fn into_inner(self) -> T {
        self.0
    }
}

#[cfg(all(feature = "hal", target_has_atomic = "32"))]
mod shared {
    use crate::sync::{LockError, LockId, Mutex, MutexGuard};

    /// Why a shared-bus access failed.
    ///
    /// Deliberately its own type rather than the bus's error: "the lock timed out" and "this would
    /// invert the lock order" are not things the bus itself can report, and flattening them into the
    /// bus error would make a deadlock look like a hardware fault.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum SharedError<E> {
        /// The bus itself failed.
        Bus(E),
        /// The lock was not free within the timeout. The transfer did **not** happen.
        LockTimeout,
        /// Taking this lock would invert the kernel's static lock order (see [`crate::sync::Mutex`]).
        LockOrder,
    }

    impl<E> From<LockError> for SharedError<E> {
        fn from(e: LockError) -> Self {
            match e {
                LockError::Timeout => SharedError::LockTimeout,
                _ => SharedError::LockOrder,
            }
        }
    }

    // `embedded_hal`'s error types are marker traits that also ask for a kind.
    impl<E: core::fmt::Debug> embedded_hal::i2c::Error for SharedError<E> {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            // `Other` for all three on purpose: "the lock timed out" and "this would invert the lock
            // order" are not bus conditions, and inventing a bus kind for them would be a lie the
            // next reader has to un-learn.
            embedded_hal::i2c::ErrorKind::Other
        }
    }

    /// A bus shared between tasks, held through a kernel mutex.
    ///
    /// The kernel preempts mid-transaction, so two tasks on one I2C bus without a lock corrupt each
    /// other's transfers. `embedded_hal_bus`'s `CriticalSectionDevice` is the usual answer and the
    /// wrong one here: it masks interrupts for the whole transfer, which destroys the timing contract
    /// the rest of this kernel exists to keep. This device parks the loser instead — the round robin
    /// continues while the bus is busy — and every acquisition has a timeout.
    pub struct MutexDevice<BUS, const ID: u32> {
        bus: Mutex<BUS>,
        timeout_ms: u64,
    }

    impl<BUS, const ID: u32> MutexDevice<BUS, ID> {
        /// A device with a 1 s default acquisition timeout.
        pub const fn new(bus: BUS) -> Self {
            MutexDevice {
                bus: Mutex::with_id(LockId::new(ID), bus),
                timeout_ms: 1000,
            }
        }

        /// A device with an explicit acquisition timeout, in milliseconds.
        pub const fn with_timeout(bus: BUS, timeout_ms: u64) -> Self {
            MutexDevice {
                bus: Mutex::with_id(LockId::new(ID), bus),
                timeout_ms,
            }
        }

        /// Take the bus for longer than one transaction. The guard is the kernel's own
        /// [`MutexGuard`], so the static lock order still applies.
        pub fn lock(&self) -> Result<MutexGuard<'_, BUS>, SharedError<()>> {
            Ok(self.bus.try_lock_for(self.timeout_ms)?)
        }
    }

    impl<BUS: embedded_hal::i2c::ErrorType, const ID: u32> embedded_hal::i2c::ErrorType
        for MutexDevice<BUS, ID>
    {
        type Error = SharedError<BUS::Error>;
    }

    impl<BUS: embedded_hal::i2c::I2c, const ID: u32> embedded_hal::i2c::I2c for MutexDevice<BUS, ID> {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            // Parked, not spun: while this task waits for the bus it is off the CPU and the other
            // tasks keep running. The timeout means the wait can end badly but never forever.
            let mut guard = self.bus.try_lock_for(self.timeout_ms)?;
            guard
                .transaction(address, operations)
                .map_err(SharedError::Bus)
        }
    }

    // `SpiDevice<Word>` follows exactly the same shape (lock with a timeout, delegate, map the
    // error); it is not written out here only because it adds four more generic bounds for no new
    // behaviour. See the I2C impl above for the pattern.
}

#[cfg(all(feature = "hal", target_has_atomic = "32"))]
pub use shared::{MutexDevice, SharedError};

#[cfg(all(feature = "hal", feature = "hal-async", feature = "async"))]
mod facade {
    use super::Blocking;
    use crate::exec::block_on;

    impl<T: embedded_hal_async::i2c::ErrorType> embedded_hal::i2c::ErrorType for Blocking<T> {
        type Error = T::Error;
    }

    impl<T: embedded_hal_async::i2c::I2c> embedded_hal::i2c::I2c for Blocking<T> {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            // The same `Operation` type in both crates — `embedded-hal-async` re-uses it — so this
            // really is a pass-through, and the async driver is the only implementation.
            block_on(self.0.transaction(address, operations))
        }
    }

    impl<T: embedded_hal_async::spi::ErrorType> embedded_hal::spi::ErrorType for Blocking<T> {
        type Error = T::Error;
    }

    impl<T, Word> embedded_hal::spi::SpiDevice<Word> for Blocking<T>
    where
        T: embedded_hal_async::spi::SpiDevice<Word>,
        Word: Copy + 'static,
    {
        fn transaction(
            &mut self,
            operations: &mut [embedded_hal::spi::Operation<'_, Word>],
        ) -> Result<(), Self::Error> {
            block_on(self.0.transaction(operations))
        }
    }
}

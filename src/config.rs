//! User-facing scheduler configuration: the **time slice** you choose at init,
//! plus the knobs the port needs.
//!
//! The kernel never hardcodes a slice it cannot honour: `scheduler_init_with`
//! validates the request against the platform's real timer capabilities and
//! returns a [`ConfigError`] instead of silently scheduling something else.

use core::fmt;

/// Requested slice length, in whatever unit is natural for the target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slice {
    /// Nanoseconds. Rounded to the nearest achievable timer cycle.
    Nanos(u64),
    /// Microseconds.
    Micros(u64),
    /// Milliseconds.
    Millis(u64),
    /// Tick rate: slices per second (e.g. `Hertz(1000)` = 1 ms slices).
    Hertz(u32),
    /// Raw hardware timer cycles (Cortex-M: `SysTick` reload value; must be
    /// in `1..=0x00FF_FFFF`). Ignored by time-based host backends.
    Cycles(u32),
}

impl Slice {
    /// Convert to nanoseconds. `timer_hz` is the timer frequency, needed only
    /// for the `Cycles` variant.
    pub const fn as_nanos(self, timer_hz: u32) -> u64 {
        match self {
            Slice::Nanos(n) => n,
            Slice::Micros(us) => us.saturating_mul(1_000),
            Slice::Millis(ms) => ms.saturating_mul(1_000_000),
            Slice::Hertz(hz) => {
                if hz == 0 {
                    0
                } else {
                    1_000_000_000u64 / hz as u64
                }
            }
            Slice::Cycles(c) => {
                if timer_hz == 0 {
                    0
                } else {
                    (c as u64).saturating_mul(1_000_000_000) / timer_hz as u64
                }
            }
        }
    }

    /// Round-trip nanoseconds back to a `Slice`.
    pub const fn from_nanos(ns: u64) -> Slice {
        Slice::Nanos(ns)
    }

    /// Nanoseconds to timer cycles, rounded to *nearest* (a 1 ms request on a
    /// 168 MHz core gives exactly 168 000 cycles, not 167 999).
    pub const fn to_cycles(self, timer_hz: u32) -> u64 {
        if timer_hz == 0 {
            return 0;
        }
        let ns = self.as_nanos(timer_hz) as u128;
        let hz = timer_hz as u128;
        let cycles = (ns * hz + 500_000_000) / 1_000_000_000;
        if cycles == 0 {
            1
        } else {
            cycles as u64
        }
    }
}

impl fmt::Display for Slice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Slice::Nanos(n) => write!(f, "{} ns", n),
            Slice::Micros(n) => write!(f, "{} us", n),
            Slice::Millis(n) => write!(f, "{} ms", n),
            Slice::Hertz(n) => write!(f, "{} Hz", n),
            Slice::Cycles(n) => write!(f, "{} cycles", n),
        }
    }
}

/// Static description of what the current platform can actually do. Query it
/// before choosing a slice: [`crate::scheduler::platform_limits`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlatformLimits {
    /// Smallest slice the platform can generate. A request below this is
    /// rejected rather than clamped, because silently clamped timing is worse
    /// than a startup error.
    pub min_slice_ns: u64,
    /// Largest slice the platform can generate (`None` = unbounded).
    pub max_slice_ns: Option<u64>,
    /// Timer frequency used for the cycle <-> time conversion (0 on the
    /// time-based host backends).
    pub timer_hz: u32,
    /// Short note about the underlying timer.
    pub timer_note: &'static str,
}

/// What the user asked for, plus the port parameters it implies.
#[derive(Debug)]
pub struct SchedulerConfig {
    /// **The time slice.** Defaults to 1 ms.
    pub slice: Slice,
    /// Timer frequency in Hz.
    ///
    /// * Cortex-M: **required** — the core clock (e.g. 16_000_000). There is
    ///   no portable way for the kernel to discover it, and the SysTick reload
    ///   value depends on it.
    /// * Host backends: ignored (0 is fine); the OS timer is programmed in
    ///   real time units.
    pub timer_hz: u32,
    /// Default stack size for [`crate::thread::spawn`]. Ignored on backends
    /// where the OS owns task stacks (Win32).
    pub stack_size: usize,
    /// Optional arena override. `None` uses the built-in 64 KiB static arena.
    pub arena: Option<&'static mut [u8]>,
    /// What to do when nothing is runnable.
    pub idle: crate::tcb::IdlePolicy,
    /// Switch-latency measurement mode.
    pub measure: crate::tcb::Measure,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        SchedulerConfig {
            slice: Slice::Millis(1),
            timer_hz: 0,
            stack_size: crate::thread::DEFAULT_STACK_SIZE,
            arena: None,
            idle: crate::tcb::IdlePolicy::Wait,
            measure: crate::tcb::Measure::Off,
        }
    }
}

impl SchedulerConfig {
    /// A config with the given slice and everything else default.
    pub const fn with_slice(slice: Slice) -> Self {
        SchedulerConfig {
            slice,
            timer_hz: 0,
            stack_size: crate::thread::DEFAULT_STACK_SIZE,
            arena: None,
            idle: crate::tcb::IdlePolicy::Wait,
            measure: crate::tcb::Measure::Off,
        }
    }

    /// The bare-metal pair: slice + core clock, measuring latency in cycles.
    pub const fn embedded(slice: Slice, core_clock_hz: u32) -> Self {
        SchedulerConfig {
            slice,
            timer_hz: core_clock_hz,
            stack_size: crate::thread::DEFAULT_STACK_SIZE,
            arena: None,
            idle: crate::tcb::IdlePolicy::Wait,
            measure: crate::tcb::Measure::Cycles,
        }
    }

    #[must_use]
    pub const fn stack_size(mut self, bytes: usize) -> Self {
        self.stack_size = bytes;
        self
    }

    #[must_use]
    pub const fn idle(mut self, policy: crate::tcb::IdlePolicy) -> Self {
        self.idle = policy;
        self
    }

    #[must_use]
    pub const fn measure(mut self, mode: crate::tcb::Measure) -> Self {
        self.measure = mode;
        self
    }

    #[must_use]
    pub fn arena(mut self, region: &'static mut [u8]) -> Self {
        self.arena = Some(region);
        self
    }
}

/// Why a configuration was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// The slice resolved to zero timer cycles / zero nanoseconds.
    ZeroSlice,
    /// Bare metal and `timer_hz == 0`: the kernel cannot derive a SysTick
    /// reload without knowing the core clock.
    TimerClockRequired,
    /// The requested slice is shorter than the platform can generate.
    SliceBelowPlatformMinimum { requested_ns: u64, minimum_ns: u64 },
    /// The requested slice exceeds the timer's range (Cortex-M SysTick has a
    /// 24-bit reload: ~16.7 M cycles).
    SliceAboveTimerRange { requested_ns: u64, maximum_ns: u64 },
    /// The supplied arena cannot hold the first task.
    ArenaTooSmall { provided: usize, minimum: usize },
    /// `scheduler_init*` was called twice.
    AlreadyInitialized,
    /// A runtime retune was requested before init.
    NotInitialized,
    /// The platform refused an operation (thread/fiber creation, timer setup).
    Platform(&'static str),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            ConfigError::ZeroSlice => write!(f, "slice resolved to zero"),
            ConfigError::TimerClockRequired => write!(
                f,
                "timer_hz (core clock) must be set on bare metal: \
                 SchedulerConfig::embedded(slice, core_clock_hz)"
            ),
            ConfigError::SliceBelowPlatformMinimum {
                requested_ns,
                minimum_ns,
            } => write!(
                f,
                "requested slice {requested_ns} ns is below the platform minimum {minimum_ns} ns"
            ),
            ConfigError::SliceAboveTimerRange {
                requested_ns,
                maximum_ns,
            } => write!(
                f,
                "requested slice {requested_ns} ns exceeds the timer range (max {maximum_ns} ns)"
            ),
            ConfigError::ArenaTooSmall { provided, minimum } => write!(
                f,
                "arena of {provided} bytes is too small (need at least {minimum} bytes)"
            ),
            ConfigError::AlreadyInitialized => write!(f, "scheduler already initialized"),
            ConfigError::NotInitialized => write!(f, "scheduler not initialized"),
            ConfigError::Platform(msg) => write!(f, "platform error: {msg}"),
        }
    }
}

//! `embedded-io` byte streams driven by an interrupt, without burning a slice while idle.
//!
//! This is the layer the plan calls "ISR-driven blocking read": a peripheral's interrupt handler
//! pushes bytes into a [`Pipe`], and a task that reads from it **parks** instead of polling. The task
//! is off the CPU between bytes, the round robin continues, and every read has a deadline.
//!
//! The same [`Pipe`] backs the blocking `embedded-io` traits and the async `embedded-io-async` ones,
//! because both wait on the same [`Signal`] — so a driver is written once and offered in both forms.
//!
//! ```ignore
//! static RX: Pipe = Pipe::new();
//!
//! #[interrupt]
//! fn USART1() {
//!     while let Some(b) = read_hw_byte() {
//!         RX.push_from_isr(b);      // ISR-safe; wakes the reader
//!     }
//! }
//! ```
//!
//! Opt-in: `features = ["io"]` (blocking) or `["io-async"]` (needs `async`).

use crate::event::{Signal, Timeout};
use core::cell::UnsafeCell;

/// Capacity of a [`Pipe`]'s buffer.
///
/// A power of two, so the ring's index arithmetic is a mask. 256 bytes lets an ISR run ahead of a
/// task by a couple of hundred bytes — the usual case — and costs nothing until a `Pipe` is declared.
pub const PIPE_CAPACITY: usize = 256;

/// Why a stream operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipeError {
    /// The deadline passed before data arrived, or before the buffer had room.
    Timeout,
    /// The reader has gone away.
    Closed,
}

impl From<Timeout> for PipeError {
    fn from(_: Timeout) -> Self {
        PipeError::Timeout
    }
}

impl core::fmt::Display for PipeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PipeError::Timeout => write!(f, "stream timed out"),
            PipeError::Closed => write!(f, "stream closed"),
        }
    }
}

/// A single-producer, single-consumer byte ring that an ISR writes and a task reads.
///
/// * `push_from_isr` never blocks and never allocates. If the ring is full it drops the byte and
///   returns `false`, which is the only honest thing an interrupt handler can do — the alternative
///   is to lose an unrelated byte somewhere else.
/// * `read`/`write` **park** the calling task, so a waiting task costs no CPU. That is the property
///   the acceptance test for this layer checks: a UART echo thread blocked on RX consumes ~0 slices.
pub struct Pipe {
    buf: UnsafeCell<[u8; PIPE_CAPACITY]>,
    head: UnsafeCell<usize>,
    tail: UnsafeCell<usize>,
    data: Signal,
    space: Signal,
}

// SAFETY: the indices are only touched inside `critical::enter`, and buffer bytes are only
// read/written at indices computed inside that same section (the ISR side takes it too).
unsafe impl Sync for Pipe {}

impl Default for Pipe {
    fn default() -> Self {
        Self::new()
    }
}

impl Pipe {
    /// An empty pipe. `const`, so it can be a `static` an ISR and a task share.
    pub const fn new() -> Self {
        Pipe {
            buf: UnsafeCell::new([0; PIPE_CAPACITY]),
            head: UnsafeCell::new(0),
            tail: UnsafeCell::new(0),
            data: Signal::new(),
            space: Signal::new(),
        }
    }

    /// Bytes available to read.
    pub fn len(&self) -> usize {
        let _g = crate::critical::enter();
        let (h, t) = unsafe { (*self.head.get(), *self.tail.get()) };
        drop(_g);
        h.wrapping_sub(t)
    }

    /// Is there nothing to read?
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Free space for writing.
    pub fn room(&self) -> usize {
        PIPE_CAPACITY - self.len()
    }

    /// Push one byte. **ISR-safe.** Returns `false` if the ring was full (the byte is dropped).
    pub fn push_from_isr(&self, b: u8) -> bool {
        let ok = {
            let _g = crate::critical::enter();
            let (h, t) = unsafe { (*self.head.get(), *self.tail.get()) };
            if h.wrapping_sub(t) >= PIPE_CAPACITY {
                false
            } else {
                let idx = h & (PIPE_CAPACITY - 1);
                unsafe {
                    (*self.buf.get())[idx] = b;
                    *self.head.get() = h.wrapping_add(1);
                }
                true
            }
        };
        if ok {
            // Wake the reader. `Signal::set` takes its own section (nesting is fine); keeping the
            // critical section above short is the habit that keeps the tick honest.
            self.data.set();
        }
        ok
    }

    /// Take up to `out.len()` bytes without blocking, returning how many. ISR-safe.
    pub fn pop_into(&self, out: &mut [u8]) -> usize {
        let n = {
            let _g = crate::critical::enter();
            let (h, t) = unsafe { (*self.head.get(), *self.tail.get()) };
            let avail = h.wrapping_sub(t).min(out.len());
            for (i, slot) in out.iter_mut().take(avail).enumerate() {
                *slot = unsafe { (*self.buf.get())[t.wrapping_add(i) & (PIPE_CAPACITY - 1)] };
            }
            unsafe { *self.tail.get() = t.wrapping_add(avail) };
            avail
        };
        if n > 0 {
            // Room appeared: let a writer that parked on a full ring run again.
            self.space.set();
        }
        n
    }

    /// Copy `data` in without blocking, returning how many bytes fit. ISR-safe.
    pub fn write_from_isr(&self, data: &[u8]) -> usize {
        let n = {
            let _g = crate::critical::enter();
            let (h, t) = unsafe { (*self.head.get(), *self.tail.get()) };
            let room = PIPE_CAPACITY - h.wrapping_sub(t);
            let n = room.min(data.len());
            for (i, b) in data.iter().take(n).enumerate() {
                unsafe {
                    (*self.buf.get())[h.wrapping_add(i) & (PIPE_CAPACITY - 1)] = *b;
                }
            }
            unsafe { *self.head.get() = h.wrapping_add(n) };
            n
        };
        if n > 0 {
            self.data.set();
        }
        n
    }

    /// The signal a reader (or a poller) can wait on.
    pub fn data_signal(&self) -> &Signal {
        &self.data
    }

    /// The signal a writer can wait on when the ring is full.
    pub fn space_signal(&self) -> &Signal {
        &self.space
    }

    /// Read, **parking** until there is something to read. `deadline == None` means unbounded.
    ///
    /// This is the call that makes an idle reader free: while it waits, the task is `Blocked`, the
    /// ring moves on, and the ISR's `push_from_isr` is what brings it back.
    pub fn read_blocking(&self, out: &mut [u8], deadline: Option<u64>) -> Result<usize, PipeError> {
        loop {
            let n = self.pop_into(out);
            if n > 0 {
                return Ok(n);
            }
            self.data.wait(deadline)?;
        }
    }

    /// Write, **parking** while the ring is full.
    pub fn write_blocking(&self, data: &[u8], deadline: Option<u64>) -> Result<usize, PipeError> {
        let mut written = 0;
        while written < data.len() {
            written += self.write_from_isr(&data[written..]);
            if written == data.len() {
                break;
            }
            self.space.wait(deadline)?;
        }
        Ok(written)
    }
}

/// Deadline used by the `embedded-io` trait impls below.
///
/// Those traits cannot express a timeout, and design rule 7 says no API blocks forever without an
/// explicit opt-in — so the trait impls use a bounded default and [`Pipe::read_blocking`] exists for
/// callers that want to choose the bound themselves.
pub const DEFAULT_IO_TIMEOUT_MS: u64 = 1000;

fn default_deadline() -> u64 {
    crate::scheduler::deadline_after_ticks(crate::time::ticks_for(
        core::time::Duration::from_millis(DEFAULT_IO_TIMEOUT_MS),
    ))
}

#[cfg(feature = "io")]
mod blocking_io {
    use super::*;

    /// `embedded-io` asks for a kind as well as `Debug`/`Display`, so the mapping is explicit
    /// rather than guessed from the message.
    impl embedded_io::Error for PipeError {
        fn kind(&self) -> embedded_io::ErrorKind {
            match self {
                PipeError::Timeout => embedded_io::ErrorKind::TimedOut,
                PipeError::Closed => embedded_io::ErrorKind::BrokenPipe,
            }
        }
    }

    // `embedded_io::Error` requires `core::error::Error`, which *is* `std::error::Error` (std
    // re-exports it), so one impl covers the hosted and the bare-metal builds alike.
    impl core::error::Error for PipeError {}

    impl embedded_io::ErrorType for Pipe {
        type Error = PipeError;
    }

    impl embedded_io::Read for Pipe {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            // Bounded by `DEFAULT_IO_TIMEOUT_MS` rather than unbounded: the trait has no way to say
            // "wait forever", so this impl does not do it.
            self.read_blocking(buf, Some(default_deadline()))
        }
    }

    impl embedded_io::Write for Pipe {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_blocking(buf, Some(default_deadline()))
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            // Nothing to flush: the bytes are the buffer.
            Ok(())
        }
    }
}

#[cfg(feature = "io-async")]
mod async_io {
    use super::*;

    impl embedded_io_async::Read for Pipe {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            loop {
                let n = self.pop_into(buf);
                if n > 0 {
                    return Ok(n);
                }
                // The async twin of `Signal::wait`: each poll that finds nothing yields the slice.
                crate::exec::SignalWait::new(self.data_signal(), default_deadline()).await?;
            }
        }
    }

    impl embedded_io_async::Write for Pipe {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            loop {
                let n = self.write_from_isr(buf);
                if n > 0 {
                    return Ok(n);
                }
                crate::exec::SignalWait::new(self.space_signal(), default_deadline()).await?;
            }
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}

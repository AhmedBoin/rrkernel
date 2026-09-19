//! Arena accounting across a recycled block.
//!
//! Regression test for a bug with no memory-safety consequences and no visible symptom until
//! you look at the numbers: when a freed block was reused for a *different* size, the block's
//! recorded `payload_size` was left at the value from its original allocation. `free` reads
//! that field to decrement `live`, so every recycle at a different size moved
//! `ArenaStats::live_bytes` by the wrong amount — and with the mixed allocation sizes a real
//! spawn/exit workload produces, that drift accumulates without bound.
//!
//! Alignment and capacity were never wrong (`total` is what bounds the payload), so nothing
//! crashed: the statistics simply stopped describing the arena.
//!
//! # Why these tests keep a companion allocation live
//! A wrongly-subtracted size makes `live` *too small*, and `free` saturates at zero rather than
//! wrapping. So if the only live allocation were the block being freed, the wrong subtraction
//! would land on 0 — which is exactly the right answer. The bug would hide. Every assertion
//! below therefore pins `live` to a value that is **not** zero and cannot be reached by any
//! saturating subtraction: the exact sum of the blocks still live.

use rrkernel::arena::Arena;

/// A 4 KiB region, large enough for the sizes used below.
static mut REGION: [u8; 4096] = [0; 4096];

/// An arena over `REGION`. Callers hold the "critical section" by virtue of being a
/// single-threaded test.
unsafe fn arena() -> Arena {
    let mut a = Arena::empty();
    a.init(std::ptr::addr_of_mut!(REGION) as *mut u8, 4096);
    a
}

#[test]
fn reuse_at_a_different_size_keeps_live_bytes_exact() {
    unsafe {
        let mut a = arena();

        // The companion: 64 bytes that stay live for the whole test (see the module docs).
        let companion = a.alloc(64, 8).expect("companion");
        assert_eq!(a.stats().live_bytes, 64);

        // A large block, freed, then reused for a much smaller request.
        let big = a.alloc(512, 8).expect("512-byte alloc");
        assert_eq!(a.stats().live_bytes, 64 + 512);
        let bump_after_big = a.stats().bytes_bump;

        a.free(big);
        let s = a.stats();
        assert_eq!(s.live_bytes, 64, "freeing returns live to the companion");
        assert_eq!(s.free_blocks, 1, "the block is on the free list");
        assert_eq!(
            s.peak_live_bytes,
            64 + 512,
            "the peak is not lowered by a free"
        );

        let small = a.alloc(128, 8).expect("reuse for a smaller request");
        let s = a.stats();
        assert_eq!(small, big, "the freed block is the first fit for 128 bytes");
        assert_eq!(
            s.bytes_bump, bump_after_big,
            "reuse must not move the bump pointer (i.e. it really is the same block)"
        );
        assert_eq!(
            s.live_bytes,
            64 + 128,
            "allocating counts the *new* request size"
        );

        // The assertion the bug violated: free must subtract the size actually handed out.
        // With the stale 512 it would subtract 512 from 192 and saturate at 0, not 64.
        a.free(small);
        assert_eq!(
            a.stats().live_bytes,
            64,
            "free must subtract 128, the size this block was handed out at, not the 512 it was \
             originally carved for"
        );

        // Reuse once more at a third size, to show the size is rewritten on *every* reuse
        // rather than only the first.
        let mid = a.alloc(200, 8).expect("second reuse");
        assert_eq!(mid, big, "still the same block");
        assert_eq!(a.stats().live_bytes, 64 + 200);
        a.free(mid);
        assert_eq!(a.stats().live_bytes, 64);

        // Counters and the high-water mark are unaffected by any of this.
        let s = a.stats();
        assert_eq!(s.allocations, 4, "companion, big, small, mid");
        assert_eq!(s.frees, 3);
        assert_eq!(s.free_blocks, 1, "the 512-byte block is free again");
        assert_eq!(s.peak_live_bytes, 64 + 512);

        a.free(companion);
        assert_eq!(a.stats().live_bytes, 0, "the arena is empty again");
        assert_eq!(a.stats().frees, 4);
    }
}

/// The drift was cumulative, so exactness after the *n*th recycle of the same block is the
/// real claim. Each iteration re-requests a smaller size and checks `live` on both sides of
/// the free.
#[test]
fn repeated_recycles_at_decreasing_sizes_never_drift() {
    unsafe {
        let mut a = arena();

        let keep = a.alloc(100, 8).expect("kept live throughout");
        let big = a.alloc(1024, 8).expect("1024-byte alloc");
        assert_eq!(a.stats().live_bytes, 1124);

        a.free(big);
        assert_eq!(a.stats().live_bytes, 100);

        // First reuse of the 1024-byte block, at 256.
        let mut cur = a.alloc(256, 8).expect("reuse at 256");
        assert_eq!(cur, big, "the big block serves the smaller request");
        assert_eq!(a.stats().live_bytes, 356);

        for want in [192usize, 128, 64, 32] {
            a.free(cur);
            assert_eq!(
                a.stats().live_bytes,
                100,
                "after freeing the {want}-size reuse, only `keep` may remain live"
            );
            cur = a.alloc(want, 8).expect("reuse at a smaller size");
            assert_eq!(cur, big, "the same block is recycled every time");
            assert_eq!(
                a.stats().live_bytes,
                100 + want,
                "recycled at {want}: live must be the sum of what is actually live"
            );
        }

        a.free(cur);
        a.free(keep);
        assert_eq!(
            a.stats().live_bytes,
            0,
            "everything freed leaves nothing live"
        );
        assert_eq!(a.stats().free_blocks, 2, "both blocks are on the free list");
        assert_eq!(
            a.stats().peak_live_bytes,
            1124,
            "the high-water mark is the first, largest total"
        );
    }
}

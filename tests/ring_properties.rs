//! Property tests for the ring: random sequences of insert / unlink / state changes, each step
//! verified against a plain `Vec` model of what the ring should look like.
//!
//! Why a model instead of more hand-written cases: the ring is the one structure every port
//! mutates from several contexts (task, tick, ISR), and hand-written cases only cover the
//! interleavings somebody thought of. `tests/ring_invariants.rs` keeps the *specific* cases
//! (the ABI offsets, the sole-member unlink, `next_runnable` skipping dead nodes); this file
//! keeps the *general* claim — after any sequence of legal operations the ring is still
//! circular, still doubly linked, and still exactly the size and order the model says.
//!
//! Determinism: one seeded xorshift64 per run, and the seed is in every failure message, so a
//! failing sequence reproduces exactly. No dependency is added for this: the library's
//! zero-dependency promise is about the target image, but a dev-dependency is still a
//! supply-chain surface this crate does not need for a 20-line PRNG.
//!
//! NOTE: like `ring_invariants.rs`, these tests build TCBs by hand (`Box`es) and must not run
//! while a scheduler instance is live in the same process.

use rrkernel::ring;
use rrkernel::tcb::{TaskControlBlock, TaskState};
use std::ptr;

/// A TCB living in a `Box`, so a test can build rings without the kernel arena.
struct Node {
    tcb: Box<TaskControlBlock>,
}

impl Node {
    fn new(id: u32) -> Self {
        let mut tcb = Box::new(TaskControlBlock {
            sp: ptr::null_mut(),
            stack_base: ptr::null_mut(),
            stack_size: 0,
            closure_block: ptr::null_mut(),
            state: TaskState::Ready,
            flags: 0,
            next: ptr::null_mut(),
            prev: ptr::null_mut(),
            id,
            slice_cycles: 0,
            slices_run: 0,
            switches: 0,
            backend: ptr::null_mut(),
            blocked_on: 0,
            block_deadline: 0,
            held_locks: [0; rrkernel::tcb::MAX_HELD_LOCKS],
            held_count: 0,
            kind: rrkernel::tcb::NodeKind::Leaf,
            parent: std::ptr::null_mut(),
            children_head: std::ptr::null_mut(),
            current_child: std::ptr::null_mut(),
            remaining_cycles: 0,
        });
        // A non-null `sp` so a bug that dereferences it is more likely to be caught than to
        // silently read zero.
        let raw: *mut TaskControlBlock = &mut *tcb;
        tcb.sp = raw as *mut u8;
        Node { tcb }
    }

    fn ptr(&self) -> *mut TaskControlBlock {
        &*self.tcb as *const TaskControlBlock as *mut TaskControlBlock
    }
}

/// xorshift64*: tiny, deterministic, good enough to shuffle indices.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

fn runnable(state: TaskState) -> bool {
    !matches!(state, TaskState::Dead | TaskState::Blocked)
}

/// What `next_runnable` must return: the first runnable *successor*, and only when no successor
/// is runnable may `from` itself be returned — which is what the implementation documents, and
/// which this test exists to pin down.
fn model_next_runnable(order: &[usize], states: &[TaskState], from_pos: usize) -> Option<usize> {
    let k = order.len();
    for step in 1..=k {
        let idx = (from_pos + step) % k;
        if idx == from_pos {
            break;
        }
        if runnable(states[order[idx]]) {
            return Some(order[idx]);
        }
    }

    if runnable(states[order[from_pos]]) {
        Some(order[from_pos])
    } else {
        None
    }
}

/// State of one property run, in model terms.
struct Model {
    pool: Vec<Node>,
    /// Ring order; `order[0]` is the head. Empty means "no ring".
    order: Vec<usize>,
    states: Vec<TaskState>,
    linked: Vec<bool>,
}

impl Model {
    fn new(n: usize) -> Self {
        Model {
            pool: (0..n).map(|i| Node::new(i as u32 + 1)).collect(),
            order: Vec::new(),
            states: vec![TaskState::Ready; n],
            linked: vec![false; n],
        }
    }

    fn pos(&self, node: usize) -> Option<usize> {
        self.order.iter().position(|&n| n == node)
    }

    /// Verify every invariant this test knows about, against the model.
    fn verify(&self, seed: u64, step: usize, op: &str) -> Result<(), String> {
        let ctx = format!("seed {seed:#x}, step {step}, after {op}");

        if self.order.is_empty() {
            for (i, n) in self.pool.iter().enumerate() {
                if unsafe { (*n.ptr()).is_linked() } {
                    return Err(format!(
                        "{ctx}: node {i} is linked but the model says empty"
                    ));
                }
            }
            return Ok(());
        }

        let head = self.pool[self.order[0]].ptr();

        // 1. The structural check the kernel itself uses.
        let n = unsafe { ring::check(head) }.map_err(|e| format!("{ctx}: ring::check: {e}"))?;
        if n != self.order.len() {
            return Err(format!(
                "{ctx}: ring::check says {n} nodes, model says {}",
                self.order.len()
            ));
        }
        if unsafe { ring::len(head) } != self.order.len() {
            return Err(format!("{ctx}: ring::len disagrees with ring::check"));
        }

        // 2. Walk `next` from the head and compare the *order*, not merely the length: a ring
        //    with the right size and the wrong arrangement would pass a length check.
        let mut walked = Vec::with_capacity(self.order.len());
        let mut p = head;
        loop {
            let idx = unsafe { (*p).id } as usize - 1;
            walked.push(idx);
            p = unsafe { (*p).next };
            if p.is_null() || p == head {
                break;
            }
        }
        if walked != self.order {
            return Err(format!(
                "{ctx}: ring order {walked:?} != model {:?}",
                self.order
            ));
        }

        // 3. Membership: a node is linked exactly when the model says so.
        for (i, node) in self.pool.iter().enumerate() {
            if unsafe { (*node.ptr()).is_linked() } != self.linked[i] {
                return Err(format!(
                    "{ctx}: node {i} membership disagrees with the model"
                ));
            }
        }

        // 4. `next_runnable` from every position, including the all-blocked and all-dead cases.
        for from_pos in 0..self.order.len() {
            let from = self.pool[self.order[from_pos]].ptr();
            let got = unsafe { ring::next_runnable(from) };
            // Compare in *task ids* (1-based, as the TCBs carry them) on both sides: the model
            // works in pool indices, and mixing the two turned a genuine mismatch into a message
            // that read like a false positive ("got id 1, model says Some(1)").
            let got_id: Option<usize> = if got.is_null() {
                None
            } else {
                Some(unsafe { (*got).id } as usize)
            };
            let want_id: Option<usize> =
                model_next_runnable(&self.order, &self.states, from_pos).map(|idx| idx + 1);
            if got_id != want_id {
                return Err(format!(
                    "{ctx}: next_runnable from pos {from_pos}: got {got_id:?}, model says \
                     {want_id:?} (task ids; None means null)"
                ));
            }
        }
        Ok(())
    }
}

/// One seeded run: random legal operations, verified against the model after each step.
fn one_run(seed: u64, nodes: usize, steps: usize) -> Result<(), String> {
    let mut rng = Rng(seed);
    let mut m = Model::new(nodes);

    for step in 0..steps {
        // Mostly structural edits, sometimes state changes: the interesting failures come from
        // the combination, so neither should dominate.
        if rng.below(10) < 7 {
            let unlinked: Vec<usize> = (0..nodes).filter(|&i| !m.linked[i]).collect();
            let linked: Vec<usize> = (0..nodes).filter(|&i| m.linked[i]).collect();

            // Which edit is legal depends on the current ring: a node cannot be linked twice,
            // and an absent node cannot be unlinked.
            let do_insert = !unlinked.is_empty() && (linked.is_empty() || rng.below(2) == 0);
            if do_insert {
                let node = unlinked[rng.below(unlinked.len())];
                // `insert_after(null, n)` makes `n` a self-linked ring of one, which is what the
                // kernel does exactly when the ring is empty.
                let after = if linked.is_empty() {
                    None
                } else {
                    Some(linked[rng.below(linked.len())])
                };
                let after_ptr = match after {
                    None => ptr::null_mut(),
                    Some(a) => m.pool[a].ptr(),
                };
                unsafe { ring::insert_after(after_ptr, m.pool[node].ptr()) };
                match after {
                    None => m.order = vec![node],
                    Some(a) => {
                        let at = m.pos(a).unwrap();
                        m.order.insert(at + 1, node);
                    }
                }
                m.linked[node] = true;
                m.verify(seed, step, &format!("insert node {node} after {after:?}"))?;
            } else if !linked.is_empty() {
                let node = linked[rng.below(linked.len())];
                let at = m.pos(node).unwrap();
                let successor = unsafe { ring::unlink(m.pool[node].ptr()) };
                m.order.remove(at);
                m.linked[node] = false;
                // The returned successor is the model's next node, or null when the ring emptied.
                let want = if m.order.is_empty() {
                    None
                } else {
                    Some(m.order[at % m.order.len()])
                };
                let returned_ok = match want {
                    None => successor.is_null(),
                    Some(w) => successor == m.pool[w].ptr(),
                };
                if !returned_ok {
                    return Err(format!(
                        "seed {seed:#x}, step {step}, unlink node {node}: wrong successor returned"
                    ));
                }
                m.verify(seed, step, &format!("unlink node {node}"))?;
            }
        } else {
            let node = rng.below(nodes);
            let state = match rng.below(4) {
                0 => TaskState::Ready,
                1 => TaskState::Running,
                2 => TaskState::Blocked,
                _ => TaskState::Dead,
            };
            m.states[node] = state;
            unsafe { (*m.pool[node].ptr()).state = state };
            m.verify(seed, step, &format!("set node {node} {state:?}"))?;
        }
    }

    // Drain: unlinking every remaining node must empty the ring and leave no dangling links.
    while !m.order.is_empty() {
        let node = m.order[rng.below(m.order.len())];
        unsafe { ring::unlink(m.pool[node].ptr()) };
        let at = m.pos(node).unwrap();
        m.order.remove(at);
        m.linked[node] = false;
    }
    m.verify(seed, steps, "drain to empty")?;
    Ok(())
}

#[test]
fn random_insert_unlink_state_sequences_keep_the_ring_consistent() {
    // Fixed seeds: a failure here reproduces exactly, and the seed list is part of the test.
    let seeds = [
        1u64,
        0xDEAD_BEEF,
        0x1234_5678_9ABC_DEF0,
        0x5EED,
        0xF00D,
        42,
        7919,
        1_048_576,
    ];
    for seed in seeds {
        for nodes in [1usize, 2, 3, 7, 16] {
            one_run(seed, nodes, 400).unwrap_or_else(|e| panic!("{e}"));
        }
    }
}

#[test]
fn first_runnable_from_skips_a_blocked_head() {
    // The regression that mattered on hardware: when the kernel has no current task it asks for
    // "the first runnable task in the ring", and taking the head blindly resumed a task that was
    // in the middle of a sleep — because `ring_head` moves on every spawn, so a blocked task can
    // be sitting there. Measured on an STM32F103: a 200-tick sleep came back after 6 ticks.
    unsafe {
        let a = Node::new(1); // becomes the head
        let b = Node::new(2);
        ring::insert_after(ptr::null_mut(), a.ptr());
        ring::insert_after(a.ptr(), b.ptr());

        // 1. Runnable head is its own answer (no rotation past it).
        assert_eq!(ring::first_runnable_from(a.ptr()), a.ptr());

        // 2. Blocked head with a runnable successor: the successor, never the blocked head.
        (*a.ptr()).state = TaskState::Blocked;
        assert_eq!(ring::first_runnable_from(a.ptr()), b.ptr());

        // 3. Blocked head, blocked successor: nothing runnable, which is what sends the switch
        //    path to its idle branch instead of running a sleeping task.
        (*b.ptr()).state = TaskState::Blocked;
        assert!(ring::first_runnable_from(a.ptr()).is_null());

        // 4. Dead nodes are skipped the same way.
        (*a.ptr()).state = TaskState::Dead;
        (*b.ptr()).state = TaskState::Ready;
        assert_eq!(ring::first_runnable_from(a.ptr()), b.ptr());

        // 5. Null in, null out.
        assert!(ring::first_runnable_from(ptr::null_mut()).is_null());
    }
}

#[test]
fn a_single_node_ring_is_its_own_successor_and_unlinks_to_nothing() {
    unsafe {
        let a = Node::new(1);
        ring::insert_after(ptr::null_mut(), a.ptr());
        assert_eq!((*a.ptr()).next, a.ptr(), "sole member points at itself");
        assert_eq!((*a.ptr()).prev, a.ptr());
        assert_eq!(ring::check(a.ptr()), Ok(1));
        // A runnable sole member is its own successor; a blocked one has none.
        assert_eq!(ring::next_runnable(a.ptr()), a.ptr());
        (*a.ptr()).state = TaskState::Blocked;
        assert!(ring::next_runnable(a.ptr()).is_null());
        (*a.ptr()).state = TaskState::Running;
        assert_eq!(ring::unlink(a.ptr()), ptr::null_mut());
        assert!(
            !(*a.ptr()).is_linked(),
            "unlinked node must not look linked"
        );
    }
}

#[test]
fn an_all_blocked_ring_reports_no_runnable_task() {
    unsafe {
        let a = Node::new(1);
        let b = Node::new(2);
        let c = Node::new(3);
        ring::insert_after(ptr::null_mut(), a.ptr());
        ring::insert_after(a.ptr(), b.ptr());
        ring::insert_after(b.ptr(), c.ptr());
        for p in [a.ptr(), b.ptr(), c.ptr()] {
            (*p).state = TaskState::Blocked;
        }
        // This is what sends the switch path to its idle branch instead of spinning. If
        // `next_runnable` ever returned a blocked task, the kernel would hand the CPU to a task
        // that is not allowed to run.
        for p in [a.ptr(), b.ptr(), c.ptr()] {
            assert!(
                ring::next_runnable(p).is_null(),
                "an all-blocked ring must report no runnable task"
            );
        }
        assert_eq!(ring::check(a.ptr()), Ok(3));
    }
}

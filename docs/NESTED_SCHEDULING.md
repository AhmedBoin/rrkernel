# Nested (hierarchical) round-robin scheduling

The design of record for nested scheduling. The quickstart elsewhere in these docs is
unchanged and needs none of this vocabulary: a plain `thread::spawn` is a level-1 leaf with
the configured slice, exactly as before.

## The model

The scheduler walks a **tree of nodes**. A node is one of two kinds:

| kind | stack | code | owns | runs |
|---|---|---|---|---|
| `Leaf` | yes | a closure | its quantum | on the CPU |
| `Group` | **no** | **no** | a quantum and a ring of children | nothing — it is *descended through* |

The root is an implicit `Group` allocated at init; `main` and every plain-spawned task are
its level-1 children. A depth-1 tree (root plus only leaves) is mechanically identical to
the flat ring this kernel had before groups — invariant **E** below, and the reason no
existing example, test or quickstart changed.

A group costs one TCB and nothing else: no stack, no closure, no backend thread or fiber
(`spawn_group_node` deliberately does not call `arch::create_task`, because on bare metal
the stack is the expensive part).

## One timer, one tick, every level

There is no per-level timer. The hardware fires at the **configured slice** — the tick —
and each tick spends one tick of quantum:

> **Each tick decrements the quantum of the running path: the current leaf and every
> ancestor group. The OUTERMOST node on that path whose quantum reached zero decides what
> happens next — the turn passes to the next runnable child of that node's parent, or, if
> that node is the root, to the root's next runnable child.**

Everything else follows:

* **A group's own window closing does not touch its cursor.** The turn moves at its
  *parent*, so the group's next dispatch resumes at the child it was on instead of
  rewinding to the first.
* **A leaf preempted because an ancestor's window closed keeps `remaining_cycles`**, so it
  resumes with exactly the time it had left.
* **Children finishing early need no special case**: their ring wraps and the lap repeats
  while the ancestor's quantum is still draining (underflow).
* **Children running over truncate the visit** mid-child and the next visit resumes — the
  same mechanism as bullet one (overflow). Note this means both children do get CPU in the
  truncating visit; a 10-tick group with 8+8-tick children runs C1 for 8 ticks and C2 for
  2, then resumes C2 with 6 left.
* **A node with no runnable descendant bubbles up**, and the root with nothing runnable is
  the existing idle path.

| quantity | unit |
|---|---|
| the tick | the slice passed to `configure()`, in timer cycles (`KERNEL.config.slice_cycles`) |
| a node's quantum (`slice_cycles`) | **ticks**; `1` is the configured slice |
| the root's quantum | unbounded (the root never expires) — top-level rotation is driven by each level-1 node's own quantum, which is what a multi-tick group visit needs |
| a node's remaining budget (`remaining_cycles`) | ticks; `0` means "not armed" |
| every sleep, timeout and deadline | global ticks (`KERNEL.ticks`) — never a per-node clock |

A nested quantum must therefore be a **whole number of ticks**, i.e. an integer multiple of
the configured slice. Shorter or non-multiple quanta are refused
(`ConfigError::QuantumBelowTick` / `QuantumNotMultipleOfTick`) rather than rounded, because
silently changing a timing request is worse than a startup error. Consequence worth
noticing: a 400 Hz requirement (2.5 ms) needs a 0.5 ms *configured* slice, since that sets
the tick for the whole system.

## Invariants

* **A** — A group's quantum bounds one *visit* only; it constrains neither the sum nor the
  direction of its children's quanta.
* **B** — A group's cursor advances only when its own level's turn genuinely ends, never on
  a preemption.
* **C** — Underflow repeats the lap inside the same visit without resetting the group's own
  budget; overflow truncates the visit and resumes at the same child.
* **D** — All deadlines use the one global tick counter. Nothing here introduces a local
  clock; the drain only ever touches `remaining_cycles`.
* **E** — A depth-1 tree is bit-for-bit the old flat scheduler, and `thread::spawn` keeps
  its signature and behaviour.

## API

```rust
// Unchanged. A level-1 leaf with the configured slice.
thread::spawn(worker);

// A task with its own quantum, still at the root. Slice::Default == the configured slice.
thread::spawn_in(Parent::Root, Slice::Millis(2), reporter)?;

// A group with its own budget, and tasks inside it.
let telemetry = thread::spawn_group(Parent::Root, Slice::Millis(2))?;
thread::spawn_in(Parent::Group(telemetry), Slice::Millis(1), heartbeat)?;
thread::spawn_in(Parent::Group(telemetry), Slice::Millis(1), param_stream)?;
```

Scheduling consequences worth knowing before nesting:

* **Latency grows with depth and with siblings.** A leaf's worst-case wait is the sum, level
  by level, of (siblings × quantum). The flat `(tasks − 1) × slice` figure describes depth 1.
* **A woken task does not preempt.** As at depth 1, a wake makes a task runnable; it runs
  when its own ring's turn comes round — inside its group's next visit.
* **A newcomer runs soon *within its own ring*** (it is inserted after that ring's cursor),
  not necessarily soon in wall-clock terms.

## Deliberate limits

* **Groups live until shutdown.** No `close()`, no teardown. Without teardown there is no way
  to free a TCB that a `children_head` or a cursor still points at — which is the entire
  class of dangling-parent bug this design is otherwise exposed to (a cursor into freed arena
  memory, or into a recycled TCB belonging to a different task). The cost is one stack-free
  TCB per group for the life of the program; an empty group schedules nothing and is skipped
  cheaply.
* **`active_threads` counts leaves** — the tasks a user asked for. A group is a scheduling
  node, not a task, and is not counted.
* **`ring::len()` on the level-1 ring counts level-1 nodes**, groups included, so the flat
  demo assertion `ring_len == active_threads` holds at depth 1 and is not a general identity.
* **Sub-tick quanta are refused**, as above.

## Corrections relative to the first draft of this design

Kept here so the reasoning is not lost:

1. **The draft's `tick_leaf` decremented only the leaf.** A group's budget would then never
   be spent, so its window could never close and the overflow case could not happen at all.
   The ancestor drain above is the fix, and it was the load-bearing gap.
2. **The draft's `ascend` had an unreachable branch** ("if `next_runnable` found nothing,
   check whether some child is runnable"). `ring::next_runnable` already walks the whole ring
   and wraps, returning a runnable node whenever one exists — so null already means "nothing
   runnable here". Underflow needs no branch: the ring wraps.
3. **The draft's test 3 expected the wrong sequence** (see the 8+8 example above). Its prose
   was right and its numbers were wrong.
4. **Three "must not change" claims were false.** `wake_expired`, `wake_blocked_on`,
   `wake_task`, `wake_first_blocked_on` and `blocked_on_count` all walked a single flat ring,
   so a leaf blocked inside a group would never have woken and its sleep would never have
   expired. They are tree walks now. `exit_task` also had to change: it moved one global
   `ring_head`, which a nested exit would have clobbered with a pointer into that group's
   ring, and it now advances the dying node's *own* parent cursor instead.

## Status

Implemented and verified: the tree dispatch, the ancestor drain, the tree-aware wake paths,
the guarded exit, `Parent`/`spawn_in`/`spawn_group`/`Slice::Default`, and the quantum
validation. Flat behaviour is unchanged (`cargo test --features std`: 24 tests + 2 doctests;
clippy clean with `-D warnings`; all six CI bare-metal targets build).

Not yet done: the hand-driven `tests/nested_scheduling.rs` cases from the design (they need
the private dispatch functions, so they belong as in-crate unit tests in `scheduler.rs`), an
example that runs a nested tree on the host, and verification of the host backends when
`schedule_next` legitimately returns the current task (which happens only once a quantum
longer than one tick exists).

---

## Verification log

### Depth is bounded by memory, not by a constant

The first implementation capped every tree walk at `MAX_TREE_DEPTH = 32`, and past that cap
`descend_and_arm` returned null. The effect was not a clean refusal: the subtree below the cap
became **unreachable**, so its tasks silently stopped running. That is the "a thread starves"
failure this design exists to rule out, so the cap is gone.

What replaces it:

* `KERNEL.nodes` counts the TCBs handed out since init -- a monotonic upper bound on the number
  of scheduling nodes in the tree. Every walk takes that many steps, plus two. A walk cannot
  legitimately take more steps than the tree has nodes, so deep trees are legal and a malformed
  one (a parent cycle) still terminates.
* `walk_tree` is now **iterative**. It used to call itself once per level, which made kernel
  stack use grow with nesting depth: fine at depth 3, an overflow on a 2 KiB MSP at depth 40.
  The parent pointers the tree already carries make an explicit stack unnecessary.

Nesting depth is therefore limited by the arena, one TCB per group. The node count is the real
constraint, not the stack.

### The root's quantum is unbounded

The root group's quantum is `u32::MAX`, so the top-level rotation is driven by each level-1
node's *own* quantum. With a one-tick root quantum -- the obvious-looking choice, and what an
earlier revision of this document described -- every level-1 node advances the root's cursor on
every tick, which truncates any deeper visit to a single tick and makes a multi-tick visit
impossible to complete. Depth 1 is still bit-for-bit the flat ring, because there it is the
level-1 *leaf's* one-tick quantum that expires and moves the cursor.

### Tests, all driven against the real tick path

The `nested_tests` module in `src/scheduler.rs` runs thirteen cases through `on_tick` and
`schedule_next` against hand-built TCB trees: depth-1 identity, underflow lap repetition,
overflow truncation with resume, mid-visit preemption keeping the exact remainder, all-blocked
bubbling up without burning the group's budget, a deadline expiring at depth while the group is
starved, stale-cursor repair, depth 3, a multi-tick quantum at level 1 (four ticks with no
switch, then exactly one), a lock wake reaching a waiter inside a group, **64 levels of
nesting**, a **200-tick fairness sweep** across a mixed tree, and a **malformed parent cycle**
that must terminate rather than spin.

The fairness sweep is the one that states the headline promise: every runnable leaf runs, every
tick goes to a leaf of the tree, and no run outlasts that leaf's own quantum.

One harness correction came out of this: the test helper that builds TCBs by hand did not
mirror `alloc_tcb`, so `KERNEL.nodes` stayed zero and every walk ran with a two-node budget.
That was enough to pass the earlier cases while proving much less than it appeared to, which is
why the helper now counts the nodes it creates.

### On the board

`examples/cortex-m-bluepill/src/bin/nested.rs` and
`examples/cortex-m-blackpill/src/bin/nested.rs` build the same three-level tree, with the sums
deliberately unequal in both directions: `ctrl` (5 ms) holding children totalling 4 ms, and
`fast` (1 ms) holding children totalling 4 ms. Each leaf holds an absolute 10 ms deadline and
reports its worst lateness, so the run measures wall-clock sleep correctness at depth. On the
Black Pill the LED is one of the leaves inside the group, so the nesting is visible on the
board rather than only in RTT. Both print a verdict; neither has been run on hardware yet.

### The host backend does not tick in this scenario, and nesting is not the cause

`nested_demo` reported `ticks 0` on Win32. `examples/host_tick_check.rs` is the probe that
settled what that means: the same `SchedulerConfig`, in three modes -- a plain task, a group,
and a group with two children -- printing `ticks` every 100 ms. **The flat mode stalls
identically**, with no group anywhere in the tree, so the stall is a property of the host tick
source (or of this workload) rather than of groups. `roundrobin_demo`, which uses the same
configuration and CPU-bound tasks, ticks correctly in the same session.

Consequence for this document: every claim here about *ordering* is verified on the host, and
the claims about *timing at depth* are verified on no host yet. They are measurable on metal,
where the tick is SysTick and the F103 fidelity run already measures ticks and early sleeps
directly, and that is where they should be measured.
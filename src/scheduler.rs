//! Scheduler policy: who runs next, what the counters say, and how a finished
//! task's memory gets reclaimed.
//!
//! Everything in this module is backend-neutral. The port (see [`crate::arch`])
//! only ever calls into [`schedule_next`], [`on_tick`] and [`record_latency`],
//! and the ring surgery happens here — in Rust, under a critical section —
//! rather than in hand-written assembly. That split is deliberate: assembly
//! does register/stack mechanics, Rust does policy.

use crate::arena::{Arena, ArenaStats, DEFAULT_ARENA};
use crate::config::{ConfigError, PlatformLimits, SchedulerConfig, Slice};
use crate::critical;
use crate::ring;
use crate::tcb::{KernelConfig, TaskControlBlock, TaskState, KERNEL};
use core::cell::UnsafeCell;
use core::ptr;

/// Kernel arena. A private wrapper (rather than `static mut Arena`) so the
/// `UnsafeCell` contract is explicit.
struct ArenaCell(UnsafeCell<Arena>);

// SAFETY: all access happens under `critical::enter`.
unsafe impl Sync for ArenaCell {}

static ARENA: ArenaCell = ArenaCell(UnsafeCell::new(Arena::empty()));

/// Raw access to the kernel arena. Caller must hold a critical section.
#[inline]
pub(crate) unsafe fn arena() -> *mut Arena {
    ARENA.0.get()
}

/// Snapshot of everything the kernel knows about itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SchedulerStats {
    /// Tasks ever created (monotonic).
    pub total_threads: usize,
    /// Tasks currently linked in the ring.
    pub active_threads: usize,
    /// Slices elapsed since init.
    pub ticks: u64,
    /// Context switches performed since init.
    pub switches: u64,
    /// Timer ticks that could not be serviced in time (host contention).
    pub ticks_deferred: u64,
    /// Worst switch latency observed (timer cycles or ns; see `Measure`).
    pub worst_latency: u32,
    /// Most recent switch latency.
    pub last_latency: u32,
    /// Worst deviation of the achieved tick period from the requested slice
    /// (nanoseconds). The headline timing number: how exact the slice is.
    pub worst_period_error_ns: u32,
    /// Most recent tick-period deviation (nanoseconds).
    pub last_period_error_ns: u32,
    /// Achieved slice length in nanoseconds.
    pub slice_ns: u64,
    /// Achieved slice length in timer cycles (0 on time-based host backends).
    pub slice_cycles: u32,
    /// Timer frequency used for the conversion (0 on host time-based timers).
    pub timer_hz: u32,
    /// Dead tasks whose memory has been reclaimed by deferred-free.
    pub reclaimed: u64,
    /// Times a task tried to block with no current task. Must stay zero: see
    /// [`crate::tcb::Kernel::blocks_without_current`]. Non-zero means a block came from
    /// interrupt or idle context and therefore did *not* wait at all.
    pub blocks_without_current: u64,
    /// Arena usage.
    pub arena: ArenaStats,
    /// Whether the tick source is live.
    pub running: bool,
}

/// Initialise the scheduler with the default configuration (1 ms slices).
///
/// The scheduler starts running **immediately**: the tick source is armed
/// before this returns, and the calling context is adopted as task 0 and
/// linked into the ring. There is no `scheduler_run()`.
///
/// # Panics
/// Panics if the default 1 ms slice is not achievable on this platform. Use
/// [`init_with`] if you want to handle that as a value.
pub fn init() {
    if let Err(e) = init_with(SchedulerConfig::default()) {
        panic!("rrkernel: scheduler_init() failed: {}", e);
    }
}

/// Initialise the scheduler with an explicit [`SchedulerConfig`].
///
/// `cfg.slice` is **the** tuning knob for timing-sensitive applications:
/// `scheduler::init_with(SchedulerConfig::embedded(Slice::Micros(100),
/// 168_000_000))` gives 100 µs slices, cycle-exact, on a 168 MHz core.
pub fn init_with(cfg: SchedulerConfig) -> Result<(), ConfigError> {
    // --- validate the time slice against the real platform -----------------
    let plan = crate::arch::plan_timer(&cfg)?;
    if plan.slice_ns == 0 || plan.slice_cycles == 0 {
        return Err(ConfigError::ZeroSlice);
    }

    // --- arena -------------------------------------------------------------
    let (base, len) = match cfg.arena {
        Some(region) => (region.as_mut_ptr(), region.len()),
        None => (DEFAULT_ARENA.as_ptr(), crate::arena::DEFAULT_ARENA_SIZE),
    };
    // The arena must at least hold task 0's TCB plus a few tasks' worth of
    // bookkeeping. Bare metal also carves each task's *stack* from here (see
    // `arch::cortex_m`), so the recommended size there is
    // `stack_size * max_tasks`; OS-backed backends only need the TCBs and the
    // closure blobs.
    let min_arena = core::mem::size_of::<TaskControlBlock>() * 4 + 512;
    if len < min_arena {
        return Err(ConfigError::ArenaTooSmall {
            provided: len,
            minimum: min_arena,
        });
    }

    let g = critical::enter();
    unsafe {
        if *KERNEL.running.get() {
            return Err(ConfigError::AlreadyInitialized);
        }

        (*arena()).init(base, len);
        *KERNEL.config.get() = KernelConfig {
            stack_size: cfg.stack_size,
            idle: cfg.idle,
            measure: cfg.measure,
            slice_cycles: plan.slice_cycles,
            timer_hz: plan.timer_hz,
            // Preserved from whatever `set_active_cores` chose before init, so the
            // multi-core decision survives this snapshot.
            active_cores: (*KERNEL.config.get()).active_cores.max(1),
        };

        // --- the implicit root group ---------------------------------------
        // The scheduling tree's root: a Group with no parent, owning the level-1
        // ring. Not a task (no stack, no closure) and deliberately not counted in
        // `total_threads`/`active_threads`, which count *leaves* — the tasks the user
        // actually asked for. Its quantum is the configured slice; that equality with
        // a level-1 leaf's quantum is what makes the flat case behave exactly as the
        // flat ring always has.
        let root = alloc_tcb().ok_or(ConfigError::ArenaTooSmall {
            provided: len,
            minimum: min_arena,
        })?;
        (*root).kind = crate::tcb::NodeKind::Group;
        (*root).parent = ptr::null_mut();
        (*root).children_head = ptr::null_mut();
        (*root).current_child = ptr::null_mut();
        (*root).state = TaskState::Ready;
        // The root never expires: its quantum is unbounded, so the top-level rotation is
        // driven by each level-1 node's *own* quantum rather than by the root's. With a
        // one-tick root quantum (the obvious-looking choice) every level-1 node would
        // advance the root's cursor on every tick, which truncates any deeper visit to a
        // single tick and makes a group unable to ever complete a multi-tick visit.
        // With an unbounded root, depth 1 is still exactly the flat scheduler: a level-1
        // leaf's quantum is one tick, so its expiry is what advances the root's cursor.
        (*root).slice_cycles = u32::MAX;
        (*root).remaining_cycles = u32::MAX;
        *KERNEL.root.get() = root;

        // --- adopt the calling context as task 0 ---------------------------
        let tcb = alloc_tcb().ok_or(ConfigError::ArenaTooSmall {
            provided: len,
            minimum: min_arena,
        })?;
        (*tcb).state = TaskState::Running;
        (*tcb).id = take_id();
        (*tcb).kind = crate::tcb::NodeKind::Leaf;
        (*tcb).parent = root;
        (*tcb).slice_cycles = 1;
        (*tcb).remaining_cycles = 1;
        crate::arch::adopt_current_task(tcb)?;
        ring::insert_after(ptr::null_mut(), tcb);
        // Task 0 is the root's first child, so the root's ring and cursor start there.
        (*root).children_head = tcb;
        (*root).current_child = tcb;
        KERNEL.set_current(tcb);
        *KERNEL.ring_head.get() = tcb;
        *KERNEL.total_threads.get() = 1;
        *KERNEL.active_threads.get() = 1;
    }
    drop(g);

    // Arm the periodic tick source. From here on the CPU is time-sliced.
    crate::arch::start_timer(plan)?;
    let g = critical::enter();
    unsafe { *KERNEL.running.get() = true };
    drop(g);
    Ok(())
}

/// Retune the slice at run time (reloads the timer; the *first* slice after
/// the change is a full slice).
pub fn set_slice(slice: Slice) -> Result<(), ConfigError> {
    let g = critical::enter();
    let timer_hz = unsafe {
        if !*KERNEL.running.get() {
            return Err(ConfigError::NotInitialized);
        }
        (*KERNEL.config.get()).timer_hz
    };
    drop(g);

    let cfg = SchedulerConfig {
        slice,
        timer_hz,
        ..SchedulerConfig::default()
    };
    let plan = crate::arch::plan_timer(&cfg)?;
    crate::arch::retune_timer(plan)?;

    let g = critical::enter();
    unsafe {
        (*KERNEL.config.get()).slice_cycles = plan.slice_cycles;
        (*KERNEL.config.get()).timer_hz = plan.timer_hz;
    }
    drop(g);
    Ok(())
}

/// What this platform can actually do. Call before choosing a slice.
pub fn platform_limits() -> PlatformLimits {
    crate::arch::platform_limits()
}

/// Current configuration snapshot (achieved slice, not the request).
pub fn config() -> KernelConfig {
    let g = critical::enter();
    let c = unsafe { *KERNEL.config.get() };
    drop(g);
    c
}

/// Achieved slice length in nanoseconds.
pub fn slice_ns() -> u64 {
    let cycles = config().slice_cycles;
    crate::arch::cycles_to_ns(cycles)
}

/// Full kernel statistics. Takes a critical section internally.
pub fn stats() -> SchedulerStats {
    let g = critical::enter();
    let out = unsafe {
        let cfg = *KERNEL.config.get();
        SchedulerStats {
            total_threads: *KERNEL.total_threads.get(),
            active_threads: *KERNEL.active_threads.get(),
            ticks: *KERNEL.ticks.get(),
            switches: *KERNEL.switches.get(),
            ticks_deferred: *KERNEL.ticks_deferred.get(),
            worst_latency: *KERNEL.worst_latency.get(),
            last_latency: *KERNEL.last_latency.get(),
            worst_period_error_ns: *KERNEL.worst_period_error_ns.get(),
            last_period_error_ns: *KERNEL.last_period_error_ns.get(),
            slice_ns: crate::arch::cycles_to_ns(cfg.slice_cycles),
            slice_cycles: cfg.slice_cycles,
            timer_hz: cfg.timer_hz,
            reclaimed: *KERNEL.reclaimed.get(),
            blocks_without_current: *KERNEL.blocks_without_current.get(),
            arena: (*arena()).stats(),
            running: *KERNEL.running.get(),
        }
    };
    drop(g);
    out
}

/// One row of [`for_each_task`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskInfo {
    pub id: u32,
    pub state: TaskState,
    pub slices_run: u32,
    pub switches: u32,
    pub stack_size: usize,
    /// True for the task currently owning the CPU.
    pub is_current: bool,
}

/// Visit every task currently linked in the ring, in ring order.
///
/// Safe to call from a task (takes a critical section) but *not* from inside
/// the switch path. The callback runs with preemption masked, so it must be
/// short and must not spawn or sleep.
pub fn for_each_task(mut f: impl FnMut(TaskInfo)) {
    let g = critical::enter();
    unsafe {
        let cur = KERNEL.current();
        // Iterate from the ring head so this also works in the window where a
        // task has unlinked itself and no switch has happened yet.
        let start = if cur.is_null() || !(*cur).is_linked() {
            *KERNEL.ring_head.get()
        } else {
            cur
        };
        if !start.is_null() && (*start).is_linked() {
            let mut p = start;
            loop {
                f(TaskInfo {
                    id: (*p).id,
                    state: (*p).state,
                    slices_run: (*p).slices_run,
                    switches: (*p).switches,
                    stack_size: (*p).stack_size,
                    is_current: p == cur,
                });
                p = (*p).next;
                if p == start || p.is_null() {
                    break;
                }
            }
        }
    }
    drop(g);
}

/// Number of tasks linked in the ring.
pub fn active_threads() -> usize {
    let g = critical::enter();
    let n = unsafe { *KERNEL.active_threads.get() };
    drop(g);
    n
}

/// Total tasks ever created.
pub fn total_threads() -> usize {
    let g = critical::enter();
    let n = unsafe { *KERNEL.total_threads.get() };
    drop(g);
    n
}

/// Id of the currently running task (0 if none).
pub fn current_task_id() -> u32 {
    let g = critical::enter();
    let id = unsafe {
        let c = KERNEL.current();
        if c.is_null() {
            0
        } else {
            (*c).id
        }
    };
    drop(g);
    id
}

/// Slices handed to the **calling task** so far. Cheap way for a task to see
/// its own share and, together with its siblings, to demonstrate that pure
/// round-robin fairness holds.
pub fn current_slices_run() -> u32 {
    let g = critical::enter();
    let n = unsafe {
        let c = KERNEL.current();
        if c.is_null() {
            0
        } else {
            (*c).slices_run
        }
    };
    drop(g);
    n
}

/// Number of context switches performed so far (cheap, lock-free read of the
/// 64-bit counter). Useful for correlating a task's timeline with the
/// scheduler's own rotation.
pub fn switch_count() -> u64 {
    let g = critical::enter();
    let n = unsafe { *KERNEL.switches.get() };
    drop(g);
    n
}

/// Number of slices elapsed so far.
pub fn tick_count() -> u64 {
    let g = critical::enter();
    let n = unsafe { *KERNEL.ticks.get() };
    drop(g);
    n
}

/// Ask the kernel to stop: on the host this exits the process, on bare metal
/// it masks interrupts and parks forever. Never returns.
pub fn shutdown(code: i32) -> ! {
    let g = critical::enter();
    unsafe { *KERNEL.shutdown.get() = true };
    drop(g);
    crate::arch::shutdown(code)
}

/// Set the shutdown flag without exiting (the host tick thread will notice and
/// terminate the process; useful for cooperative tear-down in tests).
pub fn request_shutdown() {
    let g = critical::enter();
    unsafe { *KERNEL.shutdown.get() = true };
    drop(g);
    crate::arch::request_switch();
}

/// True once [`request_shutdown`] / [`shutdown`] has been called.
pub fn shutdown_requested() -> bool {
    let g = critical::enter();
    let b = unsafe { *KERNEL.shutdown.get() };
    drop(g);
    b
}

/// Run the rest of `main` as **task 0's body**, then treat its return as a task
/// exit (unlink, `active_threads -= 1`, immediate switch) instead of falling
/// back into the C runtime.
///
/// # Why this exists
/// `scheduler_init()` adopts the calling context as task 0 — but on a hosted
/// process, *returning from `main`* is not something the kernel can intercept:
/// the C runtime takes over and calls `exit()`, which would tear the whole
/// process (and every task with it) down. `main_body` closes that hole without
/// any stack surgery: it calls `f`, and on return it runs the very same
/// teardown a task trampoline runs, ending the calling thread through the
/// kernel's own exit path (`ExitThread` on Windows, the idle loop on bare
/// metal) instead of the CRT's `exit()`.
///
/// Usage — spec semantics, host and metal alike:
///
/// ```no_run
/// # use rrkernel::{scheduler, thread, Slice, SchedulerConfig};
/// fn main() {
///     scheduler::init_with(SchedulerConfig::embedded(Slice::Millis(1), 168_000_000)).unwrap();
///     scheduler::main_body(|| {
///         thread::spawn(|| loop { /* CPU-bound, preempted every slice */ });
///         thread::spawn(|| { /* short work, then returns -> auto unlink */ });
///     });
/// }
/// ```
///
/// # Panics
/// Panics if the scheduler has not been initialised.
pub fn main_body<F: FnOnce()>(f: F) -> ! {
    // The scheduler adopted the calling context as task 0 at init time, so the
    // current TCB *is* our task (which stays true across preemptions: a task is
    // only ever running while it is the current one).
    let me = {
        let g = critical::enter();
        let me = unsafe { KERNEL.current() };
        drop(g);
        assert!(
            !me.is_null(),
            "scheduler::main_body() requires scheduler::init*() to have run first"
        );
        me
    };

    f();

    // SAFETY: we are this task, and its body has returned.
    unsafe { crate::trampoline::exit_task(me) }
}

// ---------------------------------------------------------------------------
// Called by the ports
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Nested scheduling: the tree walk
//
// One rule covers every case:
//
//   **Each tick decrements the quantum of the running path — the current leaf and
//   every ancestor. The OUTERMOST node on that path whose quantum reached zero
//   decides what happens next: the turn passes to the next runnable child of that
//   node's parent, or, if that node is the root, to the root's next runnable child.**
//
// Consequences, each a stated invariant:
// * A group whose *own* quantum closed never touches its own cursor, so its next
//   dispatch resumes at the child it was on instead of rewinding to the first.
// * A leaf preempted because an ancestor's quantum closed keeps `remaining_cycles`,
//   so it resumes with exactly the time it had left.
// * A group dispatched fresh (quantum zero) is armed to a full quantum; one re-entered
//   mid-visit keeps the rest of that visit.
// * Children finishing early simply wrap their own ring and continue — underflow needs
//   no special case, because the ancestor's quantum is still draining.
// * A node with no runnable descendant bubbles the turn to its parent, and the root
//   with nothing runnable is the existing idle path.
//
// Depth 1 stays bit-for-bit the old flat scheduler: every level-1 leaf has a quantum of one
// tick while the root's quantum is unbounded, so it is the *leaf* that expires and advances
// the root's cursor — exactly one node per tick, which is the flat rotation.
// ---------------------------------------------------------------------------

/// Step limit for every tree walk, derived from the number of TCBs the kernel has handed out.
///
/// There is deliberately **no fixed nesting limit**: a tree may be as deep as the arena
/// allows, one TCB per group. An earlier revision capped depth at 32 and stopped descending
/// past it — silently, which turned a legal tree into an unreachable subtree whose tasks
/// never ran again (the exact starvation this design has to rule out). A walk cannot
/// legitimately take more steps than the tree has nodes, so this both permits deep trees and
/// terminates on a malformed one (a parent cycle, which only a bug elsewhere could create).
///
/// Read without a critical section on purpose: the counter is monotonic, so a stale value is
/// merely a smaller bound, and every caller is already in kernel context.
#[inline]
fn walk_limit() -> u32 {
    // `+2`: one for the node being walked, one for the step that detects a wrap.
    unsafe { *crate::tcb::KERNEL.nodes.get() }.saturating_add(2)
}

/// Debug-only shape check: a group must have no stack/closure, a leaf no children.
#[inline]
unsafe fn check_node_wellformed(node: *mut TaskControlBlock) {
    debug_assert!(!node.is_null());
    if (*node).kind == crate::tcb::NodeKind::Group {
        debug_assert!(
            (*node).sp.is_null() && (*node).stack_base.is_null() && (*node).closure_block.is_null(),
            "rrkernel: a Group node must have no stack, sp or closure"
        );
    } else {
        debug_assert!(
            (*node).children_head.is_null() && (*node).current_child.is_null(),
            "rrkernel: a Leaf node must have no children"
        );
    }
}

/// Arm `node`'s quantum if it is not already running one, and report what it has.
///
/// "Zero means not armed" is the distinction between a fresh dispatch (arm a full
/// quantum) and a resumed one (keep exactly what was left), and the mid-slice-resume
/// invariant rests on it.
#[inline]
unsafe fn arm_if_needed(node: *mut TaskControlBlock) -> u32 {
    if (*node).remaining_cycles == 0 {
        // `.max(1)`: a node whose quantum was never set still gets a turn rather than
        // starving forever — a zero-tick quantum is a bug, not "never run".
        (*node).remaining_cycles = (*node).slice_cycles.max(1);
    }
    (*node).remaining_cycles
}

/// Linked into a ring? `ring::unlink` poisons both links, so a node that has finished
/// and left has `next == null`.
#[inline]
unsafe fn is_linked(node: *mut TaskControlBlock) -> bool {
    !node.is_null() && !(*node).next.is_null()
}

#[inline]
unsafe fn runnable(node: *mut TaskControlBlock) -> bool {
    !node.is_null() && !matches!((*node).state, TaskState::Dead | TaskState::Blocked)
}

/// The child of `group` this visit should resume on, *without* advancing the cursor.
///
/// Robust against every state a cursor can be in: never set, pointing at a child that
/// has since finished (whose links are poisoned, so `next_runnable` cannot be used on
/// it), or pointing at a child that is merely blocked. Each case has a fallback, so a
/// group cannot be wedged by a stale cursor.
unsafe fn resume_child(group: *mut TaskControlBlock) -> *mut TaskControlBlock {
    let head = (*group).children_head;
    if head.is_null() {
        return ptr::null_mut();
    }
    let cur = (*group).current_child;
    if runnable(cur) && is_linked(cur) && (*cur).parent == group {
        return cur;
    }
    if is_linked(cur) {
        let n = crate::ring::next_runnable(cur);
        if !n.is_null() {
            return n;
        }
    }
    crate::ring::first_runnable_from(head)
}

/// Descend through nested groups to an actual leaf, arming quanta on the way down.
///
/// Null means the subtree has nothing runnable — not an error, just "this group has
/// nothing to offer this time", and the caller bubbles up.
unsafe fn descend_and_arm(mut node: *mut TaskControlBlock) -> *mut TaskControlBlock {
    let mut depth = 0u32;
    let limit = walk_limit();
    while !node.is_null() && (*node).kind == crate::tcb::NodeKind::Group {
        depth += 1;
        if depth > limit {
            return ptr::null_mut();
        }
        check_node_wellformed(node);
        arm_if_needed(node);
        let child = resume_child(node);
        if !runnable(child) {
            return ptr::null_mut();
        }
        (*node).current_child = child;
        node = child;
    }
    if !runnable(node) {
        return ptr::null_mut();
    }
    check_node_wellformed(node);
    arm_if_needed(node);
    node
}

/// Hand the turn to the next runnable child of `group`, or bubble up to its parent.
///
/// This is the only place a cursor advances. It walks the child ring *once* looking for
/// a child whose subtree can actually run, which is what lets "no runnable child here"
/// be detected cheaply and without depending on `ring::next_runnable`'s wrap behaviour —
/// and what keeps a malformed tree from spinning.
unsafe fn advance_from(mut group: *mut TaskControlBlock) -> *mut TaskControlBlock {
    let mut depth = 0u32;
    let limit = walk_limit();
    loop {
        depth += 1;
        if group.is_null() || depth > limit {
            return ptr::null_mut();
        }
        check_node_wellformed(group);
        let head = (*group).children_head;
        if is_linked(head) {
            // Start after the cursor when it is still one of ours, else at the head.
            let cur = (*group).current_child;
            let start = if is_linked(cur) && (*cur).parent == group && !(*cur).next.is_null() {
                (*cur).next
            } else {
                head
            };
            let mut p = start;
            loop {
                if runnable(p) {
                    let leaf = descend_and_arm(p);
                    if !leaf.is_null() {
                        (*group).current_child = p;
                        return leaf;
                    }
                }
                p = (*p).next;
                if !is_linked(p) || p == start {
                    break;
                }
            }
        }
        // Nothing runnable in this ring: this group's turn is over, ask its parent.
        group = (*group).parent;
    }
}

/// Choose the leaf to run next. `cur` is the current leaf, or null when the previous one
/// finished and unlinked itself.
///
/// Returning `cur` unchanged means "no switch": the running leaf still has quantum left
/// and no ancestor's closed. That can only happen for a quantum spanning more than one
/// tick, which is impossible at depth 1 — which is exactly why depth 1 remains
/// bit-for-bit the old flat scheduler.
unsafe fn pick_next_leaf(cur: *mut TaskControlBlock) -> *mut TaskControlBlock {
    let root = *KERNEL.root.get();
    if root.is_null() {
        // Not initialised, or a port reached the switch path before init: behave like the
        // flat kernel and let the port's idle path decide.
        return ptr::null_mut();
    }

    if cur.is_null() {
        // No current task: enter the root's ring *at* its cursor, not after it. That
        // mirrors the flat path's `first_runnable_from(ring_head)` — including the fix
        // for the "sleep resumed early, about 1 in 15, right after a task was created or
        // destroyed" bug, where taking the head blindly handed the CPU to a sleeping
        // task. `first_runnable_from` is inclusive and `next_runnable` is not; getting
        // that backwards reintroduces that bug.
        let leaf = descend_and_arm(resume_child(root));
        if !leaf.is_null() {
            return leaf;
        }
        return advance_from(root);
    }

    let z = outermost_exhausted(cur);
    if z.is_null() {
        // Nothing on the path is out of quantum: the leaf keeps the CPU, so this tick
        // ends without a switch.
        if runnable(cur) {
            return cur;
        }
        // The current task is not runnable any more: it blocked (a sleep, or a lock) and still
        // had quantum left. Returning it here handed the CPU straight back to the task that had
        // just asked to be switched away from, so PendSV switched to *it*, `sleep_until_tick`
        // re-blocked it, and the whole system ping-ponged in that retry loop while ticks, the
        // other tasks and every wake-up stopped. On the board: `ticks=0` for ever, `spin=0`,
        // `wakes=0` -- with no groups involved at all, which is why it looked like a nesting bug
        // and why no host run could see it (the host tick source was stalled in that scenario).
        return advance_from(root);
    }
    if z == root {
        // The root's own quantum closed: the top-level rotation advances, which for a
        // depth-1 tree is exactly `ring::next_runnable(current_leaf)`.
        return advance_from(root);
    }
    // An inner node's quantum closed. The turn passes to that node's parent — which is
    // what leaves the node's own cursor untouched, so its next dispatch resumes.
    let parent = (*z).parent;
    if parent.is_null() {
        return advance_from(root);
    }
    advance_from(parent)
}

/// The outermost node on `node`'s path to the root whose quantum is exhausted, or null.
///
/// Outermost-wins is what lets an inner cursor survive: when a group's own quantum
/// closes, the turn passes to *its parent's* next child and the group's cursor is left
/// exactly where it was.
unsafe fn outermost_exhausted(node: *mut TaskControlBlock) -> *mut TaskControlBlock {
    let mut found: *mut TaskControlBlock = ptr::null_mut();
    let mut p = node;
    let mut depth = 0u32;
    let limit = walk_limit();
    while !p.is_null() {
        depth += 1;
        if depth > limit {
            break;
        }
        if (*p).remaining_cycles == 0 {
            found = p;
        }
        p = (*p).parent;
    }
    found
}

/// Spend one tick of quantum along the running path: the current leaf and every ancestor
/// group, including the root.
///
/// Draining *ancestors* is what gives a group's visit a bounded length at all. Draining
/// only the leaf — which is what the first draft of this design did — leaves every
/// group's `remaining_cycles` frozen at its initial value, so the overflow case ("the
/// group's window closes before all its children have had a turn") could never happen:
/// the budget would simply never be spent.
///
/// Groups *not* on the current path are deliberately untouched: a group's quantum is
/// spent only while its subtree is actually running, which is what keeps an idle group
/// paused rather than burning its budget down while it has nothing to do.
///
/// Saturating and depth-bounded, so a malformed tree cannot spin here.
unsafe fn drain_current_path() {
    let mut n = KERNEL.current();
    let mut depth = 0u32;
    let limit = walk_limit();
    while !n.is_null() {
        depth += 1;
        if depth > limit {
            break;
        }
        (*n).remaining_cycles = (*n).remaining_cycles.saturating_sub(1);
        n = (*n).parent;
    }
}

/// Walk a node ring and, recursively, every child ring beneath it.
///
/// The tree equivalent of the old flat `p = (*p).next` sweep. It exists because a leaf
/// blocked three levels down is not in the level-1 ring at all: a flat walk would never
/// look at it, so neither its sleep nor its lock timeout would ever expire — silently, and
/// only at depth.
///
/// **Iterative, not recursive.** It used to call itself once per level, which made kernel
/// stack use grow with nesting depth — fine at depth 3, an overflow on a 2 KiB MSP at depth
/// 40. The parent pointers the tree already carries make an explicit stack unnecessary:
/// descend to the children head, then move sideways to the next sibling or up to the parent,
/// and stop on the shared step budget rather than at a fixed depth.
unsafe fn walk_tree<F: FnMut(*mut TaskControlBlock)>(first: *mut TaskControlBlock, f: &mut F) {
    if !is_linked(first) {
        return;
    }
    // The ring's owner: the walk covers this ring and everything beneath it, and stops when
    // it ascends back to the owner. The owner itself is never visited — it was already seen
    // on the way *into* the ring, and for a level-1 walk it is the root.
    let owner = (*first).parent;
    let limit = walk_limit();
    let mut steps = 0u32;
    let mut p = first;
    loop {
        steps += 1;
        if steps > limit {
            return; // malformed tree (a cycle): stop rather than spin
        }
        f(p);
        // Depth-first: into this node's children when it has any.
        if (*p).kind == crate::tcb::NodeKind::Group && is_linked((*p).children_head) {
            p = (*p).children_head;
            continue;
        }
        // Otherwise sideways to the next sibling, or up. Both loops count against the one
        // step budget, so even a parent cycle terminates.
        loop {
            steps += 1;
            if steps > limit {
                return;
            }
            let parent = (*p).parent;
            if parent.is_null() {
                return;
            }
            let entry = (*parent).children_head;
            let next = (*p).next;
            if is_linked(next) && next != entry {
                p = next;
                break;
            }
            // This ring is exhausted: ascend, unless the owner ring has just been finished, in
            // which case the walk is over. Looking for a sibling *before* giving up on the owner
            // is the whole point. With a leaf at the head of a ring -- which is what the blue-pill
            // has, since task 0 is first -- the earlier ordering returned from the head itself,
            // visiting one node and leaving every later sibling, and its whole subtree, unwoken.
            // Their sleeps never expired, so the board printed nothing after its first two lines.
            if parent == owner {
                return;
            }
            p = parent;
        }
    }
}

/// Pick the next runnable task and hand it the CPU. **Runs in kernel context**
/// (Cortex-M: inside PendSV, on the kernel/main stack; host: on the tick
/// thread's stack), which is what makes it safe to reclaim the memory of the
/// task we just switched away from.
///
/// Returns the TCB to switch to, or null when nothing is runnable.
///
/// # Safety
/// Must be called from the port's switch path with the ring quiescent.
pub unsafe fn schedule_next() -> *mut TaskControlBlock {
    let mut cur = KERNEL.current();
    if !cur.is_null() && !(*cur).is_linked() {
        // The current task finished and unlinked itself; it is only waiting
        // for the port to switch away from it.
        cur = ptr::null_mut();
        KERNEL.set_current(ptr::null_mut());
    }

    let next = pick_next_leaf(cur);

    if next.is_null() || !(*next).is_linked() {
        // Nothing runnable at all: leave `current` alone, the port's idle path
        // decides what to do (wfi / sleep / exit).
        return ptr::null_mut();
    }

    if next != cur {
        if !cur.is_null() && (*cur).state == TaskState::Running {
            (*cur).state = TaskState::Ready;
        }
        (*next).state = TaskState::Running;
        KERNEL.set_current(next);
        *KERNEL.switches.get() += 1;
    }
    (*next).switches = (*next).switches.wrapping_add(1);
    (*next).slices_run = (*next).slices_run.wrapping_add(1);
    next
}

/// A timer tick fired. The port calls this before asking for a switch.
#[inline]
pub unsafe fn on_tick() {
    *KERNEL.ticks.get() += 1;
    // Spend one tick of quantum along the running path — the current leaf and every
    // ancestor group. This is the *only* place a quantum is spent, and the tick path is
    // the only caller of `on_tick`, so quanta advance with the hardware timer and never
    // with an immediate (non-tick) switch such as the one a spawn requests.
    drain_current_path();
    // Wake anything whose bounded wait has expired. This is also what makes
    // `Mutex::try_lock_for` time out: the waiter is asleep on a deadline and the tick
    // is the only clock that can notice it passing.
    wake_expired();
}

/// A tick could not be serviced (host lock contention). Visible in
/// [`SchedulerStats::ticks_deferred`]; a non-zero and growing value means the
/// slice is too short for the host scheduler to honour it.
#[inline]
pub unsafe fn on_tick_deferred() {
    *KERNEL.ticks_deferred.get() += 1;
}

/// Record switch latency (`Measure::Cycles` / `Measure::Nanos`, whichever the
/// port produced).
#[inline]
pub unsafe fn record_latency(value: u32) {
    *KERNEL.last_latency.get() = value;
    if value > *KERNEL.worst_latency.get() {
        *KERNEL.worst_latency.get() = value;
    }
}

/// Record how far the achieved tick period deviated from the requested slice
/// (nanoseconds, absolute value). This is the kernel's timing-accuracy metric:
/// on bare metal it is a handful of cycles, on a desktop OS it is what the host
/// timer allows.
#[inline]
pub unsafe fn record_period_error(value: u32) {
    *KERNEL.last_period_error_ns.get() = value;
    if value > *KERNEL.worst_period_error_ns.get() {
        *KERNEL.worst_period_error_ns.get() = value;
    }
}

// ---------------------------------------------------------------------------
// Allocation, reclamation and spawn (internal)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Blocking: sleeping `Mutex` waits and bounded sleeps
// ---------------------------------------------------------------------------
//
// A task that blocks stays **linked** in the ring and is simply skipped by
// `ring::next_runnable`, which is what makes both blocking and waking O(1) — nothing has
// to be searched to put it back, and its place in the rotation is preserved.

/// What a blocking call did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockOutcome {
    /// The caller's own condition was already satisfied, so nothing was blocked. The caller keeps
    /// the CPU and should carry on (this is what makes a wake-up impossible to lose).
    Ready,
    /// The caller is switched away and will resume when woken or its deadline passes.
    Blocked,
    /// The deadline had already passed, so nothing was waited for. Not an error.
    AlreadyPast,
    /// **No current task**: called from interrupt or idle context, where there is nothing to
    /// switch away from. The wait did *not* happen; counted in `stats().blocks_without_current`.
    NoCurrentTask,
}

/// Mark the current task blocked — **the caller must already hold the kernel critical section**.
///
/// This is the primitive every wait in the kernel builds on, and its contract is the reason
/// sleeps stopped returning early:
///
/// * the deadline, the state change and the switch-pend happen inside the caller's *one*
///   critical section, so no tick sweep can observe "marked blocked" without the switch already
///   being pended;
/// * `blocked_on`/`block_deadline` are written *before* `state`, and the state store is last,
///   so a tick that runs immediately afterwards sees a fully formed waiter.
///
/// On [`BlockOutcome::Blocked`] the caller must leave the critical section for the switch to
/// happen: on bare metal the pending `PendSV` is taken the moment the mask lifts.
///
/// # Safety
/// Caller holds a critical section, and is running on the current task (or is willing to be
/// told that it is not).
pub unsafe fn mark_blocked_locked(resource: u32, deadline_tick: u64) -> BlockOutcome {
    let me = KERNEL.current();
    if me.is_null() {
        // Interrupt or idle context. Counted and asserted rather than silently ignored: a
        // silent return here makes the caller's wait vanish, which is exactly how a sleep came
        // to "wake up early" — it never blocked at all.
        *KERNEL.blocks_without_current.get() += 1;
        debug_assert!(
            false,
            "rrkernel: blocking wait attempted with no current task (interrupt or idle context)"
        );
        return BlockOutcome::NoCurrentTask;
    }
    // Already past the deadline (or a zero-length wait): do not pretend to have waited.
    if deadline_tick != 0 && *KERNEL.ticks.get() >= deadline_tick {
        return BlockOutcome::AlreadyPast;
    }
    (*me).blocked_on = resource;
    (*me).block_deadline = deadline_tick;
    (*me).state = TaskState::Blocked;
    crate::arch::request_switch();
    BlockOutcome::Blocked
}

/// Block the calling task until `resource` is woken or the absolute `deadline_tick` passes.
///
/// `resource` is a [`crate::sync::LockId`], or 0 for a plain sleep. `deadline_tick == 0` means
/// "no deadline" — an unbounded wait, which callers must opt into explicitly.
pub fn block_until_tick(resource: u32, deadline_tick: u64) -> BlockOutcome {
    let g = critical::enter();
    let outcome = unsafe { mark_blocked_locked(resource, deadline_tick) };
    drop(g);
    outcome
}

/// Block the current task **unless** `ready()` says the wait is already over.
///
/// `ready()` runs inside the *same* critical section that would mark the task blocked. That is the
/// whole point: a waker either arrives before the section — the condition is then already true and
/// nothing blocks — or after it, in which case the task is already `Blocked` and visible to the
/// waker. A wake-up cannot fall into the gap.
///
/// A closure rather than a "check, then block" pair, deliberately: that two-step form is exactly
/// how `sync::Mutex::lock` came to lose a wake-up and block forever on a lock that was already
/// free. Everything built on top of this — `event::Signal`, `event::WaitQueue`, `event::park`, the
/// async executor — inherits the property instead of re-deriving it.
///
/// `deadline_tick == 0` means "no deadline".
pub fn block_until_tick_if(
    resource: u32,
    deadline_tick: u64,
    ready: impl FnOnce() -> bool,
) -> BlockOutcome {
    let g = critical::enter();
    let outcome = if ready() {
        BlockOutcome::Ready
    } else {
        unsafe { mark_blocked_locked(resource, deadline_tick) }
    };
    drop(g);
    outcome
}

/// Wake a specific task: set its wake flag, and make it runnable if it is blocked.
///
/// ISR-safe (short critical section, no allocation) and idempotent. It deliberately does **not**
/// preempt: a woken task waits for its normal turn in the ring (design rule 1).
///
/// Task ids are never reused — `take_id` is monotonic — so a stale id either names the task it came
/// from or names nothing at all. A waker that outlived its task therefore does nothing, which is
/// why no generation counter is needed on this path.
pub fn wake_task(id: u32) {
    if id == 0 {
        return;
    }
    let g = critical::enter();
    unsafe {
        let root = *KERNEL.root.get();
        if !root.is_null() {
            // Tree walk: `wake_task` is the async `Waker`/`park` path, so a task nested
            // inside a group must be reachable here or its wake-up is lost forever.
            let mut wake = |p: *mut TaskControlBlock| {
                if (*p).id == id {
                    (*p).flags |= crate::tcb::TCB_FLAG_WOKEN;
                    if (*p).state == TaskState::Blocked {
                        (*p).state = TaskState::Ready;
                        (*p).blocked_on = 0;
                        (*p).block_deadline = 0;
                    }
                }
            };
            walk_tree((*root).children_head, &mut wake);
        }
    }
    drop(g);
}

/// Consume the calling task's wake flag: `true` if a `wake_task` arrived and has not been taken.
///
/// # Safety
/// Caller holds the kernel critical section. The flag read belongs to the *running* task, which is
/// exactly what a blocking primitive wants to check before it blocks.
pub unsafe fn take_wake_flag_locked() -> bool {
    let me = KERNEL.current();
    if me.is_null() {
        return false;
    }
    let set = (*me).flags & crate::tcb::TCB_FLAG_WOKEN != 0;
    (*me).flags &= !crate::tcb::TCB_FLAG_WOKEN;
    set
}

/// Clear the calling task's timer deadline. Used by an executor before each poll, so only the
/// deadlines *this* poll armed are honoured.
pub fn clear_timer_deadline() {
    let g = critical::enter();
    unsafe {
        let me = KERNEL.current();
        if !me.is_null() {
            (*me).block_deadline = 0;
        }
    }
    drop(g);
}

/// Arm the calling task's timer deadline, keeping the **earliest** of the ones requested.
///
/// A task can have several pending timers at once (two futures under `join!`, say), and the tick
/// sweep can only know about one — so it gets the earliest, and each future re-arms on its next
/// poll. Whichever fires first wakes the task, which then polls all of them.
///
/// The field is the same `block_deadline` the blocking path uses: while the task is `Ready` it
/// carries "when this task wants to run again", and when the task blocks it becomes the block's own
/// deadline (the executor passes this value straight to `park`, so it is not clobbered).
pub fn set_timer_deadline(deadline_tick: u64) {
    if deadline_tick == 0 {
        return;
    }
    let g = critical::enter();
    unsafe {
        let me = KERNEL.current();
        if !me.is_null() {
            let cur = (*me).block_deadline;
            if cur == 0 || deadline_tick < cur {
                (*me).block_deadline = deadline_tick;
            }
        }
    }
    drop(g);
}

/// The calling task's timer deadline (0 = none).
pub fn task_timer_deadline() -> u64 {
    let g = critical::enter();
    let d = unsafe {
        let me = KERNEL.current();
        if me.is_null() {
            0
        } else {
            (*me).block_deadline
        }
    };
    drop(g);
    d
}

/// Drop an unconsumed wake on the floor.
///
/// An executor calls this before polling: a wake that arrives *during* the poll sets the flag
/// again, `park` then returns immediately, and the future is polled again — which is the property
/// that makes a lost wake-up impossible without any extra bookkeeping.
pub fn clear_wake_flag() {
    let g = critical::enter();
    unsafe {
        let me = KERNEL.current();
        if !me.is_null() {
            (*me).flags &= !crate::tcb::TCB_FLAG_WOKEN;
        }
    }
    drop(g);
}

/// Make the **first** task blocked on `resource` runnable, and say whether one was found. O(ring).
///
/// This is the single-waiter hand-off a `WaitQueue` needs. `wake_blocked_on` wakes everyone, which
/// is right for a lock (they contend and one wins) and wrong for a queue (it would wake the whole
/// line and hand every waiter a "signalled" that is not theirs).
pub fn wake_first_blocked_on(resource: u32) -> bool {
    let g = critical::enter();
    let mut found = false;
    unsafe {
        let root = *KERNEL.root.get();
        if !root.is_null() {
            // Tree walk: a waiter nested inside a group must be found here, or the wake is
            // lost. "First" is tree order rather than ring order, which is arbitrary either
            // way — the contract is that one waiter is woken.
            let mut wake = |p: *mut TaskControlBlock| {
                if !found && (*p).state == TaskState::Blocked && (*p).blocked_on == resource {
                    (*p).flags |= crate::tcb::TCB_FLAG_WOKEN;
                    (*p).state = TaskState::Ready;
                    (*p).blocked_on = 0;
                    (*p).block_deadline = 0;
                    found = true;
                }
            };
            walk_tree((*root).children_head, &mut wake);
        }
    }
    drop(g);
    found
}

/// How many tasks are blocked on `resource` right now. O(nodes in the tree).
pub fn blocked_on_count(resource: u32) -> usize {
    let g = critical::enter();
    let mut n = 0usize;
    unsafe {
        let root = *KERNEL.root.get();
        if !root.is_null() {
            let mut count = |p: *mut TaskControlBlock| {
                if (*p).state == TaskState::Blocked && (*p).blocked_on == resource {
                    n += 1;
                }
            };
            walk_tree((*root).children_head, &mut count);
        }
    }
    drop(g);
    n
}

/// Give up the rest of the slice but stay `Ready`: the caller goes to the back of the round.
///
/// This is the "skip to the next task immediately" primitive. An `nb::WouldBlock`, a `Pending`
/// future, an application polling a peripheral with no interrupt, or code that simply has nothing
/// useful to do until its next turn all reduce to this call. It costs one switch and never blocks,
/// so the caller stays runnable and cannot starve.
///
/// With nothing else runnable there is nowhere to yield to and the call is a no-op: the caller
/// keeps its slice (the tick still fires) rather than spinning it away.
pub fn yield_now() {
    crate::arch::request_switch();
}

/// A task identity that stays meaningful after the task is gone.
///
/// Ids are monotonic and never reused (`take_id`), so a `TaskId` held by a waker either refers to
/// the task it was taken from or to nothing at all. That is the whole safety story for stale
/// wakers: no generation counter, no lookup table, and no way to wake a recycled TCB by accident.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskId(u32);

impl TaskId {
    /// A `TaskId` from a raw id.
    ///
    /// For a waker that stored the number rather than the type — an ISR, a `u32` field in a driver, a
    /// queue of ids. Ids are never reused, so a raw id that names a task always names the right one,
    /// and a raw id that names nothing is harmless: [`crate::scheduler::wake_task`] walks the ring and
    /// does nothing.
    pub const fn from_raw(raw: u32) -> TaskId {
        TaskId(raw)
    }

    /// The raw id, as stored in the TCB. `0` means "no task".
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// True for the null id, which `current_id` returns outside a task.
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }
}

/// The calling task's id, or the null [`TaskId`] when called outside a task (interrupt or idle).
pub fn current_id() -> TaskId {
    TaskId(current_task_id())
}

/// Id allocator for `event` resources (`WaitQueue`), kept out of the task-id space.
///
/// Task ids start at 1 and grow with every spawn; resource ids start at `0x4000_0000`, so the two
/// spaces cannot collide in any realistic lifetime. A critical-section counter rather than an
/// `AtomicU32`, so the same code compiles on Cortex-M0 and AVR, which have no atomic
/// read-modify-write at all.
struct ResourceIds(UnsafeCell<u32>);

// SAFETY: only touched inside `critical::enter`.
unsafe impl Sync for ResourceIds {}

static NEXT_RESOURCE_ID: ResourceIds = ResourceIds(UnsafeCell::new(0x4000_0000));

/// Take a resource id for a new `event::WaitQueue`.
pub fn alloc_resource_id() -> u32 {
    let g = critical::enter();
    let id = unsafe { *NEXT_RESOURCE_ID.0.get() };
    unsafe { *NEXT_RESOURCE_ID.0.get() = id.wrapping_add(1) };
    drop(g);
    id
}

/// Block the calling task for **at least** `ticks` slice ticks.
///
/// The deadline is computed *inside* the critical section. Reading the tick first and blocking
/// afterwards — which is what this used to do — leaves a window: a tick landing in it makes the
/// deadline one tick short, so the sleep returns early. The symptom is intermittent and shows up
/// most often right after a spawn or an exit, because that is when a switch and a just-latched
/// tick most often coincide.
pub fn block_for_ticks(resource: u32, ticks: u64) -> BlockOutcome {
    let g = critical::enter();
    let deadline = unsafe { *KERNEL.ticks.get() }.wrapping_add(ticks);
    let outcome = unsafe { mark_blocked_locked(resource, deadline) };
    drop(g);
    outcome
}

/// Absolute wake-up tick `ticks` from now, captured atomically with the counter.
///
/// For callers that need **one** deadline across several waits (a retry loop that times out).
/// Documented tolerance: a tick landing between this call and the first block shifts the
/// deadline by one tick, so a timeout can fire up to one slice early. Sleeps deliberately do not
/// have that tolerance — they compute their deadline inside the blocking section.
pub fn deadline_after_ticks(ticks: u64) -> u64 {
    let g = critical::enter();
    let d = unsafe { *KERNEL.ticks.get() }.wrapping_add(ticks);
    drop(g);
    d
}

/// Sleep for at least `ticks` slice ticks (the only clock the kernel has).
pub fn sleep_ticks(ticks: u64) {
    let _ = block_for_ticks(0, ticks);
}

/// Sleep until an absolute tick, tolerating spurious wake-ups: a wake that arrives early simply
/// blocks again, because the loop re-checks the deadline.
///
/// What this **cannot** fix: a clock that jumps forward. A double-counted tick moves `now()`
/// past the deadline and the loop exits immediately — that is a time-base fault, not a wake-up
/// fault, and it is why a deadline clock belongs on a counter of its own rather than the slice
/// count.
pub fn sleep_until_tick(deadline_tick: u64) {
    while tick_count() < deadline_tick {
        match block_until_tick(0, deadline_tick) {
            BlockOutcome::Blocked => {}
            // `block_until_tick` has no condition, so it never reports this; the shared enum simply
            // has to name it.
            BlockOutcome::Ready => {}
            // Deadline passed while we were deciding: done.
            BlockOutcome::AlreadyPast => break,
            // Cannot block here at all; stop rather than spin.
            BlockOutcome::NoCurrentTask => break,
        }
    }
}

/// Make every task blocked on `resource` runnable again. O(ring).
///
/// Called by `Mutex::unlock`. A per-lock waiter list would make this O(waiters) at the
/// cost of another intrusive link in every TCB; at the task counts this kernel targets
/// the ring walk is the cheaper trade, and the *context switch* it triggers is O(1).
pub fn wake_blocked_on(resource: u32) {
    let g = critical::enter();
    unsafe {
        let root = *KERNEL.root.get();
        if !root.is_null() {
            // Tree walk, for the same reason as `wake_expired`: a waiter inside a group is
            // not in the level-1 ring, and if it is not woken here it never is.
            let mut waiter = |p: *mut TaskControlBlock| {
                if (*p).state == TaskState::Blocked && (*p).blocked_on == resource {
                    (*p).state = TaskState::Ready;
                    (*p).blocked_on = 0;
                    (*p).block_deadline = 0;
                }
            };
            walk_tree((*root).children_head, &mut waiter);
        }
    }
    drop(g);
}

/// Wake tasks whose bounded wait has expired.
///
/// # Safety
/// Called from the tick path, with preemption masked.
unsafe fn wake_expired() {
    let now = *KERNEL.ticks.get();
    let root = *KERNEL.root.get();
    if root.is_null() {
        return;
    }
    // The whole tree, not just the level-1 ring. A leaf blocked at depth 2+ is in no
    // level-1 ring, so the old flat walk never saw it: its sleep never expired and its
    // lock timeout never fired. That is invisible at depth 1, which is why it has to be
    // a tree walk before any group can exist.
    let mut expired = |p: *mut TaskControlBlock| {
        if (*p).state == TaskState::Blocked
            && (*p).block_deadline != 0
            && now >= (*p).block_deadline
        {
            (*p).state = TaskState::Ready;
            (*p).blocked_on = 0;
            (*p).block_deadline = 0;
        }
    };
    walk_tree((*root).children_head, &mut expired);
}

/// Times a blocking wait was attempted with no current task (interrupt or idle context).
///
/// Must stay zero: a non-zero value means some path called `sleep`/`lock`/`park` from a context
/// that has nothing to switch away from, so that wait did not happen at all. It is a counter as
/// well as a debug assertion because the assertion is compiled out in release, and this is
/// exactly the class of bug that looks like "the sleep woke up early".
pub fn blocks_without_current() -> u64 {
    let g = critical::enter();
    let n = unsafe { *KERNEL.blocks_without_current.get() };
    drop(g);
    n
}

/// Number of tasks currently blocked (waiting for a lock, or sleeping).
pub fn blocked_threads() -> usize {
    let g = critical::enter();
    let mut n = 0usize;
    unsafe {
        let head = *KERNEL.ring_head.get();
        if !head.is_null() {
            let mut p = head;
            loop {
                if (*p).state == TaskState::Blocked {
                    n += 1;
                }
                p = (*p).next;
                if p.is_null() || p == head {
                    break;
                }
            }
        }
    }
    drop(g);
    n
}

/// Set how many cores run the scheduler (validated against the target's maximum).
///
/// Cores at or above `n` park in a low-power wait loop instead of pulling tasks.
pub fn set_active_cores(n: usize) -> Result<(), ConfigError> {
    let max = crate::smp::max_cores().max(1);
    if n == 0 || n > max {
        return Err(ConfigError::Platform(
            "active_cores outside 1..=smp::max_cores() for this target",
        ));
    }
    if n > 1 && !crate::smp::supports_smp() {
        return Err(ConfigError::Platform(
            "this target has no cross-core interlock, so it cannot run more than one core",
        ));
    }
    let g = critical::enter();
    unsafe { (*KERNEL.config.get()).active_cores = n };
    drop(g);
    Ok(())
}

/// How many cores are running the scheduler.
pub fn active_cores() -> usize {
    let g = critical::enter();
    let n = unsafe { (*KERNEL.config.get()).active_cores };
    drop(g);
    n
}

use crate::SpawnError;

/// Link `node` into `parent`'s child ring, right after the parent's cursor, so a
/// newcomer runs as soon as its group's turn comes round rather than a lap later.
///
/// Handles the empty-ring case (self-linked) and both unset-parent pointers. The global
/// `ring_head` describes the level-1 ring only, so only a level-1 insert moves it.
///
/// # Safety
/// Caller holds a critical section. `node` must not already be linked.
unsafe fn insert_child(parent: *mut TaskControlBlock, node: *mut TaskControlBlock) {
    debug_assert!(!parent.is_null(), "spawn with no parent node");
    debug_assert!(
        (*parent).kind == crate::tcb::NodeKind::Group,
        "rrkernel: only a Group can own a child ring"
    );
    let anchor = {
        let cur = (*parent).current_child;
        if is_linked(cur) && (*cur).parent == parent {
            cur
        } else {
            (*parent).children_head
        }
    };
    if is_linked(anchor) {
        crate::ring::insert_after(anchor, node);
    } else {
        // First child: a self-linked ring of one.
        crate::ring::insert_after(ptr::null_mut(), node);
    }
    if (*parent).children_head.is_null() {
        (*parent).children_head = node;
    }
    if (*parent).current_child.is_null() {
        // A fresh group's cursor starts at its first child, so it is not treated as
        // "never dispatched" forever after.
        (*parent).current_child = node;
    }
    let root = *KERNEL.root.get();
    if parent == root {
        *KERNEL.ring_head.get() = node;
    }
    debug_assert!(crate::ring::check((*parent).children_head).is_ok());
}

/// Create a group node: a scheduling node with **no stack, no closure and no backend
/// resources**. It owns a quantum and a child ring, nothing else.
///
/// Deliberately does *not* call `crate::arch::create_task`: a group never runs, so it
/// must not consume a stack (on bare metal, the largest cost in the arena) nor a thread
/// or fiber on the OS-backed ports.
///
/// # Safety
/// Takes its own critical section, so no caller-side locking is required.
pub(crate) unsafe fn spawn_group_node(
    parent: *mut TaskControlBlock,
    quantum_ticks: u32,
) -> Result<*mut TaskControlBlock, SpawnError> {
    // Its own critical section, exactly like `spawn_internal`. A group spawn mutates the child
    // ring and the arena, and on metal the tick interrupt fires every slice; without this the
    // mutation races the ISR that walks the very ring being edited. That is what hung the
    // blue-pill the first time a group was spawned on hardware -- and it could not show on
    // Windows, where the tick source was stalled in this scenario and never raced it. The guard
    // is RAII, so every early return below releases it.
    let g = crate::critical::enter();
    if !(*arena()).is_ready() {
        return Err(SpawnError::NotInitialized);
    }
    if parent.is_null() {
        return Err(SpawnError::NotInitialized);
    }
    let tcb = alloc_tcb().ok_or(SpawnError::ArenaExhausted)?;
    (*tcb).id = take_id();
    (*tcb).state = TaskState::Ready;
    (*tcb).kind = crate::tcb::NodeKind::Group;
    (*tcb).parent = parent;
    (*tcb).children_head = ptr::null_mut();
    (*tcb).current_child = ptr::null_mut();
    (*tcb).slice_cycles = quantum_ticks.max(1);
    (*tcb).remaining_cycles = quantum_ticks.max(1);
    insert_child(parent, tcb);
    drop(g);
    Ok(tcb)
}

/// Convert a requested per-task quantum into ticks, rejecting anything that is not a
/// whole number of hardware ticks.
///
/// Every level of the tree runs off the single hardware tick, so a quantum has to be an
/// integer multiple of it. Rounding would silently change a timing request — the same
/// reason `configure()` refuses a slice the platform cannot honour instead of clamping —
/// so this is an error, with the numbers in it.
pub(crate) fn quantum_ticks_for(slice: Slice) -> Result<u32, SpawnError> {
    if matches!(slice, Slice::Default) {
        // Today's semantics: whatever `configure()` set, i.e. one tick.
        return Ok(1);
    }
    let cfg = crate::scheduler::config();
    let tick_ns = crate::arch::cycles_to_ns(cfg.slice_cycles);
    if tick_ns == 0 {
        return Err(SpawnError::NotInitialized);
    }
    let requested_ns = slice.as_nanos(cfg.timer_hz);
    if requested_ns == 0 || requested_ns < tick_ns {
        return Err(SpawnError::Backend(ConfigError::QuantumBelowTick {
            requested_ns,
            tick_ns,
        }));
    }
    if !requested_ns.is_multiple_of(tick_ns) {
        return Err(SpawnError::Backend(ConfigError::QuantumNotMultipleOfTick {
            requested_ns,
            tick_ns,
        }));
    }
    let ticks = requested_ns / tick_ns;
    if ticks > u32::MAX as u64 {
        return Err(SpawnError::Backend(ConfigError::QuantumTooLarge { ticks }));
    }
    Ok(ticks as u32)
}

/// Allocate and zero a TCB from the kernel arena. Caller holds a critical
/// section.
pub(crate) unsafe fn alloc_tcb() -> Option<*mut TaskControlBlock> {
    let p = (*arena()).alloc(
        core::mem::size_of::<TaskControlBlock>(),
        core::mem::align_of::<TaskControlBlock>(),
    )? as *mut TaskControlBlock;
    ptr::write_bytes(p as *mut u8, 0, core::mem::size_of::<TaskControlBlock>());
    // One more node for the tree walks to account for; see `walk_limit`.
    *KERNEL.nodes.get() += 1;
    Some(p)
}

/// Hand out the next task id.
pub(crate) unsafe fn take_id() -> u32 {
    let id = *KERNEL.next_id.get();
    *KERNEL.next_id.get() = id.wrapping_add(1);
    id
}

/// Kernel-context reclamation of finished tasks.
///
/// A returning task cannot unmap the stack it is still standing on, so
/// `task_exit` only *unlinks* and pushes itself onto `pending_free`. This
/// function — which must run in a context that is **not** on the finished task's
/// stack — performs the actual `Arena::free` calls. That is what keeps
/// `active_threads`, the ring and the arena consistent with each other: dead
/// nodes leave the ring in O(1), their memory is recycled soon after, and
/// repeated spawn/exit is steady-state.
///
/// # Where each port calls it
/// * **Cortex-M** — from `PendSV`, which runs on MSP (the kernel stack): called
///   *before* the switch, which is safe because the dying task's PSP stack is
///   not where the handler is running.
/// * **Win32** — from the tick thread, before the switch. `is_pinned` keeps the
///   TCB the tick thread still needs from being recycled underneath it.
/// * **POSIX fibres** — *after* the switch (the handler shares the interrupted
///   fibre's stack, so the dying stack must not be released from inside it; the
///   next handler invocation runs on a different fibre's stack).
///
/// # Safety
/// Kernel context, with the kernel lock/critical section held.
pub unsafe fn reclaim_finished_tasks() {
    let mut node = *KERNEL.pending_free.get();
    *KERNEL.pending_free.get() = ptr::null_mut();

    let ar = &mut *arena();
    while !node.is_null() {
        // `prev` doubles as the deferred-free list link for a dead TCB.
        let next = (*node).prev;

        debug_assert!(!(*node).is_linked(), "dead task still linked in the ring");
        debug_assert_eq!((*node).state, TaskState::Dead);

        if crate::arch::is_pinned(node) {
            // The port still needs this TCB — typically because it is the task
            // the tick thread has just taken the CPU away from, and its backend
            // resources (Win32 thread handle, fiber stack) must stay valid until
            // that switch has fully completed. Re-queue it for the next drain;
            // this pins at most one node, so the list stays bounded.
            (*node).prev = *KERNEL.pending_free.get();
            *KERNEL.pending_free.get() = node;
            node = next;
            continue;
        }

        // Let the backend release its own resources (Win32 thread handle, ...)
        // before the TCB itself is recycled.
        crate::arch::on_task_reclaimed(node);

        ar.free((*node).closure_block);
        ar.free((*node).stack_base);
        let n = node;
        node = next;
        ar.free(n as *mut u8);

        *KERNEL.reclaimed.get() += 1;
    }
}

/// Push a finished task onto the deferred-free list. Called by the trampoline
/// from the dying task's own context, with the ring already unlinked and the
/// counters already decremented.
///
/// # Safety
/// Critical section held; `tcb` must be unlinked and marked `Dead`.
pub(crate) unsafe fn push_pending_free(tcb: *mut TaskControlBlock) {
    (*tcb).prev = *KERNEL.pending_free.get();
    *KERNEL.pending_free.get() = tcb;
}

/// Bodies of `thread::spawn*`. Takes the critical section itself.
///
/// `parent` is the node the newcomer joins — the implicit root for a plain
/// `thread::spawn` — and `quantum_ticks` is its quantum in root-slice ticks (`1` is the
/// configured slice, which is what every plain-spawned task has always had). Returns the
/// new node so the tree-aware API can hand back a handle.
pub(crate) unsafe fn spawn_internal<F>(
    parent: *mut TaskControlBlock,
    quantum_ticks: u32,
    f: F,
    stack_size: usize,
) -> Result<*mut TaskControlBlock, crate::SpawnError>
where
    F: FnOnce() + Send + 'static,
{
    let g = critical::enter();

    let ar = arena();
    if !(*ar).is_ready() {
        drop(g);
        return Err(crate::SpawnError::NotInitialized);
    }

    // 1. TCB.
    let tcb = match alloc_tcb() {
        Some(t) => t,
        None => {
            drop(g);
            return Err(crate::SpawnError::ArenaExhausted);
        }
    };

    // 2. Closure blob (the *task body*), placed without any heap.
    let csize = crate::closure::block_size_for::<F>();
    let calign = crate::closure::block_align_for::<F>();
    let blob = match (*arena()).alloc(csize, calign) {
        Some(b) => b,
        None => {
            (*arena()).free(tcb as *mut u8);
            drop(g);
            return Err(crate::SpawnError::ArenaExhausted);
        }
    };
    crate::closure::place::<F>(blob, f);
    (*tcb).closure_block = blob;

    (*tcb).id = take_id();
    (*tcb).state = TaskState::Ready;
    (*tcb).slices_run = 0;
    (*tcb).switches = 0;
    // A spawned leaf joins the given parent, with the given quantum in root-slice ticks.
    // For a plain `thread::spawn` that parent is the root and the quantum is 1 — one
    // hardware tick — which is exactly what every level-1 task has always had.
    (*tcb).kind = crate::tcb::NodeKind::Leaf;
    (*tcb).parent = parent;
    (*tcb).slice_cycles = quantum_ticks.max(1);
    (*tcb).remaining_cycles = quantum_ticks.max(1);

    // 3. Backend-specific: stack, initial register frame, thread/fiber.
    if let Err(e) = crate::arch::create_task(tcb, stack_size) {
        crate::closure::drop_blob::<F>(blob);
        (*arena()).free(blob);
        (*arena()).free(tcb as *mut u8);
        drop(g);
        return Err(e);
    }

    // 4. Link into the parent's child ring, right after its cursor: the newcomer runs as
    //    soon as that ring's turn comes round, instead of waiting a whole lap. For a
    //    level-1 spawn this is the flat ring, so the old behaviour is preserved exactly.
    insert_child(parent, tcb);
    *KERNEL.total_threads.get() += 1;
    *KERNEL.active_threads.get() += 1;

    debug_assert!(crate::ring::check(KERNEL.current()).is_ok());

    // 5. Register the newcomer instantly: kick the switch path and restart the
    //    slice counter, so the new task gets a full, unreserved slice.
    if *KERNEL.running.get() {
        crate::arch::request_switch();
        // On bare metal the pending PendSV fires as soon as this critical
        // section exits; on the host the tick thread wakes (blocking on the
        // kernel lock until we drop it) and performs the switch plus the tick
        // reset.
    }
    drop(g);
    Ok(tcb)
}

// ---------------------------------------------------------------------------
// Nested scheduling tests
//
// These drive the *real* entry points (`on_tick` and `schedule_next`) against
// hand-built TCB trees: no timer, no hardware, no interrupts, deterministic. Every
// claim in docs/NESTED_SCHEDULING.md is a claim about the order these two produce,
// and an off-by-one in the drain order is exactly the bug that would otherwise only
// appear after hours of runtime.
//
// One `#[test]` function on purpose: the cases share the global `KERNEL`, and the lib
// test binary runs tests in parallel. They run in sequence instead.
// ---------------------------------------------------------------------------
#[cfg(all(test, feature = "std"))]
mod nested_tests {
    use super::*;
    use crate::tcb::NodeKind;

    /// A zeroed TCB, exactly as `alloc_tcb` produces one, leaked so it outlives the case.
    unsafe fn node(kind: NodeKind, quantum: u32, id: u32) -> *mut TaskControlBlock {
        let p = Box::into_raw(Box::new(core::mem::zeroed::<TaskControlBlock>()));
        ptr::write_bytes(p as *mut u8, 0, core::mem::size_of::<TaskControlBlock>());
        (*p).kind = kind;
        (*p).state = TaskState::Ready;
        (*p).id = id;
        (*p).slice_cycles = quantum;
        (*p).remaining_cycles = quantum;
        // Mirror `alloc_tcb`: this is what the walk budget is derived from, so a test tree
        // that skipped it would exercise a budget of two nodes and quietly prove nothing.
        *KERNEL.nodes.get() += 1;
        p
    }

    /// Link `kids` into a circular ring owned by `parent`, cursor at the first.
    unsafe fn attach(parent: *mut TaskControlBlock, kids: &[*mut TaskControlBlock]) {
        for (i, k) in kids.iter().enumerate() {
            let c = *k;
            (*c).parent = parent;
            if i == 0 {
                crate::ring::insert_after(ptr::null_mut(), c);
            } else {
                crate::ring::insert_after(kids[i - 1], c);
            }
        }
        (*parent).children_head = kids[0];
        (*parent).current_child = kids[0];
    }

    unsafe fn install(root: *mut TaskControlBlock, cur: *mut TaskControlBlock) {
        *KERNEL.root.get() = root;
        KERNEL.set_current(cur);
    }

    /// One hardware tick plus the switch the port would perform. Returns the node that is
    /// current afterwards — which may be the one that was already running, the "no switch"
    /// case the host backends have to tolerate.
    unsafe fn tick() -> *mut TaskControlBlock {
        on_tick();
        let next = schedule_next();
        if !next.is_null() {
            KERNEL.set_current(next);
        }
        KERNEL.current()
    }

    #[test]
    fn nested_scheduling_invariants() {
        unsafe {
            // --- depth 1 is the flat scheduler -------------------------------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let a = node(NodeKind::Leaf, 1, 1);
                let b = node(NodeKind::Leaf, 1, 2);
                let c = node(NodeKind::Leaf, 1, 3);
                attach(root, &[a, b, c]);
                install(root, a);
                let seq: Vec<u32> = (0..6).map(|_| (*tick()).id).collect();
                // `a` already held the CPU before the first tick, so the first tick hands
                // over to `b`: the flat rotation, unchanged.
                assert_eq!(seq, vec![2, 3, 1, 2, 3, 1], "depth-1 rotation");
            }

            // --- underflow: children finish early, so the lap repeats ---------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                // Group window 6 ticks, children 1 tick each: both children finish several
                // laps *inside* the one visit, which is the underflow case. The window must
                // not run out early, and the group's budget must not be reset by a lap.
                let g = node(NodeKind::Group, 6, 10);
                let c1 = node(NodeKind::Leaf, 1, 11);
                let c2 = node(NodeKind::Leaf, 1, 12);
                attach(root, &[g]);
                attach(g, &[c1, c2]);
                install(root, c1);
                let seq: Vec<u32> = (0..5).map(|_| (*tick()).id).collect();
                assert_eq!(
                    seq,
                    vec![12, 11, 12, 11, 12],
                    "children lap repeatedly inside one visit, alternating"
                );
                assert_eq!(
                    (*g).remaining_cycles,
                    1,
                    "the visit's budget is spent once per tick, not once per lap"
                );
                assert_eq!((*g).current_child, c2, "and the cursor advanced lap by lap");
            }

            // --- overflow: truncate mid-visit, resume without rewinding -------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g = node(NodeKind::Group, 10, 20);
                let c1 = node(NodeKind::Leaf, 8, 21);
                let c2 = node(NodeKind::Leaf, 8, 22);
                let sib = node(NodeKind::Leaf, 1, 23);
                attach(root, &[g, sib]);
                attach(g, &[c1, c2]);
                install(root, c1);
                let mut seq: Vec<u32> = Vec::new();
                for _ in 0..10 {
                    seq.push((*tick()).id);
                }
                // Ticks 1..7 keep c1 (no switch, its own 8-tick quantum still has budget);
                // tick 8 exhausts it and hands over to c2; ticks 9..10 run c2; the group's
                // 10-tick window closes on tick 10, so the turn leaves for the sibling with
                // c2 still unspent.
                assert_eq!(
                    seq,
                    vec![21, 21, 21, 21, 21, 21, 21, 22, 22, 23],
                    "c1 for 8 ticks, c2 for 2, then the window closes to the sibling"
                );
                assert_eq!((*c2).remaining_cycles, 6, "c2 keeps what it had left");
                let back = tick();
                assert_eq!((*back).id, 22, "the group resumes at the child it was on");
                assert_eq!(
                    (*c2).remaining_cycles,
                    6,
                    "and c2's budget is untouched: the drain only ever spends the running path"
                );
                assert_eq!((*g).current_child, c2, "the group's cursor was not reset");
                assert_eq!(
                    (*g).remaining_cycles,
                    10,
                    "armed to a full fresh visit during the dispatch: the drain for this tick \
                     ran before it, and the group was not on the running path"
                );
            }

            // --- mid-visit preemption keeps the exact remaining budget --------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g = node(NodeKind::Group, 3, 30);
                let leaf = node(NodeKind::Leaf, 10, 31);
                let sib = node(NodeKind::Leaf, 1, 32);
                attach(root, &[g, sib]);
                attach(g, &[leaf]);
                install(root, leaf);
                // Two ticks: the leaf's own 10-tick quantum still has budget, so no switch.
                assert_eq!((*tick()).id, 31);
                assert_eq!((*tick()).id, 31);
                // The third spends the group's 3-tick window, and the turn leaves.
                assert_eq!(
                    (*tick()).id,
                    32,
                    "the group's window closing moves the turn out"
                );
                assert_eq!((*leaf).remaining_cycles, 7, "the leaf kept 10 - 3");
                // Next visit: resumed, not re-armed. The leaf's budget is unchanged from
                // the moment the group's window closed, because the ticks in between ran a
                // different subtree and the drain only ever spends the running path.
                assert_eq!((*tick()).id, 31, "resumed when the group is visited again");
                assert_eq!((*leaf).remaining_cycles, 7, "resumed, not re-armed");
            }

            // --- every child blocked: bubble up, and do not burn the group's budget --
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g = node(NodeKind::Group, 5, 40);
                let b1 = node(NodeKind::Leaf, 1, 41);
                let b2 = node(NodeKind::Leaf, 1, 42);
                let other = node(NodeKind::Leaf, 1, 43);
                attach(root, &[g, other]);
                attach(g, &[b1, b2]);
                (*b1).state = TaskState::Blocked;
                (*b2).state = TaskState::Blocked;
                install(root, other);
                let before = (*g).remaining_cycles;
                let who = tick();
                assert_eq!(
                    (*who).id,
                    43,
                    "a group with nothing runnable must neither be switched to nor spin"
                );
                assert_eq!(
                    (*g).remaining_cycles,
                    before,
                    "an idle group's budget is paused, not spent"
                );
            }

            // --- a sleep inside a group expires even while its group is starved -----
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g = node(NodeKind::Group, 4, 50);
                let sleeper = node(NodeKind::Leaf, 1, 51);
                let busy = node(NodeKind::Leaf, 1, 52);
                attach(root, &[g, busy]);
                attach(g, &[sleeper]);
                install(root, busy);
                (*sleeper).state = TaskState::Blocked;
                (*sleeper).blocked_on = 0x1234;
                (*sleeper).block_deadline = *KERNEL.ticks.get() + 3;
                // The group is never dispatched during these ticks — the busy sibling holds
                // the CPU. The deadline must still expire, which is only possible if the
                // tick path walks the whole tree rather than the level-1 ring.
                for _ in 0..5 {
                    let _ = tick();
                }
                assert_ne!(
                    (*sleeper).state,
                    TaskState::Blocked,
                    "a deadline expires at depth even though its group was never scheduled"
                );
                assert_eq!((*sleeper).block_deadline, 0, "and the deadline is cleared");
                assert_eq!((*sleeper).blocked_on, 0, "and the resource wait is cleared");
            }

            // --- depth 3: the drain reaches every ancestor -------------------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g1 = node(NodeKind::Group, 6, 70);
                let g2 = node(NodeKind::Group, 4, 71);
                let leaf = node(NodeKind::Leaf, 2, 72);
                attach(root, &[g1]);
                attach(g1, &[g2]);
                attach(g2, &[leaf]);
                install(root, leaf);
                // The leaf is g2's only child, so it keeps the CPU while both inner
                // windows drain: the ticking is what makes the ancestors spend anything.
                let seq: Vec<u32> = (0..6).map(|_| (*tick()).id).collect();
                assert_eq!(
                    seq,
                    vec![72; 6],
                    "a lone child keeps the CPU through its parents"
                );
                // Every level's window closes as the ticks go by, and the freshest level
                // re-arms from the expiry point outward. Assert the invariant that matters —
                // the subtree keeps running and every budget is positive again — rather than
                // the exact per-tick arithmetic, which drain-then-dispatch ordering makes
                // easy to mis-guess (as two of my earlier expectations in this file were).
                let _ = tick();
                assert_eq!(
                    (*tick()).id,
                    72,
                    "a lone child keeps the CPU after its parents' windows close"
                );
                assert!(
                    (*g1).remaining_cycles > 0
                        && (*g2).remaining_cycles > 0
                        && (*leaf).remaining_cycles > 0,
                    "every level is armed again: outer {}, inner {}, leaf {}",
                    (*g1).remaining_cycles,
                    (*g2).remaining_cycles,
                    (*leaf).remaining_cycles
                );
            }

            // --- a level-1 leaf with a multi-tick quantum: no switch until it expires --
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let a = node(NodeKind::Leaf, 5, 80);
                let b = node(NodeKind::Leaf, 1, 81);
                attach(root, &[a, b]);
                install(root, a);
                let before = *KERNEL.switches.get();
                for _ in 0..4 {
                    assert_eq!(
                        (*tick()).id,
                        80,
                        "an unspent quantum means the same task is returned: the case the host \
                         backends have to tolerate"
                    );
                }
                assert_eq!(
                    *KERNEL.switches.get(),
                    before,
                    "four ticks with no switch at all"
                );
                assert_eq!(
                    (*tick()).id,
                    81,
                    "the fifth tick spends the quantum and moves on"
                );
                assert_eq!(*KERNEL.switches.get(), before + 1, "exactly one switch");
            }

            // --- a lock wake reaches a waiter inside a group -----------------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g = node(NodeKind::Group, 4, 90);
                let sleeper = node(NodeKind::Leaf, 1, 91);
                let busy = node(NodeKind::Leaf, 1, 92);
                attach(root, &[g, busy]);
                attach(g, &[sleeper]);
                install(root, busy);
                (*sleeper).state = TaskState::Blocked;
                (*sleeper).blocked_on = 0x99;
                wake_blocked_on(0x99);
                assert_eq!(
                    (*sleeper).state,
                    TaskState::Ready,
                    "a waiter inside a group must be reachable by the tree walk"
                );
                assert_eq!((*sleeper).blocked_on, 0, "and the resource wait is cleared");
                assert_eq!(
                    (*sleeper).block_deadline,
                    0,
                    "a lock wake must not invent a deadline"
                );
            }

            // --- a stale cursor (child died and unlinked) is survived --------------
            {
                let root = node(NodeKind::Group, u32::MAX, 0);
                let g = node(NodeKind::Group, 2, 60);
                let dead = node(NodeKind::Leaf, 1, 61);
                let live = node(NodeKind::Leaf, 1, 62);
                attach(root, &[g]);
                // The ring's *head* stays valid (this is what `exit_task` guarantees when it
                // repairs a parent's head and cursor); only the group's cursor is left
                // pointing at a child that has since finished and unlinked itself.
                attach(g, &[live, dead]);
                crate::ring::unlink(dead);
                (*dead).state = TaskState::Dead;
                (*g).current_child = dead;
                install(root, live);
                let who = tick();
                assert_eq!(
                    (*who).id,
                    62,
                    "a cursor pointing at a dead, unlinked child must not wedge the group"
                );
                assert_eq!((*g).current_child, live, "and the cursor is repaired");
            }

            // --- a task that blocks mid-quantum must not be handed the CPU back -----
            {
                // The board bug, in one case. `sleep_until_tick` blocks and re-checks in a loop,
                // relying on the switch actually happening; a task that blocks with quantum left
                // kept `remaining_cycles != 0`, so the no-switch path returned it and the switch
                // never happened. Every sleeper on every port spun there for ever.
                let root = node(NodeKind::Group, u32::MAX, 0);
                let blocker = node(NodeKind::Leaf, 5, 800);
                let other = node(NodeKind::Leaf, 1, 801);
                attach(root, &[blocker, other]);
                install(root, blocker);
                (*blocker).state = TaskState::Blocked;
                (*blocker).block_deadline = 100;
                (*blocker).remaining_cycles = 4; // plenty of quantum left
                let who = tick();
                assert_eq!(
                    (*who).id,
                    801,
                    "a blocked task must not be dispatched again just because it has quantum left"
                );
            }

            // --- deep nesting: bounded by memory, not by a constant -----------------
            {
                // 64 nested groups with one leaf at the bottom. The fixed depth cap this
                // replaces returned null past 32, *silently*, which turned a legal tree into
                // an unreachable subtree whose tasks never ran again. That is the starvation
                // this design has to rule out, so it gets a test rather than a comment.
                let root = node(NodeKind::Group, u32::MAX, 0);
                let mut deep = root;
                for i in 0..64u32 {
                    let g = node(NodeKind::Group, 3, 400 + i);
                    attach(deep, &[g]);
                    deep = g;
                }
                let leaf = node(NodeKind::Leaf, 2, 399);
                attach(deep, &[leaf]);
                install(root, leaf);
                let who = tick();
                assert_eq!(
                    (*who).id,
                    399,
                    "a leaf 65 levels down must still be reached and run"
                );
            }

            // --- no leaf starves, and no run outlasts its quantum -------------------
            {
                // A mixed tree: a level-1 leaf, a group with two leaves and a nested group
                // holding two more. This is the headline promise of nested scheduling --
                // every runnable task gets time -- and the run-length half of it is what
                // "inside a slot, time is continuous" means from a leaf point of view.
                let root = node(NodeKind::Group, u32::MAX, 0);
                let a = node(NodeKind::Leaf, 2, 500);
                let g1 = node(NodeKind::Group, 5, 501);
                let b = node(NodeKind::Leaf, 2, 502);
                let c = node(NodeKind::Leaf, 3, 503);
                let g2 = node(NodeKind::Group, 4, 504);
                let d = node(NodeKind::Leaf, 2, 505);
                let e = node(NodeKind::Leaf, 1, 506);
                attach(root, &[a, g1]);
                attach(g1, &[b, c, g2]);
                attach(g2, &[d, e]);
                install(root, a);

                let ids = [500u32, 502, 503, 505, 506];
                let quanta = [2u32, 2, 3, 2, 1];
                let mut served = [0u32; 5];
                let mut last = 0u32;
                let mut run = 0u32;
                for _ in 0..200 {
                    let id = (*tick()).id;
                    let i = ids
                        .iter()
                        .position(|x| *x == id)
                        .expect("every tick must return a leaf of this tree");
                    served[i] += 1;
                    if id == last {
                        run += 1;
                    } else {
                        if last != 0 {
                            let prev = ids.iter().position(|x| *x == last).unwrap();
                            assert!(
                                run <= quanta[prev],
                                "leaf {} ran {run} ticks, longer than its quantum {}",
                                last,
                                quanta[prev]
                            );
                        }
                        last = id;
                        run = 1;
                    }
                }
                for i in 0..5 {
                    assert!(
                        served[i] != 0,
                        "leaf {} never ran: a nested group starved it",
                        ids[i]
                    );
                }
                assert_eq!(
                    served.iter().sum::<u32>(),
                    200,
                    "no tick was wasted, and none went outside the tree"
                );
            }

            // --- a malformed tree terminates instead of spinning --------------------
            {
                // No application can build this with the public API (a parent is only ever
                // set at spawn, and the result is a tree), so it stands in for a future bug
                // elsewhere. It matters *now* because the walks are bounded by the node count
                // rather than by a depth constant: that bound is the only reason the fixed cap
                // could be removed at all, so it is the thing to test.
                let root = node(NodeKind::Group, u32::MAX, 0);
                let x = node(NodeKind::Group, 4, 600);
                let y = node(NodeKind::Leaf, 1, 601);
                attach(root, &[x]);
                attach(x, &[y]);
                (*x).parent = y; // a cycle: the group parent is its own child
                install(root, y);
                drain_current_path();
                let _ = outermost_exhausted(y);
                let mut visited = 0u32;
                walk_tree((*root).children_head, &mut |_| visited += 1);
                assert!(visited != 0, "the walk visited something before giving up");
            }

            // --- the walk visits every node, including siblings of the head leaf -----
            {
                // This is the shape the board found. The wake sweep walks the tree from the
                // ring head, and on the blue-pill the head is *task 0, a leaf* -- with the
                // groups after it. A walk that stopped once it had ascended to the head's
                // parent visited exactly one node, so every later task sleep never expired
                // and the board sat there with nothing to print after the first two lines.
                let root = node(NodeKind::Group, u32::MAX, 0);
                let head = node(NodeKind::Leaf, 1, 700);
                let g = node(NodeKind::Group, 4, 701);
                let inner = node(NodeKind::Leaf, 1, 702);
                let tail = node(NodeKind::Leaf, 1, 703);
                attach(root, &[head, g, tail]);
                attach(g, &[inner]);
                let mut seen: Vec<u32> = Vec::new();
                walk_tree((*root).children_head, &mut |p| seen.push((*p).id));
                assert_eq!(
                    seen.len(),
                    4,
                    "the walk must reach the head leaf, the group, its child and the last leaf"
                );
                for id in [700u32, 701, 702, 703] {
                    assert!(seen.contains(&id), "the walk missed node {}", id);
                }
            }
        }
    }
}

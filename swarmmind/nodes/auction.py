"""Tier 2: the market-based task auction, and the swarm's entire self-healing mechanism.

**There is no LLM anywhere in this file.** That is the point. The demo's central claim --
the hivemind can go offline and the swarm keeps working -- is true because allocation,
execution and recovery all live here and none of them consult Tier 3.

It is a single-round reverse auction with capability gating and greedy sequential award.
Call it that. It is **not** CBBA: no bundles, no consensus rounds.

Self-healing is emergent rather than special-cased. Tasks are regenerated from world
state every cycle, so when a robot stops heartbeating and its assignment is released,
the work it was doing simply reappears in the next cycle's task list and is bid on
again. Nothing has to notice that a robot died.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..contracts import topics
from ..control.planner import UNREACHABLE
from ..sim import grid
from ..sim.robot import DESTROYED, LANE_INDEX, OUT_OF_COMMS
from .skill_executor import Assignment, eligible
from .tasks import (
    RANK,
    OpenTask,
    TaskGenerator,
    abandoned_preemptions,
    hazard_preemptions,
    nearest_haven,
)


@dataclass(frozen=True)
class BidWeights:
    """Coefficients convert every term into seconds-equivalent, so a bid is a time."""

    battery: float = 20.0
    load: float = 15.0
    hazard: float = 40.0


#: Seconds of radio silence before an out-of-contact robot's task is offered to somebody
#: else as well. Not the same thing as detecting a loss -- see `AuctionNode.__init__`.
REOFFER_TIMEOUT_S = 30.0


class AuctionNode:
    #: Announced tasks per cycle, sized to the robots that can actually take them.
    #:
    #: A flat cap of 24 was the original design, sized against 22.4 ms fine-grid
    #: distance fields (MEASUREMENTS.md M-4). Bidding uses the *coarse* nav grid
    #: instead -- roughly 16x cheaper, and cached for the whole mission because
    #: `passable` never changes -- so the budget is far larger than that cap assumed.
    #: At 512 robots the flat cap was the binding constraint on the entire swarm: only
    #: 24 robots could be given work per second, and the rest idled.
    MIN_ANNOUNCED = 24
    MAX_ANNOUNCED = 256
    #: Announce this many tasks per free robot, so there is competition to bid on.
    ANNOUNCE_PER_FREE = 1.5

    #: Tasks at this rank or better may take a robot off lower-priority work.
    #:
    #: Without this, a rescue starves behind exploration: the auction awards only to
    #: *free* robots, so once every carrier is off searching, a confirmed casualty sits
    #: unclaimed no matter how it is ranked. That also breaks self-healing outright --
    #: the task released when a robot dies simply goes unfilled. Ranking work is
    #: meaningless if the allocator cannot act on the ranking.
    PREEMPT_RANK = RANK["clear_debris"]

    #: How much better a busy robot's bid must be before it is worth taking it off its
    #: work. Bids are times, so 0.5 means "at least twice as quick to get there".
    #:
    #: Not zero, and not one. Preempting is not free -- the abandoned task goes back on
    #: the market and its progress is lost -- so a marginal improvement is not worth the
    #: churn. But a *fallback* (the old behaviour, equivalent to a ratio of 0) let a free
    #: carrier 258 m away beat a busy one 4 m away, because the busy one was never
    #: considered while any free robot existed at all.
    PREEMPT_RATIO = 0.5

    #: Carriers held back from `explore` so one is free when a casualty is dug out.
    #:
    #: **Off, and not yet honestly measured.** The A/B that would have settled it (0 / 15
    #: / 40, giving 66.50 / 68.00 / 64.50 rescued) straddled a `demo.yaml` edit, so its
    #: arms are not comparable -- see M-60a. 40 was clearly worse; 15 was +1.5 rescues,
    #: inside the seed spread. Re-run it on a stable scenario before turning this on.
    carrier_reserve = 0

    def __init__(self, world, nav, weights: BidWeights | None = None,
                 orphan_timeout: float | None = None, genes=None,
                 commander=None) -> None:
        self.nav = nav
        #: Tier 3a. When present, *search* work is territorial: only the squadron that
        #: owns the ground may bid on exploring it. Rescue work is never territorial.
        #: `None` is the pre-command behaviour and the gate's control arm.
        self.commander = commander
        #: Evolved Tier-2 behaviour, one genome per robot (`training.mapelites.genome`).
        #: `None` is the classical baseline and the gate's control arm -- every scalar
        #: below stays a scalar, and the bid is byte-identical to the pre-D9 auction.
        self.genes = genes
        self.gen = TaskGenerator()
        self.w = weights or BidWeights(**world.scn.bid_weights)
        self.orphan_timeout = (
            orphan_timeout if orphan_timeout is not None else world.scn.rates.orphan_timeout_s
        )
        #: How long a robot may be out of radio contact before its task is *also* offered
        #: to somebody else. Deliberately much longer than `orphan_timeout`.
        #:
        #: These were one number, and sharing it was expensive. `orphan_timeout` is 2 s
        #: because that is what the self-healing claim is measured against: kill a robot,
        #: see its task reassigned inside 3 s. But comms coverage is ~87%, so a carrier
        #: crossing a dead spot for more than two seconds had its casualty duplicated
        #: onto another carrier -- **519 extract awards for 43 casualties, twelve awards
        #: each** (MEASUREMENTS.md M-35). Both carriers walked the same errand, one of
        #: them wasted, over and over.
        #:
        #: A destroyed robot still releases on `orphan_timeout`, so the demo moment is
        #: untouched. Silence alone now has to persist far longer before the swarm spends
        #: a second robot on the same job.
        self.reoffer_timeout = max(self.orphan_timeout, REOFFER_TIMEOUT_S)
        self.last_hb = np.full(world.n, world.t, dtype=np.float64)
        self._seq = 0
        self._haz_field: np.ndarray | None = None
        self.stats = {"announced": 0, "awarded": 0, "orphaned": 0, "no_bidder": 0}

    # ------------------------------------------------------------------ heartbeat

    def heartbeat(self, world) -> None:
        """Called at ``rates.heartbeat_hz``. Silence is how the swarm learns of a loss."""
        ok = (world.status <= OUT_OF_COMMS) & world.in_comms
        self.last_hb[ok] = world.t

    # ------------------------------------------------------------------ cycle

    def step(self, world, ex, tracker, emit=None, bus=None) -> None:
        emit = emit or (lambda *a, **k: None)
        self._preempt_hazard(world, ex, emit)
        self.check_orphans(world, ex, emit)

        free = ex.free_mask(world) & world.in_comms & (world.status <= OUT_OF_COMMS)
        n_free = int(free.sum())
        cap = int(np.clip(self.ANNOUNCE_PER_FREE * n_free,
                          self.MIN_ANNOUNCED, self.MAX_ANNOUNCED))

        tasks = self.gen.generate(world, ex, tracker)
        announced = tasks[:cap]
        self.stats["announced"] += len(announced)
        if bus is not None:
            bus.publish(topics.TASKS_AVAILABLE, announced)
        if not announced:
            return

        self._build_hazard_field(world)

        # A robot the unit policy has staged is held back from search work only. Rescue
        # work still sees it -- that is what it is waiting for. None without a policy.
        reserved = ex.reserved_mask(world) if ex.policy_goal else None

        for task in announced:
            pool = free
            if reserved is not None and task.kind not in ("extract", "clear_debris",
                                                          "retreat"):
                pool = free & ~reserved
            i, score = self._best_bidder(world, task, pool)
            if task.rank <= self.PREEMPT_RANK and not task.backup:
                # Preemption *competes*; it is not a last resort. See PREEMPT_RATIO.
                #
                # Backups are exempt. A redundant robot on a casualty is insurance
                # against the first one stalling, and insurance that pulls a working
                # robot off its own job is not free -- it is a swap. A backup takes an
                # idle robot or it takes nobody.
                j, pscore = self._best_preemptable(world, ex, task)
                if j >= 0 and (i < 0 or pscore < score * self.PREEMPT_RATIO):
                    self._take(world, ex, task, j, emit)
                    i, score = j, pscore
            if i < 0:
                self.stats["no_bidder"] += 1
                continue
            self._award(world, ex, task, i, score, emit, bus)
            free[i] = False

    # ------------------------------------------------------------------ bidding

    def _best_bidder(self, world, task: OpenTask, free: np.ndarray) -> tuple[int, float]:
        elig = eligible(world, task.kind) & free

        # Keep some carriers out of pure search.
        #
        # Carriers explore when they have nothing else to do, which is right -- an idle
        # lane is wasted. But measured at t=214 on seed 42, **51 of 94 living carriers
        # were off exploring** while 12 dug-out casualties lay waiting, and the nearest
        # carrier to one was 5-45 m away and already committed elsewhere. Reserving a few
        # keeps a carrier genuinely available when a digger finishes.
        #
        # `investigate` is deliberately not reserved against: it is what turns a contact
        # into a known casualty, so it feeds the chain rather than competing with it.
        if task.kind == "explore" and self.carrier_reserve > 0:
            grip = world.actuator == LANE_INDEX["gripper"]
            if int((elig & grip).sum()) <= self.carrier_reserve:
                elig = elig & ~grip
        # Territory. Without this every scout bids on the globally-best frontier, so
        # 352 of them converge on the same ground and the swarm runs at ~4% search
        # efficiency (MEASUREMENTS.md M-35). Only search work is fenced -- a casualty
        # is everybody's business.
        if self.commander is not None and task.kind in ("explore", "sweep", "relay"):
            fenced = elig & self.commander.in_own_territory(world, task.target)
            # Never let a fence idle a robot that could work: if nobody in the owning
            # squadron can take it, anyone may.
            if fenced.any():
                elig = fenced
        if not elig.any():
            return -1, 0.0

        # One BFS per task target, cached for the mission by NavFields. Every robot's
        # travel term is then a single array lookup -- the expensive term does not
        # involve N. Calling A* per (robot, task) pair would be ~10k searches/second
        # at 512 robots.
        # One field per (target, chassis). A robot whose locomotion cannot reach the
        # target must bid **infinity**, not merely a large number.
        #
        # `UNREACHABLE` is `1e9`, a finite sentinel. Dividing it by `v_max` gives a huge
        # but finite bid, `np.isfinite` accepts it, and the robot wins any round where no
        # reachable robot happens to be free -- then stands still for the rest of the
        # mission holding a task it can never execute. Measured at t=120 on seed 42:
        # **156 of the 326 robots parked at base were assigned goals their own chassis
        # cannot reach.** The comment above this loop asserted they "never win"; that was
        # only true when a reachable bidder was available.
        travel = np.full(world.n, np.inf)
        unreachable = np.zeros(world.n, dtype=bool)
        for c in range(len(self.nav.chassis)):
            m = world.chassis == c
            if not m.any():
                continue
            field = self.nav.field(c, task.target[0], task.target[1])
            d = self.nav.distance_at(c, field, world.pos[:, 0], world.pos[:, 1])
            far = d[m] >= UNREACHABLE * 0.5
            unreachable[np.nonzero(m)[0][far]] = True
            travel[m] = np.where(far, 0.0, d[m] / world.v_max[m])

        # Unreachable is applied *after* the bid expression, not as `inf` inside it:
        # the genome path multiplies `travel` by a battery deficit, and `inf * 0.0` is
        # `nan`, which then depends on numpy's NaN sort order to be excluded. A boolean
        # mask says what is meant and leaves the arithmetic finite.
        bid = self._bid(world, task, travel)
        bid = np.where(elig & ~unreachable, bid, np.inf)
        # Ties break on robot index, never on array order (CLAUDE.md invariant #5).
        i = int(np.lexsort((np.arange(world.n), bid))[0])
        return (i, float(bid[i])) if np.isfinite(bid[i]) else (-1, 0.0)

    def _bid(self, world, task: OpenTask, travel: np.ndarray) -> np.ndarray:
        """A bid is a time: every term is coefficient x quantity, in seconds-equivalent.

        With no genome this is the hand-tuned baseline unchanged. With one, each
        coefficient becomes an `(n,)` array and the whole expression stays a single
        vectorised statement -- 768 robots bid in one pass, which is the only reason the
        auction can run at 1 Hz at this swarm size.
        """
        g = self.genes
        if g is None:
            return (
                travel
                + self.w.battery * (1.0 - world.battery)
                + self.w.hazard * self._hazard_exposure(world)
            )

        bid = (
            travel * g.distance_penalty
            + self.w.battery * (1.0 - world.battery)
            + g.hazard_aversion * self._hazard_exposure(world)
        )

        # A robot below its reserve bids as if the trip were longer, in proportion to how
        # far past the reserve it is. Not a hard cutoff: a nearly-flat robot standing next
        # to a casualty should still take it.
        deficit = np.maximum(g.battery_reserve - world.battery, 0.0)
        bid = bid + travel * deficit * 4.0

        # Leaving the connected component is a real cost, not a prohibition -- a scout
        # with a low tether is exactly the risk-taking phenotype worth evolving.
        bid = bid + np.where(world.in_comms, 0.0, g.comms_tether)

        # Exploration value pulls the other way: `task.value` is the unexplored gain the
        # generator scored this frontier at.
        if task.kind in ("explore", "sweep"):
            bid = bid - g.frontier_gain * float(task.value)
            bid = bid + g.revisit_penalty * self._explored_at(world)
        return bid

    def _explored_at(self, world) -> np.ndarray:
        """How thoroughly each robot's current surroundings are already covered."""
        ix, iy = grid.world_to_cell(world.pos[:, 0], world.pos[:, 1], world.cell, world.shape)
        return world.explored[iy, ix].astype(np.float64)

    def _best_preemptable(self, world, ex, task: OpenTask) -> tuple[int, float]:
        """The best bid available from robots on strictly lower-priority work.

        Finds only -- taking the robot is `_take`. The split is the fix for a real
        failure: preemption used to run *only* when no free robot could bid at all, which
        makes it a fallback rather than a comparison. Measured on seed 42, casualty v6 was
        awarded to carriers 253-258 m away while a free carrier stood 4 m from it; three
        free carriers existed, so the "nobody free" branch never fired and the closer,
        busy one was never even considered. Ranking work is meaningless if the allocator
        cannot act on the ranking (M-15 made that point once already, about a different
        half of the same mechanism).

        Only rescue-tier tasks may preempt, and only against work that ranks worse, so a
        carrier can be pulled off a frontier for a casualty but never off a casualty for
        a frontier. Bids are computed exactly as in a normal round.
        """
        busy = np.zeros(world.n, dtype=bool)
        for i, a in enumerate(ex.assignment):
            if a is None or world.status[i] > OUT_OF_COMMS or not world.in_comms[i]:
                continue
            if RANK.get(a.kind, 99) > task.rank:
                busy[i] = True
        if not busy.any():
            return -1, 0.0
        return self._best_bidder(world, task, busy)

    def _take(self, world, ex, task: OpenTask, i: int, emit) -> None:
        """Release robot ``i`` from what it was doing. The caller awards it the task."""
        was = ex.assignment[i]
        ex.release(int(i), f"{world.robot_ids[i]} pulled off {was.kind} for {task.kind}")
        self.stats["preempted"] = self.stats.get("preempted", 0) + 1
        emit("task_orphaned", f"{was.task_id} dropped: {world.robot_ids[i]} reassigned "
                              f"to higher-priority {task.kind}",
             robot=world.robot_ids[i], task=was.task_id, pos=tuple(was.target))

    def _build_hazard_field(self, world) -> None:
        """Distance from every coarse cell to the nearest *observed* hazard cell.

        Only `hazard_known` -- bidding on the ground-truth hazard disc would let the
        swarm route around fire it has never seen (CLAUDE.md invariant #3).
        """
        if not world.hazard_known.any():
            self._haz_field = None
            return
        f = self.nav.factor
        h, w = world.hazard_known.shape
        ph, pw = -h % f, -w % f
        m = np.pad(world.hazard_known, ((0, ph), (0, pw)), constant_values=False)
        coarse = m.reshape(m.shape[0] // f, f, m.shape[1] // f, f).any(axis=(1, 3))
        # Hazard proximity is about fire, not locomotion, so one field on the
        # middle-capability chassis stands in for all of them.
        self._haz_field = (grid.distance_field_multi(self.nav.coarse_of(1), coarse)
                           * self.nav.coarse_cell)

    def _hazard_exposure(self, world) -> np.ndarray:
        if self._haz_field is None:
            return np.zeros(world.n)
        d = self.nav.distance_at(1, self._haz_field, world.pos[:, 0], world.pos[:, 1])
        return 1.0 / (1.0 + np.maximum(d, 0.0))

    # ------------------------------------------------------------------ award / orphan

    def _award(self, world, ex, task: OpenTask, i: int, score: float, emit, bus) -> None:
        self._seq += 1
        task_id = f"{task.kind}_{self._seq}"
        ex.assign(
            world, i,
            Assignment(task_id=task_id, kind=task.kind, target=tuple(task.target),
                       victim=task.victim, report=task.report, assigned_at=world.t),
            why=f"Bid won: {world.robot_ids[i]} lowest of the capable bidders ({score:.0f}s)",
        )
        self.stats["awarded"] += 1
        emit("task_awarded", f"{task_id} -> {world.robot_ids[i]}",
             robot=world.robot_ids[i], task=task_id, pos=tuple(task.target))
        if bus is not None:
            bus.publish(topics.assigned_task(world.robot_ids[i]),
                        {"task_id": task_id, "kind": task.kind, "target": tuple(task.target)})

    def check_orphans(self, world, ex, emit=None) -> bool:
        """Release the tasks of robots that have stopped reporting. True if any were.

        **Called at heartbeat rate, not on the auction cycle.** Detecting a loss only
        when the next auction happens to come round puts the worst case at
        `orphan_timeout + auction_period` -- exactly 3.0 s at the shipped rates, which
        is the number the self-healing claim is measured against, with no margin at all.
        Detecting promptly and letting the caller force an immediate auction cuts it to
        roughly the timeout plus one tick.
        """
        emit = emit or (lambda *a, **k: None)
        found = False
        quiet = world.t - self.last_hb
        silent = quiet > self.orphan_timeout
        for i in np.nonzero(silent)[0]:
            a = ex.assignment[i]
            if a is None or a.orphaned:
                continue
            if world.status[i] >= DESTROYED:
                ex.release(int(i), f"{world.robot_ids[i]} destroyed")
            elif quiet[i] <= self.reoffer_timeout:
                # Out of radio contact, but not for long enough to assume anything is
                # wrong. Leave it alone.
                continue
            else:
                # Out of contact long enough to be worth covering. Re-offer the task
                # without taking it away: releasing it would free the robot to bid on
                # something else, and for a relay that means abandoning the post the rest
                # of the swarm is talking through.
                a.orphaned = True
                ex.mark_dirty()
            self.stats["orphaned"] += 1
            found = True
            emit("task_orphaned", f"{a.task_id} orphaned: {world.robot_ids[i]} is not responding",
                 robot=world.robot_ids[i], task=a.task_id, pos=tuple(a.target))
        return found

    def _preempt_hazard(self, world, ex, emit) -> None:
        # Hazard and abandonment pull robots out for the same reason and by the same
        # mechanism; the only difference is who decided.
        for i in hazard_preemptions(world, ex) + abandoned_preemptions(world, ex):
            haven = nearest_haven(world, i)
            if haven is None:
                continue
            self._seq += 1
            ex.assign(
                world, i,
                Assignment(task_id=f"retreat_{self._seq}", kind="retreat", target=haven,
                           assigned_at=world.t),
                why=f"{world.robot_ids[i]} retreating from hazard",
            )

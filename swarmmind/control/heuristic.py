"""Greedy nearest-capable allocation: the classical baseline.

Two jobs, both load-bearing (CLAUDE.md working rules):

1. **The baseline the gate measures against.** The auction, and anything learned, must
   beat this on held-out seeds. A heuristic winning is a result, not a failure.
2. **The fallback** if the auction misbehaves.

It consumes exactly the same task list as the auction (nodes/tasks.py), so the two
differ only in allocation -- which is what makes comparing them meaningful. The
difference: this picks the nearest capable robot by straight-line distance and ignores
battery and hazard entirely.
"""

from __future__ import annotations

import numpy as np

from ..nodes.skill_executor import Assignment, SkillExecutor, eligible
from ..nodes.tasks import TaskGenerator, hazard_preemptions, nearest_haven
from ..sim.robot import DESTROYED, OUT_OF_COMMS


class GreedyAssigner:
    def __init__(self, world, period: float = 1.0) -> None:
        self.period = period
        self.gen = TaskGenerator()
        self._next_t = 0.0
        self._seq = 0
        self.stats = {"announced": 0, "awarded": 0, "orphaned": 0, "no_bidder": 0}
        self.last_hb = np.full(world.n, 0.0)
        self.orphan_timeout = world.scn.rates.orphan_timeout_s

    def due(self, world) -> bool:
        return world.t >= self._next_t

    def heartbeat(self, world) -> None:
        ok = (world.status <= OUT_OF_COMMS) & world.in_comms
        self.last_hb[ok] = world.t

    def check_orphans(self, world, ex, emit=None) -> bool:
        """Same contract as AuctionNode.check_orphans -- run at heartbeat rate."""
        emit = emit or (lambda *a, **k: None)
        found = False
        for i in np.nonzero((world.t - self.last_hb) > self.orphan_timeout)[0]:
            a = ex.assignment[i]
            if a is None or a.orphaned:
                continue
            if world.status[i] >= DESTROYED:
                ex.release(int(i), f"{world.robot_ids[i]} destroyed")
            else:
                a.orphaned = True
                ex.mark_dirty()
            self.stats["orphaned"] += 1
            found = True
            emit("task_orphaned", f"{a.task_id} orphaned", robot=world.robot_ids[i],
                 task=a.task_id, pos=tuple(a.target))
        return found

    def step(self, world, ex: SkillExecutor, tracker=None, emit=None, bus=None) -> None:
        if not self.due(world):
            return
        self._next_t = world.t + self.period
        emit = emit or (lambda *a, **k: None)

        for i in hazard_preemptions(world, ex):
            haven = nearest_haven(world, i)
            if haven is not None:
                self._seq += 1
                ex.assign(world, i, Assignment(task_id=f"retreat_{self._seq}", kind="retreat",
                                               target=haven, assigned_at=world.t),
                          why="retreating from hazard")

        self.check_orphans(world, ex, emit)

        tasks = self.gen.generate(world, ex, tracker)
        self.stats["announced"] += len(tasks)
        free = ex.free_mask(world) & world.in_comms

        for t in tasks:
            cap = eligible(world, t.kind) & free
            if not cap.any():
                self.stats["no_bidder"] += 1
                continue
            idx = np.nonzero(cap)[0]
            d = np.linalg.norm(world.pos[idx] - np.array(t.target), axis=1)
            best = int(idx[np.lexsort((idx, d))[0]])
            self._seq += 1
            task_id = f"{t.kind}_{self._seq}"
            ex.assign(world, best,
                      Assignment(task_id=task_id, kind=t.kind, target=tuple(t.target),
                                 victim=t.victim, report=t.report, assigned_at=world.t),
                      why=f"nearest capable for {t.kind}")
            self.stats["awarded"] += 1
            emit("task_awarded", f"{task_id} -> {world.robot_ids[best]}",
                 robot=world.robot_ids[best], task=task_id, pos=tuple(t.target))
            free[best] = False

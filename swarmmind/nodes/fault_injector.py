"""The scripted failure -- the demo's "watch it self-heal" beat.

**State-triggered, not wall-clock.** A fixed timestamp lands at an awkward moment if the
run goes long or short; firing on mission state keeps the beat synced to what the
audience is actually watching (MVP spec section 10).

The injector only destroys a robot. It does not touch the auction, and the auction does
not know it exists. Recovery happens because the robot stops heartbeating -- exactly what
would happen if it had failed for any other reason.
"""

from __future__ import annotations

import numpy as np

from ..sim.robot import LANE_INDEX, OUT_OF_COMMS


class FaultInjector:
    def __init__(self, scenario) -> None:
        self.cfg = scenario.fault
        self.fired_at: float | None = None
        self.victim_robot: str | None = None

    def step(self, world, ex, emit=None) -> None:
        if self.fired_at is not None or not self.cfg.enabled:
            return
        ready = world.t >= self.cfg.min_t and world.victims_rescued >= self.cfg.min_rescued
        if not (ready or world.t >= self.cfg.fallback_t):
            return
        i = self._pick(world, ex)
        if i < 0:
            return
        self.fired_at = world.t
        self.victim_robot = world.robot_ids[i]
        if emit:
            emit("robot_destroyed",
                 f"structural collapse destroys {world.robot_ids[i]}",
                 robot=world.robot_ids[i], pos=tuple(world.pos[i]))
        world._kill(i, "structural collapse")

    def _pick(self, world, ex) -> int:
        """Prefer a loaded carrier: the casualty is dropped and must be rescued again,
        which makes the recovery visible rather than merely logged."""
        alive = world.status <= OUT_OF_COMMS
        loaded = np.nonzero(alive & (world.carrying >= 0))[0]
        if len(loaded):
            return int(loaded[0])
        busy = np.array([a is not None for a in ex.assignment]) & alive
        cand = np.nonzero(busy & (world.actuator != LANE_INDEX["antenna"]))[0]
        if len(cand):
            return int(cand[len(cand) // 2])
        return int(np.nonzero(alive)[0][0]) if alive.any() else -1

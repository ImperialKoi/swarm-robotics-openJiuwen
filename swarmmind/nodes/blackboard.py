"""The shared stigmergy board: /swarm/state.

Aggregates everything the swarm knows -- robot self-reports, sector coverage, observed
hazard, and the contact set -- into one message. Robots write here; nothing goes
robot-to-robot.

**It publishes knowledge, never truth.** Sector hazard is `hazard_known` (cells actually
observed), and casualties appear only once perception has resolved a contact onto one.
The ground-truth feed is a separate topic that no node in this package may read.

Payloads are plain dicts, not pydantic instances: at 512 robots and 10 Hz, constructing
5,000 models a second is real cost for no benefit. ``contracts/schemas.py`` remains the
specification, and tests/test_blackboard.py validates the emitted dicts against it, so
the contract is still enforced -- just not on the hot path.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..contracts import topics
from ..contracts.schemas import LANE_LABEL
from ..contracts.version import SCHEMA_VERSION
from ..perception.tracker import STATE_NAME
from ..sim import grid
from ..sim.robot import LANES, STATUS_NAME

PRIORITY_NAME = {0: "high", 1: "normal", 2: "low"}


class BlackboardNode:
    def __init__(self, bus=None) -> None:
        self.bus = bus
        self.last: dict[str, Any] | None = None

    def publish(self, world, ex, tracker) -> dict[str, Any]:
        state = self.build(world, ex, tracker)
        self.last = state
        if self.bus is not None:
            self.bus.publish(topics.SWARM_STATE, state)
        return state

    def build(self, world, ex, tracker) -> dict[str, Any]:
        robots = []
        for i in range(world.n):
            a = ex.assignment[i]
            carrying = int(world.carrying[i])
            robots.append({
                "schema": SCHEMA_VERSION,
                "id": world.robot_ids[i],
                "type": LANE_LABEL[LANES[int(world.actuator[i])]],
                "actuator": LANES[int(world.actuator[i])],
                "pos": (round(float(world.pos[i, 0]), 2), round(float(world.pos[i, 1]), 2)),
                "heading": round(float(world.theta[i]), 3),
                "battery": round(float(world.battery[i]), 3),
                "status": STATUS_NAME[int(world.status[i])],
                "current_task": a.task_id if a else None,
                "carrying": world.victims[carrying].id if carrying >= 0 else None,
                "last_action_reason": ex.reason[i],
            })

        found_per_sector = self._found_by_sector(world)
        sectors = []
        for k, sid in enumerate(world.sector_ids):
            sectors.append({
                "schema": SCHEMA_VERSION,
                "id": sid,
                "explored_pct": round(float(world.sector_explored_pct[k]), 4),
                # KNOWN hazard only -- never the ground-truth disc.
                "hazard_level": round(float(world.sector_hazard_known[k]), 4),
                "victims_found": int(found_per_sector[k]),
                "priority": PRIORITY_NAME[int(world.sector_priority[k])],
                "abandoned": bool(world.sector_abandoned[k]),
            })

        reports = [{
            "schema": SCHEMA_VERSION, "id": r.id,
            "pos": (round(float(r.pos[0]), 2), round(float(r.pos[1]), 2)),
            "state": STATE_NAME[r.state], "conf": round(float(r.conf), 3),
            "n_obs": r.n_obs, "views": len(r.views),
        } for r in tracker.reports if r.state != 3] if tracker else []

        alive = world.status <= 1
        return {
            "schema": SCHEMA_VERSION,
            "tick": world.tick,
            "sim_time": round(world.t, 2),
            "robots": robots,
            "sectors": sectors,
            "victims_found_total": world.victims_found,
            "victims_rescued_total": world.victims_rescued,
            "victims_total": len(world.victims),
            "robots_active": int(alive.sum()),
            "robots_lost": world.robots_lost,
            "comms_component_size": int(world.in_comms.sum()),
            "reports": reports,
        }

    def _found_by_sector(self, world) -> np.ndarray:
        """Found casualties per sector, in one pass.

        This was `_found_in(world, sector_id)` called once **per sector**, each call
        looping over every casualty and converting its position to a cell -- 48 x 110 =
        5,280 `world_to_cell` calls per publish at 10 Hz, to compute something that is
        O(casualties). Profiled at t=240 it was **15.8 s of a 27.4 s window, 58% of the
        entire tick budget**, and it is why the demo scenario runs at 0.97x realtime:
        `DemoSim` only sleeps when it is ahead, so below 1.0x it can never catch up.

        The result is identical -- same counts, same order -- so this is a cost change
        only, and the seed-42 hash must not move.
        """
        n_sec = len(world.sector_ids)
        found = [v for v in world.victims if v.state != 0]
        if not found:
            return np.zeros(n_sec, dtype=np.int64)
        pos = np.array([v.pos for v in found], dtype=np.float64)
        ix, iy = grid.world_to_cell(pos[:, 0], pos[:, 1], world.cell, world.shape)
        return np.bincount(world.sector_of_cell[iy, ix].ravel(), minlength=n_sec)

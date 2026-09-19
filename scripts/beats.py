#!/usr/bin/env python
"""When does each demo beat actually happen, on the build that ships?

    uv run python scripts/beats.py

`docs/RUNBOOK.md` is written from this. A beat sheet taken from the plan rather than from
the build drifts silently: PLAN §4 said the hazard ignites at 2:30 when `demo.yaml` had
said 1:30 since D5d, and the scripted failure was firing on its wall-clock fallback rather
than the state trigger the plan lists as a locked decision (M-46). Re-run this whenever
`demo.yaml` changes.
"""
from __future__ import annotations

import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
import collections  # noqa: E402

from swarmmind.contracts import topics  # noqa: E402
from swarmmind.mission import Mission  # noqa: E402
from swarmmind.sim.scenario import Scenario  # noqa: E402

WATCH = ("hazard_ignited", "robot_destroyed", "task_orphaned", "task_awarded",
         "victim_rescued", "victim_cleared", "directive_issued", "directive_rejected",
         "sector_abandoned", "robot_failed")

if __name__ == "__main__":
    m = Mission(Scenario.load("demo"), 42, hivemind=True, scripted_hivemind=True)
    first: dict[str, float] = {}
    counts: collections.Counter = collections.Counter()
    fault = []

    def on_event(msg):
        k = msg.get("kind", "?")
        counts[k] += 1
        first.setdefault(k, float(msg.get("time", msg.get("t", 0.0))))
        if k in ("robot_destroyed", "robot_failed"):
            fault.append(float(msg.get("time", msg.get("t", 0.0))))

    m.bus.subscribe(topics.SWARM_EVENTS, on_event)
    while m.world.t < 420.0:
        m.tick()
    fault_t = fault[0] if fault else None

    def mmss(t):
        return f"{int(t) // 60}:{int(t) % 60:02d}"

    print("  first occurrence of each beat (demo, seed 42, Tier 3 scripted)\n")
    for k in sorted(first, key=lambda k: first[k]):
        print(f"    {mmss(first[k]):>6}  ({first[k]:6.1f}s)  {k:<22} x{counts[k]}")
    h = m.world.scn.hazard
    print(f"\n  hazard: ignites {h.ignite_t:.0f}s, peaks {h.peak_t:.0f}s, "
          f"recedes at {h.decay_rate} m/s")
    f = m.world.scn.fault
    print(f"  scripted fault fired at {fault_t}s  (state trigger t>={f.min_t:.0f} & "
          f"rescued>={f.min_rescued}, fallback t={f.fallback_t:.0f})")
    print(f"  final: rescued {m.world.victims_rescued}, found {m.world.victims_found}")

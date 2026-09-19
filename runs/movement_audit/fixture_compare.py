import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('source')
p.add_argument('out')
a = p.parse_args()
sys.path.insert(0, a.source)
import numpy as np
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario
from swarmmind.sim import grid

rows = []
for scripted in (False, True):
    m = Mission(Scenario.load('test'), 42, hivemind=scripted, scripted_hivemind=scripted)
    w = m.world
    previous = w.pos.copy()
    stopped = loaded_stopped = wet_stopped = 0
    start = time.perf_counter()
    while not w.done:
        m.tick()
        if w.tick % 20 == 0:
            # Read the goals used by the last control tick. Calling goals() here
            # would refresh the cache ahead of perception/assignment bookkeeping.
            xy, ids = m.executor._goal_xy, m.executor._goal_id
            has = ids >= 0
            at = np.zeros(w.n, dtype=bool)
            if xy:
                at[has] = np.linalg.norm(w.pos[has] - np.asarray(xy)[ids[has]], axis=1) <= 1.5
            still = (np.linalg.norm(w.pos - previous, axis=1) < .1) & (w.status <= 1)
            x, y = grid.world_to_cell(*w.pos.T, w.cell, w.shape)
            stopped += int((still & has & ~at).sum())
            loaded_stopped += int((still & (w.carrying >= 0)).sum())
            wet_stopped += int((still & (w.water[y,x] > 0)).sum())
            previous = w.pos.copy()
    card = m.scorecard(time.perf_counter()-start)
    rows.append({'tier3': 'scripted' if scripted else 'off', 'scorecard': dataclasses.asdict(card),
                 'hash': card.hash(), 'stationary_away_from_goal_robot_seconds': stopped,
                 'loaded_stationary_robot_seconds': loaded_stopped,
                 'water_stationary_robot_seconds': wet_stopped,
                 'recovery_builds': getattr(getattr(m.reflex, 'recovery_fields', None), 'builds', 0)})
    m.close()
    print(json.dumps(rows[-1]), flush=True)
Path(a.out).parent.mkdir(parents=True, exist_ok=True)
Path(a.out).write_text(json.dumps(rows, indent=2)+'\n')

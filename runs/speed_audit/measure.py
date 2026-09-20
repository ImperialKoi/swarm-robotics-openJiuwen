"""Read-only rescue timing comparison on the small fixture, never the demo."""

import dataclasses
import json
import time
from pathlib import Path

from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario


def main():
    rows = []
    for speed in (1.0, Scenario.load("demo").robot_speed_multiplier):
        scn = dataclasses.replace(Scenario.load("test"), robot_speed_multiplier=speed)
        mission = Mission(scn, 42, hivemind=True, scripted_hivemind=True)
        samples = []
        start = time.perf_counter()
        try:
            while not mission.world.done:
                mission.tick()
                w = mission.world
                if w.tick % int(60 * scn.rates.tick_hz) == 0:
                    samples.append({"t": w.t, "rescued": w.victims_rescued,
                                    "found": w.victims_found, "lost": w.robots_lost})
            card = mission.scorecard(time.perf_counter() - start)
            row = {"scenario": scn.name, "seed": 42, "tier3": "scripted",
                   "speed_multiplier": speed, "samples": samples,
                   "scorecard": dataclasses.asdict(card), "hash": card.hash(),
                   "rescue_times": sorted(v.rescued_t for v in w.victims if v.rescued_t >= 0)}
            rows.append(row)
            print(json.dumps(row), flush=True)
        finally:
            mission.close()
    Path(__file__).with_name("timing.json").write_text(json.dumps(rows, indent=2) + "\n")


if __name__ == "__main__":
    main()

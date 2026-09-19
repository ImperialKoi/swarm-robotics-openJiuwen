#!/usr/bin/env python3
"""Full-scale end-to-end test: the whole demo map, the whole swarm, every tier running.

Not a benchmark of one component -- the point is that all of it runs *together* at the
size it will be on demo day: 768 robots on 480x320 m of terrain, real perception through
occluded egocentric cameras, the market auction, and Tier 3 with whatever rung of the
ladder answers. Anything that only works at `test.yaml` scale fails here.

Run it on an idle machine. MEASUREMENTS.md M-24: a real-time factor measured while
something else holds the cores is off by 2.4x and will send you optimising a problem you
do not have.
"""

from __future__ import annotations

import argparse
import time
from collections import Counter
from pathlib import Path

import numpy as np

from swarmmind.mission import Mission
from swarmmind.sim.robot import CHASSIS, LANES, OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario
from swarmmind.viz import png
from swarmmind.viz.render3d import Renderer3D


def describe(scn: Scenario, m: Mission) -> None:
    w = m.world
    print(f"  map        {scn.map.width_m:.0f}x{scn.map.height_m:.0f} m  "
          f"{w.shape[1]}x{w.shape[0]} cells  "
          f"{int(w.passable.sum()):,} passable ({w.passable.mean():.0%})")
    print(f"  sectors    {scn.map.sector_rows}x{scn.map.sector_cols} = "
          f"{len(w.sector_ids)}")
    print(f"  swarm      {w.n} robots  "
          f"lanes {dict(Counter(LANES[i] for i in w.actuator))}")
    print(f"             chassis {dict(Counter(CHASSIS[i] for i in w.chassis))}")
    print(f"  casualties {len(w.victims)}  "
          f"({sum(1 for v in w.victims if v.buried)} buried)")
    print(f"  Tier 3     {[p.name for p in m.hivemind.providers] if m.hivemind else 'off'}")


def report(m: Mission, card, wall: float, t_ticks: list[float]) -> None:
    w = m.world
    alive = w.status <= OUT_OF_COMMS
    print(card.render())
    ticks = np.array(t_ticks) * 1000.0
    print(f"  tick cost  mean {ticks.mean():.2f} ms  p99 {np.percentile(ticks, 99):.2f} ms"
          f"  max {ticks.max():.1f} ms   (50 ms budget at 20 Hz)")
    print(f"  survivors  {int(alive.sum())}/{w.n}   in comms {int(w.in_comms.sum())}"
          f"   held by a relay {int(w.comms_via_relay.sum())}")
    for lane_i, lane in enumerate(LANES):
        sel = w.actuator == lane_i
        print(f"    {lane:<8} {int((sel & alive).sum()):>4}/{int(sel.sum()):<4} alive"
              f"   battery {w.battery[sel & alive].mean() if (sel & alive).any() else 0:.2f}"
              f"   in comms {int(w.in_comms[sel].sum()):>4}")
    for c_i, chassis in enumerate(CHASSIS):
        sel = w.chassis == c_i
        print(f"    {chassis:<8} {int((sel & alive).sum()):>4}/{int(sel.sum()):<4} alive")
    tr = m.tracker
    print(f"  perception {len(tr.reports)} reports   "
          f"{len(tr.open_reports())} awaiting   "
          f"{len({r.victim for r in tr.resolved_victims()})} confirmed real   "
          f"{sum(1 for r in tr.reports if r.state == 3)} dismissed")
    if m.hivemind:
        print(f"  hivemind   {m.hivemind.stats}")
        lat = m.hivemind._latency_ms
        if lat:
            print(f"             latency mean {np.mean(lat):.0f} ms  max {max(lat)} ms  "
                  f"n={len(lat)}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="demo")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--max-time", type=float, default=None)
    ap.add_argument("--hivemind", default="ladder",
                    choices=("ladder", "scripted", "off"))
    ap.add_argument("--out", type=Path, default=Path("runs/fullscale"))
    ap.add_argument("--shots", type=int, default=3, help="POV frames to render")
    args = ap.parse_args()

    scn = Scenario.load(args.scenario)
    m = Mission(scn, args.seed,
                hivemind=args.hivemind != "off",
                scripted_hivemind=args.hivemind == "scripted")
    describe(scn, m)

    limit = args.max_time if args.max_time is not None else scn.mission_duration_s
    print(f"\n  running {limit:.0f} s of simulation...", flush=True)
    w = m.world
    costs: list[float] = []
    t0 = time.perf_counter()
    while not w.done and w.t < limit:
        a = time.perf_counter()
        m.tick()
        costs.append(time.perf_counter() - a)
    wall = time.perf_counter() - t0
    import dataclasses
    card = dataclasses.replace(
        w.scorecard(), wall_seconds=round(wall, 3),
        rtf=round(w.t / wall, 2) if wall else 0.0,
        directives_issued=m.hivemind.stats["issued"] if m.hivemind else 0,
        directives_rejected=m.hivemind.stats["rejected"] if m.hivemind else 0,
    )
    report(m, card, wall, costs)

    args.out.mkdir(parents=True, exist_ok=True)
    r = Renderer3D(w, 1400, 860)
    png.write(str(args.out / "orbit.png"), r.render(w, r.orbit(w)))
    png.write(str(args.out / "orbit_god.png"), r.render(w, r.orbit(w), fog=False))
    png.write(str(args.out / "orbit_sectors.png"),
              r.render(w, r.orbit(w), fog=False, sectors=True))

    # POV from robots standing in open ground -- a camera pressed against rubble is
    # correct and shows nothing.
    alive = np.nonzero(w.status <= OUT_OF_COMMS)[0]
    openness = []
    for i in alive:
        ix = int(np.clip(w.pos[i, 0] / w.cell, 1, w.shape[1] - 2))
        iy = int(np.clip(w.pos[i, 1] / w.cell, 1, w.shape[0] - 2))
        openness.append((w.passable[max(0, iy - 6):iy + 7,
                                    max(0, ix - 6):ix + 7].mean(), int(i)))
    openness.sort(reverse=True)
    for n, (_, i) in enumerate(openness[: args.shots]):
        png.write(str(args.out / f"pov{n}.png"), r.render(w, r.pov(w, i), hide=i))
        a = m.executor.assignment[i]
        print(f"  pov{n}: {w.robot_ids[i]}  {LANES[int(w.actuator[i])]}/"
              f"{CHASSIS[int(w.chassis[i])]}  task={a.kind if a else 'idle'}")
    print(f"\n  frames -> {args.out}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

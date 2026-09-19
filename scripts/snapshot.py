#!/usr/bin/env python
"""Dump mission frames as PNGs. Visual debugging without Godot.

    uv run python scripts/snapshot.py --scenario test --frames 40 --out runs/shots

Produces, per sampled tick, a side-by-side swarm view | god view, plus one camera
contact sheet showing what a sample of robots is actually looking at.
"""

from __future__ import annotations

import argparse
import json
import pathlib

import numpy as np

from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario
from swarmmind.viz import png
from swarmmind.viz.render import MissionRenderer, contact_sheet


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="test")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--until", type=float, default=420.0)
    ap.add_argument("--zoom", type=int, default=2)
    ap.add_argument("--cameras", type=int, default=16)
    ap.add_argument("--out", default="runs/shots")
    args = ap.parse_args()

    scn = Scenario.load(args.scenario)
    m = Mission(scn, args.seed)
    w = m.world
    rend = MissionRenderer(w)
    # Use the mission's own rig, detector and tracker so the frames show what the swarm
    # actually believes -- phantoms included -- not a separate parallel perception pass.
    rig = m.rig

    every = max(1, int(args.until / args.frames / scn.dt))
    n = 0
    manifest: list[dict] = []
    while not w.done and w.t < args.until:
        m.tick()
        if w.tick % every:
            continue
        img = rend.raster.render(w)
        idx = np.arange(min(args.cameras, w.n))
        frames, _ = rig.capture(w, img, idx)
        reports = m.tracker.believed_positions()

        swarm, god = rend.views(w, reports)
        gap = np.full((swarm.shape[0], 3, 3), 60, np.uint8)
        side = png.upscale(np.hstack([swarm, gap, god]), args.zoom)
        png.write(f"{args.out}/t{n:03d}.png", side)
        if n == args.frames // 2:
            png.write(f"{args.out}/cameras.png", png.upscale(contact_sheet(frames), 2))
        manifest.append({
            "frame": n, "t": round(w.t, 1),
            "rescued": w.victims_rescued, "found": w.victims_found,
            "explored": round(float(w.explored[w.passable].mean()), 4),
            "active": int((w.status <= 1).sum()), "lost": w.robots_lost,
            "in_comms": int(w.in_comms.sum()), "reports": len(reports),
            "confirmed": m.tracker.stats["confirmed"],
            "resolved": m.tracker.stats["resolved"],
            "dismissed": m.tracker.stats["dismissed"],
            "hazard_r": round(float(w.hazard.radius), 1),
        })
        n += 1

    meta = {"scenario": args.scenario, "seed": args.seed, "zoom": args.zoom,
            "robots": w.n, "victims": len(w.victims), "cameras": int(min(args.cameras, w.n)),
            "cell_m": scn.map.cell, "map_m": [scn.map.width_m, scn.map.height_m],
            "frames": manifest}
    pathlib.Path(f"{args.out}/manifest.json").write_text(json.dumps(meta, indent=1))
    print(f"{n} frames -> {args.out}/  (left: swarm view, right: god view)")
    print(f"  final: {w.victims_rescued}/{len(w.victims)} rescued, "
          f"{w.victims_found} found, {w.explored[w.passable].mean():.1%} explored")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

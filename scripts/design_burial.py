#!/usr/bin/env python
"""Export casualty rubble and preview buried/cleared states without running a mission."""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CLEARED, World
from swarmmind.viz import png
from swarmmind.viz.burial import ASSETS, exported_rubble
from swarmmind.viz.render3d import Camera3D, Renderer3D


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("runs/3d/landscape"))
    args = parser.parse_args()
    world = World(Scenario.load("test"), 42)
    victim = next(v for v in world.victims if v.buried)
    renderer = Renderer3D(world, 900, 640)
    x, y = victim.pos
    z = float(renderer.surface.height_at_world(x, y))
    yaw = (x*1.7+y*3.1) % (2*np.pi)
    offset = np.array([np.cos(yaw), np.sin(yaw)])*6 - np.array([-np.sin(yaw), np.cos(yaw)])*5
    eye = np.r_[victim.pos+offset, z+5]
    eye[2] = max(eye[2], renderer.surface.height_at_world(*eye[:2])+2)
    camera = Camera3D(eye, np.array([x, y, z+.35]), fov_deg=48)
    args.out.mkdir(parents=True, exist_ok=True)
    png.write(args.out / "casualty_buried.png",
              renderer.render(world, camera, fog=False, robots=False, victims=True))
    # An isolated illustration of the actual CLEARED state; no simulator is advanced.
    cleared = copy.copy(world)
    cleared.victims = copy.deepcopy(world.victims)
    for casualty in cleared.victims:
        if casualty.id == victim.id:
            casualty.state = CLEARED
            casualty.debris_remaining = 0
    png.write(args.out / "casualty_cleared.png",
              renderer.render(cleared, camera, fog=False, robots=False, victims=True))
    (ASSETS / "burial.json").write_text(json.dumps(exported_rubble(), separators=(",", ":")) + "\n",
                                        encoding="utf-8")
    print(f"Burial geometry exported; two state previews -> {args.out}")


if __name__ == "__main__":
    main()

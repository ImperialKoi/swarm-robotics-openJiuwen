#!/usr/bin/env python
"""Small, isolated visual fixture for normal/exposed/buried thermal comparisons."""

from pathlib import Path

import numpy as np

from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World
from swarmmind.viz import png
from swarmmind.viz.render3d import Camera3D, Renderer3D


def main():
    world = World(Scenario.load("test"), 42)
    world.occ[:] = 0
    world.height[:] = 0
    world.water[:] = 0
    world.explored[:] = True
    victim = world.victims[0]
    world.victims = [victim]
    victim.pos[:] = [14, 14]
    victim.buried = True
    world.pos[0] = [7, 14]
    world.theta[0] = 0
    camera = Camera3D(np.array([7., 14., 2.8]), np.array([14., 14., .5]), fov_deg=48)
    renderer = Renderer3D(world, 640, 420)
    output = Path("runs/3d/thermal")
    output.mkdir(parents=True, exist_ok=True)
    frames = []
    for name, state, thermal in (("normal", 2, None), ("exposed", 2, 0), ("buried", 0, 0)):
        victim.state = state
        frame = renderer.render(world, camera, fog=False, robots=False,
                                victims=True, thermal_unit=thermal)
        png.write(output / f"{name}.png", frame)
        frames.append(frame)
    png.write(output / "comparison.png", np.concatenate(frames, axis=1))
    print(f"Normal / exposed thermal / buried thermal -> {output}/comparison.png")


if __name__ == "__main__":
    main()

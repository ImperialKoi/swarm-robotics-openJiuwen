#!/usr/bin/env python
"""Offline 3D frames. The verification tool for anything that will be rendered in Godot.

Godot and a browser both need a screen, and this environment has neither. Rendering the
same scene here turns "the 3D looks wrong" into an image that can actually be inspected.

    uv run python scripts/snapshot3d.py --scenario test --at 150 --pov 3
    uv run python scripts/snapshot3d.py --scenario demo --at 120 --fx --pov 0

`--fx` runs swarmmind/viz/particles.py alongside the mission and draws the field, which
is how the dashboard's dig dust was checked before it was written in GDScript. It needs a
scenario where somebody is actually digging: on `demo` that starts around t=82, and on
the `test` fixture it never happens at all.
"""

from __future__ import annotations

import argparse

import numpy as np

from swarmmind.mission import Mission
from swarmmind.sim.robot import OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario
from swarmmind.viz import png
from swarmmind.viz.particles import ParticleField
from swarmmind.viz.render3d import Renderer3D


def _dig_cam(world, x: float, y: float, dist: float = 16.0):
    """Eye level, close in, looking at an excavation."""
    from swarmmind.viz.render3d import Camera3D

    ix = int(np.clip(x / world.cell, 0, world.shape[1] - 1))
    iy = int(np.clip(y / world.cell, 0, world.shape[0] - 1))
    ground = float(world.height[iy, ix])
    a = np.deg2rad(35.0)
    pos = np.array([x - np.cos(a) * dist, y - np.sin(a) * dist, ground + 6.0], np.float32)
    return Camera3D(pos=pos, target=np.array([x, y, ground + 1.2], np.float32), fov_deg=52.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="test")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--at", type=float, default=150.0, help="sim seconds before rendering")
    ap.add_argument("--pov", type=int, default=3, help="how many robot POVs to render")
    ap.add_argument("--chase", type=int, default=2,
                    help="how many third-person follow frames to render")
    ap.add_argument("--width", type=int, default=1000)
    ap.add_argument("--height", type=int, default=620)
    ap.add_argument("--out", default="runs/3d")
    ap.add_argument("--fx", action="store_true",
                    help="run the display particle field and draw it")
    args = ap.parse_args()

    m = Mission(Scenario.load(args.scenario), args.seed)
    w = m.world
    fx = ParticleField(args.seed) if args.fx else None
    while w.t < args.at and not w.done:
        m.tick()
        if fx is not None:
            # Stepped in lockstep with the simulator so the field is in the state it
            # would be at this instant, rather than a burst fired at the last tick.
            fx.update(w, w.dt)
    r = Renderer3D(w, args.width, args.height)

    def render(cam, **kwargs):
        return r.render(w, cam, activity=m.executor.activity, **kwargs)

    png.write(f"{args.out}/orbit.png", render(r.orbit(w), fx=fx))
    png.write(f"{args.out}/orbit_god.png", render(r.orbit(w), fog=False, fx=fx))
    # Tier 3's effect has to be visible or the strategic layer is just a text feed.
    # Verified here first; the GDScript overlay is a port of this frame.
    png.write(f"{args.out}/orbit_sectors.png",
              render(r.orbit(w), fog=False, sectors=True))
    n_hi = int((w.sector_priority == 0).sum())
    n_lo = int((w.sector_priority == 2).sum())
    print(f"  sector overlay: {int(w.sector_abandoned.sum())} abandoned, "
          f"{n_hi} high, {n_lo} low")

    # Prefer robots that are out in the open: a POV pressed against rubble is correct
    # but shows nothing, and the point of these frames is to check framing.
    alive = np.nonzero(w.status <= OUT_OF_COMMS)[0]
    openness = []
    for i in alive:
        ix = int(np.clip(w.pos[i, 0] / w.cell, 1, w.shape[1] - 2))
        iy = int(np.clip(w.pos[i, 1] / w.cell, 1, w.shape[0] - 2))
        win = w.passable[max(0, iy - 6):iy + 7, max(0, ix - 6):ix + 7]
        openness.append((win.mean(), int(i)))
    openness.sort(reverse=True)

    for n, (_, i) in enumerate(openness[: args.pov]):
        a = m.executor.assignment[i]
        png.write(f"{args.out}/pov{n}.png", render(r.pov(w, i), hide=i, fx=fx))
        print(f"  pov{n}: {w.robot_ids[i]}  task={a.kind if a else 'idle'}")

    # The third-person follow, which unlike the POV must show the unit itself -- so
    # nothing is hidden here. These are the frames that say whether the default framing
    # is close enough to read the chassis and wide enough to read what it is walking
    # into; the dashboard's drag and wheel move `yaw_off_deg` and `dist`, so one frame
    # is taken at each end of that travel rather than trusting the default alone.
    for n, (_, i) in enumerate(openness[: args.chase]):
        png.write(f"{args.out}/chase{n}.png", render(r.chase(w, i), fx=fx))
        print(f"  chase{n}: {w.robot_ids[i]}  dist={r.CHASE_DIST:.1f}m")
    if args.chase > 0 and openness:
        i = openness[0][1]
        png.write(f"{args.out}/chase_turned.png",
                  render(r.chase(w, i, yaw_off_deg=75.0, elevation_deg=28.0), fx=fx))
        png.write(f"{args.out}/chase_near.png", render(r.chase(w, i, dist=2.2), fx=fx))
        png.write(f"{args.out}/chase_far.png", render(r.chase(w, i, dist=16.0), fx=fx))

    # A plume seen from 340 m is a smudge. The frame that actually verifies the dig
    # effect is one taken from beside the hole, which is also the shot the robot-POV
    # camera gives a judge.
    if fx is not None:
        print(f"  particles: {len(fx)} live, {len(w.digging)} dig site(s)")
        for n, (dx, dy, nd) in enumerate(w.digging[:2]):
            png.write(f"{args.out}/dig{n}.png", render(_dig_cam(w, dx, dy), fx=fx))
            print(f"  dig{n}: ({dx:.0f}, {dy:.0f})  {nd} digger(s)")

    print(f"t={w.t:.0f}s  explored {w.explored[w.passable].mean():.0%}  "
          f"rescued {w.victims_rescued}/{len(w.victims)}  -> {args.out}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

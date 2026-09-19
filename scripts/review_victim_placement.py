#!/usr/bin/env python
"""Construct-only placement audit and actual 3D site previews; no mission is run.

    uv run python scripts/review_victim_placement.py

Checks the four configured demo maps. Optional --before accepts a JSON mapping
seed -> [{"pos": [x, y], "buried": bool}, ...] captured before a placement change.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from swarmmind.sim import grid
from swarmmind.sim.placement import cover_masks
from swarmmind.sim.robot import CHASSIS_INDEX
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World
from swarmmind.viz import png
from swarmmind.viz.render3d import Camera3D, Renderer3D


def metrics(world, sites):
    points = np.array([v["pos"] for v in sites])
    buried = np.array([v["buried"] for v in sites])
    ix, iy = grid.world_to_cell(points[:, 0], points[:, 1], world.cell, world.shape)
    wall, rubble = cover_masks(world.occ, world.cell, world.scn.victims.cover_radius_m)
    # Ground-level sightlines from 16 directions at 6 m. Discard viewpoints outside
    # the map or on ground a legged robot cannot stand on; this is geometry, not a
    # detector-recall or mission-performance measurement.
    blocked = usable = 0
    for angle in np.arange(16) * (2 * np.pi / 16):
        observers = points + 6 * np.array([np.cos(angle), np.sin(angle)])
        ox, oy = grid.world_to_cell(observers[:, 0], observers[:, 1],
                                   world.cell, world.shape)
        valid = ((observers >= 0).all(axis=1)
                 & (observers < [world.scn.map.width_m, world.scn.map.height_m]).all(axis=1)
                 & world.chassis_passable[CHASSIS_INDEX["legged"], oy, ox])
        visible = grid.visible_cells(world.occ, observers[:, 0], observers[:, 1],
                                     ix, iy, world.cell)
        blocked += int((valid & ~visible).sum())
        usable += int(valid.sum())
    return {
        "total": len(sites), "buried": int(buried.sum()),
        "covered": int((wall | rubble)[iy, ix].sum()),
        "buried_on_rubble": int((world.occ[iy[buried], ix[buried]] == grid.RUBBLE).sum()),
        "surface_near_wall": int(wall[iy[~buried], ix[~buried]].sum()),
        "wet": int((world.water[iy, ix] > 0).sum()),
        "blocked_sightlines": blocked, "usable_sightlines": usable,
    }


def map_image(world, sites):
    image = np.array([[185, 190, 169], [70, 71, 67], [136, 113, 83]], dtype=np.uint8)[world.occ]
    image[world.water > 0] = [74, 120, 149]
    image = png.upscale(image, 3)
    for site in sites:
        x, y = (np.array(site["pos"]) / world.cell * 3).astype(int)
        # Gold = buried; red = surface. Black outlines keep them legible on rubble.
        image[max(0, y-3):y+4, max(0, x-3):x+4] = 15
        image[max(0, y-2):y+3, max(0, x-2):x+3] = (
            [255, 197, 60] if site["buried"] else [230, 62, 56])
    return image


def previews(world, out):
    renderer = Renderer3D(world, 900, 620)
    wall, _ = cover_masks(world.occ, world.cell, world.scn.victims.cover_radius_m)
    buried = next(v for v in world.victims if v.buried)
    surface = next(v for v in world.victims if not v.buried and wall[
        int(v.pos[1] / world.cell), int(v.pos[0] / world.cell)])
    for name, victim in (("buried", buried), ("sheltered", surface)):
        x, y = victim.pos
        z = float(renderer.surface.height_at_world(x, y))
        # Review from the most open approach so the actual worksite can be inspected.
        angles = np.arange(16) * (2 * np.pi / 16)
        eyes = victim.pos + 9 * np.column_stack([np.cos(angles), np.sin(angles)])
        heights = np.array([renderer.surface.height_at_world(*p) for p in eyes])
        camera_pos = eyes[int(np.argmin(heights))]
        eye_z = max(z + 6, float(heights.min()) + 3)
        camera = Camera3D(np.r_[camera_pos, eye_z], np.array([x, y, z + .4]), fov_deg=52)
        png.write(out / f"{name}.png", renderer.render(
            world, camera, fog=False, robots=False, victims=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("runs/3d/victim_placement"))
    parser.add_argument("--before", type=Path)
    args = parser.parse_args()
    before = json.loads(args.before.read_text()) if args.before else {}
    scenario = Scenario.load("demo")
    report = {}
    for seed in scenario.demo_seeds:
        start = perf_counter()
        world = World(scenario, seed)
        build_seconds = perf_counter() - start
        sites = [{"pos": v.pos.tolist(), "buried": v.buried} for v in world.victims]
        row = {"after": metrics(world, sites), "build_seconds": round(build_seconds, 3),
               "separation_m": world.victim_separation}
        if str(seed) in before:
            row["before"] = metrics(world, before[str(seed)])
        report[str(seed)] = row
        if seed == scenario.demo_seeds[0]:
            png.write(args.out / "placement.png", map_image(world, sites))
            if str(seed) in before:
                comparison = np.concatenate([map_image(world, before[str(seed)]),
                                             map_image(world, sites)], axis=1)
                png.write(args.out / "before_after.png", comparison)
            previews(world, args.out)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "review.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"Construct-only review and site previews -> {args.out}")


if __name__ == "__main__":
    main()

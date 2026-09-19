#!/usr/bin/env python
"""Review landscape geometry without running a mission or advancing its RNG streams.

    uv run python scripts/design_terrain.py
    uv run python scripts/design_terrain.py --fixture /tmp/swarmmind-landscape-fixture.json

The four cameras inspect actual rivers, high ground and occupied settlement footprints.
They keep the renderer's adaptive orbit detail; no full-map stride-one build is needed.
The optional bridge fixture is for native Godot inspection of this same initial world.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from swarmmind.nodes.bridge import BridgeNode
from swarmmind.rng import STREAMS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World
from swarmmind.viz import png
from swarmmind.viz.prop_stream import (
    MERGE_METRES,
    TILE_METRES,
    landscape_kind,
    merged_rectangles,
)
from swarmmind.viz.render3d import Camera3D, Renderer3D


def _mean(values: np.ndarray, radius: int) -> np.ndarray:
    """Local mean in linear memory, with edge padding and no optional dependencies."""
    side = radius * 2 + 1
    padded = np.pad(values.astype(float), radius, mode="edge")
    integral = np.pad(padded, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    return (integral[side:, side:] - integral[:-side, side:]
            - integral[side:, :-side] + integral[:-side, :-side]) / (side * side)


def _target(score: np.ndarray, cell: float) -> np.ndarray:
    y, x = np.unravel_index(np.argmax(score), score.shape)
    return np.array([(x + .5) * cell, (y + .5) * cell])


def review_cameras(world: World, renderer: Renderer3D) -> dict[str, Camera3D]:
    """Stable compositions from display landform and the actual occupied/wet masks."""
    surface = renderer.surface
    elevation = surface.corners[:-1, :-1]
    yy, xx = np.indices(world.shape)
    edge = np.minimum.reduce([xx, yy, world.shape[1] - 1 - xx, world.shape[0] - 1 - yy])
    interior = np.clip(edge * world.cell / 24, 0, 1)
    centre = np.hypot((xx / world.shape[1] - .5), (yy / world.shape[0] - .5))
    water = _mean(world.water > .45, max(1, round(14 / world.cell)))
    river = _target(water * interior * (1 - centre * .35), world.cell)
    mountain = _target(elevation * np.clip(interior, .5, 1), world.cell)
    slope = np.hypot(*np.gradient(elevation, world.cell))
    walls = _mean(world.occ == 1, max(1, round(10 / world.cell)))
    settlement_score = (walls * interior * np.exp(-slope * 3)
                        * (1 - centre * .3) * (world.water < .02))
    settlement = _target(settlement_score, world.cell)
    # Select an actual house-capable merged footprint, not merely a dense rock patch.
    tile = max(1, int(TILE_METRES / world.cell))
    span = max(1, int(MERGE_METRES / world.cell))
    best = -1.0
    occupancy = np.where(world.water > .02, 0, world.occ)
    for y0 in range(0, world.shape[0], tile):
        for x0 in range(0, world.shape[1], tile):
            for x, y, width, height in merged_rectangles(
                    occupancy[y0:y0+tile, x0:x0+tile], span):
                if min(width, height) * world.cell < 3:
                    continue
                gx, gy = x0 + x, y0 + y
                point = np.array([gx + width*.5, gy + height*.5]) * world.cell
                if landscape_kind(*point) != "ruin":
                    continue
                floor = surface.height_at_world(gx * world.cell, gy * world.cell)
                dx = (surface.height_at_world((gx+width)*world.cell, gy*world.cell)
                      - floor) / (width*world.cell)
                dy = (surface.height_at_world(gx*world.cell, (gy+height)*world.cell)
                      - floor) / (height*world.cell)
                if np.hypot(dx, dy) > .4:
                    continue
                score = settlement_score[int(point[1]/world.cell), int(point[0]/world.cell)]
                if score > best:
                    best, settlement = score, point

    def camera(point, distance, azimuth, lift, fov=48):
        ground = float(surface.height_at_world(*point))
        angle = np.deg2rad(azimuth)
        xy = point + distance * np.array([np.cos(angle), np.sin(angle)])
        z = max(ground + lift, float(surface.height_at_world(*xy)) + 9)
        return Camera3D(np.r_[xy, z], np.r_[point, ground + 1.5], fov_deg=fov)

    # Views over 50m retain projected-size LOD, unlike a close follow camera.
    cameras = {
        "overview": renderer.orbit(world, azimuth_deg=235, elevation_deg=42, dist_frac=1.25),
        "river": camera(river, 66, 235, 32, 50),
        "mountain": camera(mountain, 104, 250, 31, 53),
        "settlement": camera(settlement, 55, 225, 31, 46),
    }
    # Show the mountain's shoulder as well as the summit; the summit then sits in the
    # upper third instead of bisecting a frame that is mostly empty sky.
    cameras["mountain"].target[2] -= min(19, (surface.maximum - surface.minimum) * .35)
    return cameras


def _fingerprint(world: World) -> str:
    digest = hashlib.sha256()
    for field in (world.height, world.water, world.occ, world.pos, world.explored):
        digest.update(field.tobytes())
    for stream in STREAMS:
        digest.update(json.dumps(world.rng[stream].bit_generator.state,
                                 sort_keys=True).encode())
    digest.update(str((world.tick, world.t)).encode())
    return digest.hexdigest()


def write_fixture(world: World, path: Path) -> None:
    """Use the actual bridge schema without opening a socket or consuming world RNG."""
    fixture_world = copy.copy(world)
    # The bridge's static perception image consumes its map stream during creation.
    fixture_world.rng = copy.deepcopy(world.rng)
    messages = []
    server = SimpleNamespace(send_to=lambda _client, message: messages.append(message))
    bridge = BridgeNode(fixture_world, server)
    bridge._greet(None)
    idle = SimpleNamespace(activity=np.zeros(world.n, dtype=int))
    messages.extend([bridge._state(fixture_world, idle, None), bridge._fog(fixture_world)])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(messages, separators=(",", ":")), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="demo")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--width", type=int, default=1000)
    parser.add_argument("--height", type=int, default=700)
    parser.add_argument("--out", type=Path, default=Path("runs/3d/landscape"))
    parser.add_argument("--fixture", type=Path, help="also write initial bridge messages")
    parser.add_argument("--views", nargs="+", choices=("overview", "river", "mountain", "settlement"),
                        default=("overview", "river", "mountain", "settlement"))
    args = parser.parse_args()
    start = time.perf_counter()
    world = World(Scenario.load(args.scenario), args.seed)
    original = _fingerprint(world)
    renderer = Renderer3D(world, args.width, args.height)
    cameras = review_cameras(world, renderer)
    summary = {
        "scenario": args.scenario, "seed": args.seed, "simulation_ticks": 0,
        "map_metres": [world.scn.map.width_m, world.scn.map.height_m],
        "resolution": [args.width, args.height],
        "setup_seconds": round(time.perf_counter() - start, 4), "views": {},
    }
    args.out.mkdir(parents=True, exist_ok=True)
    for name in args.views:
        camera = cameras[name]
        renderer._tiles.clear()
        begin = time.perf_counter()
        frame = renderer.render(world, camera, fog=False, robots=False)
        png.write(args.out / f"{name}.png", frame)
        meshes = list(renderer._tiles.values())
        summary["views"][name] = {
            "camera": {"position": camera.pos.tolist(), "target": camera.target.tolist(),
                       "horizontal_fov_degrees": camera.fov_deg},
            "seconds": round(time.perf_counter() - begin, 4),
            "visible_terrain_tiles": len(meshes),
            "terrain_triangles": sum(len(land.triangles) for land, _water in meshes),
            "water_triangles": sum(len(water.triangles) for _land, water in meshes),
            "tile_strides": dict(sorted(Counter(key[2] for key in renderer._tiles).items())),
        }
        print(f"{name}: {summary['views'][name]['seconds']:.2f}s -> "
              f"{args.out / (name + '.png')}", flush=True)
    if args.fixture:
        write_fixture(world, args.fixture)
    if _fingerprint(world) != original:
        raise RuntimeError("Landscape review changed simulation arrays or RNG state")
    summary["simulation_fingerprint"] = original
    summary["world_unchanged"] = True
    (args.out / "review.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Authored river-valley crops in metres, using the same 2.5D traversal fields.

Layout lives in scenario YAML. Seed variation changes debris and tree footprints,
never the confluence, buildings, roads or scale. No external data/service is needed.
"""

from __future__ import annotations

import numpy as np

from . import grid
from .terrain import _segment_frame, _smoothstep, _value_noise, grade_roads


def road_network(layout):
    points, edges = [], []
    for path in layout["roads"]:
        previous = None
        for xy in path:
            point = tuple(xy)
            if point not in points:
                points.append(point)
            index = points.index(point)
            if previous is not None:
                edges.append((previous, index))
            previous = index
    return points, edges


def channel_fields(shape, cell, layout):
    """Closest channel's distance, half-width, surface level and centre depth.

    Each survey-style control point is [x, y, water elevation]. Both headwaters
    meet the downstream arm at the same point and elevation; interpolation is
    continuous, including through bends. Width and elevation remain metre-scale.
    """
    gx, gy = grid.cell_centres(shape, cell)
    distance = np.full(shape, np.inf, dtype=np.float32)
    half_width, level, depth = (np.zeros(shape, np.float32) for _ in range(3))
    for channel in layout["channels"]:
        points = np.array(channel["points"], dtype=float)
        for a, b in zip(points[:-1], points[1:], strict=True):
            d, t = _segment_frame(gx, gy, a[:2], b[:2])
            take = d < distance
            distance[take] = d[take]
            half_width[take] = channel["width_m"] * .5
            level[take] = (a[2] + t * (b[2] - a[2]))[take]
            depth[take] = channel["depth_m"]
    return distance, half_width, level, depth


def ellipse_mask(gx, gy, ellipse):
    x, y, rx, ry = ellipse[:4]
    return ((gx-x)/rx)**2 + ((gy-y)/ry)**2 <= 1


def occupancy(rng, scenario, layout):
    """Small houses on terraces, wooded shoulders, and a debris fan by the junction."""
    shape, cell = scenario.grid_shape, scenario.map.cell
    gx, gy = grid.cell_centres(shape, cell)
    occ = np.full(shape, grid.FREE, np.uint8)
    distance, width, _, _ = channel_fields(shape, cell, layout)
    dry = distance > width + layout["bank_clearance_m"]
    woods = np.zeros(shape, bool)
    for ellipse in layout["woodlands"]:
        woods |= ellipse_mask(gx, gy, ellipse)
    # Short construction-time loop over props, never over robots in a tick.
    spacing = layout["tree_spacing_m"]
    for y in np.arange(spacing/2, scenario.map.height_m, spacing):
        for x in np.arange(spacing/2, scenario.map.width_m, spacing):
            x, y0 = np.array([x, y]) + rng.uniform(-spacing*.45, spacing*.45, 2)
            ix, iy = grid.world_to_cell(x, y0, cell, shape)
            if not woods[iy, ix] or not dry[iy, ix]:
                continue
            radius = rng.uniform(*layout["tree_radius_m"])
            patch = (abs(gx-x) < radius) & (abs(gy-y0) < radius)
            occ[patch & dry] = grid.WALL
    for ellipse in layout["landslides"]:
        fan = ellipse_mask(gx, gy, ellipse) & dry
        occ[fan] = grid.RUBBLE
    # Scattered loose boulders and debris follow dry banks; no rock-studded riverbed.
    for _ in range(layout["debris_patches"]):
        x, y = rng.uniform([0, 0], [scenario.map.width_m, scenario.map.height_m])
        radius = rng.uniform(*layout["debris_radius_m"])
        patch = ((gx-x)**2 + (gy-y)**2 < radius**2) & dry
        occ[patch] = grid.RUBBLE if rng.random() < .8 else grid.WALL
    for x, y, width, height in layout["buildings"]:
        house = (gx >= x) & (gx < x+width) & (gy >= y) & (gy < y+height)
        yard = (gx >= x-cell) & (gx < x+width+cell) & (gy >= y-cell) & (gy < y+height+cell)
        occ[yard] = grid.FREE
        occ[house] = grid.WALL
    for x, y, radius in scenario.keepouts():
        occ[(gx-x)**2 + (gy-y)**2 <= radius**2] = grid.FREE
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = grid.WALL
    return occ


def generate(rng, occ, cell, layout, keepouts, roads, edges):
    distance, width, surface, depth = channel_fields(occ.shape, cell, layout)
    gx, gy = grid.cell_centres(occ.shape, cell)
    bank = np.maximum(distance-width, 0)
    low, high = layout["shoulder_distance_m"]
    rise = (layout["terrace_grade"] * bank
            + layout["shoulder_rise_m"] * _smoothstep(np.clip((bank-low)/(high-low), 0, 1)))
    z = surface + layout["bank_height_m"] + rise
    noise = _value_noise(rng, occ.shape, [(max(2, round(32/cell)), .65),
                                        (max(2, round(9/cell)), .10)])
    z += noise * np.clip(bank / 12, 0, 1)
    # Blend a parabolic bed into the bank without an abrupt bank-top step.
    u = distance / width
    bed = surface - depth * np.clip(1-u*u, 0, 1)
    blend = _smoothstep(np.clip((distance-width)/layout["bank_clearance_m"], 0, 1))
    z = bed*(1-blend) + z*blend
    water = np.where(u < 1, np.maximum(surface-z, 0), 0)
    # Pads sit on the local terrace; their feathering must never drain a river.
    for x, y, radius in keepouts:
        d = np.hypot(gx-x, gy-y)
        ix, iy = grid.world_to_cell(x, y, cell, occ.shape)
        blend = (1-_smoothstep(np.clip(d/(radius*1.6), 0, 1))) * (water == 0)
        z = z*(1-blend) + z[iy, ix]*blend
    # All segments agree at junctions, including short switchback links. Project
    # endpoint heights onto the network's grade limit before pinning each profile.
    # This fixes infeasible endpoint pairs without flattening the surrounding valley.
    points = np.asarray(roads, dtype=float)
    ix, iy = grid.world_to_cell(*points.T, cell, occ.shape)
    nodes = z[iy, ix].copy()
    for _ in range(len(points)):
        before = nodes.copy()
        for i, j in edges:
            climb = np.linalg.norm(points[i]-points[j]) * layout["road_grade"]
            nodes[i] = min(nodes[i], nodes[j]+climb)
            nodes[j] = min(nodes[j], nodes[i]+climb)
        if np.array_equal(before, nodes):
            break
    z, carriageway = grade_roads(z.astype(np.float32), water, cell, roads, edges,
                                layout["road_width_m"]*.5, node_heights=nodes)
    # Small level foundations make terrace houses sit in the hillside. Retain the
    # graded road deck through their feathered shoulders; no pad can cut a road.
    foundations = []
    ground = z.copy()
    for x, y, width, height in layout["buildings"]:
        dx = np.maximum(np.maximum(x-gx, gx-(x+width)), 0)
        dy = np.maximum(np.maximum(y-gy, gy-(y+height)), 0)
        distance = np.hypot(dx, dy)
        inside = (dx == 0) & (dy == 0)
        pad = 1-_smoothstep(np.clip((distance-cell)/layout["foundation_shoulder_m"], 0, 1))
        pad *= ~carriageway & (water == 0)
        floor = np.median(ground[inside])
        z = z*(1-pad) + floor*pad
        foundations.append(((distance <= cell) & ~carriageway & (water == 0), floor))
    # A neighbouring pad's shoulder must not tilt an already level foundation.
    for core, floor in foundations:
        z[core] = floor
    water = np.where(carriageway, 0, np.clip(surface-z, 0, water)).astype(np.float32)
    water[water < .02] = 0
    z = (z-z.min()).astype(np.float32)
    full = z.copy()
    for kind, low, span in ((grid.RUBBLE, .55, .9), (grid.WALL, .9, 1.3)):
        mask = occ == kind
        full[mask] += low + rng.random(occ.shape).astype(np.float32)[mask] * span
    return full, z, water

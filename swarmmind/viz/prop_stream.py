"""Procedural disaster scenery, the reference for ``godot/scripts/prop_stream.gd``.

Only occupied footprints receive tall geometry. Passable rubble gets shallow flakes,
not pillars a grounded robot would appear to drive through. Merged wall rectangles
locate broken stone and landmarks; no slab blankets a whole obstacle cluster.
All variation is a cell hash, independent of the simulation's random generators.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass

import numpy as np

TILE_METRES = 32.0
MERGE_METRES = 8.0
NEAR_DISTANCE = 95.0
MID_DISTANCE = 210.0
WALL_COLOR = (0.47, 0.45, 0.38)
SLAB_COLOR = (0.76, 0.70, 0.57)
RUBBLE_COLOR = (0.43, 0.39, 0.31)
ROOF_COLOR = (0.43, 0.22, 0.15)
WOOD_COLOR = (0.28, 0.21, 0.13)
LEAF_COLOR = (0.16, 0.29, 0.20)
ROCK_COLOR = (0.48, 0.49, 0.43)
REGION_METRES = 40.0


def cell_hash(x: int, y: int) -> int:
    """Integer-only 31-bit mix, identical on Python and Godot's 64-bit integers."""
    value = ((x + 1) * 73856093) ^ ((y + 1) * 19349663)
    return ((value ^ (value >> 13)) * 1274126177) & 0x7fffffff


def merged_rectangles(occupancy: np.ndarray, max_span: int = 8) -> Iterator[tuple[int, ...]]:
    """Yield disjoint (x, y, width, height) wall covers, with bounded slope span."""
    height, width = occupancy.shape
    used = np.zeros(occupancy.shape, dtype=bool)
    for y in range(height):
        for x in range(width):
            if used[y, x] or occupancy[y, x] != 1:
                continue
            right = x + 1
            while (right < min(width, x + max_span) and not used[y, right]
                   and occupancy[y, right] == 1):
                right += 1
            bottom = y + 1
            while (bottom < min(height, y + max_span)
                   and np.all(occupancy[bottom, x:right] == 1)
                   and not np.any(used[bottom, x:right])):
                bottom += 1
            used[y:bottom, x:right] = True
            yield x, y, right - x, bottom - y


@dataclass(frozen=True)
class Primitive:
    """One flat-shaded prism in simulator x/y/z coordinates."""

    vertices: np.ndarray
    color: tuple[float, float, float]
    kind: str


# Outward winding; the Godot port flips it after mapping z-up to y-up.
TRIANGLES = np.array([
    [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
    [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
    [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
], dtype=np.int32)


def box(center, size, yaw, color, kind) -> Primitive:
    vertices = np.array([
        [-.5, -.5, -.5], [.5, -.5, -.5], [.5, .5, -.5], [-.5, .5, -.5],
        [-.5, -.5, .5], [.5, -.5, .5], [.5, .5, .5], [-.5, .5, .5],
    ]) * size
    c, s = np.cos(yaw), np.sin(yaw)
    vertices[:, :2] = vertices[:, :2] @ np.array([[c, s], [-s, c]])
    return Primitive(vertices + center, color, kind)


def shaped(center, size, yaw, color, kind, taper=1.0, roll=0.0, lean=0.0) -> Primitive:
    """Eight-vertex frustum: a stone, tree crown or fallen roof, with no asset load."""
    vertices = box((0, 0, 0), size, 0, color, kind).vertices.copy()
    vertices[4:, :2] *= taper
    vertices[4:, 0] += lean * size[0]
    c, s = np.cos(roll), np.sin(roll)
    vertices[:, 1:] = vertices[:, 1:] @ np.array([[c, s], [-s, c]])
    c, s = np.cos(yaw), np.sin(yaw)
    vertices[:, :2] = vertices[:, :2] @ np.array([[c, s], [-s, c]])
    return Primitive(vertices + center, color, kind)


def landscape_kind(x: float, y: float) -> str:
    """Broad deterministic stands, instead of mixing a different object into each cell."""
    region = cell_hash(int(x // REGION_METRES), int(y // REGION_METRES)) % 9
    return "forest" if region < 4 else ("rock" if region < 7 else "ruin")


def _landmark(cx, cy, width, depth, base, hashed, biome, lod):
    """Each group has a persistent silhouette and a strict 16-primitive ceiling."""
    result = []
    bounds = (cx - width * .48, cx + width * .48,
              cy - depth * .48, cy + depth * .48)
    flip_x, flip_y = (-1 if hashed % 2 else 1), (-1 if hashed % 3 else 1)

    def add(x, y, z, size, color, kind, yaw=0.0, taper=1.0, roll=0.0, lean=0.0):
        primitive = shaped((cx+x, cy+y, base+z), size, yaw, color, kind, taper, roll, lean)
        # Half-turn variants retain the same footprint and triangle winding.
        if flip_x == flip_y:
            primitive.vertices[:, 0] = cx + (primitive.vertices[:, 0]-cx)*flip_x
            primitive.vertices[:, 1] = cy + (primitive.vertices[:, 1]-cy)*flip_y
        # Rotated rubble and crowns must never overhang a passable road or river.
        primitive.vertices[:, 0] = np.clip(primitive.vertices[:, 0], bounds[0], bounds[1])
        primitive.vertices[:, 1] = np.clip(primitive.vertices[:, 1], bounds[2], bounds[3])
        result.append(primitive)

    if biome == "forest" and min(width, depth) >= 1.8:
        trees = [(-.22, -.16), (.23, .22)] if min(width, depth) >= 4.5 else [(0, 0)]
        for i, (tx, ty) in enumerate(trees):
            radius = min(width, depth) * (.22 if len(trees) == 2 else .40)
            radius = min(radius, 1.65)
            tall = 3.2 + ((hashed >> (i * 3)) % 7) * .32
            x, y = tx * width, ty * depth
            add(x, y, tall*.37, (.20, .20, tall*.74), WOOD_COLOR, "trunk")
            for tier in range(2 if lod == 2 else 3):
                z = tall * (.40 + tier * .20)
                span = radius * (1.0 - tier * .22)
                tint = tuple(c * (1 + tier * .07) for c in LEAF_COLOR)
                add(x, y, z, (2*span, 2*span, tall*.54), tint, "crown",
                    yaw=(hashed % 11) * .13 + i, taper=.025, lean=.06)
        if lod == 0 and width >= 4 and depth >= 3:
            add(0, -depth*.30, .22, (width*.67, .26, .27), WOOD_COLOR,
                "fallen_tree", yaw=.13)
            add(-width*.21, -depth*.28, .42, (.62, .50, .65), WOOD_COLOR,
                "stump", taper=.62, lean=.1)
    elif biome == "ruin" and min(width, depth) >= 3:
        sx, sy = min(width*.76, 6.0), min(depth*.76, 5.0)
        tall = 1.5 + (hashed % 5)*.18
        if hashed % 3 == 0:
            tall *= .58  # The remaining shell has pancaked into its foundation.
        roof_color = ROOF_COLOR if hashed % 3 else (.29, .31, .29)
        # Missing front wall and broken side piers reveal a dark, debris-filled room.
        add(0, sy*.40, tall*.46, (sx*.92, .25, tall*.92), SLAB_COLOR, "house_wall",
            taper=.94)
        add(-sx*.43, sy*.05, tall*.38, (.26, sy*.75, tall*.76), SLAB_COLOR, "house_wall")
        add(sx*.43, sy*.12, tall*.27, (.26, sy*.62, tall*.54), SLAB_COLOR, "house_wall")
        # One gabled roof plane survives; the other has slipped into the room.
        add(-sx*.02, sy*.20, tall+sy*.14, (sx, sy*.58, .14), roof_color,
            "roof", roll=-.48)
        add(sx*.06, -sy*.18, tall*.51, (sx*.84, sy*.53, .17), roof_color,
            "collapsed_roof", yaw=.09, roll=.34)
        if lod < 2:
            add(-sx*.33, -sy*.31, tall*.52, (.29, .34, tall), SLAB_COLOR, "broken_pier")
            add(sx*.27, -sy*.31, tall*.25, (.33, .37, tall*.48), SLAB_COLOR, "broken_pier")
            add(sx*.21, sy*.25, tall+.50, (.42, .46, 1.0), RUBBLE_COLOR, "chimney")
        if lod == 0:
            add(-sx*.08, -sy*.29, .30, (sx*.77, .16, .20), WOOD_COLOR,
                "fallen_rafter", yaw=-.14, roll=.2)
            for i in range(3):
                add((i-1)*sx*.22, -sy*.20+(i%2)*sy*.16, .18+i*.055,
                    (.52, .41, .29), SLAB_COLOR, "buried_masonry",
                    yaw=i*.67, taper=.55, lean=.13)
    elif min(width, depth) >= 1.8:
        # Angular, half-buried rock outcrops, broad enough to read as geology.
        add(-width*.08, depth*.04, .36, (width*.75, depth*.76, 1.05),
            ROCK_COLOR, "boulder", yaw=.12, taper=.57, lean=.09)
        if lod < 2:
            add(width*.25, -depth*.21, .20, (width*.35, depth*.40, .71),
                WALL_COLOR, "boulder", yaw=-.3, taper=.46, lean=-.12)
        if lod == 0 and min(width, depth) >= 3:
            add(-width*.29, -depth*.24, .08, (width*.30, depth*.27, .43),
                ROCK_COLOR, "buried_rock", yaw=.4, taper=.40, lean=.1)
    return result


def tile_primitives(occupancy: np.ndarray, cell: float, height_at: Callable,
                    lod: int = 0, origin: tuple[int, int] = (0, 0),
                    water: np.ndarray | None = None, reference=None) -> list[Primitive]:
    """Create a tile's common obstacle cover and progressively simpler decorations.

    ``height_at(x,y)`` samples the displayed triangle surface in world metres. Origin
    is the tile's first grid cell, so independently loaded tiles always look identical.
    """
    ox, oy = origin
    if water is not None:
        occupancy = np.where(water > .02, 0, occupancy)
    primitives = []
    for x, y, w, h in merged_rectangles(occupancy, max(1, int(MERGE_METRES / cell))):
        gx, gy = x + ox, y + oy
        hashed = cell_hash(gx, gy)
        corners = np.array([(gx * cell, gy * cell), ((gx + w) * cell, gy * cell),
                            ((gx + w) * cell, (gy + h) * cell),
                            (gx * cell, (gy + h) * cell)])
        floor = np.array([height_at(*p) for p in corners])
        gradient = np.array([(floor[1] - floor[0]) / (w * cell),
                             (floor[3] - floor[0]) / (h * cell)])
        cx, cy = (gx + w * .5) * cell, (gy + h * .5) * cell
        biome = landscape_kind(cx, cy)
        if reference is not None:
            from .reference_landscape import biome_at

            biome = biome_at(reference, cx, cy)
        # Wooded Nepal shoulders carry trees on steeper slopes than house terraces.
        limit = .70 if reference is not None and biome == "forest" else .40
        if np.linalg.norm(gradient) > limit:
            biome = "rock"
        tint = RUBBLE_COLOR if biome == "ruin" else (
            (.31, .35, .25) if biome == "forest" else ROCK_COLOR)
        # Embedded scree breaks the grid silhouette. A broad flat cap, however thin,
        # reads as a concrete platform from above; these short, irregular stones
        # follow the slope individually and leave natural ground between them.
        nx, ny = max(1, int(np.ceil(w*cell/3))), max(1, int(np.ceil(h*cell/3)))
        for sy in range(ny):
            for sx in range(nx):
                stone = cell_hash(gx*11+sx, gy*11+sy)
                size = np.array([w*cell/nx, h*cell/ny])
                centre = np.array([gx*cell, gy*cell]) + size*[sx+.5, sy+.5]
                centre += size * [((stone >> 3) % 5-2)*.025, ((stone >> 6) % 5-2)*.025]
                offsets = np.array([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]])*size
                bottom = centre + offsets*.78
                factors = np.array([.30+((stone >> (i*4)) % 5)*.035 for i in range(4)])
                top = centre + offsets*factors[:, None]
                low = np.array([height_at(*p) for p in bottom])-.12
                high = np.array([height_at(*p) for p in top])+.16+(stone % 5)*.035
                vertices = np.vstack([np.column_stack([bottom, low]), np.column_stack([top, high])])
                stone_color = tuple(c*(.88+(stone % 7)*.025) for c in tint)
                primitives.append(Primitive(vertices, stone_color, "wall"))
        base = height_at(cx, cy) + .06
        primitives.extend(_landmark(cx, cy, w*cell, h*cell, base, hashed, biome, lod))
    if lod == 0:
        for y in range(occupancy.shape[0]):
            for x in range(occupancy.shape[1]):
                if occupancy[y, x] != 2 or cell_hash(x + ox, y + oy) % 13:
                    continue
                gx, gy = (x + ox + .5) * cell, (y + oy + .5) * cell
                primitives.append(box((gx, gy, height_at(gx, gy) + .03),
                                      (.56 * cell, .37 * cell, .06), 0,
                                      RUBBLE_COLOR, "flake"))
    return primitives

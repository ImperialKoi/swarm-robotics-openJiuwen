"""Obstacle coverage survives streaming detail changes; robots keep passable ground."""

import numpy as np
import pytest

from swarmmind.viz.prop_stream import TRIANGLES, cell_hash, merged_rectangles, tile_primitives


@pytest.mark.parametrize("shape", [(1, 1), (13, 31), (32, 32), (64, 48)])
def test_merged_obstacles_exactly_cover_walls_and_never_cover_a_passable_cell(shape):
    y, x = np.indices(shape)
    occupancy = ((x * 17 + y * 13 + x * y) % 7 < 5).astype(np.uint8)
    occupancy[::7, ::3] = 2
    cover = np.zeros(shape, dtype=int)
    for x, y, w, h in merged_rectangles(occupancy):
        assert 0 < w <= 8 and 0 < h <= 8
        cover[y:y+h, x:x+w] += 1
    np.testing.assert_array_equal(cover, occupancy == 1)


def test_merge_budget_reduces_a_solid_patch_by_sixty_four_times():
    rects = list(merged_rectangles(np.ones((32, 32), np.uint8)))
    assert len(rects) == 16
    assert len(rects) * 12 == 192  # former per-cell cubes: 12,288 triangles


def test_lod_retains_identical_wall_geometry_but_drops_decorations():
    occupancy = np.ones((16, 16), np.uint8)
    occupancy[8:, 8:] = 2
    levels = [tile_primitives(occupancy, 1, lambda x, y: .05*x + .1*y, lod)
              for lod in range(3)]
    assert len(levels[0]) > len(levels[1]) > len(levels[2])
    walls = [[p.vertices for p in level if p.kind == "wall"] for level in levels]
    for lod in (1, 2):
        np.testing.assert_array_equal(walls[0], walls[lod])


def test_debris_foundation_follows_ground_instead_of_building_a_deck_above_a_crest():
    def height_at(x, y):
        return 2.7 * max(0, 1 - abs(x-4)/4) + .1*y

    walls = [p for p in tile_primitives(np.ones((8, 8), np.uint8), 1, height_at, lod=2)
             if p.kind == "wall"]
    for wall in walls:
        for x, y, z in wall.vertices[:4]:
            assert z == pytest.approx(height_at(x, y) - .12)
        for x, y, z in wall.vertices[4:]:
            assert .159 < z - height_at(x, y) < .301
        # No long rectangle can become a concrete terrace across a mountain.
        assert np.ptp(wall.vertices[:, :2], axis=0).max() < 3


@pytest.mark.parametrize("lod", [0, 1, 2])
def test_landmarks_stay_inside_dry_blocked_footprints(lod):
    occupancy = np.ones((24, 24), np.uint8)
    occupancy[9:13] = 0  # road
    water = np.zeros_like(occupancy, dtype=float)
    water[:, 12:15] = .8  # channel bisecting blocked ground
    before = occupancy.copy()
    for origin in ((0, 0), (40, 40), (80, 80), (120, 120)):
        items = tile_primitives(occupancy, 1, lambda _x, _y: 0, lod, origin, water)
        assert items
        for item in items:
            # Every triangle interior stays in a genuinely blocked dry cell.
            points = item.vertices[TRIANGLES].mean(axis=1)[:, :2] - origin
            ix, iy = np.floor(np.clip(points, 0, 23.999999)).astype(int).T
            assert (occupancy[iy, ix] == 1).all(), item.kind
            assert (water[iy, ix] == 0).all(), item.kind
    np.testing.assert_array_equal(occupancy, before)


def test_landscape_contains_distinct_bounded_forest_rock_and_ruin_silhouettes():
    kinds = set()
    occupancy = np.ones((8, 8), np.uint8)
    for origin in ((x, y) for x in range(0, 160, 40) for y in range(0, 160, 40)):
        items = tile_primitives(occupancy, 1, lambda _x, _y: 0, origin=origin)
        assert len(items) <= 25  # up to nine scree stones plus a bounded landmark group
        kinds.update(item.kind for item in items)
    assert {"crown", "fallen_tree", "boulder", "collapsed_roof", "buried_masonry"} <= kinds


def test_steep_ground_has_rocks_instead_of_houses_or_trees():
    for x in range(0, 160, 40):
        items = tile_primitives(np.ones((8, 8), np.uint8), 1,
                                lambda x, y: .7*x+.4*y, origin=(x, 40))
        assert {item.kind for item in items} <= {"wall", "boulder", "buried_rock"}


def test_passable_rubble_is_a_shallow_flake_without_structural_props():
    height_at = lambda _x, _y: 2.5  # noqa: E731
    items = tile_primitives(np.full((16, 16), 2, np.uint8), 1, height_at)
    assert items and {p.kind for p in items} == {"flake"}
    for primitive in items:
        assert primitive.vertices[:, 2].min() == pytest.approx(2.5)
        assert primitive.vertices[:, 2].max() == pytest.approx(2.56)


def test_tile_builds_are_order_independent_and_do_not_change_occupancy():
    occupancy = np.ones((16, 16), np.uint8)
    before = occupancy.copy()
    sampler = lambda x, y: .1*x + .2*y  # noqa: E731
    a = tile_primitives(occupancy, .5, sampler, origin=(64, 64))
    tile_primitives(occupancy, .5, sampler, origin=(0, 0))
    b = tile_primitives(occupancy, .5, sampler, origin=(64, 64))
    np.testing.assert_array_equal(occupancy, before)
    for first, second in zip(a, b, strict=True):
        np.testing.assert_array_equal(first.vertices, second.vertices)
    assert cell_hash(1, 1) != cell_hash(64, 64)


def test_faces_point_outwards_for_the_reference_renderer():
    primitive = tile_primitives(np.ones((2, 2), np.uint8), 1, lambda _x, _y: 0, lod=2)[0]
    faces = primitive.vertices[TRIANGLES]
    normals = np.cross(faces[:, 1] - faces[:, 0], faces[:, 2] - faces[:, 0])
    assert np.all(np.sum(normals * (faces.mean(axis=1)-primitive.vertices.mean(axis=0)), axis=1) > 0)

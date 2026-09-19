"""Mesh/grounding invariants; these are display tests, never mission training runs."""

from pathlib import Path

import numpy as np
import pytest

from swarmmind.viz.terrain_surface import CLEARANCE, TerrainSurface
from swarmmind.viz.units import DIG, REST, build_model, posed_parts, valid_variants
from swarmmind.viz.water import river_frame, surface_color


def _surface(height, cell=1.0):
    height = np.asarray(height, dtype=float)
    return TerrainSurface(height, np.zeros_like(height, dtype=np.uint8),
                          np.zeros_like(height), cell)


def _plane(a=.24, b=-.18, cell=1.0):
    surface = _surface(np.zeros((16, 20)), cell)
    yy, xx = np.indices(surface.corners.shape)
    surface.corners[:] = 3 + a*xx*cell + b*yy*cell
    surface.minimum = float(surface.corners.min())
    return surface


def test_recovery_does_not_modify_world_or_consume_random_state():
    yy, xx = np.indices((20, 24))
    occ = ((xx+yy) % 3).astype(np.uint8)
    land = 4 + .1*xx + .15*yy
    height = land + np.where(occ == 1, 1.55, np.where(occ == 2, 1.0, 0))
    water = np.zeros_like(height)
    originals = [a.copy() for a in (height, occ, water)]
    first = TerrainSurface(height, occ, water, 1.0)
    second = TerrainSurface(height, occ, water, 1.0)
    for actual, original in zip((height, occ, water), originals, strict=True):
        np.testing.assert_array_equal(actual, original)
    np.testing.assert_array_equal(first.corners, second.corners)
    # Centre samples turn into shared physical corners, half a cell southwest.
    np.testing.assert_allclose(first.corners[4:-4, 4:-4],
                               (4+.1*(xx-.5)+.15*(yy-.5))[4:-3, 4:-3])


@pytest.mark.parametrize("stride", [1, 2, 4, 8])
def test_mesh_centroids_and_sampler_share_triangle_planes_including_map_edge(stride):
    yy, xx = np.indices((13, 19))
    surface = _surface(3+np.sin(xx*.4)+np.cos(yy*.3), .7)
    mesh = surface.tile_arrays(0, 0, surface.gw, surface.gh, stride, skirts=False)
    triangles = mesh.vertices[mesh.triangles]
    centroid = triangles.mean(axis=1)
    np.testing.assert_allclose(surface.height_at_world(centroid[:, 0], centroid[:, 1], stride),
                               centroid[:, 2], atol=1e-12)
    # Samples clamp safely at and beyond the last partial LOD cell.
    assert surface.height_at_world(1000, 1000, stride) == surface.corners[-1, -1]


@pytest.mark.parametrize("stride", [1, 2, 4, 8])
def test_analytic_slope_normal_and_pose(stride):
    surface = _plane()
    x, y, heading = 8.3, 7.1, .62
    expected = np.array([-.24, .18, 1])
    expected /= np.linalg.norm(expected)
    np.testing.assert_allclose(surface.normal_at_world(x, y, stride), expected)
    basis, origin = surface.robot_pose(x, y, heading, stride=stride)
    np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(basis[:, 2], expected)
    np.testing.assert_allclose(origin[2], 3+.24*x-.18*y+CLEARANCE)
    assert np.linalg.det(basis) == pytest.approx(1)


def test_ridge_between_support_samples_cannot_pierce_chassis_plane():
    surface = _surface(np.zeros((16, 20)), .4)
    surface.corners[16, 20] = 1.3
    x, y = 7.7, 6.45
    basis, origin = surface.robot_pose(x, y, .37)
    xx, yy = np.meshgrid(np.linspace(-.85, 1.85, 120), np.linspace(-.73, .73, 90))
    support = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], -1) @ basis.T + origin
    ground = surface.height_at_world(support[:, 0], support[:, 1])
    assert np.min(support[:, 2]-ground) >= CLEARANCE-1e-10


def test_fixed_basis_support_catches_fine_crest_beside_coarse_tile():
    surface = _surface(np.zeros((20, 20)))
    surface.corners[9, 9] = 1.8  # falls between stride-4 mesh vertices
    x, y = 8.4, 8.7
    basis, coarse_origin = surface.robot_pose(x, y, .2, stride=4)
    fine_height = surface.support_height(x, y, basis, stride=1)
    assert fine_height > coarse_origin[2] + 1
    xx, yy = np.meshgrid(np.linspace(-.85, 1.85, 80), np.linspace(-.73, .73, 60))
    vertices = np.stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)], -1) @ basis.T
    vertices += [x, y, fine_height]
    assert (vertices[:, 2] >= surface.height_at_world(vertices[:, 0], vertices[:, 1])).all()


def test_rotor_guards_clear_crest_beyond_landing_skids():
    surface = _surface(np.zeros((22, 24)), .5)
    surface.corners[16, 18] = 1.8
    basis, origin = surface.robot_pose(8, 7, 0, "rotor")
    model = build_model("none", "rotor")
    points = np.concatenate([v for _, v in posed_parts(model)]) @ basis.T + origin
    assert (points[:, 2] >= surface.height_at_world(points[:, 0], points[:, 1])).all()


@pytest.mark.parametrize("lane,chassis", valid_variants())
def test_unit_geometry_stays_above_sloped_ground_through_animation(lane, chassis):
    surface = _plane(.55, .23)
    model = build_model(lane, chassis)
    basis, origin = surface.robot_pose(8.2, 7.1, .31, chassis)
    for time, travel, state in [(0, 0, REST), (.25, .25, DIG), (.5, .75, DIG)]:
        vertices = np.concatenate([v for _, v in posed_parts(model, time=time, travel=travel, state=state)])
        placed = vertices @ basis.T + origin
        clearance = placed[:, 2]-surface.height_at_world(placed[:, 0], placed[:, 1])
        assert clearance.min() >= 0, (lane, chassis, clearance.min())


def test_adjacent_tiles_share_identical_heights_and_colors():
    yy, xx = np.indices((12, 16))
    surface = _surface(np.sin(xx*.3)+np.cos(yy*.4))
    left = surface.tile_arrays(0, 0, 8, 12, 1, skirts=False)
    right = surface.tile_arrays(8, 0, 16, 12, 4, skirts=False)
    lv = left.vertices[left.vertices[:, 0] == 8]
    rv = right.vertices[right.vertices[:, 0] == 8]
    np.testing.assert_array_equal(lv[::4], rv)
    for mesh in (left, right):
        assert np.isfinite(mesh.vertices).all()
        assert mesh.triangles.max() < len(mesh.vertices)


def test_water_is_connected_and_preserves_exact_dry_crossing():
    height = np.zeros((8, 12))
    depth = np.zeros_like(height)
    depth[:, 3:8] = .8
    depth[4, :] = 0  # a real ford/causeway remains dry, even in far terrain tiles
    surface = TerrainSurface(height, np.zeros_like(height), depth, 1)
    mesh = surface.water_arrays(0, 0, 12, 8)
    centroids = mesh.vertices[mesh.triangles].mean(1)
    ix, iy = np.floor(centroids[:, :2]).astype(int).T
    assert (depth[iy, ix] > 0).all()
    assert len(mesh.triangles) == np.count_nonzero(depth) * 2
    # Duplicate vertices from neighbouring quads have exactly the same elevation.
    by_position = {}
    for x, y, z in mesh.vertices:
        if (x, y) in by_position:
            assert z == by_position[x, y]
        by_position[x, y] = z
    assert (mesh.vertices[:, 2] > surface.height_at_world(mesh.vertices[:, 0], mesh.vertices[:, 1])).all()


def test_chamfered_shores_cannot_flood_a_dry_cell_for_any_local_mask():
    # Includes convex/concave corners, diagonal pools, one-cell fords and islands.
    # Check triangle interiors; a shared boundary itself belongs to neither cell.
    weights = np.array([(a, b, 1-a-b) for a in (.01, .25, .5, .75, .98)
                        for b in (.001, (1-a)*.5, (1-a)*.99)])
    for mask in range(1, 512):
        depth = np.zeros((5, 5))
        depth[1:4, 1:4] = np.array([(mask >> i) & 1 for i in range(9)]).reshape(3, 3)
        surface = TerrainSurface(np.zeros_like(depth), np.zeros_like(depth), depth, 1)
        mesh = surface.water_arrays(0, 0, 5, 5)
        points = np.einsum("wc,tcd->twd", weights, mesh.vertices[mesh.triangles])
        ix, iy = np.floor(points[..., :2]).astype(int).reshape(-1, 2).T
        assert (depth[iy, ix] > 0).all(), mask


def test_chamfered_water_matches_across_streamed_tile_boundaries():
    yy, xx = np.indices((12, 16))
    water = np.where((yy > 2) & (yy < 9) & ((xx + yy) % 7 != 0), .8, 0)
    surface = TerrainSurface(.04*xx+.08*yy, np.zeros_like(water), water, 1)
    whole = surface.water_arrays(0, 0, 16, 12)
    parts = [surface.water_arrays(x, 0, x+8, 12) for x in (0, 8)]
    # Coordinate equality, including shoreline perturbation, is independent of tile.
    expected = {tuple(row) for row in whole.vertices}
    actual = {tuple(row) for mesh in parts for row in mesh.vertices}
    assert actual == expected
    assert sum(len(mesh.triangles) for mesh in parts) == len(whole.triangles)


def test_river_current_coordinates_follow_downstream_bends():
    layout = {"channels": [{"points": [[0, 0, 3], [10, 0, 2], [10, 10, 1]]}]}
    frame = river_frame(np.array([2, 8, 10, 10]), np.array([0, 0, 2, 8]), layout)
    assert np.all(np.diff(frame[:, 0]) > 0)
    np.testing.assert_allclose(frame[:, 1], 0)
    depth = np.array([.1, .4, .7, 1.0])
    first, later = surface_color(frame, depth), surface_color(frame, depth, time=1)
    assert np.isfinite(first).all() and np.isfinite(later).all()
    assert (first >= 0).all() and (first <= 1).all()
    assert not np.array_equal(first, later)


def test_coarse_mesh_reduces_geometry_and_skirts_close_perimeter():
    surface = _surface(np.ones((32, 32)))
    fine = surface.tile_arrays(0, 0, 32, 32, 1)
    coarse = surface.tile_arrays(0, 0, 32, 32, 8)
    assert len(coarse.triangles) < len(fine.triangles)/20
    assert coarse.vertices[:, 2].min() == surface.minimum-1
    edges = np.sort(np.concatenate([coarse.triangles[:, [0, 1]], coarse.triangles[:, [1, 2]],
                                    coarse.triangles[:, [2, 0]]]), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    boundary = unique[counts == 1]
    # Only the underside remains open; no cracks along terrain or skirt sides.
    assert (coarse.vertices[boundary, 2] == surface.minimum-1).all()


def test_godot_port_keeps_geometry_contract():
    source = Path("godot/scripts/terrain_surface.gd").read_text()
    assert "const WALL_LIFT := 1.55" in source
    assert "const RUBBLE_LIFT := 1.0" in source
    assert "const SMOOTH_PASSES := 2" in source
    assert "const CLEARANCE := 0.045" in source
    assert "if u + v <= 1.0:" in source
    assert "a + width, a + 1, a + 1, a + width, a + width + 1" in source


@pytest.mark.parametrize("cell", [0, -1, float("nan")])
def test_bad_cell_rejected(cell):
    with pytest.raises(ValueError):
        _surface(np.zeros((3, 3)), cell)

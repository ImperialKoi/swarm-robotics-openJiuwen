"""Exact routing to collection points: no corner cutting, no one-cell gaps, off by default."""

from __future__ import annotations

import numpy as np

from swarmmind.control.zone_routing import direction_grid, erode8, fine_field
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario


def test_no_diagonal_squeeze_between_two_blocked_corners():
    P = np.ones((3, 3), dtype=bool)
    P[0, 1] = P[1, 0] = False
    src = np.zeros_like(P)
    src[0, 0] = True
    d = fine_field(P, src)
    assert np.isinf(d[1, 1]), "a body cannot slide between two blocked corners"


def test_detour_goes_round_a_wall_and_descent_points_downhill():
    P = np.ones((7, 7), dtype=bool)
    P[3, 0:6] = False
    src = np.zeros_like(P)
    src[0, 0] = True
    d = fine_field(P, src)
    assert np.isfinite(d[6, 0]) and d[6, 0] > 6
    dirs = direction_grid(P, d)
    assert dirs[6, 0] >= 0 and dirs[0, 0] == -1


def test_erosion_closes_one_cell_gaps():
    P = np.zeros((9, 9), dtype=bool)
    P[1:8, 1:4] = True
    P[1:8, 5:8] = True
    P[4, 4] = True                    # a one-cell gap between two open blocks
    E = erode8(P)
    src = np.zeros_like(P)
    src[4, 2] = True
    assert np.isinf(fine_field(E, src)[4, 6])


def test_off_by_default_and_identical_when_off():
    """Off because the demo runs Tier 3 live, where the two machines disagree in sign
    (M-76e vs M-76f) -- not because the fix is doubted with Tier 3 silent."""
    scn = Scenario.load("test")
    assert Mission(scn, 42, hivemind=False).nav.zones is None
    a = Mission(scn, 42, hivemind=False).run(60.0)
    b = Mission(scn, 42, hivemind=False, zone_routing=False).run(60.0)
    assert a.hash() == b.hash()


def test_routing_changes_what_the_swarm_does():
    scn = Scenario.load("test")
    on = Mission(scn, 42, hivemind=False, zone_routing=True).run(120.0)
    off = Mission(scn, 42, hivemind=False, zone_routing=False).run(120.0)
    assert on.hash() != off.hash()

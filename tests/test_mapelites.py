"""The archive, the emitters, and the run loop.

MAP-Elites is illumination, not optimisation: the deliverable is an archive with a good
robot in every cell, so most of what is worth asserting is about *coverage and
bookkeeping* rather than about any single fitness number.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from swarmmind.sim.robot import LANES
from swarmmind.training.mapelites.genome import N_GENES
from swarmmind.training.mapelites.run import (
    ARCHIVE_DIMS,
    BATCH_SIZE,
    MEASURE_RANGES,
    N_EMITTERS,
    build_scheduler,
    save,
)


def test_the_archive_has_one_cell_per_lane_and_speed_bin():
    s = build_scheduler()
    assert ARCHIVE_DIMS[0] == len(LANES), "archive lane axis does not match the lanes"
    assert s.archive.cells == ARCHIVE_DIMS[0] * ARCHIVE_DIMS[1]
    assert s.archive.solution_dim == N_GENES


def test_every_lane_index_lands_in_its_own_column():
    """Lane is a discrete measure faked as a continuous one; `lane + 0.5` must not
    round into a neighbour, or elites get filed under the wrong actuator."""
    s = build_scheduler()
    cols = set()
    for lane in range(len(LANES)):
        idx = s.archive.index_of(np.array([[lane + 0.5, 0.9]]))
        grid = s.archive.int_to_grid_index(idx)[0]
        cols.add(int(grid[0]))
    assert cols == set(range(len(LANES))), f"lanes collided into columns {cols}"


#: Measured mean speed across 52 elites from three runs (M-32). The archive axis has to
#: bracket this, and must not extend far past it.
OBSERVED_SPEED = (0.24, 1.32)


def test_the_speed_axis_brackets_what_robots_actually_do():
    """Both directions matter, and the first version got the second one wrong.

    Too narrow and real robots pile into the end bins. Too wide and cells are unreachable
    by construction: at 0.0-1.8 the bottom bin and the top two were never once reached,
    so 12 of 40 cells could not be filled however good the search was, and the coverage
    number was quietly measuring the axis rather than the search.
    """
    lo, hi = MEASURE_RANGES[1]
    obs_lo, obs_hi = OBSERVED_SPEED
    assert lo <= obs_lo and hi >= obs_hi, f"axis {MEASURE_RANGES[1]} clips real robots"
    width = hi - lo
    covered = (min(hi, obs_hi) - max(lo, obs_lo)) / width
    assert covered > 0.75, (
        f"only {covered:.0%} of the speed axis is reachable; "
        f"{(1 - covered) * ARCHIVE_DIMS[1]:.0f} bins per lane are dead cells"
    )


def test_one_emitter_per_lane_starting_at_that_lane_midpoint():
    """All five emitters used to start at the all-0.5 midpoint, which decodes to
    `gripper`: 87% of proposals landed in two lanes and the scout and relay lanes were
    barely searched at all (M-32). One emitter per lane fixes it at the source."""
    from swarmmind.training.mapelites.genome import lane_of

    s = build_scheduler()
    assert len(s.emitters) == N_EMITTERS == len(LANES)
    started = [lane_of(np.asarray(e.x0)) for e in s.emitters]
    assert sorted(started) == sorted(LANES), f"emitters cover only {set(started)}"
    for e in s.emitters:
        assert e.batch_size == BATCH_SIZE


def test_proposals_are_spread_across_every_lane():
    """The measurable consequence: no lane may be starved of samples."""
    from collections import Counter

    from swarmmind.training.mapelites.genome import lane_of

    counts = Counter(lane_of(g) for g in build_scheduler().ask())
    assert set(counts) == set(LANES), f"lanes never proposed: {set(LANES) - set(counts)}"
    assert min(counts.values()) >= 5, f"a lane is starved of samples: {dict(counts)}"


def test_batch_size_is_near_the_cma_es_default_for_this_dimensionality():
    """Batch 30 was 2.5x the default population for 15 dimensions, and at a ~1,400
    evaluation budget it bought 9 generations where CMA-ES needs tens."""
    default = int(4 + 3 * np.log(N_GENES))
    assert default <= BATCH_SIZE <= 2 * default, (
        f"batch {BATCH_SIZE} against a CMA-ES default of {default} for {N_GENES} genes"
    )


def test_the_search_is_bounded_to_the_unit_cube():
    s = build_scheduler()
    for e in s.emitters:
        assert np.allclose(e.lower_bounds, 0.0)
        assert np.allclose(e.upper_bounds, 1.0)


def test_ask_returns_a_full_batch_of_genomes():
    s = build_scheduler()
    g = s.ask()
    assert g.shape == (N_EMITTERS * BATCH_SIZE, N_GENES)


def test_tell_fills_cells_and_keeps_the_better_elite():
    """The archive must be an elite map: a worse genome may not displace a better one."""
    s = build_scheduler()
    g = s.ask()
    n = len(g)
    rng = np.random.default_rng(0)
    lanes = rng.integers(0, len(LANES), n)
    measures = np.stack([lanes + 0.5, np.full(n, 0.9)], axis=1)
    s.tell(np.full(n, 5.0), measures)
    first = s.archive.stats.num_elites
    assert first > 0, "telling the archive filled nothing"
    assert s.archive.stats.obj_max == pytest.approx(5.0)

    # Same cells, strictly worse genomes: nothing may be overwritten.
    g2 = s.ask()
    m2 = np.stack([rng.integers(0, len(LANES), len(g2)) + 0.5,
                   np.full(len(g2), 0.9)], axis=1)
    s.tell(np.full(len(g2), 1.0), m2)
    assert s.archive.stats.obj_max == pytest.approx(5.0), "a worse elite displaced a better"


def test_checkpoint_round_trips(tmp_path):
    """A ten-hour run that dies at hour nine must not have nothing to show."""
    s = build_scheduler()
    g = s.ask()
    n = len(g)
    measures = np.stack([np.full(n, 0.5), np.full(n, 0.9)], axis=1)
    s.tell(np.linspace(1.0, 2.0, n), measures)

    save(s.archive, tmp_path, {"iteration": 3, "coverage": 0.1})
    loaded = np.load(tmp_path / "archive.npz")
    assert loaded["solution"].shape[1] == N_GENES
    assert len(loaded["objective"]) == s.archive.stats.num_elites
    assert loaded["measures"].shape[1] == 2
    meta = json.loads((tmp_path / "meta.json").read_text())
    assert meta["iteration"] == 3


def test_a_stored_archive_can_be_decoded_back_into_robots():
    """The archive is only useful if `select.py` can turn it back into a roster."""
    from swarmmind.training.mapelites.genome import decode_spec, lane_of

    s = build_scheduler()
    g = s.ask()
    n = len(g)
    s.tell(np.ones(n), np.stack([np.full(n, 0.5), np.full(n, 0.9)], axis=1))
    sols = np.asarray(s.archive.data()["solution"])
    assert len(sols) > 0
    spec = decode_spec(sols[0], "r00", "tracked")
    assert spec.actuator == lane_of(sols[0])
    assert spec.v_max > 0.0 and spec.radius > 0.0


# ---------------------------------------------------------------------- selection


def _fake_archive(n_per_lane: int = 6):
    """A synthetic archive with a known best and a known speed spread per lane."""
    from swarmmind.training.mapelites.genome import GENE_INDEX, N_GENES

    rng = np.random.default_rng(4)
    sols, objs, meas = [], [], []
    for lane in range(len(LANES)):
        for k in range(n_per_lane):
            g = rng.random(N_GENES)
            # The lane measure is derived from this gene in evaluate(), so a realistic
            # archive always has them agreeing. Random genomes filed under arbitrary
            # lanes would test a situation that cannot occur.
            g[GENE_INDEX["actuator_g"]] = (lane + 0.5) / len(LANES)
            sols.append(g)
            objs.append(float(k))                     # last one is the best in the lane
            meas.append([lane + 0.5, 0.2 + 0.15 * k])  # spread across the speed axis
    return np.array(sols), np.array(objs), np.array(meas)


def test_selection_takes_the_best_and_the_speed_extremes():
    """Top-3-by-fitness would give three near-identical robots and no visible diversity."""
    from swarmmind.training.mapelites.select import pick

    sol, obj, meas = _fake_archive()
    picked = pick(sol, obj, meas, per_lane=3)
    for lane_i, lane in enumerate(LANES):
        idx = picked[lane]
        assert len(idx) == 3
        assert all(int(meas[i, 0]) == lane_i for i in idx), "an elite crossed lanes"
        assert obj[idx[0]] == max(obj[j] for j in range(len(obj))
                                  if int(meas[j, 0]) == lane_i), "not the lane's best"
        speeds = [meas[i, 1] for i in idx]
        assert max(speeds) - min(speeds) > 0.3, (
            f"{lane}: selected robots span only {max(speeds) - min(speeds):.2f} m/s "
            "-- the archive's diversity is not visible on screen"
        )


def test_selection_is_reproducible():
    """A different demo swarm on each run makes rehearsed timings meaningless."""
    from swarmmind.training.mapelites.select import pick

    sol, obj, meas = _fake_archive()
    assert pick(sol, obj, meas) == pick(sol, obj, meas)


def test_an_empty_lane_is_reported_not_faked():
    from swarmmind.training.mapelites.select import pick

    sol, obj, meas = _fake_archive()
    keep = meas[:, 0].astype(int) != 0            # delete every scout elite
    picked = pick(sol[keep], obj[keep], meas[keep])
    assert picked[LANES[0]] == []
    assert all(picked[ln] for ln in LANES[1:])


def test_the_roster_spreads_chassis_within_every_lane():
    """A roster where every carrier is legged has quietly deleted the river."""
    from swarmmind.sim.robot import CHASSIS
    from swarmmind.training.mapelites.select import build_roster, pick

    sol, obj, meas = _fake_archive()
    specs = build_roster(sol, pick(sol, obj, meas), per_lane_count=6)
    for lane in LANES:
        used = {s["chassis"] for s in specs if s["actuator"] == lane}
        assert used == set(CHASSIS), f"{lane} uses only {used}"


def test_the_roster_round_trips_through_yaml(tmp_path):
    import yaml

    from swarmmind.sim.robot import RobotSpec
    from swarmmind.training.mapelites.select import build_roster, pick, write

    sol, obj, meas = _fake_archive()
    specs = build_roster(sol, pick(sol, obj, meas), per_lane_count=4)
    out = tmp_path / "demo_roster.yaml"
    write(specs, out, {"elites": len(sol)})
    loaded = yaml.safe_load(out.read_text())["robots"]
    assert len(loaded) == len(specs)
    assert len({r["id"] for r in loaded}) == len(loaded), "duplicate robot ids"
    for r in loaded:
        RobotSpec(**{k: v for k, v in r.items() if k != "elite"})


# ------------------------------------------------------------------ evolved roster


def test_no_archive_means_the_hand_set_archetypes_exactly(tmp_path):
    """Pre-D9 behaviour must be preserved bit for bit when no roster file exists.

    The archetypes are the gate's baseline. If loading an absent roster perturbed the
    swarm even slightly, every comparison against "the classical baseline" would be
    against something else.

    The absent path is given explicitly. This test used to call `evolved_roster` with no
    path, which reads the real `assets/scenarios/demo_roster.yaml` -- so it passed only
    while no roster had ever been generated, and started failing the moment `select.py`
    produced one. The property under test is the *fallback*, and that has to be checked
    against a path that is genuinely absent rather than against ambient repo state.
    """
    from swarmmind.sim.robot import default_roster, evolved_roster

    missing = tmp_path / "no_such_roster.yaml"
    assert not missing.exists()
    a = evolved_roster(4, np.random.default_rng(0), path=missing)
    b = default_roster(4, np.random.default_rng(0))
    assert a == b


def test_an_evolved_roster_is_used_when_present(tmp_path):
    from swarmmind.sim.robot import evolved_roster, load_roster
    from swarmmind.training.mapelites.select import build_roster, pick, write

    sol, obj, meas = _fake_archive()
    write(build_roster(sol, pick(sol, obj, meas), per_lane_count=4),
          tmp_path / "demo_roster.yaml", {})
    loaded = load_roster(tmp_path / "demo_roster.yaml")
    assert loaded and len(loaded) == 4 * len(LANES)

    swarm = evolved_roster(4, np.random.default_rng(0), tmp_path / "demo_roster.yaml")
    assert len(swarm) == 4 * len(LANES)
    assert {s.actuator for s in swarm} == set(LANES)
    assert len({s.id for s in swarm}) == len(swarm), "duplicate ids in the swarm"


def test_a_small_roster_scales_up_to_a_large_swarm(tmp_path):
    """8 stored elites have to serve a 128-per-lane demo."""
    from swarmmind.sim.robot import evolved_roster
    from swarmmind.training.mapelites.select import build_roster, pick, write

    sol, obj, meas = _fake_archive()
    write(build_roster(sol, pick(sol, obj, meas), per_lane_count=3),
          tmp_path / "demo_roster.yaml", {})
    swarm = evolved_roster(32, np.random.default_rng(0), tmp_path / "demo_roster.yaml")
    assert len(swarm) == 32 * len(LANES)
    for lane in LANES:
        chassis = {s.chassis for s in swarm if s.actuator == lane}
        from swarmmind.sim.robot import CHASSIS, CHASSIS_BARRED
        want = len([c for c in CHASSIS if c not in CHASSIS_BARRED.get(lane, frozenset())])
        assert len(chassis) == want, f"{lane} lost chassis diversity when scaled up"


def test_a_lane_evolution_never_filled_falls_back_rather_than_vanishing(tmp_path):
    """A swarm with no carriers cannot rescue anyone -- an empty lane is not an option."""
    import yaml

    from swarmmind.sim.robot import evolved_roster
    from swarmmind.training.mapelites.select import build_roster, pick, write

    sol, obj, meas = _fake_archive()
    specs = build_roster(sol, pick(sol, obj, meas), per_lane_count=3)
    specs = [s for s in specs if s["actuator"] != "gripper"]     # evolution missed carriers
    write(specs, tmp_path / "demo_roster.yaml", {})
    assert "gripper" not in {r["actuator"]
                             for r in yaml.safe_load(
                                 (tmp_path / "demo_roster.yaml").read_text())["robots"]}

    swarm = evolved_roster(4, np.random.default_rng(0), tmp_path / "demo_roster.yaml")
    grippers = [s for s in swarm if s.actuator == "gripper"]
    assert len(grippers) == 4, "the carrier lane vanished instead of falling back"
    assert len({s.chassis for s in grippers}) == 3

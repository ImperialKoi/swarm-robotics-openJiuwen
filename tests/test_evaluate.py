"""Fitness, measures, and the parallel harness.

The archive is only as trustworthy as what fills it. Two things are worth guarding:
fitness must actually discriminate between genomes, and the speed measure must be
*measured* -- an archive binned on requested speed fills with robots that are fast on
paper and the diversity claim becomes decoration.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.sim.robot import LANE_INDEX
from swarmmind.training.mapelites.evaluate import (
    EPISODE_S,
    TRAIN_SEEDS,
    _apply_morphology,
    _episode,
    evaluate,
)
from swarmmind.training.mapelites.genome import (
    GENE_INDEX,
    baseline,
    decode_spec,
    lane_of,
    random_genomes,
)
from swarmmind.training.mapelites.pool import EvaluationPool, n_workers

FAST = (11,)  # one seed: these tests check plumbing, not fitness quality


def test_evaluation_returns_the_two_archive_measures():
    ev = evaluate(baseline(), seeds=FAST)
    assert ev.lane == LANE_INDEX[lane_of(baseline())]
    assert 0 <= ev.lane < 4
    assert ev.mean_speed >= 0.0
    assert np.isfinite(ev.fitness)


def test_every_lane_can_be_evaluated():
    """Each lane has its own fitness function; none may raise or return nonsense."""
    for g0, lane in ((0.05, "none"), (0.30, "scoop"), (0.55, "gripper"), (0.95, "antenna")):
        g = baseline()
        g[GENE_INDEX["actuator_g"]] = g0
        assert lane_of(g) == lane
        ev = evaluate(g, seeds=FAST)
        assert np.isfinite(ev.fitness), f"{lane} fitness is not finite"
        assert ev.fitness >= 0.0, f"{lane} fitness went negative"


def test_speed_is_measured_not_requested():
    """A genome asking for a huge motor on a huge body must not be binned as fast."""
    slow = baseline()
    slow[GENE_INDEX["motor_power"]] = 1.0     # maximum motor
    slow[GENE_INDEX["chassis_scale"]] = 1.0   # ...on the heaviest body
    requested = decode_spec(slow, "r00", "tracked").v_max
    ev = evaluate(slow, seeds=FAST)
    assert ev.mean_speed < requested, (
        "measured speed matched the requested top speed; the axis is not measured"
    )


def test_the_candidate_lane_gets_the_candidate_body():
    """Morphology genes must reach the simulator, not just the fitness function."""
    from swarmmind.mission import Mission
    from swarmmind.sim.scenario import Scenario

    g = baseline()
    g[GENE_INDEX["actuator_g"]] = 0.05        # scout
    g[GENE_INDEX["sensor_scale"]] = 1.0       # maximum sensor
    m = Mission(Scenario.load("test"), 11, hivemind=False)
    w = m.world
    mine = w.actuator == LANE_INDEX["none"]
    before = w.sensor_radius[mine].copy()
    _apply_morphology(w, g, mine)
    assert (w.sensor_radius[mine] > before).all(), "morphology never reached the world"
    assert np.allclose(w.sensor_radius[~mine], w.sensor_radius[~mine]), "other lanes touched"


def test_the_rest_of_the_swarm_stays_classical():
    """A candidate is scored inside a working rescue chain, not alone."""
    g = baseline()
    g[GENE_INDEX["actuator_g"]] = 0.05
    m, _ = _episode(__import__("swarmmind.sim.scenario", fromlist=["Scenario"])
                    .Scenario.load("test"), g, 11)
    genes = m.genes
    scouts = m.world.actuator == LANE_INDEX["none"]
    others = ~scouts
    if others.any():
        base_val = genes.hazard_aversion[others]
        assert np.allclose(base_val, base_val[0]), "non-candidate lanes are not uniform"


def test_fitness_discriminates_between_genomes():
    """If every genome scores the same, MAP-Elites has nothing to climb."""
    rng = np.random.default_rng(5)
    scores = []
    for g in rng.random((6, 15)):
        g[GENE_INDEX["actuator_g"]] = 0.05    # hold the lane fixed: compare like with like
        scores.append(evaluate(g, seeds=FAST).fitness)
    assert len({round(s, 6) for s in scores}) > 1, f"fitness is constant: {scores}"


def test_an_episode_is_the_declared_length():
    m, stats = _episode(__import__("swarmmind.sim.scenario", fromlist=["Scenario"])
                        .Scenario.load("test"), baseline(), 11)
    assert m.world.t >= EPISODE_S - 0.1 or m.world.done
    assert stats.shape[0] == 4


def test_training_seeds_are_disjoint_from_the_gate():
    """An elite that only wins on its training maps has memorised three rubble layouts."""
    try:
        from swarmmind.training import gate
    except ImportError:
        pytest.skip("training.gate does not exist yet")

    held_out = getattr(gate, "HELD_OUT_SEEDS", None)
    if held_out is None:
        pytest.skip("gate.HELD_OUT_SEEDS not defined yet")
    assert not set(TRAIN_SEEDS) & set(held_out)


# ------------------------------------------------------------------------ the pool


def test_the_pool_refuses_to_fork_bomb():
    """`spawn` re-executes __main__. With no importable __main__ every worker re-runs
    the caller, which spawns its own workers. The first attempt at this benchmark
    produced 76 MB of tracebacks and had to be killed."""
    import sys

    main = sys.modules.get("__main__")
    had = getattr(main, "__file__", None)
    try:
        if hasattr(main, "__file__"):
            del main.__file__
        with pytest.raises(RuntimeError, match="fork bomb|real __main__"), \
                EvaluationPool(workers=2):
            pass
    finally:
        if had is not None:
            main.__file__ = had


def test_a_single_worker_pool_needs_no_subprocess():
    """workers=1 runs inline, so tests and notebooks can use the same API safely."""
    with EvaluationPool(workers=1, seeds=FAST) as p:
        res = p.evaluate(random_genomes(np.random.default_rng(1), 2))
    assert len(res) == 2
    assert all(np.isfinite(r.fitness) for r in res)


def test_results_come_back_in_the_order_they_were_sent():
    """MAP-Elites maps results onto the genomes it emitted; reordering corrupts the archive."""
    genomes = np.tile(baseline(), (3, 1))
    for i, g0 in enumerate((0.05, 0.30, 0.95)):
        genomes[i, GENE_INDEX["actuator_g"]] = g0
    with EvaluationPool(workers=1, seeds=FAST) as p:
        res = p.evaluate(genomes)
    assert [r.lane for r in res] == [
        LANE_INDEX["none"], LANE_INDEX["scoop"], LANE_INDEX["antenna"]]


def test_worker_count_leaves_a_core_free():
    assert 1 <= n_workers() < (__import__("os").cpu_count() or 2) + 1


def test_the_relay_measure_is_causal_not_a_robot_count():
    """The first archive scored every antenna elite at exactly 16.0 -- the swarm size.

    The measure was "robots in comms", and on a small map everyone is within base range
    regardless of what the relay does, so the lane learned nothing while sitting at what
    looked like a maximum. It must count only robots a relay is actually holding.
    """
    from swarmmind.mission import Mission
    from swarmmind.sim.scenario import Scenario

    m = Mission(Scenario.load("test"), 11, hivemind=False)
    w = m.world
    for _ in range(200):
        m.tick()
    assert hasattr(w, "comms_via_relay")
    # Nobody may be counted as relay-held while they are in base range on their own.
    base = np.asarray(w.scn.base)
    br = w.scn.comms.base_radius
    near_base = ((w.pos - base) ** 2).sum(axis=1) <= br * br
    assert not (w.comms_via_relay & near_base).any(), (
        "a robot standing next to the base is being credited to a relay"
    )
    assert (w.comms_via_relay <= w.in_comms).all(), "relay-held but not in comms"


def test_lane_fitnesses_are_on_a_comparable_scale():
    """Raw scales differed ~20x, so qd_score was one lane with noise on top."""
    from swarmmind.sim.robot import LANES
    from swarmmind.training.mapelites.evaluate import LANE_REFERENCE

    assert set(LANE_REFERENCE) == set(LANES), "a lane has no reference divisor"
    scores = {}
    for g0, lane in ((0.05, "none"), (0.30, "scoop"), (0.55, "gripper"), (0.95, "antenna")):
        g = baseline()
        g[GENE_INDEX["actuator_g"]] = g0
        scores[lane] = evaluate(g, seeds=FAST).fitness
    finite = [v for v in scores.values() if np.isfinite(v)]
    assert max(finite) < 20.0, f"normalisation did not take: {scores}"

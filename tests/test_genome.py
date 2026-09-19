"""The 15-gene genome and the evaluator that scores it.

The genome is the archive's contract. An archive evolved today is read back on demo day
by `select.py`, so a change to gene order or a decode range silently reinterprets every
elite ever stored -- these tests exist to make that change loud.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.mission import Mission
from swarmmind.sim.robot import CHASSIS, LANES
from swarmmind.sim.scenario import Scenario
from swarmmind.training.mapelites.genome import (
    GENE_INDEX,
    GENE_NAMES,
    N_GENES,
    RANGES,
    BehaviorParams,
    baseline,
    decode_behavior,
    decode_spec,
    lane_of,
    random_genomes,
)


def test_gene_order_is_frozen():
    """If this fails, every stored archive now decodes to different robots."""
    assert GENE_NAMES[:5] == (
        "actuator_g", "chassis_scale", "motor_power", "sensor_scale", "battery_scale")
    assert N_GENES == 15
    assert len(set(GENE_NAMES)) == N_GENES


def test_every_behaviour_gene_has_a_decode_range():
    behaviour = GENE_NAMES[5:]
    missing = [g for g in behaviour if g not in RANGES]
    assert not missing, f"genes with no decode range: {missing}"
    for name, (lo, hi) in RANGES.items():
        assert hi > lo, f"{name} range is inverted"


@pytest.mark.parametrize("g", [0.0, 0.24, 0.25, 0.5, 0.74, 0.75, 0.99, 1.0])
def test_the_actuator_gene_covers_every_lane_and_never_overflows(g):
    v = baseline()
    v[GENE_INDEX["actuator_g"]] = g
    assert lane_of(v) in LANES


def test_all_four_lanes_are_reachable():
    lanes = {lane_of(np.r_[g, baseline()[1:]]) for g in np.linspace(0, 1, 40)}
    assert lanes == set(LANES), f"unreachable lanes: {set(LANES) - lanes}"


def test_decode_is_clamped_not_wrapped():
    """Emitters propose values outside [0,1]; they must saturate, not alias."""
    low = decode_behavior(np.full(N_GENES, -5.0))
    high = decode_behavior(np.full(N_GENES, 5.0))
    for name in low.__dataclass_fields__:
        lo, hi = RANGES[name]
        assert getattr(low, name) == pytest.approx(lo)
        assert getattr(high, name) == pytest.approx(hi)


def test_the_midpoint_genome_is_near_the_hand_tuned_archetypes():
    """Not identical -- see the module docstring -- but in the same neighbourhood."""
    spec = decode_spec(baseline(), "r00", "tracked")
    assert 0.2 < spec.radius < 0.5
    assert 0.8 < spec.v_max < 1.6
    assert 3.5 < spec.sensor_radius < 7.0
    assert 0.9 < spec.battery_capacity < 1.5


def test_a_bigger_body_is_a_slower_robot():
    """Without a cost, every elite is max-motor max-battery and the archive collapses."""
    small, big = baseline(), baseline()
    small[GENE_INDEX["chassis_scale"]] = 0.0
    big[GENE_INDEX["chassis_scale"]] = 1.0
    a = decode_spec(small, "r00", "tracked")
    b = decode_spec(big, "r00", "tracked")
    assert b.radius > a.radius
    assert b.v_max < a.v_max, "size carries no speed penalty; the trade-off is absent"
    assert b.omega_max < a.omega_max


def test_chassis_speed_multiplier_is_applied():
    g = baseline()
    speeds = {c: decode_spec(g, "r00", c).v_max for c in CHASSIS}
    assert speeds["wheeled"] > speeds["tracked"] > speeds["legged"]


def test_behaviour_params_are_arrays_not_a_python_loop():
    """768 robots bid in one vectorised expression; per-robot genes must broadcast."""
    genomes = random_genomes(np.random.default_rng(0), 64)
    p = BehaviorParams(genomes)
    assert len(p) == 64
    for name in ("hazard_aversion", "formation_spacing", "comms_tether"):
        arr = getattr(p, name)
        assert isinstance(arr, np.ndarray) and arr.shape == (64,)
        lo, hi = RANGES[name]
        assert arr.min() >= lo - 1e-9 and arr.max() <= hi + 1e-9


def test_uniform_applies_one_genome_to_the_whole_swarm():
    p = BehaviorParams.uniform(8)
    assert len(p) == 8
    assert np.allclose(p.hazard_aversion, p.hazard_aversion[0])


# --------------------------------------------------------------- genes in the loop


def _genes_for(scn, genome):
    n = Mission(scn, 42, hivemind=False).world.n
    return BehaviorParams(np.tile(genome, (n, 1)))


def test_a_mission_runs_with_evolved_genes():
    scn = Scenario.load("test")
    g = _genes_for(scn, baseline())
    card = Mission(scn, 42, hivemind=False, genes=g).run(max_time=90.0)
    assert card.ground_explored_frac > 0.0


def test_genes_change_behaviour():
    """A gene that alters nothing measurable is not a gene.

    `distance_penalty` rather than `hazard_aversion`: hazard aversion multiplies *observed*
    hazard, so before the swarm has seen the fire it multiplies zero and the two arms are
    byte-identical. That is correct behaviour and a useless test -- it passed only by
    incidental trajectory drift, and broke the moment relay allocation changed.
    `distance_penalty` scales the travel term in every bid unconditionally.
    """
    scn = Scenario.load("test")
    near, far = baseline(), baseline()
    near[GENE_INDEX["distance_penalty"]] = 0.0
    far[GENE_INDEX["distance_penalty"]] = 1.0
    a = Mission(scn, 42, hivemind=False, genes=_genes_for(scn, near)).run(max_time=120.0)
    b = Mission(scn, 42, hivemind=False, genes=_genes_for(scn, far)).run(max_time=120.0)
    assert a.hash() != b.hash(), "distance_penalty had no effect on the mission at all"


def test_hazard_aversion_bites_once_the_hazard_is_known():
    """The conditional gene, tested under the condition it needs."""
    import numpy as np

    scn = Scenario.load("test")
    timid, bold = baseline(), baseline()
    timid[GENE_INDEX["hazard_aversion"]] = 1.0
    bold[GENE_INDEX["hazard_aversion"]] = 0.0
    cards = []
    for g in (timid, bold):
        m = Mission(scn, 42, hivemind=False, genes=_genes_for(scn, g))
        for _ in range(400):
            m.tick()
        # Plant observed hazard over one half of the map. It has to be a *region*: a
        # uniform hazard field adds the same value to every robot's bid, which shifts
        # every bid equally and therefore changes no ranking at all.
        w = m.world
        half = w.shape[1] // 2
        w.hazard_known[:, half:] = w.passable[:, half:]
        cards.append(m.run(max_time=300.0))
    assert np.isfinite(cards[0].energy_used)
    assert cards[0].hash() != cards[1].hash(), "hazard_aversion does nothing when known"


def test_no_genome_is_byte_identical_to_the_pre_genome_auction():
    """`genes=None` is the gate's control arm; it must stay the classical baseline."""
    scn = Scenario.load("test")
    a = Mission(scn, 42, hivemind=False).run(max_time=120.0)
    b = Mission(scn, 42, hivemind=False, genes=None).run(max_time=120.0)
    assert a.hash() == b.hash()


def test_evolved_missions_are_still_deterministic():
    scn = Scenario.load("test")
    g = random_genomes(np.random.default_rng(3), 1)[0]
    a = Mission(scn, 42, hivemind=False, genes=_genes_for(scn, g)).run(max_time=90.0)
    b = Mission(scn, 42, hivemind=False, genes=_genes_for(scn, g)).run(max_time=90.0)
    assert a.hash() == b.hash()

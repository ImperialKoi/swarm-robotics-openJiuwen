"""Tier 3a: the commander, and the training that is meant to make it worth having.

The commander currently *loses* to having no commander at all, which is exactly why these
tests are about the machinery and the gate rather than about performance. What must hold
is that training can run, that it is judged on maps it never saw, and that an untrained
commander cannot quietly ship.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.mission import Mission
from swarmmind.nodes.command import Commander, CommandParams
from swarmmind.sim.scenario import Scenario
from swarmmind.training.command.evaluate import (
    HELD_OUT_MAP_SEEDS,
    TRAIN_MAP_SEEDS,
    _shrink,
)
from swarmmind.training.command.run import BOUNDS, N_PARAMS, decode, encode
from swarmmind.training.command.tripwire import EDGE_HI, Tripwires


class _StubTracker:
    """Just enough tracker for the commander: a list of open reports with positions."""

    def __init__(self, positions):
        self._positions = [np.asarray(p, dtype=float) for p in positions]

    def open_reports(self):
        return [type("R", (), {"pos": p})() for p in self._positions]


def _mission(**kw):
    return Mission(Scenario.load("test"), 42, hivemind=False, **kw)


# ------------------------------------------------------------------ the commander


def test_no_commander_is_the_default():
    """It loses to no commander at all today. Shipping it by default would be the
    mistake the gate exists to prevent."""
    assert _mission().commander is None
    assert _mission(command=True).commander is not None


def test_every_robot_belongs_to_exactly_one_squadron():
    m = _mission(command=True)
    c = m.commander
    assert len(c.squadron) == m.world.n
    assert c.squadron.min() >= 0 and c.squadron.max() < c.n_squadrons


def test_squadrons_get_a_mix_of_lanes():
    """A squadron of nothing but carriers cannot search its own territory."""
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    w, c = m.world, m.commander
    for s in range(c.n_squadrons):
        lanes = set(w.actuator[c.squadron == s].tolist())
        assert len(lanes) >= 3, f"squadron {s} has only lanes {lanes}"


def test_territory_covers_the_map_without_overlap():
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    c = m.commander
    owned = c.territory[c.territory >= 0]
    assert len(owned) > 0
    assert owned.max() < c.n_squadrons


def test_rescue_work_is_never_fenced():
    """A casualty two metres over a wedge boundary is still everybody's business."""
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    w, c = m.world, m.commander
    target = (w.scn.map.width_m * 0.5, w.scn.map.height_m * 0.5)
    mask = c.in_own_territory(w, target)
    assert mask.sum() < w.n, "the fence lets everyone through, so it is not a fence"
    # ...but the auction only applies it to search kinds.
    from swarmmind.nodes.auction import AuctionNode

    src = AuctionNode._best_bidder.__doc__ or ""
    assert "extract" not in src.lower()


def test_a_fence_never_idles_the_whole_swarm():
    """If nobody in the owning squadron can take a task, anyone may."""
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    w, c = m.world, m.commander
    c.territory[:] = -1                                   # nobody owns anything
    assert c.in_own_territory(w, (10.0, 10.0)).all()


# ---------------------------------------------------------------- parameterisation


def test_params_round_trip():
    p = CommandParams()
    assert CommandParams.from_vector(p.as_vector()) == p
    assert len(p.as_vector()) == N_PARAMS


def test_the_search_cube_decodes_inside_the_bounds():
    for x in (np.zeros(N_PARAMS), np.ones(N_PARAMS), np.full(N_PARAMS, 0.5)):
        v = decode(x).as_vector()
        for got, (lo, hi) in zip(v, BOUNDS, strict=True):
            assert lo - 1e-9 <= got <= hi + 1e-9


def test_out_of_range_proposals_are_clamped_not_wrapped():
    """CMA-ES proposes outside the cube; that must saturate, not alias to the far side."""
    lo = decode(np.full(N_PARAMS, -3.0)).as_vector()
    hi = decode(np.full(N_PARAMS, 4.0)).as_vector()
    assert np.allclose(lo, [b[0] for b in BOUNDS])
    assert np.allclose(hi, [b[1] for b in BOUNDS])


def test_the_baseline_sits_inside_the_search_space():
    """Training starts from the hand-set commander; it has to be representable.

    Compared with a tolerance, not `==`. `encode` divides by the bound width and `decode`
    multiplies it back, which is not bit-exact for every bound: widening `done_frac` to
    (0.20, 0.99) made 0.85 round-trip to 0.8499999999999999. That is float arithmetic
    doing what float arithmetic does, not the baseline falling outside the cube, and an
    exact assertion here only ever held by coincidence of the old bounds.
    """
    x = encode(CommandParams())
    assert (x >= -1e-9).all() and (x <= 1 + 1e-9).all(), x
    assert decode(x).as_vector() == pytest.approx(CommandParams().as_vector())


def test_w_contacts_actually_changes_the_cut():
    """Regression: `contacts` was `np.zeros(...)` and never filled, so `w_contacts`
    multiplied zero. Run 1 spent 121 generations tuning a parameter that did nothing."""
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    w = m.world
    # Reports clustered in one corner, so the contact signal is not uniform across
    # sectors -- a uniform signal would be normalised flat and change no cut.
    wm, hm = w.shape[1] * w.cell, w.shape[0] * w.cell
    x1, y1 = wm * 0.85, hm * 0.85
    tracker = _StubTracker([(x1, y1), (x1 + 1.0, y1), (x1, y1 + 1.0)])

    off = Commander(w, params=CommandParams(w_unexplored=1.0, w_contacts=0.0),
                    tracker=tracker)
    on = Commander(w, params=CommandParams(w_unexplored=1.0, w_contacts=4.0),
                   tracker=tracker)
    assert not np.array_equal(off.territory, on.territory), \
        "w_contacts changed no territory -- the contacts term is dead again"


def test_contacts_are_normalised_and_need_no_tracker():
    """In [0, 1] like the other three terms, and absent tracker means no signal."""
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    w = m.world
    n = len(w.sector_ids)

    bare = Commander(w, params=CommandParams(), tracker=None)
    assert np.array_equal(bare._contacts(w, n), np.zeros(n))

    c = Commander(w, params=CommandParams(),
                  tracker=_StubTracker([(w.shape[1] * w.cell * 0.5,
                                         w.shape[0] * w.cell * 0.5)]))
    v = c._contacts(w, n)
    assert v.min() >= 0.0 and v.max() == pytest.approx(1.0)


def test_tripwire_catches_training_rising_while_held_out_does_not():
    """Run 1's failure. Train climbs, held-out does not follow -> the objective is being
    gamed rather than solved."""
    wires = Tripwires(param_names=("a", "b"))
    for gen, train, held in [(10, 10.0, 5.0), (20, 20.0, 5.0), (30, 30.0, 4.0)]:
        wires.record_held_out(gen, train, held)
    wires.last_improved_gen = 30
    names = [t.name for t in wires.check(30, np.array([0.5, 0.5]))]
    assert "reward-hacking" in names

    # Held-out rising with training is healthy and must stay silent.
    ok = Tripwires(param_names=("a", "b"))
    for gen, train, held in [(10, 10.0, 5.0), (20, 20.0, 9.0), (30, 30.0, 14.0)]:
        ok.record_held_out(gen, train, held)
    ok.last_improved_gen = 30
    assert [t.name for t in ok.check(30, np.array([0.5, 0.5]))] == []


def test_tripwire_catches_a_FLAT_held_out_series_not_only_a_falling_one():
    """Regression, from run 2's real numbers.

    The first version tested `held_out_slope <= 0` and stayed silent through exactly the
    divergence it existed to catch: training gained +3.66 while held-out moved 0.00, but
    the flat series' slope was +0.0028 -- noise, and technically above zero. Flat is not
    negative, and the threshold has to be relative to the training slope.
    """
    wires = Tripwires(param_names=("a",))
    wires.last_improved_gen = 40
    for gen, train, held in [(10, -79.51, -95.02), (20, -78.37, -95.30),
                             (30, -75.85, -95.02), (40, -75.85, -95.02)]:
        wires.record_held_out(gen, train, held)
    trips = [t for t in wires.check(40, np.array([0.5])) if t.name == "reward-hacking"]
    assert trips, "a flat held-out series against rising training must fire"
    assert "capturing" in trips[0].detail


def test_tripwire_catches_a_bound_choosing_the_answer():
    """Run 1 settled squadron_size at 155.4 against a ceiling of 160 and called it a
    result."""
    wires = Tripwires(param_names=("squadron_size", "rebalance_s"))
    wires.last_improved_gen = 5
    wires.has_search_result = True
    trips = wires.check(5, np.array([0.995, 0.5]))
    assert [t.name for t in trips] == ["bound-pinned"]
    assert "squadron_size" in trips[0].detail
    # Comfortably inside the cube is silent.
    assert wires.check(5, np.array([EDGE_HI - 0.1, 0.5])) == []


def test_bound_tripwire_stays_quiet_until_the_search_beats_the_baseline():
    """`CommandParams` defaults three weights to exactly 0.0 -- the low bound. Before the
    first improvement `best_x` is that baseline, and reporting it as a pinned bound is a
    false positive (observed in a smoke run)."""
    wires = Tripwires(param_names=("w_contacts", "w_hazard", "w_distance"))
    wires.last_improved_gen = 2
    at_baseline = np.array([0.0, 0.0, 0.0])
    assert wires.check(2, at_baseline) == []
    wires.has_search_result = True
    assert [t.name for t in wires.check(2, at_baseline)] == ["bound-pinned"]


def test_reward_hacking_wire_stays_quiet_when_training_is_also_flat():
    """Regression, from run 3.

    The run plateaued so hard that all four training samples and all four held-out
    samples were identical. `np.polyfit` on constant data returns ~1e-16 rather than an
    exact zero, `+1e-16 > 0.0` is true, and the wire fired ten times reporting "training
    rising +0.000/gen". Flat training is not rising training; there is no divergence to
    report when nothing is diverging.
    """
    wires = Tripwires(param_names=("a",))
    wires.last_improved_gen = 40
    for gen in (10, 20, 30, 40):
        wires.record_held_out(gen, -76.84, -94.69)      # byte-identical, as observed
    assert [t.name for t in wires.check(40, np.array([0.5]))] == []


def test_tripwire_catches_a_stalled_search():
    wires = Tripwires(param_names=("a",), stall_generations=20)
    wires.last_improved_gen = 3
    assert [t.name for t in wires.check(10, np.array([0.5]))] == []
    assert "stalled" in [t.name for t in wires.check(40, np.array([0.5]))]


def test_command_score_is_scale_free_across_maps():
    """A 60-casualty map must not outvote a 10-casualty one purely on size."""
    from swarmmind.training.command.evaluate import command_score

    class _V:
        def __init__(self, pos, state):
            self.pos, self.state = np.asarray(pos, float), state

    class _W:
        def __init__(self, n, rescued):
            self.scn = type("S", (), {"base": (0.0, 0.0)})()
            self.victims = [_V((10.0 * (i + 1), 0.0), 4 if i < rescued else 0)
                            for i in range(n)]

    card = type("C", (), {"victims_found": 0, "victims_total": 1,
                          "ground_explored_frac": 0.0, "robots_lost": 0,
                          "robots_total": 10})()
    # Same *fraction* rescued on a small and a large map -> comparable scores.
    small = command_score(_W(10, 5), card)
    large = command_score(_W(60, 30), card)
    assert small == pytest.approx(large, abs=6.0)
    # And everyone home beats nobody home, on either size.
    assert command_score(_W(10, 10), card) > command_score(_W(10, 0), card)


def test_params_actually_change_the_cut():
    """A parameter that changes no territory is not a parameter."""
    m = Mission(Scenario.load("demo"), 42, hivemind=False, command=True)
    w = m.world
    a = Commander(w, params=CommandParams(w_unexplored=1.0, w_distance=0.0))
    b = Commander(w, params=CommandParams(w_unexplored=1.0, w_distance=3.0))
    assert not np.array_equal(a.territory, b.territory)


# ------------------------------------------------------------------------- gate


def test_training_and_gate_maps_are_disjoint():
    """A commander that only wins on its three training maps has memorised three maps."""
    assert not set(TRAIN_MAP_SEEDS) & set(HELD_OUT_MAP_SEEDS)
    assert len(HELD_OUT_MAP_SEEDS) >= 10


def test_the_gate_compares_against_no_commander_not_just_the_heuristic():
    """The heuristic currently scores worse than nothing, so beating it proves nothing."""
    from swarmmind.training.command import gate

    doc = gate.__doc__ or ""
    assert "none" in doc
    src = gate.main.__code__.co_consts
    assert any(isinstance(c, str) and c == "none" for c in src), \
        "the gate must run a no-commander arm"


def test_training_scenarios_are_shrunk_for_speed():
    from swarmmind.sim.generator import sample
    from swarmmind.training.command.evaluate import TRAIN_MAX_ROBOTS

    big = sample(2001)
    small = _shrink(big)
    assert small.n_robots <= max(TRAIN_MAX_ROBOTS, 16) * 1.3
    assert set(small.lane_counts) == set(big.lane_counts)
    assert all(v > 0 for v in small.lane_counts.values())


@pytest.mark.parametrize("seed", [1001])
def test_one_evaluation_produces_a_finite_score(seed):
    from swarmmind.training.command.evaluate import evaluate

    ev = evaluate(CommandParams(), (seed,))
    assert np.isfinite(ev.score)
    assert 0.0 <= ev.rescued <= 1.0
    assert 0.0 <= ev.explored <= 1.0
    assert len(ev.per_map) == 1


# ----------------------------------------------------------------- checkpointing


def test_the_search_state_survives_a_round_trip(tmp_path):
    """Resume has to restore CMA-ES itself, not just the best answer.

    Saving only `best.npz` and reloading it would restart the search from the hand-set
    baseline with a fresh covariance matrix -- a night's work thrown away, silently, and
    the run would then overwrite the good result with its worse fresh one.
    """
    from swarmmind.training.command.run import build, load, save

    sched = build(7)
    xs = sched.ask()
    sched.tell(np.linspace(1.0, 2.0, len(xs)), np.full((len(xs), 2), 0.5))

    # Save with no ask outstanding -- pyribs forbids two asks without a tell between,
    # and that is also the only point the real loop ever checkpoints from.
    save(tmp_path, sched, xs[0], 12.5, {"generation": 3, "baseline": 9.0})
    state = load(tmp_path)
    assert state is not None
    assert state["gen"] == 3 and state["best"] == 12.5 and state["baseline"] == 9.0
    assert np.allclose(state["sched"].ask(), sched.ask()), (
        "the restored scheduler proposes a different batch -- the search state was lost"
    )


def test_load_returns_none_with_no_checkpoint(tmp_path):
    from swarmmind.training.command.run import load

    assert load(tmp_path) is None


def test_the_saved_best_decodes_to_usable_params(tmp_path):
    """`gate.py` reads best.npz; it has to come back as a commander."""
    from swarmmind.training.command.gate import load_trained
    from swarmmind.training.command.run import build, encode, save

    sched = build(1)
    x = encode(CommandParams(w_distance=2.0))
    save(tmp_path, sched, x, 1.0, {"generation": 1})
    p = load_trained(tmp_path)
    assert isinstance(p, CommandParams)
    assert abs(p.w_distance - 2.0) < 1e-6


def test_resume_is_the_default_and_restart_is_explicit():
    """Losing a night's search to a forgotten flag is the failure this prevents."""
    import inspect

    from swarmmind.training.command import run

    src = inspect.getsource(run.main)
    assert "args.restart" in src
    assert "if args.restart else load" in src.replace("None ", "")

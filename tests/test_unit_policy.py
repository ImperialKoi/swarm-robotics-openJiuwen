"""The unit policy layer: inert when off, legal when on, deterministic either way."""

from __future__ import annotations

import numpy as np

from swarmmind.control import unit_policy as up
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario
from swarmmind.training.rl.ppo import UnitPolicy

SHORT = 60.0


def _run(controller, seconds=SHORT, seed=42):
    m = Mission(Scenario.load("test"), seed, hivemind=False, unit_policy=controller)
    return m, m.run(seconds)


def test_default_mode_is_byte_identical_to_the_shipped_swarm():
    _, off = _run(None)
    _, default = _run(up.UnitController("default"))
    assert off.hash() == default.hash()


def test_learned_mode_is_deterministic_without_a_sampling_rng():
    pol = UnitPolicy(up.CAND_DIM, up.CTX_DIM, seed=3)
    _, a = _run(up.UnitController("learned", pol))
    _, b = _run(up.UnitController("learned", pol))
    assert a.hash() == b.hash()


class _Spy:
    def __init__(self):
        self.rows = 0
        self.bad = 0
        self.finite = True

    def on_decisions(self, t, idx, c, action, logp):
        self.rows += len(idx)
        self.bad += int((~c.mask[np.arange(len(idx)), action]).sum())
        self.finite &= bool(np.isfinite(c.cand).all() and np.isfinite(c.ctx).all())
        assert c.cand.shape == (len(idx), up.K, up.CAND_DIM)
        assert c.ctx.shape == (len(idx), up.CTX_DIM)
        assert c.mask[:, up.DEFAULT].all()


def test_sampled_policy_only_chooses_offered_candidates_and_features_are_finite():
    spy = _Spy()
    pol = UnitPolicy(up.CAND_DIM, up.CTX_DIM, seed=5)
    ctl = up.UnitController("learned", pol, rng=np.random.default_rng(0), recorder=spy)
    m, _ = _run(ctl, seconds=120.0)
    assert spy.rows > 0
    assert spy.bad == 0
    assert spy.finite
    assert ctl.counts[1:].sum() > 0, "a random policy should leave the default sometimes"


def test_a_staged_robot_is_never_given_search_work():
    pol = UnitPolicy(up.CAND_DIM, up.CTX_DIM, seed=7)
    ctl = up.UnitController("learned", pol, rng=np.random.default_rng(1))
    m = Mission(Scenario.load("test"), 42, hivemind=False, unit_policy=ctl)
    w = m.world
    violations = 0
    while w.t < 120.0:
        m.tick()
        for i in m.executor.policy_goal:
            a = m.executor.assignment[i]
            if a is not None and a.kind in ("explore", "investigate", "relay"):
                violations += 1
    assert violations == 0


def test_heuristic_mode_runs_and_only_stages_carriers_and_diggers():
    m = Mission(Scenario.load("test"), 42, hivemind=False,
                unit_policy=up.UnitController("heuristic"))
    w = m.world
    staged_lanes = set()
    while w.t < 200.0:
        m.tick()
        for i in m.executor.policy_goal:
            staged_lanes.add(int(w.actuator[i]))
    assert staged_lanes <= {up.GRIP, up.SCOOP}

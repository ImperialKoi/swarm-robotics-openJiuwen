"""One training episode end to end on the tiny fixture: record, reward, batch."""

from __future__ import annotations

import numpy as np

from swarmmind.control import unit_policy as up
from swarmmind.training.rl.ppo import PPO, PPOConfig, UnitPolicy
from swarmmind.training.rl.rollout import run_episode, to_batch


def test_episode_records_decisions_and_builds_a_valid_batch():
    pol = UnitPolicy(up.CAND_DIM, up.CTX_DIM, seed=0)
    ep = run_episode(pol.state(), seed=11, scenario="test", max_time=150.0)
    assert len(ep.action) > 0
    assert ep.cand.shape[1:] == (up.K, up.CAND_DIM)
    assert ep.mask[np.arange(len(ep.action)), ep.action].all()
    b = to_batch([ep], pol, gamma=0.995, lam=0.95)
    assert len(b) == len(ep.action)
    assert np.isfinite(b.adv).all() and np.isfinite(b.ret).all()
    stats = PPO(pol, PPOConfig(minibatch=256, epochs=1)).update(b, np.random.default_rng(0))
    assert np.isfinite(stats["pi_loss"]) and np.isfinite(stats["vf_loss"])


def test_episode_is_reproducible_for_a_seed():
    pol = UnitPolicy(up.CAND_DIM, up.CTX_DIM, seed=1)
    a = run_episode(pol.state(), seed=12, scenario="test", max_time=60.0)
    b = run_episode(pol.state(), seed=12, scenario="test", max_time=60.0)
    assert np.array_equal(a.action, b.action)
    assert a.card == b.card

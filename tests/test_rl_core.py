"""Hand-written gradients are only trustworthy once checked against finite differences."""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.training.rl.nn import MLP, Adam, clip_grad_norm
from swarmmind.training.rl.ppo import (
    PPO,
    Batch,
    PPOConfig,
    UnitPolicy,
    bc_loss_and_grad,
    masked_log_softmax,
    policy_loss_and_grad,
    smdp_gae,
    value_loss_and_grad,
)

EPS = 1e-6


def _numeric(fn, params):
    out = []
    for p in params:
        g = np.zeros_like(p)
        it = np.nditer(p, flags=["multi_index"])
        for _ in it:
            ix = it.multi_index
            old = p[ix]
            p[ix] = old + EPS
            hi = fn()
            p[ix] = old - EPS
            lo = fn()
            p[ix] = old
            g[ix] = (hi - lo) / (2 * EPS)
        out.append(g)
    return out


def _close(a, b):
    for x, y in zip(a, b, strict=True):
        np.testing.assert_allclose(x, y, rtol=1e-5, atol=1e-7)


def test_mlp_backward_matches_finite_differences_with_leading_dims():
    rng = np.random.default_rng(0)
    net = MLP((5, 7, 6, 3), rng, out_gain=0.5)
    x = rng.standard_normal((4, 3, 5))
    w = rng.standard_normal((4, 3, 3))          # loss = sum(w * out)
    out, cache = net.forward(x)
    analytic = net.backward(cache, w)
    numeric = _numeric(lambda: float((w * net(x)).sum()), net.params())
    _close(analytic, numeric)


def _batch(rng, B=6, K=4, F=5, G=3):
    mask = rng.random((B, K)) < 0.7
    mask[:, 0] = True                           # candidate 0 is always offered
    return Batch(
        cand=rng.standard_normal((B, K, F)), mask=mask, ctx=rng.standard_normal((B, G)),
        action=np.array([int(rng.choice(np.nonzero(m)[0])) for m in mask]),
        logp=np.log(rng.uniform(0.1, 0.9, B)), adv=rng.standard_normal(B),
        ret=rng.standard_normal(B),
    )


@pytest.mark.parametrize("clip", [0.2, 10.0])   # clip-active and clip-inactive regimes
def test_policy_gradient_matches_finite_differences(clip):
    rng = np.random.default_rng(1)
    pol = UnitPolicy(5, 3, hidden=8, seed=2)
    for p in pol.pi.params():                   # a non-trivial policy, not near-uniform
        p += rng.standard_normal(p.shape) * 0.5
    b = _batch(rng)
    cfg = PPOConfig(clip=clip, ent_coef=0.05)
    _, analytic = policy_loss_and_grad(pol, b, cfg)
    numeric = _numeric(lambda: policy_loss_and_grad(pol, b, cfg)[0]["pi_loss"],
                       pol.pi.params())
    _close(analytic, numeric)


def test_behaviour_cloning_gradient_matches_finite_differences():
    rng = np.random.default_rng(11)
    pol = UnitPolicy(5, 3, hidden=8, seed=12)
    b = _batch(rng)
    _, analytic = bc_loss_and_grad(pol, b)
    numeric = _numeric(lambda: bc_loss_and_grad(pol, b)[0]["bc_loss"], pol.pi.params())
    _close(analytic, numeric)


def test_value_gradient_matches_finite_differences():
    rng = np.random.default_rng(3)
    pol = UnitPolicy(5, 3, hidden=8, seed=4)
    b = _batch(rng)
    cfg = PPOConfig()
    _, analytic = value_loss_and_grad(pol, b, cfg)
    numeric = _numeric(lambda: value_loss_and_grad(pol, b, cfg)[0]["vf_loss"],
                       pol.vf.params())
    _close(analytic, numeric)


def test_masked_candidates_are_never_chosen_and_argmax_breaks_ties_to_zero():
    rng = np.random.default_rng(5)
    pol = UnitPolicy(5, 3, hidden=8, seed=6)
    cand = rng.standard_normal((500, 4, 5))
    mask = np.zeros((500, 4), dtype=bool)
    mask[:, 0] = True
    mask[::2, 3] = True
    a, logp = pol.act(cand, mask, rng)
    assert mask[np.arange(500), a].all()
    assert np.isfinite(logp).all()
    # Identical candidates: the deterministic policy must pick candidate 0.
    same = np.repeat(cand[:, :1], 4, axis=1)
    a0, _ = pol.act(same, np.ones((500, 4), dtype=bool))
    assert (a0 == 0).all()


def test_log_softmax_has_no_nans_with_masked_rows():
    z = np.array([[1.0, 2.0, 3.0], [0.0, 0.0, 0.0]])
    m = np.array([[True, False, True], [True, False, False]])
    out = masked_log_softmax(z, m)
    assert np.isfinite(out[m]).all()
    np.testing.assert_allclose(np.exp(out[0, [0, 2]]).sum(), 1.0)
    np.testing.assert_allclose(out[1, 0], 0.0)


def test_smdp_gae_reduces_to_discounted_return_at_lambda_one():
    t = np.array([0.0, 3.0, 10.0])
    rew = np.array([1.0, 0.0, 2.0])
    val = np.zeros(3)
    g = 0.9
    adv, ret = smdp_gae(t, rew, val, terminal_t=12.0, gamma=g, lam=1.0)
    expect0 = 1.0 + g ** 3 * (0.0 + g ** 7 * 2.0)
    np.testing.assert_allclose(ret[0], expect0)
    np.testing.assert_allclose(ret[2], 2.0)


def test_ppo_update_raises_the_probability_of_advantaged_actions():
    rng = np.random.default_rng(7)
    pol = UnitPolicy(5, 3, hidden=16, seed=8)
    B, K = 2048, 4
    cand = rng.standard_normal((B, K, 5))
    mask = np.ones((B, K), dtype=bool)
    # Candidate whose feature 0 is largest is the good one.
    good = np.argmax(cand[..., 0], axis=1)
    before = float(np.mean(np.argmax(pol.logits(cand), axis=1) == good))
    ppo = PPO(pol, PPOConfig(lr=3e-3, minibatch=512, epochs=4, target_kl=10.0))
    for _ in range(15):
        a, logp = pol.act(cand, mask, rng)
        adv = np.where(a == good, 1.0, -0.3)
        ppo.update(Batch(cand, mask, rng.standard_normal((B, 3)), a, logp, adv,
                         np.zeros(B)), rng)
    after = float(np.mean(np.argmax(pol.logits(cand), axis=1) == good))
    assert before < 0.5 < after, (before, after)


def test_adam_and_clip_behave():
    p = [np.array([1.0, -2.0])]
    opt = Adam(p, lr=0.1)
    for _ in range(200):
        opt.step(p, [2.0 * p[0]])               # minimise x^2
    assert np.abs(p[0]).max() < 0.05
    g = [np.array([3.0, 4.0])]
    assert clip_grad_norm(g, 1.0) == pytest.approx(5.0)
    assert np.linalg.norm(g[0]) == pytest.approx(1.0)


def test_policy_state_round_trips():
    pol = UnitPolicy(5, 3, hidden=8, seed=9)
    clone = UnitPolicy.from_state(pol.state())
    x = np.random.default_rng(0).standard_normal((3, 4, 5))
    np.testing.assert_array_equal(pol.logits(x), clone.logits(x))


def test_policy_npz_round_trips(tmp_path):
    pol = UnitPolicy(5, 3, hidden=8, seed=10)
    pol.save_npz(tmp_path / "p.npz")
    clone = UnitPolicy.load_npz(tmp_path / "p.npz")
    x = np.random.default_rng(1).standard_normal((3, 4, 5))
    np.testing.assert_array_equal(pol.logits(x), clone.logits(x))
    np.testing.assert_array_equal(pol.value(x[:, 0, :3]), clone.value(x[:, 0, :3]))

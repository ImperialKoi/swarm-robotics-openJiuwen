"""PPO for a shared unit policy that picks one of K candidate goals.

**The policy scores candidates; it does not emit coordinates.** Each robot is offered a
short list of places it could be -- the auction's own target first, then nearby contacts
and rescue sites -- and one network shared by every robot scores each `(robot, candidate)`
pair. That shape is chosen for three reasons:

* **Every choice is feasible by construction.** The candidates are built from what the
  swarm knows and already filtered for reachability, so the policy can be wrong about
  value but never about geometry. It cannot drive anything into a wall -- and Tier 1's
  override would stop it if it could (invariant #2).
* **Candidate 0 is always the heuristic.** A policy that always picks it reproduces the
  shipped swarm byte for byte, so training starts from the classical behaviour and the
  gate's control arm is the same code with the policy switched off.
* **It is permutation-invariant and independent of N.** 512 robots are 512 rows through
  the same weights, which is what lets one policy be trained on every robot's experience.

Advantages use a semi-Markov GAE: decisions are not evenly spaced (a robot decides when
it becomes free, which may be one second or forty later), so discounting is per *second*
of elapsed time rather than per step, while lambda is applied per decision.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .nn import MLP, Adam, clip_grad_norm

#: Masked logits are set here rather than to -inf: -inf - -inf is nan, and a nan in one
#: row of a minibatch poisons every gradient in it.
MASKED = -1e9


def masked_log_softmax(logits: np.ndarray, mask: np.ndarray) -> np.ndarray:
    z = np.where(mask, logits, MASKED)
    z = z - z.max(axis=-1, keepdims=True)
    lse = np.log(np.exp(z).sum(axis=-1, keepdims=True))
    return np.where(mask, z - lse, MASKED)


@dataclass(frozen=True)
class PPOConfig:
    clip: float = 0.2
    lr: float = 3e-4
    epochs: int = 4
    minibatch: int = 4096
    ent_coef: float = 0.01
    vf_coef: float = 0.5
    max_grad_norm: float = 0.5
    #: Per second. 0.995 is a ~200 s horizon: long enough for a find to be credited with
    #: the rescue it leads to (mean time to rescue is ~150-200 s on the demo map).
    gamma: float = 0.995
    #: Per decision.
    lam: float = 0.95
    #: Stop this update's remaining epochs once the policy has moved this far. Cheap
    #: insurance against one bad batch undoing a night of training.
    target_kl: float = 0.03


class UnitPolicy:
    """Candidate scorer plus value head. Two separate MLPs: sharing a trunk couples the
    value loss's scale to the policy's features, which is not worth it at this size."""

    def __init__(self, cand_dim: int, ctx_dim: int, hidden: int = 64, seed: int = 0) -> None:
        rng = np.random.default_rng(seed)
        self.cand_dim, self.ctx_dim, self.hidden = cand_dim, ctx_dim, hidden
        self.pi = MLP((cand_dim, hidden, hidden, 1), rng, out_gain=0.01)
        self.vf = MLP((ctx_dim, hidden, hidden, 1), rng, out_gain=1.0)

    # ------------------------------------------------------------------ acting

    def logits(self, cand: np.ndarray) -> np.ndarray:
        return self.pi(cand)[..., 0]

    def act(self, cand: np.ndarray, mask: np.ndarray,
            rng: np.random.Generator | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(action, log-prob) per row. `rng=None` is the deployed, deterministic policy:
        argmax, ties broken toward the lowest index -- which is candidate 0, the heuristic."""
        logp = masked_log_softmax(self.logits(cand), mask)
        if rng is None:
            a = np.argmax(logp, axis=-1)
        else:
            # Inverse-CDF sampling with one uniform per row: a fixed number of draws per
            # decision, so the RNG stream does not depend on how many candidates were valid.
            p = np.exp(logp) * mask
            c = np.cumsum(p, axis=-1)
            u = rng.random(len(c))[:, None] * c[:, -1:]
            a = np.minimum((c < u).sum(axis=-1), mask.shape[-1] - 1)
        return a.astype(np.int64), logp[np.arange(len(a)), a]

    def value(self, ctx: np.ndarray) -> np.ndarray:
        return self.vf(ctx)[..., 0]

    # ------------------------------------------------------------------ persistence

    def state(self) -> dict:
        return {"cand_dim": self.cand_dim, "ctx_dim": self.ctx_dim, "hidden": self.hidden,
                "pi": [p.copy() for p in self.pi.params()],
                "vf": [p.copy() for p in self.vf.params()]}

    @classmethod
    def from_state(cls, s: dict) -> UnitPolicy:
        pol = cls(s["cand_dim"], s["ctx_dim"], s["hidden"])
        pol.pi.set_params(s["pi"])
        pol.vf.set_params(s["vf"])
        return pol

    def save_npz(self, path) -> None:
        """Plain arrays, so a checkpoint downloaded from Kaggle loads with numpy alone."""
        s = self.state()
        np.savez_compressed(path, cand_dim=s["cand_dim"], ctx_dim=s["ctx_dim"],
                            hidden=s["hidden"],
                            **{f"pi_{k}": a for k, a in enumerate(s["pi"])},
                            **{f"vf_{k}": a for k, a in enumerate(s["vf"])})

    @classmethod
    def load_npz(cls, path) -> UnitPolicy:
        with np.load(path) as z:
            n_pi = sum(1 for k in z.files if k.startswith("pi_"))
            n_vf = sum(1 for k in z.files if k.startswith("vf_"))
            return cls.from_state({
                "cand_dim": int(z["cand_dim"]), "ctx_dim": int(z["ctx_dim"]),
                "hidden": int(z["hidden"]),
                "pi": [z[f"pi_{k}"] for k in range(n_pi)],
                "vf": [z[f"vf_{k}"] for k in range(n_vf)]})


# ---------------------------------------------------------------------- advantages


def smdp_gae(t: np.ndarray, rew: np.ndarray, val: np.ndarray, terminal_t: float,
             gamma: float, lam: float, bootstrap: float = 0.0
             ) -> tuple[np.ndarray, np.ndarray]:
    """Advantages and returns for ONE robot's decisions, in time order.

    `rew[k]` is the reward earned between decision k and decision k+1, already discounted
    back to `t[k]`. After the last decision the episode ends at `terminal_t` (the mission
    clock, or the robot's death) and the value beyond it is `bootstrap` -- zero, since
    time is in the observation and a finished mission is worth nothing more.
    """
    n = len(t)
    adv = np.zeros(n)
    nxt_adv = 0.0
    for k in reversed(range(n)):
        if k == n - 1:
            dt = max(terminal_t - t[k], 0.0)
            nxt_v = bootstrap
        else:
            dt = t[k + 1] - t[k]
            nxt_v = val[k + 1]
        disc = gamma ** dt
        delta = rew[k] + disc * nxt_v - val[k]
        nxt_adv = delta + disc * lam * nxt_adv if k < n - 1 else delta
        adv[k] = nxt_adv
    return adv, adv + val


# ---------------------------------------------------------------------- the update


@dataclass
class Batch:
    cand: np.ndarray      # (B, K, F)
    mask: np.ndarray      # (B, K) bool
    ctx: np.ndarray       # (B, G)
    action: np.ndarray    # (B,)
    logp: np.ndarray      # (B,) at collection time
    adv: np.ndarray       # (B,)
    ret: np.ndarray       # (B,)

    def __len__(self) -> int:
        return len(self.action)


def policy_loss_and_grad(policy: UnitPolicy, b: Batch, cfg: PPOConfig
                         ) -> tuple[dict, list[np.ndarray]]:
    """Clipped surrogate + entropy bonus, and its exact gradient w.r.t. `policy.pi`."""
    B = len(b)
    out, cache = policy.pi.forward(b.cand)
    logits = out[..., 0]
    logp_all = masked_log_softmax(logits, b.mask)
    p = np.exp(logp_all) * b.mask
    rows = np.arange(B)
    logp = logp_all[rows, b.action]
    ratio = np.exp(logp - b.logp)
    unclipped = ratio * b.adv
    clipped = np.clip(ratio, 1.0 - cfg.clip, 1.0 + cfg.clip) * b.adv
    surr = np.minimum(unclipped, clipped)
    safe_logp = np.where(b.mask, logp_all, 0.0)
    ent = -(p * safe_logp).sum(axis=-1)
    loss = -surr.mean() - cfg.ent_coef * ent.mean()

    # d(-surr)/d(logp): the unclipped branch is live when it is the smaller of the two.
    live = unclipped <= clipped
    d_logp = np.where(live, -ratio * b.adv, 0.0) / B
    onehot = np.zeros_like(p)
    onehot[rows, b.action] = 1.0
    d_logits = d_logp[:, None] * (onehot - p)
    # d(-c*H)/dz_j = c * p_j * (log p_j + H)
    d_logits += (cfg.ent_coef / B) * p * (safe_logp + ent[:, None])
    d_logits *= b.mask
    grads = policy.pi.backward(cache, d_logits[..., None])

    approx_kl = float(((ratio - 1.0) - (logp - b.logp)).mean())
    stats = {"pi_loss": float(loss), "entropy": float(ent.mean()), "kl": approx_kl,
             "clip_frac": float((np.abs(ratio - 1.0) > cfg.clip).mean())}
    return stats, grads


def value_loss_and_grad(policy: UnitPolicy, b: Batch, cfg: PPOConfig
                        ) -> tuple[dict, list[np.ndarray]]:
    out, cache = policy.vf.forward(b.ctx)
    v = out[..., 0]
    err = v - b.ret
    loss = cfg.vf_coef * 0.5 * float((err * err).mean())
    grads = policy.vf.backward(cache, (cfg.vf_coef * err / len(b))[:, None])
    ev = 1.0 - float(np.var(b.ret - v)) / max(float(np.var(b.ret)), 1e-12)
    return {"vf_loss": loss, "explained_var": ev}, grads


def bc_loss_and_grad(policy: UnitPolicy, b: Batch) -> tuple[dict, list[np.ndarray]]:
    """Cross-entropy toward the recorded actions: behaviour cloning.

    **Why training starts here.** A freshly initialised policy is near-uniform over its
    candidates, so on its first iteration ~80% of searching robots would abandon their
    frontier for a random stage, a random patch of dark ground, or standing still. PPO
    would spend its first hours unlearning that. Cloning the classical heuristic first
    starts the search from a swarm that works -- the ladder CLAUDE.md asks for, one rung
    at a time -- and PPO then only has to find what the heuristic gets wrong.
    """
    B = len(b)
    out, cache = policy.pi.forward(b.cand)
    logp_all = masked_log_softmax(out[..., 0], b.mask)
    p = np.exp(logp_all) * b.mask
    rows = np.arange(B)
    loss = -float(logp_all[rows, b.action].mean())
    onehot = np.zeros_like(p)
    onehot[rows, b.action] = 1.0
    d_logits = (p - onehot) * b.mask / B
    grads = policy.pi.backward(cache, d_logits[..., None])
    acc = float((np.argmax(np.where(b.mask, logp_all, MASKED), axis=1) == b.action).mean())
    return {"bc_loss": loss, "bc_acc": acc}, grads


class PPO:
    def __init__(self, policy: UnitPolicy, cfg: PPOConfig | None = None) -> None:
        self.policy = policy
        self.cfg = cfg or PPOConfig()
        self.opt_pi = Adam(policy.pi.params(), lr=self.cfg.lr)
        self.opt_vf = Adam(policy.vf.params(), lr=self.cfg.lr)

    def update(self, batch: Batch, rng: np.random.Generator) -> dict:
        cfg = self.cfg
        adv = batch.adv
        # Normalised per batch: advantage scale drifts as the swarm gets better, and the
        # clip range is only meaningful against a fixed scale.
        batch.adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        n = len(batch)
        log: dict[str, list[float]] = {}
        stopped_early = False
        for _ in range(cfg.epochs):
            order = rng.permutation(n)
            for s in range(0, n, cfg.minibatch):
                idx = order[s:s + cfg.minibatch]
                mb = Batch(batch.cand[idx], batch.mask[idx], batch.ctx[idx],
                           batch.action[idx], batch.logp[idx], batch.adv[idx],
                           batch.ret[idx])
                ps, pg = policy_loss_and_grad(self.policy, mb, cfg)
                vs, vg = value_loss_and_grad(self.policy, mb, cfg)
                ps["pi_grad_norm"] = clip_grad_norm(pg, cfg.max_grad_norm)
                vs["vf_grad_norm"] = clip_grad_norm(vg, cfg.max_grad_norm)
                self.opt_pi.step(self.policy.pi.params(), pg)
                self.opt_vf.step(self.policy.vf.params(), vg)
                for k, v in {**ps, **vs}.items():
                    log.setdefault(k, []).append(v)
                if ps["kl"] > cfg.target_kl * 1.5:
                    stopped_early = True
                    break
            if stopped_early:
                break
        out = {k: float(np.mean(v)) for k, v in log.items()}
        out["stopped_early"] = float(stopped_early)
        return out

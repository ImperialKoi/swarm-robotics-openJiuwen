"""A small numpy MLP with exact backprop, and Adam.

**No torch, on purpose.** The unit policy is a few thousand parameters. CLAUDE.md forbids
training in torch on the M1, the Ryzen box has no CUDA, and rollouts dominate the cost by
orders of magnitude -- a 64-wide network's backward pass is microseconds against a
mission tick measured in milliseconds. Numpy runs identically on all three machines this
project trains on, adds no dependency to the 8 GB budget, and keeps inference
deterministic without pinning a framework's thread pool.

The price is writing the gradients by hand, which is why `tests/test_rl_nn.py` checks
every one of them against central finite differences.
"""

from __future__ import annotations

import numpy as np


class MLP:
    """tanh hidden layers, linear output. Accepts any leading shape: `(..., in) -> (..., out)`.

    Leading dimensions matter here: the policy scores `K` candidate goals per robot with
    one shared network, so its input is `(robots, K, features)` and the same weights see
    every candidate. Flattening inside `forward`/`backward` keeps callers from having to.
    """

    def __init__(self, sizes: tuple[int, ...], rng: np.random.Generator,
                 out_gain: float = 1.0) -> None:
        self.sizes = tuple(int(s) for s in sizes)
        self.W: list[np.ndarray] = []
        self.b: list[np.ndarray] = []
        last = len(self.sizes) - 2
        for i, (n_in, n_out) in enumerate(zip(self.sizes[:-1], self.sizes[1:], strict=True)):
            # Orthogonal init: the standard choice for PPO, and it keeps tanh units out of
            # saturation at the start. A small output gain starts the policy near uniform
            # over its candidates instead of confidently wrong.
            gain = out_gain if i == last else np.sqrt(2.0)
            self.W.append(gain * _orthogonal(rng, n_out, n_in))
            self.b.append(np.zeros(n_out))

    # ------------------------------------------------------------------ parameters

    def params(self) -> list[np.ndarray]:
        return [*self.W, *self.b]

    def set_params(self, flat: list[np.ndarray]) -> None:
        n = len(self.W)
        self.W = [np.array(a, dtype=np.float64) for a in flat[:n]]
        self.b = [np.array(a, dtype=np.float64) for a in flat[n:]]

    @property
    def n_params(self) -> int:
        return int(sum(p.size for p in self.params()))

    # ------------------------------------------------------------------ passes

    def forward(self, x: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
        """Output and the activations `backward` needs. `cache[0]` is the flattened input."""
        lead = x.shape[:-1]
        h = np.asarray(x, dtype=np.float64).reshape(-1, x.shape[-1])
        cache = [h]
        last = len(self.W) - 1
        for i, (W, b) in enumerate(zip(self.W, self.b, strict=True)):
            z = h @ W.T + b
            h = np.tanh(z) if i < last else z
            cache.append(h)
        return h.reshape(*lead, self.sizes[-1]), cache

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return self.forward(x)[0]

    def backward(self, cache: list[np.ndarray], dout: np.ndarray) -> list[np.ndarray]:
        """Gradients in `params()` order, given dLoss/dOutput shaped like the output."""
        d = np.asarray(dout, dtype=np.float64).reshape(-1, self.sizes[-1])
        n = len(self.W)
        gW: list[np.ndarray] = [np.empty(0)] * n
        gb: list[np.ndarray] = [np.empty(0)] * n
        for i in reversed(range(n)):
            if i < n - 1:
                d = d * (1.0 - cache[i + 1] ** 2)          # through tanh
            gW[i] = d.T @ cache[i]
            gb[i] = d.sum(axis=0)
            if i > 0:
                d = d @ self.W[i]
        return [*gW, *gb]


def _orthogonal(rng: np.random.Generator, rows: int, cols: int) -> np.ndarray:
    a = rng.standard_normal((max(rows, cols), min(rows, cols)))
    q, r = np.linalg.qr(a)
    q = q * np.sign(np.diag(r))
    return q if rows >= cols else q.T


class Adam:
    """Adam over a list of arrays, updated in place. State is plain arrays, so it pickles."""

    def __init__(self, params: list[np.ndarray], lr: float = 3e-4,
                 betas: tuple[float, float] = (0.9, 0.999), eps: float = 1e-8) -> None:
        self.lr, (self.b1, self.b2), self.eps = lr, betas, eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, params: list[np.ndarray], grads: list[np.ndarray]) -> None:
        self.t += 1
        c1 = 1.0 - self.b1 ** self.t
        c2 = 1.0 - self.b2 ** self.t
        for p, g, m, v in zip(params, grads, self.m, self.v, strict=True):
            m *= self.b1
            m += (1.0 - self.b1) * g
            v *= self.b2
            v += (1.0 - self.b2) * g * g
            p -= self.lr * (m / c1) / (np.sqrt(v / c2) + self.eps)


def clip_grad_norm(grads: list[np.ndarray], max_norm: float) -> float:
    """Scale gradients in place so their global L2 norm is at most `max_norm`. Returns
    the norm before clipping, which is worth logging: a norm pinned at the ceiling every
    update is a learning rate that is too high."""
    total = float(np.sqrt(sum(float((g * g).sum()) for g in grads)))
    if max_norm > 0 and total > max_norm:
        s = max_norm / (total + 1e-12)
        for g in grads:
            g *= s
    return total

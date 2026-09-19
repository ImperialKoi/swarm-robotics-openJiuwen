"""Seeded random number streams.

Determinism is an invariant (CLAUDE.md #5): seed 42 must produce a byte-identical
scorecard hash on every run, because the rehearsed demo timings depend on it.

Every subsystem draws from its own independent stream spawned off one master seed,
so adding a draw in one subsystem cannot shift the numbers another subsystem sees.
Never use ``np.random.*`` module functions or the stdlib ``random`` module anywhere
in the sim, the auction, or the training rollouts.
"""

from __future__ import annotations

import numpy as np

STREAMS = ("map", "victims", "hazard", "robots", "auction", "noise", "fault",
           "perception")


class RngBook:
    """One independent ``Generator`` per subsystem, all derived from a master seed."""

    __slots__ = ("_gens", "master_seed")

    def __init__(self, master_seed: int) -> None:
        self.master_seed = int(master_seed)
        children = np.random.SeedSequence(self.master_seed).spawn(len(STREAMS))
        self._gens = {
            name: np.random.default_rng(child) for name, child in zip(STREAMS, children, strict=True)
        }

    def __getitem__(self, stream: str) -> np.random.Generator:
        try:
            return self._gens[stream]
        except KeyError:
            raise KeyError(f"unknown rng stream {stream!r}; add it to swarmmind.rng.STREAMS") from None

    def __repr__(self) -> str:
        return f"RngBook(master_seed={self.master_seed})"

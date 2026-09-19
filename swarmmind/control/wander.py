"""Placeholder controller.

Produces deterministic wandering motion so the tick loop, collisions, fog, comms and
the victim state machine can all be exercised before Tier 1 exists.

**Replaced** by ``control/tier1_reflex.py``. Nothing may depend on this beyond
smoke tests; it has no obstacle avoidance and no goal.
"""

from __future__ import annotations

import numpy as np


def wander(world, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """(v_cmd, omega_cmd) -- forward drift with a slow random-walk in heading."""
    return world.v_max * 0.7, rng.normal(0.0, 0.8, world.n)

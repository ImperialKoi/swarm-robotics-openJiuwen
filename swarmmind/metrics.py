"""Mission scorecard.

Read by three consumers that must agree: the headless run's stdout, the gate
(training/gate.py), and the GRPO reward function (training/llm/reward.py).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Scorecard:
    seed: int
    sim_time: float
    victims_rescued: int
    victims_found: int
    victims_total: int
    #: Fraction of **passable ground** the swarm has observed -- not a count of sectors.
    #:
    #: Named `sectors_explored_frac` until D12, which read as "46.9% of sectors" when the
    #: same run had touched 77% of them and finished 29%. The value was always ground
    #: coverage and was always honest; the name invited a reading it does not support,
    #: and `training/gate.py` scores on it, so a reader had every reason to take it at
    #: face value. It happens to track mean per-sector coverage almost exactly (46.9%
    #: against 46.8% on seed 42) because the sectors are near-equal in passable area --
    #: which is exactly why the mislabel survived this long.
    ground_explored_frac: float
    robots_lost: int
    robots_total: int
    mean_time_to_rescue: float
    energy_used: float
    directives_issued: int
    directives_rejected: int
    wall_seconds: float = 0.0
    rtf: float = 0.0

    def hash(self) -> str:
        """Stable hash of everything except wall-clock fields.

        tests/test_determinism.py asserts this is identical across runs on the same
        seed. Wall time is excluded because it is the one thing that legitimately varies.
        """
        d = {k: v for k, v in asdict(self).items() if k not in ("wall_seconds", "rtf")}
        payload = json.dumps(d, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(payload.encode()).hexdigest()[:16]

    def render(self) -> str:
        pct = 100.0 * self.ground_explored_frac
        return (
            f"\n  SwarmMind scorecard  seed={self.seed}  t={self.sim_time:.1f}s\n"
            f"  ------------------------------------------------------------\n"
            f"  victims rescued     {self.victims_rescued}/{self.victims_total}\n"
            f"  victims found       {self.victims_found}/{self.victims_total}\n"
            f"  ground explored     {pct:.1f}%\n"
            f"  robots lost         {self.robots_lost}/{self.robots_total}\n"
            f"  mean time to rescue {self.mean_time_to_rescue:.1f}s\n"
            f"  energy used         {self.energy_used:.2f}\n"
            f"  directives          {self.directives_issued} issued, "
            f"{self.directives_rejected} rejected\n"
            f"  wall / rtf          {self.wall_seconds:.2f}s / {self.rtf:.2f}x\n"
            f"  hash                {self.hash()}\n"
        )

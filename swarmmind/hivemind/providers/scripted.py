"""The bottom of the ladder: phase-keyed directives with no model at all.

This is not a placeholder. It is the guarantee that the demo has a Tier 3 even with no
network, no GPU, and no model file -- and it is the baseline the trained hivemind has to
beat at the gate. If a scripted heuristic wins, that is a result worth reporting, not a
failure to hide.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

from ...sim.robot import OUT_OF_COMMS
from ..filter import ABANDON_HAZARD_FLOOR

CONFIG = Path(__file__).resolve().parents[3] / "assets" / "scenarios" / (
    "fallback_directives.yaml"
)


class ScriptedProvider:
    name = "scripted"

    def __init__(self, world, tracker=None, config: dict | None = None) -> None:
        self.world = world
        self.tracker = tracker
        self.cfg = (config if config is not None
                    else yaml.safe_load(CONFIG.read_text(encoding="utf-8")))
        if self.cfg["abandon_with_casualty_above"] > ABANDON_HAZARD_FLOOR:
            raise ValueError(
                "fallback_directives.yaml would propose abandoning a sector holding a "
                "casualty below the filter's floor -- the baseline would reject itself"
            )
        #: Sectors it is never worth proposing to abandon. The filter would refuse them
        #: anyway (F8), but a provider that trips the filter every cycle turns rejection
        #: events into noise -- and rejections are supposed to *mean* something.
        self.never_abandon = {
            world._sector_at(z) for z in (world.scn.base, *world.scn.extraction_zones)
        }


    def _within_reach(self, order):
        """`order`, keeping only sectors the comms component plausibly covers.

        One relay hop of margin, because the chain is built ahead of the frontier (M-40)
        and a directive lands a cycle before the swarm acts on it. Falls back to the
        unfiltered order rather than emitting nothing: a Tier 3 that goes silent when the
        swarm is out of contact is a Tier 3 that goes silent exactly when it is needed.
        """
        w = self.world
        alive = w.status <= OUT_OF_COMMS
        linked = alive & w.in_comms
        if not linked.any():
            return order
        base = np.asarray(w.scn.base, dtype=float)
        reach = float(np.linalg.norm(w.pos[linked] - base, axis=1).max())
        budget = reach + w.scn.comms.relay_radius

        near = []
        for k in order:
            x0, y0, x1, y1 = w.scn.sector_rect(w.sector_ids[int(k)])
            centre = np.array([(x0 + x1) * 0.5, (y0 + y1) * 0.5])
            if float(np.linalg.norm(centre - base)) <= budget:
                near.append(int(k))
        return np.asarray(near, dtype=int) if near else order
    def generate(self, system: str, user: str, timeout: float) -> str:
        w = self.world
        directives: list[dict] = []

        # Pull out of whatever is burning worst, if it is burning enough to matter.
        haz = np.asarray(w.sector_hazard_known)
        if haz.size:
            occupied = {
                w._sector_at(pos)
                for pos in (self.tracker.believed_positions() if self.tracker else [])
            }
            burning = [
                k for k in np.argsort(-haz)
                if haz[k] > self.cfg["abandon_above"]
                and w.sector_ids[int(k)] not in self.never_abandon
                # Don't ask to write off a sector with someone in it until it really is
                # lost. The filter would refuse (F7), but the baseline should not need
                # rescuing from itself -- a rejection ought to mean the *model* erred.
                and (w.sector_ids[int(k)] not in occupied
                     or haz[k] >= self.cfg["abandon_with_casualty_above"])
            ]
            if burning:
                worst = int(burning[0])
                directives.append({
                    "sector": w.sector_ids[worst], "priority": "abandon",
                    "action": "abandon", "reason": "hazard_spreading",
                })

        # Push into the least-explored sectors that are not already on fire -- and that
        # the swarm can still *report* from.
        #
        # "Least explored" correlates almost perfectly with "farthest from base", because
        # exploration radiates outward from base. Ranking on emptiness alone therefore
        # sends the swarm past its own comms envelope, and `World.mark_seen` writes to the
        # shared map only for in-contact robots -- so the ground gets covered and none of
        # it registers. Measured on seed 42: **61% of pushes aimed at sectors beyond the
        # median comms reach** (A8 at 434 m, A7 at 374 m, against a reach of 239 m), and
        # Tier 3 cost discovery on 4 of 4 seeds (M-42, M-43).
        #
        # This is the same wall M-33 and M-36 hit from the other side, and the fix has the
        # same shape: aim at ground the network can support. Reach is read from the
        # swarm's own robots, not from ground truth.
        expl = np.asarray(w.sector_explored_pct)
        order = np.lexsort((haz, expl))
        if self.cfg.get("push_within_reach", True):
            order = self._within_reach(order)
        for k in order[:self.cfg["push_sectors"]]:
            if haz[k] > self.cfg["push_hazard_ceiling"] or w.sector_abandoned[k]:
                continue
            directives.append({
                "sector": w.sector_ids[int(k)], "priority": "high",
                "action": "explore", "reason": "largest unexplored area",
            })

        awaiting = self.tracker.open_reports() if self.tracker else []
        if awaiting:
            sid = w._sector_at(awaiting[0].pos)
            if not any(d["sector"] == sid for d in directives):
                directives.append({
                    "sector": sid, "priority": "high", "action": "rescue",
                    "reason": "casualty located and not yet extracted",
                })

        # Distinct casualties, not resolved reports -- several reports resolve onto the
        # same person, and counting reports claimed 47 recovered out of a possible 8.
        confirmed = (len({r.victim for r in self.tracker.resolved_victims()})
                     if self.tracker else 0)
        reasoning = (
            f"{confirmed} confirmed, {len(awaiting)} awaiting extraction, with "
            f"{w.scn.mission_duration_s - w.t:.0f}s left. "
            + ("Pulling back from the hazard and " if directives and
               directives[0]["action"] == "abandon" else "")
            + "pushing search into the least-covered sectors."
        )
        return json.dumps({"reasoning": reasoning, "directives": directives})

"""Tier 3a: the commander. Territory, not tasks.

**Nothing in this file is an LLM.** The commander is the mind: it decides *who searches
where*, every cycle, deterministically and fast. The language model is an advisor
consulted occasionally (`nodes/hivemind.py`), and it sits above this rather than inside
it. A model that takes 3.6 s to answer cannot own a decision the swarm needs 20 times a
second.

**Why this exists at all.** Nothing in the architecture owned spatial allocation. Tier 2
allocates *tasks*; every scout independently bid on the globally-best frontier target, so
352 scouts converged on the same ground and fanned out only as a side effect of collision
avoidance. Measured: 0.25 new cells per scout-second against a ceiling of ~6.8 -- roughly
**4% search efficiency, 96% of the effort re-treading ground someone had already seen**
(MEASUREMENTS.md M-35). Sector *priority* could never fix that; it nudges a task's rank by
half a point. Partitioning can.

The unit of command is a **squadron**: a few dozen robots holding a contiguous wedge of
the map. Squadrons are what a sub-mind will eventually command individually; for now one
deterministic allocator drives them all, and that allocator is the baseline any learned
commander has to beat on held-out seeds before it ships.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..sim.robot import LANE_INDEX, OUT_OF_COMMS

#: Robots per squadron. Small enough that a squadron's territory is a coherent piece of
#: ground, large enough that losing a few does not leave a wedge unsearched.
SQUADRON_SIZE = 48

#: Re-cut territory this often (seconds). Cheap -- it is a sort over 48 sectors -- but
#: doing it every cycle makes scouts thrash between wedges as the boundary moves.
REBALANCE_S = 20.0

#: A squadron with less than this fraction of its territory left unexplored is finished
#: and gets re-cut onto whatever is still dark.
DONE_FRAC = 0.85


@dataclass(frozen=True)
class CommandParams:
    """The commander's policy, as numbers.

    This is what gets trained. `_cut` reads only these, so a learned commander replaces
    the *values* and not the machinery -- and the hand-set defaults below stay as the
    baseline it has to beat on held-out maps before it ships, exactly as
    `control/heuristic.py` is the baseline for Tier 2.

    Interpretable on purpose (CLAUDE.md): a judge can be shown that this commander weights
    unexplored ground four times as heavily as known contacts, and watch it do that.
    """

    #: Robots per squadron. Small wedges are coherent but fragile; large ones go stale.
    squadron_size: float = float(SQUADRON_SIZE)
    #: Seconds between re-cuts. Too eager and scouts thrash across wedge boundaries.
    rebalance_s: float = REBALANCE_S
    #: Territory is worth holding while this much of it is still dark.
    done_frac: float = DONE_FRAC
    #: How territory value is scored. Unexplored ground is the default currency; contacts
    #: pull squadrons toward casualties already found; hazard pushes them off burning
    #: ground; distance discourages wedges nobody can reach in time.
    w_unexplored: float = 1.0
    w_contacts: float = 0.0
    w_hazard: float = 0.0
    w_distance: float = 0.0

    def as_vector(self) -> np.ndarray:
        return np.array([getattr(self, f) for f in self.__dataclass_fields__],
                        dtype=np.float64)

    @classmethod
    def from_vector(cls, v) -> CommandParams:
        return cls(**dict(zip(cls.__dataclass_fields__, np.asarray(v, float), strict=True)))


class Commander:
    """Assigns every robot to a squadron, and every squadron to a wedge of the map."""

    def __init__(self, world, squadron_size: int | None = None,
                 params: CommandParams | None = None, tracker=None) -> None:
        self.p = params or CommandParams()
        #: The swarm's casualty beliefs, for the `w_contacts` term. **Reports, never
        #: `world.victims`** -- the commander is swarm-side code and invariant #3 says it
        #: may only know what the swarm has observed. `None` (in unit tests that build a
        #: bare Commander) simply means no contact signal, as before.
        self.tracker = tracker
        squadron_size = squadron_size or int(max(8, round(self.p.squadron_size)))
        self.n_squadrons = max(1, int(np.ceil(world.n / squadron_size)))
        # Squadron membership is fixed and deterministic: robots spawn in one heap, so
        # there is no spatial signal to cluster on at t=0. Striding by index spreads each
        # lane evenly across squadrons, so every squadron gets scouts, diggers, carriers
        # and relays rather than one squadron of nothing but carriers.
        self.squadron = np.arange(world.n) % self.n_squadrons
        self.territory = np.zeros(len(world.sector_ids), dtype=np.int32)
        self._next_t = 0.0
        self.stats = {"rebalances": 0}
        self._cut(world)

    # ------------------------------------------------------------------ territory

    def _contacts(self, world, n_sectors: int) -> np.ndarray:
        """Per-sector count of casualties the swarm believes are still waiting, in [0, 1].

        **This term was dead until now.** `_cut` built `contacts = np.zeros(...)` and never
        filled it, so `w_contacts` multiplied zero and could not affect a single decision.
        Run 1 spent 121 generations tuning it to 1.56 -- one of seven CMA-ES dimensions
        optimising pure noise, and a docstring claim ("contacts pull squadrons toward
        casualties already found") that the code did not implement.

        The signal is `tracker.open_reports()`: confirmed sightings nobody has resolved
        yet, which is the tracker's own definition of "worth sending a robot at". It
        empties as casualties get handled, so a sector stops pulling once its work is
        done. Resolved reports are deliberately excluded -- keeping them would leave a
        finished sector pulling squadrons for the rest of the mission.

        Normalised to [0, 1] like `dark`, `hazard` and `dist`, so `w_contacts` is on the
        same scale as the other three weights and the bounds mean the same thing.
        """
        contacts = np.zeros(n_sectors, dtype=np.float64)
        if self.tracker is None:
            return contacts
        h, w = world.shape
        for r in self.tracker.open_reports():
            ix = int(np.clip(r.pos[0] / world.cell, 0, w - 1))
            iy = int(np.clip(r.pos[1] / world.cell, 0, h - 1))
            k = int(world.sector_of_cell[iy, ix])
            if 0 <= k < n_sectors:
                contacts[k] += 1.0
        peak = contacts.max()
        return contacts / peak if peak > 0 else contacts

    def _cut(self, world) -> None:
        """Give each squadron a contiguous wedge of the sectors still worth searching.

        Wedges, not arbitrary sets: a squadron whose sectors are scattered across the map
        spends its mission in transit. Sorting sectors by bearing from base and cutting
        the ordering into equal runs gives every squadron a compact fan to work outward
        along, which is also how the comms chain wants to grow.
        """
        base = np.array(world.scn.base, dtype=np.float64)
        centres, dark = [], []
        for k, sid in enumerate(world.sector_ids):
            x0, y0, x1, y1 = world.scn.sector_rect(sid)
            centres.append(((x0 + x1) * 0.5, (y0 + y1) * 0.5))
            dark.append(1.0 - float(world.sector_explored_pct[k]))
        centres = np.array(centres)
        dark = np.array(dark)

        # What a sector is worth holding. Trained weights, not a fixed rule.
        contacts = self._contacts(world, len(dark))
        hazard = np.asarray(world.sector_hazard_known, dtype=np.float64)
        dist = np.linalg.norm(centres - base, axis=1)
        dist = dist / max(dist.max(), 1e-9)
        value = np.clip(
            self.p.w_unexplored * dark
            + self.p.w_contacts * contacts
            - self.p.w_hazard * hazard
            - self.p.w_distance * dist,
            0.0, None,
        )

        # Sectors still worth searching, ordered by bearing so a run of them is a wedge.
        worth = np.nonzero(dark > (1.0 - self.p.done_frac))[0]
        if len(worth) == 0:
            worth = np.arange(len(world.sector_ids))
        d = centres[worth] - base
        bearing = np.arctan2(d[:, 1], d[:, 0])
        order = worth[np.lexsort((worth, bearing))]

        # Equal *unexplored area* per squadron, not equal sector count: a squadron handed
        # eight nearly-finished sectors is idle while its neighbour has eight dark ones.
        weight = value[order]
        cum = np.cumsum(weight)
        total = cum[-1] if cum[-1] > 0 else 1.0
        share = np.minimum((cum / total * self.n_squadrons).astype(int),
                           self.n_squadrons - 1)
        self.territory[:] = -1
        self.territory[order] = share
        self.stats["rebalances"] += 1

    def step(self, world) -> None:
        if world.t < self._next_t:
            return
        self._next_t = world.t + self.p.rebalance_s
        self._cut(world)

    # ------------------------------------------------------------------ queries

    def owns(self, world, sector_index: int) -> int:
        return int(self.territory[sector_index])

    def in_own_territory(self, world, target) -> np.ndarray:
        """(n,) mask: may this robot take work at `target`?

        Only *search* work is territorial. A casualty is everybody's business -- refusing
        a carrier because the body lies two metres over a wedge boundary would be exactly
        the kind of tidy-minded rule that gets people killed.
        """
        ix = int(np.clip(target[0] / world.cell, 0, world.shape[1] - 1))
        iy = int(np.clip(target[1] / world.cell, 0, world.shape[0] - 1))
        owner = int(self.territory[world.sector_of_cell[iy, ix]])
        if owner < 0:
            return np.ones(world.n, dtype=bool)
        return self.squadron == owner

    def summary(self, world) -> list[dict]:
        """Per-squadron state, for the dashboard and for the advisor's prompt."""
        alive = world.status <= OUT_OF_COMMS
        out = []
        for s in range(self.n_squadrons):
            mine = self.squadron == s
            secs = np.nonzero(self.territory == s)[0]
            dark = float(np.mean([1.0 - world.sector_explored_pct[k] for k in secs])) \
                if len(secs) else 0.0
            out.append({
                "id": s,
                "robots": int((mine & alive).sum()),
                "scouts": int((mine & alive & (world.actuator == LANE_INDEX["none"])).sum()),
                "sectors": [world.sector_ids[k] for k in secs],
                "unexplored": round(dark, 3),
            })
        return out

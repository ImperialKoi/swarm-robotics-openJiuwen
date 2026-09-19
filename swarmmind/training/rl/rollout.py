"""One training episode: run a mission with a sampling unit policy, record, score.

**Rewards read the simulator, and that is correct** -- the same argument as MAP-Elites
fitness (`training/mapelites/evaluate.py`). The robots being trained see nothing but
tracker reports and the shared map; the *reward* says what actually happened, so a policy
that stages at a phantom contact is not paid for a casualty that was never there.

**Credit is per robot, not per swarm.** A team reward over 512 robots and 420 seconds
tells any one robot almost nothing about its own choice. Each event is paid to the robots
that caused it:

    delivery       +1.0   the carrier that brought the casualty in
    pickup         +0.2   the carrier that lifted it -- earlier is better, and the
                          delivery that follows is where the real reward is
    dug out        +0.3   shared by the diggers within reach when the debris cleared
    found          +0.2   shared by every robot whose sightings built the report
    destroyed      -0.5   the robot itself

The gate never sees these numbers. It scores whole missions on `gate.mission_score` on
held-out seeds, so a policy that farms pickups without improving rescues cannot ship --
the training-reward / acceptance-metric split the commander programme settled on.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ...control import unit_policy as up
from ...mission import Mission
from ...sim.robot import DESTROYED, LANE_INDEX
from ...sim.scenario import Scenario
from ...sim.world import CARRIED, CLEARED, FOUND, HIDDEN, REACH_DIG, RESCUED
from .ppo import Batch, UnitPolicy, smdp_gae

R_DELIVERY, R_PICKUP, R_DUG, R_FOUND, R_DEATH = 1.0, 0.2, 0.3, 0.2, -0.5


@dataclass
class Episode:
    seed: int
    robot: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int32))
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    cand: np.ndarray = field(default_factory=lambda: np.zeros((0, up.K, up.CAND_DIM), np.float16))
    mask: np.ndarray = field(default_factory=lambda: np.zeros((0, up.K), bool))
    ctx: np.ndarray = field(default_factory=lambda: np.zeros((0, up.CTX_DIM), np.float32))
    action: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    logp: np.ndarray = field(default_factory=lambda: np.zeros(0))
    #: (robot, time, reward) for every paid event.
    events: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    death_t: dict = field(default_factory=dict)
    end_t: float = 0.0
    card: dict = field(default_factory=dict)
    counts: list = field(default_factory=list)


class Recorder:
    def __init__(self) -> None:
        self.rows: list[tuple] = []

    def on_decisions(self, t, idx, c, action, logp) -> None:
        self.rows.append((np.asarray(idx, np.int32), float(t), c.cand.astype(np.float16),
                          c.mask.copy(), c.ctx.copy(), np.asarray(action, np.int64),
                          np.asarray(logp, np.float64)))


class _Rewarder:
    """Watches state changes each tick and pays the robots responsible."""

    def __init__(self, m: Mission) -> None:
        w = m.world
        self.m = m
        self.prev = [v.state for v in w.victims]
        self.carrier = [-1] * len(w.victims)
        self.events: list[tuple[int, float, float]] = []
        self.death_t: dict[int, float] = {}
        self.dead = w.status >= DESTROYED
        self.paid_found: set[int] = set()

    def tick(self) -> None:
        w, tr = self.m.world, self.m.tracker
        t = w.t
        for vi, v in enumerate(w.victims):
            s, p = v.state, self.prev[vi]
            if s == CARRIED:
                self.carrier[vi] = v.carrier
            if s == p:
                continue
            if p == HIDDEN and s >= FOUND and vi not in self.paid_found:
                self.paid_found.add(vi)
                who = sorted({j for r in tr.reports if r.victim == vi for j in r.robots})
                for j in who:
                    self.events.append((j, t, R_FOUND / len(who)))
            if v.buried and p == FOUND and s >= CLEARED:
                scoop = np.nonzero((w.actuator == LANE_INDEX["scoop"])
                                   & (np.linalg.norm(w.pos - v.pos, axis=1) <= REACH_DIG + 1.0))[0]
                for j in scoop:
                    self.events.append((int(j), t, R_DUG / len(scoop)))
            if s == CARRIED and p != CARRIED and v.carrier >= 0:
                self.events.append((int(v.carrier), t, R_PICKUP))
            if s == RESCUED and self.carrier[vi] >= 0:
                self.events.append((int(self.carrier[vi]), t, R_DELIVERY))
            self.prev[vi] = s
        dead = w.status >= DESTROYED
        for j in np.nonzero(dead & ~self.dead)[0]:
            self.death_t[int(j)] = t
            self.events.append((int(j), t, R_DEATH))
        self.dead = dead


def run_episode(policy_state: dict, seed: int, scenario: str = "demo",
                max_time: float | None = None, sample: bool = True,
                mode: str = "learned", routing_fix: bool = True,
                sample_seed: int = 0) -> Episode:
    """Everything a worker does. Picklable in, picklable out.

    `seed` picks the map; `sample_seed` picks the exploration noise. They are separate
    because training replays the same four demo maps every iteration, and a sampling
    stream keyed on the map alone would draw the same random numbers each time -- the
    policy would explore the same corners of its choices on every pass.
    """
    scn = Scenario.load(scenario)
    policy = UnitPolicy.from_state(policy_state) if mode == "learned" else None
    rec = Recorder()
    rng = (np.random.default_rng(np.random.SeedSequence([seed, sample_seed, 7919]))
           if sample else None)
    ctl = up.UnitController(mode, policy, rng=rng, recorder=rec)
    m = Mission(scn, seed, hivemind=False, unit_policy=ctl, zone_routing=routing_fix)
    rw = _Rewarder(m)
    w = m.world
    limit = max_time if max_time is not None else scn.mission_duration_s
    while not w.done and w.t < limit:
        m.tick()
        rw.tick()

    ep = Episode(seed=seed, end_t=float(w.t), death_t=rw.death_t,
                 counts=ctl.counts.tolist())
    if rec.rows:
        ep.robot = np.concatenate([r[0] for r in rec.rows])
        ep.t = np.concatenate([np.full(len(r[0]), r[1]) for r in rec.rows])
        ep.cand = np.concatenate([r[2] for r in rec.rows])
        ep.mask = np.concatenate([r[3] for r in rec.rows])
        ep.ctx = np.concatenate([r[4] for r in rec.rows])
        ep.action = np.concatenate([r[5] for r in rec.rows])
        ep.logp = np.concatenate([r[6] for r in rec.rows])
    ep.events = np.asarray(rw.events, dtype=np.float64).reshape(-1, 3)
    card = w.scorecard()
    ep.card = {"rescued": card.victims_rescued, "found": card.victims_found,
               "explored": card.ground_explored_frac, "lost": card.robots_lost,
               "mttr": card.mean_time_to_rescue}
    return ep


def to_batch(episodes: list[Episode], policy: UnitPolicy, gamma: float, lam: float) -> Batch:
    """Stack episodes into one PPO batch, with semi-Markov GAE per robot."""
    cands, masks, ctxs, acts, logps, advs, rets = [], [], [], [], [], [], []
    for ep in episodes:
        if len(ep.action) == 0:
            continue
        values = policy.value(ep.ctx.astype(np.float64))
        adv = np.zeros(len(ep.action))
        ret = np.zeros(len(ep.action))
        ev = ep.events
        for r in np.unique(ep.robot):
            rows = np.nonzero(ep.robot == r)[0]
            rows = rows[np.argsort(ep.t[rows], kind="stable")]
            t = ep.t[rows]
            end = ep.death_t.get(int(r), ep.end_t)
            mine = ev[ev[:, 0] == r] if len(ev) else ev
            rew = np.zeros(len(rows))
            if len(mine):
                # Each reward belongs to the decision in force when it was earned, and is
                # discounted back to that decision's time.
                k = np.searchsorted(t, mine[:, 1], side="right") - 1
                ok = k >= 0
                np.add.at(rew, k[ok], mine[ok, 2] * gamma ** (mine[ok, 1] - t[k[ok]]))
            a, rt = smdp_gae(t, rew, values[rows], end, gamma, lam)
            adv[rows], ret[rows] = a, rt
        cands.append(ep.cand.astype(np.float64))
        masks.append(ep.mask)
        ctxs.append(ep.ctx.astype(np.float64))
        acts.append(ep.action)
        logps.append(ep.logp)
        advs.append(adv)
        rets.append(ret)
    return Batch(np.concatenate(cands), np.concatenate(masks), np.concatenate(ctxs),
                 np.concatenate(acts), np.concatenate(logps), np.concatenate(advs),
                 np.concatenate(rets))

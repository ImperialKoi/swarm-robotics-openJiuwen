"""The unit policy: a per-robot decision layer inside Tier 2, for robots with nothing urgent.

**What it controls, and what it does not.** The auction still allocates every task and
still does all self-healing (invariant #1). Tier 1 still steers and still owns the wall
override (invariant #2). This layer acts only on *searching* robots -- idle, or holding an
`explore` task -- and offers each of them K choices:

    0  default  exactly what the executor would have done: the explore target, or drift
    1  stage    wait at the nearest rescue site this lane will be needed at
    2  stage    ...or the second nearest
    3  dark     the nearest unexplored ground this chassis can reach
    4  hold     stay where it is for a short while

A robot that stages is *reserved*: the auction stops offering it search work but still
offers it rescue work, so a carrier parked beside a dig is there, and free, when the
casualty comes out (`SkillExecutor.policy_goal`, `AuctionNode.step`).

**Why this action and not another.** Measured before any of this was written:

* Sending robots to inspect unconfirmed contacts, even with a ground-truth oracle choosing
  only contacts on real undiscovered casualties, moved rescues by +0.5 over four seeds.
  More discovery does not convert (M-50, M-64). That action was dropped.
* Rescue-chain time on the demo map is carrying 56%, **waiting for a carrier 33%**, waiting
  for a digger 11%. Waiting is the part a unit decision can shorten.

**Three modes, one code path.** `default` always chooses 0 and must reproduce the shipped
mission byte for byte (`tests/test_unit_policy.py`). `heuristic` is the classical baseline
for this action space -- CLAUDE.md: the heuristic is built first, and anything learned has
to beat it. `learned` runs the PPO policy (`training/rl/`), deterministically (argmax)
unless a sampling RNG is supplied for training.

**Nothing here reads ground truth** (invariants #3, #4). Sites come from the report tracker
exactly as `nodes/tasks.py` reads them; the rest is the shared explored map, known hazard,
and each robot's own state. Rewards, which do read the simulator, live in `training/rl/`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..perception.tracker import CONFIRMED
from ..sim import grid
from ..sim.robot import CHASSIS_INDEX, LANE_INDEX, OUT_OF_COMMS
from ..sim.world import FOUND

K = 5
DEFAULT, STAGE_A, STAGE_B, DARK, HOLD = range(K)
SLOT_TYPE = np.array([0, 1, 1, 2, 3])
N_TYPES = 4

#: A site further than this is a different job, not a place to wait.
STAGE_RADIUS_M = 60.0
#: How long each kind of hold lasts before the robot is asked again.
STAGE_S, DARK_S, HOLD_S = 60.0, 30.0, 15.0
#: A robot that chose the default is not asked again for this long. Asking every second
#: produces near-identical decisions and gives the policy nothing new to learn from.
DECIDE_EVERY_S = 5.0
#: Robots within this of a point are "at" it.
NEAR_M = 8.0
#: Half-width, in cells, of the windows coverage and crowding are counted over.
WIN = 6

ROBOT_DIM = 8
CAND_DIM = ROBOT_DIM + 17
GLOBAL_DIM = 8
CTX_DIM = ROBOT_DIM + GLOBAL_DIM

GRIP, SCOOP, SCOUT = LANE_INDEX["gripper"], LANE_INDEX["scoop"], LANE_INDEX["none"]


@dataclass
class Sites:
    """Places a carrier or digger will be needed. Swarm knowledge only."""

    pos: np.ndarray          # (S, 2)
    kind: np.ndarray         # (S,) 0 = buried casualty awaiting dig, 1 = confirmed contact
    age: np.ndarray          # (S,) seconds since the swarm first knew of it
    conf: np.ndarray         # (S,) report confidence
    key: list[tuple[str, object]]


class UnitController:
    """Runs the unit policy for a whole swarm, once per auction cycle."""

    def __init__(self, mode: str = "learned", policy=None,
                 rng: np.random.Generator | None = None, recorder=None) -> None:
        if mode not in ("default", "heuristic", "learned"):
            raise ValueError(f"unknown unit-policy mode {mode!r}")
        if mode == "learned" and policy is None:
            raise ValueError("learned mode needs a policy")
        self.mode, self.policy, self.rng, self.recorder = mode, policy, rng, recorder
        self._next_decision: dict[int, float] = {}
        #: robot -> (slot, site key or None, target) for holds this controller set.
        self._holds: dict[int, tuple[int, tuple | None, tuple[float, float]]] = {}
        self.counts = np.zeros(K, dtype=np.int64)

    # ------------------------------------------------------------------ the cycle

    def step(self, mission) -> None:
        w, ex, tr = mission.world, mission.executor, mission.tracker
        if self.mode == "default":
            return                                   # nothing to decide, nothing touched
        sites = collect_sites(w, tr)
        self._expire(w, ex, sites)
        idx = self._deciding(w, ex)
        if len(idx) == 0:
            return
        goal_xy, goal_id = ex.goals(w, mission.nav)
        default_xy = np.array([goal_xy[g] if g >= 0 else tuple(w.pos[i])
                               for i, g in zip(idx, goal_id[idx], strict=True)],
                              dtype=np.float64).reshape(-1, 2)
        c = build(w, ex, idx, default_xy, sites, ex._dark_points(w), ex._reach_labels(w),
                  self._holds)

        if self.mode == "heuristic":
            action = heuristic_actions(w, idx, c, sites, self._holds)
            logp = np.zeros(len(idx))
        else:
            action, logp = self.policy.act(c.cand, c.mask, self.rng)
        if self.recorder is not None:
            self.recorder.on_decisions(w.t, idx, c, action, logp)
        self._apply(w, ex, idx, c, action)

    # ------------------------------------------------------------------ bookkeeping

    def _deciding(self, w, ex) -> np.ndarray:
        alive = w.status <= OUT_OF_COMMS
        search = np.fromiter((a is None or a.kind == "explore" for a in ex.assignment),
                             dtype=bool, count=w.n)
        ok = alive & w.in_comms & search & (w.actuator != LANE_INDEX["antenna"])
        ok &= w.carrying < 0
        idx = [i for i in np.nonzero(ok)[0]
               if i not in ex.policy_goal and self._next_decision.get(int(i), -1.0) <= w.t]
        return np.array(idx, dtype=np.int64)

    def _expire(self, w, ex, sites: Sites) -> None:
        live = set(sites.key)
        for i in sorted(self._holds):
            slot, key, tgt = self._holds[i]
            pg = ex.policy_goal.get(i)
            gone = pg is None or pg[2] <= w.t or w.status[i] > OUT_OF_COMMS
            if not gone and key is not None and key not in live:
                gone = True                          # the dig finished, or the contact settled
            if not gone and slot == DARK:
                ix, iy = grid.world_to_cell(np.asarray(tgt[0]), np.asarray(tgt[1]),
                                            w.cell, w.shape)
                gone = bool(w.explored[int(iy), int(ix)])
            if gone:
                ex.clear_policy_goal(i)
                del self._holds[i]

    def _apply(self, w, ex, idx, c, action) -> None:
        for row, i in enumerate(idx):
            i = int(i)
            a = int(action[row])
            self.counts[a] += 1
            self._next_decision[i] = w.t + DECIDE_EVERY_S
            if a == DEFAULT:
                continue
            tgt = (float(c.targets[row, a, 0]), float(c.targets[row, a, 1]))
            if ex.assignment[i] is not None:
                ex.release(i, "unit policy: leaving the frontier")
            if a in (STAGE_A, STAGE_B):
                key = c.site_keys[row][a]
                why = ("unit policy: waiting at a dig for the casualty"
                       if key and key[0] == "v" else "unit policy: waiting near a contact")
                ex.set_policy_goal(i, tgt, why, w.t + STAGE_S)
            elif a == DARK:
                key = None
                ex.set_policy_goal(i, tgt, "unit policy: heading for unexplored ground",
                                   w.t + DARK_S)
            else:
                key = None
                ex.set_policy_goal(i, tgt, "unit policy: holding position", w.t + HOLD_S)
            self._holds[i] = (a, key, tgt)


# ---------------------------------------------------------------------- features


def collect_sites(w, tracker) -> Sites:
    pos, kind, age, conf, key = [], [], [], [], []
    seen: set[int] = set()
    for r in tracker.resolved_victims():
        vi = int(r.victim)
        if vi in seen:
            continue
        seen.add(vi)
        v = w.victims[vi]
        if v.state == FOUND and v.buried and v.debris_remaining > 0.0:
            pos.append(v.pos)
            kind.append(0)
            age.append(w.t - r.first_t)
            conf.append(r.conf)
            key.append(("v", vi))
    for r in tracker.reports:
        if r.state == CONFIRMED:
            pos.append(r.pos)
            kind.append(1)
            age.append(w.t - r.first_t)
            conf.append(r.conf)
            key.append(("r", r.id))
    return Sites(np.asarray(pos, dtype=np.float64).reshape(-1, 2),
                 np.asarray(kind, dtype=np.int64), np.asarray(age, dtype=np.float64),
                 np.asarray(conf, dtype=np.float64), key)


class Candidates:
    def __init__(self, cand, mask, ctx, targets, site_keys):
        self.cand, self.mask, self.ctx = cand, mask, ctx
        self.targets, self.site_keys = targets, site_keys


def _window_mean(a: np.ndarray, half: int) -> np.ndarray:
    """Mean of `a` over the (2h+1)^2 window around every cell, by integral image."""
    s = 2 * half + 1
    p = np.pad(a.astype(np.float64), half + 1, mode="edge")
    ii = p.cumsum(0).cumsum(1)
    win = ii[s:, s:] - ii[:-s, s:] - ii[s:, :-s] + ii[:-s, :-s]
    return (win / (s * s))[: a.shape[0], : a.shape[1]]


def build(w, ex, idx, default_xy, sites: Sites, dark_pts, reach, holds) -> Candidates:
    n = len(idx)
    T = w.scn.mission_duration_s
    pos = w.pos[idx]
    lane = w.actuator[idx]
    cand = np.zeros((n, K, CAND_DIM), dtype=np.float32)
    mask = np.zeros((n, K), dtype=bool)
    tgt = np.repeat(pos[:, None, :], K, axis=1).astype(np.float64)
    site_keys: list[list] = [[None] * K for _ in range(n)]
    site_row = np.full((n, K), -1, dtype=np.int64)

    rf = np.zeros((n, ROBOT_DIM), dtype=np.float32)
    rf[:, 0] = lane == SCOUT
    rf[:, 1] = lane == SCOOP
    rf[:, 2] = lane == GRIP
    rf[:, 3] = w.chassis[idx] == CHASSIS_INDEX["rotor"]
    rf[:, 4] = w.battery[idx]
    rf[:, 5] = w.t / T
    rf[:, 6] = w.v_max[idx] / 3.0
    rf[:, 7] = [ex.assignment[i] is not None for i in idx]
    cand[:, :, :ROBOT_DIM] = rf[:, None, :]

    # 0 default and 4 hold are always offered.
    mask[:, DEFAULT] = True
    mask[:, HOLD] = True
    tgt[:, DEFAULT] = default_xy

    # 1-2 stage: carriers and diggers only.
    if len(sites.pos) and n:
        d = np.linalg.norm(pos[:, None, :] - sites.pos[None, :, :], axis=2)
        worker = (lane == GRIP) | (lane == SCOOP)
        d = np.where(worker[:, None] & (d <= STAGE_RADIUS_M), d, np.inf)
        order = np.argsort(d, axis=1, kind="stable")
        for s, slot in enumerate((STAGE_A, STAGE_B)):
            if s >= d.shape[1]:
                break
            j = order[:, s]
            ok = np.isfinite(d[np.arange(n), j])
            mask[:, slot] = ok
            tgt[ok, slot] = sites.pos[j[ok]]
            site_row[ok, slot] = j[ok]
            for row in np.nonzero(ok)[0]:
                site_keys[row][slot] = sites.key[int(j[row])]

    # 3 dark
    for ch, pts in dark_pts.items():
        rows = np.nonzero(w.chassis[idx] == ch)[0]
        if not len(rows) or not len(pts):
            continue
        d = np.linalg.norm(pos[rows][:, None, :] - pts[None, :, :], axis=2)
        j = np.argmin(d, axis=1)
        mask[rows, DARK] = True
        tgt[rows, DARK] = pts[j]

    # Reachability on this robot's own component: never offer ground it cannot get to.
    tx, ty = grid.world_to_cell(tgt[..., 0].ravel(), tgt[..., 1].ravel(), w.cell, w.shape)
    tx, ty = tx.reshape(n, K), ty.reshape(n, K)
    for ch, lab in reach.items():
        rows = np.nonzero(w.chassis[idx] == ch)[0]
        if not len(rows):
            continue
        rx, ry = grid.world_to_cell(pos[rows, 0], pos[rows, 1], w.cell, w.shape)
        same = lab[ty[rows], tx[rows]] == lab[ry, rx][:, None]
        mask[rows, 1:4] &= same[:, 1:4]

    # --- geometry and context, every slot at once --------------------------------------
    alive = w.status <= OUT_OF_COMMS
    occ = np.zeros(w.shape, dtype=np.float64)
    ax, ay = grid.world_to_cell(w.pos[alive, 0], w.pos[alive, 1], w.cell, w.shape)
    np.add.at(occ, (ay, ax), 1.0)
    crowd = _window_mean(occ, WIN) * (2 * WIN + 1) ** 2
    cover = _window_mean(w.explored, WIN)

    delta = tgt - pos[:, None, :]
    dist = np.linalg.norm(delta, axis=2)
    ang = np.arctan2(delta[..., 1], delta[..., 0]) - w.theta[idx][:, None]
    o = ROBOT_DIM
    for slot, t_ in enumerate(SLOT_TYPE):
        cand[:, slot, o + int(t_)] = 1.0
    cand[..., o + 4] = np.minimum(dist / 60.0, 3.0)
    cand[..., o + 5] = np.cos(ang)
    cand[..., o + 6] = np.sin(ang)
    cand[..., o + 7] = cover[ty, tx]
    cand[..., o + 8] = np.minimum(crowd[ty, tx] / 10.0, 2.0)
    cand[..., o + 9] = w.hazard_known[ty, tx]

    # Site features, stage slots only.
    if len(sites.pos):
        dig_near = _count_near(w, sites.pos, SCOOP)
        grip_near = _count_near(w, sites.pos, GRIP)
        staged = np.zeros(len(sites.pos))
        key_index = {k: q for q, k in enumerate(sites.key)}
        for _i, (_slot, key, _t) in sorted(holds.items()):
            if key is not None and key in key_index:
                staged[key_index[key]] += 1
        for slot in (STAGE_A, STAGE_B):
            rows = np.nonzero(site_row[:, slot] >= 0)[0]
            j = site_row[rows, slot]
            cand[rows, slot, o + 10] = sites.kind[j] == 0
            cand[rows, slot, o + 11] = sites.kind[j] == 1
            cand[rows, slot, o + 12] = np.minimum(sites.age[j] / 60.0, 2.0)
            cand[rows, slot, o + 13] = np.minimum(dig_near[j] / 3.0, 2.0)
            cand[rows, slot, o + 14] = np.minimum(grip_near[j] / 3.0, 2.0)
            cand[rows, slot, o + 15] = np.minimum(staged[j] / 3.0, 2.0)
            cand[rows, slot, o + 16] = sites.conf[j]

    gf = np.zeros(GLOBAL_DIM, dtype=np.float32)
    gf[0] = float(w.explored[w.passable].mean())
    # The two counts the blackboard already publishes (`/swarm/state`), not a scan.
    gf[1] = w.victims_found / max(len(w.victims), 1)
    gf[2] = w.victims_rescued / max(len(w.victims), 1)
    gf[3] = float(alive.mean())
    gf[4] = min(float((sites.kind == 0).sum()) / 20.0, 3.0)
    gf[5] = min(float((sites.kind == 1).sum()) / 30.0, 3.0)
    gf[7] = len(holds) / max(w.n, 1)
    base = np.asarray(w.scn.base, dtype=np.float64)
    diag = float(np.hypot(*w.shape)) * w.cell
    ctx = np.concatenate([rf, np.repeat(gf[None, :], n, axis=0)], axis=1)
    ctx[:, ROBOT_DIM + 6] = np.linalg.norm(pos - base, axis=1) / diag
    return Candidates(cand, mask, ctx.astype(np.float32), tgt, site_keys)


def _count_near(w, pts: np.ndarray, lane: int) -> np.ndarray:
    m = (w.actuator == lane) & (w.status <= OUT_OF_COMMS)
    if not m.any() or not len(pts):
        return np.zeros(len(pts))
    d = np.linalg.norm(pts[:, None, :] - w.pos[m][None, :, :], axis=2)
    return (d <= NEAR_M).sum(axis=1).astype(np.float64)


# ---------------------------------------------------------------------- the baseline


def heuristic_actions(w, idx, c: Candidates, sites: Sites, holds) -> np.ndarray:
    """The classical rule for this action space. The learned policy has to beat it.

    A carrier stages at the nearest buried casualty that has no carrier on or near it;
    a digger stages at the nearest confirmed contact that has fewer than two diggers near
    it. Everyone else does the default. Claims are exclusive within one cycle, in robot
    order, so two carriers do not converge on one dig (the M-67 pile, again).
    """
    action = np.zeros(len(idx), dtype=np.int64)
    if not len(sites.pos):
        return action
    o = ROBOT_DIM
    claimed: set = {key for (_s, key, _t) in holds.values() if key is not None}
    for row, i in enumerate(idx):
        lane = w.actuator[i]
        if lane not in (GRIP, SCOOP) or not c.mask[row, STAGE_A]:
            continue
        key = c.site_keys[row][STAGE_A]
        if key in claimed:
            continue
        is_dig = c.cand[row, STAGE_A, o + 10] > 0.5
        carrier_needed = lane == GRIP and is_dig and c.cand[row, STAGE_A, o + 14] * 3.0 < 1.0
        digger_needed = (lane == SCOOP and not is_dig
                         and c.cand[row, STAGE_A, o + 13] * 3.0 < 2.0)
        if carrier_needed or digger_needed:
            action[row] = STAGE_A
            claimed.add(key)
    return action


__all__ = ["CAND_DIM", "CTX_DIM", "DARK", "DEFAULT", "HOLD", "K", "STAGE_A", "STAGE_B",
           "UnitController", "build", "collect_sites", "heuristic_actions"]

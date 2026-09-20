"""Turning detections into beliefs.

A single detection is not knowledge. At 5 m the classical detector's precision is 0.48
(MEASUREMENTS.md M-7), so acting on one sighting means half the swarm's trips go to
rubble. This is where sightings are clustered, corroborated, promoted -- and, crucially,
**dismissed** when a robot gets close enough to see there is nobody there.

This is the swarm's belief, not the world's state. Nothing here reads victim positions;
it only ever sees ``Detection`` objects. The one place ground truth enters is
``resolve()``, and it enters the way it would for a real robot: a machine standing two
metres from a target can tell a casualty from a warm rock.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..sim.robot import OUT_OF_COMMS

CANDIDATE, CONFIRMED, RESOLVED, DISMISSED = 0, 1, 2, 3
STATE_NAME = {CANDIDATE: "candidate", CONFIRMED: "confirmed",
              RESOLVED: "resolved", DISMISSED: "dismissed"}


@dataclass
class Report:
    id: str
    pos: np.ndarray
    n_obs: int = 0
    conf_sum: float = 0.0
    #: Sorted list, never a set -- set iteration order would break determinism.
    robots: list[int] = field(default_factory=list)
    first_t: float = 0.0
    last_t: float = 0.0
    state: int = CANDIDATE
    victim: int | None = None
    #: Observer positions far enough apart to count as genuinely different vantage
    #: points. Repeated looks from one spot are not independent evidence.
    views: list[np.ndarray] = field(default_factory=list)
    #: Best range-weighted confidence any observation achieved.
    best_conf: float = 0.0
    #: Sightings folded into an already-dismissed report and thrown away.
    suppressed: int = 0

    @property
    def conf(self) -> float:
        return self.conf_sum / max(1, self.n_obs)

    @property
    def distinct_robots(self) -> int:
        return len(self.robots)


class VictimReportTracker:
    """Clusters detections into reports and resolves them on close approach."""

    def __init__(self, *, merge_radius: float = 3.0, resolve_radius: float = 3.0,
                 match_radius: float = 3.5, min_views: int = 3, min_conf: float = 0.34,
                 parallax: float = 2.0, trust_near: float = 2.0, trust_far: float = 9.0,
                 stale_s: float = 20.0) -> None:
        self.merge_radius = merge_radius
        self.resolve_radius = resolve_radius
        self.match_radius = match_radius
        #: Distinct vantage points needed before a candidate is worth a robot's time.
        self.min_views = min_views
        self.min_conf = min_conf
        #: How far an observer must move for its next look to count as a new view.
        self.parallax = parallax
        #: Range trust window. Taken originally from the measured detector
        #: characteristic (MEASUREMENTS.md M-7) -- precision is 0.99 at 3.5 m and 0.05 at
        #: 6.5 m -- so a distant sighting is weak evidence and must not confirm anything
        #: on its own. That reasoning is right about *confirmation* and was wrong about
        #: everything else, because `trust_far` gates two things at once.
        #:
        #: `_range_trust` returns exactly 0.0 past `trust_far`, and that zero is used
        #: both as the confirmation weight and as `w` in the position-estimate mean in
        #: `_observe`. At 5 m a distant look was therefore discarded *entirely*: it could
        #: not corroborate, and it could not sharpen where the report thinks the casualty
        #: is either. Gate attribution on seed 44 found 5 of 15 missed casualties whose
        #: closest look was 4.4-5.0 m -- seen 11 to 40 times, every sighting scored zero.
        #:
        #: Widening to 9 m is not a recall trade. Measured on seeds 42/44/45 it gives
        #: **+5, +6, +6 rescues**, exploration flat to +0.7 points, and 11% *fewer*
        #: reports -- distant looks merge duplicates and sharpen positions, so the swarm
        #: makes fewer investigate trips and more of them pay out. Every axis improves.
        #:
        #: Do not reach for `min_views` or `stale_s` next. Both were swept alongside it:
        #: `min_views` 3 -> 2 adds finds and costs 8 rescues on two seeds of three, and
        #: `stale_s` 20 -> 60 is worse on every axis.
        self.trust_near, self.trust_far = trust_near, trust_far
        self.stale_s = stale_s
        self.reports: list[Report] = []
        self._seq = 0
        self._buffer: dict[int, list] = {}
        self.stats = {"created": 0, "confirmed": 0, "resolved": 0, "dismissed": 0}

    # ------------------------------------------------------------------ ingest

    def ingest(self, world, detections) -> None:
        """Fold this pass's detections into the belief set.

        Detections from out-of-comms robots are buffered, not published: an
        unreachable robot cannot tell anyone what it saw.
        """
        live: list = []
        for d in detections:
            if world.in_comms[d.robot]:
                live.append(d)
            else:
                self._buffer.setdefault(d.robot, []).append(d)
        self._merge(world, live)

    def flush(self, world, robot: int) -> int:
        """Publish a reconnected robot's buffered sightings. Returns how many."""
        held = self._buffer.pop(robot, [])
        if held:
            self._merge(world, held)
        return len(held)

    def _merge(self, world, detections) -> None:
        if not detections:
            return
        pts = np.array([d.pos for d in detections], dtype=np.float64)
        conf = np.array([d.conf for d in detections], dtype=np.float64)
        who = np.array([d.robot for d in detections], dtype=np.int32)

        rng_m = np.array([d.range_m for d in detections], dtype=np.float64)
        # Dismissed reports take part in matching so they can absorb and suppress
        # repeat sightings of the thing that already turned out to be rubble.
        open_reports = [r for r in self.reports
                        if r.state in (CANDIDATE, CONFIRMED, DISMISSED)]
        assign = np.full(len(detections), -1, dtype=np.int32)
        if open_reports:
            rp = np.array([r.pos for r in open_reports])
            d2 = ((pts[:, None, :] - rp[None, :, :]) ** 2).sum(axis=2)
            best = np.argmin(d2, axis=1)
            near = d2[np.arange(len(pts)), best] <= self.merge_radius ** 2
            assign[near] = best[near]

        for k in np.nonzero(assign >= 0)[0]:
            r = open_reports[int(assign[k])]
            if r.state == DISMISSED:
                # A robot already stood here and saw nobody. Remember that, so the
                # swarm does not re-report the same rock for the rest of the mission.
                r.suppressed += 1
                r.last_t = world.t
                continue
            self._observe(world, r, pts[k], float(conf[k]), int(who[k]),
                          float(rng_m[k]), world.t)

        # Unassigned detections are grid-snapped so a burst of sightings of the same
        # thing becomes one new report rather than dozens.
        fresh = np.nonzero(assign < 0)[0]
        if len(fresh):
            keys = np.round(pts[fresh] / self.merge_radius).astype(np.int64)
            _, first, inv = np.unique(keys, axis=0, return_index=True, return_inverse=True)
            for g in range(len(first)):
                members = fresh[inv == g]
                self._seq += 1
                r = Report(id=f"rep{self._seq:04d}", pos=pts[members].mean(axis=0).copy(),
                           first_t=world.t, last_t=world.t)
                self.reports.append(r)
                self.stats["created"] += 1
                for k in members:
                    self._observe(world, r, pts[k], float(conf[k]), int(who[k]),
                                  float(rng_m[k]), world.t)

    def _observe(self, world, r: Report, pos, conf: float, robot: int,
                 range_m: float, t: float) -> None:
        eff = conf * self._range_trust(range_m)
        r.best_conf = max(r.best_conf, eff)

        # A new vantage point only counts if the observer actually moved. Otherwise a
        # robot parked in front of a warm rock manufactures corroboration at 5 Hz.
        obs = world.pos[robot]
        if all(float(np.linalg.norm(obs - v)) > self.parallax for v in r.views):
            r.views.append(obs.copy())

        # Confidence-weighted running mean, so confident close-range sightings pull the
        # estimate harder than marginal distant ones.
        w = max(eff, 1e-3)
        total = r.conf_sum + w
        r.pos = (r.pos * r.conf_sum + np.asarray(pos) * w) / total
        r.conf_sum = total
        r.n_obs += 1
        r.last_t = t
        if robot not in r.robots:
            r.robots.append(robot)
            r.robots.sort()
        if r.state == CANDIDATE and self._promotable(r):
            r.state = CONFIRMED
            self.stats["confirmed"] += 1

    def _range_trust(self, range_m: float) -> float:
        """How much a sighting at this range is worth, from the measured precision curve."""
        if range_m <= self.trust_near:
            return 1.0
        if range_m >= self.trust_far:
            return 0.0
        return float((self.trust_far - range_m) / (self.trust_far - self.trust_near))

    def _promotable(self, r: Report) -> bool:
        """Corroboration from genuinely different vantage points, plus one good look.

        Both halves matter. Requiring several views stops a single robot manufacturing
        agreement with itself at 5 Hz; requiring one confident close-range look stops a
        crowd of distant, near-worthless sightings promoting each other. Without them
        96% of confirmed reports were rubble and the swarm spent the mission
        investigating rocks.
        """
        return len(r.views) >= self.min_views and r.best_conf >= self.min_conf

    # ------------------------------------------------------------------ resolve

    def resolve(self, world) -> list[tuple[Report, bool, int]]:
        """Settle reports a robot is standing next to. Returns (report, was_real, by).

        This is where phantoms die. A robot within ``resolve_radius`` of a report can
        see plainly whether anyone is there -- which is exactly what a real machine at
        two metres could do, and is why chasing a false positive costs a trip rather
        than costing the mission.

        **Candidates resolve too, not only confirmed reports.** They used to be
        excluded, and that exclusion was a recall hole with a mission-sized cost: a
        robot can stand *on top of* a candidate -- 0.1 m away, measured -- and learn
        nothing, because nothing had yet corroborated the sighting. On seed 42, 21 of
        110 casualties finished the mission hidden with a report 0.3-1.8 m from them
        and robots passing within 0.1-3 m: the evidence was all there and no one was
        allowed to look. Resolution is a physical act, not a promotion; a candidate
        that a ground robot reaches is settled the same way a confirmed one is.
        """
        out: list[tuple[Report, bool, int]] = []
        open_reports = [r for r in self.reports if r.state in (CANDIDATE, CONFIRMED)]
        if not open_reports:
            return out
        # This is a shared resolution, requiring both a ground inspection and a link.
        # An aircraft passing overhead sees no ground frame, and a disconnected robot
        # cannot publish confirmation. Keep the report pending until someone can.
        alive = (world.status <= OUT_OF_COMMS) & ~world.airborne & world.in_comms
        if not alive.any():
            return out

        alive_idx = np.nonzero(alive)[0]
        rp = np.array([r.pos for r in open_reports])
        d2 = ((world.pos[alive_idx][:, None, :] - rp[None, :, :]) ** 2).sum(axis=2)
        reached = (d2 <= self.resolve_radius ** 2).any(axis=0)
        closest = alive_idx[np.argmin(d2, axis=0)]

        vpos = np.array([v.pos for v in world.victims]) if world.victims else np.zeros((0, 2))
        for j in np.nonzero(reached)[0]:
            r = open_reports[j]
            hit = -1
            if len(vpos):
                dv = np.linalg.norm(vpos - r.pos, axis=1)
                k = int(np.argmin(dv))
                if dv[k] <= self.match_radius and world.victims[k].state != 4:
                    hit = k
            if hit >= 0:
                r.state, r.victim = RESOLVED, hit
                self.stats["resolved"] += 1
                out.append((r, True, int(closest[j])))
            else:
                r.state = DISMISSED
                self.stats["dismissed"] += 1
                out.append((r, False, int(closest[j])))
        return out

    def prune(self, world) -> None:
        """Drop candidates nothing has corroborated in a while."""
        keep = []
        for r in self.reports:
            if r.state == CANDIDATE and world.t - r.last_t > self.stale_s:
                continue
            # Dismissed reports are kept for the whole mission on purpose: they are the
            # swarm's memory of "we looked there, there was nobody". Dropping them means
            # re-discovering and re-investigating the same rubble indefinitely.
            keep.append(r)
        self.reports = keep

    # ------------------------------------------------------------------ queries

    def open_reports(self) -> list[Report]:
        """Confirmed but not yet resolved -- the things worth sending a robot at."""
        return [r for r in self.reports if r.state == CONFIRMED]

    def resolved_victims(self) -> list[Report]:
        return [r for r in self.reports if r.state == RESOLVED and r.victim is not None]

    def believed_positions(self) -> list[tuple[float, float]]:
        return [
            (float(r.pos[0]), float(r.pos[1]))
            for r in self.reports
            if r.state in (CONFIRMED, RESOLVED)
        ]

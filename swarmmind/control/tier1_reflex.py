"""Tier 1: the reflex layer. 20 Hz, fully vectorised, no learning.

**This is the safety floor (CLAUDE.md invariant #2).** The swept-circle wall override at
the end of ``commands`` is not bypassable by any higher tier. Whatever the auction wants
and whatever the hivemind directs, a robot does not drive into a wall.

Global guidance comes from a shared flow field per goal (control/planner.py), not from
per-robot A*: cost is per goal, not per robot, which is what makes 512 robots affordable.
Local avoidance and inter-robot separation are computed here every tick.

Every buffer is preallocated. At 512 robots the pairwise separation term alone would
otherwise churn ~160 MB/s of temporaries at 20 Hz.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..sim import grid
from ..sim.robot import OUT_OF_COMMS

#: Body-relative ray directions for local obstacle probing, fixed order.
_RAYS = np.linspace(-np.pi, np.pi, 8, endpoint=False, dtype=np.float64)

#: Sum of probe weights across both rings, used to normalise repulsion into [0, 1].
_MAX_PROBE_WEIGHT = len(_RAYS) * (1.0 + 0.45)

#: Half-angle of the "in front of me" arc, and the repulsion magnitude guaranteed when
#: something inside it is blocked. Tuned against the override rate, not by eye: see
#: MEASUREMENTS.md M-34.
FRONT_ARC = np.pi / 4
FRONT_MIN_PUSH = 0.65


@dataclass(frozen=True)
class ReflexParams:
    """Tuning for the reflex layer.

    These are hand-set defaults and the classical baseline. Several are also evolved as
    part of the Tier-2 genome (``formation_spacing`` -> ``separation_radius``); the
    evolved values override per robot when a genome is supplied.
    """

    w_goal: float = 1.0
    w_obstacle: float = 2.2
    w_separation: float = 0.7
    #: Repulsion from hazard the swarm has *observed*. Not a strategy -- a reflex.
    #: Weighted above obstacles: a wall costs a robot time, fire costs the robot.
    w_hazard: float = 3.5
    probe_near: float = 1.0      # metres beyond the body radius
    probe_far: float = 2.6
    probe_hazard: float = 6.0
    separation_radius: float = 2.5
    k_omega: float = 2.5
    arrive_radius: float = 1.5
    #: Forward speed is gated by cos(heading error): a robot that must turn hard slows
    #: down rather than carving a wide arc into an obstacle.
    min_speed_frac: float = 0.05


class ReflexController:
    def __init__(self, world, params: ReflexParams | None = None) -> None:
        self.p = params or ReflexParams()
        n = world.n
        self.n = n
        self._dir = np.zeros((n, 2))
        self._rep = np.zeros((n, 2))
        self._sep = np.zeros((n, 2))
        self._haz = np.zeros((n, 2))
        self._dx = np.zeros((n, n), dtype=np.float32)
        self._dy = np.zeros((n, n), dtype=np.float32)
        self._d2 = np.zeros((n, n), dtype=np.float32)
        #: Per-robot separation radius once a genome is loaded; None = the scalar.
        self._spacing = None
        self.last_arrived = np.zeros(n, dtype=bool)
        self.blocked_count = 0   # diagnostic: how often the hard override fired

    # ------------------------------------------------------------------ main entry

    def commands(self, world, nav, goal_xy: list[tuple[float, float]],
                 goal_id: np.ndarray,
                 stop_radius: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(v_cmd, omega_cmd) for every robot.

        ``goal_xy`` is the list of *distinct* goals this tick; ``goal_id`` is (N,) with
        -1 meaning "no assignment". Grouping by goal is what keeps flow-field cost
        proportional to goals rather than robots.
        """
        p = self.p
        alive = world.status <= OUT_OF_COMMS

        self._goal_directions(world, nav, goal_xy, goal_id)
        self._obstacle_repulsion(world)
        self._separation(world)

        desired = (p.w_goal * self._dir + p.w_obstacle * self._rep
                   + p.w_separation * self._sep + p.w_hazard * self._haz)

        # A robot with no goal and nothing pushing it holds position rather than
        # drifting -- drift looks like malfunction on the dashboard.
        mag = np.linalg.norm(desired, axis=1)
        moving = alive & (mag > 1e-6)

        target = np.arctan2(desired[:, 1], desired[:, 0])
        err = _wrap_pi(target - world.theta)
        omega = np.clip(p.k_omega * err, -world.omega_max, world.omega_max) * moving

        gate = np.maximum(np.cos(err), p.min_speed_frac)
        v = world.v_max * gate * moving

        # Stop on arrival so robots settle instead of orbiting their goal.
        arrived = self._arrived(world, goal_xy, goal_id, stop_radius)
        # Kept so callers can tell who is parked without recomputing it -- `Mission`
        # uses it to decide which rotors are still in the air.
        self.last_arrived = arrived
        v = np.where(arrived, 0.0, v)

        return self._wall_override(world, v), omega

    # ------------------------------------------------------------------ terms

    def _goal_directions(self, world, nav, goal_xy, goal_id) -> None:
        self._dir[:] = 0.0
        if not goal_xy:
            return
        # Grouped by (goal, chassis): a flow field built on one locomotion's passability
        # is a lie for any other, and following it walks a wheeled unit into a river.
        n_chassis = len(nav.chassis)
        for gi in range(len(goal_xy)):
            at_goal = goal_id == gi
            if not at_goal.any():
                continue
            gx, gy = goal_xy[gi]
            for c in range(n_chassis):
                m = at_goal & (world.chassis == c)
                if not m.any():
                    continue
                d = nav.descend_to(c, gx, gy, world.pos[m, 0], world.pos[m, 1])
            # On the goal's own coarse cell the field is flat, so steer straight at it.
                flat = (np.abs(d).sum(axis=1) == 0.0)
                if flat.any():
                    idx = np.nonzero(m)[0][flat]
                    delta = np.array([gx, gy]) - world.pos[idx]
                    nrm = np.linalg.norm(delta, axis=1, keepdims=True)
                    d[flat] = np.divide(delta, np.maximum(nrm, 1e-9))
                self._dir[m] = d

    def _obstacle_repulsion(self, world) -> None:
        """Two rings of 8 body-relative probes. Pushes away from blocked directions."""
        p = self.p
        self._rep[:] = 0.0
        self._haz[:] = 0.0
        blocked_near = np.zeros((world.n, len(_RAYS)), dtype=bool)
        ang = world.theta[:, None] + _RAYS[None, :]
        ca, sa = np.cos(ang), np.sin(ang)
        # Hazard is sensed further out than walls: the useful reaction distance is set
        # by how fast the front spreads, not by the body radius.
        for reach, weight in ((p.probe_near, 1.0), (p.probe_far, 0.45), (p.probe_hazard, 0.8)):
            r = (world.radius[:, None] + reach)
            ix, iy = grid.world_to_cell(
                world.pos[:, 0:1] + r * ca, world.pos[:, 1:2] + r * sa, world.cell, world.shape
            )
            if reach != p.probe_hazard:
                # Impassable *for this robot* -- walls plus ground its chassis cannot
                # cross. Probing only for walls left robots with no repulsion from
                # water and steep slope: they drove straight into terrain, stalled
                # against it, and covered 62 m in a 420 s mission out of a possible 630.
                blocked = ~world.chassis_passable[world.chassis[:, None], iy, ix]
                if reach == p.probe_near:
                    blocked_near = blocked
                hit = blocked.astype(np.float64) * weight
                self._rep[:, 0] -= (hit * ca).sum(axis=1)
                self._rep[:, 1] -= (hit * sa).sum(axis=1)
            burn = world.hazard_known[iy, ix].astype(np.float64) * weight
            self._haz[:, 0] -= (burn * ca).sum(axis=1)
            self._haz[:, 1] -= (burn * sa).sum(axis=1)
        self._haz /= _MAX_PROBE_WEIGHT
        np.clip(self._haz, -1.0, 1.0, out=self._haz)
        # Scale by how surrounded the robot is, do NOT normalise to a unit vector.
        # Normalising throws away magnitude, so one probe grazing a wall pushes exactly
        # as hard as being boxed in on seven sides -- and at w_obstacle > w_goal that
        # means a robot in a rubble field spends its mission fleeing walls instead of
        # reaching goals.
        self._rep /= _MAX_PROBE_WEIGHT
        np.clip(self._rep, -1.0, 1.0, out=self._rep)

        # ...but that scaling had a failure of its own, and it was expensive.
        #
        # A robot driving straight at a single wall trips only one or two probes of
        # eight, so it gets a push of ~0.2. Against `w_obstacle` 2.2 that is ~0.44,
        # while the goal term contributes a full 1.0 -- so the goal wins, the robot
        # keeps going, and it arrives nose-first at terrain it cannot cross. The safety
        # floor then correctly zeroes its speed, and it spends the next dozen ticks
        # rotating on the spot. Measured: **52.5% of scout-ticks with a goal were
        # stopped by the override**, and scouts averaged 0.46-0.90 m/s against a v_max
        # of 1.99 (MEASUREMENTS.md M-34).
        #
        # What is in front of you is urgent in a way that flank clearance does not
        # soften. So when anything is blocked within the forward arc, the repulsion is
        # lifted to a magnitude that can actually outvote the goal -- keeping its
        # direction, which the full probe ring has already worked out.
        front = np.abs(_wrap_pi(_RAYS)) <= FRONT_ARC
        blocked_ahead = blocked_near[:, front].any(axis=1)
        mag = np.linalg.norm(self._rep, axis=1)
        need = blocked_ahead & (mag > 1e-9) & (mag < FRONT_MIN_PUSH)
        if need.any():
            self._rep[need] *= (FRONT_MIN_PUSH / mag[need])[:, None]

    def _separation(self, world) -> None:
        """Inverse-square push away from neighbours, O(N^2) but preallocated.

        At N=512 this is ~262k pairs, about 1 ms. It is the term that will break first
        if N grows much further -- replace with spatial binning, not with removal:
        without separation 512 robots collapse into one moving blob.
        """
        x = world.pos[:, 0].astype(np.float32)
        y = world.pos[:, 1].astype(np.float32)
        np.subtract(x[:, None], x[None, :], out=self._dx)
        np.subtract(y[:, None], y[None, :], out=self._dy)
        np.multiply(self._dx, self._dx, out=self._d2)
        self._d2 += self._dy * self._dy
        # Per-robot when a genome is loaded: `formation_spacing` is what makes a tight
        # scouting screen and a loose sweep line different phenotypes rather than the
        # same swarm with different labels. The pairwise test uses the *larger* of the
        # two radii, so a robot that wants space gets it even from one that does not.
        if self._spacing is None:
            r2 = np.float32(self.p.separation_radius ** 2)
        else:
            r2 = np.maximum(self._spacing[:, None], self._spacing[None, :]) ** 2
        w = np.where((self._d2 < r2) & (self._d2 > 1e-6), 1.0 / (self._d2 + 1e-3), 0.0)
        self._sep[:, 0] = (self._dx * w).sum(axis=1)
        self._sep[:, 1] = (self._dy * w).sum(axis=1)
        # Soft-saturate rather than normalise: crowding should push harder than a single
        # neighbour, but a dense cluster must not produce an unbounded force.
        nrm = np.linalg.norm(self._sep, axis=1, keepdims=True)
        self._sep *= np.tanh(nrm) / np.maximum(nrm, 1e-9)

    def set_genes(self, genes) -> None:
        """Adopt evolved per-robot behaviour. `None` restores the hand-tuned scalars."""
        self._spacing = (None if genes is None
                         else np.asarray(genes.formation_spacing, dtype=np.float32))

    def _arrived(self, world, goal_xy, goal_id, stop_radius=None) -> np.ndarray:
        arrived = np.zeros(world.n, dtype=bool)
        if not goal_xy:
            return arrived
        g = np.array(goal_xy, dtype=np.float64)
        has = goal_id >= 0
        if not has.any():
            return arrived
        idx = np.nonzero(has)[0]
        d = np.linalg.norm(world.pos[idx] - g[goal_id[idx]], axis=1)
        r = self.p.arrive_radius if stop_radius is None else stop_radius[idx]
        arrived[idx] = d <= r
        return arrived

    # ------------------------------------------------------------------ safety floor

    def _wall_override(self, world, v: np.ndarray) -> np.ndarray:
        """**The safety floor.** Zero forward speed if the swept circle hits ground this
        robot cannot traverse.

        Rotation is still permitted, so a blocked robot turns in place until it faces a
        free direction rather than grinding. World.step's axis-separated collision would
        also prevent penetration; this exists so robots do not *try*, which is both
        visible on the dashboard and a waste of battery.
        """
        step = v * world.dt
        ca, cb = np.cos(world.theta), np.sin(world.theta)
        blocked = np.zeros(world.n, dtype=bool)
        for frac in (0.5, 1.0):
            reach = world.radius + step * frac
            ix, iy = grid.world_to_cell(
                world.pos[:, 0] + reach * ca, world.pos[:, 1] + reach * cb,
                world.cell, world.shape
            )
            # Chassis-aware: a slope a legged unit walks up is a wall to a wheeled one,
            # and the safety floor has to mean the same thing to both.
            blocked |= ~world.chassis_passable[world.chassis, iy, ix]
        # A robot already standing on ground it cannot traverse must be allowed to move
        # -- otherwise any shove into bad terrain is permanent, since the override would
        # zero it forever and the repulsion term could never push it clear.
        ix0, iy0 = grid.world_to_cell(world.pos[:, 0], world.pos[:, 1],
                                      world.cell, world.shape)
        stranded = ~world.chassis_passable[world.chassis, iy0, ix0]
        blocked &= ~stranded

        self.blocked_count += int(blocked.sum())
        self.stranded_count = int(stranded.sum())
        return np.where(blocked, 0.0, v)


def _wrap_pi(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi

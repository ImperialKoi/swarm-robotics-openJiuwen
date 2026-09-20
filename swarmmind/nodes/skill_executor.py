"""Tier 2 execution: carrying out an assigned task.

Contains **no allocation logic**. Which robot gets which task is the auction's job
(D3, nodes/auction.py); this module only turns an assignment into a navigation goal,
detects completion, and writes the ``last_action_reason`` string that feeds the
dashboard's follow-cam panel and the Tier-2 ticker.

``last_action_reason`` is a template, never an LLM call (contracts/schemas.py).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..contracts.schemas import ACTIVITY
from ..control.planner import UNREACHABLE
from ..sim import grid
from ..sim.robot import CHASSIS_INDEX, LANE_INDEX, OUT_OF_COMMS
from ..sim.world import CARRIED, CLEARED, FOUND, HIDDEN, REACH_DIG, RESCUED
from .tasks import in_known_hazard

#: A sector this well explored stops attracting idle robots. Not 1.0: the last few
#: percent of a sector are usually cells no chassis can stand in, so a swarm told to
#: finish them never stops walking at ground it cannot reach.
IDLE_EXPLORE_UNTIL = 0.96

#: An idle robot under this charge walks to the base pad instead of at dark ground.
#:
#: Deliberately only on the *idle* path. A low-battery reflex that overrides live work
#: would send robots on 400 m round trips across the map mid-task, and with the endurance
#: floor in `World` no robot dies of movement drain anyway -- so charging is insurance
#: and a visible behaviour, not a rescue. It costs nothing because the robots taking it
#: had no task to abandon.
#: Within this of its post, a relay counts as holding it rather than walking to it.
RELAY_ON_POST_M = 3.0

RECHARGE_SEEK_BELOW = 0.55

#: And only if the pad is worth the walk. Beyond this a robot is better off drifting at
#: unexplored ground on the charge it has.
RECHARGE_SEEK_RANGE = 140.0

#: How often the idle-target set is rebuilt, in seconds. Exploration moves slowly and
#: the scan is over every passable cell, so recomputing it on every `goals()` rebuild
#: would put an 86,400-cell pass inside the tick loop -- which is how M-58 happened.
DARK_REFRESH_S = 2.0

#: Which actuator each task kind requires. Mirrors contracts.schemas.TASK_REQUIRES;
#: the contract is the definition, this is the execution-side lookup.
REQUIRES = {
    "explore": None,
    "investigate": None,
    "retreat": None,
    "clear_debris": "scoop",
    "extract": "gripper",
    "relay": "antenna",
}


@dataclass
class Assignment:  # mutable: orphaning and opportunistic pickup both rewrite it
    task_id: str
    kind: str
    target: tuple[float, float]
    victim: int | None = None
    report: str | None = None
    assigned_at: float = 0.0
    #: The auctioneer has given up hearing from this robot and re-offered the work.
    #: The robot keeps executing -- it does not know it was written off -- so the task
    #: may be done twice. That duplicate effort is what really happens when a machine
    #: goes out of contact, and it is cheaper than assuming the work is lost.
    orphaned: bool = False
    progress_at: float = 0.0
    last_dist: float = float("inf")


#: Release a task after this long **without progress**, not after this long in total.
#:
#: A wall-clock deadline is the wrong mechanism here: a carrier crossing a 96 m map at
#: 0.72 m/s legitimately needs minutes, so any cap short enough to free a genuinely
#: stuck robot also fires mid-carry on a healthy one. Distance-to-goal decreasing is
#: the signal that separates the two.
NO_PROGRESS_AFTER = {"explore": 20.0, "investigate": 25.0, "relay": 25.0, "retreat": 15.0,
                     "clear_debris": 30.0, "extract": 30.0}

#: Distance-to-goal must improve by at least this much to count as progress.
PROGRESS_EPS = 0.5


class SkillExecutor:
    """Per-robot task execution for the whole swarm."""

    def __init__(self, world) -> None:
        self.assignment: list[Assignment | None] = [None] * world.n
        self.reason: list[str] = ["idle at base"] * world.n
        self._dirty = True
        self._goal_xy: list[tuple[float, float]] = []
        self._goal_id = np.full(world.n, -1, dtype=np.int32)
        #: `False`, and measured that way. Completing an explore task only when its own
        #: holder arrives is the obvious reading of the defect POSSIBLE_BUG3 §2 describes,
        #: and it does exactly what it claims -- assignment-ends 4277 -> 2400, median hold
        #: 18 -> 22 s, idle robots 48 -> 40 -- **and costs 2.5 rescues, down on 3 of 4
        #: seeds** (M-59).
        #:
        #: The churn is load-bearing. Retiring on the shared map is continuous re-planning
        #: in disguise: a robot released mid-walk immediately re-bids on the frontier that
        #: is best *now*, while committing it to arrival makes it walk to a target that
        #: may already be stale. The harm from churn is not the churn, it is that a
        #: released robot in contact is given no goal and stops dead -- which is a
        #: separate defect and the one worth fixing.
        self.explore_needs_arrival = False
        #: Idle relays walk toward cut-off robots instead of standing still.
        self.roaming_relays = True
        #: Give every idle robot somewhere to be. Without it an idle, in-contact robot
        #: that is not a relay falls off the end of `goals()` with no goal at all and
        #: stands still: measured on seed 42, 84-131 robots of 512 at every sample, and
        #: the swarm's mean speed halves over a mission as more of it parks.
        self.idle_explore = True
        self._dark_pts: dict[int, np.ndarray] = {}
        #: chassis -> connected-component label per cell. Terrain never changes, so this
        #: is built once on first use and reused for the whole mission.
        self._reach: dict[int, np.ndarray] = {}
        #: What each robot is doing, as an `ACTIVITY` code, for the dashboard. Filled in
        #: `goals()` rather than recomputed by the bridge: that loop already walks every
        #: robot to set `reason`, so this costs nothing, and deriving it at bridge rate
        #: would put a 512-iteration Python loop at 10 Hz next to the tick loop.
        self.activity = np.zeros(len(world.pos), dtype=np.int8)
        self._dark_at = -1e9
        #: Carriers deliver to the nearest *reachable* collection point.
        #: False = straight-line nearest, which strands them across rivers.
        self.reachable_zones = True
        self._carrying = world.carrying.copy()
        self._in_comms = world.in_comms.copy()
        self._refresh_at = -1e9
        self._priorities = world.sector_priority.copy()
        self._abandoned = world.sector_abandoned.copy()
        #: Goals set by the unit policy (`control/unit_policy.py`) for robots with no
        #: auction assignment: `{robot: ((x, y), reason, expires_at)}`. Empty unless a
        #: unit policy is running, in which case every branch that reads it is a no-op and
        #: the mission is byte-identical to the shipped one.
        #:
        #: A robot holding one is *reserved*: the auction will not hand it search work,
        #: but it stays eligible for rescue-tier work. That is the point of staging -- a
        #: carrier parked beside a dig must still be there, and free, when the casualty
        #: comes out.
        self.policy_goal: dict[int, tuple[tuple[float, float], str, float]] = {}

    # ------------------------------------------------------------------ assignment

    def assign(self, world, i: int, a: Assignment, why: str = "") -> None:
        a.progress_at = world.t
        a.last_dist = float("inf")
        # Real work outranks a policy hold, always.
        self.policy_goal.pop(i, None)
        self.assignment[i] = a
        self._dirty = True
        lane = _lane_of(world, i)
        self.reason[i] = why or f"{lane} assigned {a.kind} ({a.task_id})"

    def release(self, i: int, why: str = "task complete") -> None:
        self.assignment[i] = None
        self.reason[i] = why
        self._dirty = True

    #: How close Tier 1 should park a robot for each task kind. Interaction tasks must
    #: stop INSIDE the world's reach radius (REACH_GRAB / REACH_DIG) or the robot sits
    #: just outside pickup range forever; navigation tasks stop at the general radius.
    STOP_RADIUS = {"extract": 0.6, "clear_debris": 0.6}

    def stop_radii(self, world, arrive_radius: float) -> np.ndarray:
        r = np.full(world.n, arrive_radius, dtype=np.float64)
        for i, a in enumerate(self.assignment):
            if a is not None and a.kind in self.STOP_RADIUS:
                r[i] = self.STOP_RADIUS[a.kind]
        return r

    def mark_dirty(self) -> None:
        self._dirty = True

    def free_mask(self, world) -> np.ndarray:
        alive = world.status <= OUT_OF_COMMS
        return alive & (world.carrying < 0) & np.array([a is None for a in self.assignment])

    def set_policy_goal(self, i: int, goal: tuple[float, float], why: str,
                        expires_at: float) -> None:
        self.policy_goal[int(i)] = ((float(goal[0]), float(goal[1])), why, float(expires_at))
        self.reason[i] = why
        self._dirty = True

    def clear_policy_goal(self, i: int) -> None:
        if self.policy_goal.pop(int(i), None) is not None:
            self._dirty = True

    def reserved_mask(self, world) -> np.ndarray:
        """Robots holding a unit-policy goal. All False when no policy is running."""
        m = np.zeros(world.n, dtype=bool)
        if self.policy_goal:
            m[sorted(self.policy_goal)] = True
        return m

    # ------------------------------------------------------------------ goals

    def goals(self, world, nav=None) -> tuple[list[tuple[float, float]], np.ndarray]:
        """(distinct goal positions, per-robot index into that list; -1 = none).

        Deduplicating goals is what lets Tier 1 compute one flow field per goal instead
        of one per robot.

        Cached. Rebuilding this walks every robot in Python, which at 512 robots and
        20 Hz is 10k iterations a second inside the tick loop -- exactly what the
        scaling rules forbid. Assignments, payloads and changed orders invalidate the
        cache immediately; idle fog/link targets also refresh every DARK_REFRESH_S.
        """
        if not np.array_equal(self._carrying, world.carrying):
            self._carrying = world.carrying.copy()
            self._dirty = True
        # Comms state is a goal input now that recovery aims at the nearest link rather
        # than at a fixed point, so a change in it has to invalidate the cache. Without
        # this a robot keeps steering at a link that moved, or at one it already reached.
        if not np.array_equal(self._in_comms, world.in_comms):
            self._in_comms = world.in_comms.copy()
            self._dirty = True
        # Unassigned movement is still work: refresh when fog/links move even when
        # the auction has no new award to invalidate this cache. Orders apply now.
        orders_changed = (not np.array_equal(self._priorities, world.sector_priority)
                          or not np.array_equal(self._abandoned, world.sector_abandoned))
        if orders_changed or world.t >= self._refresh_at:
            self._dirty = True
            self._refresh_at = world.t + DARK_REFRESH_S
            self._priorities = world.sector_priority.copy()
            self._abandoned = world.sector_abandoned.copy()
            self._dark_at = -1e9
        if not self._dirty:
            return self._goal_xy, self._goal_id
        self._dirty = False

        goal_xy: list[tuple[float, float]] = []
        index: dict[tuple[float, float], int] = {}
        goal_id = np.full(world.n, -1, dtype=np.int32)

        idle = np.array(
            [i for i, a in enumerate(self.assignment)
             if a is None and world.status[i] <= OUT_OF_COMMS and world.in_comms[i]],
            dtype=int)

        # Idle and low on charge outranks both roaming and drifting: a robot with no
        # task loses nothing by topping up, and it is the only thing that ever uses the
        # base pad -- carriers deliver to the *nearest* collection point, which is rarely
        # base, so without this the recharge pad fires zero times in a mission.
        charge: dict[int, tuple[float, float]] = {}
        if len(idle):
            pad = np.asarray(world.scn.base, dtype=float)
            low = world.battery[idle] < RECHARGE_SEEK_BELOW
            near = np.linalg.norm(world.pos[idle] - pad, axis=1) < RECHARGE_SEEK_RANGE
            charge = {int(j): (float(pad[0]), float(pad[1]))
                      for j in idle[low & near]}

        # Relays heading for the pad are excluded, not merely overridden later. The
        # recharge branch below outranks the lane's orders, so a claimed relay that goes
        # to charge instead leaves the robot it claimed with nobody coming -- the claim
        # is exclusive, so no second relay takes that robot either. Silent, and exactly
        # the kind of leak the exclusivity is supposed to prevent.
        roam: dict[int, tuple[tuple[float, float], str]] = {}
        if self.roaming_relays and len(idle):
            avail = np.array([i for i in idle
                              if world.actuator[i] == LANE_INDEX["antenna"]
                              and i not in charge], dtype=int)
            roam = _relay_orders(world, avail)

        # Everyone the auction could not employ, and every relay with no gap left to
        # bridge, walks at the nearest unexplored ground rather than standing still.
        drift: dict[int, tuple[float, float]] = {}
        if self.idle_explore and len(idle):
            spare = np.array([i for i in idle if i not in roam and i not in charge
                              and i not in self.policy_goal], dtype=int)
            drift = _drift_targets(world, spare, self._dark_points(world),
                                   self._reach_labels(world))

        # Casualty staging: an idle carrier or digger waits beside the casualty it can
        # act on next, instead of walking at dark ground.
        #
        # The auction can only employ a robot when a task exists, and a casualty task
        # exists only once the casualty reaches the stage that needs that lane -- so a
        # carrier spends the dig walking at a frontier, and when the victim clears the
        # response is a walk back across the map. Measured at t=180 on seed 42: 43 of
        # 96 carriers and 96 of 117 diggers on search work while 20 cleared casualties
        # waited, and the mean rescue took 92.7 s. Staging is unassigned movement, the
        # same contract as drift and the relay orders: the robot stays free, bids on
        # the real task the moment it is announced, and is already standing there.
        stage: dict[int, tuple[tuple[float, float], str]] = {}
        if len(idle):
            spare_stage = np.array([i for i in idle if i not in roam and i not in charge
                                    and i not in self.policy_goal], dtype=int)
            stage = _casualty_staging(world, spare_stage, self._reach_labels(world))
        # Last resort before standing still: any lane with nothing to do walks to a
        # known casualty. At 99% explored the dark-ground targets run out long before
        # the free robots do -- measured at t=180 on seed 42: 126 robots idle, 22
        # cleared casualties waiting, and the two facts had nothing to do with each
        # other. A scout cannot carry or dig, but it can be standing where the rescue
        # is, which is both a pair of eyes and what an operator expects to see.
        support: dict[int, tuple[tuple[float, float], str]] = {}
        if len(idle):
            spare_support = np.array([i for i in idle if i not in roam and i not in charge
                                      and i not in self.policy_goal], dtype=int)
            support = _support_staging(world, spare_support, self._reach_labels(world))

        for i, a in enumerate(self.assignment):
            if world.status[i] > OUT_OF_COMMS:
                continue
            if a is None:
                # Comms-recovery reflex. A robot outside the component cannot be
                # awarded work, so an idle one would otherwise sit where it is
                # forever -- and because its discoveries stay buffered, the shared
                # map stops growing, the frontier freezes, and the whole swarm
                # deadlocks. Walking back toward contact needs no coordination.
                #
                # **Toward the nearest robot still in contact, not toward base.**
                # Aiming at base sent a robot 286 m out on a three-minute walk home to
                # regain a link that was often tens of metres away, and it dragged it
                # off the frontier -- which is where the unexplored ground is. Measured
                # at t=420 on seed 42: 30 idle robots out of contact, median 281 m from
                # base, every one of them trudging back to the corner (M-53).
                if not world.in_comms[i]:
                    g = _nearest_contact(world, i)
                    self.reason[i] = "out of contact, closing on the nearest link"
                    self.activity[i] = ACTIVITY["recover"]
                elif (tgt := charge.get(i)) is not None:
                    # Unemployed and low: top up rather than drift.
                    g = tgt
                    self.reason[i] = "idle and low on charge, returning to base"
                    self.activity[i] = ACTIVITY["recharge"]
                elif (order := roam.get(i)) is not None:
                    # The antenna lane working its own job: repair a broken link, or
                    # shore up one about to break. `_relay_orders` carries the reason
                    # because which rung a relay is on is the interesting thing to show
                    # on the dashboard -- "holding position" told nobody anything.
                    #
                    # Relays already extend the network wherever they stand: the comms
                    # graph is built from every living antenna, not only the ones holding
                    # a post. Standing still inside a net that already reaches you adds
                    # nothing, so a relay with no link to mend falls through to the drift
                    # branch below and goes looking at dark ground instead.
                    g, self.reason[i] = order
                    self.activity[i] = ACTIVITY["relay_move"]
                elif (pg := self.policy_goal.get(i)) is not None:
                    # The unit policy chose where this robot waits or looks. Shown as
                    # drift: it is unassigned movement, and the contract is frozen.
                    g, self.reason[i] = pg[0], pg[1]
                    self.activity[i] = ACTIVITY["drift"]
                elif (stg := stage.get(i)) is not None:
                    # No task yet, so wait where the next one will be.
                    g, self.reason[i] = stg
                    self.activity[i] = ACTIVITY["drift"]
                elif (tgt := drift.get(i)) is not None:
                    # No task, so go where the map is still dark.
                    #
                    # This branch used to be `continue` -- no goal at all -- and it is
                    # the single largest source of stopped robots in the swarm. The
                    # auction can only employ as many robots as it has tasks, and task
                    # supply is bounded by design (48 frontier targets, one sweep point
                    # per sector, one task per casualty), so at 512 robots there are
                    # always far more free robots than tasks. Measured on seed 42: 84 to
                    # 131 robots idle at every sample, mean swarm speed halving from 61%
                    # to 33% of v_max over a mission as more of them parked.
                    #
                    # Targets are shared, not per-robot: one aim point per under-explored
                    # sector, so Tier 1 still builds one flow field per goal. Spreading
                    # search *thinly* is measurably worse than overlapping it -- 3x3
                    # sweep points per sector took rescues 13 -> 5 (M-14), and claiming
                    # search targets took 13 -> 5 again (M-36) -- so this deliberately
                    # sends many robots at few points.
                    g = tgt
                    tx, ty = _cell(world, tgt)
                    priority = world.sector_priority[world.sector_of_cell[ty, tx]]
                    self.reason[i] = (f"following priority search in {_sector(world, tgt)}"
                                      if priority == 0 else "no task, closing on unexplored ground")
                    self.activity[i] = ACTIVITY["drift"]
                elif (sup := support.get(i)) is not None:
                    # Nothing left to search, but a casualty still wants eyes on it.
                    g, self.reason[i] = sup
                    self.activity[i] = ACTIVITY["drift"]
                else:
                    self.reason[i] = "no reachable search work; awaiting assignment"
                    self.activity[i] = ACTIVITY["idle"]
                    continue
                if "holding the only link" not in self.reason[i]:
                    g = _dry_standing_goal(world, i, g, self._reach_labels(world))
            else:
                self.activity[i] = _activity_of(world, i, a)
                g = self._goal_for(world, i, a, nav)
            if g is None:
                continue
            # Preserve cell centres and interaction coordinates. Rounding a dry bank
            # target onto a cell boundary could put it in the neighbouring river.
            # NavFields already deduplicates distance fields by coarse goal cell.
            key = (float(g[0]), float(g[1]))
            gi = index.get(key)
            if gi is None:
                gi = len(goal_xy)
                index[key] = gi
                goal_xy.append(key)
            goal_id[i] = gi
        self._goal_xy, self._goal_id = goal_xy, goal_id
        return goal_xy, goal_id

    def _goal_for(self, world, i: int, a: Assignment,
                  nav=None) -> tuple[float, float] | None:
        if a.kind == "extract" and a.victim is not None:
            if world.carrying[i] == a.victim:
                return _nearest_zone(world, world.pos[i],
                                     nav if self.reachable_zones else None,
                                     int(world.chassis[i]))
            return tuple(world.victims[a.victim].pos)
        if a.kind == "clear_debris" and a.victim is not None:
            return tuple(world.victims[a.victim].pos)
        return a.target

    # ------------------------------------------------------------------ progress

    def update(self, world, nav=None, arrive_radius: float = 1.5,
               full: bool = True) -> list[str]:
        """Retire finished or invalidated assignments. Returns completed task ids.

        Runs every tick, but only the *detection* is per-robot and it is vectorised;
        the Python loop walks the handful of robots that actually finished. Doing the
        whole scan at 20 Hz costs 10k iterations a second at 512 robots; doing it at
        1 Hz instead makes every robot idle up to a second after arriving, which at
        short trip lengths cut task throughput by 4x and stalled exploration near base.

        ``full=True`` additionally runs the stall scan and refreshes narration; the
        caller does that at the auction rate.
        """
        done: list[str] = []
        touch = self._candidates(world, arrive_radius, nav) if not full else range(world.n)
        # Tier 1 stops the robot within arrive_radius of a goal that has been snapped to
        # the cell grid, so completion must tolerate that snap. Requiring the tighter
        # radius here deadlocks: the robot parks in the gap, stops moving, and holds the
        # task forever.
        reach = arrive_radius + world.cell
        for i in touch:
            if world.status[i] > OUT_OF_COMMS:
                # Dead robots keep their assignment until the auction times them out;
                # self-healing is the auction's story to tell, not ours.
                continue
            a = self._adopt_carried(world, int(i), self.assignment[i])
            if a is None:
                continue
            if full and self._stalled(world, nav, i, a):
                self.release(i, f"{_lane_of(world, i)} abandoned stalled {a.kind}")
                continue
            state = self._state(world, i, a, reach)
            if state == "done":
                done.append(a.task_id)
                self.release(i, f"{_lane_of(world, i)} completed {a.kind}")
            elif state == "invalid":
                self.release(i, f"{_lane_of(world, i)} dropped {a.kind}: no longer needed")
            elif full:
                self._narrate(world, i, a)
        return done

    def _adopt_carried(self, world, i: int, a: Assignment | None) -> Assignment | None:
        """Deliver the casualty actually held, after any immediate hazard retreat.

        Pickup is proximity-based in the world -- a gripper standing next to a cleared
        casualty picks them up, which is the right behaviour. But the *task* did not
        follow, so a carrier that grabbed someone while investigating a contact would
        carry them to the contact and onward forever, never to an extraction zone.
        """
        held = int(world.carrying[i])
        if held < 0 or (a is not None and (
                a.kind == "retreat" or (a.kind == "extract" and a.victim == held))):
            return a
        if a is None:
            a = Assignment(f"extract_pickup_{world.robot_ids[i]}_{held}", "extract",
                           tuple(world.pos[i]), assigned_at=world.t)
        # An extract assignment may name a different casualty picked up in passing.
        # Retaining it sends a loaded carrier to work it cannot collect.
        a.kind = "extract"
        a.victim = held
        a.target = tuple(world.pos[i])
        a.report = None
        self.assign(world, i, a,
                    why=f"{_lane_of(world, i)} picked up {world.victims[held].id} "
                        "in passing, diverting to extraction")
        return a

    def _candidates(self, world, arrive_radius: float, nav=None) -> np.ndarray:
        """Robots worth inspecting this tick: those at their goal, plus anyone whose
        victim changed state. Vectorised; typically a handful of indices."""
        goal_xy, goal_id = self.goals(world, nav)
        carrying = np.flatnonzero(world.carrying >= 0).astype(np.int32)
        has = goal_id >= 0
        if not has.any():
            return carrying
        g = np.array(goal_xy, dtype=np.float64)
        idx = np.nonzero(has)[0]
        d = np.linalg.norm(world.pos[idx] - g[goal_id[idx]], axis=1)
        arrived = idx[d <= arrive_radius + world.cell]
        # Victim-driven transitions are found by walking victims (tens), not robots.
        vic = [i for i, a in enumerate(self.assignment)
               if a is not None and a.victim is not None
               and world.victims[a.victim].state in (CLEARED, CARRIED, RESCUED)]
        return np.unique(np.concatenate(
            [arrived, np.array(vic, dtype=np.int32), carrying]))

    def _stalled(self, world, nav, i: int, a: Assignment) -> bool:
        """True if the robot has not closed on its goal for too long.

        Distance is **geodesic** (read off the goal's flow field), not euclidean. A robot
        routing around an obstacle legitimately increases its straight-line distance, so
        a euclidean test calls a healthy detour a stall and releases the task mid-journey.

        The reference is the distance at the last checkpoint, not the best ever seen:
        with a min-ever reference a robot that detours can never re-establish progress.

        A digger actively reducing debris counts as progress even while standing still,
        and so does a carrier holding a victim -- both are working, not stuck.
        """
        working = (
            (a.kind == "extract" and world.carrying[i] == a.victim)
            or (a.kind == "clear_debris" and a.victim is not None
                and not world.airborne[i]
                and world.victims[a.victim].state == FOUND
                and world.victims[a.victim].debris_remaining > 0.0
                and _dist(world.pos[i], world.victims[a.victim].pos) <= REACH_DIG)
        )
        if working:
            a.progress_at = world.t
            return False
        goal = self._goal_for(world, i, a, nav)
        if goal is not None:
            # A robot standing on its goal is not stalled -- it has arrived. A relay
            # holding its post is the whole point of the relay lane, and without this
            # every relay is released 25 s after reaching station.
            if _dist(world.pos[i], goal) <= self.STOP_RADIUS.get(a.kind, 1.5) + world.cell:
                a.progress_at = world.t
                return False
            if nav is not None:
                c = int(world.chassis[i])
                field = nav.field(c, goal[0], goal[1])
                d = float(nav.distance_at(c, field, world.pos[i, 0:1], world.pos[i, 1:2])[0])
                if d >= UNREACHABLE * 0.5:
                    # The coarse grid says this robot is nowhere. It is somewhere: the
                    # nav grid is a 4x downsample with a majority rule (M-23), so a robot
                    # can legitimately stand on a fine cell inside a coarse cell marked
                    # impassable for its chassis, and read UNREACHABLE for its own
                    # position.
                    #
                    # Counting that as "no progress" is how the stall detector came to
                    # kill **300 of 310 relay post assignments at a median 106 m from the
                    # post** (M-55): a relay crossing 300 m of rubble takes enough of
                    # these readings to burn the 25 s window, gets released, is
                    # reassigned, and starts again. The lane spent the mission commuting.
                    #
                    # No reading is no evidence. The clock does not run on it.
                    a.progress_at = world.t
                    return False
            else:
                d = _dist(world.pos[i], goal)
            if d < a.last_dist - PROGRESS_EPS:
                a.progress_at = world.t
            # The reference is the distance at the **last checkpoint**, which is what the
            # docstring above has always said and what the code did not do: it kept
            # `min(a.last_dist, d)`, a best-ever reference. Under that rule a robot that
            # detours can never re-establish progress until it beats its old best, so any
            # detour longer than the no-progress window is fatal.
            #
            # It is fatal mostly to relays, whose posts are the longest walks in the
            # mission. Measured on seed 42: **300 of 310 relay post assignments ended in
            # `stalled`, at a median 106 m from the post**, only 3 of 310 arriving -- so
            # the lane spent the mission being reassigned instead of building the chain,
            # and comms reach sat at 309 m (M-55).
            a.last_dist = d
        return world.t - a.progress_at > NO_PROGRESS_AFTER.get(a.kind, 30.0)

    def _dark_points(self, world) -> dict[int, np.ndarray]:
        """chassis index -> (k, 2) aim points, one per sector still under-covered.

        **Aim points are real dark cells, not centroids of them.** The mean of a sector's
        unexplored cells is not itself guaranteed to be one: a ring of dark ground around
        a rock cluster has its centroid *inside the rock*. Every idle robot in that
        sector then walks at a point it cannot stand on and piles against the obstacle --
        which is what a crowd stacked along one wall on the dashboard actually is. Every
        other target generator in this codebase snaps for the same reason
        (`_chain_targets`, relay posts, sweep points all call `snap_passable`); picking a
        member of the set is stronger than snapping, because the result is guaranteed
        both passable *and* unexplored rather than merely passable.

        **And it is per chassis.** A wheeled unit sent at dark ground beyond a river
        stalls at the bank in exactly the same heap. `chassis_passable` is what the flow
        fields are built from, so drift has to agree with it or it aims robots at ground
        their own locomotion rules out.

        Rebuilt at most every `DARK_REFRESH_S`: the scan touches every passable cell and
        `goals()` rebuilds on most ticks, so doing this per rebuild would put an
        86,400-cell pass in the hot loop -- which is how M-58 happened.
        """
        if world.t - self._dark_at < DARK_REFRESH_S:
            return self._dark_pts
        self._dark_at = float(world.t)
        self._dark_pts = {}

        pct = np.asarray(world.sector_explored_pct, dtype=float)
        want = (pct < IDLE_EXPLORE_UNTIL) & ~world.sector_abandoned
        if not want.any():
            return self._dark_pts

        # Search stops belong on land. Legged robots may traverse water, but a
        # river cell is not a useful place to park and survey for ground casualties.
        base_dark = world.passable & ~world.explored & (world.water == 0) & ~world.hazard_known
        for c in range(len(world.chassis_passable)):
            dark = base_dark & world.chassis_passable[c]
            iy, ix = np.nonzero(dark)
            if len(ix) == 0:
                continue
            sec = world.sector_of_cell[iy, ix].astype(np.int64)
            keep = want[sec]
            if not keep.any():
                continue
            iy, ix, sec = iy[keep], ix[keep], sec[keep]
            gx = (ix + 0.5) * world.cell
            gy = (iy + 0.5) * world.cell

            n = len(world.sector_ids)
            cnt = np.bincount(sec, minlength=n).astype(float)
            cx = np.bincount(sec, weights=gx, minlength=n)
            cy = np.bincount(sec, weights=gy, minlength=n)
            nz = cnt > 0
            cx[nz] /= cnt[nz]
            cy[nz] /= cnt[nz]

            # The dark cell nearest its own sector's dark centroid: the middle of the
            # unexplored ground, and a cell a robot of this chassis can actually occupy.
            d2 = (gx - cx[sec]) ** 2 + (gy - cy[sec]) ** 2
            order = np.lexsort((d2, sec))
            first = np.unique(sec[order], return_index=True)[1]
            pick = order[first]
            self._dark_pts[c] = np.stack([gx[pick], gy[pick]], axis=1)
        return self._dark_pts

    def _reach_labels(self, world) -> dict[int, np.ndarray]:
        """chassis -> connected-component id per cell; -1 where that chassis cannot go.

        `chassis_passable` says whether a robot could *stand* on a cell, which is not the
        same as whether it can *get there*. Dark ground across a river is passable to a
        wheeled unit on both banks and reachable from neither, so aiming one at it parks
        it on the near bank -- the same heap as an impassable target, one step removed.
        The auction already makes this distinction, bidding `inf` for a robot whose
        locomotion cannot reach a task (`AuctionNode._best_bidder`); drift has to make it
        too or it re-creates the problem the auction was fixed for.

        Components rather than per-target distance fields on purpose: a field costs
        22.4 ms (M-4) and there are up to 48 targets x 4 chassis, which is seconds of
        compute per refresh. Terrain is static, so this is built once and answered with
        an array lookup thereafter.
        """
        if self._reach:
            return self._reach
        from ..sim.world import NAV_DOWNSAMPLE

        f = NAV_DOWNSAMPLE
        h, w_ = world.shape
        for c in range(len(world.chassis_passable)):
            if c == CHASSIS_INDEX["rotor"]:
                # Match NavSet's airspace: disconnected landing sites remain
                # mutually reachable, including from a rotor currently over water.
                self._reach[c] = np.zeros(world.shape, dtype=np.int32)
                continue
            # Labelled on the *coarse* grid, because that is the grid the flow fields are
            # built from. A pocket connected at 1 m resolution can have no route at 4 m,
            # and `World._navigable` records that exact trap for casualty placement.
            ok = grid.downsample(world.chassis_passable[c], f)
            edges = grid.coarse_edge_masks(world.chassis_passable[c], f)
            lab = np.full(ok.shape, -1, dtype=np.int32)
            nxt = 0
            ys, xs = np.nonzero(ok)
            for y, x in zip(ys, xs, strict=True):
                if lab[y, x] >= 0:
                    continue
                lab[np.isfinite(grid.distance_field(ok, (int(x), int(y)), edges))] = nxt
                nxt += 1
            self._reach[c] = np.repeat(np.repeat(lab, f, axis=0), f, axis=1)[:h, :w_]
        return self._reach

    def _state(self, world, i: int, a: Assignment, arrive_radius: float) -> str:
        if a.kind == "explore":
            # "This ground is now known" is not "this robot's job is done".
            #
            # The shared-map test retires an explore task the moment *anybody* reveals
            # the target cell, so a robot walking to a frontier is released where it
            # stands. Measured on seed 42: 60% of explore assignments end `completed`, a
            # median **88 m from the target**, having been held a median of **1.0 s**;
            # only 7% end within 5 m of the goal. `investigate`, which completes on its
            # own holder arriving, ends within 5 m **58%** of the time (M-57).
            #
            # The released robot is then idle and in contact, which `goals()` gives no
            # goal at all, so it stops dead until the auction re-employs it. That is the
            # churn underneath the whole dwell problem (M-56).
            if not self.explore_needs_arrival:
                ix, iy = _cell(world, a.target)
                if world.explored[iy, ix]:
                    return "done"
            if _dist(world.pos[i], a.target) <= arrive_radius:
                return "done"
            return "running"

        if a.kind in ("retreat", "investigate"):
            # Arriving IS the whole task: the tracker resolves the report the moment a
            # robot is close enough to see whether anyone is actually there.
            return "done" if _dist(world.pos[i], a.target) <= arrive_radius else "running"

        if a.kind == "relay":
            # A relay that reaches its post HOLDS it. Reporting "done" on arrival frees
            # the robot, it gets an explore task, walks off, and the comms chain behind
            # it collapses -- taking the whole swarm out of contact with it.
            #
            # But holding unconditionally is how relays burn: the hazard grows over the
            # post and a robot with no completion condition sits in it until its battery
            # is gone. A post inside observed hazard is abandoned.
            ix, iy = _cell(world, a.target)
            if world.hazard_known[iy, ix]:
                return "invalid"
            return "running"

        v = world.victims[a.victim] if a.victim is not None else None
        if v is None:
            return "invalid"

        if a.kind == "clear_debris":
            if v.state == HIDDEN:
                return "invalid"
            return "done" if v.state >= CLEARED else "running"

        if a.kind == "extract":
            if v.state == RESCUED:
                return "done"
            if v.state < CLEARED:
                return "invalid"
            if v.state == CARRIED and v.carrier != i:
                return "invalid"          # another carrier got there first
            return "running"
        return "running"

    def _narrate(self, world, i: int, a: Assignment) -> None:
        lane = _lane_of(world, i)
        if a.kind == "extract" and a.victim is not None:
            v = world.victims[a.victim]
            if world.carrying[i] == a.victim:
                self.reason[i] = f"{lane} carrying {v.id} to the nearest extraction zone"
            else:
                self.reason[i] = f"{lane} en route to {v.id} for extraction"
        elif a.kind == "clear_debris" and a.victim is not None:
            v = world.victims[a.victim]
            pct = int(100 * (1.0 - v.debris_remaining))
            self.reason[i] = f"{lane} clearing debris over {v.id} ({pct}%)"
        elif a.kind == "explore":
            self.reason[i] = f"{lane} exploring frontier in {_sector(world, a.target)}"
        elif a.kind == "investigate":
            self.reason[i] = f"{lane} investigating a contact in {_sector(world, a.target)}"
        elif a.kind == "relay":
            verb = "holding" if _dist(world.pos[i], a.target) <= 1.5 else "moving to"
            self.reason[i] = f"{lane} {verb} relay position in {_sector(world, a.target)}"
        elif a.kind == "retreat":
            self.reason[i] = f"{lane} retreating from hazard"


# --- helpers --------------------------------------------------------------------------


def _lane_of(world, i: int) -> str:
    from ..contracts.schemas import LANE_LABEL
    from ..sim.robot import LANES

    return LANE_LABEL[LANES[int(world.actuator[i])]]


def _cell(world, p) -> tuple[int, int]:
    ix, iy = grid.world_to_cell(np.asarray(p[0]), np.asarray(p[1]), world.cell, world.shape)
    return int(ix), int(iy)


def _sector(world, p) -> str:
    ix, iy = _cell(world, p)
    return world.sector_ids[int(world.sector_of_cell[iy, ix])]


def _dist(a, b) -> float:
    return float(np.hypot(a[0] - b[0], a[1] - b[1]))


def _load_bearing(world, relays: np.ndarray) -> set[int]:
    """Relays that are the sole link for at least one robot, and so must not move.

    A robot in the component reaches base through base itself or through some relay
    within `relay_radius`. If exactly one relay covers it and base does not, that relay
    *is* its post: walking away disconnects it, and a disconnected robot cannot be given
    work at all until somebody comes back for it (M-64).

    Only relays that are candidates for reassignment are considered -- one already
    holding an auctioned post is not going anywhere anyway.
    """
    alive = world.status <= OUT_OF_COMMS
    is_relay = world.actuator == LANE_INDEX["antenna"]
    base = np.asarray(world.scn.base, dtype=float)
    br, rr = world.scn.comms.base_radius, world.scn.comms.relay_radius

    linked = np.nonzero(alive & is_relay & world.in_comms)[0]
    if len(linked) == 0:
        return set()
    # Robots that need a relay at all: in contact, not a relay, and out of base range.
    dep = np.nonzero(alive & world.in_comms & ~is_relay
                     & (((world.pos - base) ** 2).sum(axis=1) > br * br))[0]
    if len(dep) == 0:
        return set()

    within = (((world.pos[dep][:, None, :] - world.pos[linked][None, :, :]) ** 2)
              .sum(axis=2) <= rr * rr)
    sole = within.sum(axis=1) == 1
    if not sole.any():
        return set()
    keep = {int(linked[j]) for j in np.argmax(within[sole], axis=1)}
    return keep & {int(i) for i in relays}


def _relay_orders(world, relays: np.ndarray
                  ) -> dict[int, tuple[tuple[float, float], str]]:
    """The antenna lane's standing orders, in priority order. `{robot: (goal, why)}`.

    Every other lane is told what to do by the auction. The antenna lane mostly is not:
    it is barred from search work by `_INFRASTRUCTURE_EXEMPT`, and there are far more
    relays alive (~126) than there are relay posts to hold, so most of the lane spends
    most of the mission with no assignment at all. What it does in that time was, until
    this function, either "stand still" or a patch. These are the lane's actual orders:

    1. **Repair.** Somebody is cut off *now*. Become the link that reaches them. A robot
       out of the component cannot be given work at all, so each one recovered is a robot
       returned to the swarm, not merely a report delivered (M-64).
    2. **Reinforce.** Somebody's only link is at the edge of range and about to break.
       Insert a hop before it does. This is the half the lane never had: it acted only
       once contact was already lost, which is the one moment the robot it must reach can
       no longer be coordinated with.
    3. **Explore.** Nothing to repair, nothing to shore up: the caller sends what is left
       to dark ground through `_drift_targets`, which aims at the frontier and creeps
       relays rather than marching them (see its rule 2).

    The rungs are strictly ordered and each consumes relays exclusively -- one relay per
    needy robot, claimed nearest-first, in robot-id order for determinism (invariant #6).
    That exclusivity is the fix for the pile in M-67: asking every relay independently for
    *the nearest* robot in trouble puts 72 of them on one point, and a pile of relays is
    worth exactly one relay.
    """
    if len(relays) == 0:
        return {}
    out: dict[int, tuple[tuple[float, float], str]] = {}
    reach = world.scn.comms.relay_radius * 0.8

    # 0. Hold. A relay does not need a post to *be* a post: the comms graph is built from
    # every living antenna, so one standing in the right place is load-bearing whether or
    # not the auction ever said so. What it must not do is wander off while somebody is
    # hanging off it -- and rungs 1-3 below would happily send it away.
    #
    # This is the half of "relays are moving posts" that works. The other half, teaching
    # `relay_posts` to plan chains around idle relays, measured **-1.5 rescued and -2.0
    # found** (M-69): the planner cannot trust a position the robot is about to leave.
    # The robot can, because it is the one deciding to leave.
    holding = _load_bearing(world, relays)
    for i in sorted(holding):
        out[i] = ((float(world.pos[i][0]), float(world.pos[i][1])),
                  "relay: holding the only link a robot has")
    if len(holding):
        relays = np.array([i for i in relays if int(i) not in holding], dtype=int)
        if len(relays) == 0:
            return out

    # 1. Repair. The target is not the stranded robot but a point `reach` short of it,
    # back along the line toward the claiming relay -- which is inside the net. A relay
    # that walks all the way to a cut-off robot is simply cut off too; one that stops at
    # radius bridges the gap, which is the whole job.
    def _bridge(i: int, s: int) -> tuple[float, float]:
        v = world.pos[i] - world.pos[s]
        n = float(np.linalg.norm(v))
        if n <= 1e-6:
            return (float(world.pos[s][0]), float(world.pos[s][1]))
        q = world.pos[s] + v / n * min(reach, n)
        return (float(q[0]), float(q[1]))

    stranded = np.nonzero((world.status <= OUT_OF_COMMS) & ~world.in_comms)[0]
    left = _claim(world, relays, stranded, _bridge,
                  "relay: bridging to a cut-off robot", out)

    # 2. Reinforce. Stand `reach` out from the anchor the fragile robot is hanging off,
    # in its direction: that halves the gap and turns one marginal hop into two solid
    # ones, without stepping outside the net to do it.
    if len(left):
        fragile, anchor = _fragile_links(world)

        def _shore(i: int, s: int) -> tuple[float, float]:
            a = anchor[s]
            v = world.pos[s] - a
            n = float(np.linalg.norm(v))
            if n <= 1e-6:
                return (float(a[0]), float(a[1]))
            q = a + v / n * min(reach, n)
            return (float(q[0]), float(q[1]))

        _claim(world, left, fragile, _shore,
               "relay: shoring up a link at the edge of range", out)

    return out


def _claim(world, relays: np.ndarray, needy: np.ndarray, target, why: str,
           out: dict) -> np.ndarray:
    """Greedy exclusive matching, nearest-first. Returns the relays left unused.

    Nearest-first rather than worst-first: every candidate here is a robot the lane can
    help, so the thing worth minimising is how long the help takes to arrive.
    """
    if len(needy) == 0 or len(relays) == 0:
        return relays
    d = np.linalg.norm(world.pos[relays][:, None, :] - world.pos[needy][None, :, :],
                       axis=2)
    for row in range(len(relays)):
        j = int(np.argmin(d[row]))
        if not np.isfinite(d[row, j]):
            return relays[row:]                     # every needy robot claimed
        i, s = int(relays[row]), int(needy[j])
        d[:, j] = np.inf                            # one relay per needy robot
        out[i] = (target(i, s), why)
    return relays[:0]


#: A link this close to the limit of its range is one step from breaking. Below this a
#: robot has room to move without dropping out and needs no help.
FRAGILE_AT = 0.80


def _fragile_links(world) -> tuple[np.ndarray, dict[int, np.ndarray]]:
    """In-contact robots hanging off a link near its range limit, and what they hang off.

    A robot is in contact through the nearest thing that can hear it -- base within
    `base_radius`, or any relay in the component within `relay_radius`. Expressed as a
    fraction of that radius, how far it sits from that anchor is exactly how much slack
    it has left. Past `FRAGILE_AT` it is one step from becoming a rescue job for rung 1.

    Relays are excluded as *candidates*: a relay near the edge of the net is a relay
    doing its job, not a robot in trouble.
    """
    alive = world.status <= OUT_OF_COMMS
    is_relay = world.actuator == LANE_INDEX["antenna"]
    base = np.asarray(world.scn.base, dtype=float)
    br, rr = world.scn.comms.base_radius, world.scn.comms.relay_radius

    cand = np.nonzero(alive & world.in_comms & ~is_relay)[0]
    if len(cand) == 0:
        return cand, {}

    slack = np.linalg.norm(world.pos[cand] - base, axis=1) / br
    apos = np.repeat(base[None, :], len(cand), axis=0)

    anchors = np.nonzero(alive & world.in_comms & is_relay)[0]
    if len(anchors):
        dr = np.linalg.norm(world.pos[cand][:, None, :] - world.pos[anchors][None, :, :],
                            axis=2) / rr
        k = np.argmin(dr, axis=1)
        near = dr[np.arange(len(cand)), k]
        closer = near < slack
        slack = np.where(closer, near, slack)
        apos[closer] = world.pos[anchors[k[closer]]]

    hit = slack > FRAGILE_AT
    idx = cand[hit]
    return idx, {int(j): apos[hit][n] for n, j in enumerate(idx)}


def _drift_targets(world, spare: np.ndarray, dark: dict[int, np.ndarray],
                   reach: dict[int, np.ndarray] | None = None
                   ) -> dict[int, tuple[float, float]]:
    """Where each unemployed robot goes: the nearest sector the swarm has not finished.

    Three rules, each because the obvious version is worse:

    1. **Targets are shared.** Each robot takes the nearest of at most 48 aim points, so
       hundreds converge on a handful of goals. That is deliberate. Giving search work its
       own target per robot has been measured twice and lost twice (M-14, M-36):
       spreading thin pushes robots past the comms envelope, and a robot out of contact
       reveals nothing to the shared map and cannot be given work at all.

    2. **Targets are per chassis**, taken from `_dark_points`, so a wheeled unit is never
       aimed across a river it cannot ford. Aiming a robot at ground its own locomotion
       rules out does not merely waste it -- it parks it against the obstacle, and a
       column of them reads as a crowd stacked on a wall.

    3. **A relay creeps; everything else walks.** A relay's value is being a link, so it
       advances at most `0.8 x relay_radius` toward the dark ground per goal -- far enough
       to extend the net, short enough to stay inside it. Sending relays the whole way
       turns the chain into a crowd at the far end and drops the swarm behind it out of
       contact, which is the failure `_relay_orders` rung 1 exists to clean up.
    """
    if len(spare) == 0 or not dark:
        return {}
    creep = world.scn.comms.relay_radius * 0.8
    is_relay = world.actuator[spare] == LANE_INDEX["antenna"]
    out: dict[int, tuple[float, float]] = {}
    for c in sorted(dark):
        pts = dark[c]
        rows = np.nonzero(world.chassis[spare] == c)[0]
        if len(rows) == 0 or len(pts) == 0:
            continue
        pos = world.pos[spare[rows]]
        d = np.linalg.norm(pos[:, None, :] - pts[None, :, :], axis=2)

        # Rule out targets this robot cannot walk to, not merely ones it cannot stand on.
        if reach is not None and c in reach:
            lab = reach[c]
            rx, ry = grid.world_to_cell(pos[:, 0], pos[:, 1], world.cell, world.shape)
            px, py = grid.world_to_cell(pts[:, 0], pts[:, 1], world.cell, world.shape)
            same = ((lab[ry, rx][:, None] == lab[py, px][None, :])
                    & (lab[ry, rx][:, None] >= 0))
            d = np.where(same, d, np.inf)

        px, py = grid.world_to_cell(pts[:, 0], pts[:, 1], world.cell, world.shape)
        sectors = world.sector_of_cell[py, px]
        d[:, world.sector_abandoned[sectors]] = np.inf
        # A reachable leader priority precedes autonomous proximity preference.
        priority = np.where(np.isfinite(d), world.sector_priority[sectors], 99)
        d = np.where(priority == priority.min(axis=1, keepdims=True), d, np.inf)

        pick = np.argmin(d, axis=1)
        for k, row in enumerate(rows):
            if not np.isfinite(d[k, pick[k]]):
                continue                 # nothing dark this robot can actually reach
            tgt = pts[pick[k]]
            if is_relay[row]:
                # A creeping relay stops short of its target, and the point it stops at
                # is a *computed* one -- it was never in the dark set and nothing has
                # checked it. Landing it inside a rock parks the relay against the rock,
                # which is the same heap the aim points were just fixed for, one step
                # removed. Back off along the line until the point is somewhere this
                # chassis can actually stand and reach.
                v = tgt - pos[k]
                n = float(np.linalg.norm(v))
                if n > creep:
                    tgt = _last_reachable(world, pos[k], v / n, creep,
                                          reach.get(c) if reach else None)
                    if tgt is None:
                        continue
            out[int(spare[row])] = (float(tgt[0]), float(tgt[1]))
    return out


#: Idle robots staged per casualty, by the lane that can act on it. Carriers per
#: cleared casualty matches `TaskGenerator.carriers_per_victim`; diggers match the
#: simulation's `MAX_DIGGERS`, because a fourth scoop round the hole adds nothing.
STAGE_CARRIERS_PER_VICTIM = 2
STAGE_DIGGERS_PER_VICTIM = 3


def _casualty_staging(world, spare: np.ndarray, reach: dict[int, np.ndarray] | None = None
                      ) -> dict[int, tuple[tuple[float, float], str]]:
    """Where an idle carrier or digger waits: beside the casualty it acts on next.

    **Unassigned movement, exactly like drift.** The robot holds no task, so it stays
    in every auction's free pool and bids on the real task the moment one is announced
    -- it is simply already standing next to the work instead of a hundred metres away.
    When it arrives, the simulation's own proximity rules take over: a gripper within
    `REACH_GRAB` of a cleared casualty picks it up, and a scoop within `REACH_DIG` of a
    buried one digs it, both without any assignment at all.

    Posts are casualties that are *known* to the swarm (`FOUND`/`CLEARED`, which only
    a resolved report sets) -- never the hidden list. Cleared casualties get carriers;
    buried ones get diggers and, as a second rank, carriers pre-staged for the pickup
    that follows. Per-post caps stop the whole lane piling onto one body, and the
    assignment is nearest-first so a robot that is already close stays close.

    Hazard and reachability are filtered the same way drift filters them: a robot
    parked against a river bank or inside the fire is not staged, it is lost.
    """
    posts: list[tuple[np.ndarray, int, int, str]] = []
    for v in world.victims:
        if v.state == CLEARED:
            posts.append((np.asarray(v.pos, dtype=float), LANE_INDEX["gripper"],
                          STAGE_CARRIERS_PER_VICTIM, f"staging at {v.id} for pickup"))
        elif v.state == FOUND and v.buried and v.debris_remaining > 0.0:
            posts.append((np.asarray(v.pos, dtype=float), LANE_INDEX["scoop"],
                          STAGE_DIGGERS_PER_VICTIM, f"staging at {v.id} to dig"))
            posts.append((np.asarray(v.pos, dtype=float), LANE_INDEX["gripper"],
                          STAGE_CARRIERS_PER_VICTIM, f"staging near {v.id} for pickup"))
    if not posts or len(spare) == 0:
        return {}

    lanes = (LANE_INDEX["gripper"], LANE_INDEX["scoop"])
    cand = spare[np.isin(world.actuator[spare], lanes)]
    if len(cand) == 0:
        return {}

    want_lane = np.array([p[1] for p in posts])
    pos_posts = np.stack([p[0] for p in posts])
    caps = np.array([p[2] for p in posts])
    d = np.linalg.norm(world.pos[cand][:, None, :] - pos_posts[None, :, :], axis=2)
    d[:, np.array([in_known_hazard(world, p) for p in pos_posts])] = np.inf
    d[world.actuator[cand][:, None] != want_lane[None, :]] = np.inf

    if reach is not None:
        rx, ry = grid.world_to_cell(world.pos[cand, 0], world.pos[cand, 1],
                                    world.cell, world.shape)
        px, py = grid.world_to_cell(pos_posts[:, 0], pos_posts[:, 1],
                                    world.cell, world.shape)
        for c, lab in reach.items():
            rows = np.nonzero(world.chassis[cand] == c)[0]
            if len(rows) == 0:
                continue
            same = ((lab[ry[rows], rx[rows]][:, None] == lab[py, px][None, :])
                    & (lab[ry[rows], rx[rows]][:, None] >= 0))
            d[rows] = np.where(same, d[rows], np.inf)

    out: dict[int, tuple[tuple[float, float], str]] = {}
    used = np.zeros(len(posts), dtype=int)
    for f in np.argsort(d, axis=None):
        row, col = divmod(int(f), d.shape[1])
        if not np.isfinite(d[row, col]) or used[col] >= caps[col]:
            continue
        i = int(cand[row])
        if i in out:
            continue
        used[col] += 1
        out[i] = ((float(pos_posts[col, 0]), float(pos_posts[col, 1])), posts[col][3])
    return out


#: Robots of any lane posted per casualty when there is nothing else to do. Higher
#: than the carrier/digger caps because these are not doing the work -- they are
#: simply better placed than standing at base, and the crowd is bounded by the
#: casualty count rather than by a fleet.
SUPPORT_PER_VICTIM = 4


def _support_staging(world, spare: np.ndarray, reach: dict[int, np.ndarray] | None = None
                     ) -> dict[int, tuple[tuple[float, float], str]]:
    """Last-resort posts for robots with no search work: stand by a casualty.

    Deliberately the *last* branch before idle, not a competitor to drift: exploring
    dark ground is worth more than standing near somebody who already has a carrier.
    It only fires when the map has no reachable dark ground left to offer, which is
    the endgame state -- measured at t=180 on seed 42, 126 robots idle while 22
    cleared casualties waited for pickup, and the idle count was the larger of the
    two. A scout cannot carry or dig; it can be the pair of eyes an operator would
    expect to see at a rescue, and its camera is what finds the next casualty.
    """
    posts: list[tuple[np.ndarray, str]] = []
    for v in world.victims:
        if v.state == CLEARED:
            posts.append((np.asarray(v.pos, dtype=float), f"supporting pickup of {v.id}"))
        elif v.state == FOUND:
            posts.append((np.asarray(v.pos, dtype=float), f"supporting dig at {v.id}"))
        elif v.state == CARRIED:
            posts.append((np.asarray(v.pos, dtype=float), f"escorting {v.id} home"))
    if not posts or len(spare) == 0:
        return {}
    pos_posts = np.stack([p[0] for p in posts])
    caps = np.full(len(posts), SUPPORT_PER_VICTIM, dtype=int)
    d = np.linalg.norm(world.pos[spare][:, None, :] - pos_posts[None, :, :], axis=2)
    d[:, np.array([in_known_hazard(world, p) for p in pos_posts])] = np.inf
    if reach is not None:
        rx, ry = grid.world_to_cell(world.pos[spare, 0], world.pos[spare, 1],
                                    world.cell, world.shape)
        px, py = grid.world_to_cell(pos_posts[:, 0], pos_posts[:, 1],
                                    world.cell, world.shape)
        for c, lab in reach.items():
            rows = np.nonzero(world.chassis[spare] == c)[0]
            if len(rows) == 0:
                continue
            same = ((lab[ry[rows], rx[rows]][:, None] == lab[py, px][None, :])
                    & (lab[ry[rows], rx[rows]][:, None] >= 0))
            d[rows] = np.where(same, d[rows], np.inf)

    out: dict[int, tuple[tuple[float, float], str]] = {}
    used = np.zeros(len(posts), dtype=int)
    for f in np.argsort(d, axis=None):
        row, col = divmod(int(f), d.shape[1])
        if not np.isfinite(d[row, col]) or used[col] >= caps[col]:
            continue
        i = int(spare[row])
        if i in out:
            continue
        used[col] += 1
        out[i] = ((float(pos_posts[col, 0]), float(pos_posts[col, 1])), posts[col][1])
    return out


def _activity_of(world, i: int, a: Assignment) -> int:
    """An assignment's `ACTIVITY` code. Split where the *picture* differs, not the task.

    `extract` is two states to an operator -- walking at a casualty, and carrying one --
    and a relay at its post is the case this table exists for: motionless and correct.
    """
    if a.kind == "extract":
        return ACTIVITY["carry"] if world.carrying[i] >= 0 else ACTIVITY["extract"]
    if a.kind == "relay":
        arrived = float(np.linalg.norm(world.pos[i] - np.asarray(a.target))) <= RELAY_ON_POST_M
        return ACTIVITY["relay_post"] if arrived else ACTIVITY["relay_move"]
    return ACTIVITY.get(
        {"clear_debris": "dig"}.get(a.kind, a.kind), ACTIVITY["idle"])


def _last_reachable(world, origin, direction, span: float, lab):
    """The furthest point up to `span` along `direction` still in `origin`'s component.

    Stepped rather than solved: the ray can leave and re-enter passable ground, and the
    useful answer is the last point reachable *without crossing a gap*, not the furthest
    passable one. Eight samples is enough at these spans (0.8 x relay_radius = 36.8 m on
    the demo map, so ~4.6 m per step) and costs four array lookups.
    """
    if lab is None:
        return origin + direction * span
    ox, oy = grid.world_to_cell(np.asarray(origin[0]), np.asarray(origin[1]),
                                world.cell, world.shape)
    home = lab[int(oy), int(ox)]
    best = None
    for k in range(1, 9):
        p = origin + direction * (span * k / 8.0)
        px, py = grid.world_to_cell(np.asarray(p[0]), np.asarray(p[1]),
                                    world.cell, world.shape)
        if lab[int(py), int(px)] != home:
            break
        best = p
    return best


def _nearest_contact(world, i: int) -> tuple[float, float]:
    """Position of the closest robot still inside the comms component, else base.

    The component is what has to be rejoined, and its edge is usually far closer than
    base -- a relay holding a post 30 m away is a link, and walking to it costs seconds
    rather than minutes.
    """
    linked = world.in_comms & (world.status <= OUT_OF_COMMS)
    linked[i] = False
    if not linked.any():
        return tuple(world.scn.base)
    idx = np.nonzero(linked)[0]
    d2 = ((world.pos[idx] - world.pos[i]) ** 2).sum(axis=1)
    return tuple(world.pos[idx[int(np.argmin(d2))]])


def _dry_standing_goal(world, i, goal, reach):
    """Keep autonomous waiting/relay targets on reachable banks, not in the river.

    A computed midpoint has none of the guarantees of an auction target. Search
    locally around it; do not redirect a committed assignment or a vital held link.
    """
    c = int(world.chassis[i])
    gx, gy = _cell(world, goal)
    if world.water[gy, gx] == 0 and world.chassis_passable[c, gy, gx]:
        return goal
    radius = max(2, int(np.ceil(world.scn.comms.relay_radius * .5 / world.cell)))
    x0, x1 = max(0, gx-radius), min(world.shape[1], gx+radius+1)
    y0, y1 = max(0, gy-radius), min(world.shape[0], gy+radius+1)
    region = (slice(y0, y1), slice(x0, x1))
    ok = world.chassis_passable[c][region] & (world.water[region] == 0)
    ok &= ~world.hazard_known[region] & ~world.sector_abandoned[world.sector_of_cell[region]]
    rx, ry = _cell(world, world.pos[i])
    label = reach[c][ry, rx]
    ok &= (reach[c][region] == label) & (label >= 0)
    yy, xx = np.nonzero(ok)
    if not len(xx):
        return None
    pts = np.column_stack([xx+x0+.5, yy+y0+.5]) * world.cell
    pick = np.argmin(np.sum((pts-np.asarray(goal))**2, axis=1))
    return tuple(pts[pick])


def _nearest_zone(world, pos, nav=None, chassis: int = 0) -> tuple[float, float]:
    """The nearest collection point this robot can actually **reach**.

    Straight-line nearest is wrong wherever water or slope divides the map, and the
    rivers are now 16-26 m wide. Measured at t=330 on seed 42: **9 of 17 loaded carriers
    were standing still**, in contact, batteries at 0.56-0.71, with a collection point
    13-55 m away — pointed across a river at a zone their chassis cannot enter. The flow
    field returns UNREACHABLE, Tier 1 has nothing to descend, and they hold the casualty
    for the rest of the mission.

    They are not even released to try again: `_stalled` treats an UNREACHABLE reading as
    *no evidence* rather than *no progress* (M-55), which is right for the transient case
    the downsample produces and wrong for a goal that is permanently unreachable. Choosing
    a reachable zone in the first place fixes both.

    Falls back to straight-line if no nav is available or no zone is reachable — a carrier
    walking at a zone it cannot reach is still better than one with no goal at all.
    """
    zones = [world.scn.base, *world.scn.extraction_zones]
    if nav is not None:
        best, best_d = None, float("inf")
        for z in zones:
            f = nav.field(chassis, z[0], z[1])
            d = float(nav.distance_at(chassis, f, np.array([pos[0]]), np.array([pos[1]]))[0])
            if d < best_d:
                best, best_d = z, d
        if best is not None and best_d < UNREACHABLE * 0.5:
            return tuple(best)
    d = [(_dist(pos, z), z) for z in zones]
    return tuple(min(d, key=lambda t: t[0])[1])


def capable(world, kind: str) -> np.ndarray:
    """Boolean mask of robots physically able to execute ``kind``."""
    need = REQUIRES.get(kind)
    if need is None:
        return np.ones(world.n, dtype=bool)
    return world.actuator == LANE_INDEX[need]


#: Tried and reverted (M-53): sending an out-of-contact robot home once its unreported
#: backlog passed ~1500 cells. It reads as obviously right -- data has no value until it
#: is delivered -- and it measured as a 4.5-point loss of exploration and 3.25 rescues,
#: because the far sectors are beyond the comms envelope by more than a backlog's worth
#: of travel. Robots filled the buffer on the way out, turned around, delivered, were
#: re-tasked outward, and commuted for the rest of the mission: afterwards **no robot
#: reached the far sectors at all**, where before at least five of six were stood in.
#: The constraint is the reach of the network, not the willingness of the robot.
#: Task kinds an antenna robot must never be given, however cheap its bid.
_INFRASTRUCTURE_EXEMPT = ("explore", "investigate", "extract", "clear_debris")


def eligible(world, kind: str) -> np.ndarray:
    """Physically capable **and** allowed by policy.

    Antenna robots are infrastructure, not searchers. They can physically explore, and
    both allocators will happily hand them a frontier if only physical capability is
    checked -- at which point they walk out past the chain they are holding up and the
    whole swarm drops out of contact behind them. Every allocator uses this, not
    ``capable``.
    """
    m = capable(world, kind)
    if kind in _INFRASTRUCTURE_EXEMPT:
        m = m & (world.actuator != LANE_INDEX["antenna"])
    return m


__all__ = ["Assignment", "SkillExecutor", "capable", "REQUIRES", "FOUND",
           "NO_PROGRESS_AFTER", "PROGRESS_EPS"]

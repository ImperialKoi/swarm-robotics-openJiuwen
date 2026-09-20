"""Manual override: an operator driving one robot from the dashboard.

Layered on the autonomy, not in place of it. The override replaces exactly one robot's
goal-seeking command, and it does so **before** the Tier 1 swept-circle wall override
(`ReflexController.commands`), so a human at the keys is held to the same safety floor as
the auction (CLAUDE.md invariant 2). Everything else keeps running underneath: the robot
keeps its assignment, the stall detector keeps watching it, and when the override lapses
it steers for that task again from wherever it was left.

**One robot at a time, decided here.** A drive command for a second robot takes over from
the first as it is received, so there is no tick on which neither the operator nor the
autonomy commands a robot, and none on which both do.

Control lapses on its own; the dashboard never has to hand it back:

* a drive command older than ``CMD_TTL_S`` is a stop. The dashboard re-sends at 10 Hz
  while a key is held, so a dropped socket or a crashed client halts the robot within
  0.3 s rather than leaving it driving;
* ``HOLD_S`` after the last command of any kind, the robot returns to its autonomy.
  Until then it holds still, so an operator nudging it with taps is not fought between
  them.

Only the demo has this. The override is owned by `BridgeNode`, and `--headless`, the gate,
and every training rollout run without a bridge -- nothing here can move the seed-42 hash.

**The driven unit is faster than its rating.** `OPERATOR_SPEED` lifts its ceiling while
the lease is held -- see robot.py for why, and for why the safety floor is unaffected.

**The action key is one key and the simulator chooses what it does.** Whether a casualty
is in reach is a ground-truth question, so nothing on this side of the socket may answer
it: an ``act`` message says only *which robot pressed it*, and `World.operator_act`
decides between picking a casualty up, setting one down, and taking a rotor off or
landing it. The same call is used, as `World.operator_offer`, to tell the dashboard what
the key would do, so the badge and the key can never disagree.

Wire format, dashboard -> simulator, one JSON object per frame (godot/scripts/manual_drive.gd):

    {"t": "drive", "robot": "r07", "v": 1.0, "w": -0.5}    # fractions of the unit's limits
    {"t": "act", "robot": "r07"}                            # the action key, once per press
    {"t": "release", "robot": "r07"}                        # hand it back now

``v`` is forward speed as a fraction of the unit's own top speed, negative for reverse
(which tops out at `REVERSE_SPEED` of it); ``w`` is turn rate as a fraction of its own
``omega_max``, positive toward +heading.
"""

from __future__ import annotations

import math

from ..sim.robot import OPERATOR_SPEED, OUT_OF_COMMS, REVERSE_SPEED

#: Seconds after the last command before the robot goes back to its autonomy. Long enough
#: to tap-steer without the swarm grabbing the unit between taps; short enough that a
#: robot let go mid-demo is visibly back at work almost at once.
HOLD_S = 1.5
#: A drive command older than this is treated as a stop -- the deadman.
CMD_TTL_S = 0.3

#: Message kinds this module owns. Anything else from the dashboard is not ours.
KINDS = ("drive", "release", "act")


class ManualOverride:
    """The operator's lease on at most one robot."""

    def __init__(self, world) -> None:
        self._index = {rid: i for i, rid in enumerate(world.robot_ids)}
        #: The robot under manual control, or -1.
        self.robot = -1
        self._v = 0.0
        self._w = 0.0
        self._cmd_until = 0.0
        self._until = 0.0

    def receive(self, world, msg) -> bool:
        """Apply one dashboard message. False if it is not a drive message at all.

        Malformed drive messages are swallowed, not raised: this runs inside the sim tick,
        and a bad frame from the dashboard must not stop the mission.
        """
        if not isinstance(msg, dict) or msg.get("t") not in KINDS:
            return False
        i = self._index.get(msg.get("robot"))
        if i is None:
            return True
        if msg["t"] == "release":
            if i == self.robot:
                self.robot = -1
            return True
        if msg["t"] == "act":
            if world.status[i] > OUT_OF_COMMS:
                return True
            # Takes the lease like a drive does, and renews the hold so the swarm does
            # not grab the unit back between an operator's presses. It deliberately
            # leaves `_v`/`_w` alone: pressing it mid-drive must not stutter the robot,
            # and pressing it while parked leaves the last command stale, which is a
            # stop. What it does is the simulator's to decide.
            self.robot = i
            self._until = world.t + HOLD_S
            # The lease first, then the action. Taking a unit clears the latches the
            # last one was holding (`set_operator`), and a rotor's action sets one of
            # its own -- in the other order the take-off would be cancelled by the
            # lease that the same key press acquired.
            world.set_operator(i)
            world.operator_act(i)
            return True
        try:
            v, w = float(msg.get("v", 0.0)), float(msg.get("w", 0.0))
        except (TypeError, ValueError):
            return True
        if not (math.isfinite(v) and math.isfinite(w)) or world.status[i] > OUT_OF_COMMS:
            return True
        self.robot = i
        self._v = min(max(v, -1.0), 1.0)
        self._w = min(max(w, -1.0), 1.0)
        self._cmd_until = world.t + CMD_TTL_S
        self._until = world.t + HOLD_S
        return True

    def active(self, world) -> int:
        """The robot under manual control now, or -1. Lets a lapsed lease go.

        The single funnel for who holds the lease, so it is also where the world is told
        -- `World.set_operator` raises that robot's speed ceiling and stops its autonomy
        picking casualties up behind the operator's back, and both have to end the tick
        the lease does.
        """
        i = self.robot
        if i >= 0 and (world.t >= self._until or world.status[i] > OUT_OF_COMMS):
            self.robot = -1
        world.set_operator(self.robot)
        return self.robot

    def command(self, world) -> tuple[int, float, float] | None:
        """``(robot, v, omega)`` in that robot's own units for Tier 1, or None."""
        i = self.active(world)
        if i < 0:
            return None
        if world.t >= self._cmd_until:
            return i, 0.0, 0.0
        top = (world.v_max[i] * OPERATOR_SPEED
               * (REVERSE_SPEED if self._v < 0.0 else 1.0))
        return i, float(self._v * top), float(self._w * world.omega_max[i])

    def wire(self, world) -> list:
        """The state frame's ``manual`` field: ``[robot index, seconds until autonomy,
        action code]``, or empty. The dashboard's badge is drawn from this, not from its
        own keys -- including the action hint, which is an `OPERATOR_ACTION` code."""
        i = self.active(world)
        if i < 0:
            return []
        return [i, round(max(self._until - world.t, 0.0), 1), world.operator_offer(i)]

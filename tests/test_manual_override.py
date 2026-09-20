"""The dashboard's WASD override: layered on autonomy, never past the safety floor.

Most of these drive the override the way the dashboard does -- JSON messages into the
bridge's inbound queue -- and watch what the simulator makes of them. The last runs the
Godot half (godot/tests/manual_drive_check.gd) in native headless Godot where it is
installed. The wire agreement between the two is checked in tests/test_bridge_protocol.py.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.bus.ws_server import WebSocketServer
from swarmmind.contracts.schemas import OPERATOR_ACTION
from swarmmind.control.manual import CMD_TTL_S, HOLD_S
from swarmmind.control.planner import NavSet
from swarmmind.control.tier1_reflex import ReflexController
from swarmmind.mission import Mission
from swarmmind.nodes.bridge import BridgeNode
from swarmmind.sim import grid
from swarmmind.sim.robot import (
    CHASSIS_INDEX,
    FAILED,
    LANE_INDEX,
    OPERATOR_SPEED,
    REVERSE_SPEED,
)
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import CARRIED, CLEARED, REACH_GRAB, World


def _demo(seed: int = 42) -> Mission:
    """A mission wired the way `cli run --demo` wires it, minus the socket and the clock."""
    m = Mission(Scenario.load("test"), seed, hivemind=False)
    m.bridge = BridgeNode(m.world, WebSocketServer(port=8799), m.bus)
    return m


def _say(m: Mission, **msg) -> None:
    """Queue one dashboard message. The bridge takes it at the end of the next tick."""
    m.bridge.server._commands.append(msg)


def _moving_ground_robot(m: Mission) -> int:
    """Warm up, then pick a live ground robot that autonomy is actually moving."""
    w = m.world
    start = w.pos.copy()
    for _ in range(60):
        m.tick()
    moved = np.linalg.norm(w.pos - start, axis=1) > 0.5
    ok = moved & (w.status == 0) & (w.chassis != CHASSIS_INDEX["rotor"])
    assert ok.any(), "no ground robot moved in the warm-up"
    return int(np.flatnonzero(ok)[0])


def _open_spot(w: World, i: int, clear: int = 4) -> tuple[float, float]:
    """A cell centre with `clear` cells of plain ground this robot can cross on every side,
    so the physics under test is the override's and not a wall's or the rubble's."""
    ok = w.chassis_passable[w.chassis[i]] & (w.occ != grid.RUBBLE)
    h, wd = ok.shape
    for cy in range(clear, h - clear):
        for cx in range(clear, wd - clear):
            if ok[cy - clear:cy + clear + 1, cx - clear:cx + clear + 1].all():
                return (cx + 0.5) * w.cell, (cy + 0.5) * w.cell
    raise AssertionError("no open ground on the fixture")


def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def test_the_keys_drive_the_unit_and_autonomy_takes_it_back():
    m = _demo()
    w = m.world
    i = _moving_ground_robot(m)
    _say(m, t="drive", robot=w.robot_ids[i], v=0.0, w=1.0)
    m.tick()
    assert m.bridge.manual.robot == i
    t0 = w.t                                   # the lease runs from receipt

    # Turning on the spot at exactly the commanded rate, with no travel at all -- the
    # robot's own goal-seeking is not mixed in.
    pos, th = w.pos[i].copy(), float(w.theta[i])
    m.tick()
    assert np.array_equal(w.pos[i], pos)
    assert np.isclose(_wrap(w.theta[i] - th), w.omega_max[i] * w.dt)

    # No more keys. The command goes stale and the unit holds still under the operator...
    # (Bounds are a tick clear of each deadline; sim time is tick / hz, and a bound that
    # sits exactly on one is decided by float rounding.)
    while w.t < t0 + CMD_TTL_S + w.dt:
        m.tick()
    pos, th = w.pos[i].copy(), float(w.theta[i])
    while w.t < t0 + HOLD_S - 2 * w.dt:
        m.tick()
        assert m.bridge.manual.wire(w)[0] == i, "control lapsed before HOLD_S"
        assert np.array_equal(w.pos[i], pos) and w.theta[i] == th, "a held unit moved"

    # ...then goes back to its autonomy without anyone releasing it.
    while w.t < t0 + HOLD_S + 2 * w.dt:
        m.tick()
    assert m.bridge.manual.robot == -1
    assert m.bridge.manual.wire(w) == []
    for _ in range(100):
        m.tick()
    assert np.linalg.norm(w.pos[i] - pos) > 0.5, "autonomy did not resume after the lease"


def test_forward_and_reverse_follow_the_heading_and_reverse_is_capped():
    """...and the driven unit runs at `OPERATOR_SPEED` x its own rating.

    The boost has to be checked at the *position*, not at the command: it is applied in
    two places that must agree -- `ManualOverride.command` raises what Tier 1 is handed,
    and `World.step`'s per-robot ceiling has to let it through. Either one alone and the
    unit moves at its rated speed with nothing to show that anything is wrong.
    """
    m = _demo()
    w = m.world
    i = _moving_ground_robot(m)
    rid = w.robot_ids[i]
    for v in (1.0, -1.0):
        w.pos[i] = _open_spot(w, i)
        w.theta[i] = 0.3
        _say(m, t="drive", robot=rid, v=v, w=0.0)
        m.tick()
        p0 = w.pos[i].copy()
        m.tick()
        step = w.pos[i] - p0
        heading = np.array([np.cos(w.theta[i]), np.sin(w.theta[i])])
        top = w.v_max[i] * OPERATOR_SPEED * w.dt * (1.0 if v > 0 else REVERSE_SPEED)
        assert np.sign(step @ heading) == np.sign(v), f"v={v} moved the wrong way"
        assert np.isclose(np.linalg.norm(step), top), f"v={v} moved {step}, not {top}"


def test_the_boost_belongs_to_the_lease_and_nobody_else():
    """Exactly one robot is ever fast, and it is fast for exactly as long as it is held.

    A ceiling left raised on a robot the operator has let go is the quiet version of this
    bug: the swarm keeps running and one unit is permanently 2x, which reads as a physics
    glitch rather than as an override.
    """
    m = _demo()
    w = m.world
    assert np.array_equal(w.speed_boost, np.ones(w.n)), "the swarm starts unboosted"
    # Two ticks: the bridge takes inbound commands at the *end* of a tick, and the
    # ceiling is raised by `command()` at the start of the next -- the same tick whose
    # `step` has to let the faster command through, which is the pairing that matters.
    _say(m, t="drive", robot=w.robot_ids[1], v=1.0, w=0.0)
    m.tick()
    m.tick()
    assert w.speed_boost[1] == OPERATOR_SPEED
    assert w.speed_boost.sum() == OPERATOR_SPEED + (w.n - 1)

    # Switching units takes it with them.
    _say(m, t="drive", robot=w.robot_ids[2], v=1.0, w=0.0)
    m.tick()
    m.tick()
    assert w.speed_boost[1] == 1.0 and w.speed_boost[2] == OPERATOR_SPEED

    # And the lease lapsing puts it back.
    lapse = w.t + HOLD_S + 2 * w.dt
    while w.t < lapse:
        m.tick()
    assert m.bridge.manual.robot == -1
    assert np.array_equal(w.speed_boost, np.ones(w.n))


def test_headless_never_sees_the_boost():
    """Invariant 6. The ceiling is per-robot now, so the clip is new code on the path
    every evaluation and the seed-42 hash run through."""
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    for _ in range(40):
        m.tick()
        assert np.array_equal(m.world.speed_boost, np.ones(m.world.n))
    assert m.world.operator == -1 and not m.world.operator_hover


def _rotor(m: Mission) -> int:
    """A live rotor standing on ground it can set down on, so its height is the only
    thing the action key is being judged on."""
    w = m.world
    i = int(np.flatnonzero((w.chassis == CHASSIS_INDEX["rotor"]) & (w.status == 0))[0])
    w.pos[i] = _open_spot(w, i, clear=6)
    return i


def _lane_robot(m: Mission, lane: str) -> int:
    """A live robot of one actuator lane, parked on open ground away from the spawn pile.

    Away matters: `_update_victims` lets *any* free gripper in reach take a casualty, so a
    test about one carrier has to be the only carrier near it.
    """
    w = m.world
    i = int(np.flatnonzero((w.actuator == LANE_INDEX[lane]) & (w.status == 0))[0])
    w.pos[i] = _open_spot(w, i, clear=6)
    return i


def _casualty_at(m: Mission, i: int) -> int:
    """A cleared casualty lying at robot `i`'s feet, waiting for a carrier."""
    w = m.world
    vi = 0
    v = w.victims[vi]
    v.pos = w.pos[i].copy()
    v.state, v.carrier, v.buried, v.debris_remaining = CLEARED, -1, False, 0.0
    return vi


def _held(m: Mission, i: int) -> None:
    """Park robot `i` under the operator and wait for the world to know it.

    A drive of (0, 0) rather than nothing at all: the lease is what suppresses the
    automatic pickup, and it reaches `World` on the tick *after* the bridge takes the
    message. Every test below that is about the key and not about autonomy has to get
    past that gap first, or `_update_victims` resolves the situation before the key does.
    """
    w = m.world
    _say(m, t="drive", robot=w.robot_ids[i], v=0.0, w=0.0)
    m.tick()
    m.tick()
    assert w.operator == i, "the lease never reached the world"


def test_the_action_key_picks_a_casualty_up_and_sets_it_down():
    """The operator's half of the rescue chain: the two things a carrier does, on demand."""
    m = _demo()
    w = m.world
    i = _lane_robot(m, "gripper")
    rid = w.robot_ids[i]
    _held(m, i)
    vi = _casualty_at(m, i)

    _say(m, t="act", robot=rid)
    m.tick()
    assert w.victims[vi].state == CARRIED, "the action key did not pick the casualty up"
    assert w.victims[vi].carrier == i and w.carrying[i] == vi
    assert m.bridge.manual.robot == i, "acting on a unit takes the lease, like driving it"

    # The same key sets it down again -- the one thing no autonomous carrier ever does.
    _say(m, t="act", robot=rid)
    m.tick()
    assert w.victims[vi].state == CLEARED, "the action key did not set the casualty down"
    assert w.carrying[i] == -1 and w.victims[vi].carrier == -1
    assert np.allclose(w.victims[vi].pos, w.pos[i]), "it was put down somewhere else"


def test_autonomy_does_not_grab_a_casualty_behind_the_operator():
    """A driven carrier picks up when it is told to, not by walking past.

    Without this the set-down key looks broken: `_update_victims` would hand the casualty
    straight back to the same robot on the next tick.
    """
    m = _demo()
    w = m.world
    i = _lane_robot(m, "gripper")
    _held(m, i)
    vi = _casualty_at(m, i)
    m.tick()
    assert w.victims[vi].state == CLEARED, "autonomy took the casualty from under the keys"
    assert w.carrying[i] == -1

    # ...and it is the lease doing it, not a rule that broke the swarm's own pickup.
    lapse = w.t + HOLD_S + 2 * w.dt
    while w.t < lapse:
        m.tick()
    assert w.operator == -1
    m.tick()
    assert w.victims[vi].state == CARRIED, "autonomy never took the casualty back"


def test_the_badge_is_offered_exactly_the_action_the_key_performs():
    """`operator_offer` is what the dashboard draws and `operator_act` is what happens.
    A hint that does not match the key is worse than no hint at all."""
    m = _demo()
    w = m.world
    i = _lane_robot(m, "gripper")
    rid = w.robot_ids[i]
    _held(m, i)
    assert m.bridge.manual.wire(w)[2] == OPERATOR_ACTION["none"], "nothing is in reach"

    _casualty_at(m, i)
    assert m.bridge.manual.wire(w)[2] == OPERATOR_ACTION["pick_up"]
    _say(m, t="act", robot=rid)
    m.tick()
    assert m.bridge.manual.wire(w)[2] == OPERATOR_ACTION["set_down"]


def test_the_key_reaches_exactly_as_far_as_the_swarms_own_pickup():
    """`REACH_GRAB` and not a metre more -- the key is a trigger for a rule the swarm
    already follows, not a longer arm for the operator."""
    m = _demo()
    w = m.world
    i = _lane_robot(m, "gripper")
    vi = _casualty_at(m, i)
    w.victims[vi].pos = w.pos[i] + np.array([REACH_GRAB + 0.2, 0.0])
    assert w.operator_offer(i) == OPERATOR_ACTION["none"]
    assert w.operator_act(i) == OPERATOR_ACTION["none"]
    assert w.victims[vi].state == CLEARED

    w.victims[vi].pos = w.pos[i] + np.array([REACH_GRAB - 0.2, 0.0])
    assert w.operator_act(i) == OPERATOR_ACTION["pick_up"]


def test_a_lane_with_nothing_to_do_is_offered_nothing():
    """A scout has no actuator to act with, and the badge says so rather than inventing
    a key that does nothing when pressed."""
    m = _demo()
    w = m.world
    i = _lane_robot(m, "none")
    _casualty_at(m, i)
    assert w.operator_offer(i) == OPERATOR_ACTION["none"]
    assert w.operator_act(i) == OPERATOR_ACTION["none"]
    assert w.carrying[i] == -1


def test_the_action_key_flies_a_drone_and_sets_it_down():
    """A driven rotor's height is the operator's, and it is the key that changes it --
    not the throttle. The rotor lane is the one that cannot carry, so this is its action.
    """
    m = _demo()
    w = m.world
    i = _rotor(m)
    rid = w.robot_ids[i]
    _held(m, i)
    # The lease grounds it: with the latch off and nothing to fly to, autonomy's height
    # is no longer the one being applied (`Mission.tick`).
    assert not w.airborne[i] and w.can_land()[i]
    assert w.operator_offer(i) == OPERATOR_ACTION["take_off"]

    _say(m, t="act", robot=rid)
    m.tick()
    assert w.operator_hover, "the key did not latch the drone airborne"
    m.tick()
    assert w.airborne[i], "Mission did not act on the latch"
    assert w.operator_offer(i) == OPERATOR_ACTION["land"]

    _say(m, t="act", robot=rid)
    m.tick()
    assert not w.operator_hover
    m.tick()
    assert not w.airborne[i], "the key did not set the drone down"


def test_a_landed_drone_does_not_taxi_and_says_so():
    """The behaviour change the latch buys, and the one the badge has to cover.

    `W` used to take a rotor off implicitly. Now it does nothing until the operator says
    so, which is only defensible because the unit offers TAKE OFF the moment the keys
    touch it -- so the offer is asserted here next to the stillness that makes it needed.
    """
    m = _demo()
    w = m.world
    i = _rotor(m)
    _held(m, i)
    at = w.pos[i].copy()
    _say(m, t="drive", robot=w.robot_ids[i], v=1.0, w=0.0)
    for _ in range(20):
        m.tick()
    assert not w.airborne[i]
    assert np.allclose(w.pos[i], at), "a landed rotor taxied across the ground"
    assert m.bridge.manual.wire(w)[2] == OPERATOR_ACTION["take_off"], (
        "the keys took the unit but the badge never offered the key that moves it")


def test_a_drone_let_go_of_mid_flight_goes_back_to_its_autonomys_height():
    """The latch is the lease's, not the robot's: a rotor left hovering forever because
    an operator wandered off is a unit quietly removed from the swarm."""
    m = _demo()
    w = m.world
    i = _rotor(m)
    _held(m, i)
    _say(m, t="act", robot=w.robot_ids[i])
    m.tick()
    assert w.operator_hover
    lapse = w.t + HOLD_S + 2 * w.dt
    while w.t < lapse:
        m.tick()
    assert w.operator == -1 and not w.operator_hover


def test_a_downed_unit_is_offered_no_action():
    m = _demo()
    w = m.world
    i = _lane_robot(m, "gripper")
    _casualty_at(m, i)
    w.status[i] = FAILED
    assert w.operator_offer(i) == OPERATOR_ACTION["none"]
    _say(m, t="act", robot=w.robot_ids[i])
    m.tick()
    assert m.bridge.manual.robot == -1 and w.carrying[i] == -1


def _against_a_wall(w: World, i: int) -> tuple[float, float]:
    """Robot `i`'s body 1 cm from the face of a wall to its east, on ground it can use.

    Not the centre of the neighbouring cell: cells are 0.5 m, so from there a half-speed
    reverse step does not reach the wall this tick and the floor is right not to fire.
    """
    ok = w.chassis_passable[w.chassis[i]]
    for cy, cx in zip(*np.nonzero(~w.passable), strict=True):
        if cx >= 3 and ok[cy, cx - 1] and ok[cy, cx - 2]:
            return cx * w.cell - w.radius[i] - 0.01, (cy + 0.5) * w.cell
    raise AssertionError("no wall with traversable ground beside it")


def test_the_safety_floor_holds_when_the_operator_reverses_into_a_wall():
    """Invariant 2. A floor that swept only ahead would let a reversing robot into
    anything behind it -- reverse is new, so this is the case that was never tested."""
    w = World(Scenario.load("test"), 5)
    nav = NavSet(w)
    reflex = ReflexController(w)
    w.pos[0] = _against_a_wall(w, 0)
    no_goal = np.full(w.n, -1, dtype=np.int32)
    back = -REVERSE_SPEED * float(w.v_max[0])

    w.theta[0] = np.pi                         # facing west, the wall behind it
    v, _ = reflex.commands(w, nav, [], no_goal, manual=(0, back, 0.0))
    assert v[0] == 0.0, "the operator reversed a robot into a wall"

    w.theta[0] = 0.0                           # facing the wall, backing away from it
    v, _ = reflex.commands(w, nav, [], no_goal, manual=(0, back, 0.0))
    assert v[0] == back, "the floor stopped a robot reversing onto open ground"
    v, _ = reflex.commands(w, nav, [], no_goal, manual=(0, float(w.v_max[0]), 0.0))
    assert v[0] == 0.0, "the operator drove a robot into a wall"


def test_a_second_unit_takes_over_and_the_first_goes_straight_back():
    m = _demo()
    w = m.world
    a, b = w.robot_ids[1], w.robot_ids[2]
    _say(m, t="drive", robot=a, v=1.0, w=0.0)
    m.tick()
    assert m.bridge.manual.robot == 1
    # Switching units: the dashboard releases the old one and drives the new one in the
    # same frame. Both land on one tick -- no tick with neither, none with both.
    _say(m, t="release", robot=a)
    _say(m, t="drive", robot=b, v=1.0, w=0.0)
    m.tick()
    assert m.bridge.manual.robot == 2
    # A drive for another unit takes over even without the release.
    _say(m, t="drive", robot=a, v=1.0, w=0.0)
    m.tick()
    assert m.bridge.manual.robot == 1
    # A release for a unit nobody is driving changes nothing.
    _say(m, t="release", robot=b)
    m.tick()
    assert m.bridge.manual.robot == 1
    _say(m, t="release", robot=a)
    m.tick()
    assert m.bridge.manual.robot == -1


def test_a_downed_unit_cannot_be_driven_and_drops_out_of_control():
    m = _demo()
    w = m.world
    w.status[3] = FAILED
    _say(m, t="drive", robot=w.robot_ids[3], v=1.0, w=0.0)
    m.tick()
    assert m.bridge.manual.robot == -1

    _say(m, t="drive", robot=w.robot_ids[4], v=1.0, w=0.0)
    m.tick()
    assert m.bridge.manual.robot == 4
    w.status[4] = FAILED
    m.tick()
    assert m.bridge.manual.robot == -1


def test_bad_frames_from_the_dashboard_do_not_stop_the_mission():
    m = _demo()
    rid = m.world.robot_ids[0]
    _say(m, t="drive", robot=rid, v="fast", w=0.0)
    _say(m, t="drive", robot=rid, v=float("nan"), w=0.0)
    _say(m, t="drive", robot="nobody", v=1.0, w=0.0)
    _say(m, t="drive")
    m.bridge.server._commands.append([1, 2])
    _say(m, t="something_else")
    m.tick()
    m.tick()
    assert m.bridge.manual.robot == -1
    # Not ours, so left where they were for whatever reads them.
    assert m.bridge.commands == [[1, 2], {"t": "something_else"}]

    # Out-of-range inputs are clamped to the unit's limits, not refused.
    _say(m, t="drive", robot=rid, v=40.0, w=-9.0)
    m.tick()
    i, v, omega = m.bridge.manual.command(m.world)
    assert (i, v, omega) == (0, float(m.world.v_max[0]) * OPERATOR_SPEED,
                             -float(m.world.omega_max[0]))


def test_headless_has_no_override_at_all():
    """Only the demo has a bridge, so no evaluation, gate or hash can meet this code."""
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    assert m.bridge is None
    m.tick()


def test_native_dashboard_keys_badge_and_handoff(tmp_path):
    """The Godot half, run in real headless Godot: key state, the input order that lets
    the drive keys shadow S only in a unit view, keep-alives, the stop on key-up, the
    release on switching units, and the badge's draw path in every mode."""
    import shutil
    import subprocess
    from pathlib import Path

    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Native Godot unavailable; the wire agreement is tested separately")
    root = Path(__file__).resolve().parents[1]
    # --quit-after: a failed GDScript assert stops the check's coroutine before `quit()`,
    # and without a frame cap Godot then idles until the timeout and hides the reason.
    result = subprocess.run(
        [native, "--headless", "--path", str(root / "godot"), "--quit-after", "600",
         "--log-file", str(tmp_path / "godot.log"),
         "--script", "res://tests/manual_drive_check.gd"],
        text=True, capture_output=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "MANUAL_DRIVE_OK" in result.stdout, result.stdout + result.stderr
    assert "SCRIPT ERROR" not in result.stderr, result.stderr


def test_space_picks_up_a_casualty_the_carrier_is_standing_on():
    """The reported bug: stand a carrier on a casualty, press space, nothing happens."""
    import numpy as np

    from swarmmind.contracts.schemas import OPERATOR_ACTION
    from swarmmind.control.manual import ManualOverride
    from swarmmind.sim.robot import LANE_INDEX
    from swarmmind.sim.scenario import Scenario
    from swarmmind.sim.world import CARRIED, CLEARED, World

    w = World(Scenario.load("test"), 42)
    i = int(np.nonzero(w.actuator == LANE_INDEX["gripper"])[0][0])
    v = w.victims[0]
    v.state, v.buried, v.debris_remaining = CLEARED, False, 0.0
    w.pos[i] = v.pos                                     # right on top of it

    assert w.operator_offer(i) == OPERATOR_ACTION["pick_up"], "the badge does not offer it"
    manual = ManualOverride(w)
    manual.receive(w, {"t": "act", "robot": w.robot_ids[i]})
    assert int(w.carrying[i]) == 0 and v.state == CARRIED, "space did not pick it up"
    assert manual.wire(w)[3] == v.id, "the dashboard is not told what it is carrying"

    # Pressing again sets it down rather than picking up a second one.
    assert w.operator_offer(i) == OPERATOR_ACTION["set_down"]
    manual.receive(w, {"t": "act", "robot": w.robot_ids[i]})
    assert int(w.carrying[i]) == -1 and manual.wire(w)[3] == ""


def test_a_buried_casualty_says_why_the_key_will_not_lift_it():
    """Silence is what made the key look broken. `FOUND` is already known to the swarm."""
    import numpy as np

    from swarmmind.contracts.schemas import OPERATOR_ACTION
    from swarmmind.control.manual import ManualOverride
    from swarmmind.sim.robot import LANE_INDEX
    from swarmmind.sim.scenario import Scenario
    from swarmmind.sim.world import FOUND, World

    w = World(Scenario.load("test"), 42)
    i = int(np.nonzero(w.actuator == LANE_INDEX["gripper"])[0][0])
    v = w.victims[0]
    v.state, v.buried, v.debris_remaining = FOUND, True, 1.0
    w.pos[i] = v.pos

    assert w.operator_offer(i) == OPERATOR_ACTION["dig_first"]
    seen = []
    w._emit = lambda kind, text, **kw: seen.append(text)
    ManualOverride(w).receive(w, {"t": "act", "robot": w.robot_ids[i]})
    assert int(w.carrying[i]) == -1, "a buried casualty was lifted straight out of the rubble"
    assert seen and "under debris" in seen[0], "the operator was told nothing"


def test_a_casualty_the_swarm_has_not_found_still_offers_nothing():
    """The offer must never reveal a HIDDEN casualty -- that is invariant #3."""
    import numpy as np

    from swarmmind.contracts.schemas import OPERATOR_ACTION
    from swarmmind.sim.robot import LANE_INDEX
    from swarmmind.sim.scenario import Scenario
    from swarmmind.sim.world import HIDDEN, World

    w = World(Scenario.load("test"), 42)
    i = int(np.nonzero(w.actuator == LANE_INDEX["gripper"])[0][0])
    v = w.victims[0]
    v.state = HIDDEN
    w.pos[i] = v.pos
    assert w.operator_offer(i) == OPERATOR_ACTION["none"], (
        "standing on an undetected casualty told the operator it was there")

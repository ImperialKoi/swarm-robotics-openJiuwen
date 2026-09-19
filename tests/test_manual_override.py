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
from swarmmind.control.manual import CMD_TTL_S, HOLD_S
from swarmmind.control.planner import NavSet
from swarmmind.control.tier1_reflex import ReflexController
from swarmmind.mission import Mission
from swarmmind.nodes.bridge import BridgeNode
from swarmmind.sim import grid
from swarmmind.sim.robot import CHASSIS_INDEX, FAILED, REVERSE_SPEED
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World


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
        top = w.v_max[i] * w.dt * (1.0 if v > 0 else REVERSE_SPEED)
        assert np.sign(step @ heading) == np.sign(v), f"v={v} moved the wrong way"
        assert np.isclose(np.linalg.norm(step), top), f"v={v} moved {step}, not {top}"


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
    assert (i, v, omega) == (0, float(m.world.v_max[0]), -float(m.world.omega_max[0]))


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

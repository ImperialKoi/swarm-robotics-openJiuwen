"""The relay chain, which is the mechanism M-39 identified as capping the mission.

Reach gates exploration, exploration gates discovery, and discovery gates rescues --
so a regression here is invisible in the relay lane and shows up as a rescue count.
Both properties below are the ones that were false before D12.
"""

from __future__ import annotations

import numpy as np

from swarmmind.control.planner import FrontierTarget, frontier_targets
from swarmmind.mission import Mission
from swarmmind.nodes.skill_executor import SkillExecutor
from swarmmind.nodes.tasks import TaskGenerator, ray_exit
from swarmmind.sim.robot import LANE_INDEX, OUT_OF_COMMS
from swarmmind.sim.scenario import Scenario
from swarmmind.sim.world import World

WARMUP_TICKS = 120  # enough for a frontier to exist and the first chains to commit


def _mission_with_frontier():
    m = Mission(Scenario.load("test"), 42, hivemind=False)
    for _ in range(WARMUP_TICKS):
        m.tick()
    return m


def _reach(posts, base) -> float:
    return max((float(np.linalg.norm(np.asarray(p) - base)) for p in posts), default=0.0)


def test_chains_are_offered_ground_beyond_the_frontier():
    """M-39's deadlock, as a property.

    The frontier is the edge of explored ground and explored ground ends at the comms
    boundary, so a chain that stops at the frontier can never move the boundary that
    defines it. `lookahead_steps=0` is the old behaviour, kept here as the control: the
    assertion is that looking ahead reaches strictly further, not that some absolute
    distance is hit, which would only be a fact about this fixture.

    Rays are swept rather than taken one at a time because the test map has a river and
    ditches in it -- a single ray can legitimately hit impassable ground and stop the
    chain early, which is the *other* behaviour under test and not a failure.
    """
    m = _mission_with_frontier()
    w = m.world
    base = np.asarray(w.scn.base, dtype=float)
    # The relay lane is tiny in this fixture (4) and fully committed by now. Free it:
    # what is under test is which posts get generated, not who is standing on one.
    for i in range(w.n):
        if m.executor.assignment[i] is not None and m.executor.assignment[i].kind == "relay":
            m.executor.assignment[i] = None

    control, ahead = [], []
    for angle in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False):
        d = np.array([np.cos(angle), np.sin(angle)])
        span = min(50.0, ray_exit(base, d, w))
        if span < w.scn.comms.relay_radius:
            continue
        fts = [FrontierTarget(pos=tuple(base + d * span), size=10, sector=0)]
        c = TaskGenerator(lookahead_steps=0).relay_posts(w, m.executor, fts)
        a = TaskGenerator(lookahead_steps=3).relay_posts(w, m.executor, fts)
        assert _reach(a, base) >= _reach(c, base), (
            "looking ahead reached less far than stopping at the frontier"
        )
        control.append(_reach(c, base))
        ahead.append(_reach(a, base))

    assert control, "no usable ray on this fixture"
    assert max(ahead) > max(control), (
        "the chain still stops at the frontier -- the M-39 deadlock is back"
    )


def test_no_more_posts_are_announced_than_relays_to_fill_them():
    """A chain with a hole in it carries nothing.

    Lengthening chains without this spends the lane on partial chains: more posts,
    less reach. Every alive relay is put on a post here, so the correct answer is to
    announce none at all.
    """
    m = _mission_with_frontier()
    w = m.world
    fts = frontier_targets(w, max_targets=48)
    gen = TaskGenerator()

    alive_relay = (w.actuator == LANE_INDEX["antenna"]) & (w.status <= OUT_OF_COMMS)
    assert alive_relay.any(), "fixture has no relays; the test would pass vacuously"

    from swarmmind.nodes.skill_executor import Assignment

    for i in np.nonzero(alive_relay)[0]:
        m.executor.assignment[int(i)] = Assignment(
            task_id=f"t{int(i)}", kind="relay",
            target=(float(w.pos[i, 0]), float(w.pos[i, 1])),
        )
    assert gen.relay_posts(w, m.executor, fts) == []


def test_ray_exit_keeps_the_endpoint_inside_the_map():
    """`grid.world_to_cell` clips instead of failing, so an off-map target silently
    becomes an on-map one and every chain leaving on that side piles onto the border."""
    m = _mission_with_frontier()
    w = m.world
    origin = np.asarray(w.scn.base, dtype=float)
    for ang in np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False):
        d = np.array([np.cos(ang), np.sin(ang)])
        end = origin + d * ray_exit(origin, d, w)
        assert 0.0 <= end[0] <= w.shape[1] * w.cell
        assert 0.0 <= end[1] <= w.shape[0] * w.cell


def _relay_fixture():
    """Fresh world, every relay idle at spawn, one chain aimed away from base."""
    w = World(Scenario.load("test"), 42)
    ex = SkillExecutor(w)
    for i in range(w.n):
        ex.assignment[i] = None
    w.status[:] = 0
    w.in_comms[:] = True
    base = np.asarray(w.scn.base, dtype=float)
    w.pos[w.actuator == LANE_INDEX["antenna"]] = base

    for angle in np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False):
        d = np.array([np.cos(angle), np.sin(angle)])
        span = min(90.0, ray_exit(base, d, w))
        if span < w.scn.comms.relay_radius * 1.5:
            continue
        fts = [FrontierTarget(pos=tuple(base + d * span), size=10, sector=0)]
        if TaskGenerator(lookahead_steps=0).relay_posts(w, ex, fts):
            return w, ex, fts
    raise AssertionError("no direction out of base produced a chain")


def _post_in_the_field(w, posts):
    """The first post far enough out that a relay on it is a chain, not the spawn wall."""
    base = np.asarray(w.scn.base, dtype=float)
    for q in posts:
        if float(np.linalg.norm(np.asarray(q) - base)) > w.scn.comms.base_radius:
            return np.asarray(q, dtype=float)
    raise AssertionError("chain never left base radius")


def test_relays_at_spawn_are_never_anchors():
    """The wall in `relay_posts`: relays inside base radius blocked every first link."""
    w, ex, fts = _relay_fixture()
    assert TaskGenerator(lookahead_steps=0).relay_posts(w, ex, fts), (
        "relays sitting on base must not anchor the ground the first link needs")


def test_mobile_anchors_stays_off():
    """M-69: planning chains around drifting relays cost 1.5 rescues and 2.0 found."""
    assert TaskGenerator().mobile_anchors is False


def test_an_idle_relay_in_the_field_can_count_as_network_topology():
    """The opt-in mechanism, kept switchable because M-69 may not hold at other radii."""
    w, ex, fts = _relay_fixture()
    gen = TaskGenerator(lookahead_steps=0, mobile_anchors=True)
    rr = w.scn.comms.relay_radius
    before = gen.relay_posts(w, ex, fts)

    relay = int(np.nonzero(w.actuator == LANE_INDEX["antenna"])[0][0])
    w.pos[relay] = _post_in_the_field(w, before)           # park it on that post
    after = gen.relay_posts(w, ex, fts)

    assert all(float(np.linalg.norm(np.asarray(p) - w.pos[relay])) > rr * 0.75
               for p in after), "the planner re-announced a link that already exists"


def test_a_relay_out_of_contact_is_not_topology():
    """A drifted relay is rung 1's next job, not a link to plan around."""
    w, ex, fts = _relay_fixture()
    gen = TaskGenerator(lookahead_steps=0, mobile_anchors=True)
    before = gen.relay_posts(w, ex, fts)

    relay = int(np.nonzero(w.actuator == LANE_INDEX["antenna"])[0][0])
    w.pos[relay] = _post_in_the_field(w, before)
    w.in_comms[relay] = False
    assert gen.relay_posts(w, ex, fts) == before, (
        "an out-of-contact relay must not suppress the post that would restore the link")

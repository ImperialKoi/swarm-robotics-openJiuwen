"""Tier 3: the hivemind, its filter, and the ways a strategic layer can go wrong.

Two failure modes are worth more than the rest combined, and most of this file is aimed
at them:

  * **A directive that does nothing.** If `abandon` only prints a line in a feed, the
    hivemind is decoration and the demo's central claim is false in the other direction.
  * **A directive that never stops.** A stale `abandon` locking a third of the map is the
    quietest way for this system to fail -- the swarm keeps working, just not where the
    casualties are, and nothing looks broken.
"""

from __future__ import annotations

import json
import re
import time

import numpy as np
import pytest

from swarmmind.hivemind.fallback import build_ladder
from swarmmind.hivemind.filter import MAX_DIRECTIVES, MIN_OPEN_SECTORS, FeasibilityFilter
from swarmmind.hivemind.prompt import SYSTEM, build
from swarmmind.hivemind.providers.scripted import ScriptedProvider
from swarmmind.mission import Mission
from swarmmind.nodes.hivemind import HivemindNode
from swarmmind.nodes.tasks import TaskGenerator, abandoned_preemptions
from swarmmind.sim.scenario import Scenario


def _mission(**kw):
    return Mission(Scenario.load("test"), 42, scripted_hivemind=True, **kw)


def _payload(*directives, reasoning="because"):
    return json.dumps({"reasoning": reasoning, "directives": list(directives)})


def _d(sector, priority="high", action="explore"):
    return {"sector": sector, "priority": priority, "action": action, "reason": "test"}


def _settle(node, m, limit: float = 20.0):
    """Pump the node until its in-flight request resolves.

    Tier 3 spans ticks now: one step starts a request, a later one collects it. Tests
    have to pump rather than assume a single call does everything.
    """
    deadline = time.perf_counter() + limit
    node.step(m.world, m.executor, m.tracker, m.events)
    while node._inflight is not None and time.perf_counter() < deadline:
        time.sleep(0.01)
        node.step(m.world, m.executor, m.tracker, m.events)
    return node


# --------------------------------------------------------------------------- the filter


def test_filter_accepts_a_well_formed_directive():
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    msg, err = f.parse(_payload(_d(m.world.sector_ids[0])))
    assert err is None
    accepted, rejected, reasoning = f.validate(msg)
    assert len(accepted) == 1 and not rejected and reasoning == "because"


@pytest.mark.parametrize("raw", [
    "",
    "I'm sorry, I can't help with that.",
    "{",
    '{"reasoning": "x"}',                      # no directives key
    '{"directives": "all of them"}',           # wrong type
    '[{"sector": "A1"}]',                      # top level is not an object
])
def test_filter_survives_garbage(raw):
    """A 1.5B model under a grammar still emits surprises. None may raise."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    msg, err = f.parse(raw)
    if msg is None:
        assert err
    else:
        f.validate(msg)  # must not raise


def test_filter_tolerates_fences_and_prose():
    """Small models like to explain themselves before the JSON. That is not a rejection."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    body = _payload(_d(m.world.sector_ids[0]))
    msg, err = f.parse(f"Here is my plan:\n```json\n{body}\n```\nHope that helps!")
    assert err is None and msg is not None
    assert len(f.validate(msg)[0]) == 1


def test_filter_rejects_unknown_sector_and_bad_enums():
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    sid = m.world.sector_ids[0]
    msg, _ = f.parse(_payload(
        _d("ZZ99"),                              # F2
        _d(sid, priority="urgent"),              # F3
        _d(m.world.sector_ids[1], action="nuke"),  # F3
    ))
    accepted, rejected, _ = f.validate(msg)
    assert not accepted and len(rejected) == 3


def test_filter_caps_directive_count():
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    # "high", not "normal": every sector starts at normal, so a normal directive is a
    # no-op and F10 would reject it before F4 ever counted.
    msg, _ = f.parse(_payload(*[
        _d(s, priority="high") for s in m.world.sector_ids[:MAX_DIRECTIVES + 3]
    ]))
    accepted, rejected, _ = f.validate(msg)
    assert len(accepted) == MAX_DIRECTIVES
    assert len(rejected) == 3


def test_filter_refuses_to_abandon_the_whole_map():
    """The failure mode that looks most like decisiveness."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    msg, _ = f.parse(_payload(*[
        _d(s, priority="abandon", action="abandon") for s in m.world.sector_ids
    ]))
    accepted, rejected, _ = f.validate(msg)
    assert rejected
    remaining = int((~m.world.sector_abandoned).sum()) - len(accepted)
    assert remaining >= MIN_OPEN_SECTORS


def test_filter_will_not_abandon_a_sector_holding_a_known_casualty():
    m = _mission()
    w = m.world
    sid = _a_plain_sector(w)
    f = FeasibilityFilter(w, _FakeTracker([_sector_centre(w, sid)]))
    k = w.sector_ids.index(sid)
    w.sector_hazard_known[k] = 0.1                       # not remotely burning
    msg, _ = f.parse(_payload(_d(sid, priority="abandon", action="abandon")))
    accepted, rejected, _ = f.validate(msg)
    assert not accepted and "casualty" in rejected[0].rule


def test_a_genuinely_burning_sector_can_be_abandoned_anyway():
    """F7 is a sanity check, not a veto. Writing off a lost sector is a real decision."""
    m = _mission()
    w = m.world
    sid = _a_plain_sector(w)
    f = FeasibilityFilter(w, _FakeTracker([_sector_centre(w, sid)]))
    w.sector_hazard_known[w.sector_ids.index(sid)] = 0.95
    msg, _ = f.parse(_payload(_d(sid, priority="abandon", action="abandon")))
    accepted, _, _ = f.validate(msg)
    assert len(accepted) == 1


def test_filter_will_not_abandon_a_collection_point():
    m = _mission()
    w = m.world
    home = w._sector_at(w.scn.base)
    f = FeasibilityFilter(w, m.tracker)
    msg, _ = f.parse(_payload(_d(home, priority="abandon", action="abandon")))
    accepted, rejected, _ = f.validate(msg)
    assert not accepted and "collection point" in rejected[0].rule


class _FakeTracker:
    def __init__(self, positions):
        self._p = positions

    def believed_positions(self):
        return self._p

    def open_reports(self):
        return []

    def resolved_victims(self):
        return []


def _a_plain_sector(world) -> str:
    """A sector that is neither the base nor an extraction zone, so F8 stays out of it."""
    homes = {world._sector_at(z) for z in (world.scn.base, *world.scn.extraction_zones)}
    return next(s for s in world.sector_ids if s not in homes)


def _sector_centre(world, sid):
    k = world.sector_ids.index(sid)
    ys, xs = np.nonzero(world.sector_of_cell == k)
    return (float(xs.mean()) * world.cell, float(ys.mean()) * world.cell)


# ------------------------------------------------------------------- mechanical effect


def test_abandoning_a_sector_stops_producing_search_work_there():
    """The difference between a strategic layer and a text feed."""
    m = _mission()
    w, gen = m.world, TaskGenerator()
    for _ in range(40):
        m.tick()
    before = gen.generate(w, m.executor, m.tracker)
    target = next((t for t in before if t.kind in ("explore", "investigate")), None)
    if target is None:
        pytest.skip("no search work open at this point in the mission")

    k = int(w.sector_of_cell[
        int(target.target[1] / w.cell), int(target.target[0] / w.cell)
    ])
    w.sector_abandoned[k] = True
    after = gen.generate(w, m.executor, m.tracker)
    assert not any(
        t.kind in ("explore", "investigate", "relay")
        and int(w.sector_of_cell[int(t.target[1] / w.cell), int(t.target[0] / w.cell)]) == k
        for t in after
    ), "an abandoned sector is still generating search work"


def test_high_priority_outranks_low_within_the_same_kind():
    m = _mission()
    w, gen = m.world, TaskGenerator()
    for _ in range(40):
        m.tick()
    base = {id(t): t.rank for t in gen.generate(w, m.executor, m.tracker)}
    if not base:
        pytest.skip("no open tasks yet")
    w.sector_priority[:] = 0                             # everything high
    high = gen.generate(w, m.executor, m.tracker)
    w.sector_priority[:] = 2                             # everything low
    low = gen.generate(w, m.executor, m.tracker)
    assert min(t.rank for t in high) < min(t.rank for t in low)


def test_robots_are_pulled_out_of_an_abandoned_sector():
    m = _mission()
    w = m.world
    for _ in range(60):
        m.tick()
    ix = (w.pos[:, 0] / w.cell).astype(int)
    iy = (w.pos[:, 1] / w.cell).astype(int)
    occupied = np.bincount(w.sector_of_cell[iy, ix], minlength=len(w.sector_ids))
    k = int(occupied.argmax())
    assert occupied[k] > 0
    assert not abandoned_preemptions(w, m.executor)      # nothing closed yet
    w.sector_abandoned[k] = True
    assert abandoned_preemptions(w, m.executor), "closing a sector left its robots in it"


# -------------------------------------------------------------------------- the node


def test_directives_expire():
    """The single most likely quiet failure: an abandon nobody renews and nobody clears."""
    m = _mission(hivemind=False)
    w = m.world
    node = HivemindNode(w, [ScriptedProvider(w, m.tracker)], m.tracker,
                        period=1.0, expiry=30.0)
    sid = w.sector_ids[1]
    node._apply(w, _d(sid, priority="abandon", action="abandon"), lambda *a, **k: None)
    k = w.sector_ids.index(sid)
    assert w.sector_abandoned[k]

    w.t = 29.0
    node._expire(w)
    assert w.sector_abandoned[k], "expired early"

    w.t = 31.0
    node._expire(w)
    assert not w.sector_abandoned[k], "a stale abandon outlived its 30 s"
    assert w.sector_priority[k] == 1
    assert not node.applied


def test_a_renewed_directive_does_not_expire():
    m = _mission(hivemind=False)
    w = m.world
    node = HivemindNode(w, [ScriptedProvider(w, m.tracker)], m.tracker, expiry=30.0)
    sid = w.sector_ids[1]
    node._apply(w, _d(sid, priority="high"), lambda *a, **k: None)
    w.t = 25.0
    node._apply(w, _d(sid, priority="high"), lambda *a, **k: None)   # renewal
    w.t = 40.0
    node._expire(w)
    assert node.applied, "a renewed directive expired on the original clock"


def test_renewal_is_counted_separately_from_a_decision():
    m = _mission(hivemind=False)
    w = m.world
    node = HivemindNode(w, [ScriptedProvider(w, m.tracker)], m.tracker)
    noop = lambda *a, **k: None                                       # noqa: E731
    assert node._apply(w, _d(w.sector_ids[1], priority="high"), noop) is True
    assert node._apply(w, _d(w.sector_ids[1], priority="high"), noop) is False
    assert node._apply(w, _d(w.sector_ids[1], priority="low"), noop) is True


def test_a_hanging_provider_cannot_stall_the_simulation():
    """The property that matters is not "the ladder recovers" -- it is that a single
    step returns immediately. DemoSim runs at wall-clock speed; a step that blocks for
    the rung timeout would freeze the dashboard for seconds at a time."""
    m = _mission(hivemind=False)
    w = m.world

    class Hangs:
        name = "hangs"

        def generate(self, system, user, timeout):
            time.sleep(30.0)
            return "{}"

    node = HivemindNode(w, [Hangs(), ScriptedProvider(w, m.tracker)], m.tracker,
                        period=0.0, timeout=0.3)
    worst = 0.0
    for _ in range(200):
        t0 = time.perf_counter()
        node.step(w, m.executor, m.tracker, m.events)
        worst = max(worst, time.perf_counter() - t0)
        if node.last_message is not None:
            break
        time.sleep(0.01)
    assert worst < 0.1, f"a single step blocked for {worst * 1000:.0f} ms"
    assert node.stats["timed_out"] >= 1
    assert node.last_message is not None and node.last_message["source"] == "scripted"


def test_a_dead_ladder_leaves_the_previous_directive_standing():
    m = _mission(hivemind=False)
    w = m.world

    class Broken:
        name = "broken"

        def generate(self, system, user, timeout):
            raise ConnectionRefusedError("no server")

    node = HivemindNode(w, [Broken()], m.tracker, period=0.0)
    node._apply(w, _d(w.sector_ids[1], priority="high"), lambda *a, **k: None)
    events = []
    emit = lambda k, t, **kw: events.append(k)                        # noqa: E731
    for _ in range(200):
        node.step(w, m.executor, m.tracker, m.events, emit=emit)
        if node.stats["offline"]:
            break
        time.sleep(0.01)
    assert node.stats["offline"] == 1
    assert w.sector_priority[w.sector_ids.index(w.sector_ids[1])] == 0
    assert "hivemind_offline" in events


def test_a_totally_rejected_message_spends_the_rung():
    """The ladder falls through when a model produces nothing survivable."""
    m = _mission(hivemind=False)
    w = m.world

    class AllBad:
        name = "bad"

        def generate(self, system, user, timeout):
            return _payload(_d("NOPE"), _d("ALSO_NOPE"))

    node = _settle(HivemindNode(w, [AllBad(), ScriptedProvider(w, m.tracker)], m.tracker,
                                period=0.0), m)
    assert node.stats["fell_through"] == 1
    assert node.last_message is not None
    assert node.last_message["source"] == "scripted"


def test_an_empty_directive_list_is_a_valid_answer():
    """"Nothing needs changing" must not be treated as a failed rung.

    Falling through on it would silently replace a correct model with the baseline every
    time the swarm was doing fine -- the exact situation where the model is right.
    """
    m = _mission(hivemind=False)
    w = m.world

    class SaysNothing:
        name = "quiet"

        def generate(self, system, user, timeout):
            return _payload(reasoning="the swarm is well distributed; no change")

    node = _settle(HivemindNode(w, [SaysNothing(), ScriptedProvider(w, m.tracker)],
                                m.tracker, period=0.0), m)
    assert node.stats["fell_through"] == 0
    assert node.last_message["source"] == "quiet"
    assert node.last_message["directives"] == []


# ------------------------------------------------------------------ prompt and ladder


def test_the_prompt_never_reveals_how_many_casualties_exist():
    """Invariant #3 reaches Tier 3 too: the denominator is privileged information."""
    m = _mission()
    for _ in range(40):
        m.tick()
    text = build(m.world, m.executor, m.tracker, m.events)
    total = len(m.world.victims)
    assert f"/{total}" not in text
    assert "unknown" in text.lower()


def test_prompt_size_does_not_grow_with_the_swarm():
    """768 robots and 16 robots must render the same size, or the local model chokes."""
    m = _mission()
    for _ in range(40):
        m.tick()
    text = build(m.world, m.executor, m.tracker, m.events)
    assert len(text) < 6000, f"prompt is {len(text)} chars"
    # The event feed names the handful of robots it reports on, which is context, not a
    # roster. What must never appear is a per-robot listing -- that would both explode
    # the prompt at 768 robots and invite Tier 3 to command individuals.
    named = sum(1 for rid in m.world.robot_ids if rid in text)
    assert named <= 6, f"{named} robot ids in the prompt -- that is a roster, not context"


def test_the_ladder_always_has_a_working_bottom_rung():
    m = _mission(hivemind=False)
    ladder = build_ladder(m.world, m.tracker)
    assert ladder and ladder[-1].name == "scripted"


def test_the_scripted_provider_output_passes_its_own_filter():
    """The fallback must never be the thing that gets rejected."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    p = ScriptedProvider(m.world, m.tracker)
    for _ in range(6):
        for _ in range(20):
            m.tick()
        msg, err = f.parse(p.generate(SYSTEM, "", 1.0))
        assert err is None, err
        accepted, rejected, _ = f.validate(msg)
        assert not any("invalid" in r.rule or "unknown" in r.rule for r in rejected)


def test_a_mission_with_the_hivemind_running_still_completes():
    m = _mission()
    card = m.run(max_time=90.0)
    assert card.directives_issued > 0
    assert m.world.t >= 90.0 or m.world.done


def test_the_fallback_config_cannot_contradict_the_filter():
    """A baseline tuned to propose what its own filter forbids is worse than no baseline.

    The two thresholds are in different files by necessity -- one is scenario config, the
    other a safety rule -- so nothing but this check keeps them consistent.
    """
    import yaml

    from swarmmind.hivemind.filter import ABANDON_HAZARD_FLOOR
    from swarmmind.hivemind.providers.scripted import CONFIG

    cfg = yaml.safe_load(CONFIG.read_text())
    assert cfg["abandon_with_casualty_above"] <= ABANDON_HAZARD_FLOOR
    assert 0.0 < cfg["abandon_above"] <= 1.0
    assert cfg["push_sectors"] >= 1

    m = _mission(hivemind=False)
    bad = dict(cfg, abandon_with_casualty_above=ABANDON_HAZARD_FLOOR + 0.1)
    with pytest.raises(ValueError, match="reject itself"):
        ScriptedProvider(m.world, m.tracker, config=bad)


@pytest.mark.parametrize("priority", ["high", "low"])
def test_filter_rejects_abandon_combined_with_a_search_priority(priority):
    """F9: a sector cannot both close to new work and outrank everything for it."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    sid = _a_plain_sector(m.world)
    msg, _ = f.parse(_payload(_d(sid, priority=priority, action="abandon")))
    accepted, rejected, _ = f.validate(msg)
    assert not accepted and "contradicts" in rejected[0].rule


def test_a_coherent_abandon_still_passes():
    """F9 must reject the contradiction, not abandonment itself."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    sid = _a_plain_sector(m.world)
    msg, _ = f.parse(_payload(_d(sid, priority="abandon", action="abandon")))
    assert len(f.validate(msg)[0]) == 1


def test_the_recent_feed_is_not_drowned_in_task_churn():
    """At 768 robots `task_awarded` fires several times a second.

    Left unfiltered it was the entire RECENT section, so the model's only narrative
    context was a list of task ids -- and four of the remaining lines were the hivemind
    quoting its own previous reasoning back at itself.
    """
    m = _mission()
    for _ in range(1200):
        m.tick()
    text = build(m.world, m.executor, m.tracker, m.events)
    assert "RECENT" in text
    feed = text.split("RECENT", 1)[1]
    for noisy in ("task_awarded", "->"):
        assert noisy not in feed, f"{noisy!r} is still crowding the feed"
    assert m.events.counts.get("task_awarded", 0) > 0, "no churn happened; test proves nothing"


def test_counts_are_distinct_casualties_not_reports():
    """Several reports resolve onto one person; counting reports claimed 48 of 8."""
    m = _mission()
    for _ in range(1200):
        m.tick()
    text = build(m.world, m.executor, m.tracker, m.events)
    confirmed = int(re.search(r"CONFIRMED (\d+)", text).group(1))
    assert confirmed <= len(m.world.victims), (
        f"prompt claims {confirmed} confirmed casualties; only "
        f"{len(m.world.victims)} exist"
    )
    assert confirmed <= len(m.tracker.resolved_victims())
def test_filter_rejects_a_directive_that_changes_nothing():
    """Four slots, and a 1.5B model spent a whole cycle sending `normal` to sectors that
    were already normal -- well-formed, accepted, zero effect."""
    m = _mission()
    f = FeasibilityFilter(m.world, m.tracker)
    sid = _a_plain_sector(m.world)
    assert m.world.sector_priority[m.world.sector_ids.index(sid)] == 1
    msg, _ = f.parse(_payload(_d(sid, priority="normal")))
    accepted, rejected, _ = f.validate(msg)
    assert not accepted and "changes nothing" in rejected[0].rule


def test_resetting_a_raised_sector_back_to_normal_is_a_real_change():
    """F10 must reject no-ops, not the act of standing a sector back down."""
    m = _mission()
    w = m.world
    sid = _a_plain_sector(w)
    w.sector_priority[w.sector_ids.index(sid)] = 0          # currently high
    f = FeasibilityFilter(w, m.tracker)
    msg, _ = f.parse(_payload(_d(sid, priority="normal")))
    assert len(f.validate(msg)[0]) == 1


def test_renewing_a_live_directive_is_exempt_from_the_no_op_rule():
    """A renewal looks exactly like a no-op and must survive, or the sector reopens."""
    m = _mission()
    w = m.world
    sid = _a_plain_sector(w)
    w.sector_priority[w.sector_ids.index(sid)] = 0          # raised by a live directive
    f = FeasibilityFilter(w, m.tracker)
    msg, _ = f.parse(_payload(_d(sid, priority="high")))

    assert not f.validate(msg)[0], "a repeat with no live directive should be a no-op"
    accepted, _, _ = f.validate(msg, active=frozenset({sid}))
    assert len(accepted) == 1, "a renewal was rejected; the directive would expire"


def test_a_live_abandon_can_be_renewed_indefinitely():
    """End-to-end: the node must keep a sector closed while it keeps asking for it."""
    m = _mission(hivemind=False)
    w = m.world
    node = HivemindNode(w, [ScriptedProvider(w, m.tracker)], m.tracker, expiry=30.0)
    sid = _a_plain_sector(w)
    d = _d(sid, priority="abandon", action="abandon")
    noop = lambda *a, **k: None                                       # noqa: E731

    node._apply(w, d, noop)
    k = w.sector_ids.index(sid)
    for step in range(1, 5):
        w.t = step * 20.0
        accepted, _, _ = node.filter.validate(
            {"reasoning": "", "directives": [d]}, frozenset(node.applied))
        assert accepted, f"renewal rejected at t={w.t}"
        node._apply(w, accepted[0], noop)
        node._expire(w)
        assert w.sector_abandoned[k], f"sector reopened at t={w.t} despite renewal"


def test_the_api_rung_is_skipped_cleanly_when_it_cannot_work():
    """`--hivemind-allow-api` on a machine with no key and no SDK must be a no-op.

    This is the configuration the project is actually developed in, so it is the one most
    likely to break unnoticed: the ladder has to drop the rung, not raise, and not spend a
    5 s timeout discovering it every cycle.
    """
    m = _mission(hivemind=False)
    ladder = build_ladder(m.world, m.tracker, allow_api=True)
    assert ladder, "asking for the API rung emptied the ladder"
    assert ladder[-1].name == "scripted", "the bottom rung must always be there"

    t0 = time.perf_counter()
    build_ladder(m.world, m.tracker, allow_api=True)
    assert time.perf_counter() - t0 < 1.0, "ladder construction is probing the network"


def test_ladder_construction_is_cheap_with_no_server_listening():
    """Training builds thousands of Missions; a slow probe per construction would cost."""
    m = _mission(hivemind=False)
    t0 = time.perf_counter()
    for _ in range(5):
        build_ladder(m.world, m.tracker)
    assert time.perf_counter() - t0 < 3.0

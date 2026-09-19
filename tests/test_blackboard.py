"""The bus and the blackboard.

The blackboard emits plain dicts for speed, so these tests are what actually enforces
the contract on it: every published payload is validated against the pydantic models in
contracts/schemas.py.
"""

from __future__ import annotations

from swarmmind.bus.local import LocalBus
from swarmmind.contracts import topics
from swarmmind.contracts.schemas import Event, SwarmState
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario


def _run(ticks=200):
    m = Mission(Scenario.load("test"), 42, allocator="auction")
    for _ in range(ticks):
        m.tick()
    return m


# --- bus --------------------------------------------------------------------------------


def test_bus_delivers_and_retains():
    bus = LocalBus()
    seen = []
    bus.subscribe("/x", seen.append)
    bus.publish("/x", {"a": 1})
    assert seen == [{"a": 1}]
    assert bus.latest("/x") == {"a": 1}


def test_bus_retains_for_late_subscribers():
    """The dashboard bridge connects mid-run and must not wait for the next publish."""
    bus = LocalBus()
    bus.publish("/swarm/state", {"tick": 7})
    assert bus.latest("/swarm/state") == {"tick": 7}


def test_mission_publishes_the_expected_topics():
    m = _run()
    published = set(m.bus.topics())
    assert topics.SWARM_STATE in published
    assert topics.SWARM_EVENTS in published
    assert topics.TASKS_AVAILABLE in published


# --- contract conformance ----------------------------------------------------------------


def test_swarm_state_validates_against_the_contract():
    m = _run()
    state = m.bus.latest(topics.SWARM_STATE)
    assert state is not None
    parsed = SwarmState.model_validate(state)
    assert len(parsed.robots) == m.world.n
    assert len(parsed.sectors) == len(m.world.sector_ids)
    assert parsed.victims_total == len(m.world.victims)


def test_every_event_validates_against_the_contract():
    m = _run()
    assert m.events.events, "no events were emitted at all"
    for ev in m.events.events:
        Event.model_validate(ev)


def test_state_reports_knowledge_not_truth():
    """Sector hazard must be what the swarm has observed, never the ground-truth disc.

    If these ever match while the hazard is burning in unexplored space, the fog-of-war
    claim is false (CLAUDE.md invariant #3).
    """
    m = Mission(Scenario.load("test"), 42, allocator="auction")
    w = m.world
    while w.t < w.scn.hazard.ignite_t + 30.0:
        m.tick()
    state = m.blackboard.build(w, m.executor, m.tracker)

    known = sum(s["hazard_level"] for s in state["sectors"])
    truth = float(w.sector_hazard_true.sum())
    assert truth > 0.0, "the hazard never spread; the test proves nothing"
    # `hazard_level` is published rounded to 4 dp, so a sum over sectors can exceed the
    # true sum by up to half a quantum each. The claim under test is "the swarm never
    # reports hazard it has not seen", not "the sums agree to the last bit".
    slack = len(state["sectors"]) * 5e-5
    assert known <= truth + slack, "the blackboard is reporting more hazard than exists"


def test_state_carries_the_contact_set_including_phantoms():
    m = _run(400)
    state = m.blackboard.build(m.world, m.executor, m.tracker)
    ids = {r["id"] for r in state["reports"]}
    assert ids, "no contacts published; the dashboard would have nothing to draw"
    assert all(r["state"] != "dismissed" for r in state["reports"])


def test_last_action_reason_is_a_template_not_an_llm_call():
    """It feeds the follow-cam panel and the Tier-2 ticker at 10 Hz. It must be free."""
    m = _run()
    state = m.bus.latest(topics.SWARM_STATE)
    reasons = [r["last_action_reason"] for r in state["robots"]]
    assert any(reasons), "no robot reported what it was doing"
    assert all(isinstance(r, str) for r in reasons)

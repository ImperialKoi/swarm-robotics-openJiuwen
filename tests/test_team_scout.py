"""The visual scout: the one team role that reads pixels, wired without paid requests."""

from __future__ import annotations

import asyncio
import json
from unittest.mock import MagicMock

import pytest

from swarmmind.hivemind.team.scout import DEFAULT_SCOUT_MODEL, SCOUT_SCHEMA, VisionScout
from swarmmind.hivemind.team.workflow import collaborate, heuristic_decide
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario


@pytest.fixture
def world():
    m = Mission(Scenario.load("test"), 42)
    yield m.world
    m.close()


@pytest.fixture
def key(monkeypatch):
    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")


def fake_http(monkeypatch, content):
    payload = {"choices": [{"finish_reason": "stop", "message": {"content": json.dumps(content)}}]}
    http = MagicMock()
    http.return_value.__enter__.return_value.read.return_value = json.dumps(payload).encode()
    monkeypatch.setattr("urllib.request.urlopen", http)
    return http


def settle(scout, tries=200):
    import time

    for _ in range(tries):
        out = scout.collect()
        if out is not None or not scout.busy:
            return out
        time.sleep(0.01)
    return None


CANDIDATES = [{"directive": {"sector": "A1", "priority": "high", "action": "explore"}},
              {"directive": {"sector": "B2", "priority": "high", "action": "rescue"}}]


def test_the_scout_is_shown_the_map_and_the_candidate_sectors(world, key, monkeypatch):
    http = fake_http(monkeypatch, {"note": "North bank is bare.", "sector": "B2"})
    scout = VisionScout(world)
    assert scout.start(world, CANDIDATES)
    finding = settle(scout)

    body = json.loads(http.call_args.args[0].data)
    assert body["model"] == DEFAULT_SCOUT_MODEL
    parts = body["messages"][1]["content"]
    assert [p["type"] for p in parts] == ["text", "image_url"], "the scout got no picture"
    assert "A1, B2" in parts[0]["text"]
    assert parts[1]["image_url"]["url"].startswith("data:image/png;base64,")
    assert finding == {"note": "North bank is bare.", "sector": "B2",
                       "by": "scout", "model": DEFAULT_SCOUT_MODEL}


def test_a_sector_the_scout_invented_is_dropped(world, key, monkeypatch):
    """A made-up id would point the lead at a candidate index that does not exist."""
    fake_http(monkeypatch, {"note": "Ridge looks thin.", "sector": "ZZ99"})
    scout = VisionScout(world)
    scout.start(world, CANDIDATES)
    finding = settle(scout)
    assert finding["sector"] == "", "an invented sector reached the lead"
    assert finding["note"] == "Ridge looks thin.", "the observation was thrown away too"


def test_one_look_at_a_time(world, key, monkeypatch):
    fake_http(monkeypatch, {"note": "n", "sector": "A1"})
    scout = VisionScout(world)
    assert scout.start(world, CANDIDATES)
    assert not scout.start(world, CANDIDATES), "a second look started while one was in flight"
    settle(scout)


def test_no_candidates_means_nothing_to_look_for(world, key, monkeypatch):
    fake_http(monkeypatch, {"note": "n", "sector": ""})
    assert not VisionScout(world).start(world, [])


def test_a_failed_look_costs_an_observation_not_an_episode(world, key, monkeypatch):
    def boom(*a, **kw):
        raise TimeoutError("venue wifi")

    monkeypatch.setattr("urllib.request.urlopen", boom)
    scout = VisionScout(world)
    scout.start(world, CANDIDATES)
    assert settle(scout) is None
    assert scout.stats["failed"] == 1 and scout.last is None


def test_the_note_is_bounded_because_three_models_pay_for_it(world, key, monkeypatch):
    fake_http(monkeypatch, {"note": "x" * 900, "sector": "A1"})
    scout = VisionScout(world)
    scout.start(world, CANDIDATES)
    assert len(settle(scout)["note"]) <= 180
    assert SCOUT_SCHEMA["properties"]["note"]["maxLength"] == 180
    assert SCOUT_SCHEMA["additionalProperties"] is False


# --------------------------------------------------------------- reaching the team

@pytest.fixture(scope="module")
def snapshot():
    """A real observation, from the real capture, so these test the shipped shape."""
    from swarmmind.hivemind.team.observations import capture

    m = Mission(Scenario.load("test"), 42)
    for _ in range(40):
        m.tick()
    snap = capture(m.world, m.executor, m.tracker, m.bus, "response-0001")
    snap["events"], snap["previous"] = [], {}
    m.close()
    return snap


def with_scout(snapshot, scout=None):
    snap = {**snapshot}
    if scout:
        snap["scout"] = scout
    return snap


class Cfg:
    goal = "Search and rescue."
    hazard_ceiling = 0.35
    explored_ceiling = 0.96
    max_candidates = 8


def test_the_lead_is_given_the_scouts_finding(snapshot):
    """Intermediate-result sharing: one agent's output is another's input."""
    seen = {}

    async def decide(role, context, count):
        seen.setdefault(role, context)
        return await heuristic_decide(role, context, count)

    first = snapshot["sectors"][0]["id"]
    asyncio.run(collaborate(
        with_scout(snapshot, {"note": "South bank is bare.", "sector": first, "by": "scout"}),
        Cfg(), decide, lambda *a, **kw: None))
    assert "scout" in seen["lead"], "the lead never saw the scout's note"
    assert seen["lead"]["scout"]["note"] == "South bank is bare."
    assert seen["lead"]["scout"]["suggests"] == first
    assert "evidence, not as a decision" in seen["lead"]["scout"]["weight"]


def test_the_team_works_unchanged_with_no_scout(snapshot):
    """The other three roles must not depend on the one that needs a network."""
    seen = {}

    async def decide(role, context, count):
        seen.setdefault(role, context)
        return await heuristic_decide(role, context, count)

    out = asyncio.run(collaborate(with_scout(snapshot), Cfg(), decide,
                                  lambda *a, **kw: None))
    assert "scout" not in seen["lead"]
    assert out["directive"] is not None, "the team stalled without a scout"


def test_the_scout_appears_in_the_decomposition_only_when_it_looked(snapshot):
    events = []
    first = snapshot["sectors"][0]["id"]
    asyncio.run(collaborate(with_scout(snapshot, {"note": "n", "sector": first}), Cfg(),
                            heuristic_decide,
                            lambda ev, **kw: events.append((ev, kw))))
    decomposed = [kw for ev, kw in events if ev == "decomposed"][0]
    assert "read the map" in decomposed["tasks"]
    assert any(ev == "observed" for ev, _ in events), "the scout's turn was not recorded"

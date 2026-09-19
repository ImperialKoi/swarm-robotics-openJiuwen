"""Team authority, knowledge boundaries, peer review and process failure handling."""

import asyncio
import copy
import json
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from swarmmind.cli import main
from swarmmind.hivemind.team.client import WorkerClient
from swarmmind.hivemind.team.config import TeamConfig
from swarmmind.hivemind.team.controller import ResponseTeam
from swarmmind.hivemind.team.observations import capture, observe_event
from swarmmind.hivemind.team.tools import MissionTools
from swarmmind.hivemind.team.trace import Trace
from swarmmind.hivemind.team.workflow import collaborate, heuristic_decide
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario


@pytest.fixture
def mission():
    m = Mission(Scenario.load("test"), 42, scripted_hivemind=True)
    yield m
    m.close()


def snapshot(m):
    return capture(m.world, m.executor, m.tracker, m.bus, "fixture")


def test_observation_is_detached_and_uses_an_explicit_knowledge_boundary(mission):
    w = mission.world
    allowed = {name: getattr(w, name) for name in (
        "in_comms", "status", "pos", "cell", "shape", "sector_of_cell", "sector_ids",
        "_sector_at", "scn", "sector_explored_pct", "sector_hazard_known",
        "sector_priority", "sector_abandoned", "actuator", "battery", "t",
    )}
    guarded = SimpleNamespace(**allowed)  # any undeclared simulator access fails
    a = capture(guarded, mission.executor, mission.tracker, mission.bus, "fixture")
    before = json.dumps(a, sort_keys=True)
    w.victims.clear()  # identical beliefs despite a radically different hidden world
    b = capture(guarded, mission.executor, mission.tracker, mission.bus, "fixture")
    assert a == b
    assert not any(word in before for word in ("victims_total", "seed", "appearance", "hazard_disc"))
    w.sector_priority[:] = 2
    assert json.dumps(a, sort_keys=True) == before


def test_disconnected_telemetry_is_not_exported_as_fresh(mission):
    mission.world.in_comms[:] = False
    snap = snapshot(mission)
    assert all(s["connected"] == 0 for s in snap["sectors"])
    assert all(lane == {"connected": 0, "battery": None} for lane in snap["lanes"].values())


def test_event_allowlist_strips_hidden_text_and_positions():
    assert observe_event({"kind": "hazard_ignited", "pos": [1, 2]}) is None
    assert observe_event({"kind": "victim_found", "t": 3, "text": "secret", "pos": [1, 2]}) == {
        "kind": "victim_found", "t": 3,
    }


def peer_fixture(m):
    snap = snapshot(m)
    for s in snap["sectors"]:
        s.update(explored=1, connected=0, backlog={}, hazard=0, abandoned=False)
    # Highest rescue demand is beyond radio coverage; another sector is reachable.
    snap["sectors"][0].update(backlog={"extract": 2}, connected=0)
    snap["sectors"][1].update(explored=0, connected=2)
    return snap


def test_logistics_evidence_changes_lead_plan_before_safety_review(mission):
    snap = peer_fixture(mission)
    rows = []
    result = asyncio.run(collaborate(snap, TeamConfig.load(), heuristic_decide,
                                    lambda event, **data: rows.append({"event": event, **data})))
    assert result["directive"]["sector"] == snap["sectors"][1]["id"]
    stages = [r["event"] for r in rows]
    assert stages.index("revision_requested") < stages.index("revised") < stages.index("reviewed")
    assert [r["role"] for r in rows if r["event"] == "message"] == [
        "lead", "logistics", "lead", "safety",
    ]
    capacity = next(r for r in rows if r.get("tool") == "read_capacity")
    assert capacity["result"]["in_contact_here"] == 0
    assert capacity["result"]["route_feasibility"] == "unknown"


def test_safety_veto_prevents_submission(mission):
    async def decide(role, context, count):
        if role == "safety":
            return {"choice": -1, "note": "Withhold this proposal."}
        return await heuristic_decide(role, context, count)

    result = asyncio.run(collaborate(peer_fixture(mission), TeamConfig.load(), decide,
                                    lambda *a, **kw: None))
    assert result["directive"] is None
    assert "Safety" in result["reason"]


@pytest.mark.parametrize("bad", [None, "x", {"choice": True, "note": "x"},
                                {"choice": 90, "note": "x"}, {"choice": 0}])
def test_malformed_role_response_fails_closed(mission, bad):
    async def decide(*args):
        return bad

    with pytest.raises(ValueError, match="malformed"):
        asyncio.run(collaborate(snapshot(mission), TeamConfig.load(), decide, lambda *a, **k: None))


def test_role_cannot_call_another_roles_tool(mission):
    tools = MissionTools(snapshot(mission), TeamConfig.load(), lambda *a, **k: None)
    with pytest.raises(ValueError, match="cannot call"):
        tools.call("lead", "check_plan", directive={})


@pytest.mark.parametrize("bad", ["x", [], {"sector": []},
                                {"sector": "A1", "action": "explore", "priority": "abandon"}])
def test_malformed_directive_is_rejected_without_crashing(mission, bad):
    assert not MissionTools(snapshot(mission), TeamConfig.load(), lambda *a, **k: None).check_plan(bad)["ok"]


class FakeWorker:
    def __init__(self, *args):
        self.rows = []
        self.requests = []
        self.closed = False

    def submit(self, snap):
        self.requests.append(copy.deepcopy(snap))

    def poll(self):
        rows, self.rows = self.rows, []
        return rows

    def close(self):
        self.closed = True


@pytest.fixture
def team(mission, monkeypatch, tmp_path):
    monkeypatch.setattr("swarmmind.hivemind.team.controller.WorkerClient", FakeWorker)
    t = ResponseTeam(mission.world, mission.bus, mode="local", trace_path=tmp_path / "trace.jsonl")
    yield t
    t.close()


def step(team, m):
    team.step(m.world, m.executor, m.tracker, m.bus, m.hivemind, m._emit)


def reply(team, mission):
    snap = team.worker.requests[-1]
    s = next(s for s in snap["sectors"] if s["connected"])
    return {"kind": "result", "request": snap["id"], "result": {
        "directive": {"sector": s["id"], "priority": "high", "action": "explore"},
        "reason": "Observed connected units can explore this sector.",
        "evidence": f"sector:{s['id']}",
    }}


def test_application_arbitration_duplicate_suppression_and_expiry(team, mission):
    step(team, mission)
    row = reply(team, mission)
    sid = row["result"]["directive"]["sector"]
    team.worker.rows = [row, row]
    step(team, mission)
    assert team.stats["applied"] == 1
    assert sid in mission.hivemind.protected_sectors
    k = mission.world.sector_ids.index(sid)
    assert mission.world.sector_priority[k] == 0
    # A completed scripted proposal may not overwrite or renew a team lease.
    thread = SimpleNamespace(is_alive=lambda: False)
    mission.hivemind._inflight = {
        "started": time.perf_counter(), "thread": thread, "rung": 0, "user": "",
        "box": {"out": json.dumps({"directives": [
            {"sector": sid, "priority": "low", "action": "explore"},
        ]})},
    }
    mission.world.t = 5
    mission.hivemind._collect(mission.world, mission._emit, mission.bus)
    assert mission.world.sector_priority[k] == 0
    assert mission.hivemind.applied[sid].at == 0
    team.disabled = True
    mission.world.t = 31
    step(team, mission)
    mission.hivemind._expire(mission.world)
    assert sid not in mission.hivemind.applied
    assert mission.world.sector_priority[k] == 1


@pytest.mark.parametrize("change", ["stale", "coverage", "hazard", "wrong_request", "evidence"])
def test_changed_or_stale_proposals_cannot_apply(team, mission, change):
    step(team, mission)
    row = reply(team, mission)
    if change == "stale":
        mission.world.t = 21
    elif change == "coverage":
        mission.world.in_comms[:] = False
    elif change == "hazard":
        mission.world.sector_hazard_known[:] = 0.9
    elif change == "wrong_request":
        row["request"] = "previous-mission"
    else:
        row["result"]["evidence"] = "unrelated-sector"
    team.worker.rows = [row]
    step(team, mission)
    assert team.stats["applied"] == 0


def test_pending_worker_never_blocks_tick_and_failure_preserves_swarm(team, mission):
    step(team, mission)
    start = time.perf_counter()
    for _ in range(20):
        mission.tick()
        step(team, mission)
    assert time.perf_counter() - start < 2
    assert len(team.worker.requests) == 1
    team.worker.rows = [{"kind": "fatal", "error": "runtime deadline exceeded"}]
    step(team, mission)
    assert team.disabled and team.pending is None
    assert mission.hivemind.providers[0].name == "scripted"
    before = mission.world.t
    mission.tick()
    assert mission.world.t > before


def test_stop_file_ignores_a_completed_result_and_report_is_written(team, mission):
    step(team, mission)
    team.worker.rows = [reply(team, mission), {"kind": "disabled", "error": "operator stop file"}]
    team.stop_file.touch()
    step(team, mission)
    assert team.disabled and not team.stats["applied"]
    team.close()
    assert team.worker.closed
    assert "not a claim of rescue uplift" in team.trace.path.with_suffix(".md").read_text()


def test_real_worker_crash_is_observable_and_cleans_up():
    client = WorkerClient("/definitely/missing/python", "local", TeamConfig.load(), Trace())
    client.thread.join(timeout=3)
    assert not client.thread.is_alive()
    assert client.poll()[0]["kind"] == "fatal"
    client.close()


def test_real_worker_ready_and_stop(tmp_path):
    stop = tmp_path / "worker.stop"
    client = WorkerClient(sys.executable, "local", TeamConfig.load(), Trace(), stop)
    try:
        deadline = time.monotonic() + 5
        rows = []
        while not rows and time.monotonic() < deadline:
            rows = client.poll()
            time.sleep(0.01)
        assert rows[0]["kind"] == "ready"
        stop.touch()
        client.thread.join(timeout=3)
        assert not client.thread.is_alive()
        assert client.poll()[0]["kind"] == "disabled"
    finally:
        client.close()


def test_hung_worker_is_killed_at_deadline(monkeypatch, mission):
    real_popen = subprocess.Popen
    processes = []

    def hung(*args, **kwargs):
        code = ('import sys,time; print(\'{"kind":"ready"}\', flush=True); '
                'sys.stdin.readline(); time.sleep(30)')
        p = real_popen([sys.executable, "-u", "-c", code], **kwargs)
        processes.append(p)
        return p

    monkeypatch.setattr("swarmmind.hivemind.team.client.subprocess.Popen", hung)
    config = TeamConfig.load().model_copy(update={"deadline_s": 0.15})
    client = WorkerClient(sys.executable, "local", config, Trace())
    try:
        client.submit(snapshot(mission))
        client.thread.join(timeout=4)
        assert not client.thread.is_alive()
        assert any(r["kind"] == "fatal" and "deadline" in r["error"] for r in client.poll())
        assert processes[0].poll() is not None
    finally:
        client.close()


def test_closed_mission_ignores_late_response(team, mission):
    step(team, mission)
    team.worker.rows = [reply(team, mission)]
    team.close()
    step(team, mission)
    assert not team.stats["applied"]


@pytest.mark.parametrize("args", [
    ["--headless", "--response-team"], ["--demo", "--headless"],
    ["--response-team", "heuristic", "--no-hivemind"],
    ["--demo", "--response-team", "--hivemind-allow-api"],
])
def test_cli_refuses_conflicting_modes(args):
    with pytest.raises(SystemExit) as exc:
        main(["run", *args])
    assert exc.value.code == 2

"""Single-writer integration: detached team results revalidated on the sim thread."""

import asyncio
import sys
import time
from collections import deque
from pathlib import Path

from ...contracts import topics
from ..providers.openai_api import OPENROUTER_URL
from .client import WorkerClient
from .config import ROOT, TeamConfig
from .observations import capture, observe_event
from .tools import MissionTools
from .trace import Trace, render_report
from .workflow import collaborate, heuristic_decide


class ResponseTeam:
    def __init__(self, world, bus, *, mode="workswarm", python=None, trace_path=None,
                 goal=None, console=False, config=None, scout=False):
        self.config = config or TeamConfig.load()
        if goal is not None:
            self.config = TeamConfig(**{**self.config.model_dump(), "goal": goal})
        if mode not in {"workswarm", "local", "heuristic"}:
            raise ValueError(f"unknown team runtime: {mode}")
        interpreter = Path(python) if python else (
            ROOT / "integrations/workswarm/.venv/bin/python" if mode == "workswarm"
            else Path(sys.executable))
        if mode != "heuristic" and not interpreter.is_file():
            raise ValueError(f"No team interpreter at {interpreter}; run scripts/setup_response_team.sh")
        self.mode = mode
        self.trace = Trace(trace_path, console=console)
        self.stop_file = Path(trace_path).with_suffix(".stop") if trace_path else None
        self.worker = None
        self.events = deque(maxlen=24)
        self.previous = {}
        self.pending = None
        self.leases = {}
        #: Which operator goal the team is currently planning under. Bumped by
        #: `set_goal`, so a spoken order re-plans once rather than every tick.
        self.goal_seq = 0
        self.next_t = 0.0
        self.seq = 0
        self.disabled = False
        self.closed = False
        self.stats = {"episodes": 0, "applied": 0, "rejected": 0, "fallbacks": 0}
        #: Optional fourth role (`scout.py`), the only one that reads pixels. Off unless
        #: asked for: it needs a key and the network, and the other three must keep
        #: working without it -- a scout that never answers costs an observation, not an
        #: episode.
        self.scout = None
        if scout:
            from .scout import VisionScout

            self.scout = VisionScout(world)
        self.trace.record("started", runtime=mode, model="rule-based" if mode == "heuristic"
                        else self.config.model, goal=self.config.goal)

        def event(ev):
            known = observe_event(ev)
            if known:
                if ev.get("kind") == "task_awarded" and "pos" in ev:
                    known["sector"] = world._sector_at(ev["pos"])
                self.events.append(known)
                # Association only: later events are not proof of causal rescue uplift.
                if known.get("sector") in self.leases:
                    self.trace.record("outcome", request=self.leases[known["sector"]][1],
                                    observed=known, attribution="subsequent task in same sector")

        bus.subscribe(topics.SWARM_EVENTS, event)
        if mode != "heuristic":
            self.worker = WorkerClient(interpreter, mode, self.config, self.trace, self.stop_file)

    def set_goal(self, goal: str, seq: int) -> bool:
        """Re-plan under the operator's spoken intent. True if this was a new goal.

        The operator does not issue the team's orders -- they set what the team is *for*,
        and the lead, logistics and safety roles decompose it themselves, exactly as they
        do the scenario's standing goal. Their own sector orders still go through
        `check_plan` and the feasibility filter; the only thing that changed is what they
        are planning toward.

        `TeamConfig` is frozen (it is a contract, not state), so this builds a new one.
        An in-flight episode is left alone -- cancelling it would throw away a round trip
        already paid for -- and the next one, scheduled immediately, picks up the goal.
        """
        if seq == self.goal_seq or self.closed:
            return False
        goal = str(goal).strip()[:500]
        if not goal:
            return False
        self.goal_seq = seq
        self.config = TeamConfig(**{**self.config.model_dump(), "goal": goal})
        if self.worker is not None:
            self.worker.config = self.config
        # Plan against the new goal at the next opportunity rather than up to
        # `interval_s` later: an operator who just spoke expects the team to react.
        if self.pending is None:
            self.next_t = 0.0
        self.trace.record("goal", goal=goal, source="operator", seq=seq)
        return True

    def _snapshot(self, world, ex, tracker, bus, snapshot_id):
        out = capture(world, ex, tracker, bus, snapshot_id)
        out["events"] = list(self.events)
        out["previous"] = dict(self.previous)
        # The scout's latest look, shared with the three reviewing roles as an ordinary
        # observation. Absent when it has not answered yet, which the prompt tolerates.
        if self.scout is not None and self.scout.last:
            out["scout"] = dict(self.scout.last)
        return out

    def step(self, world, ex, tracker, bus, hivemind, emit):
        if self.closed:
            return
        for sid in sorted(list(self.leases)):
            if world.t - self.leases[sid][0] >= hivemind.expiry:
                self.trace.record("expired", request=self.leases[sid][1], sector=sid)
                if self.previous.get("request") == self.leases[sid][1]:
                    self.previous["status"] = "expired"
                del self.leases[sid]
        hivemind.protected_sectors = frozenset(self.leases)
        if self.worker:
            # Stop/crash acknowledgments take precedence over queued completed work.
            rows = self.worker.poll()
            rows.sort(key=lambda row: row["kind"] not in {"disabled", "fatal"})
            for row in rows:
                kind = row["kind"]
                if kind == "ready":
                    self.trace.record("ready", runtime=self.mode)
                elif kind in {"disabled", "fatal"}:
                    self.disabled = True
                    self.pending = None
                    self.stats["fallbacks"] += 1
                    self.trace.record("disabled", reason=row.get("error"), mode="SCRIPTED FALLBACK")
                elif not self.disabled and self.pending and row.get("request") == self.pending["id"]:
                    if kind == "result":
                        self._accept(row["result"], world, ex, tracker, bus, hivemind, emit)
                    else:
                        self.stats["fallbacks"] += 1
                        self.trace.record("fallback", request=self.pending["id"], reason=row.get("error"))
                    self.pending = None
        if self.scout is not None:
            finding = self.scout.collect()
            if finding is not None:
                self.trace.record("observed", role="scout", note=finding["note"],
                                  sector=finding["sector"], model=finding["model"])
        if self.disabled or self.pending is not None or world.t < self.next_t:
            return
        # Look *before* the snapshot is taken, so the finding that lands is used by the
        # next episode rather than this one -- one look per episode, never a stall.
        if self.scout is not None and not self.scout.busy:
            from .tools import MissionTools

            preview = self._snapshot(world, ex, tracker, bus, "scout-preview")
            self.scout.start(world, MissionTools(
                preview, self.config, lambda *a, **kw: None).candidates())
        self.next_t = world.t + self.config.interval_s
        self.seq += 1
        self.pending = self._snapshot(world, ex, tracker, bus, f"response-{self.seq:04d}")
        self.pending_started = time.monotonic()
        self.stats["episodes"] += 1
        self.trace.record("snapshot", request=self.pending["id"], observation=self.pending)
        if self.worker:
            self.worker.submit(self.pending)
        else:
            request = self.pending["id"]

            def record(event, **data):
                self.trace.record(event, request=request, **data)

            result = asyncio.run(collaborate(self.pending, self.config, heuristic_decide, record))
            self._accept(result, world, ex, tracker, bus, hivemind, emit)
            self.pending = None

    def _accept(self, result, world, ex, tracker, bus, hivemind, emit):
        request = self.pending["id"]
        if not isinstance(result, dict):
            result = {"directive": None, "reason": "malformed worker result"}
        directive = result.get("directive")
        self.previous = {"request": request, "status": "withheld", "reason": result.get("reason", "")}
        if not directive:
            self.trace.record("withheld", request=request, reason=result.get("reason"))
            return
        current = self._snapshot(world, ex, tracker, bus, request)
        check = MissionTools(current, self.config, lambda *a, **kw: None).check_plan(directive)
        evidence_ok = isinstance(directive, dict) and result.get("evidence") == f"sector:{directive.get('sector')}"
        age = world.t - self.pending["sim_time"]
        if age < 0 or age > self.config.max_age_s or not check["ok"] or not evidence_ok:
            self.stats["rejected"] += 1
            self.trace.record("rejected", request=request, reason="stale or changed observations",
                            age=age, check=check, evidence_valid=evidence_ok)
            return
        hosted = self.config.model_url == OPENROUTER_URL
        source = "scripted" if self.mode == "heuristic" else "api" if hosted else "base-local"
        reasoning = (f"Response team reviewed {directive['action']} in {directive['sector']}; "
                     f"evidence {result['evidence']}; logistics and safety passed.")
        accepted = hivemind.apply_reviewed(
            world, {"reasoning": reasoning, "directives": [directive]},
            source, emit=emit, bus=bus,
        )
        if not accepted:
            self.stats["rejected"] += 1
            self.trace.record("rejected", request=request, reason="final feasibility filter")
            return
        self.leases[directive["sector"]] = (world.t, request)
        hivemind.protected_sectors = frozenset(self.leases)
        self.stats["applied"] += 1
        self.previous = {"request": request, "status": "applied", "directive": directive,
                         "at": world.t}
        self.trace.record("applied", request=request, directive=directive, at=world.t,
                        evidence=result.get("evidence"), runtime=self.mode, source=source,
                        latency_ms=round((time.monotonic() - self.pending_started) * 1000))

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.worker:
            self.worker.close()
        self.trace.record("finished", stats=self.stats, previous=self.previous)
        self.trace.close()
        if self.trace.path and not self.trace.error:
            render_report(self.trace.path)

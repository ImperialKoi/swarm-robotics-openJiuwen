"""Single-writer integration: detached team results revalidated on the sim thread."""

import asyncio
import sys
import time
from collections import deque
from pathlib import Path

from ...contracts import topics
from .client import WorkerClient
from .config import ROOT, TeamConfig
from .observations import capture, observe_event
from .tools import MissionTools
from .trace import Trace, render_report
from .workflow import collaborate, heuristic_decide


class ResponseTeam:
    def __init__(self, world, bus, *, mode="workswarm", python=None, trace_path=None,
                 goal=None, console=False, config=None):
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
        self.next_t = 0.0
        self.seq = 0
        self.disabled = False
        self.closed = False
        self.stats = {"episodes": 0, "applied": 0, "rejected": 0, "fallbacks": 0}
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

    def _snapshot(self, world, ex, tracker, bus, snapshot_id):
        out = capture(world, ex, tracker, bus, snapshot_id)
        out["events"] = list(self.events)
        out["previous"] = dict(self.previous)
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
        if self.disabled or self.pending is not None or world.t < self.next_t:
            return
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
        source = "scripted" if self.mode == "heuristic" else "base-local"
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

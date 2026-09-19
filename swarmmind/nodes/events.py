"""The event stream: /swarm/events.

Every narrative beat in the demo is an event -- a casualty found, a task orphaned and
reassigned, a robot destroyed, a phantom contact dismissed. The dashboard's timeline log
renders this verbatim, so a new event kind without a dashboard handler is invisible
(CLAUDE.md working rules).
"""

from __future__ import annotations

from typing import Any

from ..contracts import topics
from ..contracts.version import SCHEMA_VERSION


class EventLog:
    def __init__(self, bus=None, keep: int = 800) -> None:
        self.bus = bus
        self.keep = keep
        self.events: list[dict[str, Any]] = []
        self.counts: dict[str, int] = {}

    def emit(self, kind: str, text: str, *, t: float = 0.0, **kw: Any) -> dict:
        ev = {"schema": SCHEMA_VERSION, "t": round(t, 3), "kind": kind, "text": text, **kw}
        self._append(ev)
        return ev

    def drain_world(self, world) -> None:
        """Fold in events the simulation raised (deaths, reconnects, hazard ignition)."""
        for ev in world.drain_events():
            self._append({"schema": SCHEMA_VERSION, **ev})

    def _append(self, ev: dict) -> None:
        self.events.append(ev)
        self.counts[ev["kind"]] = self.counts.get(ev["kind"], 0) + 1
        if len(self.events) > self.keep:
            del self.events[: len(self.events) - self.keep]
        if self.bus is not None:
            self.bus.publish(topics.SWARM_EVENTS, ev)

    def recent(self, n: int = 8) -> list[dict]:
        return self.events[-n:]

    def of_kind(self, kind: str) -> list[dict]:
        return [e for e in self.events if e["kind"] == kind]

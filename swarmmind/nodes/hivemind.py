"""Tier 3: the hivemind.

**Nothing below this file may depend on it.** The auction allocates, executes and
self-heals with `/hivemind/directives` silent, and `tests/test_mission.py` runs with the
hivemind disabled precisely so that stays true (CLAUDE.md invariant #1). Directives are an
*optional modifier* on sector priority, never a requirement.

Two properties make the difference between a strategic layer and decoration:

* **Directives have mechanical effect.** A priority changes which tasks the auction
  announces first; an abandon closes a sector, penalises routing through it, and pulls
  robots out. A directive that only appears in a text feed is theatre.
* **Directives expire.** 30 seconds, unless renewed. A stale `abandon` quietly locking a
  third of the map is the likeliest silent failure of the whole system.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..contracts import topics
from ..contracts.version import SCHEMA_VERSION
from ..hivemind.filter import PRIORITY_CODE, FeasibilityFilter
from ..hivemind.prompt import SYSTEM, build

#: One definition, in the filter. Two copies drifting apart would mean the rule that
#: rejects no-op directives and the code that applies them disagree about what a
#: priority *is*.
PRIORITY_RANK = PRIORITY_CODE


@dataclass
class Applied:
    sector: str
    priority: str
    action: str
    at: float


class HivemindNode:
    def __init__(self, world, providers, tracker=None, *, period: float | None = None,
                 timeout: float = 8.0, expiry: float = 30.0) -> None:
        self.providers = list(providers)
        self.filter = FeasibilityFilter(world, tracker)
        self.period = period if period is not None else world.scn.rates.hivemind_period_s
        self.timeout = timeout
        self.expiry = expiry
        self._next_t = 0.0
        self.applied: dict[str, Applied] = {}
        # Optional peer-reviewed orders own their sectors until their ordinary expiry.
        self.protected_sectors: frozenset[str] = frozenset()
        #: Sectors the **operator** ordered, by the sim time of the order. A human at the
        #: microphone outranks every model in the building: while one of these is live,
        #: neither the Tier 3 cycle nor a reviewed team proposal may retask the sector.
        #: Kept separate from `protected_sectors` because the response team *overwrites*
        #: that set every step from its own leases (`team/controller.py`), and the two
        #: must not be able to clear each other. They expire on the same 30 s clock.
        self.operator_sectors: dict[str, float] = {}
        self.last_message: dict | None = None
        # `issued` counts directives that *changed* something. A model that re-sends an
        # identical directive every cycle is holding a position, not making 200 decisions,
        # and a scorecard reading "221 issued" would misrepresent that to a judge.
        self._inflight: dict | None = None
        self._latency_ms: list[int] = []
        self.stats = {"issued": 0, "renewed": 0, "rejected": 0, "offline": 0,
                      "calls": 0, "fell_through": 0, "timed_out": 0, "errored": 0,
                      "unparseable": 0}

    # ------------------------------------------------------------------ cycle

    def due(self, world) -> bool:
        return world.t >= self._next_t and self._inflight is None

    def step(self, world, executor, tracker, events, emit=None, bus=None) -> None:
        """Non-blocking. Starts a request, or collects one already running.

        **The tick loop is never held here.** An earlier version joined the worker thread
        with the rung timeout, which is fine headless and fatal on the demo: `DemoSim`
        runs at wall-clock speed, so a 3.5 s model call every 6 s would have frozen the
        dashboard for more than half the mission. Inference now spans however many ticks
        it needs and the result is picked up whenever it lands.
        """
        emit = emit or (lambda *a, **k: None)
        self._expire(world, emit)
        if self._inflight is not None:
            self._collect(world, emit, bus)
            return
        if world.t < self._next_t:
            return
        self._next_t = world.t + self.period
        user = build(world, executor, tracker, events,
                     [{"sector": a.sector, "priority": a.priority, "action": a.action}
                      for a in self.applied.values()])
        self._start(user, 0, emit)

    def _start(self, user: str, rung: int, emit) -> None:
        """Kick off one rung of the ladder on a worker thread."""
        if rung >= len(self.providers):
            self.stats["offline"] += 1
            emit("hivemind_offline",
                 "no usable directive this cycle; the previous one still stands")
            self._inflight = None
            return
        provider = self.providers[rung]
        box: dict = {}

        def run(p=provider, b=box):
            try:
                b["out"] = p.generate(SYSTEM, user, self.timeout)
            except Exception as exc:                            # noqa: BLE001
                b["err"] = exc

        th = threading.Thread(target=run, daemon=True)
        th.start()
        self.stats["calls"] += 1
        self._inflight = {"thread": th, "box": box, "rung": rung, "user": user,
                          "started": time.perf_counter()}

    def _collect(self, world, emit, bus) -> None:
        """Poll the in-flight request. Never blocks.

        A rung is spent by a timeout, an exception, unparseable output, *or* a message
        whose every directive the filter refused -- in that last case the model produced
        something, but nothing survivable, and the next rung deserves the cycle.

        A message with **no** directives is not a failure. "Nothing needs changing" is a
        legitimate answer from a strategic layer, and falling through on it would mean a
        correct model is silently replaced by the baseline whenever the swarm is fine.
        """
        f = self._inflight
        elapsed = time.perf_counter() - f["started"]
        if "out" not in f["box"]:
            if f["thread"].is_alive() and elapsed < self.timeout:
                return                                          # still thinking
            self.stats["timed_out" if f["thread"].is_alive() else "errored"] += 1
            self._start(f["user"], f["rung"] + 1, emit)
            return

        source = self.providers[f["rung"]].name
        latency_ms = int(elapsed * 1000)
        self._latency_ms.append(latency_ms)
        msg, _err = self.filter.parse(f["box"]["out"])
        if msg is None:
            self.stats["unparseable"] += 1
            self._start(f["user"], f["rung"] + 1, emit)
            return
        accepted, rejected, reasoning = self.filter.validate(msg, frozenset(self.applied))
        if not accepted and rejected:
            self.stats["fell_through"] += 1
            self._start(f["user"], f["rung"] + 1, emit)
            return

        self._inflight = None
        accepted = [d for d in accepted if d["sector"] not in self.held_sectors]
        for r in rejected:
            self.stats["rejected"] += 1
            emit("directive_rejected", f"rejected {r.directive.get('sector', '?')}: {r.rule}",
                 sector=r.directive.get("sector"))
        for d in accepted:
            self.stats["issued" if self._apply(world, d, emit) else "renewed"] += 1

        self.last_message = {
            "schema": SCHEMA_VERSION, "issued_at": round(world.t, 2), "source": source,
            "latency_ms": latency_ms, "reasoning": reasoning, "directives": accepted,
            "rejected": [{"directive": r.directive, "rule": r.rule} for r in rejected],
        }
        if accepted:
            emit("directive_issued", reasoning or "(no reasoning given)")
        if bus is not None:
            bus.publish(topics.HIVEMIND_DIRECTIVES, self.last_message)

    # ------------------------------------------------------------------ effect

    @property
    def held_sectors(self) -> frozenset[str]:
        """Sectors no fresh model directive may touch: team leases plus operator orders."""
        return self.protected_sectors | frozenset(self.operator_sectors)

    def apply_operator(self, world, msg, *, emit, bus):
        """Apply the operator's spoken order. Outranks Tier 3 and the response team.

        **Still filtered.** `hivemind/filter.py` runs on a human's words exactly as it
        runs on a model's, and that is deliberate on two counts: the feasibility rules it
        enforces are physical (a real sector id, never abandoning the last open sector),
        and a rejection spoken back to the operator is the system explaining itself out
        loud. Precedence is about outranking *other models*, not about outranking the
        safety floor -- CLAUDE.md invariant #2 is not negotiable from a microphone either.
        """
        accepted, rejected, reasoning = self.filter.validate(msg, frozenset(self.applied))
        for r in rejected:
            self.stats["rejected"] += 1
            emit("operator_order_rejected",
                 f"operator order refused for {r.directive.get('sector', '?')}: {r.rule}",
                 sector=r.directive.get("sector"))
        for d in accepted:
            self.stats["issued" if self._apply(world, d, emit) else "renewed"] += 1
            # Claim the sector *after* the filter passed it, so a refused order holds
            # nothing and the team stays free to work there.
            self.operator_sectors[d["sector"]] = world.t
        if accepted:
            self.last_message = {
                "schema": SCHEMA_VERSION, "issued_at": round(world.t, 2),
                "source": "operator", "latency_ms": 0, "reasoning": reasoning,
                "directives": accepted, "rejected": [],
            }
            emit("operator_order", reasoning or "operator order")
            if bus is not None:
                bus.publish(topics.HIVEMIND_DIRECTIVES, self.last_message)
        return accepted, [r.directive.get("sector", "?") for r in rejected]

    def apply_reviewed(self, world, msg, source, *, emit, bus):
        """Apply a complete team proposal on the simulation thread through the same filter."""
        # The operator outranks the team. Their sectors are dropped before validation
        # rather than failing the whole proposal: the team is meant to keep working the
        # rest of the map under the operator's goal, not to stall because one sector of
        # its plan was spoken for.
        held = frozenset(self.operator_sectors)
        if held:
            keep = [d for d in msg.get("directives", [])
                    if isinstance(d, dict) and d.get("sector") not in held]
            if len(keep) != len(msg.get("directives", [])):
                emit("directive_rejected", "team deferred to a live operator order")
                if not keep:
                    return []
                msg = {**msg, "directives": keep}
        accepted, rejected, reasoning = self.filter.validate(msg, frozenset(self.applied))
        for r in rejected:
            self.stats["rejected"] += 1
            emit("directive_rejected", f"team rejected: {r.rule}",
                 sector=r.directive.get("sector"))
        if rejected or not accepted:
            return []
        for d in accepted:
            self.stats["issued" if self._apply(world, d, emit) else "renewed"] += 1
        self.last_message = {
            "schema": SCHEMA_VERSION, "issued_at": round(world.t, 2), "source": source,
            "latency_ms": 0, "reasoning": reasoning, "directives": accepted, "rejected": [],
        }
        emit("directive_issued", reasoning or "response team reviewed this plan")
        bus.publish(topics.HIVEMIND_DIRECTIVES, self.last_message)
        return accepted

    def _apply(self, world, d: dict, emit) -> bool:
        """Returns True if this changed the swarm's orders, False if it renewed them."""
        prev = self.applied.get(d["sector"])
        changed = (prev is None or prev.priority != d["priority"]
                   or prev.action != d["action"])
        k = world.sector_ids.index(d["sector"])
        closing = d["action"] == "abandon" or d["priority"] == "abandon"
        world.sector_priority[k] = PRIORITY_RANK[d["priority"]]
        if closing and not world.sector_abandoned[k]:
            world.sector_abandoned[k] = True
            emit("sector_abandoned", f"{d['sector']} abandoned: {d.get('reason') or 'directive'}",
                 sector=d["sector"])
        self.applied[d["sector"]] = Applied(d["sector"], d["priority"], d["action"], world.t)
        return changed

    def _expire(self, world, emit=None) -> None:
        emit = emit or (lambda *a, **k: None)
        # An operator's claim expires on the same clock as everything else. A human who
        # abandons a sector and then walks away from the microphone must not leave a
        # third of the map locked for the rest of the mission -- that is the quiet
        # failure mode the 30 s expiry exists to prevent, and a person can cause it too.
        for sector, at in list(self.operator_sectors.items()):
            if world.t - at >= self.expiry:
                del self.operator_sectors[sector]
        for sector, a in list(self.applied.items()):
            if world.t - a.at < self.expiry:
                continue
            k = world.sector_ids.index(sector)
            world.sector_priority[k] = 1
            if world.sector_abandoned[k]:
                world.sector_abandoned[k] = False
                emit("sector_abandoned", f"{sector} reopened: directive expired",
                     sector=sector)
            del self.applied[sector]

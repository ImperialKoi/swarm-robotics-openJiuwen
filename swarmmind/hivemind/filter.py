"""The feasibility filter: nothing the hivemind says reaches the swarm unchecked.

Every rule rejects a single directive, never the whole message -- one bad sector should
not discard three good ones. Rejections are published and logged deliberately: the filter
catching a bad directive is a good demo moment, and hiding it would misrepresent how much
the model is trusted.

The rule that matters most is F7. A stale `abandon` locking a third of the map is the
likeliest quiet failure of the whole system, and the most tempting one to leave in.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from ..sim.robot import LANES

VALID_PRIORITY = {"high", "normal", "low", "abandon"}
VALID_ACTION = {"explore", "rescue", "hold", "abandon"}

#: Capability an action implicitly requires, if any.
ACTION_NEEDS = {"rescue": "gripper"}

MAX_DIRECTIVES = 4
MIN_OPEN_SECTORS = 3
#: A sector with a known casualty may only be abandoned if it is genuinely burning.
ABANDON_HAZARD_FLOOR = 0.6

#: Must stay in step with nodes.hivemind.PRIORITY_RANK -- both map a directive's
#: priority onto the sector_priority code the world stores.
PRIORITY_CODE = {"high": 0, "normal": 1, "low": 2, "abandon": 1}


@dataclass(frozen=True)
class Rejection:
    directive: dict[str, Any]
    rule: str


class FeasibilityFilter:
    def __init__(self, world, tracker=None) -> None:
        self.sector_ids = set(world.sector_ids)
        self.world = world
        self.tracker = tracker

    # ------------------------------------------------------------------ parse

    def parse(self, raw: str) -> tuple[dict | None, str | None]:
        """Text -> message dict, or (None, reason). Tolerates fenced code blocks."""
        text = (raw or "").strip()
        if text.startswith("```"):
            text = text.split("```")[1] if "```" in text[3:] else text[3:]
            text = text.removeprefix("json").strip()
        # Some models prepend prose; take the outermost JSON object if there is one.
        if not text.startswith("{"):
            i, j = text.find("{"), text.rfind("}")
            if i < 0 or j <= i:
                return None, "no JSON object in output"
            text = text[i : j + 1]
        try:
            msg = json.loads(text)
        except ValueError as exc:
            return None, f"malformed JSON: {exc}"
        if not isinstance(msg, dict):
            return None, "top-level value is not an object"
        return msg, None

    # ------------------------------------------------------------------ validate

    def validate(self, msg: dict, active: frozenset[str] = frozenset()
                 ) -> tuple[list[dict], list[Rejection], str]:
        """(accepted, rejected, reasoning).

        `active` is the set of sectors currently under a live directive. They are exempt
        from F10: re-sending an identical directive is a *renewal* that resets the 30 s
        clock, and rejecting it would let the sector silently reopen.
        """
        reasoning = str(msg.get("reasoning", "") or "")[:400]
        raw = msg.get("directives")
        if not isinstance(raw, list):
            return [], [Rejection({}, "directives is not a list")], reasoning

        accepted: list[dict] = []
        rejected: list[Rejection] = []

        for item in raw:
            if not isinstance(item, dict):
                rejected.append(Rejection({"raw": str(item)[:80]}, "directive is not an object"))
                continue
            bad = self._check(item, accepted, active)
            if bad:
                rejected.append(Rejection(item, bad))
            else:
                accepted.append({
                    "sector": item["sector"],
                    "priority": item["priority"],
                    "action": item["action"],
                    "reason": str(item.get("reason", ""))[:120] or None,
                })

        # F4: cap the count, dropping the tail rather than the whole message.
        if len(accepted) > MAX_DIRECTIVES:
            for extra in accepted[MAX_DIRECTIVES:]:
                rejected.append(Rejection(extra, f"over the {MAX_DIRECTIVES}-directive cap"))
            accepted = accepted[:MAX_DIRECTIVES]

        # F5: never let the swarm be left with nowhere to work.
        accepted, more = self._guard_open_sectors(accepted)
        rejected.extend(more)
        return accepted, rejected, reasoning

    def _check(self, d: dict, accepted: list[dict],
               active: frozenset[str] = frozenset()) -> str | None:
        sector = d.get("sector")
        if sector not in self.sector_ids:                                    # F2
            return f"unknown sector {sector!r}"
        if d.get("priority") not in VALID_PRIORITY:                          # F3
            return f"invalid priority {d.get('priority')!r}"
        if d.get("action") not in VALID_ACTION:                              # F3
            return f"invalid action {d.get('action')!r}"
        if any(a["sector"] == sector for a in accepted):
            return f"duplicate directive for {sector}"

        need = ACTION_NEEDS.get(d["action"])
        if need is not None:                                                 # F6
            alive = (self.world.status <= 1) & (self.world.actuator == LANES.index(need))
            if not alive.any():
                return f"no {need} robot is still active"

        # F9: `abandon` and a search priority are contradictory instructions -- the sector
        # would close to new work and simultaneously outrank every other sector for it.
        # A model asking for both has not decided, and guessing which half it meant is
        # how a filter starts inventing strategy of its own.
        closing = d["action"] == "abandon" or d["priority"] == "abandon"
        if closing and d["priority"] in ("high", "low"):
            return (
                f"{sector}: action 'abandon' contradicts priority "
                f"{d['priority']!r} -- abandon closes the sector"
            )

        # F10: a directive that asks for the state the sector is already in does nothing,
        # and there are only four slots. A 1.5B model spent an entire cycle sending four
        # `normal` directives -- all well-formed, all accepted, zero effect
        # (MEASUREMENTS.md M-27). Renewals are exempt; they reset the expiry clock.
        if sector not in active:
            k = self.world.sector_ids.index(sector)
            same_priority = int(self.world.sector_priority[k]) == PRIORITY_CODE.get(
                d["priority"], 1)
            if same_priority and bool(self.world.sector_abandoned[k]) == closing:
                return f"{sector} is already {d['priority']}; the directive changes nothing"

        if closing:
            k = self.world.sector_ids.index(sector)
            burning = float(self.world.sector_hazard_known[k]) >= ABANDON_HAZARD_FLOOR
            if self._holds_a_casualty(sector) and not burning:               # F7
                return (
                    f"{sector} holds a known casualty and is only "
                    f"{self.world.sector_hazard_known[k] * 100:.0f}% hazardous"
                )
            if self._holds_a_collection_point(sector):                       # F8
                return f"{sector} contains a collection point"
        return None

    def _holds_a_casualty(self, sector: str) -> bool:
        """Known to *the swarm* -- confirmed contacts, not the simulator's victim list.

        The distinction matters: the filter must be able to make this judgement from the
        same incomplete picture the rest of the stack has, or it is quietly vetoing
        directives using information nothing else in the system possesses.
        """
        if self.tracker is None:
            return False
        return any(
            self.world._sector_at(pos) == sector
            for pos in self.tracker.believed_positions()
        )

    def _holds_a_collection_point(self, sector: str) -> bool:
        zones = [self.world.scn.base, *self.world.scn.extraction_zones]
        return any(self.world._sector_at(z) == sector for z in zones)

    def _guard_open_sectors(self, accepted: list[dict]) -> tuple[list[dict], list[Rejection]]:
        """F5: at least MIN_OPEN_SECTORS must remain workable after this message.

        Abandoning the map is the failure mode that looks most like decisiveness.
        """
        already = int((~self.world.sector_abandoned).sum())
        closing = [d for d in accepted if d["action"] == "abandon" or d["priority"] == "abandon"]
        rejected: list[Rejection] = []
        for d in closing:
            k = self.world.sector_ids.index(d["sector"])
            if self.world.sector_abandoned[k]:
                continue
            if already - 1 < MIN_OPEN_SECTORS:
                rejected.append(Rejection(d, "would leave too few sectors open"))
                accepted = [a for a in accepted if a is not d]
            else:
                already -= 1
        return accepted, rejected

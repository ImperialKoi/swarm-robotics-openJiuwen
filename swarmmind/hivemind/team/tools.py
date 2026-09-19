"""Role-specific, read-only tools over one detached snapshot."""

import copy

from ..filter import ABANDON_HAZARD_FLOOR, MIN_OPEN_SECTORS


class MissionTools:
    def __init__(self, snapshot: dict, config, record):
        self.snapshot = copy.deepcopy(snapshot)
        self.config = config
        self.record = record
        self.sectors = {s["id"]: s for s in self.snapshot["sectors"]}

    def call(self, role: str, name: str, **arguments) -> dict:
        allowed = {
            "lead": {"read_incident", "read_outcome"},
            "logistics": {"read_capacity", "read_rescue_backlog"},
            "safety": {"check_plan", "read_outcome"},
        }
        if name not in allowed.get(role, set()):
            raise ValueError(f"{role} cannot call {name}")
        result = getattr(self, name)(**arguments)
        self.record("tool", role=role, tool=name, arguments=arguments, result=result)
        return result

    def read_incident(self) -> dict:
        return {"snapshot": self.snapshot["id"], "time": self.snapshot["sim_time"],
                "candidates": self.candidates()}

    def read_outcome(self) -> dict:
        return {"previous": self.snapshot.get("previous", {}),
                "events": self.snapshot.get("events", [])}

    def read_capacity(self, sector: str) -> dict:
        s = self.sectors[sector]
        return {"sector": sector, "in_contact_here": s["connected"],
                "lanes": copy.deepcopy(self.snapshot["lanes"]),
                "route_feasibility": "unknown"}

    def read_rescue_backlog(self, sector: str) -> dict:
        s = self.sectors[sector]
        return {"sector": sector, "announced_work": s["backlog"],
                "unresolved_contacts": s["contacts"],
                "note": "announcements include backups; not a count of casualties"}

    def candidates(self) -> list[dict]:
        ranked = []
        for sid, s in sorted(self.sectors.items()):
            if s["abandoned"]:
                continue
            rescue = s["backlog"].get("extract", 0) + s["backlog"].get("clear_debris", 0)
            if s["hazard"] >= self.config.hazard_ceiling and not s["collection"]:
                d = {"sector": sid, "priority": "abandon", "action": "abandon"}
                score = 20 + s["hazard"]
            elif rescue:
                d = {"sector": sid, "priority": "high", "action": "rescue"}
                score = 10 + rescue
            elif s["explored"] < self.config.explored_ceiling:
                d = {"sector": sid, "priority": "high", "action": "explore"}
                score = 1 - s["explored"] + (1 if s["connected"] else 0)
            else:
                continue
            ranked.append((score, sid, {"directive": d, "evidence": f"sector:{sid}",
                                       "observed": copy.deepcopy(s)}))
        ranked.sort(key=lambda row: (-row[0], row[1]))
        return [row[2] for row in ranked[:self.config.max_candidates]]

    def check_plan(self, directive: dict) -> dict:
        if not isinstance(directive, dict) or not all(
            isinstance(directive.get(k), str) for k in ("sector", "action", "priority")
        ):
            return {"ok": False, "issues": ["malformed directive"]}
        expected = "abandon" if directive["action"] == "abandon" else "high"
        if directive["priority"] != expected:
            return {"ok": False, "issues": ["unsupported team priority"]}
        sid = directive.get("sector")
        s = self.sectors.get(sid)
        issues = []
        if s is None:
            return {"ok": False, "issues": ["unknown sector"]}
        action = directive.get("action")
        if action == "abandon":
            if s["collection"]:
                issues.append("collection point must remain open")
            if s["hazard"] < self.config.hazard_ceiling:
                issues.append("insufficient observed hazard")
            if s["contacts"] + s["resolved_reports"] and s["hazard"] < ABANDON_HAZARD_FLOOR:
                issues.append("known contacts need protection")
            if sum(not x["abandoned"] for x in self.sectors.values()) <= MIN_OPEN_SECTORS:
                issues.append("too few open sectors")
        elif action in ("explore", "rescue"):
            if s["abandoned"] or s["hazard"] >= self.config.hazard_ceiling:
                issues.append("work target closed or threatened")
            if s["connected"] == 0:
                issues.append("no fresh in-contact unit in sector; coverage unverified")
            if action == "rescue":
                if not self.snapshot["lanes"]["gripper"]["connected"]:
                    issues.append("no carrier in contact")
                if not (s["backlog"].get("extract", 0) + s["backlog"].get("clear_debris", 0)):
                    issues.append("no observed rescue work")
        else:
            issues.append("unsupported team action")
        return {"ok": not issues, "issues": issues, "evidence": f"sector:{sid}"}

"""Bounded collaboration shared by SwarmFlow and the small offline runner.

The three roles own separate tools and messages. Logistics can change the lead's
plan; the lead must respond; safety has the final veto. No role can actuate a robot.
"""

from .tools import MissionTools

#: The scout (`scout.py`) is the fourth role and does not appear here: it does not vote
#: on a candidate, it contributes an observation. Its finding reaches the lead through
#: the snapshot, which is why it needs no turn in the loop below.
ROLES = {
    "lead": "Rescue lead: select a candidate by its numeric index to meet the goal. These are proposed NEW priorities, so the sector need not already be high priority. Prefer rescue work, otherwise explore with connected units. Use -1 only if no useful choice exists. Use peer feedback when revising.",
    "logistics": "Logistics specialist: challenge unsupported capacity/coverage. Choose only an index in feasible_choices, or -1 to reject. If the lead's choice is infeasible, choose a feasible alternative. Your note must describe the capacity or coverage finding, not repeat the lead's note.",
    "safety": "Safety reviewer: approve only the reviewed proposal with a passing check. Otherwise reject. Your note must state the check result and any issues; do not repeat a peer's note.",
}


async def collaborate(snapshot, config, decide, record):
    tools = MissionTools(snapshot, config, record)
    incident = tools.call("lead", "read_incident")
    outcome = tools.call("lead", "read_outcome")
    candidates = incident["candidates"]
    for i, candidate in enumerate(candidates):
        candidate["index"] = i
    record("decomposed", role="lead", goal=config.goal,
           tasks=(["read the map"] if snapshot.get("scout") else [])
           + ["prioritize rescue/search", "check logistics", "review safety", "verify outcome"])
    if not candidates:
        return {"directive": None, "reason": "No observed work needs a new priority."}
    context = {"goal": config.goal, "snapshot": snapshot["id"],
               "instruction": "Select one candidate index for a new order, or -1 to withhold.",
               "choices": candidates, "previous_outcome": outcome}
    # What the scout saw, when it has looked. It is advice from a teammate with a
    # different sense, not an instruction: the lead still chooses, logistics can still
    # overrule the choice on capacity, and safety can still veto the result.
    scout = snapshot.get("scout")
    if scout:
        context["scout"] = {
            "note": scout.get("note", ""), "suggests": scout.get("sector", ""),
            "weight": "A teammate's visual reading of the operator's map. It sees "
                      "coverage the numbers do not show, but it cannot check capacity "
                      "or safety. Treat it as evidence, not as a decision.",
        }
        record("observed", role="scout", note=scout.get("note", ""),
               sector=scout.get("sector", ""))

    async def turn(role, message):
        result = await decide(role, message, len(candidates))
        if (not isinstance(result, dict) or type(result.get("choice")) is not int
                or not -1 <= result["choice"] < len(candidates)
                or not isinstance(result.get("note"), str)):
            raise ValueError(f"malformed {role} result")
        record("message", role=role, choice=result["choice"], note=result["note"][:120])
        return result

    lead = await turn("lead", context)
    if lead["choice"] < 0:
        return {"directive": None, "reason": "Lead requested no change."}
    draft = candidates[lead["choice"]]["directive"]
    capacity = tools.call("logistics", "read_capacity", sector=draft["sector"])
    backlog = tools.call("logistics", "read_rescue_backlog", sector=draft["sector"])
    # Each alternative carries an explicit tool-derived check, not a model guess.
    feasible = [i for i, c in enumerate(candidates) if tools.check_plan(c["directive"])["ok"]]
    logistics = await turn("logistics", {
        **context, "proposal": lead, "capacity": capacity, "backlog": backlog,
        "feasible_choices": feasible,
        "instruction": "Choose a feasible choice; -1 if none. Explain a changed choice to the lead.",
    })
    if logistics["choice"] < 0 or logistics["choice"] not in feasible:
        record("veto", role="logistics", reason="No supported logistics plan")
        return {"directive": None, "reason": "Logistics withheld dispatch."}
    final = lead
    if logistics["choice"] != lead["choice"]:
        record("revision_requested", role="logistics", original=lead, alternative=logistics)
        final = await turn("lead", {**context, "original": lead, "peer": logistics,
                                    "allowed_choices": [logistics["choice"]],
                                    "instruction": "Accept the peer's supported alternative or choose -1."})
        if final["choice"] != logistics["choice"]:
            return {"directive": None, "reason": "Lead did not accept the supported revision."}
        record("revised", role="lead", before=draft,
               after=candidates[final["choice"]]["directive"])
    proposed = candidates[final["choice"]]
    check = tools.call("safety", "check_plan", directive=proposed["directive"])
    tools.call("safety", "read_outcome")
    safety = await turn("safety", {
        "goal": config.goal, "snapshot": snapshot["id"], "proposal": final,
        "directive": proposed["directive"], "logistics": logistics, "check": check,
        "instruction": "Choose the proposal's index only if check.ok is true, otherwise -1.",
    })
    if not check["ok"] or safety["choice"] != final["choice"]:
        record("veto", role="safety", reason=safety["note"])
        return {"directive": None, "reason": "Safety withheld dispatch."}
    record("reviewed", role="safety", directive=proposed["directive"],
           evidence=proposed["evidence"])
    return {"directive": proposed["directive"], "reason": final["note"][:120],
            "evidence": proposed["evidence"], "reviewed_sector": proposed["observed"]}


async def heuristic_decide(role, context, count):
    """Real rule-based roles, explicitly labeled as such; never an LLM impersonation."""
    if role == "lead":
        index = context.get("allowed_choices", [0])[0]
        note = "Prioritize observed work; accept the logistics alternative on revision."
    elif role == "logistics":
        feasible = context["feasible_choices"]
        original = context["proposal"]["choice"]
        index = original if original in feasible else (feasible[0] if feasible else -1)
        note = "Checked in-contact capacity and announced backlog; route feasibility remains unknown."
    else:
        index = context["proposal"]["choice"] if context["check"]["ok"] else -1
        note = "Independent preflight passed." if index >= 0 else "Preflight rejected the plan."
    return {"choice": index, "note": note}

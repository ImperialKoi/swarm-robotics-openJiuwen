# Rescue response team — observed run

This is an execution trace, not a claim of rescue uplift.

| Request | Stage | Role | Evidence / result |
|---|---|---|---|
|  | started |  | {"input_kind": "RECORDED OBSERVATION FIXTURE", "runtime": "native WorkSwarm Leader/Teammate", "scope": "Real local Qwen model and default budgets. No simulator dispatch in this standalone SDK check."} |
|  | native_team |  | {"implementation": "openjiuwen.agent_teams.TeamAgent", "leader": "lead", "teammates": ["logistics", "safety"], "dispatch": "scheduled"} |
|  | decomposed | lead | {"native_task": "rescue-plan", "tasks": ["logistics assessment", "independent safety verification", "leader acceptance"]} |
|  | message | lead | {"choice": 0, "note": "Search the disaster zone, prioritize the"} |
|  | native_task |  | {"task": "rescue-plan", "operation": "create_task", "assignee": "logistics", "reviewer": "safety", "directive": {"sector": "A1", "priority": "high", "action": "explore"}} |
|  | message | logistics | {"choice": 0, "note": "Search the disaster zone, prioritize the"} |
|  | native_task |  | {"task": "rescue-plan", "operation": "member_complete_task", "status": "in_review"} |
|  | message | safety | {"choice": 0, "note": "The deliverable meets the acceptance and"} |
|  | reviewed | safety | {"directive": {"sector": "A1", "priority": "high", "action": "explore"}} |
|  | native_task |  | {"task": "rescue-plan", "operation": "verify_task", "reviewer": "safety", "decision": "pass"} |
|  | message | lead | {"choice": 0, "note": "Task completed and safety verified."} |
|  | native_task |  | {"task": "rescue-plan", "operation": "submit_reviewed_plan", "status": "completed"} |

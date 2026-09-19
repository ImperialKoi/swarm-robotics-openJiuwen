"""Native openJiuwen TeamAgent leader, scheduled teammates and task review.

Imported only inside the isolated WorkSwarm worker. The native scheduler owns all
member execution, model requests and handoffs. Application tools wrap its task API;
a registered client adapts local JSON output without reimplementing team execution.
"""

import asyncio
import json
import tempfile
import time
from collections import Counter
from pathlib import Path
from urllib.parse import urlparse

from openjiuwen.agent_teams import (
    DeepAgentSpec,
    LeaderSpec,
    ModelPoolEntry,
    StorageSpec,
    TeamAgentSpec,
    TeamMemberSpec,
)
from openjiuwen.agent_teams.paths import configure_openjiuwen_home
from openjiuwen.agent_teams.rails.team_context import get_team_backend
from openjiuwen.agent_teams.schema.deep_agent_spec import (
    BuiltinToolSpec,
    ModelSpec,
    RailSpec,
    WorkspaceSpec,
    register_rail_provider,
    register_tool_provider,
)
from openjiuwen.agent_teams.tools.locales import make_translator
from openjiuwen.agent_teams.tools.team import CapabilityOverrides
from openjiuwen.agent_teams.tools.tool_task import (
    MemberCompleteTaskTool,
    ScheduledTaskCreateTool,
)
from openjiuwen.core.foundation.llm import ModelClientConfig, ModelRequestConfig, UserMessage
from openjiuwen.core.foundation.tool.base import Tool, ToolCard
from openjiuwen.core.runner import Runner
from openjiuwen.core.single_agent.prompts.builder import PromptSection
from openjiuwen.harness.prompts.prompt_attachment_manager import (
    PROMPT_ATTACHMENT_HISTORY_METADATA_KEY,
)
from openjiuwen.harness.rails.base import DeepAgentRail

from .native_model import PROVIDER
from .tools import MissionTools

ALLOWED = {
    "lead": {"dispatch_plan", "submit_reviewed_plan"},
    "logistics": {"recommend_plan"},
    "safety": {"verify_task"},
}

ROLE_PROMPTS = {
    "lead": (
        "You are the rescue LEADER. Your native team already exists with logistics and safety. "
        "For the user goal call dispatch_plan(choice, note), choosing a listed candidate index. "
        "This creates a native task assigned to logistics with safety as reviewer. "
        "The native scheduler handles execution. When the task is completed and verified, "
        "call submit_reviewed_plan(choice, note) with the logistics choice approved by safety. "
        "Accept supported peer revisions. Never invent a review. -1 means withhold."
    ),
    "logistics": (
        "You are the LOGISTICS teammate. Your assigned native task contains candidate choices "
        "and feasible_choices from observed capacity/hazards. Call recommend_plan(choice, note) "
        "with a feasible candidate; change the leader's draft if unsupported. Use -1 if none. "
        "The tool completes your native task and requests its safety review. "
        "Do not complete anyone else's task. Connected counts robots; route feasibility is unknown."
    ),
    "safety": (
        "You are the SAFETY teammate and native task reviewer. Review the logistics choice in "
        "the task record. Call verify_task(task_id=rescue-plan, decision=pass or fail, feedback): pass only when its preflight is ok. "
        "A fail vetoes dispatch. "
        "Never propose another choice or approve an unreviewed recommendation."
    ),
}


class NativeEpisode:
    def __init__(self, snapshot, config, record):
        if urlparse(config.model_url).hostname not in {"127.0.0.1", "localhost", "::1"}:
            raise ValueError("response team requires a loopback model endpoint")
        self.snapshot, self.config, self.record = snapshot, config, record
        self.tools = MissionTools(snapshot, config, record)
        self.candidates = self.tools.candidates()
        self.original = None
        self.recommendation = None
        self.approved = False
        self.task_id = "rescue-plan"
        self.result = None
        self.done = asyncio.Event()
        self.calls = 0
        self.tokens = 0
        self.deadline = time.monotonic() + config.deadline_s

    def candidate(self, choice):
        if type(choice) is not int or not 0 <= choice < len(self.candidates):
            raise ValueError("choice must be an observed candidate index")
        return self.candidates[choice]

    def brief(self):
        return {
            "snapshot": self.snapshot["id"], "goal": self.config.goal,
            "choices": [{"index": i, "directive": c["directive"],
                         "connected": c["observed"]["connected"], "backlog": c["observed"]["backlog"]}
                        for i, c in enumerate(self.candidates)],
            "feasible_choices": [i for i, c in enumerate(self.candidates)
                                 if self.tools.check_plan(c["directive"])["ok"]],
            "previous_outcome": self.snapshot.get("previous", {}),
            "event_counts": dict(sorted(Counter(e["kind"] for e in self.snapshot.get("events", [])).items())),
            "route_feasibility": "unknown",
        }

    def withhold(self, role, note):
        self.record("veto", role=role, reason=note)
        self.result = {"directive": None, "reason": f"{role} withheld: {note}"}
        self.done.set()

    async def dispatch(self, backend, choice, note):
        if self.original is not None:
            raise ValueError("one task per snapshot; await the native review")
        if choice == -1:
            self.withhold("lead", note)
            return {"withheld": True}
        proposal = self.candidate(choice)
        await backend.build_team("Rescue response", self.config.goal, "Rescue Lead", "Mission coordination",
                                 overrides=CapabilityOverrides(enable_task_verification=True))
        content = json.dumps({**self.brief(), "leader_choice": choice, "leader_note": note}, separators=(",", ":"))
        tool = ScheduledTaskCreateTool(backend, make_translator("en"))
        out = await tool.invoke({"tasks": [{
            "task_id": self.task_id, "title": "Choose a supported rescue/search priority",
            "content": content, "assignee": "logistics",
            "reviewer": [{"reviewer_id": "safety", "type": "verifier",
                          "instruction": "Use verify_task to approve or veto the logistics recommendation."}],
            "max_review_rounds": 1,
        }]})
        if not out.success:
            raise RuntimeError(out.error)
        self.original = choice
        self.record("decomposed", role="lead", native_task=self.task_id,
                    tasks=["logistics assessment", "independent safety verification", "leader acceptance"])
        self.record("message", role="lead", choice=choice, note=note)
        self.record("native_task", task=self.task_id, operation="create_task",
                    assignee="logistics", reviewer="safety", directive=proposal["directive"])
        return {"task": self.task_id, "status": "delegated", "wait_for": "native scheduler and reviewer"}

    async def recommend(self, backend, choice, note):
        if self.original is None or self.recommendation is not None:
            raise ValueError("no new assigned task to complete")
        if choice == -1:
            self.withhold("logistics", note)
            return {"withheld": True}
        proposed = self.candidate(choice)
        sid = proposed["directive"]["sector"]
        self.tools.call("logistics", "read_capacity", sector=sid)
        self.tools.call("logistics", "read_rescue_backlog", sector=sid)
        check = self.tools.check_plan(proposed["directive"])
        if not check["ok"]:
            return {"ok": False, "check": check, "feasible_choices": self.brief()["feasible_choices"]}
        payload = {"choice": choice, "note": note, "directive": proposed["directive"], "check": check}
        # The recommendation becomes native shared task content before task completion.
        update = await backend.task_manager.update_task(self.task_id, content=json.dumps(payload))
        if not update.ok:
            raise RuntimeError(update.reason)
        self.recommendation = choice
        out = await MemberCompleteTaskTool(backend.task_manager, make_translator("en")).invoke(
            {"task_id": self.task_id, "note": json.dumps(payload)})
        if not out.success:
            self.recommendation = None
            raise RuntimeError(out.error)
        self.record("message", role="logistics", choice=choice, note=note)
        if choice != self.original:
            self.record("revision_requested", role="logistics",
                        original={"choice": self.original}, alternative={"choice": choice, "note": note})
        self.record("native_task", task=self.task_id, operation="member_complete_task", status="in_review")
        return {"ok": True, "status": "in_review", **payload}

    async def submit(self, backend, choice, note):
        if choice == -1:
            self.withhold("lead", note)
            return {"withheld": True}
        if not self.approved or choice != self.recommendation:
            raise ValueError("both teammates must approve this exact choice")
        task = await backend.task_manager.get(self.task_id)
        if task is None or task.status != "completed":
            raise ValueError("native scheduler has not completed and verified the task")
        proposed = self.candidate(choice)
        if choice != self.original:
            self.record("revised", role="lead", before=self.candidate(self.original)["directive"],
                        after=proposed["directive"])
        self.record("message", role="lead", choice=choice, note=note)
        self.record("native_task", task=self.task_id, operation="submit_reviewed_plan", status=task.status)
        self.result = {"directive": proposed["directive"], "reason": note,
                       "evidence": proposed["evidence"], "native_task": self.task_id}
        self.done.set()
        return {"submitted": True, "requires": "simulator current-state validation"}


def register_episode(episode):
    """Bind identity and native backend through the SDK's public provider registries."""
    class MissionTool(Tool):
        def __init__(self, name, role, backend, method):
            self.role, self.backend, self.method = role, backend, method
            properties = {"note": {"type": "string", "maxLength": 24}}
            properties["choice"] = {"type": "integer", "minimum": -1,
                                    "maximum": len(episode.candidates) - 1}
            super().__init__(ToolCard(id=f"rescue.{name}.{role}", name=name, parallel_safe=False,
                                     description=f"{name}: {role} rescue decision. Give a short factual note.",
                                     input_params={"type": "object", "properties": properties,
                                                   "required": list(properties), "additionalProperties": False}))

        async def invoke(self, inputs, **kwargs):
            if episode.done.is_set():
                raise ValueError("episode is already closed")
            if self.backend.member_name != self.role:
                raise ValueError("native caller identity mismatch")
            note = str(inputs.get("note", ""))[:100]
            value = inputs.get("choice")
            result = await self.method(self.backend, value, note)
            return json.dumps(result)

        async def stream(self, inputs, **kwargs):
            yield await self.invoke(inputs, **kwargs)

    def build_tools(params, context):
        role = context.member_name
        backend = get_team_backend(context)
        if role == "lead":
            return [MissionTool("dispatch_plan", role, backend, episode.dispatch),
                    MissionTool("submit_reviewed_plan", role, backend, episode.submit)]
        if role == "logistics":
            return MissionTool("recommend_plan", role, backend, episode.recommend)
        if role == "safety":
            return []  # native scheduler injects its reviewer-scoped VerifyTaskTool
        raise ValueError(f"unexpected native member {role}")

    class MissionRail(DeepAgentRail):
        priority = 200

        def __init__(self, role, backend):
            super().__init__()
            self.role, self.backend = role, backend

        def init(self, agent):
            super().init(agent)
            self.builder = agent.system_prompt_builder
            self.attachments = agent.prompt_attachment_manager

        async def before_model_call(self, ctx):
            if (episode.done.is_set() or time.monotonic() > episode.deadline or episode.calls >= 6
                    or episode.tokens >= episode.config.token_budget
                    or (self.role == "logistics" and episode.recommendation is not None)):
                ctx.request_force_finish({"output": "Response episode stopped."})
                return
            # The fixed three-member roster and role identity are already in
            # our short system prompt. Exclude the SDK's repeated workspace and
            # roster announcements, preserving its actual task/inbox messages.
            await self.attachments.clear_section(session_id=ctx.context.session_id(), section="team.context")
            messages = ctx.context.get_messages()
            ctx.context.set_messages([m for m in messages if not (
                (getattr(m, "metadata", {}) or {}).get(PROMPT_ATTACHMENT_HISTORY_METADATA_KEY)
                and "<team-context>" in str(getattr(m, "content", "")))])
            if self.role == "logistics":
                task = await self.backend.task_manager.get(episode.task_id)
                if task is None:
                    raise ValueError("native assigned task is missing")
                ctx.context.set_messages([UserMessage(content=f"Native task {task.task_id} "
                    f"({task.status}). Task content: {task.content}")])
            if self.role == "lead" and episode.approved:
                task = await self.backend.task_manager.get(episode.task_id)
                if task is None or task.status != "completed":
                    ctx.request_force_finish({"output": "Awaiting native task settlement."})
                    return
                # Compact completed work from the native shared task record. The
                # SDK still owns scheduling, message delivery and the model call.
                tally = await self.backend.task_manager.get_review_tally(task)
                ctx.context.set_messages([UserMessage(content=json.dumps({
                    "task": task.task_id, "status": task.status, "choice": episode.recommendation,
                    "directive": episode.candidate(episode.recommendation)["directive"],
                    "review": {k: tally[k] for k in ("pass_count", "fail_count", "voted")},
                    "next": "submit_reviewed_plan with the reviewed choice"}, separators=(",", ":")))])
            prompt = ROLE_PROMPTS[self.role]
            if self.role == "lead" and episode.approved:
                prompt = ("You are rescue lead. The native rescue-plan task is completed and safety verified. "
                          "Call submit_reviewed_plan now with the reviewed choice in the task record and a short note. "
                          "Use the tool. Do not describe a next step in prose. -1 withholds if you disagree.")
            self.builder.add_section(PromptSection("identity", {"en": prompt}))
            allowed = ALLOWED[self.role]
            if self.role == "lead":
                allowed = {"dispatch_plan" if episode.original is None else "submit_reviewed_plan"}
            ctx.inputs.tools = [t for t in ctx.inputs.tools or [] if t.name in allowed]
            if self.role == "safety":
                for tool in ctx.inputs.tools:
                    tool.description = "Vote pass or fail on the assigned rescue-plan; give brief feedback."
                    tool.parameters = {"type": "object", "properties": {
                        "task_id": {"type": "string", "enum": [episode.task_id]},
                        "decision": {"type": "string", "enum": ["pass", "fail"]},
                        "feedback": {"type": "string", "maxLength": 24}},
                        "required": ["task_id", "decision", "feedback"], "additionalProperties": False}
            episode.calls += 1
            episode.record("native_model", role=self.role, call=episode.calls,
                           tools=[t.name for t in ctx.inputs.tools])

        async def after_model_call(self, ctx):
            if ctx.inputs.response is None:
                return
            if not getattr(ctx.inputs.response, "tool_calls", None):
                episode.record("native_no_tool", role=self.role,
                               content=str(getattr(ctx.inputs.response, "content", ""))[:900])
            usage = getattr(ctx.inputs.response, "usage_metadata", None)
            tokens = getattr(usage, "total_tokens", 0) or 0
            episode.tokens += tokens
            episode.record("model", role=self.role, model=episode.config.model, tokens=tokens,
                           usage_reported=bool(tokens), native=True)
            if time.monotonic() > episode.deadline or episode.tokens > episode.config.token_budget:
                episode.withhold("budget", "Native response budget exhausted")
                ctx.request_force_finish({"output": "Response budget exhausted."})

        async def before_tool_call(self, ctx):
            if episode.done.is_set() or time.monotonic() > episode.deadline:
                raise TimeoutError("response episode closed before tool execution")
            if ctx.inputs.tool_name not in ALLOWED[self.role]:
                raise ValueError("tool is outside this native member's mission permissions")
            if self.role == "safety":
                args = ctx.inputs.tool_args
                args = json.loads(args) if isinstance(args, str) else args
                if episode.recommendation is None or args.get("task_id") != episode.task_id:
                    raise ValueError("review must refer to the logistics task")
                if args.get("decision") not in {"pass", "fail"}:
                    raise ValueError("review must pass or fail")
                proposal = episode.candidate(episode.recommendation)
                check = episode.tools.call("safety", "check_plan", directive=proposal["directive"])
                if args["decision"] == "pass" and not check["ok"]:
                    raise ValueError("unsafe recommendation cannot pass")
                self.vote = args
            episode.record("native_tool", role=self.role, tool=ctx.inputs.tool_name,
                           arguments=ctx.inputs.tool_args)

        async def after_tool_call(self, ctx):
            if self.role == "safety" and ctx.inputs.tool_name == "verify_task":
                result = ctx.inputs.tool_result
                if not getattr(result, "success", False):
                    raise RuntimeError("native verify_task did not record a vote")
                episode.approved = self.vote["decision"] == "pass"
                episode.record("message", role="safety", choice=episode.recommendation if episode.approved else -1,
                               note=self.vote.get("feedback", ""))
                if episode.approved:
                    episode.record("reviewed", role="safety", directive=episode.candidate(episode.recommendation)["directive"])
                episode.record("native_task", task=episode.task_id, operation="verify_task",
                               reviewer="safety", decision=self.vote["decision"])
                if not episode.approved:
                    episode.withhold("safety", self.vote.get("feedback", "veto"))
            # End just this native member's turn after its handoff. The TeamAgent
            # scheduler continues the team round and wakes the next member itself.
            if ((self.role == "lead" and episode.original is not None)
                    or (self.role == "logistics" and episode.recommendation is not None)
                    or (self.role == "safety" and episode.approved) or episode.done.is_set()):
                ctx.request_force_finish({"output": "Native task handoff recorded."})

    register_tool_provider("swarmmind.mission", build_tools)
    register_rail_provider("swarmmind.mission_guard", lambda params, ctx: MissionRail(ctx.member_name, get_team_backend(ctx)))


async def run_native(snapshot, config, record):
    episode = NativeEpisode(snapshot, config, record)
    if not episode.candidates:
        return {"directive": None, "reason": "No observed candidate work."}
    register_episode(episode)
    with tempfile.TemporaryDirectory(prefix="swarmmind-native-") as folder:
        configure_openjiuwen_home(folder)
        root = Path(folder)
        model_spec = ModelSpec(
            model_client_config=ModelClientConfig(client_provider=PROVIDER, api_key="local-no-secret",
                api_base=config.model_url.removesuffix("/chat/completions"), timeout=config.call_timeout_s, max_retries=0,
                stream_first_chunk_timeout=config.call_timeout_s),
            model_request_config=ModelRequestConfig(model=config.model, temperature=0, max_tokens=config.max_tokens,
                parallel_tool_calls=False, tool_choice="required"),
        )
        agent_spec = DeepAgentSpec(
            model=model_spec,
            tools=[BuiltinToolSpec(type="swarmmind.mission")],
            rails=[RailSpec(type="swarmmind.mission_guard")],
            workspace=WorkspaceSpec(root_path=str(root / "workspace"), language="en"),
            cwd=str(root), project_root=str(root), language="en",
            enable_sys_operation=False, enable_security_rail=False,
            enable_tool_resilience_rail=False, enable_task_planning=False,
            enable_skill_discovery=False, enable_read_image_multimodal=False,
            prompt_mode="none", skills=[], mcps=[], subagents=[],
            max_iterations=6, completion_timeout=config.deadline_s,
        )
        spec = TeamAgentSpec(
            team_name=snapshot["id"], language="en", team_mode="predefined",
            spawn_mode="inprocess", dispatch_mode="scheduled", enable_task_verification=True,
            evolution_enabled=False, enable_fork=False, enable_swarmflow=False,
            storage=StorageSpec(type="sqlite", params={"connection_string": str(root / "team.db"),
                "read_pool_size": 2, "write_pool_size": 1, "write_cache_size_kb": 2048,
                "read_cache_size_kb": 1024, "mmap_size_mb": 0}), default_max_review_rounds=1,
            agents={"leader": agent_spec, "teammate": agent_spec.model_copy(deep=True)},
            leader=LeaderSpec(member_name="lead", display_name="Rescue Lead", prompt=ROLE_PROMPTS["lead"]),
            predefined_members=[TeamMemberSpec(member_name=role, display_name=role.title(),
                                                desc=f"Rescue {role} specialist", prompt=ROLE_PROMPTS[role])
                                for role in ("logistics", "safety")],
            model_pool=[ModelPoolEntry(
                model_name=config.model, api_provider=PROVIDER, api_key="local-no-secret",
                api_base_url=config.model_url.removesuffix("/chat/completions"),
                metadata={"client": {"timeout": config.call_timeout_s, "max_retries": 0,
                                     "stream_first_chunk_timeout": config.call_timeout_s},
                          "request": {"temperature": 0, "max_tokens": config.max_tokens,
                                      "context_window": 8192, "parallel_tool_calls": False, "tool_choice": "required"}},
            )],
        )
        record("native_team", implementation="openjiuwen.agent_teams.TeamAgent",
               leader="lead", teammates=["logistics", "safety"], dispatch="scheduled")
        try:
            async def drive():
                async for chunk in Runner.run_agent_team_streaming(
                        agent_team=spec, inputs={"query": json.dumps(episode.brief(), separators=(",", ":"))},
                        session=snapshot["id"]):
                    if getattr(chunk, "type", "") in {"answer", "team.completed", "team.idle"}:
                        record("native_stream", member=getattr(chunk, "source_member", None),
                               stream_type=getattr(chunk, "type", None))

            driver = asyncio.create_task(drive())
            waiter = asyncio.create_task(episode.done.wait())
            try:
                finished, _ = await asyncio.wait({driver, waiter}, timeout=config.deadline_s,
                                                 return_when=asyncio.FIRST_COMPLETED)
                if driver in finished:
                    await driver  # surface native engine errors, never relabel fallback as native
                if episode.result is None:
                    raise TimeoutError("native team did not submit a reviewed result before its deadline")
                return episode.result
            finally:
                driver.cancel()
                waiter.cancel()
                await asyncio.gather(driver, waiter, return_exceptions=True)
        finally:
            try:
                await asyncio.wait_for(Runner.delete_agent_team(team_name=spec.team_name,
                                       session_ids=[snapshot["id"]], force=True), timeout=2)
            except Exception as exc:
                record("native_cleanup_error", error=f"{type(exc).__name__}: {exc}"[:240])
            record("native_metrics", wall_s=round(time.monotonic() - episode.deadline + config.deadline_s, 3),
                   calls=episode.calls, tokens=episode.tokens, submitted=bool(episode.result))

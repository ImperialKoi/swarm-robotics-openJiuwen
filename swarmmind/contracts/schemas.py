"""Message schemas. FROZEN.

This module is the single definition of every payload that crosses the bus. No other
module may define these shapes, and the Godot client parses these and only these.
See CLAUDE.md invariant #4.

Field names and nesting match docs/TECHNICAL.md section 4 verbatim.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from .version import SCHEMA_VERSION

# --- enums, as Literal aliases so pydantic validates them at the boundary ----------

Actuator = Literal["none", "gripper", "scoop", "antenna"]
LaneLabel = Literal["scout", "carrier", "digger", "relay"]
RobotStatus = Literal["active", "out_of_comms", "failed", "destroyed"]
TaskKind = Literal["explore", "investigate", "clear_debris", "extract", "relay", "retreat"]
Requires = Literal["any", "scoop", "gripper", "antenna"]
Priority = Literal["high", "normal", "low", "abandon"]
DirectiveAction = Literal["explore", "rescue", "hold", "abandon"]
HivemindSource = Literal["tuned-local", "base-local", "api", "scripted"]
VictimState = Literal["hidden", "found", "cleared", "carried", "rescued"]

EventKind = Literal[
    "victim_found", "victim_cleared", "victim_rescued", "report_dismissed",
    "task_created", "task_awarded", "task_orphaned", "task_completed",
    "robot_destroyed", "robot_out_of_comms", "robot_reconnected", "robot_recharged",
    "hazard_ignited", "sector_abandoned", "operator_action",
    "directive_issued", "directive_rejected", "hivemind_offline",
    "operator_said", "operator_order", "operator_order_rejected",
    "mission_complete",
]

#: What a robot is doing, as a small int on the wire. **Not the same question as
#: `status`**, which says whether a robot is alive and in contact.
#:
#: Added because the dashboard could not tell a working robot from a broken one. A relay
#: parked on a post is motionless for the rest of the mission *by design* -- it is holding
#: the link a dozen robots report through -- and it rendered as an identical still box to
#: one wedged against a rock. Measured on seed 42: of ~80 stationary robots, ~32 were
#: doing exactly what they should and ~42 were genuinely stuck, and nothing on screen
#: separated them. `bridge.py` was handed the executor and never read it.
#:
#: main.gd has the same table as ACTIVITY_*, cross-checked by test_bridge_protocol.py.
ACTIVITY: dict[str, int] = {
    "idle": 0,          # no assignment and nowhere useful to be
    "drift": 1,         # no assignment, walking at unexplored ground
    "explore": 2,
    "investigate": 3,
    "dig": 4,
    "extract": 5,       # en route to a casualty
    "carry": 6,         # holding one
    "relay_post": 7,    # holding a post: motionless ON PURPOSE
    "relay_move": 8,    # bridging, reinforcing, or creeping toward dark ground
    "retreat": 9,
    "recover": 10,      # out of contact, closing on the nearest link
    "recharge": 11,
}

#: What the operator's action key would do to the unit they are driving, as a small int
#: on the wire -- the third field of a state frame's `manual` (nodes/bridge.py).
#:
#: **The simulator decides, the dashboard only draws it.** Whether a casualty is within
#: reach is a ground-truth question, so the dashboard cannot answer it and must not
#: guess: a key that offers "PICK UP" and then does nothing is worse than no hint.
#: `World.operator_offer` fills this in and `World.operator_act` performs exactly the
#: action it named.
#:
#: hud.gd has the same table as ACTION_LABELS, cross-checked by test_bridge_protocol.py.
OPERATOR_ACTION: dict[str, int] = {
    "none": 0,          # nothing in reach, or this unit cannot do anything here
    "pick_up": 1,       # a carrier over a casualty that is clear of debris
    "set_down": 2,      # a carrier holding one: put it down here
    "take_off": 3,      # a landed rotor
    "land": 4,          # a rotor in flight, over ground it can set down on
    # A carrier standing over a casualty the swarm has FOUND but not yet dug out. The
    # key cannot lift it, and saying so is the point: silently doing nothing is what
    # made the action key look broken. No ground truth escapes -- a HIDDEN casualty
    # the swarm has not detected still offers `none`, exactly as before.
    "dig_first": 5,     # a carrier over a casualty still under debris
}

#: actuator -> the display label the dashboard and the original spec use.
#: ``actuator`` is the source of truth for capability checks; ``type`` is cosmetic.
LANE_LABEL: dict[str, str] = {
    "none": "scout",
    "scoop": "digger",
    "gripper": "carrier",
    "antenna": "relay",
}

#: task kind -> the actuator required to execute it.
TASK_REQUIRES: dict[str, str] = {
    "explore": "any",
    #: Go look at a candidate the detector reported. Roughly half of these at long
    #: range turn out to be warm rubble, and finding that out costs a trip -- which is
    #: the honest consequence of perceiving rather than being told.
    "investigate": "any",
    "clear_debris": "scoop",
    "extract": "gripper",
    "relay": "antenna",
    "retreat": "any",
}


class _Msg(BaseModel):
    """Base for every on-bus payload."""

    model_config = ConfigDict(extra="forbid", frozen=True)
    schema_: str = Field(default=SCHEMA_VERSION, alias="schema")


# --- /swarm/state ------------------------------------------------------------------


class RobotState(_Msg):
    id: str
    type: LaneLabel
    actuator: Actuator
    pos: tuple[float, float]
    heading: float
    battery: float
    status: RobotStatus
    current_task: str | None = None
    carrying: str | None = None
    last_action_reason: str = ""


class DashboardFlight(BaseModel):
    """Additive dashboard `state.flight` payload; indices match the eight-column `r`.

    Actual simulator flight mode, never inferred from chassis or activity. Older
    recordings omit this object and render grounded. Bus RobotState is unchanged.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)
    airborne: list[bool]


class SectorState(_Msg):
    id: str
    explored_pct: float
    #: KNOWN hazard only -- hazard in cells the swarm has actually observed.
    #: Never the ground-truth hazard extent.
    hazard_level: float
    victims_found: int
    priority: Priority = "normal"
    abandoned: bool = False


class ReportState(_Msg):
    """A contact the swarm believes in. May be a phantom -- that is the honest part."""

    id: str
    pos: tuple[float, float]
    state: Literal["candidate", "confirmed", "resolved", "dismissed"]
    conf: float
    n_obs: int
    views: int


class SwarmState(_Msg):
    tick: int
    sim_time: float
    robots: list[RobotState]
    sectors: list[SectorState]
    victims_found_total: int
    victims_rescued_total: int
    victims_total: int
    robots_active: int
    robots_lost: int
    comms_component_size: int
    reports: list[ReportState] = Field(default_factory=list)


# --- /world/ground_truth  (dashboard only) ----------------------------------------


class VictimTruth(_Msg):
    id: str
    pos: tuple[float, float]
    state: VictimState
    buried: bool


class HazardTruth(_Msg):
    centre: tuple[float, float]
    radius: float
    active: bool


class GroundTruth(_Msg):
    victims_actual: list[VictimTruth]
    hazard_zone_actual: HazardTruth
    explored_mask_rle: str = ""


# --- /hivemind/directives ----------------------------------------------------------


class Directive(_Msg):
    sector: str
    priority: Priority
    action: DirectiveAction
    reason: str | None = None


class RejectedDirective(_Msg):
    directive: dict
    rule: str


class DirectiveMessage(_Msg):
    issued_at: float
    source: HivemindSource
    latency_ms: int
    reasoning: str
    directives: list[Directive]
    rejected: list[RejectedDirective] = Field(default_factory=list)


# --- auction -----------------------------------------------------------------------


class Task(_Msg):
    task_id: str
    kind: TaskKind
    sector: str
    target: tuple[float, float]
    requires: Requires
    priority_rank: int = 1
    created_at: float = 0.0
    value: float = 1.0


class Bid(_Msg):
    robot_id: str
    task_id: str
    bid_score: float
    capable: bool


class Award(_Msg):
    robot_id: str
    task_id: str
    awarded_at: float
    reason: str


class Heartbeat(_Msg):
    robot_id: str
    t: float


# --- /operator/voice ---------------------------------------------------------------

#: Where the exchange has got to, so the dashboard can show a live state rather than
#: text that appears from nowhere 2 s after the operator stopped talking.
VoicePhase = Literal["idle", "listening", "thinking", "speaking", "muted"]


class VoiceCaption(_Msg):
    """One frame of the operator's conversation, as the dashboard draws it.

    `said` and `reply` persist between turns on purpose -- a caption that cleared the
    moment playback ended would be gone before a judge standing behind the operator had
    read it. `sectors` is what the order actually moved, for the caption's second line.
    """

    t: float
    phase: VoicePhase
    said: str = ""
    reply: str = ""
    sectors: list[str] = []
    rejected: list[str] = []
    #: Round-trip for the spoken turn, end of speech to reply in hand.
    latency_ms: int = 0
    #: Microphone loudness, roughly 0..1, for the dashboard's recording meter. Live
    #: whenever the channel is armed, so the bars move before the key goes down.
    level: float = 0.0
    #: Whether the gate is actually open. Separate from `level` because a pause in a
    #: sentence is still recording -- the dot must not flicker grey mid-word.
    recording: bool = False


# --- /swarm/events -----------------------------------------------------------------


class Event(_Msg):
    t: float
    kind: EventKind
    text: str
    robot: str | None = None
    victim: str | None = None
    task: str | None = None
    sector: str | None = None
    pos: tuple[float, float] | None = None

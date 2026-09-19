"""The contract is FROZEN. These tests are the freeze."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from swarmmind.contracts import schemas as S
from swarmmind.contracts.version import SCHEMA_VERSION


def test_schema_version_pinned():
    assert SCHEMA_VERSION == "1.0"


def test_lane_label_covers_every_actuator():
    from swarmmind.sim.robot import LANES

    assert set(S.LANE_LABEL) == set(LANES)
    assert set(S.TASK_REQUIRES) == {
        "explore", "investigate", "clear_debris", "extract", "relay", "retreat"
    }


def test_swarm_state_round_trip():
    msg = S.SwarmState(
        tick=1234, sim_time=61.7,
        robots=[S.RobotState(
            id="r07", type="scout", actuator="none", pos=(12.4, 8.1), heading=1.02,
            battery=0.82, status="active", current_task="explore_A3",
            last_action_reason="Bid won: closest capable scout for A3, 3.1 m")],
        sectors=[S.SectorState(id="A3", explored_pct=0.6, hazard_level=0.0, victims_found=1)],
        victims_found_total=3, victims_rescued_total=1, victims_total=8,
        robots_active=63, robots_lost=1, comms_component_size=60,
    )
    blob = msg.model_dump_json(by_alias=True)
    assert '"schema":"1.0"' in blob.replace(" ", "")
    assert S.SwarmState.model_validate_json(blob) == msg


def test_directive_round_trip():
    msg = S.DirectiveMessage(
        issued_at=66.0, source="tuned-local", latency_ms=1840,
        reasoning="A3 has two unexplored pockets and no known hazard.",
        directives=[S.Directive(sector="A3", priority="high", action="explore")],
    )
    assert S.DirectiveMessage.model_validate_json(msg.model_dump_json(by_alias=True)) == msg


@pytest.mark.parametrize("bad", [
    {"sector": "A3", "priority": "urgent", "action": "explore"},   # priority not in enum
    {"sector": "A3", "priority": "high", "action": "nuke"},        # action not in enum
    {"sector": "A3", "priority": "high"},                          # missing action
    {"sector": "A3", "priority": "high", "action": "explore", "x": 1},  # extra field
])
def test_directive_rejects_malformed(bad):
    with pytest.raises(ValidationError):
        S.Directive.model_validate(bad)


def test_messages_are_frozen():
    d = S.Directive(sector="A3", priority="high", action="explore")
    with pytest.raises(ValidationError):
        d.sector = "B1"


def test_the_node_publishes_what_the_contract_declares():
    """Invariant #5: contracts/ is the only definition of a message shape.

    A node that publishes a dict which merely *resembles* the schema is how a frozen
    contract quietly stops being true -- the Godot parser reads the contract, the bridge
    ships the dict, and the mismatch surfaces on demo day.
    """
    from swarmmind.mission import Mission
    from swarmmind.sim.scenario import Scenario

    m = Mission(Scenario.load("test"), 42, scripted_hivemind=True)
    while m.hivemind.last_message is None and m.world.t < 60.0:
        m.tick()
    assert m.hivemind.last_message is not None, "the hivemind never published"
    S.DirectiveMessage.model_validate(m.hivemind.last_message)


def test_every_provider_name_is_a_declared_source():
    """`source` is a closed enum. A provider named off-contract fails validation late."""
    import typing

    from swarmmind.hivemind.providers.anthropic_api import AnthropicProvider
    from swarmmind.hivemind.providers.local_gguf import LocalGGUFProvider
    from swarmmind.hivemind.providers.scripted import ScriptedProvider

    allowed = set(typing.get_args(S.HivemindSource))
    names = {
        LocalGGUFProvider(tuned=True).name,
        LocalGGUFProvider(tuned=False).name,
        AnthropicProvider.name,
        ScriptedProvider.name,
    }
    assert names <= allowed, f"providers name themselves off-contract: {names - allowed}"

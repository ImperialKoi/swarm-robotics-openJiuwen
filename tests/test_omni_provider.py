"""The operator's voice channel, wired without paid requests or real credentials."""

import base64
import json
from unittest.mock import MagicMock

import pytest

from swarmmind.hivemind.providers.omni import (
    DEFAULT_OMNI_MODEL,
    DEFAULT_VOICE_MODEL,
    OPERATOR_SCHEMA,
    OmniProvider,
    OperatorTurn,
)
from swarmmind.hivemind.providers.openai_api import OPENROUTER_URL, TUNING

TURN = {"transcript": "Pull everyone out of D4.", "reply": "Clearing D4 now.",
        "goal": "Evacuate sector D4 and hold the line.",
        "directives": [{"sector": "D4", "priority": "abandon", "action": "abandon"}]}


@pytest.fixture
def key(monkeypatch):
    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")
    monkeypatch.delenv("OMNI_MODEL", raising=False)
    monkeypatch.delenv("OMNI_VOICE_MODEL", raising=False)


def fake_http(monkeypatch, content):
    payload = {"choices": [{"finish_reason": "stop", "message": {"content": json.dumps(content)}}]}
    http = MagicMock()
    http.return_value.__enter__.return_value.read.return_value = json.dumps(payload).encode()
    monkeypatch.setattr("urllib.request.urlopen", http)
    return http


def test_one_request_carries_speech_vision_and_text(key, monkeypatch):
    """The whole claim of this module: three modalities, one call."""
    http = fake_http(monkeypatch, TURN)
    turn = OmniProvider().understand(b"RIFFfake", b"\x89PNGfake", "61% explored", ["D4", "C3"])

    request = http.call_args.args[0]
    assert request.full_url == OPENROUTER_URL
    assert request.get_header("Authorization") == "Bearer test-key"
    body = json.loads(request.data)
    assert body["model"] == DEFAULT_OMNI_MODEL

    parts = body["messages"][1]["content"]
    kinds = [p["type"] for p in parts]
    assert kinds == ["text", "image_url", "input_audio"], "one call, all three modalities"
    assert "D4, C3" in parts[0]["text"], "the model is told which sector ids are real"
    assert parts[1]["image_url"]["url"].startswith("data:image/png;base64,")
    assert base64.b64decode(parts[2]["input_audio"]["data"]) == b"RIFFfake"

    assert turn == OperatorTurn(TURN["transcript"], TURN["reply"], TURN["goal"],
                                TURN["directives"])
    assert turn.is_order


def test_the_key_never_reaches_the_request_body(key, monkeypatch):
    http = fake_http(monkeypatch, TURN)
    OmniProvider().understand(b"RIFF", None, "bb", ["A1"])
    assert "test-key" not in http.call_args.args[0].data.decode()


def test_a_question_is_not_an_order(key, monkeypatch):
    fake_http(monkeypatch, {"transcript": "How is D4?", "reply": "Two contacts.",
                            "goal": "", "directives": []})
    turn = OmniProvider().understand(b"RIFF", None, "bb", ["D4"])
    assert not turn.is_order, "a question must not re-task the swarm"


def test_a_view_is_optional_so_a_blind_turn_still_answers(key, monkeypatch):
    """The render is 22 ms of work; a turn that misses one still has to answer."""
    http = fake_http(monkeypatch, TURN)
    OmniProvider().understand(b"RIFF", None, "bb", ["D4"])
    kinds = [p["type"] for p in json.loads(http.call_args.args[0].data)["messages"][1]["content"]]
    assert kinds == ["text", "input_audio"]


def test_reasoning_is_minimal_and_the_token_budget_covers_it(key, monkeypatch):
    """M-92: `none` is a 400 on this endpoint, and 400 tokens truncated the reply."""
    http = fake_http(monkeypatch, TURN)
    OmniProvider().understand(b"RIFF", None, "bb", ["D4"])
    body = json.loads(http.call_args.args[0].data)
    assert body["reasoning"] == {"effort": "minimal"}
    assert body["max_tokens"] >= 1500
    assert body["provider"] == {"require_parameters": True, "sort": "latency"}
    assert TUNING[DEFAULT_OMNI_MODEL]["reasoning"] == {"effort": "minimal"}


def test_a_refusal_or_truncation_is_an_error_not_a_silent_empty_turn(key, monkeypatch):
    payload = {"choices": [{"finish_reason": "length", "message": {"content": "{}"}}]}
    http = MagicMock()
    http.return_value.__enter__.return_value.read.return_value = json.dumps(payload).encode()
    monkeypatch.setattr("urllib.request.urlopen", http)
    with pytest.raises(ValueError, match="refused or did not finish"):
        OmniProvider().understand(b"RIFF", None, "bb", ["D4"])


def test_directives_match_the_frozen_contract_shape():
    """An operator order must need no translation before `hivemind/filter.py` sees it."""
    item = OPERATOR_SCHEMA["properties"]["directives"]["items"]
    assert item["required"] == ["sector", "priority", "action"]
    assert item["additionalProperties"] is False
    assert OPERATOR_SCHEMA["properties"]["directives"]["maxItems"] == 4
    assert OPERATOR_SCHEMA["additionalProperties"] is False


def test_speech_streams_because_the_endpoint_refuses_anything_else(key, monkeypatch):
    """M-92: audio output is a 400 without `stream: true`."""
    chunks = [b"\x01\x02" * 8, b"\x03\x04" * 8]
    lines = [b"data: " + json.dumps({"choices": [{"delta": {"audio": {
        "data": base64.b64encode(c).decode()}}}]}).encode() for c in chunks]
    lines += [b": keep-alive", b"data: [DONE]"]
    http = MagicMock()
    http.return_value.__enter__.return_value = iter(lines)
    monkeypatch.setattr("urllib.request.urlopen", http)

    out = list(OmniProvider().speak("North ridge is thin."))
    assert out == chunks, "PCM is yielded as it arrives, not buffered to the end"
    body = json.loads(http.call_args.args[0].data)
    assert body["stream"] is True
    assert body["model"] == DEFAULT_VOICE_MODEL
    assert body["audio"]["format"] == "pcm16"
    assert body["modalities"] == ["text", "audio"]


def test_no_key_is_a_clear_error_not_a_crash_mid_demo(monkeypatch):
    monkeypatch.delenv("OPENROUTER_API_KEY", raising=False)
    with pytest.raises(ValueError, match="OPENROUTER_API_KEY"):
        OmniProvider()

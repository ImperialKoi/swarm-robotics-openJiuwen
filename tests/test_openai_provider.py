"""Hosted model wiring without paid requests or real credentials."""

import json
from unittest.mock import MagicMock

import pytest

from swarmmind.hivemind.providers.openai_api import (
    OPENROUTER_URL,
    TUNING,
    OpenAIProvider,
    response_text,
    routing,
    tuning,
)
from swarmmind.hivemind.team.config import TeamConfig
from swarmmind.hivemind.team.model import LocalModel


def fake_http(monkeypatch, content):
    payload = {"choices": [{"finish_reason": "stop", "message": {"content": content}}],
               "usage": {"total_tokens": 40}}
    http = MagicMock()
    http.return_value.__enter__.return_value.read.return_value = json.dumps(payload).encode()
    monkeypatch.setattr("urllib.request.urlopen", http)
    return http


def test_openrouter_request_is_authenticated_bounded_and_structured(monkeypatch):
    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")
    monkeypatch.setenv("OPENROUTER_MODEL", "openai/gpt-5.6-terra")
    http = fake_http(monkeypatch, '{"reasoning":"Observed work","directives":[]}')
    result = OpenAIProvider().generate("system", "observations", 3.0)
    request = http.call_args.args[0]
    assert request.full_url == OPENROUTER_URL
    assert request.get_header("Authorization") == "Bearer test-key"
    assert http.call_args.kwargs["timeout"] == 3.0
    body = json.loads(request.data)
    assert body["model"] == "openai/gpt-5.6-terra"
    assert body["response_format"]["json_schema"]["strict"]
    assert body["provider"] == {"require_parameters": True, "sort": "latency"}
    assert "test-key" not in request.data.decode()
    assert json.loads(result)["directives"] == []


def test_only_openrouter_requests_carry_routing_preferences():
    assert routing(OPENROUTER_URL)["provider"] == {"require_parameters": True, "sort": "latency"}
    assert routing("http://127.0.0.1:8080/v1/chat/completions", "openai/gpt-5.6-terra") == {}


@pytest.mark.parametrize("model", sorted(TUNING))
def test_measured_models_send_only_fields_their_hosts_accept(monkeypatch, model):
    """M-88: `temperature` is a 404 on frontier hosts, and reasoning tokens need room."""
    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")
    http = fake_http(monkeypatch, '{"reasoning":"ok","directives":[]}')
    OpenAIProvider(model).generate("system", "observations", 3.0)
    body = json.loads(http.call_args.args[0].data)
    tune = tuning(model)
    assert ("temperature" in body) is tune["temperature"]
    assert body["max_tokens"] == tune["max_tokens"] >= 200
    assert body.get("reasoning") == tune["reasoning"] or tune["reasoning"] is None
    # A reasoning model must be given more room than the answer itself needs.
    assert tune["reasoning"] is None or body["max_tokens"] >= 400


@pytest.mark.parametrize("url,key,routed", [
    (OPENROUTER_URL, "test-key", True),
    ("http://127.0.0.1:8080/v1/chat/completions", "local-no-secret", False),
])
def test_team_client_routes_by_endpoint(monkeypatch, url, key, routed):
    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")
    http = fake_http(monkeypatch, '{"choice":0,"note":"observed"}')
    config = TeamConfig.load().model_copy(update={"model_url": url})
    assert LocalModel(config, lambda *a, **k: None).generate("lead", {}, 2)["choice"] == 0
    request = http.call_args.args[0]
    body = json.loads(request.data)
    assert request.full_url == url
    assert request.get_header("Authorization") == f"Bearer {key}"
    assert ("provider" in body) is routed
    # A loopback server keeps the request it always had, temperature included.
    assert ("temperature" in body) is not routed


@pytest.mark.parametrize("reason,refusal", [("length", None), ("stop", "refused")])
def test_incomplete_or_refused_output_is_rejected(reason, refusal):
    with pytest.raises(ValueError):
        response_text({"choices": [{"finish_reason": reason, "message": {
            "content": "{}", "refusal": refusal}}]})


def test_key_is_required_but_never_serialized_in_team_config(monkeypatch):
    monkeypatch.delenv("OPENROUTER_API_KEY", raising=False)
    config = TeamConfig.load()
    assert config.model_url == OPENROUTER_URL
    with pytest.raises(ValueError, match="OPENROUTER_API_KEY"):
        config.model_api_key()
    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")
    assert config.model_api_key() == "test-key"
    assert "test-key" not in config.model_dump_json()
    with pytest.raises(ValueError, match="endpoint"):
        config.model_copy(update={"model_url": "https://other.example/v1/chat/completions"}).model_api_key()
    local = config.model_copy(update={"model_url": "http://localhost:8080/v1/chat/completions"})
    assert local.model_api_key() == "local-no-secret"


def test_api_selection_skips_local_model_and_retains_scripted(monkeypatch):
    from swarmmind.hivemind.fallback import build_ladder

    monkeypatch.setenv("OPENROUTER_API_KEY", "test-key")
    local = MagicMock(side_effect=AssertionError("must not probe local server"))
    monkeypatch.setattr("swarmmind.hivemind.providers.local_gguf.LocalGGUFProvider", local)
    monkeypatch.setattr("swarmmind.hivemind.fallback.ScriptedProvider",
                        lambda *args: type("Scripted", (), {"name": "scripted"})())
    assert [p.name for p in build_ladder(None, allow_api=True)] == ["api", "scripted"]
    monkeypatch.delenv("OPENROUTER_API_KEY")
    assert [p.name for p in build_ladder(None, allow_api=True)] == ["scripted"]

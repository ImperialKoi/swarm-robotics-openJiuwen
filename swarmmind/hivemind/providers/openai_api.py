"""OpenRouter structured directives (OpenAI-compatible), using an environment-only API key."""

import json
import os
import urllib.request
from copy import deepcopy

from .anthropic_api import DIRECTIVE_SCHEMA

OPENROUTER_URL = "https://openrouter.ai/api/v1/chat/completions"
DEFAULT_MODEL = "openai/gpt-5.6-terra"
# OpenRouter may route a model to several upstream hosts. Only accept hosts that honour
# every request parameter -- a host without structured outputs answers this prompt in
# prose (M-88) -- and among those prefer the fastest.
OPENROUTER_ROUTING = {"provider": {"require_parameters": True, "sort": "latency"}}

#: Per-model request quirks, measured in MEASUREMENTS.md M-88, not guessed.
#: `temperature` is False for models whose structured-output hosts reject the field:
#: sending it anyway is a 404 under `require_parameters`. `max_tokens` has to cover
#: hidden reasoning tokens, which count against it -- 200 truncated every astra answer.
TUNING = {
    "openai/gpt-6-astra":   {"reasoning": {"effort": "minimal"}, "temperature": False, "max_tokens": 1200},
    "openai/gpt-5.6-sol":   {"reasoning": {"effort": "none"}, "temperature": False, "max_tokens": 400},
    "openai/gpt-5.6-terra": {"reasoning": {"effort": "none"}, "temperature": False, "max_tokens": 400},
    "openai/gpt-5.6-luna":  {"reasoning": {"effort": "none"}, "temperature": False, "max_tokens": 400},
    "openai/gpt-4.1-mini":  {"reasoning": None, "temperature": True, "max_tokens": 200},
}
#: An unmeasured model gets the cautious shape: no temperature, no reasoning control,
#: room to answer. That costs quality at worst, never a 404 or a truncated directive.
DEFAULT_TUNING = {"reasoning": None, "temperature": False, "max_tokens": 1200}


def api_key() -> str:
    key = os.environ.get("OPENROUTER_API_KEY", "").strip()
    if not key:
        raise ValueError("Set OPENROUTER_API_KEY to use the OpenRouter model")
    return key


def tuning(model: str | None) -> dict:
    return TUNING.get(model, DEFAULT_TUNING)


def routing(url: str, model: str | None = None) -> dict:
    """Extra request-body fields for `url`; loopback servers get none."""
    if url != OPENROUTER_URL:
        return {}
    reasoning = tuning(model)["reasoning"]
    return {**OPENROUTER_ROUTING, **({"reasoning": reasoning} if reasoning else {})}


def tune_body(body: dict, url: str, model: str) -> dict:
    """Shape one request body for `url`. A loopback server is left exactly as it was."""
    extra = routing(url, model)
    if not extra:
        return body
    if not tuning(model)["temperature"]:
        body.pop("temperature", None)
    body.update(extra)
    return body


def response_text(payload: dict) -> str:
    choice = payload["choices"][0]
    message = choice["message"]
    if choice.get("finish_reason") != "stop" or message.get("refusal"):
        raise ValueError("Model refused or did not complete its structured response")
    content = message.get("content")
    if not content:
        raise ValueError("Model returned no structured response")
    json.loads(content)
    return content


class OpenAIProvider:
    # Frozen wire contract already identifies hosted providers as "api".
    name = "api"

    def __init__(self, model: str | None = None):
        self._key = api_key()
        self.model = model or os.environ.get("OPENROUTER_MODEL") or DEFAULT_MODEL

    def generate(self, system: str, user: str, timeout: float) -> str:
        schema = deepcopy(DIRECTIVE_SCHEMA["schema"])
        properties = schema["properties"]["directives"]["items"]["properties"]
        for field in ("priority", "action"):
            properties[field]["type"] = "string"
        body = tune_body({
            "model": self.model, "temperature": 0.4,
            "max_tokens": tuning(self.model)["max_tokens"],
            "messages": [{"role": "system", "content": system},
                         {"role": "user", "content": user}],
            "response_format": {"type": "json_schema", "json_schema": {
                "name": "directives", "strict": True, "schema": schema}},
        }, OPENROUTER_URL, self.model)
        request = urllib.request.Request(OPENROUTER_URL, data=json.dumps(body).encode(), headers={
            "Content-Type": "application/json", "Authorization": f"Bearer {self._key}"})
        with urllib.request.urlopen(request, timeout=timeout) as response:  # noqa: S310
            return response_text(json.loads(response.read(65537)))

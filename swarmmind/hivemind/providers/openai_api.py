"""OpenRouter structured directives (OpenAI-compatible), using an environment-only API key."""

import json
import os
import urllib.request
from copy import deepcopy

from .anthropic_api import DIRECTIVE_SCHEMA

OPENROUTER_URL = "https://openrouter.ai/api/v1/chat/completions"
DEFAULT_MODEL = "openai/gpt-4.1-mini"
# OpenRouter may route a model to several upstream hosts. Only accept hosts that honour
# every request parameter, so the strict schema and max_tokens are never silently dropped.
OPENROUTER_ROUTING = {"provider": {"require_parameters": True}}


def api_key() -> str:
    key = os.environ.get("OPENROUTER_API_KEY", "").strip()
    if not key:
        raise ValueError("Set OPENROUTER_API_KEY to use the OpenRouter model")
    return key


def routing(url: str) -> dict:
    """Extra request-body fields for `url`; loopback servers get none."""
    return OPENROUTER_ROUTING if url == OPENROUTER_URL else {}


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
        body = {
            "model": self.model, "temperature": 0.4, "max_tokens": 200,
            "messages": [{"role": "system", "content": system},
                         {"role": "user", "content": user}],
            "response_format": {"type": "json_schema", "json_schema": {
                "name": "directives", "strict": True, "schema": schema}},
            **routing(OPENROUTER_URL),
        }
        request = urllib.request.Request(OPENROUTER_URL, data=json.dumps(body).encode(), headers={
            "Content-Type": "application/json", "Authorization": f"Bearer {self._key}"})
        with urllib.request.urlopen(request, timeout=timeout) as response:  # noqa: S310
            return response_text(json.loads(response.read(65537)))

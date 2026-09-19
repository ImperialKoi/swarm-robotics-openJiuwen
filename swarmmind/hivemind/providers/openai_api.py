"""OpenAI structured directives, using an environment-only API key."""

import json
import os
import urllib.request
from copy import deepcopy

from .anthropic_api import DIRECTIVE_SCHEMA

OPENAI_URL = "https://api.openai.com/v1/chat/completions"
DEFAULT_MODEL = "gpt-4.1-mini"


def api_key() -> str:
    key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not key:
        raise ValueError("Set OPENAI_API_KEY to use the OpenAI model")
    return key


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
        self.model = model or os.environ.get("OPENAI_MODEL") or DEFAULT_MODEL

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
        }
        request = urllib.request.Request(OPENAI_URL, data=json.dumps(body).encode(), headers={
            "Content-Type": "application/json", "Authorization": f"Bearer {self._key}"})
        with urllib.request.urlopen(request, timeout=timeout) as response:  # noqa: S310
            return response_text(json.loads(response.read(65537)))

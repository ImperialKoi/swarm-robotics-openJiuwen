"""Bounded OpenRouter/loopback JSON client, without general tools or simulator access."""

import json
import time
import urllib.request

from ..providers.openai_api import response_text, tune_body
from .workflow import ROLES


def choice_schema(count):
    return {"type": "object", "properties": {
        "choice": {"type": "integer", "minimum": -1, "maximum": count - 1},
        "note": {"type": "string", "maxLength": 100}},
        "required": ["choice", "note"], "additionalProperties": False}


class LocalModel:
    def __init__(self, config, record):
        self._key = config.model_api_key()
        self.config, self.record = config, record
        self.deadline = time.monotonic() + config.deadline_s
        self.tokens = 0
        self.calls = 0

    def generate(self, role, context, count):
        remaining = self.deadline - time.monotonic()
        if remaining <= 0 or self.calls >= 4 or self.tokens >= self.config.token_budget:
            raise TimeoutError("team budget exhausted")
        self.calls += 1
        body = {
            "model": self.config.model, "temperature": 0,
            "max_tokens": self.config.max_tokens,
            "messages": [
                {"role": "system", "content": ROLES[role] +
                 " Use only supplied observations and peer messages. 'connected' counts robots, not sectors. Output choice index and a short factual note as JSON. -1 means withhold."},
                {"role": "user", "content": json.dumps(context, separators=(",", ":"))}],
            "response_format": {"type": "json_schema", "json_schema": {
                "name": "role_choice", "strict": True, "schema": choice_schema(count)}},
        }
        tune_body(body, self.config.model_url, self.config.model)
        request = urllib.request.Request(self.config.model_url,
                                         data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json",
                                                  "Authorization": f"Bearer {self._key}"})
        started = time.monotonic()
        with urllib.request.urlopen(request, timeout=min(remaining, self.config.call_timeout_s)) as r:
            payload = json.loads(r.read(65537))
        usage = payload.get("usage", {})
        tokens = int(usage.get("total_tokens", 0))
        self.tokens += tokens
        self.record("model", role=role, model=self.config.model, tokens=tokens,
                    usage_reported="total_tokens" in usage,
                    latency_ms=round((time.monotonic() - started) * 1000))
        if time.monotonic() > self.deadline or self.tokens > self.config.token_budget:
            raise TimeoutError("team budget exhausted")
        return json.loads(response_text(payload))

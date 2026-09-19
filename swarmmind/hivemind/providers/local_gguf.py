"""Rung 1: `Qwen2.5-1.5B-Instruct` served locally by `llama-server`.

The primary path, and the reason the demo does not need network. Talks the OpenAI-
compatible endpoint over `urllib` rather than pulling in a client library -- one HTTP
POST does not justify a dependency, and stdlib cannot drift out of sync with `uv sync`.

Decoding is constrained by JSON schema (llama.cpp converts it to a GGBNF grammar), which
is what makes a 1.5B model usable here: it cannot emit prose where an enum belongs, so the
filter only ever has to judge *whether the plan is sane*, never whether it is parseable.

Start the server with `scripts/serve_hivemind.sh`, or directly:

    llama serve -m assets/models/hivemind-base.gguf --port 8080 --ctx-size 4096 --jinja

Note `llama serve`, not `llama-server`: llama.cpp now ships a unified `llama` binary
with subcommands. The HTTP surface is unchanged.
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request

from .anthropic_api import DIRECTIVE_SCHEMA


class LocalGGUFProvider:
    def __init__(self, url: str = "http://127.0.0.1:8080/v1/chat/completions",
                 model: str = "qwen2.5-1.5b-instruct", temperature: float = 0.4,
                 tuned: bool = False) -> None:
        # The contract distinguishes the fine-tuned model from the base one, and so must
        # the scorecard: "the hivemind worked" means nothing if we cannot say which model
        # was answering. The gate (D12) decides which GGUF actually ships.
        self.name = "tuned-local" if tuned else "base-local"
        self.url = url
        self.model = model
        self.temperature = temperature

    def generate(self, system: str, user: str, timeout: float) -> str:
        body = json.dumps({
            "model": self.model,
            "temperature": self.temperature,
            "max_tokens": 200,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user},
            ],
            "response_format": {
                "type": "json_schema",
                "json_schema": {"name": "directives", "strict": True,
                                "schema": DIRECTIVE_SCHEMA["schema"]},
            },
        }).encode()
        req = urllib.request.Request(
            self.url, data=body, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:   # noqa: S310
            payload = json.loads(resp.read())
        return payload["choices"][0]["message"]["content"]

    def available(self) -> bool:
        """Cheap liveness check so the ladder can skip a dead rung without burning 4 s."""
        root = self.url.split("/v1/")[0]
        try:
            with urllib.request.urlopen(f"{root}/health", timeout=0.5):  # noqa: S310
                return True
        except (urllib.error.URLError, OSError, ValueError):
            return False

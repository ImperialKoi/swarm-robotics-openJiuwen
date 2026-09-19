"""Rung 3: the Anthropic API.

Behind `--hivemind-allow-api` and never the default. The demo must not require network --
venue wifi is the one failure mode no amount of engineering here can fix -- so the local
model leads and this exists for comparison and as a backstop.

Notes that cost time if rediscovered later:
  * Assistant prefill returns 400 on Opus 5. Constrain the shape with structured outputs
    (`output_config.format`), never by prefilling an opening brace.
  * `budget_tokens` is gone; depth is controlled with `output_config.effort`.
  * ~1k in / ~200 out per call, ~60 calls a run: roughly $0.60 per 7-minute mission.
"""

from __future__ import annotations

import json

#: Length caps are not cosmetic -- they are the latency budget.
#:
#: Generation dominates the cycle: an unconstrained 1.5B model wrote a 700-character
#: essay plus four verbose `reason` fields and took **10.6 s** against a 4 s timeout, so
#: every cycle would have fallen through to the scripted rung. llama.cpp compiles
#: `maxLength` into the GBNF grammar, so the cap is enforced at decode time rather than
#: hoped for in the prompt. See MEASUREMENTS.md M-27.
DIRECTIVE_SCHEMA = {
    "type": "json_schema",
    "schema": {
        "type": "object",
        "properties": {
            "reasoning": {"type": "string", "maxLength": 120},
            "directives": {
                "type": "array",
                "maxItems": 4,
                "items": {
                    "type": "object",
                    "properties": {
                        "sector": {"type": "string", "maxLength": 4},
                        "priority": {"enum": ["high", "normal", "low", "abandon"]},
                        "action": {"enum": ["explore", "rescue", "hold", "abandon"]},
                        # No per-directive `reason`. It was roughly half the generated
                        # tokens -- four short essays restating the sector table -- and it
                        # appears nowhere except a fallback in one event string. The
                        # single `reasoning` field carries the plan; the contract keeps
                        # `reason` optional so a larger model may still supply it.
                    },
                    "required": ["sector", "priority", "action"],
                    "additionalProperties": False,
                },
            },
        },
        "required": ["reasoning", "directives"],
        "additionalProperties": False,
    },
}


class AnthropicProvider:
    name = "api"

    def __init__(self, model: str = "claude-opus-5", max_tokens: int = 1024) -> None:
        try:
            import anthropic
        except ImportError as exc:                              # pragma: no cover
            raise RuntimeError(
                "the anthropic package is not installed; run "
                "`uv sync --extra api` to enable the API rung"
            ) from exc
        self._client = anthropic.Anthropic()
        self.model = model
        self.max_tokens = max_tokens

    def generate(self, system: str, user: str, timeout: float) -> str:
        resp = self._client.with_options(timeout=timeout).messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=system,
            messages=[{"role": "user", "content": user}],
            thinking={"type": "adaptive"},
            output_config={"effort": "low", "format": DIRECTIVE_SCHEMA},
        )
        if getattr(resp, "stop_reason", None) == "refusal":     # pragma: no cover
            raise RuntimeError("model declined the request")
        parts = [b.text for b in resp.content if getattr(b, "type", "") == "text"]
        if not parts:                                           # pragma: no cover
            raise RuntimeError("no text content in response")
        return json.dumps(json.loads("".join(parts)))

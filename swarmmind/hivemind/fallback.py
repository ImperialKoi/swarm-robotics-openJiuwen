"""The provider ladder.

Four rungs, tried in order, each with a hard timeout. The point is that **the bottom rung
cannot fail**: `ScriptedProvider` needs no network, no GPU and no model file, so there is
always *something* on `/hivemind/directives`. A demo that dies because a venue blocked
port 8080 would be a self-inflicted wound.

    1. tuned-local  llama-server + the fine-tuned GGUF, schema-constrained  (primary)
    2. base-local   the same server running stock Qwen2.5-1.5B-Instruct     (fallback)
    3. api          Anthropic, only behind --hivemind-allow-api             (opt-in)
    4. scripted     phase-keyed heuristic directives                        (always works)

Rungs that cannot possibly work are dropped at construction rather than tried and timed
out, so a laptop with no llama-server does not spend 8 s per cycle discovering that.
"""

from __future__ import annotations

from .providers.scripted import ScriptedProvider


def build_ladder(world, tracker=None, *, allow_api: bool = False,
                 local_url: str | None = None, tuned: bool = False,
                 scripted_only: bool = False) -> list:
    if scripted_only:
        return [ScriptedProvider(world, tracker)]

    ladder: list = []
    from .providers.local_gguf import LocalGGUFProvider

    local = LocalGGUFProvider(tuned=tuned, **({"url": local_url} if local_url else {}))
    if local.available():
        ladder.append(local)

    if allow_api:
        try:
            from .providers.anthropic_api import AnthropicProvider

            ladder.append(AnthropicProvider())
        except Exception:                                          # noqa: BLE001
            pass  # no key, no package -- the ladder simply has one rung fewer

    ladder.append(ScriptedProvider(world, tracker))
    return ladder

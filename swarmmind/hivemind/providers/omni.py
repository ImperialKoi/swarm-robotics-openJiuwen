"""The operator's voice channel: one call that hears, sees and answers.

Tier 3 reasons over a text table (`hivemind/prompt.py`) and has never seen a pixel. This
is the other half of that: a human incident commander who talks to the swarm, and a model
that answers with the *operator's own view* in front of it.

**One request carries all three modalities**, and that is the point rather than a
convenience. Speech in, the fog-limited orbit render in, the blackboard summary in; a
transcript, a spoken reply, a team goal and swarm directives out. Splitting it into
transcribe-then-look-then-answer costs three round trips and loses the cross-modal part --
"the north ridge is thin" is a claim about the picture, made in response to a question
that was only ever spoken.

**The render the model sees is `fog=True, victims=False`** (`viz/render3d.py`). That is
what the operator sees with god-view off: terrain, buildings, and the swarm's own
revealed ground. It is not ground truth, so nothing here can leak a casualty position
into a directive and CLAUDE.md invariant #4 survives -- `tests/test_no_ground_truth_leak.py`
covers this module like every other guarded tree. Feeding it the *detector's* 48x48
frames was the obvious alternative and is measurably worse: those are colour-coded
rasters, not photographs, and a VLM asked to read one is colour-thresholding what
`perception/classical.py` already thresholds better (M-92).

**Directives come back in the frozen `DIRECTIVE_SCHEMA` shape** so an operator order is
the same object a hivemind order is, and passes the same `hivemind/filter.py`. The
operator outranks Tier 3 in `nodes/hivemind.py`, not here; this module only reports what
was said.

Measured on the demo map, M-92: fused call 1.82 s at $0.0029, speech out 0.58 s to first
audio. Two quirks that are not guesses:

* ``reasoning.effort`` **"minimal" is both faster and cheaper than "low"** -- 0 reasoning
  tokens against 261 -- and ``"none"`` is a hard 400 on this endpoint, where reasoning is
  mandatory. `max_tokens` must still cover hidden reasoning tokens; 400 truncated the
  reply mid-sentence.
* Audio output is a **400 without ``stream: true``**. Streaming is what the interaction
  wants anyway: the first PCM chunk lands in 0.58 s and plays while the rest generates.
"""

from __future__ import annotations

import base64
import json
import os
import urllib.request
from collections.abc import Iterator
from dataclasses import dataclass, field

from .openai_api import OPENROUTER_ROUTING, OPENROUTER_URL, api_key

#: Hears speech, reads images, answers in text. Chosen over the Qwen line because
#: OpenRouter serves no Qwen omni model -- every `qwen/*` there is text+image only
#: (M-92). `qwen/qwen3-vl-*` still carries the team's vision specialist.
DEFAULT_OMNI_MODEL = "google/gemini-3.5-flash"
#: Speech out. The only OpenRouter models with audio in the output modality are this and
#: its full-size sibling; neither accepts images, which is why it cannot be the fused call.
DEFAULT_VOICE_MODEL = "openai/gpt-audio-mini"
DEFAULT_VOICE = "alloy"

#: 24 kHz mono signed 16-bit, which is what `pcm16` streams and what playback assumes.
VOICE_RATE = 24_000

#: The fused turn. `directives` is deliberately the frozen DIRECTIVE_SCHEMA item shape so
#: an operator order needs no translation before `hivemind/filter.py` sees it.
#: `goal` is what the WorkSwarm team re-plans under -- the operator's intent in one line,
#: not a directive, because the team is supposed to decompose it rather than execute it.
OPERATOR_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["transcript", "reply", "goal", "directives"],
    "properties": {
        "transcript": {"type": "string", "maxLength": 240},
        "reply": {"type": "string", "maxLength": 240},
        "goal": {"type": "string", "maxLength": 200},
        "directives": {
            "type": "array",
            "maxItems": 4,
            "items": {
                "type": "object",
                "additionalProperties": False,
                "required": ["sector", "priority", "action"],
                "properties": {
                    "sector": {"type": "string", "maxLength": 4},
                    "priority": {"type": "string"},
                    "action": {"type": "string"},
                },
            },
        },
    },
}

SYSTEM = """You are the voice assistant to the operator of a disaster search-and-rescue \
robot swarm. You hear the operator, you see their screen, and you answer out loud.

The image is the operator's live view with god-view OFF: terrain, buildings, and coloured \
squares that are robots by lane. Casualty positions are NOT drawn and you do not know \
them -- speak only about what the picture and the numbers actually support.

Return four things:
- transcript: what the operator said, verbatim.
- reply: your spoken answer. Under 30 words, calm, radio-brief. No markdown, no lists.
- goal: one line restating the operator's intent for the response team to plan under. \
Keep their priority; do not invent one. Empty string if they asked a question rather \
than giving an order.
- directives: up to 4 sector orders, or an empty list for a question. `sector` MUST be \
one of the sector ids listed below and nothing else. `priority` is high, normal, low or \
abandon. `action` is explore, rescue, hold or abandon. Sending `normal` changes nothing, \
so do not spend a directive on it.

If the operator asks a question, answer it and return no directives. If they give an \
order, acknowledge it in one short sentence and return the directives that carry it out."""


@dataclass(frozen=True)
class OperatorTurn:
    """One spoken exchange. `transcript` and `reply` are what the dashboard captions."""

    transcript: str
    reply: str
    goal: str = ""
    directives: list[dict] = field(default_factory=list)

    @property
    def is_order(self) -> bool:
        return bool(self.directives) or bool(self.goal)


def _post(body: dict, key: str, timeout: float):
    request = urllib.request.Request(
        OPENROUTER_URL,
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"},
    )
    return urllib.request.urlopen(request, timeout=timeout)  # noqa: S310


class OmniProvider:
    """Speech + the operator's view + the blackboard, in one request."""

    #: The wire contract already names hosted providers "api"; this is a second hosted
    #: rung with its own label so the dashboard badge can tell them apart.
    name = "omni"

    def __init__(self, model: str | None = None, voice_model: str | None = None,
                 voice: str = DEFAULT_VOICE) -> None:
        self._key = api_key()
        self.model = model or os.environ.get("OMNI_MODEL") or DEFAULT_OMNI_MODEL
        self.voice_model = voice_model or os.environ.get("OMNI_VOICE_MODEL") or DEFAULT_VOICE_MODEL
        self.voice = voice

    # -- hear + see ---------------------------------------------------------------
    def understand(self, audio_wav: bytes, view_png: bytes | None, blackboard: str,
                   sectors: list[str], timeout: float = 20.0) -> OperatorTurn:
        """One fused call. `audio_wav` is 16-bit PCM WAV; `view_png` the fog-limited render."""
        content: list[dict] = [{
            "type": "text",
            "text": f"{blackboard}\n\nValid sector ids: {', '.join(sectors)}",
        }]
        # Order matters for the caption the model writes: it should read the numbers and
        # look at the map before it hears the question, the way the operator did.
        if view_png:
            url = "data:image/png;base64," + base64.b64encode(view_png).decode()
            content.append({"type": "image_url", "image_url": {"url": url}})
        content.append({"type": "input_audio", "input_audio": {
            "data": base64.b64encode(audio_wav).decode(), "format": "wav"}})

        body = {
            "model": self.model,
            # Hidden reasoning tokens count against this, and "none" is refused outright
            # on this endpoint -- both measured, M-92.
            "max_tokens": 1500,
            "reasoning": {"effort": "minimal"},
            "messages": [{"role": "system", "content": SYSTEM},
                         {"role": "user", "content": content}],
            "response_format": {"type": "json_schema", "json_schema": {
                "name": "operator_turn", "strict": True, "schema": OPERATOR_SCHEMA}},
            **OPENROUTER_ROUTING,
        }
        with _post(body, self._key, timeout) as response:
            payload = json.loads(response.read(1 << 20))
        choice = payload["choices"][0]
        if choice.get("finish_reason") != "stop" or choice["message"].get("refusal"):
            raise ValueError("omni model refused or did not finish the turn")
        data = json.loads(choice["message"]["content"])
        return OperatorTurn(
            transcript=str(data.get("transcript", "")).strip(),
            reply=str(data.get("reply", "")).strip(),
            goal=str(data.get("goal", "")).strip(),
            directives=list(data.get("directives") or []),
        )

    # -- speak --------------------------------------------------------------------
    def speak(self, text: str, timeout: float = 30.0) -> Iterator[bytes]:
        """Yield 24 kHz mono PCM16 chunks as they generate. First chunk lands in ~0.6 s.

        Streaming is not optional: the endpoint returns 400 for audio output without it.
        """
        body = {
            "model": self.voice_model,
            "stream": True,
            "modalities": ["text", "audio"],
            "audio": {"voice": self.voice, "format": "pcm16"},
            "max_tokens": 400,
            "messages": [{"role": "user", "content":
                          f"Say exactly this, in a calm radio-operator voice: {text}"}],
        }
        with _post(body, self._key, timeout) as response:
            for raw in response:
                line = raw.decode("utf-8", "replace").strip()
                if not line.startswith("data: "):
                    continue
                if line == "data: [DONE]":
                    break
                try:
                    event = json.loads(line[6:])
                except json.JSONDecodeError:      # keep-alive comment frames
                    continue
                for choice in event.get("choices", ()):
                    chunk = ((choice.get("delta") or {}).get("audio") or {}).get("data")
                    if chunk:
                        yield base64.b64decode(chunk)

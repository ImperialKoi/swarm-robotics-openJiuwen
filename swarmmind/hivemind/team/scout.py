"""The fourth role: the one that looks.

The rescue lead, the logistics specialist and the safety reviewer all reason over the
same table of numbers. Adding a fourth text agent to that would be four models arguing
about one spreadsheet. **The scout reads pixels instead**, and that is the whole reason
it earns a seat: it contributes something none of its peers can derive from the snapshot,
which is what makes this a team rather than an ensemble.

What it answers is deliberately narrow -- *which sector looks least covered, and is there
anything in the picture the numbers do not show* -- because a broad prompt gets a broad
answer and the lead needs a fact it can act on.

**It runs in this process, not in the isolated WorkSwarm venv.** The three reviewing
roles take a JSON snapshot through `client.py`; base64-ing a PNG through that protocol to
give one of them eyes would have meant changing the worker contract for every run,
including the ones with no scout. Instead the scout observes here, on the team's own 20 s
cadence, and its finding is attached to the snapshot as an ordinary observation the other
roles read. That is intermediate-result sharing, and it costs the worker protocol nothing.

**Never blocking**, for the reason everything else here is not: `DemoSim` runs at
wall-clock speed. `start()` kicks off a worker thread, `collect()` polls it, and a scout
that has not answered yet simply is not in the next snapshot.

Ground truth cannot reach it: it renders `MissionRenderer.views()[0]`, the appearance
raster masked by the fog, never `[1]`. `hivemind/` is a guarded tree and
`tests/test_no_ground_truth_leak.py` enforces that.
"""

from __future__ import annotations

import json
import threading
import time
import urllib.request

from ...viz import png
from ...viz.render import MissionRenderer
from ..providers.openai_api import OPENROUTER_ROUTING, OPENROUTER_URL, api_key

#: Vision-only and cheap: $0.00009 and 1.81 s for one look at the operator's view
#: (M-92), which is what makes a per-episode observation affordable at all.
DEFAULT_SCOUT_MODEL = "qwen/qwen3-vl-30b-a3b-instruct"

SCOUT_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["note", "sector"],
    "properties": {
        # One line, because it is pasted into three other models' prompts and every
        # token of it is paid for four times.
        "note": {"type": "string", "maxLength": 180},
        "sector": {"type": "string", "maxLength": 4},
    },
}

SYSTEM = """You are the visual scout for a disaster search-and-rescue swarm's response team.

You are shown the operator's live map with god-view off. Coloured squares are robots; \
dark ground is fog the swarm has not revealed; casualty positions are NOT drawn. Your \
three teammates see only a table of numbers, so report what the picture shows that a \
table would not -- where coverage is thin, where the swarm has bunched up, which listed \
sector looks worst served.

note: one sentence, under 25 words, about what you see. No preamble.
sector: the sector id from the candidate list that the picture most supports working \
next, or "" if the picture does not favour any of them."""


class VisionScout:
    """Looks at the operator's view and hands the team one observation."""

    role = "scout"

    def __init__(self, world, model: str | None = None, *, timeout: float = 12.0) -> None:
        self.model = model or DEFAULT_SCOUT_MODEL
        self.timeout = timeout
        self._key = api_key()
        self._renderer = MissionRenderer(world)
        self._inflight: dict | None = None
        self.last: dict | None = None
        self.stats = {"looks": 0, "answered": 0, "failed": 0}

    @property
    def busy(self) -> bool:
        return self._inflight is not None

    def start(self, world, candidates: list[dict]) -> bool:
        """Render now, on the simulation thread, and ask on a worker thread."""
        if self._inflight is not None or not candidates:
            return False
        sectors = [c["directive"]["sector"] for c in candidates]
        try:
            view = png.encode(self._renderer.views(world)[0])
        except Exception:                                        # noqa: BLE001
            return False
        self.stats["looks"] += 1
        box: dict = {}

        def run() -> None:
            try:
                box["out"] = self._ask(view, sectors)
            except Exception as exc:                             # noqa: BLE001
                box["err"] = exc

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        self._inflight = {"thread": thread, "box": box, "started": time.perf_counter()}
        return True

    def collect(self) -> dict | None:
        """Non-blocking. The finding when it lands, else None."""
        f = self._inflight
        if f is None:
            return None
        if "out" not in f["box"] and "err" not in f["box"]:
            if f["thread"].is_alive() and time.perf_counter() - f["started"] < self.timeout + 5:
                return None
            self._inflight = None
            self.stats["failed"] += 1
            return None
        self._inflight = None
        if "err" in f["box"]:
            self.stats["failed"] += 1
            return None
        self.stats["answered"] += 1
        self.last = {**f["box"]["out"], "by": self.role, "model": self.model}
        return self.last

    def _ask(self, view: bytes, sectors: list[str]) -> dict:
        import base64

        body = {
            "model": self.model,
            "max_tokens": 300,
            "messages": [
                {"role": "system", "content": SYSTEM},
                {"role": "user", "content": [
                    {"type": "text", "text": "Candidate sectors: " + ", ".join(sectors)},
                    {"type": "image_url", "image_url": {
                        "url": "data:image/png;base64," + base64.b64encode(view).decode()}},
                ]},
            ],
            "response_format": {"type": "json_schema", "json_schema": {
                "name": "scout_note", "strict": True, "schema": SCOUT_SCHEMA}},
            **OPENROUTER_ROUTING,
        }
        request = urllib.request.Request(
            OPENROUTER_URL, data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {self._key}"})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:  # noqa: S310
            payload = json.loads(response.read(1 << 18))
        data = json.loads(payload["choices"][0]["message"]["content"])
        sector = str(data.get("sector", "")).strip()
        return {"note": str(data.get("note", "")).strip()[:180],
                # A sector the scout invented helps nobody; drop it rather than let the
                # lead choose an index that does not exist.
                "sector": sector if sector in sectors else ""}

"""The operator console: one spoken turn, start to finish.

Sequence, none of which blocks the tick loop:

    U released -> render the operator's view -> fused call on a worker thread
    -> captions published -> directives applied -> speech streamed back

**Push to talk.** The microphone records only while the operator holds U on the
dashboard; the key arrives over the bridge and is handed here as `on_mic_key`. A demo
floor is loud, and an open microphone there hears the next table, the operator talking to
a judge, and its own reply coming back out of the speaker. See `voice/mic.py`.

**Asynchronous for the same reason `nodes/hivemind.py` is.** `DemoSim` runs at
wall-clock speed, so the 1.87 s fused call (M-92) would freeze the dashboard outright if
it were joined. One tick starts the request; a later tick collects it. The only work done
on the simulation thread is the render and the directive application, and both are
bounded below.

**What the model is shown is the swarm view, never the god view.**
`MissionRenderer.views()` returns `(swarm, god)` and this module takes `[0]` and only
`[0]`. The god view carries `COLOR_VICTIM_TRUE` dots drawn straight from `world.victims`;
the swarm view is the appearance raster masked by the fog -- **the same pixels
`perception/` already feeds the detector**, over ground the swarm has actually revealed.
So the assistant is a second, better consumer of sensor data the swarm already had, not a
back door to the simulator's casualty list. `tests/test_no_ground_truth_leak.py` guards
this tree along with `nodes`, `control`, `hivemind` and `training`.

**The operator outranks the models, not the filter.** Directives go to
`HivemindNode.apply_operator`, which runs `hivemind/filter.py` over a human's words
exactly as it does over a model's, and speaks any rejection back. See that method for why.

With no microphone, no key, or `sounddevice` absent, `step()` returns immediately and the
mission is untouched -- invariant #1, applied to a human.
"""

from __future__ import annotations

import threading
import time

from ..contracts import topics
from ..hivemind.prompt import build as build_blackboard
from ..viz import png
from ..viz.render import MissionRenderer
from .mic import Microphone
from .speaker import Speaker

#: Phases the dashboard draws. Kept in step with `contracts.schemas.VoicePhase`.
IDLE, LISTENING, THINKING, SPEAKING, MUTED = (
    "idle", "listening", "thinking", "speaking", "muted")

#: How long a caption stays on screen after the reply finishes. Long enough for somebody
#: standing behind the operator to read it; short enough not to sit there all mission.
CAPTION_HOLD_S = 12.0


class OperatorConsole:
    """Hears the operator, answers out loud, and re-tasks the swarm."""

    def __init__(self, world, provider, *, mic=None, speaker=None,
                 timeout: float = 20.0) -> None:
        self.provider = provider
        self.timeout = timeout
        self.mic = mic if mic is not None else Microphone()
        self.speaker = speaker if speaker is not None else Speaker()
        self._renderer = MissionRenderer(world)
        self._inflight: dict | None = None
        self._view: bytes | None = None
        self._phase = MUTED
        self._said = ""
        self._reply = ""
        self._sectors: list[str] = []
        self._rejected: list[str] = []
        self._latency_ms = 0
        self._caption_until = 0.0
        self._published: tuple | None = None
        #: The operator's standing intent, handed to the response team so its lead,
        #: logistics and safety roles re-plan under it. None until they say something.
        self.goal: str | None = None
        self.goal_seq = 0
        self.stats = {"turns": 0, "orders": 0, "questions": 0,
                      "rejected": 0, "errored": 0, "barge_ins": 0}

    # ------------------------------------------------------------------ lifecycle

    def start(self) -> bool:
        """Open the microphone. False is not fatal -- the mission runs regardless."""
        if not self.mic.start():
            self._phase = MUTED
            #: Why the channel is dead, shown on the dashboard rather than only logged.
            self._reply = self.mic.error or "no microphone available"
            return False
        self._phase = IDLE
        return True

    def close(self) -> None:
        self.mic.close()
        self.speaker.close()

    # ------------------------------------------------------------------ the key

    def on_mic_key(self, down: bool) -> None:
        """U, from the dashboard through `nodes/bridge.py`. Never blocks.

        Pressing also cuts the assistant off mid-sentence: reaching for the key *is* the
        interruption, so there is nothing else to press to interrupt.
        """
        if self._phase == MUTED:
            return
        if down:
            if self.speaker.busy:
                self.speaker.stop()
                self.stats["barge_ins"] += 1
            self.mic.press()
        else:
            self.mic.release()

    @property
    def phase(self) -> str:
        return self._phase

    # ------------------------------------------------------------------ cycle

    def step(self, world, executor, tracker, events, hivemind, *, emit=None, bus=None) -> None:
        """Non-blocking. Call once per tick from the mission."""
        emit = emit or (lambda *a, **k: None)
        if self._phase == MUTED:
            # **Still publishes.** A console whose microphone never opened used to
            # return here before `_publish`, so the dashboard was told nothing at all
            # and drew no badge -- indistinguishable from a healthy channel nobody has
            # spoken into yet. The operator needs to see MIC OFF, not an absence.
            self._publish(world, bus)
            return

        # A press that stops being renewed is a release: a dashboard that crashed or a
        # socket that dropped must not leave the microphone open.
        self.mic.expire()
        # Barge-in. Under push-to-talk `on_mic_key` has already done this on the press;
        # this covers the energy-gated mode, where talking over the reply is the only
        # way to interrupt it.
        if self.mic.hot and self.speaker.busy:
            self.speaker.stop()
            self.stats["barge_ins"] += 1

        if self._inflight is not None:
            self._collect(world, hivemind, emit, bus)
        else:
            self._listen(world, executor, tracker, events, emit)

        self._publish(world, bus)

    def _listen(self, world, executor, tracker, events, emit) -> None:
        """Poll the microphone and, on a complete utterance, start the fused call."""
        if self.mic.hot:
            self._phase = LISTENING
            # Render while they are still speaking: ~31 ms on the demo map, once per
            # utterance, and it buys the worker a view that is current when it starts.
            if self._view is None:
                self._view = self._render(world)
            return

        audio = self.mic.take()
        if audio is None:
            if self._phase == LISTENING:
                self._phase = SPEAKING if self.speaker.busy else IDLE
            elif self._phase == SPEAKING and not self.speaker.busy:
                self._phase = IDLE
            return

        view = self._view or self._render(world)
        self._view = None
        blackboard = build_blackboard(world, executor, tracker, events, [])
        sectors = list(world.sector_ids)
        self._phase = THINKING
        self.stats["turns"] += 1
        box: dict = {}

        def run() -> None:
            try:
                box["out"] = self.provider.understand(
                    audio, view, blackboard, sectors, self.timeout)
            except Exception as exc:                              # noqa: BLE001
                box["err"] = exc

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        self._inflight = {"thread": thread, "box": box, "started": time.perf_counter()}

    def _collect(self, world, hivemind, emit, bus) -> None:
        """Poll the in-flight turn. Never blocks."""
        f = self._inflight
        elapsed = time.perf_counter() - f["started"]
        if "out" not in f["box"] and "err" not in f["box"]:
            if f["thread"].is_alive() and elapsed < self.timeout + 5.0:
                return
            self._fail(world, "the link to the assistant timed out", emit)
            return
        if "err" in f["box"]:
            self._fail(world, f"assistant error: {type(f['box']['err']).__name__}", emit)
            return

        self._inflight = None
        turn = f["box"]["out"]
        self._latency_ms = int(elapsed * 1000)
        self._said = turn.transcript
        self._reply = turn.reply
        self._sectors, self._rejected = [], []
        emit("operator_said", turn.transcript or "(unclear)")

        if turn.directives:
            accepted, refused = hivemind.apply_operator(
                world, {"reasoning": f"Operator: {turn.goal or turn.transcript}",
                        "directives": turn.directives},
                emit=emit, bus=bus)
            self._sectors = [d["sector"] for d in accepted]
            self._rejected = refused
            self.stats["orders"] += bool(accepted)
            self.stats["rejected"] += len(refused)
        else:
            self.stats["questions"] += 1

        # The team re-plans under the operator's intent even when the order named no
        # sector -- "watch the south bank" is a goal, not a directive.
        if turn.goal:
            self.goal = turn.goal
            self.goal_seq += 1

        # If the filter refused everything, say so instead of the model's acknowledgement,
        # which promised something that did not happen.
        if self._rejected and not self._sectors:
            self._reply = (f"I could not carry that out: {', '.join(self._rejected)} "
                           f"refused by the feasibility check.")
        self._caption_until = world.t + CAPTION_HOLD_S
        self._speak(self._reply)

    def _speak(self, text: str) -> None:
        if not text:
            self._phase = IDLE
            return
        self._phase = SPEAKING
        try:
            self.speaker.play(self.provider.speak(text))
        except Exception:                                         # noqa: BLE001
            self._phase = IDLE                                    # captions still stand

    def _fail(self, world, why: str, emit) -> None:
        self._inflight = None
        self.stats["errored"] += 1
        self._phase = IDLE
        self._reply = why
        self._caption_until = world.t + CAPTION_HOLD_S
        emit("hivemind_offline", f"operator console: {why}")

    # ------------------------------------------------------------------ output

    def _render(self, world) -> bytes | None:
        """The operator's own fog-limited view. `[0]` is the swarm view; `[1]` is truth."""
        try:
            swarm_view = self._renderer.views(world)[0]
            return png.encode(swarm_view)
        except Exception:                                         # noqa: BLE001
            return None                                           # a blind turn still answers

    def _publish(self, world, bus) -> None:
        if bus is None:
            return
        # A muted channel keeps its reason on screen: it is a standing fault, not a
        # caption that should scroll away after twelve seconds.
        if self._phase != MUTED and world.t > self._caption_until:
            self._said = self._reply = ""
            self._sectors, self._rejected = [], []
            # The round-trip belongs to the turn that is on screen. Left standing it
            # sat on every idle frame afterwards, so the dashboard reported the latency
            # of an exchange whose captions had already scrolled away.
            self._latency_ms = 0
        # The meter needs a moving number, so the level is quantised into 20 steps and
        # made part of the change key: while the operator is talking this republishes as
        # the level moves, and the rest of the mission it stays as quiet as before.
        level = round(float(getattr(self.mic, "level", 0.0)), 2)
        frame = (self._phase, self._said, self._reply,
                 tuple(self._sectors), tuple(self._rejected),
                 int(level * 20) if self._phase in (IDLE, LISTENING) else 0)
        if frame == self._published:
            return                                    # only on change; this is not a 10 Hz feed
        self._published = frame
        bus.publish(topics.OPERATOR_VOICE, {
            "t": round(world.t, 2), "phase": self._phase, "said": self._said,
            "reply": self._reply, "sectors": self._sectors,
            "rejected": self._rejected, "latency_ms": self._latency_ms,
            "level": level, "recording": self._phase == LISTENING,
        })

"""Listening for the operator.

**Push to talk, on U, and that is the default.** A demo floor is not a quiet room: an
open microphone in a hall full of people picks up the next table's conversation, the
operator explaining the project to a judge, and its own reply through the laptop speaker.
Holding a key makes the boundary of an utterance a decision rather than a guess, and it
means nothing is sent to a model -- or paid for -- that the operator did not intend to
say. The key is pressed in Godot and arrives over the bridge (`nodes/bridge.py`), because
the dashboard owns the keyboard.

**The gate is a deadman, like the drive keys.** `press()` must be renewed or it lapses
after ``KEY_TTL_S``, so a dashboard that crashes or a socket that drops closes the
microphone instead of leaving it recording. `control/manual.py` holds a driven robot to
the same rule for the same reason.

Energy gating is kept behind ``push_to_talk=False`` rather than deleted: it is the mode
that needs no dashboard at all, and the tests cover both. In that mode speech starts when
the frame RMS crosses a threshold calibrated against the room's own noise floor and ends
after ``SILENCE_S`` of quiet.

**Calibration is measured, not assumed.** A fixed threshold works in one room and in no
other -- a laptop fan, a hackathon hall and a quiet desk differ by more than an order of
magnitude in noise floor. The first ``CALIBRATE_S`` of audio is taken as ambient and the
gate is set a fixed multiple above it, so the same code works in both rooms. Push-to-talk
skips it entirely: the key already said when to listen.

**Capture never blocks the tick loop.** PortAudio calls back on its own thread and this
module only ever appends to a buffer there; `Microphone.take()` is a non-blocking poll
that the console drains from the simulation thread, the same shape `nodes/hivemind.py`
uses for inference. Nothing here joins a thread.

`sounddevice` is an optional extra (`uv sync --extra voice`). Import failure is not an
error: `Microphone.available()` reports False and the demo runs silent.
"""

from __future__ import annotations

import contextlib
import io
import queue
import threading
import time
import wave

import numpy as np

#: What the fused model is fed. 16 kHz mono is plenty for speech and a quarter the bytes
#: of the 48 kHz the built-in microphone actually opens at.
RATE = 16_000
#: PortAudio callback granularity. 30 ms is short enough that the end of an utterance is
#: detected promptly and long enough that RMS is a stable measure of one frame.
BLOCK = 480

#: Seconds of ambient noise measured before the gate opens for the first time.
CALIBRATE_S = 1.0
#: The gate sits this far above the measured noise floor. Below ~3x, a fan trips it.
NOISE_GATE = 4.0
#: An absolute floor, for a room quiet enough that 4x ambient is still essentially zero.
MIN_RMS = 0.012
#: Quiet for this long ends the utterance. Shorter clips words at natural pauses.
SILENCE_S = 0.9
#: Shorter than this is a cough, a chair or a door, not an order.
MIN_UTTERANCE_S = 0.45
#: Longer than this is cut and sent. Bounds both latency and the audio-token bill.
MAX_UTTERANCE_S = 15.0
#: Audio kept from *before* the gate opened, so the first syllable is not clipped. It
#: matters more with a key than with a gate: people start talking as they press.
PREROLL_S = 0.3
#: A `press` older than this is a release. The dashboard renews at 10 Hz while U is held
#: (`godot/scripts/voice_key.gd`), so this leaves room for two lost frames -- the same
#: budget `control/manual.py` gives a held drive key.
KEY_TTL_S = 0.4


def wav_bytes(pcm: np.ndarray, rate: int = RATE) -> bytes:
    """int16 mono samples -> a WAV container, which is what the model wants."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(np.asarray(pcm, dtype=np.int16).tobytes())
    return buf.getvalue()


class Microphone:
    """Cuts the operator's speech into utterances on a background thread."""

    def __init__(self, *, rate: int = RATE, device=None,
                 push_to_talk: bool = True) -> None:
        self.rate = rate
        self.device = device
        #: True: record only while U is held. False: cut utterances by energy.
        self.push_to_talk = push_to_talk
        self._gate_t = 0.0
        self._stream = None
        self._utterances: queue.Queue[bytes] = queue.Queue()
        self._lock = threading.Lock()
        self._speaking = False
        self._buf: list[np.ndarray] = []
        self._preroll: list[np.ndarray] = []
        self._quiet_blocks = 0
        self._floor: float | None = None
        self._ambient: list[float] = []
        #: Set while the operator is mid-utterance. The console reads it to duck the
        #: assistant's own playback -- barge-in, without a key to press.
        self.hot = False
        #: Loudness of the most recent block, roughly 0..1. Drives the dashboard's
        #: recording meter, so it is updated in **both** modes and whether or not the
        #: gate is open -- a meter that only moved while recording could not show the
        #: operator that the microphone hears them before they press the key.
        self.level = 0.0
        self.error: str | None = None

    # ------------------------------------------------------------------ lifecycle

    @staticmethod
    def available() -> bool:
        try:
            import sounddevice  # noqa: F401
        except Exception:                                        # noqa: BLE001
            return False
        return True

    def start(self) -> bool:
        """True if the microphone opened. False is not fatal -- the demo runs silent."""
        try:
            import sounddevice as sd

            self._stream = sd.InputStream(
                samplerate=self.rate, channels=1, dtype="int16",
                blocksize=BLOCK, device=self.device, callback=self._on_audio)
            self._stream.start()
        except Exception as exc:                                 # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            self._stream = None
            return False
        return True

    def close(self) -> None:
        if self._stream is not None:
            with contextlib.suppress(Exception):
                self._stream.stop()
                self._stream.close()
            self._stream = None

    # ------------------------------------------------------------------ capture

    def _on_audio(self, indata, frames, time_info, status) -> None:
        """PortAudio's thread. Appends only; never calls a model or touches the world."""
        block = np.asarray(indata, dtype=np.int16).reshape(-1).copy()
        rms = float(np.sqrt(np.mean((block.astype(np.float32) / 32768.0) ** 2)))
        # Perceptual rather than linear: speech sits around 0.02-0.15 RMS, which is a
        # meter that never leaves the floor. A square root opens the bottom of the range
        # where the signal actually lives.
        self.level = float(min(1.0, (rms / 0.25) ** 0.5))

        if self.push_to_talk:
            with self._lock:
                if not self._speaking:
                    # Pre-roll runs even with the key up: people start the first word
                    # before the key is fully down, and clipping it costs the sentence.
                    self._keep_preroll(block)
                    return
                self._buf.append(block)
                held = len(self._buf) * BLOCK / self.rate
                if held >= MAX_UTTERANCE_S:
                    self._finish(held)
            return

        with self._lock:
            if self._floor is None:                       # still learning the room
                self._ambient.append(rms)
                if len(self._ambient) * BLOCK / self.rate >= CALIBRATE_S:
                    self._floor = max(float(np.median(self._ambient)) * NOISE_GATE, MIN_RMS)
                return

            loud = rms >= self._floor
            if not self._speaking:
                # Keep a rolling pre-roll so the gate never eats the first syllable.
                self._keep_preroll(block)
                if loud:
                    self._speaking = self.hot = True
                    self._buf = [*self._preroll, block]
                    self._preroll = []
                    self._quiet_blocks = 0
                return

            self._buf.append(block)
            self._quiet_blocks = 0 if loud else self._quiet_blocks + 1
            held = len(self._buf) * BLOCK / self.rate
            done = self._quiet_blocks * BLOCK / self.rate >= SILENCE_S
            if done or held >= MAX_UTTERANCE_S:
                self._finish(held)

    def _keep_preroll(self, block) -> None:
        """Called under the lock. A rolling window of the audio just before the gate."""
        self._preroll.append(block)
        keep = int(PREROLL_S * self.rate / BLOCK) + 1
        del self._preroll[:-keep]

    # ------------------------------------------------------------------ the key

    def press(self) -> None:
        """U is down. Opens the gate, or renews a gate already open."""
        if not self.push_to_talk:
            return
        with self._lock:
            self._gate_t = time.monotonic()
            if not self._speaking:
                self._speaking = self.hot = True
                self._buf = list(self._preroll)
                self._preroll = []

    def release(self) -> None:
        """U is up. Cuts the utterance and queues it, if it was long enough to be one."""
        if not self.push_to_talk:
            return
        with self._lock:
            if self._speaking:
                self._finish(len(self._buf) * BLOCK / self.rate)

    def expire(self) -> None:
        """Deadman. Called from the tick loop: an unrenewed key is a released one.

        Without it, a dashboard that crashes mid-press leaves the microphone recording
        until `MAX_UTTERANCE_S` and then sends whatever the room said.
        """
        if not (self.push_to_talk and self._speaking):
            return
        if time.monotonic() - self._gate_t >= KEY_TTL_S:
            self.release()

    def _finish(self, held: float) -> None:
        """Called under the lock, on the audio thread."""
        pcm = np.concatenate(self._buf) if self._buf else np.zeros(0, np.int16)
        self._buf = []
        self._speaking = self.hot = False
        self._quiet_blocks = 0
        if held >= MIN_UTTERANCE_S:
            self._utterances.put(wav_bytes(pcm, self.rate))

    # ------------------------------------------------------------------ drain

    def take(self) -> bytes | None:
        """Non-blocking. The newest complete utterance as WAV, or None.

        Older queued utterances are dropped rather than played catch-up: an operator who
        spoke twice while a turn was in flight meant the second thing.
        """
        newest = None
        while True:
            try:
                newest = self._utterances.get_nowait()
            except queue.Empty:
                return newest

    @property
    def calibrated(self) -> bool:
        """Ready to hear. Push-to-talk needs no noise floor -- the key already said when."""
        return self.push_to_talk or self._floor is not None

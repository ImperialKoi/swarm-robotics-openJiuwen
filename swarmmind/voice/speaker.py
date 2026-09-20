"""Playing the assistant's reply while it is still being generated.

`OmniProvider.speak` yields PCM as it arrives -- first chunk at 0.54 s against 1.27 s for
the whole clip (M-92). Waiting for the last chunk would throw that away and make every
answer feel a second slower than it is, so playback starts on the first chunk and the
stream feeds a ring buffer that the rest fills in behind it.

**Barge-in is the reason this is interruptible.** `stop()` drops the queue and silences
the device within one callback (~20 ms), so the operator talking over the assistant cuts
it off the way a real radio does. The console calls it the moment the microphone goes
hot; nothing has to be pressed.

Two rungs, because a demo that dies on an audio device is a self-inflicted wound:
PortAudio if `sounddevice` imported, and `/usr/bin/afplay` on a temporary WAV if it did
not. The fallback cannot stream or barge in -- it is there so the reply is still heard.
"""

from __future__ import annotations

import contextlib
import os
import queue
import subprocess
import tempfile
import threading
from collections.abc import Iterator

import numpy as np

from .mic import wav_bytes

#: `gpt-audio-mini` streams pcm16 at this rate; resampling would only add latency.
RATE = 24_000
BLOCK = 1024


class Speaker:
    """Streamed, interruptible playback of int16 mono PCM."""

    def __init__(self, *, rate: int = RATE, device=None) -> None:
        self.rate = rate
        self.device = device
        self._chunks: queue.Queue[np.ndarray | None] = queue.Queue()
        self._stream = None
        self._thread: threading.Thread | None = None
        self._tail = np.zeros(0, np.int16)
        self._generation = 0
        self._lock = threading.Lock()
        self.error: str | None = None

    @property
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------ play

    def play(self, chunks: Iterator[bytes]) -> None:
        """Start playing `chunks` as they arrive. Returns at once; never blocks."""
        self.stop()
        with self._lock:
            self._generation += 1
            generation = self._generation
        self._thread = threading.Thread(
            target=self._pump, args=(chunks, generation), daemon=True)
        self._thread.start()

    def _pump(self, chunks: Iterator[bytes], generation: int) -> None:
        """Pull from the network on a worker thread and hand PCM to the device."""
        opened = False
        collected: list[bytes] = []
        try:
            for raw in chunks:
                with self._lock:
                    if generation != self._generation:       # barged in on
                        return
                if not raw:
                    continue
                collected.append(raw)
                if self._stream is None and not opened:
                    opened = True
                    if not self._open():
                        continue                              # fall back after the loop
                if self._stream is not None:
                    self._chunks.put(np.frombuffer(raw, dtype=np.int16))
        except Exception as exc:                              # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if self._stream is not None:
                self._chunks.put(None)                        # end-of-speech marker
            elif collected:
                self._afplay(b"".join(collected), generation)

    def _open(self) -> bool:
        try:
            import sounddevice as sd

            self._stream = sd.OutputStream(
                samplerate=self.rate, channels=1, dtype="int16",
                blocksize=BLOCK, device=self.device, callback=self._on_need)
            self._stream.start()
        except Exception as exc:                              # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            self._stream = None
            return False
        return True

    def _on_need(self, outdata, frames, time_info, status) -> None:
        """PortAudio's thread. Drains the queue; silence when it runs dry."""
        out = self._tail
        while out.size < frames:
            try:
                nxt = self._chunks.get_nowait()
            except queue.Empty:
                break
            if nxt is None:
                break
            out = np.concatenate([out, nxt])
        if out.size >= frames:
            outdata[:, 0] = out[:frames]
            self._tail = out[frames:]
        else:
            outdata[:out.size, 0] = out
            outdata[out.size:, 0] = 0
            self._tail = np.zeros(0, np.int16)

    def _afplay(self, pcm: bytes, generation: int) -> None:
        """Rung 2: no PortAudio output device. Not interruptible, but audible."""
        path = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fh:
                fh.write(wav_bytes(np.frombuffer(pcm, dtype=np.int16), self.rate))
                path = fh.name
            with self._lock:
                if generation != self._generation:
                    return
            subprocess.run(["/usr/bin/afplay", path], check=False,  # noqa: S603
                           timeout=60, capture_output=True)
        except Exception as exc:                              # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            if path:
                with contextlib.suppress(OSError):
                    os.unlink(path)

    # ------------------------------------------------------------------ stop

    def stop(self) -> None:
        """Barge-in. Silences the device and abandons whatever is still downloading."""
        with self._lock:
            self._generation += 1
        while True:
            try:
                self._chunks.get_nowait()
            except queue.Empty:
                break
        self._tail = np.zeros(0, np.int16)
        if self._stream is not None:
            with contextlib.suppress(Exception):
                self._stream.stop()
                self._stream.close()
            self._stream = None

    def close(self) -> None:
        self.stop()

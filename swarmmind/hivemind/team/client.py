"""Supervise the optional runtime off the simulation thread."""

import json
import os
import queue
import selectors
import subprocess
import threading
import time
from contextlib import suppress

from .config import ROOT


class WorkerClient:
    def __init__(self, python, runtime, config, trace, stop_file=None):
        self.python, self.runtime, self.config, self.trace = python, runtime, config, trace
        self.stop_file = stop_file
        self.inbox = queue.Queue(maxsize=1)
        self.outbox = queue.Queue(maxsize=64)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="response-team")
        self.thread.start()

    def submit(self, snapshot):
        self.inbox.put_nowait({"snapshot": snapshot, "config": self.config.model_dump()})

    def poll(self):
        rows = []
        while True:
            try:
                rows.append(self.outbox.get_nowait())
            except queue.Empty:
                return rows

    def _run(self):
        proc = None
        selector = selectors.DefaultSelector()
        try:
            proc = subprocess.Popen(
                [str(self.python), "-u", "-m", "swarmmind.hivemind.team.worker",
                 "--runtime", self.runtime], cwd=ROOT, stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                env={**os.environ, "PYTHONPATH": str(ROOT)},
            )
            selector.register(proc.stdout, selectors.EVENT_READ)
            buffer = b""
            ready, pending = False, None
            started = time.monotonic()
            while not self.stop.is_set():
                if self.stop_file and self.stop_file.exists():
                    self.outbox.put_nowait({"kind": "disabled", "error": "operator stop file"})
                    return
                if proc.poll() is not None:
                    raise RuntimeError(f"team worker exited ({proc.returncode})")
                if ready and pending is None:
                    try:
                        pending = self.inbox.get_nowait()
                        proc.stdin.write((json.dumps(pending) + "\n").encode())
                        proc.stdin.flush()
                        started = time.monotonic()
                    except queue.Empty:
                        pass
                limit = self.config.deadline_s if ready else 90.0
                if (pending is not None or not ready) and time.monotonic() - started > limit:
                    raise TimeoutError("team runtime deadline exceeded")
                for key, _ in selector.select(timeout=0.05):
                    chunk = os.read(key.fileobj.fileno(), 65536)
                    if not chunk:
                        raise RuntimeError("team worker closed its output")
                    buffer += chunk
                    if len(buffer) > 262144:
                        raise ValueError("team worker output exceeded limit")
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        try:
                            row = json.loads(line)
                        except ValueError:
                            continue  # framework startup logging, never a protocol result
                        if not isinstance(row, dict):
                            continue
                        kind = row.get("kind")
                        if kind == "trace":
                            self.trace.record(row.pop("event"), **{k: v for k, v in row.items()
                                                                  if k != "kind"})
                        elif kind in {"ready", "result", "error"}:
                            if kind == "ready":
                                ready = True
                            else:
                                pending = None
                            self.outbox.put_nowait(row)
        except Exception as exc:
            with suppress(queue.Full):
                self.outbox.put_nowait({"kind": "fatal", "error": f"{type(exc).__name__}: {exc}"[:240]})
        finally:
            selector.close()
            if proc is not None:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=1)
                for pipe in (proc.stdin, proc.stdout):
                    if pipe:
                        pipe.close()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3)

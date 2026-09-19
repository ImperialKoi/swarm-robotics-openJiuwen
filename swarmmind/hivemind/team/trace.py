"""Bounded asynchronous trace writing; no file I/O in the simulation tick."""

import json
import queue
import threading
from collections import Counter, deque
from pathlib import Path


class Trace:
    def __init__(self, path=None, *, console=False):
        self.path = Path(path) if path else None
        self.console = console
        self.records = deque(maxlen=512)
        self.queue = queue.Queue(maxsize=4096)
        self.dropped = 0
        self.error = None
        self.closed = False
        self.thread = None
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            # Surface unwritable destinations at startup, not halfway through a demo.
            with self.path.open("w", encoding="utf-8"):
                pass
            self.thread = threading.Thread(target=self._write, daemon=True, name="team-trace")
            self.thread.start()

    def record(self, event, **data):
        if self.closed:
            return
        row = {"event": event, **data}
        self.records.append(row)
        if self.thread:
            try:
                self.queue.put_nowait(row)
            except queue.Full:
                self.dropped += 1

    def _write(self):
        shown_outcomes = set()
        try:
            with self.path.open("a", encoding="utf-8") as f:
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    f.write(json.dumps(row, allow_nan=False) + "\n")
                    f.flush()
                    outcome = row["event"] == "outcome" and row.get("request") not in shown_outcomes
                    if self.console and (outcome or row["event"] in {
                        "ready", "message", "revision_requested", "revised", "applied",
                        "rejected", "fallback", "disabled", "withheld", "finished",
                    }):
                        note = row.get("note", row.get("reason", row.get("directive", "")))
                        if row["event"] == "revision_requested":
                            note = f"choice {row['original']['choice']} -> {row['alternative']['choice']}: {row['alternative']['note']}"
                        elif row["event"] == "revised":
                            note = f"{row['before']} -> {row['after']}"
                        elif outcome:
                            shown_outcomes.add(row.get("request"))
                            note = f"{row['observed']} (association, not causal attribution)"
                        elif row["event"] == "ready":
                            note = f"LIVE TEAM ({row['runtime']})"
                        elif row["event"] == "finished":
                            note = row.get("stats", row.get("result", ""))
                        print(f"  TEAM {row.get('request', '')} {row.get('role', '')} "
                              f"{row['event']}: {note}", flush=True)
        except (OSError, ValueError) as exc:
            self.error = str(exc)

    def close(self):
        if self.closed:
            return
        self.record("trace_status", dropped=self.dropped, error=self.error)
        self.closed = True
        if self.thread and self.thread.is_alive():
            try:
                self.queue.put(None, timeout=2)
                self.thread.join(timeout=3)
                if self.thread.is_alive():
                    self.error = "trace writer did not finish before shutdown"
            except queue.Full:
                self.error = "trace writer did not drain"


def render_report(path):
    path = Path(path)
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    lines = ["# Rescue response team — observed run", "",
             "This is an execution trace, not a claim of rescue uplift.", "",
             "| Request | Stage | Role | Evidence / result |", "|---|---|---|---|"]
    for row in rows:
        if row["event"] not in {"started", "decomposed", "message", "revision_requested", "revised",
                               "reviewed", "applied", "rejected", "fallback", "withheld", "veto",
                               "expired", "disabled", "finished", "trace_status"}:
            continue
        detail = {k: v for k, v in row.items() if k not in {"request", "event", "role"}}
        text = json.dumps(detail, ensure_ascii=False).replace("|", "\\|").replace("\n", " ")
        lines.append(f"| {row.get('request', '')} | {row['event']} | {row.get('role', '')} | {text} |")
    outcomes = Counter(r["request"] for r in rows if r["event"] == "outcome")
    if outcomes:
        lines.extend(["", "Subsequent task awards in a leased sector (association only):", ""])
        lines.extend(f"- {request}: {count} awards; individual events are in the JSONL trace."
                     for request, count in sorted(outcomes.items()))
    out = path.with_suffix(".md")
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out

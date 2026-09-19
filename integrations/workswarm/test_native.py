"""Native SDK integration checks with deterministic HTTP model responses.

Run with this integration's interpreter. No real LLM or API key is used here;
check_response_team.py and the recorded live traces cover real local inference.
"""

import asyncio
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from swarmmind.hivemind.team.client import WorkerClient  # noqa: E402
from swarmmind.hivemind.team.config import TeamConfig  # noqa: E402
from swarmmind.hivemind.team.native import run_native  # noqa: E402
from swarmmind.hivemind.team.trace import Trace  # noqa: E402


def fixture(name):
    sectors = [{"id": sid, "explored": 1.0, "hazard": 0.0, "connected": 0,
                "contacts": 0, "resolved_reports": 0, "backlog": {}, "collection": False,
                "priority": 1, "abandoned": False} for sid in ("A1", "B1", "C1", "D1")]
    sectors[0].update(contacts=1, backlog={"extract": 1})
    sectors[1].update(explored=0.2, connected=3)
    return {"id": name, "sim_time": 60.0, "duration": 420, "sectors": sectors,
            "lanes": {lane: {"connected": 1, "battery": 0.8}
                      for lane in ("none", "scoop", "gripper", "antenna")},
            "events": [], "previous": {}}


class ModelStub(BaseHTTPRequestHandler):
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        assert not body.get("tools"), "the model adapter constrains arguments with JSON schema"
        name = body["response_format"]["json_schema"]["name"]
        self.server.calls.append(name)
        if name == "verify_task":
            args = {"task_id": "rescue-plan", "decision": self.server.vote, "feedback": "Observed review"}
        else:
            args = {"choice": 0 if name == "dispatch_plan" else 1, "note": "B1 supports exploration"}
        if name == "recommend_plan" and self.server.attack:
            self.server.attack = False
            args["tool"] = "submit_reviewed_plan"  # schema must reject the forged tool selection
        assert not body.get("stream"), "adapter returns one validated SDK tool-call chunk"
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        payload = {"id": "stub", "object": "chat.completion", "created": 0, "model": "test-double",
                   "choices": [{"index": 0, "message": {"role": "assistant", "content": json.dumps(args)},
                                "finish_reason": "stop"}],
                   "usage": {"prompt_tokens": 20, "completion_tokens": 20, "total_tokens": 40}}
        self.wfile.write(json.dumps(payload).encode())
        self.wfile.flush()

    def log_message(self, *args):
        pass


async def checks(server):
    config = TeamConfig.load().model_copy(update={
        "model_url": f"http://127.0.0.1:{server.server_port}/v1/chat/completions"})
    for case in ("revision", "veto", "authority", "budget", "repeat"):
        server.vote = "fail" if case == "veto" else "pass"
        server.attack = case == "authority"
        server.calls = []
        rows = []
        selected = config.model_copy(update={"token_budget": 1}) if case == "budget" else config
        if case == "authority":
            selected = selected.model_copy(update={"deadline_s": 2.0})
        try:
            result = await run_native(fixture(f"native-test-{case}"), selected,
                                      lambda event, rows=rows, **data: rows.append({"event": event, **data}))
        except TimeoutError:
            assert case == "authority", case
            result = {"directive": None}
        if case in {"veto", "budget", "authority"}:
            assert result["directive"] is None, (case, result)
            assert "submit_reviewed_plan" not in server.calls, server.calls
        else:
            assert result["directive"]["sector"] == "B1", result
            tasks = [r for r in rows if r["event"] == "native_task"]
            assert [r["operation"] for r in tasks] == [
                "create_task", "member_complete_task", "verify_task", "submit_reviewed_plan"], tasks
            assert tasks[-1]["status"] == "completed"
            assert any(r["event"] == "revised" for r in rows)
            assert [r["role"] for r in rows if r["event"] == "message"] == [
                "lead", "logistics", "safety", "lead"]
        print(f"NATIVE SDK TEST PASS: {case}; calls={server.calls}", flush=True)
    print("NATIVE SDK TESTS: 5 passed; deterministic HTTP responses, no real model", flush=True)


def check_worker(server):
    server.vote, server.attack, server.calls = "pass", False, []
    config = TeamConfig.load().model_copy(update={
        "model_url": f"http://127.0.0.1:{server.server_port}/v1/chat/completions"})
    trace = Trace()
    client = WorkerClient(Path(sys.executable), "workswarm", config, trace)
    received, submitted = 0, 0
    try:
        deadline = time.monotonic() + 130
        while time.monotonic() < deadline and received < 2:
            for row in client.poll():
                assert row["kind"] not in {"fatal", "error"}, row
                if row["kind"] == "ready":
                    client.submit(fixture("native-worker-1"))
                    submitted = 1
                elif row["kind"] == "result":
                    assert row["result"]["directive"]["sector"] == "B1", row
                    received += 1
                    if submitted == 1:
                        client.submit(fixture("native-worker-2"))
                        submitted += 1
            time.sleep(0.02)
        assert received == 2, (received, list(trace.records)[-5:])
        assert any(r["event"] == "native_stream" for r in trace.records)
        print("NATIVE WORKER TESTS: 2 repeated requests passed through the real stdio protocol", flush=True)
    finally:
        client.close()
        trace.close()


def main():
    server = ThreadingHTTPServer(("127.0.0.1", 0), ModelStub)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        asyncio.run(checks(server))
        check_worker(server)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()

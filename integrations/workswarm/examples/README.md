# Recorded examples

These files are recordings, not live inference or canned responses used by the app.

## Final-build fixture

[`final-fixture.md`](final-fixture.md), from [`final-fixture.jsonl`](final-fixture.jsonl),
records the final code running the real WorkSwarm/local-model pipeline on `test.yaml`:
the lead initially withholds, then logistics changes A3 to A2, the lead accepts, safety
approves, and the simulator applies the order. A subsequent task award in A2 is recorded.
The operator stops the team after t=40; the mission continues to t=65 at 1.00× realtime.
There is exactly one shutdown acknowledgment. This small fixture is verification, not
an additional demo map or a rescue-performance comparison.

## Real mission excerpt

[`seed42-excerpt.md`](seed42-excerpt.md) is rendered from
[`seed42-excerpt.jsonl`](seed42-excerpt.jsonl). It retains startup/final records plus
three complete selected proposal IDs from a real 512-robot, seed-42 mission using
WorkSwarm 0.2.6 and the stock local Qwen2.5-1.5B model:

- `response-0009`: logistics requests a different candidate; the lead revises to B7;
  safety approves; the order is applied and subsequent task awards are recorded.
- `response-0010`: another peer revision reaches safety, which withholds dispatch.
- `response-0012`: the model approves, but current observations have changed and the
  simulator rejects the proposal before it takes effect.

The complete run had 18 episodes, nine applied orders and two current-state rejections;
its final score was 59/110 rescued and 92/110 found. The final statistics describe the
whole run, not only this excerpt. No matched baseline was run, so this is not rescue
uplift evidence. The full local trace is `runs/team/rehearsal.jsonl`.

The team was stopped between t=340 and t=350; the swarm finished t=420. An earlier
version counted the same stop acknowledgment twice; these original records preserve
that defect. The final implementation prioritizes supervisor shutdown acknowledgments
and has a regression test. Clock and trace-display fixes followed this rehearsal;
see M-79 for the exact verification scope.

Model notes are untrusted, short explanations and sometimes misuse terms such as
“connected sectors.” The numeric tool records count connected **robots**. The final
HUD message uses validated structured facts; it does not repeat free-text model claims.

## Failed synthetic diagnostic

[`synthetic-peer-check.md`](synthetic-peer-check.md) and its JSONL preserve a separate
real-runtime/model check on explicitly synthetic observations. The lead withheld its
choice while its note proposed rescue work; no revision was dispatched and the check
failed. This is a model limitation, not a passing demonstration. Deterministic tests
exercise peer revision/veto independently; the live mission excerpt shows an actual
successful revision. No synthetic observation was sent into the simulator.

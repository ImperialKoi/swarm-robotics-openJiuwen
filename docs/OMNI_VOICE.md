# The operator voice channel

An incident commander talks to a 512-robot search-and-rescue swarm, and it answers out
loud. One model call hears the question, looks at the operator's map, and replies with
speech plus a sector order that the swarm actually executes.

This is an **optional application layer**. With the microphone muted, `--voice` omitted,
or the network down, the mission runs exactly as it did before: the auction allocates,
Tier 1 keeps the safety floor, and Tier 2 self-heals. No rescue-score improvement is
claimed for it, and none has been measured.

---

## The problem

An operator watching 512 robots across 96 sectors is hands-busy and eyes-busy. The
dashboard's sector table is the swarm's whole strategic picture and it does not fit on a
screen, let alone in the attention of somebody who is also watching a hazard front grow.
Reading it is not the job; deciding is. Speech is the only input channel that leaves the
eyes on the map, and audio is the only output channel that does not require looking away
to receive an answer.

That is the case for multimodal, and it is why a text chatbot bolted to this dashboard
would be worse than nothing: it would demand exactly the attention the operator does not
have.

## What it does

```
operator holds U on the dashboard and speaks
      │  16 kHz WAV, cut by the key: press to open, release to send
      ▼
┌─ ONE request ─────────────────────────────────────────────┐
│  in : speech + the operator's fog-limited map + the       │  google/gemini-3.5-flash
│       blackboard summary Tier 3 reads                     │  1.87 s, $0.0029
│  out: transcript · spoken reply · team goal · directives  │
└───────────────────────────────────────────────────────────┘
      │                              │
      │ directives                   │ reply text
      ▼                              ▼
 hivemind/filter.py            openai/gpt-audio-mini, streamed
      │  (survivors)                 │  first audio at 0.54 s
      ▼                              ▼
 operator_sectors ──outranks──▶ Tier 3 and the response team
      │                         captions under the map (Godot) + audio
      ▼
 auction → 512 robots
```

**End of speech to first spoken word: ~2.4 s.** About $0.005 per turn, so a 15-turn demo
costs roughly $0.08. Numbers measured live in [MEASUREMENTS.md M-92](MEASUREMENTS.md).

## The three modalities are one call, not three features

The weak version of this is `mic → transcriber → text model → TTS`, with a vision model
bolted on beside it. That is four pipelines and the modalities never meet.

Here, **one request carries the speech, the image and the text together**, and the
cross-modal part is the point: *"how is coverage on the north ridge?"* is a question
about the picture, asked out loud, answered against the swarm's own telemetry. No single
modality answers it. `tests/test_omni_provider.py` asserts the three content parts are in
one message, because that property is the claim.

Speech generation is a second model only because **no model on OpenRouter both accepts
images and emits audio** — the two `openai/gpt-audio*` models are the only ones with
audio output and neither takes an image (M-92). That is a platform constraint, recorded
rather than designed around.

## What the model is allowed to see

The image is `MissionRenderer.views(world)[0]` — the **swarm view**: the appearance
raster masked by the fog, over ground the swarm has actually revealed. It is the same
pixels `perception/` already feeds the victim detector, so the assistant is a second and
better-read consumer of sensor data the swarm already had.

It is never `views()[1]`, the god view, which draws `COLOR_VICTIM_TRUE` dots straight
from `world.victims`. `swarmmind/voice/` is a guarded tree in
`tests/test_no_ground_truth_leak.py` alongside `nodes`, `control`, `hivemind` and
`training`, so CLAUDE.md invariant #4 covers this code like any other.

The detector's own 48×48 egocentric frames were the obvious candidate and are measurably
the wrong choice: they are colour-coded rasters rather than photographs, and a VLM asked
to read one is colour-thresholding what `perception/classical.py` already thresholds
faster and locally. That comparison is in M-92.

## Precedence: the operator outranks the models

| rank | source | may retask a sector held by |
|---|---|---|
| 1 | **operator** (spoken) | anyone |
| 2 | response team (lead/logistics/safety) | Tier 3 |
| 3 | Tier 3 hivemind | — |
| 4 | scripted rung | — |

A spoken order claims its sectors in `HivemindNode.operator_sectors`. While a claim is
live, neither the Tier 3 cycle nor a reviewed team proposal may touch that sector — but
**the team keeps working every sector the operator did not name**, and re-plans under the
operator's spoken goal rather than being silenced by it. That is the difference between
outranking a team and replacing one.

Two things the operator does *not* outrank, on purpose:

- **`hivemind/filter.py`.** A human's words go through the same feasibility filter a
  model's do. Its rules are physical — a real sector id, never abandoning the last open
  sector — and a refusal spoken back (*"I could not carry that out: D4 refused by the
  feasibility check"*) is the system explaining itself out loud, which is a better demo
  beat than silent compliance.
- **Tier 1.** The swept-circle wall override is not bypassable from a microphone any more
  than from anywhere else (CLAUDE.md invariant #2).

Operator claims **expire on the same 30 s clock as every other directive**. A commander
who abandons a sector and then walks away from the microphone must not leave a third of
the map locked, and a person can cause that failure exactly as a stale model directive can.

## The fourth agent: a scout that reads pixels

`--team-scout` adds `VisionScout` to the response team. The rescue lead, the logistics
specialist and the safety reviewer all reason over the same table of numbers; a fourth
text agent would be four models arguing about one spreadsheet. The scout looks at the
operator's map instead and hands the lead one bounded observation — *where coverage is
thin, which listed sector the picture favours* — which none of its peers can derive from
the snapshot.

It runs in-process on the team's own 20 s cadence, and its finding is attached to the
snapshot as an ordinary observation, so the isolated WorkSwarm worker protocol is
unchanged for every run that does not use it. `qwen/qwen3-vl-30b-a3b-instruct`, 1.81 s
and $0.00009 per look. It advises; it does not vote. The lead still chooses, logistics
can still overrule on capacity, and safety can still veto the result.

A scout that never answers costs an observation, not an episode — the other three roles
run unchanged without it, asserted in `tests/test_team_scout.py`.

---

## Running it

```bash
uv sync --extra bridge --extra voice --extra dev --extra evo
echo 'OPENROUTER_API_KEY=sk-or-...' >> .env          # gitignored

# the voice channel on its own
uv run --env-file .env python -m swarmmind.cli run --demo --voice

# with the response team and its visual scout: both tracks in one run
uv run --env-file .env python -m swarmmind.cli run --demo --voice \
    --response-team --team-scout --team-trace runs/team/live.jsonl
```

Then start the Godot dashboard. Captions appear under the map; the reply is played by the
process that generated it, so no audio crosses the WebSocket and nothing competes with
the state frames for Godot's 64 KB inbound buffer.

`--voice` is a flag on the existing `run` command, not a second process to launch.

### Dependencies

`sounddevice` (the `voice` extra) for the microphone and playback, plus
`OPENROUTER_API_KEY` and network. Everything else is already in the project. PortAudio
ships inside the wheel; the microphone is a thread in the demo process, not a new
resident one, so the 8 GB budget in [TECHNICAL.md §9](TECHNICAL.md) is unchanged.

### Try saying

- *"How is coverage on the north ridge?"* — a question. Answered, nothing retasked.
- *"Pull everyone out of D4 and push C3."* — an order. Directives applied, sectors
  claimed, the team re-plans around it.
- *"Abandon the whole map."* — refused by the filter, out loud, with the reason.

**Hold `U` while you speak and release to send.** Pressing it again while the assistant
is mid-sentence cuts it off — reaching for the key *is* the interruption, so there is
nothing else to press. The key works in every view, including the orbit: you are
addressing the swarm, not a unit.

Push-to-talk is the default because a demo floor is loud. An always-open microphone there
hears the next table, the operator explaining the project to a judge, and the assistant's
own reply out of the laptop speaker — and each of those is a model call that gets paid
for and answered. The energy-gated mode is still there behind
`Microphone(push_to_talk=False)` for a setup with no dashboard.

The held key is a **deadman**: Godot renews it at 10 Hz and the simulator treats a press
older than 0.4 s as a release, so a crashed dashboard or a dropped socket closes the
microphone instead of leaving it recording. The drive keys work the same way and for the
same reason.

## Limitations, stated plainly

- **No rescue-score claim.** This has not been through `training/gate.py` and no arm has
  been measured with it on. It is an interface, not a performance change.
- **It needs the network.** This is a deliberate departure from the project's offline
  rule, taken for these prize tracks. The mission itself still runs with the channel
  dead: a model failure leaves the swarm running and is asserted in
  `tests/test_voice_console.py`.
- **Push-to-talk needs the dashboard.** The key is pressed in Godot and arrives over the
  bridge, so a headless or dashboard-less setup must use `Microphone(push_to_talk=False)`
  and its energy gate, where a loud room will cut utterances early.
- **Nothing here is fine-tuned.** Both models are stock and hosted. The honest
  description is *schema-constrained structured output over a hosted multimodal model*.
- **Captions are the dashboard's only voice surface**, and they are verified rather than
  assumed: `godot/tests/voice_check.gd` builds the real HUD in real Godot and asserts the
  real `Label`s. After adding it, run
  `Godot --headless --path godot --import` once — the class cache is generated and
  gitignored, so a new `class_name` does not resolve until the project is reimported.

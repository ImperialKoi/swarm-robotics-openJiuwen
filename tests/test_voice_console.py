"""The operator voice channel, with no microphone, no network and no audio device.

The console is the one place a human can outrank Tier 3, so the precedence rules are
what this file is mostly about. Every model call is faked; nothing here is billable.
"""

from __future__ import annotations

import numpy as np
import pytest

from swarmmind.hivemind.providers.omni import OperatorTurn
from swarmmind.mission import Mission
from swarmmind.sim.scenario import Scenario
from swarmmind.voice.console import IDLE, LISTENING, MUTED, SPEAKING, THINKING, OperatorConsole
from swarmmind.voice.mic import (
    KEY_TTL_S,
    MIN_UTTERANCE_S,
    RATE,
    Microphone,
    wav_bytes,
)


class FakeMic:
    """Stands in for PortAudio. The key interface plus `hot` and `take()`."""

    def __init__(self):
        self.hot = False
        self._queued: list[bytes] = []
        self.closed = False
        self.presses = 0
        self.releases = 0
        self.expiries = 0
        self.audio = b"RIFFfake"

    def say(self, audio=b"RIFFfake"):
        self._queued.append(audio)

    def press(self):
        self.presses += 1
        self.hot = True

    def release(self):
        self.releases += 1
        if self.hot:
            self._queued.append(self.audio)
        self.hot = False

    def expire(self):
        self.expiries += 1

    def start(self):
        return True

    def take(self):
        return self._queued.pop(0) if self._queued else None

    def close(self):
        self.closed = True


class FakeSpeaker:
    def __init__(self):
        self.played: list[str] = []
        self.stops = 0
        self.busy = False

    def play(self, chunks):
        self.played.append(b"".join(chunks).decode())
        self.busy = True

    def stop(self):
        self.stops += 1
        self.busy = False

    def close(self):
        self.busy = False


class FakeOmni:
    def __init__(self, turn):
        self.turn = turn
        self.calls: list[tuple] = []

    def understand(self, audio, view, blackboard, sectors, timeout=20.0):
        self.calls.append((audio, view, blackboard, tuple(sectors)))
        return self.turn

    def speak(self, text, timeout=30.0):
        yield text.encode()


@pytest.fixture
def mission():
    m = Mission(Scenario.load("test"), 42)
    yield m
    m.close()


def console_for(mission, turn, **kw):
    mic, speaker = FakeMic(), FakeSpeaker()
    c = OperatorConsole(mission.world, FakeOmni(turn), mic=mic, speaker=speaker, **kw)
    c.start()
    return c, mic, speaker


def pump(mission, console, ticks=60):
    """Step until the turn lands. The fused call runs on a worker thread, so the
    console needs several ticks to collect it -- exactly as it does in the mission."""
    import time

    for _ in range(ticks):
        console.step(mission.world, mission.executor, mission.tracker, mission.events,
                     mission.hivemind, emit=mission._emit, bus=mission.bus)
        if console.stats["turns"] and console._inflight is None and console.phase != THINKING:
            return console
        time.sleep(0.01)
    return console


# --------------------------------------------------------------- the mission survives

def test_a_mission_with_no_voice_is_untouched(mission):
    """Invariant #1, applied to a human: the swarm never waits on the operator."""
    assert mission.voice is None
    for _ in range(20):
        mission.tick()
    assert mission.world.t > 0


def test_a_muted_console_does_nothing_at_all(mission):
    c = OperatorConsole(mission.world, FakeOmni(OperatorTurn("", "")),
                        mic=FakeMic(), speaker=FakeSpeaker())
    assert c.phase == MUTED                      # start() not called
    c.step(mission.world, mission.executor, mission.tracker, mission.events,
           mission.hivemind, bus=mission.bus)
    assert c.stats["turns"] == 0


def test_voice_without_the_filter_is_refused(mission):
    with pytest.raises(ValueError, match="voice requires the directive filter"):
        Mission(Scenario.load("test"), 42, hivemind=False, voice=object())


# --------------------------------------------------------------- one spoken turn

def test_a_spoken_order_retasks_the_swarm_and_is_spoken_back(mission):
    sector = mission.world.sector_ids[0]
    turn = OperatorTurn("Pull out of " + sector, "Clearing it now.",
                        goal="Evacuate and hold.",
                        directives=[{"sector": sector, "priority": "high",
                                     "action": "rescue"}])
    c, mic, speaker = console_for(mission, turn)
    mic.say()
    pump(mission, c)

    assert c.stats["turns"] == 1 and c.stats["orders"] == 1
    assert sector in mission.hivemind.applied, "the order never reached the swarm"
    assert sector in mission.hivemind.operator_sectors, "the operator did not claim it"
    assert speaker.played == ["Clearing it now."], "the reply was not spoken"
    assert c.goal == "Evacuate and hold." and c.goal_seq == 1


def test_a_question_answers_without_retasking_anything(mission):
    c, mic, speaker = console_for(
        mission, OperatorTurn("How is D4?", "Two contacts, hazard 40 m out."))
    mic.say()
    pump(mission, c)
    assert c.stats["questions"] == 1 and c.stats["orders"] == 0
    assert not mission.hivemind.operator_sectors, "a question claimed a sector"
    assert speaker.played == ["Two contacts, hazard 40 m out."]
    assert c.goal is None


def test_the_model_is_shown_speech_the_view_and_the_blackboard(mission):
    c, mic, _ = console_for(mission, OperatorTurn("hi", "hello"))
    mic.say(b"RIFF-the-operator")
    pump(mission, c)
    audio, view, blackboard, sectors = c.provider.calls[0]
    assert audio == b"RIFF-the-operator"
    assert view and view[:8] == b"\x89PNG\r\n\x1a\n", "no rendered view was sent"
    assert blackboard and sectors == tuple(mission.world.sector_ids)


# --------------------------------------------------------------- precedence

def test_the_operator_outranks_a_live_tier_3_directive(mission):
    """The central hivemind may not retask a sector the operator just claimed."""
    sector = mission.world.sector_ids[0]
    c, mic, _ = console_for(mission, OperatorTurn(
        "hold " + sector, "Holding.",
        directives=[{"sector": sector, "priority": "high", "action": "rescue"}]))
    mic.say()
    pump(mission, c)
    assert mission.hivemind.applied[sector].action == "rescue"

    # Tier 3 now tries to abandon the same sector, through its ordinary path.
    assert sector in mission.hivemind.held_sectors
    kept = [d for d in [{"sector": sector, "priority": "abandon", "action": "abandon"}]
            if d["sector"] not in mission.hivemind.held_sectors]
    assert kept == [], "Tier 3 was allowed to overrule the operator"
    assert mission.hivemind.applied[sector].action == "rescue"


def test_the_response_team_defers_to_a_live_operator_order(mission):
    sector = mission.world.sector_ids[0]
    mission.hivemind.operator_sectors[sector] = mission.world.t
    seen = []
    accepted = mission.hivemind.apply_reviewed(
        mission.world,
        {"reasoning": "team plan", "directives": [
            {"sector": sector, "priority": "abandon", "action": "abandon"}]},
        "api", emit=lambda k, t, **kw: seen.append(k), bus=mission.bus)
    assert accepted == [], "the team overruled the operator"
    assert "directive_rejected" in seen


def test_the_team_still_works_every_sector_the_operator_did_not_name(mission):
    """'Team leaders still issue their own commands' -- precedence is per sector."""
    held, free = mission.world.sector_ids[0], mission.world.sector_ids[1]
    mission.hivemind.operator_sectors[held] = mission.world.t
    accepted = mission.hivemind.apply_reviewed(
        mission.world,
        {"reasoning": "team plan", "directives": [
            {"sector": held, "priority": "high", "action": "rescue"},
            {"sector": free, "priority": "high", "action": "explore"}]},
        "api", emit=lambda *a, **kw: None, bus=mission.bus)
    assert [d["sector"] for d in accepted] == [free]


def test_an_operator_claim_expires_like_every_other_directive(mission):
    """A human who walks away from the microphone must not lock the map."""
    sector = mission.world.sector_ids[0]
    mission.hivemind.operator_sectors[sector] = mission.world.t
    mission.world.t += mission.hivemind.expiry + 1.0
    mission.hivemind._expire(mission.world)
    assert sector not in mission.hivemind.operator_sectors


def test_an_infeasible_spoken_order_is_refused_and_said_out_loud(mission):
    """The filter runs on a human's words too, and the refusal is the answer."""
    c, mic, speaker = console_for(mission, OperatorTurn(
        "abandon everything", "Abandoning the map.",
        directives=[{"sector": "ZZ99", "priority": "abandon", "action": "abandon"}]))
    mic.say()
    pump(mission, c)
    assert not mission.hivemind.operator_sectors, "a refused order still claimed a sector"
    assert c.stats["rejected"] == 1
    assert "could not carry that out" in speaker.played[0], (
        "the operator was told the order succeeded when it did not")


# --------------------------------------------------------------- interaction

def test_pressing_the_key_cuts_the_reply_off(mission):
    """Reaching for U *is* the interruption; there is nothing else to press."""
    c, mic, speaker = console_for(mission, OperatorTurn("hi", "a long answer"))
    speaker.busy = True
    c.on_mic_key(True)
    assert speaker.stops == 1 and c.stats["barge_ins"] == 1
    assert mic.presses == 1


def test_the_key_opens_and_closes_the_microphone(mission):
    c, mic, _ = console_for(mission, OperatorTurn("hi", "hello"))
    c.on_mic_key(True)
    assert mic.presses == 1 and mic.hot, "U down did not start recording"
    c.on_mic_key(False)
    assert mic.releases == 1 and not mic.hot, "U up did not stop recording"


def test_a_muted_console_ignores_the_key(mission):
    c = OperatorConsole(mission.world, FakeOmni(OperatorTurn("", "")),
                        mic=FakeMic(), speaker=FakeSpeaker())
    c.on_mic_key(True)
    assert c.mic.presses == 0, "a muted console opened the microphone"


def test_the_deadman_runs_every_tick(mission):
    """A crashed dashboard must not leave the microphone recording the room."""
    c, mic, _ = console_for(mission, OperatorTurn("hi", "hello"))
    for _ in range(5):
        c.step(mission.world, mission.executor, mission.tracker, mission.events,
               mission.hivemind, bus=mission.bus)
    assert mic.expiries == 5


def test_the_phase_tells_the_dashboard_what_is_happening(mission):
    c, mic, _ = console_for(mission, OperatorTurn("hi", "hello"))
    assert c.phase == IDLE
    c.on_mic_key(True)
    c.step(mission.world, mission.executor, mission.tracker, mission.events,
           mission.hivemind, bus=mission.bus)
    assert c.phase == LISTENING
    c.on_mic_key(False)
    c.step(mission.world, mission.executor, mission.tracker, mission.events,
           mission.hivemind, bus=mission.bus)
    assert c.phase == THINKING
    pump(mission, c)
    assert c.phase in (SPEAKING, IDLE)


def test_captions_are_published_on_change_not_every_tick(mission):
    seen = []
    mission.bus.subscribe("/operator/voice", seen.append)
    c, mic, _ = console_for(mission, OperatorTurn("hi", "hello"))
    for _ in range(10):
        c.step(mission.world, mission.executor, mission.tracker, mission.events,
               mission.hivemind, bus=mission.bus)
    assert len(seen) == 1, "the caption topic is not a 10 Hz feed"


def test_a_model_failure_leaves_the_mission_running(mission):
    class Broken(FakeOmni):
        def understand(self, *a, **kw):
            raise TimeoutError("venue wifi")

    c = OperatorConsole(mission.world, Broken(None), mic=FakeMic(), speaker=FakeSpeaker())
    c.start()
    c.mic.say()
    pump(mission, c)
    assert c.stats["errored"] == 1 and c.phase == IDLE
    mission.tick()                                  # the swarm carries on regardless


# --------------------------------------------------------------- the microphone

def test_wav_bytes_round_trips_as_mono_16k():
    import io
    import wave

    pcm = (np.sin(np.arange(RATE) / 20) * 3000).astype(np.int16)
    with wave.open(io.BytesIO(wav_bytes(pcm)), "rb") as w:
        assert (w.getnchannels(), w.getsampwidth(), w.getframerate()) == (1, 2, RATE)
        assert w.getnframes() == RATE


def test_the_gate_calibrates_to_the_room_before_it_opens():
    """The energy-gated mode, for a setup with no dashboard to press a key on."""
    mic = Microphone(push_to_talk=False)
    assert not mic.calibrated
    quiet = np.full(480, 40, np.int16)
    for _ in range(int(RATE / 480) + 2):
        mic._on_audio(quiet.reshape(-1, 1), 480, None, None)
    assert mic.calibrated, "the gate never finished measuring the noise floor"
    assert mic.take() is None, "ambient noise was mistaken for speech"


def test_speech_is_cut_into_an_utterance_and_silence_ends_it():
    mic = Microphone(push_to_talk=False)
    quiet = np.full(480, 40, np.int16)
    loud = (np.random.default_rng(0).normal(0, 6000, 480)).astype(np.int16)
    for _ in range(int(RATE / 480) + 2):
        mic._on_audio(quiet.reshape(-1, 1), 480, None, None)
    for _ in range(int(RATE * (MIN_UTTERANCE_S + 0.3) / 480)):
        mic._on_audio(loud.reshape(-1, 1), 480, None, None)
    assert mic.hot, "the gate did not open on speech"
    for _ in range(int(RATE / 480)):
        mic._on_audio(quiet.reshape(-1, 1), 480, None, None)
    assert not mic.hot
    out = mic.take()
    assert out and out[:4] == b"RIFF"


# --------------------------------------------------------------- push to talk

def blocks(mic, n, level=6000):
    """Feed `n` blocks of audio straight into the PortAudio callback."""
    rng = np.random.default_rng(0)
    for _ in range(n):
        block = rng.normal(0, level, 480).astype(np.int16)
        mic._on_audio(block.reshape(-1, 1), 480, None, None)


def test_push_to_talk_is_the_default_and_records_nothing_unheld():
    """The whole point: a loud room is not an utterance."""
    mic = Microphone()
    assert mic.push_to_talk
    assert mic.calibrated, "push-to-talk should need no noise-floor calibration"
    blocks(mic, int(RATE * 2 / 480))
    assert not mic.hot, "the microphone opened without the key"
    assert mic.take() is None, "audio was captured with the key up"


def test_the_key_cuts_the_utterance():
    mic = Microphone()
    mic.press()
    assert mic.hot
    blocks(mic, int(RATE * (MIN_UTTERANCE_S + 0.3) / 480))
    assert mic.take() is None, "the utterance was sent before the key came up"
    mic.release()
    assert not mic.hot
    out = mic.take()
    assert out and out[:4] == b"RIFF"


def test_a_tap_is_not_an_order():
    """MIN_UTTERANCE_S still guards: a bumped key must not spend a model call."""
    mic = Microphone()
    mic.press()
    blocks(mic, 2)                                   # ~60 ms
    mic.release()
    assert mic.take() is None


def test_the_first_syllable_survives_the_key():
    """People start talking as they press, so pre-roll runs with the key up."""
    mic = Microphone()
    blocks(mic, 10)                                  # spoken just before the press
    mic.press()
    blocks(mic, int(RATE * MIN_UTTERANCE_S / 480))
    mic.release()
    out = mic.take()
    assert out is not None
    # Longer than what arrived after the press alone: the pre-roll was kept.
    assert len(out) > int(RATE * MIN_UTTERANCE_S) * 2 * 0.9


def test_an_unrenewed_key_lapses_so_a_crashed_dashboard_closes_the_mic():
    mic = Microphone()
    mic.press()
    blocks(mic, int(RATE * (MIN_UTTERANCE_S + 0.2) / 480))
    mic.expire()
    assert mic.hot, "the key lapsed while it was still being renewed"
    mic._gate_t -= KEY_TTL_S + 0.1                   # the dashboard stopped talking
    mic.expire()
    assert not mic.hot, "an unrenewed key never lapsed"
    assert mic.take() is not None, "the deadman dropped what had been said"


def test_a_renewed_key_does_not_restart_the_utterance():
    mic = Microphone()
    mic.press()
    blocks(mic, 10)
    mic.press()                                      # the 10 Hz keep-alive
    blocks(mic, int(RATE * MIN_UTTERANCE_S / 480))
    mic.release()
    assert mic.take() is not None, "a keep-alive truncated the utterance"


def test_the_key_does_nothing_in_energy_mode():
    mic = Microphone(push_to_talk=False)
    mic.press()
    assert not mic.hot


def test_only_the_newest_utterance_survives_a_backlog():
    """Somebody who spoke twice while a turn was in flight meant the second thing."""
    mic = Microphone()
    mic._utterances.put(b"first")
    mic._utterances.put(b"second")
    assert mic.take() == b"second"
    assert mic.take() is None


# --------------------------------------------------------------- the dashboard

def test_native_voice_key_and_captions(tmp_path):
    """The captions and the U key, in real Godot rather than as parsed text.

    `test_bridge_protocol.py` can only read `hud.gd` and `voice_key.gd` as source. This
    runs them: the caption card is built by the real builder and its `Label`s are
    asserted, and the key's wire message is taken off a real `send`.
    """
    import shutil
    import subprocess
    from pathlib import Path

    native = shutil.which("godot") or "/Applications/Godot.app/Contents/MacOS/Godot"
    if not Path(native).is_file():
        pytest.skip("Godot unavailable; the wire format is covered by test_bridge_protocol")
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        [native, "--headless", "--path", str(root / "godot"),
         "--log-file", str(tmp_path / "godot.log"), "--script", "res://tests/voice_check.gd"],
        text=True, capture_output=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "VOICE_CHECK_OK" in result.stdout, result.stdout + result.stderr
    assert "SCRIPT ERROR" not in result.stderr, result.stderr

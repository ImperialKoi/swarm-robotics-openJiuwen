extends SceneTree
##
## The operator voice channel, in real Godot: the U key's wire messages and the caption
## card the HUD draws under the map.
##
## `tests/test_bridge_protocol.py` can only read these files as text. This runs them --
## the key is a real `InputEventKey` through the real dashboard, and the captions are
## real `Label`s whose text is asserted after the real builder has laid them out.

class Fixture extends SwarmDashboard:
	#: Every `send` the dashboard made, so the key can be checked on the wire.
	var sent: Array = []

	func _connect_ws() -> void:
		pass  # real UI, no socket

	func send(msg: Dictionary) -> bool:
		sent.append(msg)
		return true

	func link_up() -> bool:
		return true


func _initialize() -> void:
	call_deferred("run")


func run() -> void:
	var dashboard := Fixture.new()
	root.add_child(dashboard)
	dashboard.set_process(false)
	dashboard.gw = 40
	dashboard.gh = 40
	dashboard.cell = 1.0
	dashboard.occ.resize(1600)
	dashboard.occ.fill(0)
	dashboard.robots = [[4.5, 10.5, 0.0, 0, 0, 100, 0, 0]]
	dashboard.robot_ids = ["scout-0"]
	dashboard.ready_world = true

	# ---------------------------------------------------------------- the key
	var key := dashboard.voice_key
	assert(key != null, "main.gd never built the VoiceKey")
	assert(not key.talking())
	assert(dashboard.sent.is_empty())

	# `_process` polls the physical key, which a headless run cannot press, so drive
	# the send path directly and assert the message the bridge is written to parse.
	key._send(true)
	assert(dashboard.sent.size() == 1)
	assert(dashboard.sent[0] == {"t": "mic", "on": true}, str(dashboard.sent[0]))
	key._send(false)
	assert(dashboard.sent[1] == {"t": "mic", "on": false}, str(dashboard.sent[1]))

	# U must be claimed by this script and by nothing else, in every view.
	var u := InputEventKey.new()
	u.physical_keycode = KEY_U
	u.pressed = true
	var before := dashboard.view_mode
	dashboard._unhandled_input(u)
	assert(dashboard.view_mode == before, "U also changed the view")

	# ---------------------------------------------------------- the product name
	# Neither bundled face has 蚁 or 群 (both Latin, `has_char` false), so without the
	# SystemFont fallback in hud.gd `_font` the title draws as two tofu boxes. Assert
	# the composed font resolves them rather than trusting the fallback list.
	var title_font := dashboard.ui._font(SwarmHud.DISP_BOLD, 0)
	assert(title_font.has_char("蚁".unicode_at(0)), "蚁 has no glyph: the title is tofu")
	assert(title_font.has_char("群".unicode_at(0)), "群 has no glyph: the title is tofu")
	var mono := dashboard.ui._font(SwarmHud.MONO, 0)
	assert(mono.has_char("蚁".unicode_at(0)), "蚁 has no glyph in the voice badge face")
	assert(SwarmHud.PHASE_LABEL["speaking"].contains("蚁群"),
		"the voice badge still says SWARMMIND")

	# ---------------------------------------------------------------- captions
	var ui := dashboard.ui
	assert(ui._cap_root != null, "the caption card was never built")
	assert(not ui._cap_root.visible, "the caption card is up before anybody spoke")

	# The card stays up once the channel has spoken: it is the operator's only signal
	# that the microphone is alive, and an absent card cannot say "the mic is off".
	ui.on_voice({"phase": "idle", "said": "", "reply": "", "sectors": [], "rejected": []})
	assert(ui._cap_root.visible, "the mic signal vanished when idle")
	assert(not ui._cap_card.visible, "an empty subtitle plate sat over the map")
	assert(not ui._cap_said.visible and not ui._cap_reply.visible,
		"empty captions drew a line")

	# --- the recording meter --------------------------------------------------------
	# Grey and near-flat when the key is up, whatever the room is doing.
	ui.on_voice({"phase": "idle", "said": "", "reply": "", "sectors": [], "rejected": [],
		"recording": false, "level": 0.9})
	assert(not ui._mic_recording, "the meter claimed to be recording with the key up")
	assert(ui._mic_bars[ui._mic_bars.size() - 1] <= 0.121,
		"room noise drove the meter while the key was up")

	# Red and following the operator's own level while it is held.
	for lv in [0.2, 0.7, 0.35, 0.9]:
		ui.on_voice({"phase": "listening", "said": "", "reply": "", "sectors": [],
			"rejected": [], "recording": true, "level": lv})
	assert(ui._mic_recording, "the meter did not go live while recording")
	# `is_equal_approx`, not `==`: _mic_bars is a PackedFloat32Array, and 0.9 stored as
	# float32 is 0.899999976 against GDScript's float64 literal.
	var bars := ui._mic_bars
	assert(is_equal_approx(bars[bars.size() - 1], 0.9),
		"the meter is not tracking the microphone level")
	assert(is_equal_approx(bars[bars.size() - 3], 0.7), "the meter history is out of order")
	assert(not is_equal_approx(bars[bars.size() - 1], bars[bars.size() - 2]),
		"the meter is not fluctuating")
	assert(bars.size() <= SwarmHud.METER_BARS, "the level history grew without bound")
	assert(SwarmHud.MIC_LIVE.r > SwarmHud.MIC_IDLE.r, "recording is not the redder state")

	# A dead microphone says so, in the warning colour, with its reason.
	ui.on_voice({"phase": "muted", "said": "", "reply": "no microphone available",
		"sectors": [], "rejected": []})
	assert(ui._cap_root.visible and ui._cap_card.visible)
	assert(not ui._mic_recording, "a dead microphone showed as recording")
	assert(ui._cap_reply.visible and ui._cap_reply.text == "no microphone available")

	# Holding U shows the badge before any text exists -- the operator needs to see
	# that the channel heard them open their mouth.
	ui.on_voice({"phase": "listening", "said": "", "reply": "", "sectors": [], "rejected": []})
	assert(ui._cap_root.visible, "LISTENING drew nothing")
	assert(ui._cap_phase.text == SwarmHud.PHASE_LABEL["listening"])
	assert(not ui._cap_said.visible and not ui._cap_reply.visible)

	# A full turn.
	ui.on_voice({"phase": "speaking", "said": "Pull everyone out of D4.",
		"reply": "Clearing D4 now.", "sectors": ["D4", "E4"], "rejected": [],
		"latency_ms": 1870})
	assert(ui._cap_root.visible and ui._cap_card.visible)
	assert(ui._cap_said.visible and "Pull everyone out of D4." in ui._cap_said.text)
	assert(ui._cap_reply.visible and ui._cap_reply.text == "Clearing D4 now.")
	assert(ui._cap_tags.visible and "D4" in ui._cap_tags.text and "E4" in ui._cap_tags.text)
	assert("RETASKED" in ui._cap_tags.text)
	assert(ui._cap_meta.text == "1870 ms")

	# A refusal is the demo's best moment; it has to be legible and coloured to notice.
	ui.on_voice({"phase": "idle", "said": "Abandon the whole map.",
		"reply": "I could not carry that out.", "sectors": [], "rejected": ["A7", "A8"],
		"latency_ms": 2390})
	assert("REFUSED" in ui._cap_tags.text)
	assert("A7" in ui._cap_tags.text and "A8" in ui._cap_tags.text)
	assert(ui._cap_tags.get_theme_color("font_color") == SwarmHud.C_WARN,
		"a refused order is not coloured to be noticed")

	# Every phase the simulator can send must produce a label and a colour.
	for phase in ["idle", "listening", "thinking", "speaking", "muted"]:
		assert(SwarmHud.PHASE_LABEL.has(phase), "no label for phase " + phase)
		assert(SwarmHud.PHASE_COLOUR.has(phase), "no colour for phase " + phase)
		ui.on_voice({"phase": phase, "said": "x", "reply": "y",
			"sectors": [], "rejected": []})
		assert(ui._cap_phase.text == SwarmHud.PHASE_LABEL[phase])

	# The caption text clears when the turn ages out; the badge itself stays.
	ui.on_voice({"phase": "idle", "said": "", "reply": "", "sectors": [], "rejected": []})
	assert(not ui._cap_said.visible and not ui._cap_reply.visible, "captions never cleared")
	assert(not ui._cap_card.visible, "the black plate outlived its text")
	assert(ui._cap_root.visible, "the mic signal cleared with the captions")

	# Nothing in the block may eat a click meant for a robot behind it.
	assert(ui._cap_root.mouse_filter == Control.MOUSE_FILTER_IGNORE)
	assert(ui._cap_card.mouse_filter == Control.MOUSE_FILTER_IGNORE)
	assert(ui._cap_meter.mouse_filter == Control.MOUSE_FILTER_IGNORE)

	# Subtitles, not a HUD panel: white text on a near-solid black plate.
	assert(ui._cap_said.get_theme_color("font_color") == Color.WHITE,
		"the operator's words are not white")
	var plate := ui._cap_card.get_theme_stylebox("panel") as StyleBoxFlat
	assert(plate.bg_color.a > 0.6 and plate.bg_color.r < 0.1,
		"the subtitle plate is not black")

	# A frame from an older simulator, with fields missing, must not crash the dashboard.
	ui.on_voice({"phase": "speaking"})
	ui.on_voice({})

	dashboard.queue_free()
	await process_frame
	print("VOICE_CHECK_OK")
	quit()

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

	# ---------------------------------------------------------------- captions
	var ui := dashboard.ui
	assert(ui._cap_root != null, "the caption card was never built")
	assert(not ui._cap_root.visible, "the caption card is up before anybody spoke")

	# Idle with nothing said stays hidden: the hint must not sit over the map all run.
	ui.on_voice({"phase": "idle", "said": "", "reply": "", "sectors": [], "rejected": []})
	assert(not ui._cap_root.visible)

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
	assert(ui._cap_root.visible)
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

	# The caption clears when the simulator says the turn has aged out.
	ui.on_voice({"phase": "idle", "said": "", "reply": "", "sectors": [], "rejected": []})
	assert(not ui._cap_root.visible, "the caption never cleared")

	# The card must not eat a click meant for a robot behind it.
	assert(ui._cap_root.mouse_filter == Control.MOUSE_FILTER_IGNORE)
	assert(ui._cap_card.mouse_filter == Control.MOUSE_FILTER_IGNORE)

	# A frame from an older simulator, with fields missing, must not crash the dashboard.
	ui.on_voice({"phase": "speaking"})
	ui.on_voice({})

	dashboard.queue_free()
	await process_frame
	print("VOICE_CHECK_OK")
	quit()

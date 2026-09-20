class_name VoiceKey
extends Node
##
## Push to talk: hold U to speak to the swarm, release to send.
##
## A demo floor is loud. An always-open microphone there hears the next table, the
## operator explaining the project to a judge, and the assistant's own reply coming back
## out of the laptop speaker -- and every one of those is a model call that gets paid for
## and answered. Holding a key makes the start and end of an utterance a decision.
##
## **This script only asks.** It reports the key state to the simulator, which owns the
## microphone (swarmmind/voice/mic.py) and decides what to record. The dashboard has no
## audio path of its own in either direction: the reply is played by the process that
## generated it, so nothing competes with the state frames for the socket.
##
## **The held key is a deadman, exactly like a drive key.** It is re-sent at `SEND_EVERY`
## while U is down, and the simulator treats a press older than `KEY_TTL_S` (0.4 s) as a
## release -- so a dashboard that crashes or a socket that drops closes the microphone
## rather than leaving it recording the room. `control/manual.py` holds a driven robot to
## the same rule for the same reason.
##
## U is deliberately not a drive key and needs no unit: the operator is talking to the
## whole swarm, so it works in every view including the orbit. The badge that shows
## LISTENING is drawn by hud.gd from the `voice` frames the simulator sends back, not
## from this key -- so it reports what is actually being recorded rather than what is
## being pressed.

#: Physical, so it sits under the same finger on any keyboard layout.
const KEY_TALK := KEY_U

#: Keep-alive while the key is held. The simulator's `KEY_TTL_S` is 0.4 s, so this leaves
#: room for two lost frames.
const SEND_EVERY := 0.1

var host: SwarmDashboard
var _down := false
var _since_send := 0.0


func setup(dashboard: SwarmDashboard) -> void:
	host = dashboard


func talking() -> bool:
	"""Whether this dashboard currently believes it is holding the key open."""
	return _down


func _input(event: InputEvent) -> void:
	# Claim U before the GUI and before main.gd's `_unhandled_input`, so it can never
	# also do a second job later. Nothing else binds it today; this keeps that true.
	if event is InputEventKey and event.physical_keycode == KEY_TALK:
		get_viewport().set_input_as_handled()


func _process(delta: float) -> void:
	if host == null:
		return
	var down := Input.is_physical_key_pressed(KEY_TALK) and host.link_up()
	_since_send += delta
	if down and not _down:
		_send(true)
		host._log("[color=#ffbf3d]listening -- hold U[/color]")
	elif not down and _down:
		_send(false)
	elif down and _since_send >= SEND_EVERY:
		_send(true)                      # renew, so the deadman does not lapse
	_down = down


func _send(on: bool) -> void:
	_since_send = 0.0
	host.send({"t": "mic", "on": on})

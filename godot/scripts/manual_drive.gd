class_name ManualDrive
extends Node
##
## Manual override: WASD drives the followed unit, on top of its autonomy, and space acts.
##
## Layered on the swarm, not in place of it. Commands go to the simulator, which puts them
## in place of that one robot's goal-seeking *before* the Tier 1 wall override
## (swarmmind/control/manual.py) -- an operator can no more drive a robot into a wall than
## the auction can. The autonomy keeps running underneath: the robot keeps its task, and
## the moment the override lapses it steers for that task again from wherever it was left.
##
## **The simulator decides who is driving, one robot at a time**, so a handoff has no gap
## and no fight: on every tick exactly one source commands each robot. This script only
## asks. It sends `drive` while a key is held, a stop when the last key comes up, and
## `release` the moment the followed unit changes. The simulator holds a stopped unit for
## 1.5 s after the last command and then hands it back on its own, so there is no "give
## control back" key -- and a dashboard that crashes mid-press loses its robot the same way.
##
## Driving needs a unit view, POV or chase. In the orbit, S is still the sector toggle.
##
## **Space is the action key, and the simulator decides what it does.** Whether there is a
## casualty within reach is a ground-truth question, so this script may not answer it and
## must not guess: a press sends `act` naming only the unit, and `World.operator_act`
## chooses between picking a casualty up, setting one down, and taking a rotor off or
## landing it. The same call fills in `manual[2]` of every state frame, which is what the
## badge offers -- so the label and the key cannot come apart.
##
## The badge that says who has the unit is drawn by hud.gd from `mode()` and `action()`,
## next to the rest of the viewport frame's readouts, together with a CARRYING badge
## whenever the simulator says the unit is holding a casualty.

#: Physical keys, so the diamond sits under the same fingers on any keyboard layout.
const KEYS_FWD := [KEY_W, KEY_UP]
const KEYS_BACK := [KEY_S, KEY_DOWN]
const KEYS_LEFT := [KEY_A, KEY_LEFT]
const KEYS_RIGHT := [KEY_D, KEY_RIGHT]
#: One press, one action. Polled like the rest, not bound in `_input`, so it can only
#: fire on a unit this script is actually allowed to drive.
const KEY_ACT := KEY_SPACE
#: The arrow keys among the four above.
#:
#: **On a drone, left/right turn the aircraft and up/down aim the camera.** They used to
#: do neither -- all four went to the camera (main.gd `_camera_keys`) -- which left the
#: drone with no way to be pointed except A/D, and made WASD read as unrelated to what
#: was on screen. Turning the aircraft instead keeps the chase camera behind its heading
#: (`chase_yaw` is an offset from behind, not a compass bearing), so **W is always
#: "forward into the shot"**: the drive keys become camera-relative by construction
#: rather than by a second frame of reference nobody can see.
#:
#: Up/down stay with the camera because pitch has no equivalent on a 2.5D aircraft --
#: the simulation is flat and `world.height` is render-only.
const ARROW_KEYS := [KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT]
#: The arrows a drone's camera keeps. Left/right go to the aircraft.
const CAMERA_ARROWS := [KEY_UP, KEY_DOWN]

#: Fraction of the unit's own turn rate that A/D ask for. Full rate is 2.0-3.5 rad/s: a
#: scout at full rate turns 200 degrees a second, which from its own eye is a blur.
const TURN := 0.5
#: Keep-alive while a key is held. The simulator treats a command older than 0.3 s as a
#: stop (`CMD_TTL_S` in control/manual.py), so this leaves room for two lost frames.
const SEND_EVERY := 0.1
#: How long a held key may go unconfirmed before the badge says so. The first echo takes
#: one state frame (100 ms); a sim that never confirms is an older build or a replay, and
#: the badge must not claim control the simulator is not giving.
const ACK_WAIT := 0.5

enum Mode { NONE, AUTO, MANUAL, HOLDING, NO_ACK }

var host: SwarmDashboard
#: The robot index commands currently go to, or -1.
var _bound := -1
var _held := false
var _v := 0.0
var _w := 0.0
var _since_send := 0.0
var _unacked := 0.0
#: True while the action key is down, so one press is one action rather than sixty.
var _acting := false
#: The simulator's `manual` field from the last state frame: `[robot index, seconds until
#: autonomy resumes, action code]`, or empty when nobody is being driven.
var _echo: Array = []


func setup(dashboard: SwarmDashboard) -> void:
	host = dashboard


func target() -> int:
	"""The robot the keys drive: the followed unit in POV or chase, still working, and
	reachable -- or -1."""
	if host == null or host.view_mode == SwarmDashboard.View.ORBIT or not host._on_unit():
		return -1
	if not host.link_up():
		return -1                 # a frozen last frame is not a robot anyone can reach
	if host.follow >= host.robot_ids.size():
		return -1
	var r: Array = host.robots[host.follow]
	if int(r[4]) >= 2:
		return -1                 # failed or destroyed: there is nothing to drive
	return host.follow


func driven() -> int:
	"""The robot the simulator is taking manual commands for, or -1."""
	return int(_echo[0]) if _echo.size() >= 2 else -1


func hold_left() -> float:
	"""Seconds until the simulator hands the driven unit back to its autonomy."""
	return float(_echo[1]) if _echo.size() >= 2 else 0.0


func action() -> int:
	"""What the simulator says the action key would do to the driven unit right now: an
	`OPERATOR_ACTION` code from swarmmind/contracts/schemas.py, 0 for nothing.

	Its word and not ours. The dashboard cannot see a casualty until the swarm reports
	one, and offering a key that turns out to do nothing is worse than offering none.
	"""
	return int(_echo[2]) if _echo.size() >= 3 else 0


func carrying() -> String:
	"""The casualty the driven unit is holding, or "" -- the simulator's word.

	A fourth `manual` field, added because SET DOWN only *implied* a load. An older
	simulator sends three fields and this reads empty, which draws no badge.
	"""
	return str(_echo[3]) if _echo.size() >= 4 else ""


func camera_owns_arrows() -> bool:
	"""Whether the camera owns *up and down* this frame. Left/right are always the drive's.

	Asked of the host, not worked out here: `main.gd` owns the followed unit and reads
	its chassis off the state frame.
	"""
	return host != null and host.followed_is_drone()


func mode() -> int:
	"""What the badge shows for the followed unit.

	Taken from the simulator's echo whenever it has one, so the badge reports what the robot
	is actually obeying rather than which keys are down. The one exception is the first
	state frame after a key goes down, which shows MANUAL immediately rather than flashing
	AUTO for 100 ms; `NO_ACK` catches a simulator that never confirms.
	"""
	if _bound < 0:
		return Mode.NONE
	if driven() == _bound:
		return Mode.MANUAL if _held else Mode.HOLDING
	if _held:
		return Mode.NO_ACK if _unacked >= ACK_WAIT else Mode.MANUAL
	return Mode.AUTO


func on_state(echo: Array) -> void:
	"""The `manual` field of a state frame. Logs each time control changes hands."""
	var was := driven()
	_echo = echo
	var now := driven()
	if now == was:
		return
	if was >= 0 and was < host.robot_ids.size():
		host._log("%s back under autonomy" % str(host.robot_ids[was]).to_upper())
	if now >= 0 and now < host.robot_ids.size():
		host._log("[color=#ffbf3d]operator driving %s[/color]" % str(host.robot_ids[now]).to_upper())


func _input(event: InputEvent) -> void:
	# `_input` runs before the GUI and before every `_unhandled_input`, so this takes the
	# drive keys ahead of main.gd's S (sectors) by engine order rather than tree order --
	# but only while there is a unit to drive. The key state itself is polled in
	# `_process`; this only stops the key doing a second job.
	if event is InputEventKey and _is_drive_key(event.physical_keycode) and target() >= 0:
		get_viewport().set_input_as_handled()


func _process(delta: float) -> void:
	var want := target()
	if want != _bound:
		# Hand the old unit back in the same frame the view leaves it. The simulator reads
		# both in arrival order, so the old unit's autonomy resumes on the tick the new
		# unit's first command lands on.
		if _bound >= 0 and _bound < host.robot_ids.size():
			host.send({"t": "release", "robot": host.robot_ids[_bound]})
		_bound = want
		_held = false
		_unacked = 0.0
		# A key already down when the view arrives is not a press on the new unit. Without
		# this, clicking through a line of carriers with space held would empty every one
		# of them onto the ground on the way past.
		_acting = Input.is_physical_key_pressed(KEY_ACT)
	if _bound < 0:
		return
	_act()
	# A positive turn rate swings the heading from sim +x toward sim +y, and sim +y is
	# Godot +z -- the right-hand side of a unit facing +x (world_to_godot). So positive is
	# a right turn on screen, and D sends it. Worked from `_update_camera`'s POV basis.
	var fwd := _axis(KEYS_FWD, KEYS_BACK)
	var turn := _axis(KEYS_RIGHT, KEYS_LEFT) * TURN
	var held := _any(KEYS_FWD) or _any(KEYS_BACK) or _any(KEYS_LEFT) or _any(KEYS_RIGHT)
	_since_send += delta
	if held:
		if not _held or fwd != _v or turn != _w or _since_send >= SEND_EVERY:
			_drive(fwd, turn)
		_unacked = 0.0 if driven() == _bound else _unacked + delta
	elif _held:
		# Stop now rather than coasting on the last command until it goes stale. The
		# simulator then holds the unit still and returns it to autonomy by itself.
		_drive(0.0, 0.0)
	_held = held


func _act() -> void:
	"""Send one `act` on the frame the key goes down, and nothing while it is held.

	No keep-alive and no stop: unlike a drive this is an event, not a state, and the
	simulator renews the hold on it by itself so the swarm does not reclaim the unit
	between an operator's presses.
	"""
	var down := Input.is_physical_key_pressed(KEY_ACT)
	if down and not _acting:
		host.send({"t": "act", "robot": host.robot_ids[_bound]})
	_acting = down


func _drive(v: float, w: float) -> void:
	_v = v
	_w = w
	_since_send = 0.0
	host.send({"t": "drive", "robot": host.robot_ids[_bound], "v": v, "w": w})


func _is_drive_key(k: int) -> bool:
	if k == KEY_ACT:
		return true
	if k in CAMERA_ARROWS and camera_owns_arrows():
		return false
	return k in KEYS_FWD or k in KEYS_BACK or k in KEYS_LEFT or k in KEYS_RIGHT


func _any(keys: Array) -> bool:
	# Only up/down are ever withheld, and only on a drone. Left/right turn the aircraft
	# on every chassis, which is what makes W mean "forward into the shot".
	var tilt_taken := camera_owns_arrows()
	for k in keys:
		if tilt_taken and k in CAMERA_ARROWS:
			continue
		if Input.is_physical_key_pressed(k):
			return true
	return false


func _axis(plus: Array, minus: Array) -> float:
	return (1.0 if _any(plus) else 0.0) - (1.0 if _any(minus) else 0.0)

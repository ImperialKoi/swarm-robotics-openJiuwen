extends SceneTree
## Drive the manual override the way an operator does: real key state, the real input
## order against main.gd's S binding, and the real HUD draw path -- against a recorded
## socket. The simulator's half is tests/test_manual_override.py.

class Fixture extends SwarmDashboard:
	var sent: Array = []

	func _connect_ws() -> void:
		pass

	func link_up() -> bool:
		return true

	func send(msg: Dictionary) -> bool:
		sent.append(msg)
		return true


class Probe extends Control:
	var drawn := false

	func _draw() -> void:
		drawn = true


func _initialize() -> void:
	call_deferred("run")


func _event(k: Key, down: bool) -> InputEventKey:
	var ev := InputEventKey.new()
	ev.physical_keycode = k
	ev.keycode = k
	ev.pressed = down
	return ev


func _key(k: Key, down: bool) -> void:
	Input.parse_input_event(_event(k, down))
	Input.flush_buffered_events()


func _draw_hud(d: Fixture) -> void:
	d.ui._view_draw.queue_redraw()
	await process_frame


func run() -> void:
	root.size = Vector2i(1280, 720)
	var d := Fixture.new()
	root.add_child(d)
	d.set_process(false)
	var drive: ManualDrive = d.drive
	drive.set_process(false)
	# r00/r01 are wheeled (chassis 0), r02 is destroyed, r03 is a rotor (chassis 3) --
	# the one chassis the arrow keys belong to the camera on.
	d.robot_ids = ["r00", "r01", "r02", "r03"]
	d.robots = [[10.0, 10.0, 0.0, 0, 0, 1.0, 0, 2],
		[20.0, 10.0, 0.0, 0, 0, 1.0, 0, 2],
		[30.0, 10.0, 0.0, 0, 3, 0.0, 0, 0],
		[40.0, 10.0, 0.0, 0, 0, 1.0, 3, 2]]
	d.follow = 0
	await process_frame

	# Orbit: nothing to drive, and S is still the sector toggle.
	d.view_mode = SwarmDashboard.View.ORBIT
	assert(drive.target() == -1)
	var sectors := d.show_sectors
	root.push_input(_event(KEY_S, true))
	root.push_input(_event(KEY_S, false))
	assert(d.show_sectors != sectors)
	sectors = d.show_sectors

	# POV on a live unit: S belongs to the drive, and does not flip the sectors.
	d.view_mode = SwarmDashboard.View.POV
	assert(drive.target() == 0)
	root.push_input(_event(KEY_S, true))
	root.push_input(_event(KEY_S, false))
	assert(d.show_sectors == sectors)

	drive._process(0.016)
	assert(d.sent.is_empty())
	assert(drive.mode() == ManualDrive.Mode.AUTO)

	# W drives; MANUAL at once, before the first echo lands.
	_key(KEY_W, true)
	drive._process(0.016)
	assert(d.sent.back() == {"t": "drive", "robot": "r00", "v": 1.0, "w": 0.0})
	assert(drive.mode() == ManualDrive.Mode.MANUAL)
	drive.on_state([0, 1.5])
	assert(drive.mode() == ManualDrive.Mode.MANUAL)
	# D is a positive turn rate: a right turn on screen.
	_key(KEY_D, true)
	drive._process(0.016)
	assert(d.sent.back() == {"t": "drive", "robot": "r00", "v": 1.0, "w": ManualDrive.TURN})
	# Unchanged keys re-send only at the keep-alive rate.
	var n := d.sent.size()
	drive._process(0.05)
	assert(d.sent.size() == n)
	drive._process(0.06)
	assert(d.sent.size() == n + 1)
	d.ready_world = true
	await _draw_hud(d)

	# Keys up: one stop, then silence -- the simulator hands the unit back by itself.
	_key(KEY_W, false)
	_key(KEY_D, false)
	drive._process(0.016)
	assert(d.sent.back() == {"t": "drive", "robot": "r00", "v": 0.0, "w": 0.0})
	assert(drive.mode() == ManualDrive.Mode.HOLDING)
	await _draw_hud(d)
	n = d.sent.size()
	drive._process(0.5)
	assert(d.sent.size() == n)
	drive.on_state([])
	assert(drive.mode() == ManualDrive.Mode.AUTO)
	await _draw_hud(d)

	# The arrow keys are the same controls -- on a ground unit.
	assert(not d.followed_is_drone())
	_key(KEY_DOWN, true)
	_key(KEY_LEFT, true)
	drive._process(0.016)
	assert(d.sent.back() == {"t": "drive", "robot": "r00", "v": -1.0, "w": -ManualDrive.TURN})
	_key(KEY_DOWN, false)
	_key(KEY_LEFT, false)
	drive._process(0.016)

	# Space is one action per press: one `act` on the way down, nothing while held.
	var n_act := d.sent.size()
	_key(KEY_SPACE, true)
	drive._process(0.016)
	assert(d.sent.back() == {"t": "act", "robot": "r00"})
	assert(d.sent.size() == n_act + 1)
	drive._process(0.2)
	assert(d.sent.size() == n_act + 1)
	_key(KEY_SPACE, false)
	drive._process(0.016)
	assert(d.sent.size() == n_act + 1)

	# What the badge offers comes from the simulator's echo and nowhere else.
	assert(drive.action() == 0)
	drive.on_state([0, 1.5, 1])
	assert(drive.action() == 1 and drive.driven() == 0 and drive.hold_left() == 1.5)
	await _draw_hud(d)
	drive.on_state([0, 1.5])
	assert(drive.action() == 0)          # an older simulator offers nothing, not garbage

	# On a drone the arrow keys are the camera's, and the drive must not read them.
	d.follow = 3
	drive._process(0.016)
	drive.on_state([3, 1.5, 3])
	assert(d.followed_is_drone() and drive.camera_owns_arrows())
	n_act = d.sent.size()
	_key(KEY_LEFT, true)
	_key(KEY_UP, true)
	drive._process(0.016)
	assert(d.sent.size() == n_act, "the arrow keys drove a drone")
	assert(drive.mode() == ManualDrive.Mode.HOLDING)
	var aim := d.pov_look
	d._camera_keys(0.1)
	assert(d.pov_look.x < aim.x and d.pov_look.y > aim.y, "the arrows did not aim the camera")
	# ...while WASD still flies it.
	_key(KEY_W, true)
	drive._process(0.016)
	assert(d.sent.back() == {"t": "drive", "robot": "r03", "v": 1.0, "w": 0.0})
	_key(KEY_W, false)
	_key(KEY_LEFT, false)
	_key(KEY_UP, false)
	drive._process(0.016)

	# The chase rig takes the same keys, and the aim is dropped with the unit.
	d.view_mode = SwarmDashboard.View.CHASE
	var yaw := d.chase_yaw
	_key(KEY_RIGHT, true)
	d._camera_keys(0.1)
	assert(d.chase_yaw > yaw)
	_key(KEY_RIGHT, false)
	d.view_mode = SwarmDashboard.View.POV
	d.follow = 0
	# What `_update_camera` does on the first frame of a new unit. Called directly: the
	# camera itself wants a terrain surface, and this check has no world.
	d._pov_bind(d.follow)
	assert(d.pov_look == Vector2.ZERO, "a ground unit inherited a drone's camera angle")
	drive._process(0.016)
	drive.on_state([0, 1.5, 0])

	# Switching units with a key held: release the old one, drive the new one, one frame.
	_key(KEY_W, true)
	drive._process(0.016)
	d.follow = 1
	drive._process(0.016)
	assert(d.sent[-2] == {"t": "release", "robot": "r00"})
	assert(d.sent[-1] == {"t": "drive", "robot": "r01", "v": 1.0, "w": 0.0})
	drive.on_state([1, 1.5])

	# Leaving the unit view hands it straight back.
	d.view_mode = SwarmDashboard.View.ORBIT
	drive._process(0.016)
	assert(d.sent.back() == {"t": "release", "robot": "r01"})
	assert(drive.mode() == ManualDrive.Mode.NONE)
	_key(KEY_W, false)
	drive._process(0.016)

	# A destroyed unit is not drivable, and offers no action either.
	d.view_mode = SwarmDashboard.View.CHASE
	d.follow = 2
	assert(drive.target() == -1)
	assert(not d.followed_is_drone())

	# A simulator that never confirms is reported, not papered over.
	d.follow = 0
	drive.on_state([])
	_key(KEY_W, true)
	drive._process(0.016)
	drive._process(ManualDrive.ACK_WAIT)
	assert(drive.mode() == ManualDrive.Mode.NO_ACK)
	await _draw_hud(d)
	_key(KEY_W, false)

	# The HUD's draw callbacks really ran headless, so the badge paths above were executed.
	var probe := Probe.new()
	root.add_child(probe)
	probe.queue_redraw()
	await process_frame
	assert(probe.drawn)
	print("MANUAL_DRIVE_OK")
	quit()

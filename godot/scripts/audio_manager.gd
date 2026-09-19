class_name SwarmAudio
extends Node3D
## Dashboard-only sound, fed by main.gd's existing truth/state/event handlers.
## The current bridge sends the hazard as truth.hz = [centre_x, centre_y, radius].
## /world/hazard_zone is declared in Python but has no publisher in this build.

const RUMBLE := preload("res://audio/sfx/soundreality-landslide-128314.mp3")
const SLIDE := preload("res://audio/sfx/floraphonic-rocks-and-gravel-slide-4-204995.mp3")
# Owner-selected step, 19 Sep: the simulator has continuous growth, no growth event.
const GROWTH_STEP_M := 5.0

var rumble: AudioStreamPlayer3D
var slide: AudioStreamPlayer3D
var host: SwarmDashboard
var _active := false
var _complete := false
var _sector := ""
var _sector_codes: Array = []
var _next_slide_radius := -1.0


func _ready() -> void:
	rumble = _player("HazardRumble", RUMBLE, true)
	slide = _player("HazardSlide", SLIDE, false)


func _player(player_name: String, stream: AudioStreamMP3, looping: bool) -> AudioStreamPlayer3D:
	var player := AudioStreamPlayer3D.new()
	player.name = player_name
	var sound := stream.duplicate() as AudioStreamMP3
	sound.loop = looping
	player.stream = sound
	# The camera is the listener; Godot handles distance attenuation and panning.
	add_child(player)
	return player


func setup(dashboard: SwarmDashboard) -> void:
	host = dashboard


func reset() -> void:
	_clear_hazard()
	_complete = false
	_sector_codes.clear()


func _clear_hazard() -> void:
	rumble.stop()
	slide.stop()
	_active = false
	_sector = ""
	_next_slide_radius = -1.0


func on_truth(zone: Variant) -> void:
	if _complete:
		return
	# The simulator retains active=true after decay reaches zero. Radius zero is
	# therefore resolved too, even though the bridge still sends a three-item row.
	if not zone is Array or zone.size() < 3 or float(zone[2]) <= 0.0:
		_clear_hazard()
		return
	_place(float(zone[0]), float(zone[1]))
	_activate()
	var radius := float(zone[2])
	if _next_slide_radius < 0.0:
		_next_slide_radius = radius + GROWTH_STEP_M
		return
	var crossed := false
	while radius > _next_slide_radius or is_equal_approx(radius, _next_slide_radius):
		_next_slide_radius += GROWTH_STEP_M
		crossed = true
	# Keep the high-water mark through decay and abandonment. Coalesce skipped
	# snapshots into one current impact instead of replaying a backlog at once.
	if crossed and not _abandoned():
		slide.play()


func on_state(codes: Array) -> void:
	_sector_codes = codes.duplicate()
	_sync_rumble()


func on_event(msg: Dictionary) -> void:
	match str(msg.get("kind", "")):
		"mission_complete":
			_clear_hazard()
			_complete = true
		"hazard_ignited":
			if _complete:
				return
			var at = msg.get("pos", null)
			if not at is Array or at.size() < 2:
				return
			_sector = str(msg.get("sector", ""))
			_place(float(at[0]), float(at[1]))
			_activate()
	# sector_abandoned also denotes reopening on directive expiry. Use the next
	# state frame's sector code (10 Hz), rather than guessing from the event text.


func _place(x: float, y: float) -> void:
	position = host.world_to_godot(x, y, host._surface_height(x, y))
	if not _sector.is_empty():
		return
	# Joining an active mission has no ignition event to replay. Locate its sector
	# from the existing hello rectangles; subsequent drift does not change ownership.
	for k in range(mini(host.sector_rects.size(), host.sector_ids.size())):
		var rect: Array = host.sector_rects[k]
		if x >= rect[0] and x < rect[2] and y >= rect[1] and y < rect[3]:
			_sector = str(host.sector_ids[k])
			break


func _abandoned() -> bool:
	var k := host.sector_ids.find(_sector)
	return k >= 0 and k < _sector_codes.size() and int(_sector_codes[k]) == 3


func _activate() -> void:
	var first := not _active
	_active = true
	_sync_rumble()
	if first and not _abandoned():
		slide.play()


func _sync_rumble() -> void:
	if not _active or _complete or _abandoned():
		rumble.stop()
		slide.stop()
	elif not rumble.playing:
		rumble.play()

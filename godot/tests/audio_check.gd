extends SceneTree
## Exercise the actual dashboard dispatcher and MP3 resources, with a dummy driver.

class Fixture extends SwarmDashboard:
	func _connect_ws() -> void:
		pass


func _initialize() -> void:
	call_deferred("run")


func run() -> void:
	var d := Fixture.new()
	root.add_child(d)
	d.set_process(false)
	d.drive.set_process(false)
	var a: SwarmAudio = d.audio
	var hello := {
		"t": "hello", "schema": "1.0", "map_m": [80, 40],
		"sectors": [{"id": "A1", "r": [0, 0, 40, 40]},
			{"id": "A2", "r": [40, 0, 80, 40]}],
	}
	d._handle(hello)
	d._handle({"t": "state", "sec": [1, 1]})
	d._handle({"t": "truth", "hz": null})
	assert(not a.rumble.playing and not a.slide.playing)
	assert((a.rumble.stream as AudioStreamMP3).loop)
	assert(not (a.slide.stream as AudioStreamMP3).loop)
	assert(absf(a.rumble.stream.get_length() - 22.032) < 0.1)
	assert(absf(a.slide.stream.get_length() - 5.256) < 0.1)
	assert(a.rumble.attenuation_model == AudioStreamPlayer3D.ATTENUATION_INVERSE_DISTANCE)
	assert(a.slide.attenuation_model == AudioStreamPlayer3D.ATTENUATION_INVERSE_DISTANCE)

	var ignition := {"t": "event", "kind": "hazard_ignited", "pos": [12, 23], "sector": "A1"}
	d._handle(ignition)
	await physics_frame
	assert(a.rumble.playing and a.slide.playing, "Impact must layer over the rumble")
	assert(a.rumble.global_position == Vector3(12, 0, 23))
	assert(a.slide.global_position == a.rumble.global_position)
	# An ignition and its following truth frame must not double-trigger either asset.
	await create_timer(0.1).timeout
	a.rumble.seek(2.0)
	a.slide.seek(2.0)
	await create_timer(0.1).timeout
	d._handle({"t": "truth", "hz": [13, 24, 6.1]})
	d._handle(ignition)
	await physics_frame
	assert(a.rumble.get_playback_position() >= 2.0, "Repeated truth restarted the rumble")
	assert(a.slide.get_playback_position() >= 2.0, "Ignition was played twice")
	d._handle({"t": "truth", "hz": [14, 25, 6.1]})
	assert(a.rumble.global_position == Vector3(14, 0, 25))
	assert(a.slide.global_position == a.rumble.global_position)
	d._handle({"t": "truth", "hz": [14, 25, 11.0]})
	assert(a.slide.get_playback_position() >= 2.0, "Impact before five metres of growth")
	d._handle({"t": "truth", "hz": [14, 25, 11.1]})
	await create_timer(0.1).timeout
	assert(a.slide.get_playback_position() < 1.0, "No impact at the five-metre boundary")
	assert(a.rumble.get_playback_position() >= 2.0, "Growth restarted the rumble")
	a.slide.seek(2.0)
	await create_timer(0.1).timeout
	for radius in [11.1, 10.0, 11.1, 16.0]:
		d._handle({"t": "truth", "hz": [14, 25, radius]})
		await physics_frame
		assert(a.slide.get_playback_position() >= 2.0, "Repeated/decaying radius retriggered")
	d._handle({"t": "truth", "hz": [14, 25, 16.1]})
	await create_timer(0.1).timeout
	assert(a.slide.get_playback_position() < 1.0, "Second growth step did not trigger")
	# A dropped truth snapshot should not leave overdue impacts for later frames.
	d._handle({"t": "truth", "hz": [14, 25, 31.1]})
	await create_timer(0.1).timeout
	a.slide.seek(2.0)
	await create_timer(0.1).timeout
	d._handle({"t": "truth", "hz": [14, 25, 31.1]})
	await physics_frame
	assert(a.slide.get_playback_position() >= 2.0, "Skipped steps left an impact backlog")
	# Cross the actual decoder boundaries without waiting for a full 22-second loop.
	a.rumble.seek(a.rumble.stream.get_length() - 0.05)
	a.slide.seek(a.slide.stream.get_length() - 0.05)
	await create_timer(0.3).timeout
	assert(a.rumble.playing, "The sustained rumble ended instead of looping")
	assert(a.rumble.get_playback_position() < 1.0)
	assert(not a.slide.playing, "The discrete slide looped")

	# A different sector's abandonment does not mute this source. The authoritative
	# state distinguishes abandonment from the identically named reopening event.
	d._handle({"t": "state", "sec": [1, 3]})
	assert(a.rumble.playing)
	d._handle({"t": "state", "sec": [3, 1]})
	assert(not a.rumble.playing and not a.slide.playing)
	d._handle({"t": "truth", "hz": [15, 26, 36.1]})
	assert(not a.rumble.playing, "An abandoned hazard restarted on the next truth frame")
	d._handle({"t": "event", "kind": "sector_abandoned", "sector": "A1",
		"text": "A1 reopened: directive expired"})
	d._handle({"t": "state", "sec": [1, 1]})
	await physics_frame
	assert(a.rumble.playing and not a.slide.playing, "Reopening is not a new landslide")
	d._handle({"t": "truth", "hz": [15, 26, 36.1]})
	assert(not a.slide.playing, "Muted growth replayed after reopening")

	# Burnout can remain a non-null truth row in this simulator.
	d._handle({"t": "truth", "hz": [15, 26, 0]})
	assert(not a.rumble.playing and not a.slide.playing)
	d._handle({"t": "truth", "hz": [15, 26, 7]})
	await physics_frame
	assert(a.rumble.playing and a.slide.playing)
	d._handle({"t": "truth", "hz": null})
	assert(not a.rumble.playing and not a.slide.playing)

	# Joining a running mission has no ignition event; hello + state + truth suffice.
	d._handle(hello)
	d._handle({"t": "state", "sec": [1, 3]})
	d._handle({"t": "truth", "hz": [52, 23, 10]})
	assert(not a.rumble.playing and not a.slide.playing)
	d._handle({"t": "state", "sec": [1, 1]})
	await physics_frame
	assert(a.rumble.playing)
	d._handle({"t": "event", "kind": "mission_complete"})
	d._handle({"t": "truth", "hz": [52, 23, 10]})
	assert(not a.rumble.playing and not a.slide.playing, "Completion must survive queued truth")

	d._handle(hello)
	d._handle({"t": "truth", "hz": [12, 23, 6]})
	await physics_frame
	assert(a.rumble.playing and a.slide.playing)
	# The fixture has a closed socket; the dashboard's normal disconnect path stops it.
	d._process(0.0)
	assert(not a.rumble.playing and not a.slide.playing)
	d._handle(hello)
	assert(not a.rumble.playing and not a.slide.playing)

	d.queue_free()
	await process_frame
	await create_timer(0.1).timeout  # let the mixer release stopped stream playbacks
	print("AUDIO_CHECK_OK")
	quit()

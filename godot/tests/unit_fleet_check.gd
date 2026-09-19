extends SceneTree
## Execute real MultiMesh packing/camera culling without a GUI or demo mission.

const Fleet = preload("res://scripts/unit_fleet.gd")
var failures := 0
var ground_calls := 0
var ground_level := 0.25


func _initialize() -> void:
	call_deferred("_run")


func _expect(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		push_error(message)


func _ground_pose(x: float, z: float, heading: float, _chassis: int) -> Transform3D:
	ground_calls += 1
	return Transform3D(Basis(Vector3.UP, -heading), Vector3(x, ground_level, z))


func _run() -> void:
	root.size = Vector2i(1280, 720)
	var fleet := Fleet.new()
	root.add_child(fleet)
	fleet.load_models(360.0, 240.0, 40.0)
	_expect(fleet._groups.is_empty(), "Startup must not create any geometry")
	var camera := Camera3D.new()
	root.add_child(camera)
	camera.far = 2000.0
	camera.position = Vector3(64, 200, 240)
	camera.look_at(Vector3(64, 0, 32))
	camera.make_current()
	await process_frame
	var rows: Array = []
	var variants: Array = fleet._catalog.keys()
	variants.sort()
	for i in range(512):
		var key: int = variants[i % variants.size()]
		rows.append([float((i % 32)*4), float((i / 32)*4), 0.0,
			key / 4, 0, 100.0, key % 4, 2])
	var height := func(_x: int, _y: int) -> float: return 0.0
	fleet.sync_state(rows, [], 1.0, 10.0, height, 1.0, 360, 240, _ground_pose)
	_expect(fleet._groups.is_empty(), "Telemetry alone must not create GPU meshes")
	_expect(ground_calls == 0, "Footprint sampling must wait until visibility is known")
	fleet.update_view(camera, -1)
	_expect(fleet.stats["visible"] == 512, "Overview should contain every test unit")
	_expect(fleet.stats["lod_counts"] == [0, 0, 512], "Overview should use only far silhouettes")
	_expect(fleet.stats["triangles"] <= 512*120, "Overview triangle budget regressed")
	_expect(fleet.stats["loaded_meshes"] == 15, "Overview should load one cheap LOD per variant")
	_expect(ground_calls == 512, "Each visible footprint should be sampled once")
	var overview: Dictionary = fleet.stats.duplicate(true)
	var first_mesh: MultiMesh = fleet._groups[2]["mesh"]
	var started := Time.get_ticks_usec()
	for step in range(30):
		fleet._dirty = true
		fleet.update_view(camera, -1)
	var update_ms := (Time.get_ticks_usec() - started) / 30000.0
	_expect(fleet._groups[2]["mesh"] == first_mesh, "Visible batches were rebuilt every frame")
	# A camera-only terrain LOD switch must bypass both timers immediately: waiting
	# for a new telemetry packet leaves units buried in the newly raised mesh.
	ground_level = 4.0
	var calls_before := ground_calls
	fleet.invalidate_grounding()
	fleet.update_view(camera, -1)
	_expect(ground_calls == calls_before + 512, "LOD change left stale ground poses")
	_expect(is_equal_approx(fleet._poses[0].origin.y, ground_level),
		"Ground pose did not follow the new terrain in the same frame")
	ground_level = 0.25
	fleet.invalidate_grounding()
	fleet.update_view(camera, -1)
	# A camera looking entirely away must submit no instances at all. Invisible
	# telemetry still advances, including painted/dead states, before reappearance.
	camera.look_at(camera.position + Vector3(0, 1, 1))
	fleet._dirty = true
	fleet.update_view(camera, -1)
	_expect(fleet.stats["visible"] == 0, "Behind-camera units were submitted")
	for group in fleet._groups.values():
		_expect(group["mesh"].visible_instance_count == 0, "Offscreen batch still draws instances")
	var previous_ground_calls := ground_calls
	rows[0][0] = 0.15
	rows[1][4] = 2
	fleet.sync_state(rows, [], 1.1, 10.1, height, 1.0, 360, 240, _ground_pose)
	fleet.set_unit_color(0, Color(0.2, 0.4, 0.7), 0.35)
	fleet._dirty = true
	fleet.update_view(camera, -1)
	_expect(ground_calls == previous_ground_calls, "Offscreen footprints consumed terrain work")
	_expect(fleet._travel[0] > 0.1, "Invisible motion history was lost")
	_expect(fleet._states[1] == Fleet.DISABLED, "Invisible disabled state was lost")
	fleet.advance(10.15, true, -1)
	var frozen: float = fleet._extra
	fleet.advance(15.0, false, -1)
	_expect(fleet._extra == frozen, "Disconnected mechanisms must freeze")
	fleet._release_idle_batches(Time.get_ticks_msec()*0.001 + Fleet.RELEASE_DELAY + 1.0)
	_expect(fleet._groups.is_empty(), "Inactive meshes were not reclaimed")
	camera.position = Vector3(0.15, 3, 8)
	camera.look_at(Vector3(0.15, 0.7, 0))
	fleet._dirty = true
	fleet.update_view(camera, 0, true)
	_expect(fleet.stats["culled"] > 400, "Close view did not compact offscreen units")
	_expect(fleet._lods[0] == 0, "Followed visible robot must retain its detailed model")
	_expect(fleet.stats["lod_counts"][0] <= Fleet.MAX_HIGH_UNITS, "Detail crowd cap regressed")
	var batch: int = fleet._slot_batches[0]
	var slot: int = fleet._slots[0]
	var mm: MultiMesh = fleet._groups[batch]["mesh"]
	# The headless Dummy rasterizer does not retain GPU MultiMesh buffers. Check
	# the retained CPU state and compacted slot mapping; GPU readback is a GUI check.
	var actual: Color = fleet._colors[0]
	_expect(absf(actual.r - 0.2) < 0.01 and absf(actual.a - 0.35) < 0.01,
		"Reappearing unit lost its contact/state colors")
	_expect(absf(fleet._poses[0].origin.y - 0.25) < 0.001 and mm.visible_instance_count > slot,
		"Displayed transform ignored the terrain footprint")
	fleet.advance(10.2, true, 0)
	_expect(int(fleet._groups[batch]["material"].get_shader_parameter("hidden_instance")) == slot,
		"POV hide did not follow the compacted instance slot")
	_expect(Fleet.choose_lod(31, 0, false) == 0, "High-detail hysteresis was lost")
	_expect(Fleet.choose_lod(31, 2, false) == 1, "LOD upgrade threshold was ignored")
	_expect(Fleet.choose_lod(8.5, 1, false) == 1, "Medium-detail hysteresis was lost")
	_expect(Fleet.choose_lod(8.5, 2, false) == 2, "Far detail threshold was ignored")
	# A narrow FOV must upgrade the same physical unit without moving the camera.
	camera.position = Vector3(0.15, 15, 100)
	camera.look_at(Vector3(0.15, 0.7, 0))
	camera.fov = 90.0
	fleet._lods.fill(2)
	fleet._dirty = true
	fleet.update_view(camera, -1)
	var wide_lod: int = fleet._lods[0]
	camera.fov = 12.0
	fleet._dirty = true
	fleet.update_view(camera, -1)
	_expect(fleet._lods[0] < wide_lod, "Zoom/FOV did not increase projected detail: %d -> %d" % [wide_lod, fleet._lods[0]])
	# Deliberately overlap a crowded close shot to exercise the high-detail cap,
	# independent of the generated maps' robot distribution.
	var crowd: Array = []
	for i in range(40):
		crowd.append([0.0, 0.0, 0.0, 0, 0, 100.0, 0, 2])
	fleet.sync_state(crowd, [], 2.0, 11.0, height, 1.0, 360, 240, _ground_pose)
	camera.position = Vector3(0, 3, 8)
	camera.look_at(Vector3(0, 0.7, 0))
	camera.fov = 70.0
	fleet._dirty = true
	fleet.update_view(camera, 39)
	_expect(fleet.stats["lod_counts"][0] == Fleet.MAX_HIGH_UNITS,
		"A close crowd must use the complete bounded full-detail allowance")
	_expect(fleet._lods[39] == 0 and fleet._slot_batches[39] >= 0,
		"The followed robot must win the detail budget even with a larger id")
	fleet.reset_motion()
	fleet.update_view(camera, 39)
	_expect(fleet.stats["visible"] == 0, "Reconnect drew units before a new trajectory arrived")
	print("FLEET_LOD_CHECK ", JSON.stringify({"overview": overview,
		"cpu_update_ms": update_ms, "failures": failures}))
	if failures == 0:
		print("FLEET_LOD_CHECK_OK")
	quit(0 if failures == 0 else 1)

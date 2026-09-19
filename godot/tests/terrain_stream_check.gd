extends SceneTree
## Runtime regression: real frusta, lazy residency, LOD changes and terrain contact.

var failures := 0

func _initialize() -> void:
	call_deferred("_run")

func _expect(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		push_error(message)

func _run() -> void:
	root.size = Vector2i(1280, 720)
	var source := TerrainSurface.new()
	var heights := PackedFloat32Array()
	var occ := PackedByteArray()
	var water := PackedFloat32Array()
	for y in range(64):
		for x in range(128):
			heights.append(0.025 * x + 0.015 * y + 0.4 * sin(x * 0.2))
			occ.append(0)
			water.append(0.2 if y == 20 else 0.0)
	source.setup(heights, occ, water, 128, 64, 1.0)
	var stream := TerrainStream.new()
	root.add_child(stream)
	var material := ShaderMaterial.new()
	material.shader = load("res://scripts/world_shader.gdshader")
	stream.setup(source, 128, 64, material)
	_expect(stream.get_child_count() == 0 and stream._resident.is_empty(),
		"Terrain setup eagerly creates GPU geometry")
	var camera := Camera3D.new()
	root.add_child(camera)
	camera.position = Vector3(64, 120, 190)
	camera.look_at(Vector3(64, 0, 32))
	camera.make_current()
	await process_frame
	stream.update_view(camera)
	_expect(stream._resident.size() <= TerrainStream.MAX_BUILDS,
		"New geometry exceeded the per-frame construction budget")
	for _frame in range(12):
		stream.update_view(camera)
	_expect(stream.stats.pending == 0, "Visible terrain never finished loading")
	var orbit_triangles := int(stream.stats.triangles)
	_expect(stream.stats.lod[1] < stream.stats.visible_tiles,
		"Distant orbit terrain did not simplify")
	for _frame in range(12):
		stream.update_view(camera, true)
	_expect(stream.stats.lod[1] == stream.stats.visible_tiles,
		"Full-detail view degraded distant visible terrain")
	_expect(stream.stats.triangles > orbit_triangles, "Terrain LOD did not reduce geometry")
	# The support query must follow the mesh that is actually resident, including a
	# full/coarse tile boundary beneath a long chassis.
	stream._build_tile(0, 1, Time.get_ticks_msec() * 0.001)
	stream._build_tile(1, 8, Time.get_ticks_msec() * 0.001)
	for x in [30.9, 31.5, 32.1]:
		var pose := stream.robot_pose(x, 16.0, 0.0, 0)
		for along in [-0.85, -0.175, 0.5, 1.175, 1.85]:
			for across in [-0.73, 0.0, 0.73]:
				var support := pose * Vector3(along, 0.0, across)
				_expect(support.y >= stream.height_at_world(support.x, support.z) - 0.00001,
					"Unit support sank through a mixed-LOD terrain seam")
	camera.position = Vector3(-500, 10, -500)
	camera.look_at(Vector3(-600, 10, -600))
	stream.update_view(camera)
	_expect(stream.stats.visible_tiles == 0 and stream.stats.triangles == 0,
		"Offscreen terrain is still submitted")
	for item in stream._resident.values():
		_expect(not item.node.visible, "Offscreen tile remains visible during eviction grace")
		item.last_seen = Time.get_ticks_msec() * 0.001 - 2.0
	stream.update_view(camera)
	await process_frame
	_expect(stream._resident.is_empty() and stream.get_child_count() == 0,
		"Offscreen geometry was hidden but never unloaded")
	stream.setup(source, 128, 64, material)
	_expect(stream._resident.is_empty(), "Reset retained previous world geometry")
	print("TERRAIN_STREAM_CHECK_OK" if failures == 0 else "TERRAIN_STREAM_CHECK_FAILED")
	quit(failures)

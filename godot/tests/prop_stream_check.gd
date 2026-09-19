extends SceneTree
## Native regression: --headless --path godot --script tests/prop_stream_check.gd

const Props = preload("res://scripts/prop_stream.gd")
const Surface = preload("res://scripts/terrain_surface.gd")
var failures := 0


class GroundStub extends Node:
	const TILE_CELLS := 32
	var stride := 1

	func stride_at(_x: float, _y: float) -> int:
		return stride

	func height_at_world(_x: float, _y: float) -> float:
		return float(stride - 1) * 0.25


func _initialize() -> void:
	call_deferred("_run")


func check(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		printerr(message)


func _run() -> void:
	var occupancy := PackedByteArray()
	occupancy.resize(128 * 64)
	occupancy.fill(1)
	var heights := PackedFloat32Array()
	heights.resize(128 * 64)
	heights.fill(1.55)
	var water := PackedFloat32Array()
	water.resize(128 * 64)
	var surface := Surface.new()
	surface.setup(heights, occupancy, water, 128, 64, 1.0)
	var ground := GroundStub.new()
	root.add_child(ground)
	var props := Props.new()
	root.add_child(props)
	props.setup(surface, occupancy, 128, 64, 1.0, ShaderMaterial.new(), ground)
	check(props.get_child_count() == 0, "setup eagerly created geometry")
	var camera := Camera3D.new()
	root.add_child(camera)
	camera.position = Vector3(64, 45, 118)
	camera.look_at(Vector3(64, 0, 28))
	camera.fov = 70
	for _i in range(8):
		var before := props.stats()["loaded_tiles"] as int
		props.update_view(camera)
		check(int(props.stats()["loaded_tiles"]) - before <= 2, "more than two tiles built in a frame")
	check(int(props.stats()["visible_tiles"]) > 0, "visible tiles did not stream in")
	check(props._rectangles(Vector2i.ZERO, Vector2i(32, 32)).size() == 16,
		"solid tile must merge to 16 wall footprints, not 1024 cubes")
	for _i in range(8):
		props.update_view(camera, true)
	for entry: Dictionary in props._resident.values():
		check(int(entry["lod"]) == 0, "full view mode left a visible tile simplified")
	var full_stats := props.stats()
	var ids := {}
	for key: int in props._resident:
		ids[key] = (props._resident[key]["node"] as Node).get_instance_id()
	ground.stride = 4
	props._next_scan = 0
	for _i in range(8):
		props.update_view(camera, true)
	for key: int in props._resident:
		check(str(props._resident[key]["ground_key"]) == "4", "ground LOD change did not invalidate props")
		check((props._resident[key]["node"] as Node).get_instance_id() != int(ids[key]),
			"terrain changed but old prop geometry was retained")
	camera.position = Vector3(64, 20, -150)
	camera.look_at(Vector3(64, 20, -250))
	props._next_scan = 0
	props.update_view(camera, true)
	check(int(props.stats()["visible_tiles"]) == 0, "behind-camera props remain visible")
	for entry: Dictionary in props._resident.values():
		entry["last_seen"] = Time.get_ticks_msec() - 1500
	props._next_scan = 0
	props.update_view(camera, true)
	check(int(props.stats()["loaded_tiles"]) == 0, "offscreen geometry not released after grace period")
	print(JSON.stringify({"failures": failures, "full_detail": full_stats,
		"released": props.stats(), "hash": Props._cell_hash(359, 239)}))
	quit(failures)

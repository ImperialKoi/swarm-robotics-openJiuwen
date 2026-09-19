extends SceneTree
## Exercise the real dashboard camera and picking paths against airborne telemetry.

class Fixture extends SwarmDashboard:
	func _connect_ws() -> void:
		pass


func _initialize() -> void:
	call_deferred("run")


func run() -> void:
	root.size = Vector2i(1280, 720)
	var dashboard := Fixture.new()
	root.add_child(dashboard)
	dashboard.set_process(false)
	dashboard.gw = 64
	dashboard.gh = 48
	dashboard.map_w = 64.0
	dashboard.map_h = 48.0
	var heights := PackedFloat32Array()
	heights.resize(64 * 48)
	heights.fill(0.0)
	var occupancy := PackedByteArray()
	occupancy.resize(heights.size())
	var water := heights.duplicate()
	for y in range(48):
		for x in range(36, 44):
			heights[y * 64 + x] = 25.0
	dashboard.surface = TerrainSurface.new()
	dashboard.surface.setup(heights, occupancy, water, 64, 48, 1.0)
	dashboard.ground = TerrainStream.new()
	dashboard.ground.surface = dashboard.surface
	dashboard.add_child(dashboard.ground)
	dashboard.robots = [[16.0, 24.0, 0.0, 0, 0, 1.0, 3, 2],
		[16.0, 24.0, 0.0, 0, 0, 1.0, 0, 2]]
	dashboard.airborne = [true, false]
	dashboard.follow = 0
	dashboard.ready_world = true
	await process_frame
	var aircraft := dashboard._unit_pose(0).origin
	assert(aircraft.y > dashboard._unit_pose(1).origin.y + 8.0)
	dashboard.view_mode = SwarmDashboard.View.POV
	dashboard._update_camera(0.016)
	assert(is_equal_approx(dashboard.cam.position.y, aircraft.y + 1.1))
	dashboard.view_mode = SwarmDashboard.View.CHASE
	dashboard._update_camera(0.016)
	assert(is_equal_approx(dashboard.cam_target.y, aircraft.y + dashboard.CHASE_ANCHOR_Z))
	dashboard.view_mode = SwarmDashboard.View.ORBIT
	dashboard._update_camera(0.016)
	assert(is_equal_approx(dashboard.cam_target.y, aircraft.y))
	# Same horizontal coordinate, distinct heights: clicking the aircraft must not
	# select the ground unit below it or miss both at the old ground position.
	dashboard.cam.position = aircraft + Vector3(0, 1, 18)
	dashboard.cam.look_at(aircraft + Vector3.UP * 0.6)
	dashboard.follow = -1
	dashboard._pick(dashboard.cam.unproject_position(aircraft + Vector3.UP * 0.6))
	assert(dashboard.follow == 0)
	dashboard.airborne = []  # Previous recordings do not imply flight from chassis.
	assert(dashboard._unit_pose(0).origin.y < aircraft.y - 5.0)
	dashboard.airborne = [true, false]
	dashboard.robots[0][4] = 2
	assert(not dashboard._is_airborne(0))
	print("FLIGHT_DASHBOARD_OK")
	quit()

extends SceneTree

class Fixture extends SwarmDashboard:
	func _connect_ws() -> void:
		pass  # real UI and controls, no socket needed for this test


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
	dashboard.mat = ShaderMaterial.new()
	dashboard.mat.shader = load("res://scripts/world_shader.gdshader")
	dashboard._build_markers()
	dashboard.robots = [[4.5, 10.5, 0.0, 0, 0, 100, 0, 0]]
	dashboard.robot_ids = ["scout-0"]
	dashboard.truth_victims = [[12.5, 10.5, 0, 1], [14.5, 10.5, 2, 0],
		[2.5, 10.5, 2, 0], [32.5, 10.5, 2, 0], [11.5, 10.5, 3, 0]]
	dashboard.ready_world = true
	var key := InputEventKey.new()
	key.keycode = KEY_H
	key.pressed = true
	dashboard._unhandled_input(key)
	dashboard.thermal.update_view(dashboard)
	assert(dashboard.view_mode == SwarmDashboard.View.POV and dashboard.follow == 0)
	assert(dashboard.thermal.active and dashboard.thermal.screen.visible)
	assert(dashboard.thermal.covers.multimesh.instance_count == 1)
	assert(dashboard.thermal.bodies.multimesh.instance_count == 1)
	assert(not dashboard.mk_burial.visible and not dashboard.mk_body.visible)
	assert(dashboard.ui._overlay_state("thermal") == "ON")
	if "--capture" in OS.get_cmdline_user_args():
		# Optional real Compatibility-renderer inspection, after the offline preview.
		dashboard.truth_victims[1][1] = 13.5
		dashboard.thermal.update_view(dashboard)
		dashboard.cam.position = Vector3(4.5, 3.5, 10.5)
		dashboard.cam.look_at(Vector3(12.5, 0.5, 11.5))
		var floor_mesh := MeshInstance3D.new()
		var plane := PlaneMesh.new()
		plane.size = Vector2(40, 40)
		floor_mesh.mesh = plane
		floor_mesh.position = Vector3(20, -0.1, 20)
		root.add_child(floor_mesh)
		await process_frame
		dashboard.ui.tick(0.3)
		dashboard.ui.refresh()
		await process_frame
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png("/tmp/swarmmind-thermal-native.png")
		dashboard.truth_victims[1][1] = 10.5
	dashboard.truth_victims[0][2] = 2
	dashboard.thermal.update_view(dashboard)
	assert(dashboard.thermal.covers.multimesh.instance_count == 0)
	assert(dashboard.thermal.bodies.multimesh.instance_count == 2)
	dashboard.occ[10 * 40 + 8] = 1
	dashboard.thermal.update_view(dashboard)
	assert(dashboard.thermal.bodies.multimesh.instance_count == 0, "Heat passed through a wall")
	dashboard.occ.fill(0)
	dashboard.view_mode = SwarmDashboard.View.CHASE
	dashboard.thermal.update_view(dashboard)
	assert(dashboard.thermal.active)
	dashboard._unhandled_input(key)
	dashboard.thermal.update_view(dashboard)
	assert(not dashboard.thermal.active and not dashboard.thermal.screen.visible)
	assert(dashboard.mk_burial.visible)
	dashboard.toggle("thermal")  # clickable overlay shares the key's path
	dashboard.robots[0][4] = 2
	dashboard.thermal.update_view(dashboard)
	assert(not dashboard.thermal.active, "Destroyed robot retained a thermal feed")
	dashboard.robots[0][4] = 0
	dashboard._eagle_eye()
	dashboard.thermal.update_view(dashboard)
	assert(not dashboard.thermal.active)
	assert(dashboard.ui._overlay_state("thermal") == "READY")
	dashboard._cycle_view()
	dashboard.thermal.update_view(dashboard)
	assert(dashboard.thermal.active)
	dashboard.ready_world = false
	dashboard.thermal.update_view(dashboard)
	assert(not dashboard.thermal.active)
	assert(is_equal_approx(ThermalVision.intensity(2, 3, 2, true, 5),
		ThermalVision.intensity(2, 3, 2, false, 5)))
	dashboard.queue_free()
	await process_frame
	print("THERMAL_CHECK_OK")
	quit()

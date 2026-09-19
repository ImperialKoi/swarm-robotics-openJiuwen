extends SceneTree
## Exercise the dashboard's real marker path, including hidden truth and cleared state.

func _initialize() -> void:
	call_deferred("run")


func run() -> void:
	var dashboard: Node3D = load("res://Main.tscn").instantiate()
	# Test the actual marker methods without starting the dashboard's socket/UI.
	dashboard.set_process(false)
	dashboard.mat = ShaderMaterial.new()
	dashboard.mat.shader = load("res://scripts/world_shader.gdshader")
	dashboard._build_markers()
	dashboard.truth_victims = [[10, 10, 0, 1], [20, 10, 1, 1], [30, 10, 2, 1], [40, 10, 2, 0]]
	dashboard._update_markers()
	assert(dashboard.mk_burial.multimesh.instance_count == 0, "Undiscovered truth leaked into normal view")
	assert(not dashboard.mk_body.visible)
	dashboard.reports = [[20, 10, 1], [30, 10, 3], [40, 10, 2], [50, 10, 0]]
	dashboard._update_markers()
	assert(dashboard.mk_burial.multimesh.instance_count == 1, "Only confirmed buried contacts get rubble")
	dashboard.god_view = true
	dashboard._update_markers()
	assert(dashboard.mk_burial.multimesh.instance_count == 2)
	assert(dashboard.mk_body.multimesh.instance_count == 4)
	# Headless's dummy rendering backend does not retain MultiMesh GPU transforms.
	assert(is_equal_approx(dashboard._casualty_body_pose(10, 10, true).origin.y, Burial.BURIED_LIFT))
	assert(is_equal_approx(dashboard._casualty_body_pose(30, 10, false).origin.y, Burial.SURFACE_LIFT))
	dashboard.truth_victims = [[10, 10, 2, 1], [20, 10, 3, 1], [30, 10, 4, 1]]
	dashboard._update_markers()
	assert(dashboard.mk_burial.multimesh.instance_count == 0, "Historical buried flag reburied a cleared casualty")
	assert(dashboard.mk_body.multimesh.instance_count == 1, "Carried/rescued bodies remain on the ground")
	assert(dashboard.mk_victim.multimesh.instance_count == 2)
	dashboard.god_view = false
	dashboard.reports = []
	dashboard._update_markers()
	assert(dashboard.mk_burial.multimesh.instance_count == 0)
	assert(not dashboard.mk_body.visible)
	dashboard.free()
	print("BURIAL_CHECK_OK")
	quit()

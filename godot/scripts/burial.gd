class_name Burial
extends RefCounted
## Original display mesh exported by scripts/design_burial.py; no simulation access.

const SOURCE := "res://assets/victims/burial.json"
const BURIED_LIFT := -0.28
const SURFACE_LIFT := 0.05


static func needs_excavation(row: Array) -> bool:
	# The fourth column is historical: a dug-out casualty still has buried == 1.
	return int(row[3]) == 1 and int(row[2]) < 2


static func build_mesh() -> ArrayMesh:
	var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(SOURCE))
	var vertices := PackedVector3Array()
	var colors := PackedColorArray()
	for value: Array in data.vertices:
		vertices.append(Vector3(value[0], value[1], value[2]))
	for value: Array in data.colors:
		colors.append(Color(value[0], value[1], value[2]))
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_COLOR] = colors
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh

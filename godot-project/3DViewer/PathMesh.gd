extends MeshInstance3D
## Drivable-area boundary lines rendered as neon cyan-white pulsing ribbons
## (path_boundary.gdshader). Vertex COLOR tints the uniform line color.

const BOUNDARY_COLOR := Color(1.0, 1.0, 1.0, 1.0)

var path := BehaviorPath.new()


func _ready() -> void:
	path.subscribe("/planning/scenario_planning/lane_driving/behavior_planning/path", false)

	var boundary_material := ShaderMaterial.new()
	boundary_material.shader = preload("res://3DViewer/Shaders/path_boundary.gdshader")
	boundary_material.render_priority = 0
	material_override = boundary_material


func _process(_delta: float) -> void:
	if !path.has_new():
		return

	var drivable_area_triangle_strip: Dictionary = path.get_drivable_area_triangle_strip(0.1)
	var left_line_arr := MeshUtils.strip_to_surface_arrays(
		drivable_area_triangle_strip["left_line"], BOUNDARY_COLOR)
	var right_line_arr := MeshUtils.strip_to_surface_arrays(
		drivable_area_triangle_strip["right_line"], BOUNDARY_COLOR)

	if MeshUtils.has_vertices(left_line_arr):
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, left_line_arr)
		if MeshUtils.has_vertices(right_line_arr):
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, right_line_arr)
	path.set_old()

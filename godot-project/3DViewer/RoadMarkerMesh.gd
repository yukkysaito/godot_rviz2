extends MeshInstance3D
## Road markers rendered slightly emissive so lanes read clearly at night
## (road_marker.gdshader). Vertex COLOR carries the tint: cool-white for lane
## markings / pedestrian markings, amber for stop lines.

const MARKER_COLOR := Color(0.812, 0.910, 1.0, 0.9) # cool white #CFE8FF
const STOP_LINE_COLOR := Color(1.0, 0.702, 0.0, 0.95) # amber #FFB300


func _ready() -> void:
	var marker_material := ShaderMaterial.new()
	marker_material.shader = preload("res://3DViewer/Shaders/road_marker.gdshader")
	marker_material.render_priority = -2
	material_override = marker_material


func visualize_mesh(marker_triangle_list: Array, stop_line_triangle_list: Array = []) -> void:
	mesh.clear_surfaces()
	var marker_arr := MeshUtils.triangle_list_to_surface_arrays(
		marker_triangle_list, MARKER_COLOR)
	if MeshUtils.has_vertices(marker_arr):
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, marker_arr)
	var stop_line_arr := MeshUtils.triangle_list_to_surface_arrays(
		stop_line_triangle_list, STOP_LINE_COLOR)
	if MeshUtils.has_vertices(stop_line_arr):
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, stop_line_arr)

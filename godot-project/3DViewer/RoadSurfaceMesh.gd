extends MeshInstance3D
## Road surface: very dark navy with subtle large-scale procedural variation
## and a far-distance fade into the background (road_surface.gdshader).


func _ready() -> void:
	var surface_material := ShaderMaterial.new()
	surface_material.shader = preload("res://3DViewer/Shaders/road_surface.gdshader")
	surface_material.render_priority = -3
	material_override = surface_material


func visualize_mesh(triangle_list: Array) -> void:
	mesh.clear_surfaces()
	var arr := MeshUtils.triangle_list_to_surface_arrays(triangle_list)
	if MeshUtils.has_vertices(arr):
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

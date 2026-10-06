extends MeshInstance3D

# Triangle vertices (every 3 vertices form a triangle), facing up
func visualize_vertices(vertices: PackedVector3Array) -> void:
	mesh.clear_surfaces()
	if vertices.is_empty():
		return
	var normals := PackedVector3Array()
	normals.resize(vertices.size())
	normals.fill(Vector3.UP)
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = vertices
	arr[Mesh.ARRAY_NORMAL] = normals
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

extends MeshInstance3D

func visualize_mesh(triangle_list):
	mesh.clear_surfaces()

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PackedVector3Array()
	var normals = PackedVector3Array()
#	var colors = PoolColorArray()
	
	for point in triangle_list:
		verts.append(point["position"])
		normals.append(Vector3(0,1,0))
			
	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

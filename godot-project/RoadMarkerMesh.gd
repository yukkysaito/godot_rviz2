extends MeshInstance

func visualize_mesh(verts):
	mesh.clear_surfaces()

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
#	var uvs = PoolVector2Array()
	var normals = PoolVector3Array()
#	var indices = PoolIntArray()
#	var colors = PoolColorArray()
	
	for i in verts.size():
		normals.append(Vector3(0,1,0))
			
	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)

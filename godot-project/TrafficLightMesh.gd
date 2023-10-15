extends MeshInstance

func cross_product(a, b):
	return Vector3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)

func visualize_mesh(boards_verts, traffic_lights):
	mesh.clear_surfaces()

	# Traffic Board
	var boards_arr = []
	boards_arr.resize(Mesh.ARRAY_MAX)
#	var uvs = PoolVector2Array()
	var boards_normals = PoolVector3Array()
#	var boards_indices = PoolIntArray()
	var boards_colors = PoolColorArray()

	var boards_verts_size = boards_verts.size()
	# back board
	for i in boards_verts_size:
		if(i % 3 ==2):
			boards_normals.append(cross_product(boards_verts[i] - boards_verts[i-2], boards_verts[i-1] - boards_verts[i-2]).normalized())
			boards_normals.append(boards_normals[boards_normals.size()-1])
			boards_normals.append(boards_normals[boards_normals.size()-1])
		boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))

	# front board
	for i in boards_verts_size:
		if(i % 3 ==2):
			var j_0 = i-1
			var j_1 = i-2
			var j_2 = i
			boards_verts.append(boards_verts[j_0])
			boards_verts.append(boards_verts[j_1])
			boards_verts.append(boards_verts[j_2])
			boards_normals.append(cross_product(boards_verts[j_2] - boards_verts[j_2-2], boards_verts[j_2-1] - boards_verts[j_2-2]).normalized())
			boards_normals.append(boards_normals[boards_normals.size()-1])
			boards_normals.append(boards_normals[boards_normals.size()-1])
		boards_colors.append(Color(0.5, 0.5, 0.5, 1.0))


	boards_arr[Mesh.ARRAY_VERTEX] = boards_verts
	boards_arr[Mesh.ARRAY_NORMAL] = boards_normals
	boards_arr[Mesh.ARRAY_COLOR] = boards_colors

	# Traffic Light
	var spheres_arr = []
	var sphere_verts = PoolVector3Array()
	var sphere_normals = PoolVector3Array()
	var sphere_indices = PoolIntArray()
	var sphere_colors = PoolColorArray()
	spheres_arr.resize(Mesh.ARRAY_MAX)

	var index_offset = 0
	var rgb_scale = 0.05
	for traffic_light in traffic_lights:
		var sphere_mesh = SphereMesh.new()
		sphere_mesh.set_height(traffic_light[3].x)
		sphere_mesh.set_radius(traffic_light[3].x * 0.5)
		sphere_mesh.set_radial_segments(8)
		sphere_mesh.set_rings(4)

		var sphere_arr = sphere_mesh.get_mesh_arrays()
		for sphere_vert in sphere_arr[Mesh.ARRAY_VERTEX]: 
			sphere_verts.append(sphere_vert + Vector3(traffic_light[1].x, traffic_light[1].y, traffic_light[1].z))
			sphere_colors.append(Color(traffic_light[0].r * rgb_scale, traffic_light[0].g * rgb_scale, traffic_light[0].b * rgb_scale, 1.0))
		for sphere_normal in sphere_arr[Mesh.ARRAY_NORMAL]: 
			sphere_normals.append(sphere_normal)
		for sphere_index in sphere_arr[Mesh.ARRAY_INDEX]: 
			sphere_indices.append(sphere_index+index_offset)
		index_offset += sphere_arr[Mesh.ARRAY_VERTEX].size()
	spheres_arr[Mesh.ARRAY_VERTEX] = sphere_verts
	spheres_arr[Mesh.ARRAY_NORMAL] = sphere_normals
	spheres_arr[Mesh.ARRAY_INDEX] = sphere_indices
	spheres_arr[Mesh.ARRAY_COLOR] = sphere_colors
	
	# visualize
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, spheres_arr)
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, boards_arr)

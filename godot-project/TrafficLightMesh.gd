extends MeshInstance3D

func visualize_mesh(board_triangle_list, traffic_lights):
	mesh.clear_surfaces()

	# Traffic Board
	var boards_arr = []
	boards_arr.resize(Mesh.ARRAY_MAX)
	var boards_verts = PackedVector3Array()
	var boards_normals = PackedVector3Array()
	var boards_colors = PackedColorArray()

	# back board
	for point in board_triangle_list:
		boards_verts.append(point["position"])
		boards_normals.append(point["normal"])
		boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))

	# front board
	for i in board_triangle_list.size():
		if(i % 3 ==2):
			boards_verts.append(board_triangle_list[i]["position"])
			boards_verts.append(board_triangle_list[i-1]["position"])
			boards_verts.append(board_triangle_list[i-2]["position"])
			boards_normals.append(-1.0*board_triangle_list[i]["normal"])
			boards_normals.append(boards_normals[boards_normals.size()-1])
			boards_normals.append(boards_normals[boards_normals.size()-1])
		boards_colors.append(Color(0.5, 0.5, 0.5, 1.0))

	boards_arr[Mesh.ARRAY_VERTEX] = boards_verts
	boards_arr[Mesh.ARRAY_NORMAL] = boards_normals
	boards_arr[Mesh.ARRAY_COLOR] = boards_colors

	# Traffic Light
	var spheres_arr = []
	var sphere_verts = PackedVector3Array()
	var sphere_normals = PackedVector3Array()
	var sphere_indices = PackedInt32Array()
	var sphere_colors = PackedColorArray()
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

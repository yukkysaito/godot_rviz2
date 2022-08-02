extends MeshInstance

var vector_map = MarkerArray.new()

func cross_product(a, b):
	return Vector3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)

func _ready():
	vector_map.subscribe("/map/vector_map_marker", true)

func _process(_delta):
	if !vector_map.is_new():
		return
	mesh.clear_surfaces()

	var boards_arr = []
	boards_arr.resize(Mesh.ARRAY_MAX)
	var boards_verts = PoolVector3Array()
#	var uvs = PoolVector2Array()
	var boards_normals = PoolVector3Array()
	var boards_indices = PoolIntArray()
	var boards_colors = PoolColorArray()

	boards_verts.append_array(vector_map.get_triangle_marker("traffic_light_triangle"))
	var boards_verts_size = boards_verts.size()
	for i in boards_verts_size:
		if(i % 3 ==2):
			boards_normals.append(cross_product(boards_verts[i] - boards_verts[i-2], boards_verts[i-1] - boards_verts[i-2]).normalized())
			boards_normals.append(boards_normals[boards_normals.size()-1])
			boards_normals.append(boards_normals[boards_normals.size()-1])
		boards_colors.append(Color(0, 0, 0, 0.7))
	# inversed board
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
		boards_colors.append(Color(0, 0, 0, 0.1))

	boards_arr[Mesh.ARRAY_VERTEX] = boards_verts
	boards_arr[Mesh.ARRAY_NORMAL] = boards_normals
	boards_arr[Mesh.ARRAY_COLOR] = boards_colors
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, boards_arr)

	var traffic_lights = vector_map.get_color_spheres("traffic_light")
	var spheres_arr = []
	var sphere_verts = PoolVector3Array()
	var sphere_normals = PoolVector3Array()
	var sphere_indices = PoolIntArray()
	var sphere_colors = PoolColorArray()
	spheres_arr.resize(Mesh.ARRAY_MAX)

	var index_offset = 0
	for traffic_light in traffic_lights:
		var sphere_mesh = SphereMesh.new()
		sphere_mesh.set_height(traffic_light[3].x)
		sphere_mesh.set_radius(traffic_light[3].x * 0.5)
		sphere_mesh.set_radial_segments(8)
		sphere_mesh.set_rings(4)

		var sphere_arr = sphere_mesh.get_mesh_arrays()
		for sphere_vert in sphere_arr[Mesh.ARRAY_VERTEX]: 
			sphere_verts.append(sphere_vert + Vector3(traffic_light[1].x, traffic_light[1].y, traffic_light[1].z))
			sphere_colors.append(Color(traffic_light[0].r, traffic_light[0].g, traffic_light[0].b, traffic_light[0].a))
		for sphere_normal in sphere_arr[Mesh.ARRAY_NORMAL]: 
			sphere_normals.append(sphere_normal)
		for sphere_index in sphere_arr[Mesh.ARRAY_INDEX]: 
			sphere_indices.append(sphere_index+index_offset)
		index_offset += sphere_arr[Mesh.ARRAY_VERTEX].size()
	spheres_arr[Mesh.ARRAY_VERTEX] = sphere_verts
	spheres_arr[Mesh.ARRAY_NORMAL] = sphere_normals
	spheres_arr[Mesh.ARRAY_INDEX] = sphere_indices
	spheres_arr[Mesh.ARRAY_COLOR] = sphere_colors
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, spheres_arr)

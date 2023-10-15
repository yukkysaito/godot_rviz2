extends MeshInstance

var traffic_light_markers = MarkerArray.new()

func _ready():
	traffic_light_markers.subscribe("/perception/traffic_light_recognition/traffic_signals/markers", false)

func _process(_delta):
	if !traffic_light_markers.has_new():
		return
	mesh.clear_surfaces()

	var traffic_lights = traffic_light_markers.get_color_spheres("traffic_light")
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
			sphere_colors.append(Color(traffic_light[0].r, traffic_light[0].g, traffic_light[0].b, 1.0))
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
	traffic_light_markers.set_old()

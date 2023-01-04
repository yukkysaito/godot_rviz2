extends MeshInstance

var path = BehaviorPath.new()

func _ready():
	path.subscribe("/planning/scenario_planning/lane_driving/behavior_planning/path", false)
	
func _process(_delta):
	if !path.has_new():
		return

	# Drivable area
	var drivable_area_triangle_strips = path.get_drivable_area_triangle_strips(0.1)

	# Left line
	var left_line_arr = []
	left_line_arr.resize(Mesh.ARRAY_MAX)
	var left_line_verts = PoolVector3Array()
	var left_line_normals = PoolVector3Array()
	var left_line_colors = PoolColorArray()
	for left_line_point in drivable_area_triangle_strips[0]:
		left_line_verts.append(left_line_point)
		left_line_normals.append(Vector3(0,1,0))
		left_line_colors.append(Color(0.0, 0.25, 1.0, 0.9))
	left_line_arr[Mesh.ARRAY_VERTEX] = left_line_verts
	left_line_arr[Mesh.ARRAY_NORMAL] = left_line_normals
	left_line_arr[Mesh.ARRAY_COLOR] = left_line_colors

	# Right line
	var right_line_arr = []
	right_line_arr.resize(Mesh.ARRAY_MAX)
	var right_line_verts = PoolVector3Array()
	var right_line_normals = PoolVector3Array()
	var right_line_colors = PoolColorArray()
	for right_line_point in drivable_area_triangle_strips[1]:
		right_line_verts.append(right_line_point)
		right_line_normals.append(Vector3(0,1,0))
		right_line_colors.append(Color(0.0, 0.25, 1.0, 0.9))
	right_line_arr[Mesh.ARRAY_VERTEX] = right_line_verts
	right_line_arr[Mesh.ARRAY_NORMAL] = right_line_normals
	right_line_arr[Mesh.ARRAY_COLOR] = right_line_colors

	if !left_line_verts.empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, left_line_arr)
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, right_line_arr)
	path.set_old()

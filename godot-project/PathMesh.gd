extends MeshInstance3D

var path = BehaviorPath.new()

func _ready():
	path.subscribe("/planning/scenario_planning/lane_driving/behavior_planning/path", false)
	
func _process(_delta):
	if !path.has_new():
		return

	# Drivable area
	var drivable_area_triangle_strip = path.get_drivable_area_triangle_strip(0.1)

	# Left line
	var left_line_triangle_strip = drivable_area_triangle_strip["left_line"]
	var left_line_arr = []
	left_line_arr.resize(Mesh.ARRAY_MAX)
	var left_line_verts = PackedVector3Array()
	var left_line_normals = PackedVector3Array()
	var left_line_colors = PackedColorArray()
	for point in left_line_triangle_strip:
		left_line_verts.append(point["position"])
		left_line_normals.append(point["normal"])
		left_line_colors.append(Color(0.0, 0.25, 1.0, 0.9))
	left_line_arr[Mesh.ARRAY_VERTEX] = left_line_verts
	left_line_arr[Mesh.ARRAY_NORMAL] = left_line_normals
	left_line_arr[Mesh.ARRAY_COLOR] = left_line_colors

	# Right line
	var right_line_triangle_strip = drivable_area_triangle_strip["right_line"]
	var right_line_arr = []
	right_line_arr.resize(Mesh.ARRAY_MAX)
	var right_line_verts = PackedVector3Array()
	var right_line_normals = PackedVector3Array()
	var right_line_colors = PackedColorArray()
	for point in right_line_triangle_strip:
		right_line_verts.append(point["position"])
		right_line_normals.append(point["normal"])
		right_line_colors.append(Color(0.0, 0.25, 1.0, 0.9))
	right_line_arr[Mesh.ARRAY_VERTEX] = right_line_verts
	right_line_arr[Mesh.ARRAY_NORMAL] = right_line_normals
	right_line_arr[Mesh.ARRAY_COLOR] = right_line_colors

	if !left_line_verts.is_empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, left_line_arr)
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, right_line_arr)
	path.set_old()

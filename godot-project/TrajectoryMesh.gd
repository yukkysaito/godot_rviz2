extends MeshInstance

var trajectory = Trajectory.new()
var wheelbase_to_front = 2.78 + 1.0

func cross_product(a, b):
	return Vector3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
func velocity_to_normalized_value(velocity):
	return clamp(velocity / 5.0, 0.0, 1.0)
func velocity_to_color(velocity):
	var alpha = clamp(velocity_to_normalized_value(velocity), 0.2, 0.9)
	return Color(0, 0.2, 1.0, alpha)

func create_wall_triangle_strip(first_point, second_point, width, height, length_offset):
	var wall_arr = []
	wall_arr.resize(Mesh.ARRAY_MAX)
	var wall_verts = PoolVector3Array()
#	var wall_uvs = PoolVector2Array()
	var wall_normals = PoolVector3Array()
	var wall_colors = PoolColorArray()
	
	var z_offset_vector = Vector2((second_point.x - first_point.x), (second_point.z - first_point.z))
	var z_offset_scale = 0.5*(width - z_offset_vector.length())
	var normalized_z_offset_vector = z_offset_vector.normalized()
	var x_rotate_vector = Vector2(0, 1) # 90 deg
	var x_offset_vector = length_offset * Vector2(
		(normalized_z_offset_vector.x * x_rotate_vector.x - normalized_z_offset_vector.y * x_rotate_vector.y),
		(normalized_z_offset_vector.x * x_rotate_vector.y + normalized_z_offset_vector.y * x_rotate_vector.x))

	wall_verts.append(Vector3(first_point.x - z_offset_scale * normalized_z_offset_vector.x + x_offset_vector.x, first_point.y, first_point.z - z_offset_scale * normalized_z_offset_vector.y + x_offset_vector.y))
	wall_verts.append(Vector3(second_point.x + z_offset_scale * normalized_z_offset_vector.x + x_offset_vector.x, second_point.y, second_point.z + z_offset_scale * normalized_z_offset_vector.y + x_offset_vector.y))
	wall_verts.append(Vector3(first_point.x - z_offset_scale * normalized_z_offset_vector.x + x_offset_vector.x, first_point.y + height, first_point.z - z_offset_scale * normalized_z_offset_vector.y + x_offset_vector.y))
	wall_verts.append(Vector3(second_point.x + z_offset_scale * normalized_z_offset_vector.x + x_offset_vector.x, second_point.y + height, second_point.z + z_offset_scale * normalized_z_offset_vector.y + x_offset_vector.y))
	wall_normals.append(cross_product(wall_verts[1]-wall_verts[0], wall_verts[2]-wall_verts[0]))
	wall_normals.append(wall_normals[0])
	wall_normals.append(wall_normals[0])
	wall_normals.append(wall_normals[0])
	wall_colors.append(Color(1.0, 0.0, 0.0, 1.0))
	wall_colors.append(Color(1.0, 0.0, 0.0, 1.0))
	wall_colors.append(Color(1.0, 0.0, 0.0, 0.0))
	wall_colors.append(Color(1.0, 0.0, 0.0, 0.0))
	wall_arr[Mesh.ARRAY_VERTEX] = wall_verts
	wall_arr[Mesh.ARRAY_NORMAL] = wall_normals
	wall_arr[Mesh.ARRAY_COLOR] = wall_colors
#	wall_arr[Mesh.ARRAY_INDEX] = wall_indices
#	wall_arr[Mesh.ARRAY_TEX_UV] = wall_uvs
	
	return wall_arr

func _ready():
	trajectory.subscribe("/planning/scenario_planning/trajectory", false)
	
func _process(_delta):
	if !trajectory.has_new():
		return

	# Trajectory
	var traj_arr = []
	traj_arr.resize(Mesh.ARRAY_MAX)
	var traj_verts = PoolVector3Array()
	var traj_uvs = PoolVector2Array()
	var traj_normals = PoolVector3Array()
#	var traj_indices = PoolIntArray()
	var traj_colors = PoolColorArray()
	# Create triangle
	var triangle_strip_with_velocity = trajectory.get_triangle_strip_with_velocity(2.0)
	for point_with_velocity in triangle_strip_with_velocity:
		traj_verts.append(point_with_velocity[1])
		traj_normals.append(Vector3(0,1,0))
		traj_uvs.append(Vector2(velocity_to_normalized_value(point_with_velocity[0]), 0))
		traj_colors.append(Color(0.0, 0.02, 1.0, 0.8))
	traj_arr[Mesh.ARRAY_VERTEX] = traj_verts
	traj_arr[Mesh.ARRAY_NORMAL] = traj_normals
	traj_arr[Mesh.ARRAY_COLOR] = traj_colors
#	traj_arr[Mesh.ARRAY_INDEX] = traj_indices
#	traj_arr[Mesh.ARRAY_TEX_UV] = traj_uvs


	# Wall
	var wall_base_points = PoolVector3Array()
	if 1 < triangle_strip_with_velocity.size():
		for i in triangle_strip_with_velocity.size() - 2:
			if (triangle_strip_with_velocity[i][0] == 0.0):
				wall_base_points.append(Vector3(triangle_strip_with_velocity[i][1]))
				wall_base_points.append(Vector3(triangle_strip_with_velocity[i+1][1]))
				break
	
	if !traj_verts.empty():
		mesh.clear_surfaces()
		# Wall
		if not wall_base_points.empty():
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, 
			create_wall_triangle_strip(wall_base_points[0], wall_base_points[1], 4.0, 2.0, wheelbase_to_front))
		# Trajectory
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, traj_arr)
	trajectory.set_old()

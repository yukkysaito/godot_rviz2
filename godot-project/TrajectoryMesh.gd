extends MeshInstance

var trajectory = Trajectory.new()
var wheelbase_to_front = 2.78 + 1.0

func velocity_to_normalized_value(velocity):
	return clamp(velocity / 5.0, 0.0, 1.0)
func velocity_to_color(velocity):
	var alpha = clamp(velocity_to_normalized_value(velocity), 0.2, 0.9)
	return Color(0, 0.2, 1.0, alpha)

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
	var traj_indices = PoolIntArray()
	var traj_colors = PoolColorArray()
	# Create triangle
	var trajectory_triangle_strip = trajectory.get_trajectory_triangle_strip(2.0)
	for point in trajectory_triangle_strip:
		traj_verts.append(point["position"])
		traj_normals.append(point["normal"])
		traj_uvs.append(Vector2(velocity_to_normalized_value(point["velocity"]), 0))
		traj_colors.append(Color(0.0, 0.02, 1.0, 0.8))
	traj_arr[Mesh.ARRAY_VERTEX] = traj_verts
	traj_arr[Mesh.ARRAY_NORMAL] = traj_normals
	traj_arr[Mesh.ARRAY_COLOR] = traj_colors
#	traj_arr[Mesh.ARRAY_INDEX] = traj_indices
#	traj_arr[Mesh.ARRAY_TEX_UV] = traj_uvs


	# Wall
	var wall_arr = []
	wall_arr.resize(Mesh.ARRAY_MAX)
	var wall_verts = PoolVector3Array()
	var wall_uvs = PoolVector2Array()
	var wall_normals = PoolVector3Array()
	var wall_indices = PoolIntArray()
	var wall_colors = PoolColorArray()
	# Create triangle
	var wall_triangle_strip = trajectory.get_wall_triangle_strip(4.0, 2.0, wheelbase_to_front, true, true)
	for point in wall_triangle_strip:
		wall_verts.append(point["position"])
		wall_normals.append(point["normal"])
		wall_colors.append(point["color"])
	wall_arr[Mesh.ARRAY_VERTEX] = wall_verts
	wall_arr[Mesh.ARRAY_NORMAL] = wall_normals
	wall_arr[Mesh.ARRAY_COLOR] = wall_colors
	
	if !traj_verts.empty():
		mesh.clear_surfaces()
		# Trajectory
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, traj_arr)
		# Wall
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, wall_arr)
	trajectory.set_old()

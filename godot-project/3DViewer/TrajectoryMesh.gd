extends MeshInstance3D

var trajectory = Trajectory.new()
@export var wheelbase_to_front: float = 3.78
var high_contrast_enabled: bool = false

func velocity_to_normalized_value(velocity):
	return clamp(velocity / 5.0, 0.0, 1.0)
func velocity_to_color(velocity):
	var alpha = clamp(velocity_to_normalized_value(velocity), 0.2, 0.9)
	return Color(0, 0.2, 1.0, alpha)

func apply_high_contrast(_color: Color):
	return Color(0.0, 1.0, 0.0, 1.0)

func update_distance_fade_mode():
	var trajectory_material = material_override as StandardMaterial3D
	if trajectory_material == null:
		return

	if high_contrast_enabled:
		trajectory_material.distance_fade_mode = BaseMaterial3D.DISTANCE_FADE_DISABLED
	else:
		trajectory_material.distance_fade_mode = BaseMaterial3D.DISTANCE_FADE_PIXEL_ALPHA

func _ready():
	trajectory.subscribe("/planning/trajectory", false)
	update_distance_fade_mode()

func _on_high_contrast_toggle_toggled(toggled_on):
	high_contrast_enabled = toggled_on
	update_distance_fade_mode()
	
func _process(_delta):
	if !trajectory.has_new():
		return

	# Trajectory
	var traj_arr = []
	traj_arr.resize(Mesh.ARRAY_MAX)
	var traj_verts = PackedVector3Array()
	#var traj_uvs = PackedVector2Array()
	var traj_normals = PackedVector3Array()
	#var traj_indices = PackedInt32Array()
	var traj_colors = PackedColorArray()
	# Create triangle
	var trajectory_triangle_strip = trajectory.get_trajectory_triangle_strip(2.0)
	var trajectory_color = Color(0.0, 0.02, 1.0, 0.8)
	if high_contrast_enabled:
		trajectory_color = apply_high_contrast(trajectory_color)
	for point in trajectory_triangle_strip:
		traj_verts.append(point["position"])
		traj_normals.append(point["normal"])
		#traj_uvs.append(Vector2(velocity_to_normalized_value(point["velocity"]), 0))
		traj_colors.append(trajectory_color)
	traj_arr[Mesh.ARRAY_VERTEX] = traj_verts
	traj_arr[Mesh.ARRAY_NORMAL] = traj_normals
	traj_arr[Mesh.ARRAY_COLOR] = traj_colors
#	traj_arr[Mesh.ARRAY_INDEX] = traj_indices
#	traj_arr[Mesh.ARRAY_TEX_UV] = traj_uvs


	# Wall
	var wall_arr = []
	wall_arr.resize(Mesh.ARRAY_MAX)
	var wall_verts = PackedVector3Array()
	#var wall_uvs = PackedVector2Array()
	var wall_normals = PackedVector3Array()
	#var wall_indices = PackedInt32Array()
	var wall_colors = PackedColorArray()
	# Create triangle
	var wall_triangle_strip = trajectory.get_wall_triangle_strip(4.0, 2.0, wheelbase_to_front, true, true)
	for point in wall_triangle_strip:
		wall_verts.append(point["position"])
		wall_normals.append(point["normal"])
		var wall_color = point["color"]
		if high_contrast_enabled:
			wall_color = apply_high_contrast(wall_color)
		wall_colors.append(wall_color)
	wall_arr[Mesh.ARRAY_VERTEX] = wall_verts
	wall_arr[Mesh.ARRAY_NORMAL] = wall_normals
	wall_arr[Mesh.ARRAY_COLOR] = wall_colors
	
	if !traj_verts.is_empty():
		mesh.clear_surfaces()
		# Trajectory
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, traj_arr)
		# Wall
		if !wall_triangle_strip.is_empty():
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, wall_arr)
	trajectory.set_old()

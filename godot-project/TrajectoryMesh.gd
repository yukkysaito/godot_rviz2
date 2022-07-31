extends MeshInstance

var trajectory = Trajectory.new()

func cross_product(a, b):
	return Vector3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)
func velocity_to_normalized_value(velocity):
	return clamp(velocity / 6.0, 0.0, 1.0)
func velocity_to_color(velocity):
	var alpha = clamp(velocity_to_normalized_value(velocity), 0.2, 0.9)
	return Color(0, 0.2, 1.0, alpha)

func _ready():
	trajectory.subscribe("/planning/scenario_planning/trajectory", false)

func _process(_delta):
	if !trajectory.is_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PoolVector3Array()
	var uvs = PoolVector2Array()
	var normals = PoolVector3Array()
#	var indices = PoolIntArray()
#	var colors = PoolColorArray()

	var triangle_strip_with_velocity = trajectory.get_triangle_strip_with_velocity(2.0)
	for point_with_velocity in triangle_strip_with_velocity:
		verts.append(point_with_velocity[1])
		normals.append(Vector3(0,1,0))
#		colors.append(velocity_to_color(point_with_velocity[0]))
#		indices.append(0)
#		uvs.append(Vector2(0.9, 0))
		uvs.append(Vector2(velocity_to_normalized_value(point_with_velocity[0]), 0))


	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
#	arr[Mesh.ARRAY_COLOR] = colors
#	arr[Mesh.ARRAY_INDEX] = indices
	arr[Mesh.ARRAY_TEX_UV] = uvs
	if !verts.empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, arr)

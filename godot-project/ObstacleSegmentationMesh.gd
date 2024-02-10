extends MeshInstance3D

var pointcloud = PointCloud.new()
var sum_time = 0.0
var transparency_speed = 4.0
var cycle_time = 3.0

func _ready():
	pointcloud.subscribe("/perception/obstacle_segmentation/pointcloud", false)

func _process(delta):
	sum_time += delta
	if !pointcloud.has_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PackedVector3Array()
#	var uvs = PoolVector2Array()
#	var normals = PoolVector3Array()
#	var indices = PoolIntArray()

	verts = pointcloud.get_pointcloud("map")
#	for vert in verts:
#		uvs.append(Vector2(vert.y, 0))

	arr[Mesh.ARRAY_VERTEX] = verts
#	arr[Mesh.ARRAY_TEX_UV] = uvs

	if sum_time * transparency_speed < 2 * PI:
		transparency = (cos(sum_time * transparency_speed) + 1.0) * 0.5
	if (cycle_time < sum_time):
		sum_time = 0.0


	if !verts.is_empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)
	pointcloud.set_old()

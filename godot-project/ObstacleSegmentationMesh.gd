extends MeshInstance

var pointcloud = PointCloud.new()

func _ready():
	pointcloud.subscribe("/perception/obstacle_segmentation/pointcloud", false)

func _process(_delta):
	if !pointcloud.has_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PoolVector3Array()
#	var uvs = PoolVector2Array()
#	var normals = PoolVector3Array()
#	var indices = PoolIntArray()

	verts = pointcloud.get_pointcloud("map")
#	for vert in verts:
#		uvs.append(Vector2(vert.y, 0))

	arr[Mesh.ARRAY_VERTEX] = verts
#	arr[Mesh.ARRAY_TEX_UV] = uvs
	if !verts.empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)
	pointcloud.set_old()

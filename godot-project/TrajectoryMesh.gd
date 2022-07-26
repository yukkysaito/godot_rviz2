extends MeshInstance

var trajectory = Trajectory.new()

func cross_product(a, b):
	return Vector3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)

func _ready():
	trajectory.subscribe("/planning/scenario_planning/trajectory", false)

func _process(_delta):
	if !trajectory.is_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PoolVector3Array()
#	var uvs = PoolVector2Array()
	var normals = PoolVector3Array()
#	var indices = PoolIntArray()

	verts = trajectory.get_triangle_strip(2.0)
	for i in verts.size():
		normals.append(Vector3(0,1,0))

	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
	if !verts.empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, arr)

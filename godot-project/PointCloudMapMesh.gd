extends MeshInstance

var pointcloud = PointCloud.new()
var visualize_again = false

func _ready():
	pointcloud.subscribe("/map/pointcloud_map", true)

func _process(_delta):
	if not (pointcloud.has_new() or visualize_again):
		return
	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PoolVector3Array()
#	var uvs = PoolVector2Array()
#	var normals = PoolVector3Array()
#	var indices = PoolIntArray()

	verts = pointcloud.get_pointcloud("map")

	arr[Mesh.ARRAY_VERTEX] = verts
#	arr[Mesh.ARRAY_TEX_UV] = uvs
	mesh.clear_surfaces()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)
	visualize_again = false
	pointcloud.set_old()

func _on_CheckButton_toggled(button_pressed):

	visible = button_pressed
	if not visible:
		mesh.clear_surfaces()
	elif visible:
		visualize_again = true

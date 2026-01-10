extends MeshInstance3D

var pointcloud = PointCloud.new()
var visualize_again = false

@export var visualize_pointcloud_map_toggle: BaseButton

func _ready():
	pointcloud.subscribe("/map/pointcloud_map", true)	
	visible = visualize_pointcloud_map_toggle.button_pressed

func _process(_delta):
	if not visible:
		return
	if not (pointcloud.has_new() or visualize_again):
		return
	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PackedVector3Array()
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


func _on_point_cloud_map_toggle_toggled(toggled_on):
	visible = toggled_on
	if not visible:
		mesh.clear_surfaces()
	else:
		visualize_again = true

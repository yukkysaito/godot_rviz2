extends MeshInstance3D


@export var obstacle_segmentation_toggle: BaseButton

var pointcloud = PointCloud.new()
var sum_time = 0.0
@export var transparency_speed = 3.0
@export var transparency_scale = 1.0
@export var cycle_time = 3.0

func _ready():
	pointcloud.subscribe("/perception/obstacle_segmentation/pointcloud", false)
	visible = obstacle_segmentation_toggle.button_pressed

func _process(delta):
	if not visible:
		return
	sum_time += delta

	if sum_time * transparency_speed < 2 * PI:
		transparency = clamp((cos(sum_time * transparency_speed) * 0.5 + 0.5) * transparency_scale, 0.0, 1.0)
	else:
		transparency = 1.0
	if (cycle_time < sum_time):
		sum_time = 0.0


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

	if !verts.is_empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)
	pointcloud.set_old()


func _on_enable_obstacle_segmentation_button_toggled(toggled_on):
	visible = toggled_on

extends MeshInstance3D

const MAP_POINTS_SHADER := preload("res://3DViewer/Shaders/pc_map_points.gdshader")

var pointcloud = PointCloud.new()
var visualize_again := false

var _point_material := ShaderMaterial.new()
var _ego_node: Node3D = null

@export var visualize_pointcloud_map_toggle: BaseButton

func _ready():
	pointcloud.subscribe("/map/pointcloud_map", true)
	visible = visualize_pointcloud_map_toggle.button_pressed

	# Replace the scene's StandardMaterial3D with the holographic LiDAR shader.
	_point_material.shader = MAP_POINTS_SHADER
	material_override = _point_material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF

	# Ego vehicle drives the expanding LiDAR pulse ring.
	_ego_node = get_node_or_null("../EgoVehicle") as Node3D

func _process(_delta):
	if not visible:
		return

	# Keep the pulse ring centered on the ego vehicle.
	if _ego_node != null:
		_point_material.set_shader_parameter("ego_pos", _ego_node.global_position)

	if not (pointcloud.has_new() or visualize_again):
		return
	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PackedVector3Array()

	verts = pointcloud.get_pointcloud("map")

	arr[Mesh.ARRAY_VERTEX] = verts
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

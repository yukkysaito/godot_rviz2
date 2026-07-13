extends MeshInstance3D

const OBSTACLE_POINTS_SHADER := preload("res://3DViewer/Shaders/pc_obstacle_points.gdshader")

@export var obstacle_segmentation_toggle: BaseButton

# Export names kept for Main.tscn compatibility (the scene assigns
# transparency_speed / cycle_time). They now drive the shader-side
# emission "breathing" pulse instead of a CPU transparency blink.
@export var transparency_speed := 3.0  # pulse angular speed [rad/s]
@export var transparency_scale := 1.0  # pulse depth (0 = steady, 1 = full breath)
@export var cycle_time := 3.0          # seconds between breaths

var pointcloud = PointCloud.new()
var _point_material := ShaderMaterial.new()

func _ready():
	pointcloud.subscribe("/perception/obstacle_segmentation/pointcloud", false)
	visible = obstacle_segmentation_toggle.button_pressed

	# Replace the scene's StandardMaterial3D with the amber LiDAR shader.
	# All blinking happens on the GPU now.
	_point_material.shader = OBSTACLE_POINTS_SHADER
	_point_material.set_shader_parameter("pulse_speed", transparency_speed)
	_point_material.set_shader_parameter("pulse_depth", clampf(transparency_scale, 0.0, 1.0))
	_point_material.set_shader_parameter("cycle_time", cycle_time)
	material_override = _point_material
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF

func _process(_delta):
	if not visible:
		return

	if !pointcloud.has_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PackedVector3Array()

	verts = pointcloud.get_pointcloud("map")

	arr[Mesh.ARRAY_VERTEX] = verts

	if !verts.is_empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)
	pointcloud.set_old()

func _on_obstacle_segmentation_toggle_toggled(toggled_on):
	visible = toggled_on

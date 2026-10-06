extends MeshInstance3D

# Planned trajectory (a band with flowing chevrons, see Shaders/trajectory.gdshader) and the stop
# wall in front of the vehicle where it plans to stop.

const TRAJECTORY_WIDTH := 2.0  # [m]
const COLOR := Color(0.05, 0.3, 1.0)
const HIGH_CONTRAST_COLOR := Color(0.0, 1.0, 0.0)

var trajectory: Trajectory = RosBridge.trajectory
var wheelbase_to_front: float = VehicleProfile.current().wheelbase_to_front
var high_contrast_enabled: bool = false

var _band_material := ShaderMaterial.new()
var _wall_material: StandardMaterial3D

func _ready():
	_band_material.shader = preload("res://3DViewer/Shaders/trajectory.gdshader")
	_band_material.render_priority = material_override.render_priority if material_override else 0
	_wall_material = material_override as StandardMaterial3D  # vertex colors, see the scene
	material_override = null
	Settings.bind("view/high_contrast", _on_high_contrast_toggle_toggled)

func _on_high_contrast_toggle_toggled(toggled_on):
	high_contrast_enabled = toggled_on
	_band_material.set_shader_parameter("color", HIGH_CONTRAST_COLOR if toggled_on else COLOR)
	_band_material.set_shader_parameter("fill_alpha", 0.6 if toggled_on else 0.22)
	if _wall_material != null:
		_wall_material.distance_fade_mode = (BaseMaterial3D.DISTANCE_FADE_DISABLED if toggled_on
			else BaseMaterial3D.DISTANCE_FADE_PIXEL_ALPHA)

func _process(_delta):
	if !trajectory.has_new():
		return

	# Band: the strip has a left and a right vertex per trajectory point
	var strip = trajectory.get_trajectory_triangle_strip(TRAJECTORY_WIDTH)
	var band_verts := PackedVector3Array()
	var band_normals := PackedVector3Array()
	var band_uvs := PackedVector2Array()
	var along := 0.0
	var previous_center := Vector3.ZERO
	for i in range(0, strip.size() - 1, 2):
		var left: Vector3 = strip[i]["position"]
		var right: Vector3 = strip[i + 1]["position"]
		var center := (left + right) * 0.5
		if i > 0:
			along += center.distance_to(previous_center)
		previous_center = center
		band_verts.append_array([left, right])
		band_normals.append_array([strip[i]["normal"], strip[i + 1]["normal"]])
		band_uvs.append_array([Vector2(0.0, along), Vector2(1.0, along)])
	_band_material.set_shader_parameter("length", along)

	# Wall
	var wall_strip = trajectory.get_wall_triangle_strip(4.0, 2.0, wheelbase_to_front, true, true)
	var wall_verts := PackedVector3Array()
	var wall_normals := PackedVector3Array()
	var wall_colors := PackedColorArray()
	for point in wall_strip:
		wall_verts.append(point["position"])
		wall_normals.append(point["normal"])
		wall_colors.append(HIGH_CONTRAST_COLOR if high_contrast_enabled else point["color"])

	if band_verts.size() >= 4:
		mesh.clear_surfaces()
		var band := []
		band.resize(Mesh.ARRAY_MAX)
		band[Mesh.ARRAY_VERTEX] = band_verts
		band[Mesh.ARRAY_NORMAL] = band_normals
		band[Mesh.ARRAY_TEX_UV] = band_uvs
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, band)
		mesh.surface_set_material(0, _band_material)
		if not wall_verts.is_empty():
			var wall := []
			wall.resize(Mesh.ARRAY_MAX)
			wall[Mesh.ARRAY_VERTEX] = wall_verts
			wall[Mesh.ARRAY_NORMAL] = wall_normals
			wall[Mesh.ARRAY_COLOR] = wall_colors
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, wall)
			mesh.surface_set_material(1, _wall_material)
	trajectory.set_old()

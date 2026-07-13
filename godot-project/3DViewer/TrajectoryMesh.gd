extends MeshInstance3D
## Planned-trajectory ribbon with an animated holographic flow shader, plus a
## hologram-barrier wall at obstacles.
##
## Channel encoding written into the mesh (consumed by the shaders):
##   Trajectory surface (trajectory_flow.gdshader):
##     UV.x    = normalized velocity 0..1
##     UV.y    = cumulative along-path distance in meters
##     COLOR.r = across-ribbon coordinate (0 = left edge, 1 = right edge)
##     COLOR.a = alpha multiplier (1.0)
##   Wall surface (trajectory_wall.gdshader):
##     COLOR   = per-vertex color from the C++ Trajectory module (unchanged)

var trajectory := Trajectory.new()
@export var wheelbase_to_front: float = 3.78
var high_contrast_enabled: bool = false

var _trajectory_material: ShaderMaterial
var _wall_material: ShaderMaterial


func velocity_to_normalized_value(velocity: float) -> float:
	return clamp(velocity / 5.0, 0.0, 1.0)


func _ready() -> void:
	trajectory.subscribe("/planning/trajectory", false)

	_trajectory_material = ShaderMaterial.new()
	_trajectory_material.shader = preload("res://3DViewer/Shaders/trajectory_flow.gdshader")
	_trajectory_material.render_priority = 0

	_wall_material = ShaderMaterial.new()
	_wall_material.shader = preload("res://3DViewer/Shaders/trajectory_wall.gdshader")
	_wall_material.render_priority = 1

	# The two surfaces need different shaders, so clear the scene-wide
	# override (set in Main.tscn) and assign materials per surface instead.
	material_override = null
	_update_high_contrast()


func _update_high_contrast() -> void:
	_trajectory_material.set_shader_parameter("high_contrast", high_contrast_enabled)
	_wall_material.set_shader_parameter("high_contrast", high_contrast_enabled)


func _on_high_contrast_toggle_toggled(toggled_on: bool) -> void:
	high_contrast_enabled = toggled_on
	_update_high_contrast()


func _build_trajectory_arrays(strip: Array) -> Array:
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var colors := PackedColorArray()
	var uvs := PackedVector2Array()

	# The strip alternates left/right edge points; walk it pair-wise and
	# accumulate the distance between successive pair centers.
	var cumulative := 0.0
	var prev_center := Vector3.ZERO
	var has_prev := false
	var i := 0
	while i + 1 < strip.size():
		var p0: Dictionary = strip[i]
		var p1: Dictionary = strip[i + 1]
		var pos0: Vector3 = p0["position"]
		var pos1: Vector3 = p1["position"]
		var center := (pos0 + pos1) * 0.5
		if has_prev:
			cumulative += center.distance_to(prev_center)
		prev_center = center
		has_prev = true

		verts.append(pos0)
		normals.append(p0["normal"])
		colors.append(Color(0.0, 0.0, 0.0, 1.0))
		uvs.append(Vector2(velocity_to_normalized_value(p0["velocity"]), cumulative))

		verts.append(pos1)
		normals.append(p1["normal"])
		colors.append(Color(1.0, 0.0, 0.0, 1.0))
		uvs.append(Vector2(velocity_to_normalized_value(p1["velocity"]), cumulative))
		i += 2

	# Defensive: keep a trailing unpaired point if the strip is odd-sized.
	if i < strip.size():
		var p: Dictionary = strip[i]
		verts.append(p["position"])
		normals.append(p["normal"])
		colors.append(Color(0.0, 0.0, 0.0, 1.0))
		uvs.append(Vector2(velocity_to_normalized_value(p["velocity"]), cumulative))

	return MeshUtils.make_surface_arrays(verts, normals, colors, uvs)


func _process(_delta: float) -> void:
	if !trajectory.has_new():
		return

	# Trajectory ribbon
	var trajectory_triangle_strip: Array = trajectory.get_trajectory_triangle_strip(2.0)
	var traj_arr := _build_trajectory_arrays(trajectory_triangle_strip)

	# Obstacle stop wall (per-vertex colors from C++ kept as-is)
	var wall_triangle_strip: Array = trajectory.get_wall_triangle_strip(
		4.0, 2.0, wheelbase_to_front, true, true)
	var wall_arr := MeshUtils.colored_strip_to_surface_arrays(wall_triangle_strip)

	if MeshUtils.has_vertices(traj_arr):
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, traj_arr)
		mesh.surface_set_material(0, _trajectory_material)
		if MeshUtils.has_vertices(wall_arr):
			mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, wall_arr)
			mesh.surface_set_material(1, _wall_material)
	trajectory.set_old()

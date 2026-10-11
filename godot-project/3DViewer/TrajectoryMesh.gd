extends MeshInstance3D

# Planned trajectory as flowing chevrons showing the planned speed and acceleration
# (Shaders/trajectory.gdshader) and, where the vehicle is planned to stop, a stop wall in front of it
# (Shaders/stop_wall.gdshader).

@export var width: float = 1.8  # [m]
@export var wall_width: float = 2.6  # [m]
@export var wall_height: float = 1.8  # [m] a little above the vehicle, to be seen over it
@export var wall_line_depth: float = 0.25  # [m] the line on the ground at the foot of the wall
@export var wall_margin: float = 0.3  # [m] in front of the vehicle's front when stopped there
@export var stop_speed: float = 0.05  # [m/s] below this a point counts as stopped
@export var real_stop_speed: float = 1.0  # [m/s] see _wall_arrays

const ACCEL_SPAN := 4.0  # [m] the acceleration is averaged over this, so that it changes smoothly

var trajectory: Trajectory = RosBridge.trajectory
var wheelbase_to_front: float = VehicleProfile.current().wheelbase_to_front

var _chevron_material := ShaderMaterial.new()
var _wall_material := ShaderMaterial.new()

func _ready() -> void:
	_chevron_material.shader = preload("res://3DViewer/Shaders/trajectory.gdshader")
	_chevron_material.set_shader_parameter("half_width", width * 0.5)
	_wall_material.shader = preload("res://3DViewer/Shaders/stop_wall.gdshader")
	_wall_material.set_shader_parameter("size", Vector2(wall_width, wall_height))
	Settings.bind("view/high_contrast", func(enabled):
		_chevron_material.set_shader_parameter("high_contrast", enabled)
		_wall_material.set_shader_parameter("high_contrast", enabled))

func _process(_delta: float) -> void:
	if not trajectory.has_new():
		return
	var array_mesh := mesh as ArrayMesh
	array_mesh.clear_surfaces()
	var strip := trajectory.get_trajectory_triangle_strip(width)
	var chevrons := _chevron_arrays(strip)
	if not chevrons.is_empty():
		array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLE_STRIP, chevrons, [], {},
			Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT)
		array_mesh.surface_set_material(array_mesh.get_surface_count() - 1, _chevron_material)
		var wall := _wall_arrays(strip)
		if not wall.is_empty():
			array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, wall)
			array_mesh.surface_set_material(array_mesh.get_surface_count() - 1, _wall_material)
	trajectory.set_old()

# The strip's points come in pairs (right, left) along the trajectory. Per point the shader gets
# (CUSTOM0) the planned speed, the planned time to get there, the acceleration and the length.
func _chevron_arrays(strip: Array) -> Array:
	var count := strip.size() / 2
	if count < 2:
		return []
	var middles := PackedVector3Array()
	var speeds := PackedFloat32Array()
	for i in count:
		middles.append((strip[i * 2]["position"] + strip[i * 2 + 1]["position"]) * 0.5)
		speeds.append(maxf(float(strip[i * 2]["velocity"]), 0.0))
	var distances := PackedFloat32Array([0.0])
	var times := PackedFloat32Array([0.0])
	for i in range(1, count):
		var step := middles[i].distance_to(middles[i - 1])
		distances.append(distances[i - 1] + step)
		var mean_speed := (speeds[i] + speeds[i - 1]) * 0.5
		times.append(times[i - 1] + (step / mean_speed if mean_speed > 0.05 else 0.0))
	var length := distances[count - 1]
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var uvs := PackedVector2Array()
	var custom := PackedFloat32Array()
	for i in count:
		# Acceleration over ACCEL_SPAN around the point: (v1^2 - v0^2) / (2 d)
		var a := i
		var b := i
		while a > 0 and distances[i] - distances[a] < ACCEL_SPAN * 0.5:
			a -= 1
		while b < count - 1 and distances[b] - distances[i] < ACCEL_SPAN * 0.5:
			b += 1
		var span := distances[b] - distances[a]
		var accel := (speeds[b] * speeds[b] - speeds[a] * speeds[a]) / (2.0 * span) if span > 0.01 else 0.0
		for side in 2:
			var point: Dictionary = strip[i * 2 + side]
			vertices.append(point["position"])
			normals.append(point["normal"])
			uvs.append(Vector2(side, distances[i]))
			custom.append_array([speeds[i], times[i], accel, length])
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_CUSTOM0] = custom
	return arrays

# The stop wall in front of the vehicle where it is planned to stop: at the first stopped point
# (speed below stop_speed; Autoware's stopped points are a few mm/s, not exactly 0), past the first
# point. The last point is often stopped only because the trajectory ends there; it counts when
# the speed comes down to it. A line on the ground is added at the foot of the wall.
func _wall_arrays(strip: Array) -> Array:
	var count := strip.size() / 2
	var stop := -1
	for i in range(1, count):
		if float(strip[i * 2]["velocity"]) < stop_speed:
			if i < count - 1 or float(strip[(i - 1) * 2]["velocity"]) < real_stop_speed:
				stop = i
			break
	if stop < 0:
		return []
	var right: Vector3 = strip[stop * 2]["position"]
	var left: Vector3 = strip[stop * 2 + 1]["position"]
	var across := (left - right).normalized()
	var forward := across.cross(Vector3.UP).normalized()
	var base := (left + right) * 0.5 + forward * (wheelbase_to_front + wall_margin)
	var half := across * wall_width * 0.5
	var up := Vector3.UP * wall_height
	var back := -forward * wall_line_depth
	var vertices := PackedVector3Array([
		base - half, base + half, base - half + up, base - half + up, base + half, base + half + up,
		base - half + back, base + half + back, base - half, base - half, base + half + back, base + half])
	var uvs := PackedVector2Array([
		Vector2(0, 0), Vector2(1, 0), Vector2(0, 1), Vector2(0, 1), Vector2(1, 0), Vector2(1, 1),
		Vector2(0, 0), Vector2(1, 0), Vector2(0, 0), Vector2(0, 0), Vector2(1, 0), Vector2(1, 0)])
	var uv2s := PackedVector2Array()
	var normals := PackedVector3Array()
	for i in vertices.size():
		uv2s.append(Vector2(1.0 if i >= 6 else 0.0, 0.0))
		normals.append(-forward if i < 6 else Vector3.UP)
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_TEX_UV2] = uv2s
	return arrays

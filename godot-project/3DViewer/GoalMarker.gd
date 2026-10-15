extends Node3D

# Goal of the route: a map pin above it (Shaders/goal_pin.gdshader), high enough to stay over the
# vehicle when it gets there, and on the ground a ring (Shaders/goal_ground.gdshader) with a low
# curtain of light rising from it, to be seen from low camera angles too
# (Shaders/goal_curtain.gdshader).
# Shown while there is a route; hidden once there is none or the vehicle arrived.

@export var color: Color = Color(0.9, 0.0, 0.07)
@export var pin_height: float = 1.5  # [m] nearby (kept visible on screen far away)
@export var pin_lift: float = 3.0  # [m] of the pin's tip above the goal, along the view's up
@export var ring_radius: float = 0.8  # [m]
@export var curtain_height: float = 0.6  # [m]
@export var ground_offset: float = 0.05  # [m] above the goal, to stay over the road surface
@export var fade_time: float = 0.6  # [s]

var routing_state: NavigationState = RosBridge.routing_state

var _materials: Array[ShaderMaterial] = []
var _visibility := 0.0

func _ready() -> void:
	var quad := QuadMesh.new()
	quad.center_offset = Vector3(0.0, 0.5, 0.0)  # the shader expects the bottom edge at the origin
	var pin := _add_part(quad, preload("res://3DViewer/Shaders/goal_pin.gdshader"), 0.0)
	pin.material_override.set_shader_parameter("height", pin_height)
	pin.material_override.set_shader_parameter("lift", pin_lift)
	# Drawn over everything, so after the other transparent objects
	pin.material_override.render_priority = 1
	# The vertex shader turns the quad to the camera and makes it larger far away
	pin.extra_cull_margin = 16384.0

	var plane := PlaneMesh.new()
	var ground_size := ring_radius * 2.5
	plane.size = Vector2(ground_size, ground_size)
	var ground := _add_part(plane, preload("res://3DViewer/Shaders/goal_ground.gdshader"),
		ground_offset)
	ground.material_override.set_shader_parameter("size", ground_size)
	ground.material_override.set_shader_parameter("radius", ring_radius)

	var cylinder := CylinderMesh.new()
	cylinder.top_radius = ring_radius
	cylinder.bottom_radius = ring_radius
	cylinder.height = curtain_height
	cylinder.cap_top = false
	cylinder.cap_bottom = false
	cylinder.rings = 1
	cylinder.radial_segments = 48
	# Its bottom on the ground (the cylinder is centered on its origin)
	var curtain := _add_part(cylinder, preload("res://3DViewer/Shaders/goal_curtain.gdshader"),
		ground_offset + curtain_height * 0.5)
	curtain.material_override.set_shader_parameter("height", curtain_height)

	visible = false
	position = RosBridge.goal_position
	RosBridge.route_changed.connect(func(): position = RosBridge.goal_position)
	var linear := color.srgb_to_linear()  # the shaders take linear colors
	_set_parameter("color", Vector3(linear.r, linear.g, linear.b))
	Settings.bind("view/high_contrast", func(enabled): _set_parameter("high_contrast", enabled))

func _process(delta: float) -> void:
	# Without a routing state (e.g. an older Autoware) the goal is shown while there is a route
	var shown := (RosBridge.has_route and not routing_state.has_no_route()
		and not routing_state.is_arrived())
	_visibility = move_toward(_visibility, 1.0 if shown else 0.0, delta / fade_time)
	visible = _visibility > 0.0
	if visible:
		_set_parameter("visibility", _visibility)

# Adds a part of the marker drawn with `shader`, `height` above the goal
func _add_part(mesh: Mesh, shader: Shader, height: float) -> MeshInstance3D:
	var material := ShaderMaterial.new()
	material.shader = shader
	_materials.append(material)
	var part := MeshInstance3D.new()
	part.mesh = mesh
	part.position.y = height
	part.material_override = material
	part.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(part)
	return part

func _set_parameter(parameter: String, value: Variant) -> void:
	for material in _materials:
		material.set_shader_parameter(parameter, value)

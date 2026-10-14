extends Node3D

# Goal of the route: a map pin standing on it (Shaders/goal_pin.gdshader) over a dot on the ground
# (Shaders/goal_ground.gdshader).
# Shown while there is a route; hidden once there is none or the vehicle arrived.

@export var pin_height: float = 3.0  # [m] nearby (kept visible on screen far away)
@export var dot_radius: float = 0.8  # [m]
@export var ground_offset: float = 0.05  # [m] above the goal, to stay over the road surface
@export var fade_time: float = 0.6  # [s]

var route: Route = RosBridge.route
var routing_state: NavigationState = RosBridge.routing_state

var _pin_material := ShaderMaterial.new()
var _ground_material := ShaderMaterial.new()
var _visibility := 0.0

func _ready() -> void:
	var quad := QuadMesh.new()
	quad.center_offset = Vector3(0.0, 0.5, 0.0)  # the shader expects the bottom edge at the origin
	_pin_material.shader = preload("res://3DViewer/Shaders/goal_pin.gdshader")
	_pin_material.set_shader_parameter("height", pin_height)
	_pin_material.render_priority = 1  # drawn over everything, so after the other transparent objects
	var pin := MeshInstance3D.new()
	pin.mesh = quad
	pin.material_override = _pin_material
	pin.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	# The vertex shader turns the quad to the camera and makes it larger far away
	pin.extra_cull_margin = 16384.0
	add_child(pin)

	var plane := PlaneMesh.new()
	var ground_size := dot_radius * 2.5
	plane.size = Vector2(ground_size, ground_size)
	_ground_material.shader = preload("res://3DViewer/Shaders/goal_ground.gdshader")
	_ground_material.set_shader_parameter("size", ground_size)
	_ground_material.set_shader_parameter("radius", dot_radius)
	var ground := MeshInstance3D.new()
	ground.mesh = plane
	ground.position.y = ground_offset
	ground.material_override = _ground_material
	ground.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(ground)

	visible = false
	Settings.bind("view/high_contrast", func(enabled):
		_pin_material.set_shader_parameter("high_contrast", enabled)
		_ground_material.set_shader_parameter("high_contrast", enabled))

func _process(delta: float) -> void:
	if route.has_new():
		position = route.get_goal_position()
		route.set_old()
	# Without a routing state (e.g. an older Autoware) the goal is shown while there is a route
	var shown := route.has_route() and not routing_state.has_no_route() and not routing_state.is_arrived()
	_visibility = move_toward(_visibility, 1.0 if shown else 0.0, delta / fade_time)
	visible = _visibility > 0.0
	if visible:
		_pin_material.set_shader_parameter("visibility", _visibility)
		_ground_material.set_shader_parameter("visibility", _visibility)

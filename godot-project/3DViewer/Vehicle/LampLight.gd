extends Node3D

# Light of a lamp (e.g. a turn signal, see VehicleBodyController) spilling around it, so that it
# reads as a light from any view, also from above where the lamp itself is small or hidden:
# - its light on the ground, as from a point light at the lamp's height, fading out under the body
#   (Shaders/lamp_ground_light.gdshader; the road is drawn unlit, so this is drawn on it)
# - a small real light that tints the body around the lamp
# Placed in a vehicle body's scene at the lamp; whatever the frame and scale of the body, the
# ground is the vehicle's (its base_link frame, the parent of the body) and sizes are in meters.

@export var color: Color = Color(1.0, 0.62, 0.12)  # amber
@export var ground_strength: float = 0.12
# Size of the light on the ground, as the height of the point light that would cast it relative to
# the lamp's (< 1: smaller and softer than a bare bulb at the lamp)
@export var ground_spread: float = 0.37
@export var body_light_energy: float = 0.3
@export var body_light_range: float = 0.55  # [m]
@export var fade_in: float = 0.04  # [s]
@export var fade_out: float = 0.12  # [s]

const GROUND_OFFSET := 0.03  # [m] above the ground, to stay over the road surface

var _lit := false
var _brightness := 0.0
var _ground_material := ShaderMaterial.new()
var _body_light := OmniLight3D.new()

func _ready() -> void:
	var base := _base_frame()
	var height := 1.0
	var side := 0.0  # z of the vehicle's side the lamp is on: -1 left, 1 right
	if base != null:
		var in_base := base.global_transform.orthonormalized().affine_inverse() * global_position
		height = maxf(in_base.y, 0.2)
		side = signf(in_base.z)
		# Upright in the vehicle's frame, at the lamp
		global_transform = Transform3D(base.global_transform.orthonormalized().basis, global_position)

	# On the ground below the lamp, wide enough for the light to fade out
	var light_height := height * ground_spread
	var reach := light_height * 3.0
	var plane := PlaneMesh.new()
	plane.size = Vector2.ONE * reach * 2.0
	_ground_material.shader = preload("res://3DViewer/Shaders/lamp_ground_light.gdshader")
	var linear := color.srgb_to_linear()  # the shader takes linear colors
	_ground_material.set_shader_parameter("color", Vector3(linear.r, linear.g, linear.b))
	_ground_material.set_shader_parameter("strength", ground_strength)
	_ground_material.set_shader_parameter("height", light_height)
	_ground_material.set_shader_parameter("side", side)
	var ground := MeshInstance3D.new()
	ground.mesh = plane
	ground.material_override = _ground_material
	ground.position.y = GROUND_OFFSET - height
	ground.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(ground)

	_body_light.light_color = color
	_body_light.omni_range = body_light_range
	_body_light.shadow_enabled = false
	_body_light.position.z = side * 0.15  # just outside the lamp, to light the body around it
	add_child(_body_light)

	Settings.bind("view/high_contrast", func(enabled):
		_ground_material.set_shader_parameter("high_contrast", enabled))
	_apply(0.0)

func set_lit(lit: bool) -> void:
	_lit = lit

func _process(delta: float) -> void:
	var target := 1.0 if _lit else 0.0
	if _brightness != target:
		_apply(move_toward(_brightness, target, delta / (fade_in if _lit else fade_out)))

func _apply(brightness: float) -> void:
	_brightness = brightness
	_ground_material.set_shader_parameter("brightness", brightness)
	_body_light.light_energy = body_light_energy * brightness
	visible = brightness > 0.0

# The vehicle's base_link frame: the parent of the body (VehicleBodyController)
func _base_frame() -> Node3D:
	var node := get_parent()
	while node != null and not node is VehicleBodyController:
		node = node.get_parent()
	return node.get_parent() as Node3D if node != null else null

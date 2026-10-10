extends Node3D
class_name TrafficLightBoard

# The board behind the lamps of a traffic light: one quad drawn as a rounded rectangle (see
# Shaders/signal_board.gdshader).

@export var board_color: Color = Color(0.2, 0.2, 0.2, 1.0)

static var _materials: Dictionary = {}  # color -> ShaderMaterial (shared)

var _quad: MeshInstance3D

func _ready() -> void:
	_build_if_needed()

func _build_if_needed() -> void:
	if _quad != null:
		return
	_quad = MeshInstance3D.new()
	_quad.mesh = QuadMesh.new()
	_quad.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(_quad)
	set_color(board_color)

func set_color(c: Color) -> void:
	board_color = c
	_build_if_needed()
	if not _materials.has(c):
		var material := ShaderMaterial.new()
		material.shader = preload("res://3DViewer/Shaders/signal_board.gdshader")
		var linear := c.srgb_to_linear()
		material.set_shader_parameter("color", Vector3(linear.r, linear.g, linear.b))
		_materials[c] = material
	_quad.material_override = _materials[c]

func set_size(width: float, height: float) -> void:
	_build_if_needed()
	(_quad.mesh as QuadMesh).size = Vector2(width, height)
	_quad.set_instance_shader_parameter("size", Vector2(width, height))

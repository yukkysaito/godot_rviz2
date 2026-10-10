class_name TrafficLightBulb
extends Node3D

# One lamp of a traffic light, drawn procedurally (see Shaders/signal_lamp.gdshader): a crisp
# round lens at any distance, with an arrow for arrow lamps.

# Lamp colors (linear)
const COLORS := {
	"red": Vector3(1.0, 0.04, 0.03),
	"yellow": Vector3(1.0, 0.5, 0.0),
	"green": Vector3(0.0, 1.0, 0.38),
}
const ARROWS := {"none": 0, "left": 1, "right": 2, "up": 3, "down": 4, "up_left": 5, "up_right": 6}

var _lens: MeshInstance3D
var _glow: MeshInstance3D  # only while lit; tells the color from the side too

const GLOW_SIZE := 6.0  # glow diameter / lamp radius

# Materials are shared by all lamps with the same look (color, arrow, lit), so hundreds of
# traffic lights do not create hundreds of materials.
static var _material_cache: Dictionary = {}
static var _glow_materials: Dictionary = {}

var _base_color: String = "none"
var _base_arrow: String = "none"
var _is_lit: bool = false

func _ready() -> void:
	_ensure_built()
	_apply_material()

# -------------------------------------------------------------------
# Public API
# -------------------------------------------------------------------

# Geometry only (safe before entering the tree: no global_* access)
func setup_geometry(pos: Vector3, normal: Vector3, radius: float) -> void:
	_ensure_built()
	(_lens.mesh as QuadMesh).size = Vector2(radius * 2.0, radius * 2.0)
	(_glow.mesh as QuadMesh).size = Vector2(radius * GLOW_SIZE, radius * GLOW_SIZE)
	position = pos
	# Align +Z to the given normal
	var n := normal.normalized()
	var up := Vector3.UP
	if abs(n.dot(up)) > 0.98:
		up = Vector3.RIGHT
	basis = Basis.looking_at(n, up)

# Identity only (color / arrow)
func setup_identity(color: String, arrow: String) -> void:
	_base_color = color
	_base_arrow = arrow
	_apply_material()

# State only (on / off)
func set_lit(on: bool) -> void:
	if on != _is_lit:
		_is_lit = on
		_apply_material()

func turn_off() -> void:
	set_lit(false)

func setup_from_hdmap(pos: Vector3, normal: Vector3, radius: float, color: String, arrow: String) -> void:
	setup_geometry(pos, normal, radius)
	setup_identity(color, arrow)
	set_lit(false)

# -------------------------------------------------------------------
# Internal
# -------------------------------------------------------------------

func _ensure_built() -> void:
	if _lens != null:
		return
	_lens = MeshInstance3D.new()
	_lens.name = "Lens"
	var quad := QuadMesh.new()
	quad.size = Vector2(0.3, 0.3)
	_lens.mesh = quad
	_lens.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(_lens)
	_glow = MeshInstance3D.new()
	_glow.name = "Glow"
	var glow_quad := QuadMesh.new()
	glow_quad.size = Vector2(0.6, 0.6)
	_glow.mesh = glow_quad
	_glow.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_glow.visible = false
	add_child(_glow)

func _apply_material() -> void:
	_ensure_built()
	var key := "%s|%s|%s" % [_base_color, _base_arrow, _is_lit]
	if not _material_cache.has(key):
		var material := ShaderMaterial.new()
		material.shader = preload("res://3DViewer/Shaders/signal_lamp.gdshader")
		material.set_shader_parameter("color", COLORS.get(_base_color, Vector3(0.6, 0.6, 0.6)))
		material.set_shader_parameter("arrow", ARROWS.get(_base_arrow, 0))
		material.set_shader_parameter("lit", _is_lit)
		_material_cache[key] = material
	_lens.material_override = _material_cache[key]
	_glow.visible = _is_lit
	if _is_lit:
		if not _glow_materials.has(_base_color):
			var glow := ShaderMaterial.new()
			glow.shader = preload("res://3DViewer/Shaders/signal_glow.gdshader")
			glow.set_shader_parameter("color", COLORS.get(_base_color, Vector3(0.6, 0.6, 0.6)))
			_glow_materials[_base_color] = glow
		_glow.material_override = _glow_materials[_base_color]

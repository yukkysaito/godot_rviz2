class_name TrafficLightBulb
extends Node3D

# Textures for each (color, arrow) combination.
# "none:none" is used as a safe fallback.
const TEX := {
	"none:none":   preload("res://3DViewer/TrafficLight/Textures/traffic_light_black.png"),
	"red:none":    preload("res://3DViewer/TrafficLight/Textures/traffic_light_red.png"),
	"yellow:none": preload("res://3DViewer/TrafficLight/Textures/traffic_light_yellow.png"),
	"green:none":  preload("res://3DViewer/TrafficLight/Textures/traffic_light_green.png"),
	"green:left":  preload("res://3DViewer/TrafficLight/Textures/traffic_light_green_left.png"),
	"green:right": preload("res://3DViewer/TrafficLight/Textures/traffic_light_green_right.png"),
	"green:up":    preload("res://3DViewer/TrafficLight/Textures/traffic_light_green_up.png"),
	"green:down":  preload("res://3DViewer/TrafficLight/Textures/traffic_light_green_down.png"),
}

static func _make_key(color: String, arrow: String) -> String:
	return "%s:%s" % [color, arrow]

@export var emission_power: float = 20.0
@export var lens_tint: Color = Color(0.15, 0.15, 0.15, 1.0)
@export var double_sided: bool = false

# Local Z offsets to avoid z-fighting between lens/glow.
const Z_LENS: float = 0.000
const Z_GLOW: float = 0.005

# Scene nodes
var _lens: MeshInstance3D
var _glow: MeshInstance3D

# Materials (reused, do NOT recreate every frame)
var _lens_mat: StandardMaterial3D
var _glow_mat: StandardMaterial3D

# Identity (which bulb type this is in HDMap)
var _base_color: String = "none"
var _base_arrow: String = "none"

# State
var _is_lit: bool = false


func _ready() -> void:
	_ensure_built()
	# Default appearance: dark lens, glow off
	_apply_lens_appearance()
	_apply_lit(false)


# -------------------------------------------------------------------
# Public API (recommended)
# -------------------------------------------------------------------

# 1) Geometry only (safe before entering tree: no global_* access)
func setup_geometry(pos: Vector3, normal: Vector3, radius: float) -> void:
	_ensure_built()

	# Set size (diameter)
	var d: float = radius * 2.0
	(_lens.mesh as PlaneMesh).size = Vector2(d, d)
	(_glow.mesh as PlaneMesh).size = Vector2(d, d)

	# Local position
	position = pos

	# Local orientation: align +Z to the given normal
	var n := normal.normalized()
	var up := Vector3.UP
	if abs(n.dot(up)) > 0.98:
		up = Vector3.RIGHT
	basis = Basis.looking_at(n, up)

# 2) Identity only (color/arrow). Updates lens texture immediately.
func setup_identity(color: String, arrow: String) -> void:
	_base_color = color
	_base_arrow = arrow
	_apply_lens_appearance()

	# If currently lit, refresh glow texture to match identity.
	if _is_lit:
		_apply_glow_texture()

# 3) State only (on/off). Does NOT change identity.
func set_lit(on: bool) -> void:
	_apply_lit(on)

# Compatibility with your previous calls
func turn_off() -> void:
	_apply_lit(false)

# Convenience: old "one-shot" setup (kept for compatibility)
func setup_from_hdmap(pos: Vector3, normal: Vector3, radius: float, color: String, arrow: String) -> void:
	setup_geometry(pos, normal, radius)
	setup_identity(color, arrow)
	_apply_lit(false)


# -------------------------------------------------------------------
# Internal
# -------------------------------------------------------------------

func _ensure_built() -> void:
	if _lens != null:
		return

	# --- Lens mesh ---
	_lens = MeshInstance3D.new()
	_lens.name = "Lens"
	var lens_mesh := PlaneMesh.new()
	lens_mesh.orientation = PlaneMesh.FACE_Z
	_lens.mesh = lens_mesh
	_lens.position.z = Z_LENS

	_lens_mat = StandardMaterial3D.new()
	_lens_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_lens_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_lens_mat.albedo_color = lens_tint
	if double_sided:
		_lens_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	_lens.material_override = _lens_mat

	add_child(_lens)

	# --- Glow mesh ---
	_glow = MeshInstance3D.new()
	_glow.name = "Glow"
	var glow_mesh := PlaneMesh.new()
	glow_mesh.orientation = PlaneMesh.FACE_Z
	_glow.mesh = glow_mesh
	_glow.position.z = Z_GLOW
	_glow.visible = false

	_glow_mat = StandardMaterial3D.new()
	_glow_mat.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL
	_glow_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	_glow_mat.emission_enabled = true
	_glow_mat.emission_energy_multiplier = emission_power
	if double_sided:
		_glow_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	_glow.material_override = _glow_mat

	add_child(_glow)

func _apply_lit(on: bool) -> void:
	_ensure_built()
	_is_lit = on

	if not on:
		_glow.visible = false
		return

	_apply_glow_texture()
	_glow.visible = true

func _apply_lens_appearance() -> void:
	_ensure_built()
	var tex: Texture2D = _get_tex(_base_color, _base_arrow)
	_lens_mat.albedo_texture = tex
	# Darken by tint (keeps "off" look)
	_lens_mat.albedo_color = lens_tint

func _apply_glow_texture() -> void:
	var tex: Texture2D = _get_tex(_base_color, _base_arrow)
	_glow_mat.albedo_texture = tex
	_glow_mat.emission_texture = tex

func _get_tex(color: String, arrow: String) -> Texture2D:
	var k := TrafficLightBulb._make_key(color, arrow) # call static properly
	if TEX.has(k):
		return TEX[k]
	return TEX["none:none"]

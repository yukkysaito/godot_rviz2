extends Node

@export var world_env: WorldEnvironment
@export var sun: DirectionalLight3D
@export var night_sky_top_color: Color = Color(0.0, 0.0, 0.0, 1.0)

var _default_sky_top_color: Color
var _sky_material: Material

func _ready():
	if world_env == null:
		world_env = $WorldEnv as WorldEnvironment
	if sun == null:
		sun = $Sun as DirectionalLight3D
	_sky_material = world_env.environment.sky.sky_material
	if _sky_material != null and _sky_material.has_method("get_sky_top_color"):
		_default_sky_top_color = _sky_material.call("get_sky_top_color")

func set_night_mode(enabled: bool) -> void:
	# Sun
	if sun:
		sun.visible = not enabled

	# Sky top color
	if _sky_material == null:
		return
	if _sky_material.has_method("set_sky_top_color"):
		if enabled:
			_sky_material.call("set_sky_top_color", night_sky_top_color)
		else:
			_sky_material.call("set_sky_top_color", _default_sky_top_color)

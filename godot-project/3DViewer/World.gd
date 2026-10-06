extends Node

@export var world_env: WorldEnvironment
@export var sun: DirectionalLight3D
@export var night_sky_top_color: Color = Color(0.0, 0.0, 0.0, 1.0)

# At night the ground (the lower half of the sky) gets darker by this factor
@export var night_ground_brightness: float = 0.35

var _sky_material: Material
var _day_colors := {}  # property -> day color, for the sky colors changed at night

func _ready():
	if world_env == null:
		world_env = $WorldEnv as WorldEnvironment
	if sun == null:
		sun = $Sun as DirectionalLight3D
	_sky_material = world_env.environment.sky.sky_material
	for property in ["sky_top_color", "ground_bottom_color", "ground_horizon_color"]:
		if property in _sky_material:
			_day_colors[property] = _sky_material.get(property)
	Settings.bind("view/day_mode", func(mode): set_night_mode(mode == "night"))

func set_night_mode(enabled: bool) -> void:
	# Sun
	if sun:
		sun.visible = not enabled

	# Sky colors
	for property in _day_colors:
		var color: Color = _day_colors[property]
		if enabled:
			if property == "sky_top_color":
				color = night_sky_top_color
			else:
				color = Color(color * night_ground_brightness, color.a)
		_sky_material.set(property, color)

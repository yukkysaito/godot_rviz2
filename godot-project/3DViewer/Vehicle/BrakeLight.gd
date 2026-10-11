extends MeshInstance3D

var night_light = false
var night_emission_energey = 0.2
var brake_emission_energy = 6.0

# A see-through lens shows only its alpha of the glow, so the glow is scaled up to look the same
var _glow_scale := 1.0

func _ready():
	var material := get_surface_override_material(0) as BaseMaterial3D
	if material != null and material.transparency != BaseMaterial3D.TRANSPARENCY_DISABLED:
		_glow_scale = 1.0 / maxf(material.albedo_color.a, 0.05)
	turn_off()

func night_light_turn_on():
	night_light = true
	_set_glow(night_emission_energey)

func night_light_turn_off():
	night_light = false
	_set_glow(0.0)

func turn_on():
	_set_glow(brake_emission_energy)

func turn_off():
	if night_light:
		_set_glow(night_emission_energey)
	else:
		_set_glow(0.0)

func _set_glow(energy: float) -> void:
	get_surface_override_material(0).emission_energy_multiplier = energy * _glow_scale

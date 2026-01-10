extends MeshInstance3D

var night_light = false
var night_emission_energey = 0.2

func _ready():
	turn_off()

func night_light_turn_on():
	night_light = true
	get_surface_override_material(0).emission_energy = night_emission_energey

func night_light_turn_off():
	night_light = false
	get_surface_override_material(0).emission_energy = 0.0

func turn_on():
	get_surface_override_material(0).emission_energy = 6

func turn_off():
	if night_light:
		get_surface_override_material(0).emission_energy = night_emission_energey
	else:
		get_surface_override_material(0).emission_energy = 0.0


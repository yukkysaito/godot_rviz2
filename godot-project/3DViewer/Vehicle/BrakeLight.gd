extends MeshInstance3D

var night_light = false
var default_emission_energey = 0.0

func _ready():
	turn_off()

func night_light_turn_on():
	default_emission_energey = 0.2
	get_surface_override_material(0).emission_energy = default_emission_energey

func night_light_turn_off():
	default_emission_energey = 0
	get_surface_override_material(0).emission_energy = default_emission_energey

func turn_on():
	get_surface_override_material(0).emission_energy = 6

func turn_off():
	get_surface_override_material(0).emission_energy = default_emission_energey

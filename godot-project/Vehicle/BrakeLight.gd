extends MeshInstance

var night_light = false
var default_emission_energey = 0.0

func _ready():
	turn_off()

func night_light_turn_on():
	default_emission_energey = 0.3
	self.mesh.surface_get_material(0).emission_energy = default_emission_energey

func night_light_turn_off():
	default_emission_energey = 0
	self.mesh.surface_get_material(0).emission_energy = default_emission_energey

func turn_on():
	self.mesh.surface_get_material(0).emission_energy = 16

func turn_off():
	self.mesh.surface_get_material(0).emission_energy = default_emission_energey

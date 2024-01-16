extends MeshInstance3D

func _ready():
	turn_off()
	pass

func turn_on():
	get_surface_override_material(0).emission_enabled = true
	
func turn_off():
	get_surface_override_material(0).emission_enabled = false

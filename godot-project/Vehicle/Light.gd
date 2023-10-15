extends MeshInstance

func _ready():
	turn_off()
	pass

func turn_on():
	self.mesh.surface_get_material(0).emission_enabled = true
	
func turn_off():
	self.mesh.surface_get_material(0).emission_enabled = false

extends MeshInstance

func _ready():
	turn_off()

func turn_on():
	self.get_surface_material(0).emission_enabled = true
	
func turn_off():
	self.get_surface_material(0).emission_enabled = false

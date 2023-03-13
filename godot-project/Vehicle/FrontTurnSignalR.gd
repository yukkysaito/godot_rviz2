extends MeshInstance

var is_turn_on = false

var sum_time = 0.0

func _ready():
	turn_off()

func turn_on():
	is_turn_on = true;

func turn_off():
	is_turn_on = false;

func _process(delta):
	if !is_turn_on:
		self.get_surface_material(0).emission_enabled = false
		sum_time = 0
		return

	sum_time += delta
	if (sum_time < 0.5):
		self.get_surface_material(0).emission_enabled = true
	elif (sum_time < 1.0):
		self.get_surface_material(0).emission_enabled = false
	else:
		sum_time = 0

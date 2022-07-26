extends SpotLight

var is_turn_on = false

var sum_time = 0.0

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

func turn_on():
	is_turn_on = true;

func turn_off():
	is_turn_on = false;

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	if !is_turn_on:
		visible = false
		sum_time = 0
		return

	sum_time += delta
	if (sum_time < 0.5):
		visible = true
	elif (sum_time < 1.0):
		visible = false
	else:
		sum_time = 0

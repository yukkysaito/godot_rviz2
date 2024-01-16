extends WorldEnvironment


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass


func _on_night_mode_check_button_toggled(toggled_on):
	if (toggled_on):
		environment.sky.sky_material.set_sky_top_color(Color(0.0,0.0,0.0,1.0))
	else:
		environment.sky.sky_material.set_sky_top_color(Color(1.0,1.0,1.0,1.0))

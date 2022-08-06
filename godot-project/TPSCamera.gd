extends Camera

func _ready():
	pass

func _process(_delta):
	pass

func _on_NightModeCheckButton_toggled(button_pressed):
	if (button_pressed):
		environment.background_sky.set_sky_top_color(Color(0.2,0.2,0.2,1))
	else:
		environment.background_sky.set_sky_top_color(Color(1,1,1,1))
		

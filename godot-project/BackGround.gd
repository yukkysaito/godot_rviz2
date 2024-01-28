extends WorldEnvironment

func _on_night_mode_check_button_toggled(toggled_on):
	if (toggled_on):
		environment.sky.sky_material.set_sky_top_color(Color(0.0,0.0,0.0,1.0))
	else:
		environment.sky.sky_material.set_sky_top_color(Color(1.0,1.0,1.0,1.0))

extends DirectionalLight3D

func _on_night_mode_check_button_toggled(toggled_on):
	if (toggled_on):
		visible = false
	else:
		visible = true

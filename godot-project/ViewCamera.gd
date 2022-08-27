extends Camera

var view_mode = [
	["tpv", Vector3(-18, 12, 0), Vector3(deg2rad(-32), deg2rad(-90), deg2rad(0))], 
	["bev", Vector3(0, 100, 0), Vector3(deg2rad(-90), deg2rad(0), deg2rad(0))]]

var current_view_mode = 0
func _ready():
	pass

func _process(_delta):
	pass

func _on_NightModeCheckButton_toggled(button_pressed):
	if (button_pressed):
		environment.background_sky.set_sky_top_color(Color(0.2,0.2,0.2,1))
	else:
		environment.background_sky.set_sky_top_color(Color(1,1,1,1))

func _input(_event):
	if Input.is_action_just_pressed("ui_view_switch"):
		current_view_mode = (current_view_mode + 1) % view_mode.size()
		set_translation(view_mode[current_view_mode][1])
		set_rotation(view_mode[current_view_mode][2])



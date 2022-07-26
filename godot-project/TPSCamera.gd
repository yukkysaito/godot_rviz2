extends Camera


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	pass


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass


func _on_NightModeCheckButton_toggled(button_pressed):
	if (button_pressed):
		environment.background_sky.set_sky_top_color(Color(0.2,0.2,0.2,1))
	else:
		environment.background_sky.set_sky_top_color(Color(1,1,1,1))
		

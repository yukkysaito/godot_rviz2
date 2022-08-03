extends Spatial

func _ready():
	visible = false

func _on_NightModeCheckButton_toggled(button_pressed):
	visible = button_pressed

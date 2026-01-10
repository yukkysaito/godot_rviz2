extends Button

func _ready() -> void:
	pressed.connect(_on_pressed)
	focus_mode = Control.FOCUS_NONE

func _on_pressed() -> void:
	get_tree().quit()

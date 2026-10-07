extends CheckButton
class_name SettingToggle

# A toggle that shows and changes a boolean setting (see Core/Settings.gd).

@export var key: String

func _ready() -> void:
	Settings.bind(key, func(value): set_pressed_no_signal(bool(value)))
	toggled.connect(func(on: bool): Settings.set_value(key, on))

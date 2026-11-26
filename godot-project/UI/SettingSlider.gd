extends HBoxContainer
class_name SettingSlider

# A slider that shows and changes a number setting (see Core/Settings.gd), with its value beside it
# as value_format of the value times value_scale (e.g. "%.0f%%" and 100 for a ratio). A double click
# on it brings the setting back to its default.

@export var key: String
@export var min_value: float = 0.0
@export var max_value: float = 1.0
@export var step: float = 0.05
@export var value_format: String = "%.2f"
@export var value_scale: float = 1.0
@export var slider_width: float = 110.0  # [px]

var _slider := HSlider.new()
var _value_label := Label.new()

func _ready() -> void:
	_slider.min_value = min_value
	_slider.max_value = max_value
	_slider.step = step
	_slider.focus_mode = Control.FOCUS_NONE
	_slider.tooltip_text = "Double-click to reset"
	_slider.custom_minimum_size.x = slider_width
	_slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	add_child(_slider)
	_value_label.custom_minimum_size.x = 44
	_value_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	add_child(_value_label)
	Settings.bind(key, func(value):
		_slider.set_value_no_signal(float(value))
		_show_value(float(value)))
	_slider.value_changed.connect(func(value: float):
		_show_value(value)
		Settings.set_value(key, value))
	_slider.gui_input.connect(_on_slider_input)

func _on_slider_input(event: InputEvent) -> void:
	var button := event as InputEventMouseButton
	if button != null and button.double_click and button.button_index == MOUSE_BUTTON_LEFT:
		Settings.reset_value(key)
		_slider.accept_event()  # the slider does not move to the click

func _show_value(value: float) -> void:
	_value_label.text = value_format % (value * value_scale)

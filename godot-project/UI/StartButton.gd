# StartButton.gd
extends PressColorAnimButton

## Subtle idle "breathing" pulse (scale) so the button invites a press.
@export var idle_pulse_enabled: bool = true
@export_range(1.0, 1.2, 0.005) var idle_pulse_scale: float = 1.025
@export_range(0.2, 5.0, 0.1) var idle_pulse_half_period_sec: float = 1.1

var operation_mode_changer := OperationModeChanger.new()
var _pulse_tween: Tween

func _ready() -> void:
	operation_mode_changer.create_client("/api/operation_mode/change_to_autonomous")
	super._ready()
	pressed.connect(_on_pressed)

	resized.connect(_center_pivot)
	_center_pivot()
	if idle_pulse_enabled:
		_start_idle_pulse()

func _on_pressed() -> void:
	operation_mode_changer.change_to_autonomous_mode()

func _center_pivot() -> void:
	pivot_offset = size * 0.5

func _start_idle_pulse() -> void:
	if _pulse_tween != null and _pulse_tween.is_valid():
		_pulse_tween.kill()
	_pulse_tween = create_tween().set_loops().set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)
	_pulse_tween.tween_property(self, "scale", Vector2.ONE * idle_pulse_scale, idle_pulse_half_period_sec)
	_pulse_tween.tween_property(self, "scale", Vector2.ONE, idle_pulse_half_period_sec)

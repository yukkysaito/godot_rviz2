extends Control

@export var vehicle_actuator_status_toggle: BaseButton

func _ready():
	visible = vehicle_actuator_status_toggle.button_pressed

func _on_vehicle_actuator_atatus_toggle_toggled(toggled_on):
	visible = toggled_on

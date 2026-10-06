extends Control

func _ready():
	Settings.bind("hud/vehicle_actuator_status", set_visible)

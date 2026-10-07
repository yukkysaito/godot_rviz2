extends Node3D
class_name VehicleBodyController

# Interface of a vehicle body (the root of a vehicle model scene, see VehicleProfile).
# Each vehicle implements it in its own script (extends VehicleBodyController, without a
# class_name), so that several vehicles can live in one project.

@export var tire_radius: float = 0.378

func set_night_mode(_enabled: bool) -> void:
	pass

# Rotates the wheels for a travelled distance [m]
func rotate_wheels_by_distance(_move_delta: float) -> void:
	pass

# Steers the front wheels [rad]
func set_steering_angle(_angle: float) -> void:
	pass

func turn_on_right_signal() -> void:
	pass

func turn_off_right_signal() -> void:
	pass

func turn_on_left_signal() -> void:
	pass

func turn_off_left_signal() -> void:
	pass

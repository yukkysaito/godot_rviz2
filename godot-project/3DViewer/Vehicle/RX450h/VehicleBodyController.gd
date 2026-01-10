extends Node3D
class_name VehicleBodyController

@export var tire_radius: float = 0.378

# Wheels
@export var wheel_back_r_path: NodePath
@export var wheel_back_l_path: NodePath
@export var wheel_front_r_path: NodePath
@export var wheel_front_l_path: NodePath

# Turn signals
@export var turn_signal_r_path: NodePath
@export var turn_signal_r_small_path: NodePath
@export var turn_signal_l_path: NodePath
@export var turn_signal_l_small_path: NodePath

# Night lights
@export var head_light_path: NodePath
@export var head_small_light_path: NodePath
@export var fog_light_path: NodePath
@export var brake_light_path: NodePath
@export var head_beam_light_path: NodePath

# --- Cached nodes ---
@onready var wheel_back_r: Node3D = get_node(wheel_back_r_path) as Node3D
@onready var wheel_back_l: Node3D = get_node(wheel_back_l_path) as Node3D
@onready var wheel_front_r: Node3D = get_node(wheel_front_r_path) as Node3D
@onready var wheel_front_l: Node3D = get_node(wheel_front_l_path) as Node3D

@onready var turn_signal_r: Node = get_node(turn_signal_r_path)
@onready var turn_signal_r_small: Node = get_node(turn_signal_r_small_path)
@onready var turn_signal_l: Node = get_node(turn_signal_l_path)
@onready var turn_signal_l_small: Node = get_node(turn_signal_l_small_path)

@onready var head_light: Node = get_node(head_light_path)
@onready var head_small_light: Node = get_node(head_small_light_path)
@onready var fog_light: Node = get_node(fog_light_path)
@onready var brake_light: Node = get_node(brake_light_path)

func _ready() -> void:
	# setup once
	wheel_front_r.rotation_order = EULER_ORDER_ZYX
	wheel_front_l.rotation_order = EULER_ORDER_ZYX

# -----------------------
# Public API (requested)
# -----------------------

func set_night_mode(enabled: bool) -> void:
	if enabled:
		head_light.call("turn_on")
		head_small_light.call("turn_on")
		fog_light.call("turn_on")
		brake_light.call("night_light_turn_on")
	else:
		head_light.call("turn_off")
		head_small_light.call("turn_off")
		fog_light.call("turn_off")
		brake_light.call("night_light_turn_off")

func rotate_wheels_by_distance(move_delta: float) -> void:
	if tire_radius <= 0.0:
		return
	var rotation_delta := move_delta / tire_radius
	wheel_back_r.rotate_x(rotation_delta)
	wheel_back_l.rotate_x(rotation_delta)
	wheel_front_r.rotate_x(rotation_delta)
	wheel_front_l.rotate_x(rotation_delta)

func set_steering_angle(angle: float) -> void:
	wheel_front_r.rotation = Vector3(wheel_front_r.rotation.x, angle, 0.0)
	wheel_front_l.rotation = Vector3(wheel_front_l.rotation.x, angle, 0.0)

func turn_on_right_signal() -> void:
	turn_signal_r.call("turn_on")
	turn_signal_r_small.call("turn_on")

func turn_off_right_signal() -> void:
	turn_signal_r.call("turn_off")
	turn_signal_r_small.call("turn_off")

func turn_on_left_signal() -> void:
	turn_signal_l.call("turn_on")
	turn_signal_l_small.call("turn_on")

func turn_off_left_signal() -> void:
	turn_signal_l.call("turn_off")
	turn_signal_l_small.call("turn_off")

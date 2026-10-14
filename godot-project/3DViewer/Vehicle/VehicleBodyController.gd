extends Node3D
class_name VehicleBodyController

# Interface of a vehicle body (the root of a vehicle model scene, see VehicleProfile).
# Each vehicle implements it in its own script (extends VehicleBodyController, without a
# class_name), so that several vehicles can live in one project.
#
# The turn signals blink here, with one clock for both sides (they blink together as hazard
# lights), lighting the nodes of the body's scene listed in left_turn_signals / right_turn_signals:
# its lamps (Light.gd) and the light they spill around (LampLight.gd). A body has no code of its own
# for them.

enum Side { LEFT, RIGHT }

@export var tire_radius: float = 0.0  # [m] set by each vehicle
@export var turn_signal_period: float = 1.0  # [s] of a blink, lit for its first half
# Nodes lit with each turn signal (they have set_lit())
@export var left_turn_signals: Array[NodePath] = []
@export var right_turn_signals: Array[NodePath] = []

var _signal_on := [false, false]  # per Side
var _signal_lit := [false, false]
var _blink_time := 0.0  # [s] into the current blink
@onready var _turn_signals := [_nodes(left_turn_signals), _nodes(right_turn_signals)]

func set_night_mode(_enabled: bool) -> void:
	pass

# Rotates the wheels for a travelled distance [m]
func rotate_wheels_by_distance(_move_delta: float) -> void:
	pass

# Steers the front wheels [rad]
func set_steering_angle(_angle: float) -> void:
	pass

func turn_on_right_signal() -> void:
	_set_signal(Side.RIGHT, true)

func turn_off_right_signal() -> void:
	_set_signal(Side.RIGHT, false)

func turn_on_left_signal() -> void:
	_set_signal(Side.LEFT, true)

func turn_off_left_signal() -> void:
	_set_signal(Side.LEFT, false)

# Blinks also when the body's script has its own _process
func _notification(what: int) -> void:
	if what == NOTIFICATION_READY:
		set_process(true)
	elif what == NOTIFICATION_PROCESS:
		_update_signals(get_process_delta_time())

func _set_signal(side: int, on: bool) -> void:
	if on and not _signal_on.has(true):
		_blink_time = 0.0  # starts lit
	_signal_on[side] = on
	_update_signals(0.0)

func _update_signals(delta: float) -> void:
	if _signal_on.has(true):
		_blink_time = fmod(_blink_time + delta, turn_signal_period)
	var lit_phase := _blink_time < turn_signal_period * 0.5
	for side in [Side.LEFT, Side.RIGHT]:
		var lit: bool = _signal_on[side] and lit_phase
		if lit == _signal_lit[side]:
			continue
		_signal_lit[side] = lit
		for node in _turn_signals[side]:
			node.set_lit(lit)

func _nodes(paths: Array[NodePath]) -> Array[Node]:
	var nodes: Array[Node] = []
	for path in paths:
		nodes.append(get_node(path))
	return nodes

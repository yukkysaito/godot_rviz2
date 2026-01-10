extends Node3D

@export var vehicle_body_path: NodePath
@onready var vehicle_body: VehicleBodyController = get_node(vehicle_body_path) as VehicleBodyController

@export var head_beam_light_path: NodePath
@onready var head_beam_light: Node3D = get_node(head_beam_light_path) as Node3D


var ego_pose = EgoPose.new()
var vehicle_status = VehicleStatus.new()
var velocity_report = VelocityReport.new()
var steering_report = SteeringReport.new()

func _ready():
	vehicle_status.subscribe("/vehicle/status/turn_indicators_status", false)
	velocity_report.subscribe("/vehicle/status/velocity_status", false)
	steering_report.subscribe("/vehicle/status/steering_status", false)

func _process(delta):
	# Ego pose
	set_position(ego_pose.get_ego_position())
	set_rotation(ego_pose.get_ego_rotation())
	
	# Tire rotation
	if(velocity_report.has_new()):
		var current_speed  = velocity_report.get_velocity()
		vehicle_body.rotate_wheels_by_distance(current_speed * delta)
		velocity_report.set_old()
	if(steering_report.has_new()):
		vehicle_body.set_steering_angle(steering_report.get_angle())
		steering_report.set_old()

	# Indicators
	if(vehicle_status.has_new()):
		if (vehicle_status.is_turn_on_right()):
			vehicle_body.turn_on_right_signal()
		else:
			vehicle_body.turn_off_right_signal()

		if (vehicle_status.is_turn_on_left()):
			vehicle_body.turn_on_left_signal()
		else:
			vehicle_body.turn_off_left_signal()
		vehicle_status.set_old()

func set_night_mode(enabled: bool) -> void:
	vehicle_body.set_night_mode(enabled)
	head_beam_light.visible = enabled

extends Node3D

@export var tire_radius: float = 0.378

var ego_pose = EgoPose.new()
var vehicle_status = VehicleStatus.new()
var velocity_report = VelocityReport.new()
var steering_report = SteeringReport.new()

# Wheel nodes
var wheel_back_r: Node3D
var wheel_back_l: Node3D
var wheel_front_r: Node3D
var wheel_front_l: Node3D

func _ready():
	vehicle_status.subscribe("/vehicle/status/turn_indicators_status", false)
	velocity_report.subscribe("/vehicle/status/velocity_status", false)
	steering_report.subscribe("/vehicle/status/steering_status", false)
	
	wheel_back_r = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelBackR")
	wheel_back_l = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelBackL")
	wheel_front_r = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelFrontR")
	wheel_front_l = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelFrontL")


func _process(delta):
	# Ego pose
	set_position(ego_pose.get_ego_position())
	set_rotation(ego_pose.get_ego_rotation())
	
	# Tire rotation
	if(velocity_report.has_new()):
		var current_speed  = velocity_report.get_velocity()
		rotate_wheels(current_speed * delta)
		velocity_report.set_old()
	if(steering_report.has_new()):
		rotate_steerings()
		steering_report.set_old()

	# Indicators
	if(vehicle_status.has_new()):
		var right_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/TurnSignalR")
		var right_small_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/BodyCar/TurnSignalR_002")
		if (vehicle_status.is_turn_on_right()):
			right_light.turn_on()
			right_small_light.turn_on()
		else:
			right_light.turn_off()
			right_small_light.turn_off()

		var left_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/TurnSignalL")
		if (vehicle_status.is_turn_on_left()):
			left_light.turn_on()
		else:
			left_light.turn_off()
		vehicle_status.set_old()
		
# Rotate the wheels based on the current speed
func rotate_wheels(move_delta):
	var rotation_delta = (move_delta / tire_radius) 

	wheel_back_r.rotate_x(rotation_delta)
	wheel_back_l.rotate_x(rotation_delta)
	wheel_front_r.rotate_x(rotation_delta)
	wheel_front_l.rotate_x(rotation_delta)

func rotate_steerings():
	var angle = steering_report.get_angle()
	var current_wheel_front_r_rotation = wheel_front_r.rotation
	var current_wheel_front_l_rotation = wheel_front_l.rotation
	wheel_front_r.rotation_order = EULER_ORDER_ZYX
	wheel_front_l.rotation_order = EULER_ORDER_ZYX
	wheel_front_r.rotation = Vector3(current_wheel_front_r_rotation.x, angle, 0)
	wheel_front_l.rotation = Vector3(current_wheel_front_l_rotation.x, angle, 0)

func _on_night_mode_check_button_toggled(toggled_on):
	var head_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/HeadLight")
	var head_small_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/BodyCar/HeadLight_002")
	var fog_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/FogLight")
	var brake_light = get_node("EgoVehicleKinematicBody/VehicleBody3D/BreakLight")
	var head_beam_light = get_node("EgoVehicleKinematicBody/HeadBeamLight")
	if (toggled_on):
		head_light.turn_on()
		head_small_light.turn_on()
		fog_light.turn_on()
		brake_light.night_light_turn_on()
		head_beam_light.turn_on()
	else:
		head_light.turn_off()
		head_small_light.turn_off()
		fog_light.turn_off()
		brake_light.night_light_turn_off()
		head_beam_light.turn_off()

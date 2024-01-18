extends Node3D

var ego_pose = EgoPose.new()
var vehicle_status = VehicleStatus.new()
var velocity_report = VelocityReport.new()

# Wheel nodes
var wheel_back_r: Node3D
var wheel_back_l: Node3D
var wheel_front_r: Node3D
var wheel_front_l: Node3D

func _ready():
	vehicle_status.subscribe("/vehicle/status/turn_indicators_status", false)
	velocity_report.subscribe("/vehicle/status/velocity_status", false)
	
	wheel_back_r = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelBackR")
	wheel_back_l = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelBackL")
	wheel_front_r = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelFrontR")
	wheel_front_l = get_node("EgoVehicleKinematicBody/VehicleBody3D/WheelFrontL")


func _process(_delta):
	# Ego pose
	set_position(ego_pose.get_ego_position())
	set_rotation(ego_pose.get_ego_rotation())
	
	if(velocity_report.has_new()):
		rotate_wheels(_delta)
		velocity_report.set_old()

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
func rotate_wheels(delta):
	var current_speed  = velocity_report.get_velocity()
	var rotation_amount = current_speed * delta * 2 * PI * 0.25  # Adjust this formula as needed

	wheel_back_r.rotate_x(rotation_amount)
	wheel_back_l.rotate_x(rotation_amount)
	wheel_front_r.rotate_x(rotation_amount)
	wheel_front_l.rotate_x(rotation_amount)


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

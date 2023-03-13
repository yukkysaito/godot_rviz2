extends Spatial

var ego_pose = EgoPose.new()
var vehicle_status = VehicleStatus.new()

func _ready():
	vehicle_status.subscribe("/vehicle/status/turn_indicators_status", false)

func _process(_delta):
	# Ego pose
	set_translation(ego_pose.get_ego_position())
	set_rotation(ego_pose.get_ego_rotation())

	# Indicators
	if(vehicle_status.has_new()):
		var right_light = get_node("EgoVehicleKinematicBody/VehicleBody/TurnSignalR")
		var right_small_light = get_node("EgoVehicleKinematicBody/VehicleBody/FrontTurnSignalR")
		if (vehicle_status.is_turn_on_right()):
			right_light.turn_on()
			right_small_light.turn_on()
		else:
			right_light.turn_off()
			right_small_light.turn_off()

		var left_light = get_node("EgoVehicleKinematicBody/VehicleBody/TurnSignalL")
		var left_small_light = get_node("EgoVehicleKinematicBody/VehicleBody/FrontTurnSignalL")
		if (vehicle_status.is_turn_on_left()):
			left_light.turn_on()
			left_small_light.turn_on()
		else:
			left_light.turn_off()
			left_small_light.turn_off()
		vehicle_status.set_old()



func _on_NightModeCheckButton_toggled(button_pressed):
	var head_light = get_node("EgoVehicleKinematicBody/VehicleBody/HeadLight")
	var head_small_light = get_node("EgoVehicleKinematicBody/VehicleBody/HeadSmallLight")
	var brake_light = get_node("EgoVehicleKinematicBody/VehicleBody/BrakeLight")
	var head_beam_light = get_node("EgoVehicleKinematicBody/HeadBeamLight")
	if (button_pressed):
		head_light.turn_on()
		head_small_light.turn_on()
		brake_light.night_light_turn_on()
		head_beam_light.turn_on()
	else:
		head_light.turn_off()
		head_small_light.turn_off()
		brake_light.night_light_turn_off()
		head_beam_light.turn_off()

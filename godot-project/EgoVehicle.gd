extends Spatial

var ego_pose = EgoPose.new()
var vehicle_status = VehicleStatus.new()

func _ready():
	vehicle_status.subscribe("/vehicle/status/turn_indicators_status",false)

func _process(_delta):
	# Ego pose
	set_translation(ego_pose.get_ego_position())
	set_rotation(ego_pose.get_ego_rotation())

	# Indicators
	var right_light = get_node("EgoVehicleKinematicBody/RearRightIndicatorLight")
	if (vehicle_status.is_turn_on_right()):
		right_light.turn_on()
	else:
		right_light.turn_off()

	var left_light = get_node("EgoVehicleKinematicBody/RearLeftIndicatorLight")
	if (vehicle_status.is_turn_on_left()):
		left_light.turn_on()
	else:
		left_light.turn_off()

#	print(_delta)

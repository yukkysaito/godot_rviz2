extends Node3D

# Ego vehicle: follows the ego pose and drives the vehicle body (wheels, lights) from the vehicle
# state. The body and its placement come from the selected VehicleProfile.

@export var head_beam_light_path: NodePath
@onready var head_beam_light: Node3D = get_node(head_beam_light_path) as Node3D

var profile: VehicleProfile = VehicleProfile.current()
var vehicle_body: VehicleBodyController

var ego_pose = EgoPose.new()

var _turn_left := false
var _turn_right := false

func _ready():
	vehicle_body = profile.body_scene.instantiate() as VehicleBodyController
	vehicle_body.name = "VehicleBody3D"
	vehicle_body.transform = profile.body_transform
	$EgoVehicleKinematicBody.add_child(vehicle_body)
	head_beam_light.position = profile.head_beam_position
	$Camera3D/Horizon/Vertical/ViewCamera.near = profile.camera_near

	Settings.bind("view/day_mode", func(mode): set_night_mode(mode == "night"))

func _process(delta):
	# Ego pose
	set_position(ego_pose.get_ego_position())
	set_rotation(ego_pose.get_ego_rotation())
	if position != Vector3.ZERO:
		PerfMonitor.mark("ego_pose_valid")
	
	# Tires: rotate every frame with the latest speed
	vehicle_body.rotate_wheels_by_distance(RosBridge.velocity * delta)
	vehicle_body.set_steering_angle(RosBridge.steering_angle)

	# Indicators
	if RosBridge.turn_right != _turn_right:
		_turn_right = RosBridge.turn_right
		if _turn_right:
			vehicle_body.turn_on_right_signal()
		else:
			vehicle_body.turn_off_right_signal()
	if RosBridge.turn_left != _turn_left:
		_turn_left = RosBridge.turn_left
		if _turn_left:
			vehicle_body.turn_on_left_signal()
		else:
			vehicle_body.turn_off_left_signal()

func is_turn_indicator_active() -> bool:
	return RosBridge.turn_right or RosBridge.turn_left

func set_night_mode(enabled: bool) -> void:
	vehicle_body.set_night_mode(enabled)
	head_beam_light.visible = enabled

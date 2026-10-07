extends Resource
class_name VehicleProfile

# Everything specific to a vehicle model, so that a vehicle is swapped by adding
# Vehicle/<Name>/<Name>.tres (with its body scene) and selecting it with the "vehicle/profile"
# setting or "-- --vehicle=<Name>", without editing the main scene.
#
# The body scene's root must be a VehicleBodyController (wheels, lights, turn signals).

@export var body_scene: PackedScene
# Placement of the body scene relative to base_link (Godot coordinates: +x forward, +y up),
# applied on top of the transform of the body scene's root
@export var body_transform: Transform3D = Transform3D.IDENTITY
# Distance from base_link (rear axle) to the front end [m]; used for the trajectory's stop wall
@export var wheelbase_to_front: float = 3.78
# Position of the head beam light (night mode) relative to base_link
@export var head_beam_position: Vector3 = Vector3(4.3, 1.1, 0.0)
# Near clip distance of the camera [m]
@export var camera_near: float = 0.2

const DEFAULT_PATH := "res://3DViewer/Vehicle/RX450h/RX450h.tres"

static var _current: VehicleProfile

# The profile selected by the settings (loaded once)
static func current() -> VehicleProfile:
	if _current == null:
		var path := str(Settings.get_value("vehicle/profile"))
		_current = load(path) as VehicleProfile
		if _current == null:
			push_error("Cannot load the vehicle profile %s; using %s" % [path, DEFAULT_PATH])
			_current = load(DEFAULT_PATH) as VehicleProfile
	return _current

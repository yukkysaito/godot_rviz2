extends Node

# Connection to ROS / Autoware (autoload).
#
# ROS is started and all topics are subscribed when the application starts, so messages such as
# the maps are already being received while the splash screen loads the main scene.
# Topic names are defined here only.
#
# - Topics with a single consumer: the consumer uses the subscriber object below with
#   has_new() / get_*() / set_old().
# - Vehicle state used by several nodes is polled here every frame and exposed as properties.

const TOPICS := {
	"vector_map": "/map/vector_map",
	"pointcloud_map": "/map/pointcloud_map",
	"obstacle_segmentation": "/perception/obstacle_segmentation/pointcloud",
	"objects": "/perception/object_recognition/objects",
	"traffic_signals": "/perception/traffic_light_recognition/traffic_signals",
	"trajectory": "/planning/trajectory",
	"behavior_path": "/planning/scenario_planning/lane_driving/behavior_planning/path",
	"turn_indicators": "/vehicle/status/turn_indicators_status",
	"velocity": "/vehicle/status/velocity_status",
	"steering": "/vehicle/status/steering_status",
	"operation_mode": "/api/operation_mode/state",
	"routing_state": "/api/routing/state",
}
const SERVICES := {
	"change_to_autonomous": "/api/operation_mode/change_to_autonomous",
}

# Vector map geometry built on a worker thread as soon as the map arrives (even during the
# splash): name -> [kind, layer(, width)] parts, see VectorMap.start_build()
const MAP_LAYERS := [
	{"name": "road_surface", "parts": [
		["lanelet", "road"], ["lanelet", "shoulder"],
		["polygon", "intersection_area"], ["polygon", "hatched_road_markings_area"],
		["polygon", "parking_lots"]]},
	# Lane lines as in the map: ["lines", types, subtypes, width(, dash, gap)] [m]
	{"name": "road_marker", "parts": [
		["polygon", "pedestrian_marking"],
		["lines", "line_thin", "solid,solid_solid", 0.12],
		["lines", "line_thick", "solid,solid_solid", 0.25],
		["lines", "line_thin", "dashed", 0.12, 5.0, 5.0],
		["lines", "line_thick", "dashed", 0.25, 5.0, 5.0],
		["linestring", "stop_line", 0.5]]},
	# Curbs, guard rails, fences and walls: ["walls", types, height] [m]
	{"name": "barrier", "parts": [
		["walls", "road_border", 0.15],
		["walls", "guard_rail", 0.7],
		["walls", "fence", 1.2],
		["walls", "wall", 2.0]]},
]

const MAP_TILE_SIZE := 100.0  # the map layers are split into tiles of this size [m]

signal map_ready  # map_geometry was (re)built

# {"layers": {name: Array of tiles}, "traffic_lights": Array} (see VectorMap.take_build_result());
# empty until the map is built
var map_geometry: Dictionary = {}
var is_map_ready := false
var is_map_building := false  # the vector map arrived and its geometry is being built

# --- Subscribers (one per topic)
var vector_map := VectorMap.new()
var pointcloud_map := PointCloud.new()
var obstacle_segmentation := PointCloud.new()
var objects := DynamicObjects.new()
var traffic_signals := TrafficLights.new()
var trajectory := Trajectory.new()
var behavior_path := BehaviorPath.new()
var operation_mode := OperationModeState.new()
var routing_state := NavigationState.new()

var _turn_indicators := VehicleStatus.new()
var _velocity := VelocityReport.new()
var _steering := SteeringReport.new()

var _ego_pose := EgoPose.new()

# --- Vehicle state (latest values)
var velocity: float = 0.0          # [m/s]
var steering_angle: float = 0.0    # [rad]
var turn_left: bool = false
var turn_right: bool = false

func _enter_tree() -> void:
	GodotRviz2Spinner.new().spin_some()  # starts the ROS node and its executor thread

	# Maps and API states are latched (transient local)
	vector_map.subscribe(TOPICS["vector_map"], true)
	pointcloud_map.subscribe(TOPICS["pointcloud_map"], true)
	operation_mode.subscribe(TOPICS["operation_mode"], true)
	routing_state.subscribe(TOPICS["routing_state"], true)

	obstacle_segmentation.subscribe(TOPICS["obstacle_segmentation"], false)
	objects.subscribe(TOPICS["objects"], false)
	traffic_signals.subscribe(TOPICS["traffic_signals"], false)
	trajectory.subscribe(TOPICS["trajectory"], false)
	behavior_path.subscribe(TOPICS["behavior_path"], false)
	_turn_indicators.subscribe(TOPICS["turn_indicators"], false)
	_velocity.subscribe(TOPICS["velocity"], false)
	_steering.subscribe(TOPICS["steering"], false)

func _process(_delta: float) -> void:
	_update_map()
	if pointcloud_map.has_new():
		PerfMonitor.mark("pointcloud_map_arrived")
	if _velocity.has_new():
		velocity = _velocity.get_velocity()
		_velocity.set_old()
	if _steering.has_new():
		steering_angle = _steering.get_angle()
		_steering.set_old()
	if _turn_indicators.has_new():
		turn_left = _turn_indicators.is_turn_on_left()
		turn_right = _turn_indicators.is_turn_on_right()
		_turn_indicators.set_old()

func _update_map() -> void:
	if vector_map.is_build_done():
		map_geometry = vector_map.take_build_result()
		is_map_ready = true
		is_map_building = false
		PerfMonitor.measure_end("vector_map_geometry")
		PerfMonitor.mark("vector_map_geometry_ready")
		map_ready.emit()
	if vector_map.has_new() and vector_map.start_build(MAP_LAYERS, MAP_TILE_SIZE):
		vector_map.set_old()
		is_map_building = true
		PerfMonitor.mark("vector_map_arrived")
		PerfMonitor.measure_begin("vector_map_geometry")

# True if some node publishes the vector map (false e.g. while Autoware is not running)
func has_map_publisher() -> bool:
	return vector_map.get_publisher_count() > 0

# Ego position in Godot coordinates (Vector3.ZERO while unknown)
func get_ego_position() -> Vector3:
	return _ego_pose.get_ego_position()

# True once the ego pose (map -> base_link) is available
func is_ego_pose_ready() -> bool:
	return get_ego_position() != Vector3.ZERO

func create_operation_mode_changer() -> OperationModeChanger:
	var changer := OperationModeChanger.new()
	changer.create_client(SERVICES["change_to_autonomous"])
	return changer

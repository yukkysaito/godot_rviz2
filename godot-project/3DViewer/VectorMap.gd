extends Node3D

# Vector map: the geometry is built by RosBridge on a worker thread (started as soon as the map
# arrives); this node only turns it into meshes and traffic lights.

func _ready():
	RosBridge.map_ready.connect(_apply_map)
	if RosBridge.is_map_ready:
		_apply_map()

func _apply_map() -> void:
	PerfMonitor.measure_begin("vector_map_build")
	var layers: Dictionary = RosBridge.map_geometry.get("layers", {})
	$RoadSurfaceMesh.visualize_vertices(layers.get("road_surface", PackedVector3Array()))
	$RoadMarkerMesh.visualize_vertices(layers.get("road_marker", PackedVector3Array()))
	($TrafficLightGroupsManager as TrafficLightGroupsManager).set_map(
		RosBridge.map_geometry.get("traffic_lights", []))
	PerfMonitor.measure_end("vector_map_build")
	PerfMonitor.mark("vector_map_built")

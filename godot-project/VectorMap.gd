extends Spatial

var vector_map = MarkerArray.new()
func _ready():
	vector_map.subscribe("/map/vector_map_marker", true)


func _process(_delta):
	if !vector_map.has_new():
		return
	# Road Surface
	var road_surface = get_node("RoadSurfaceMesh")
	var road_surface_verts = PoolVector3Array()
	road_surface_verts.append_array(vector_map.get_triangle_marker("road_lanelets"))
	road_surface_verts.append_array(vector_map.get_triangle_marker("shoulder_road_lanelets"))
	road_surface.visualize_mesh(road_surface_verts)
	# Road Marker
	var road_marker = get_node("RoadMarkerMesh")
	var road_marker_verts = PoolVector3Array()
	road_marker_verts.append_array(vector_map.get_triangle_marker("right_lane_bound"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("left_lane_bound"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("shoulder_right_lane_bound"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("shoulder_left_lane_bound"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("pedestrian_marking"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("stop_lines"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("intersection_area"))
	road_marker_verts.append_array(vector_map.get_triangle_marker("parking_lots"))
	road_marker.visualize_mesh(road_marker_verts)
	# Traffic Light
	var traffic_light = get_node("TrafficLightMesh")
	traffic_light.visualize_mesh(vector_map.get_triangle_marker("traffic_light_triangle"), vector_map.get_color_spheres("traffic_light"))
	vector_map.set_old()



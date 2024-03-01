extends Node3D

var vector_map = VectorMap.new()

func _ready():
	vector_map.subscribe("/map/vector_map", true)


func _process(_delta):
	if !vector_map.has_new():
		return
	if !vector_map.generate_graph_structure():
		return
	# Road Surface
	var road_surface = get_node("RoadSurfaceMesh")
	var road_surface_triangle_list = Array()
	road_surface_triangle_list.append_array(vector_map.get_lanelet_triangle_list("road"))
	road_surface_triangle_list.append_array(vector_map.get_lanelet_triangle_list("shoulder"))
	#road_surface_triangle_list.append_array(vector_map.get_lanelet_triangle_list("crosswalk"))
	#road_surface_triangle_list.append_array(vector_map.get_lanelet_triangle_list("walkway"))
	road_surface_triangle_list.append_array(vector_map.get_polygon_triangle_list("intersection_area"))
	road_surface_triangle_list.append_array(vector_map.get_polygon_triangle_list("hatched_road_markings_area"))
	road_surface_triangle_list.append_array(vector_map.get_polygon_triangle_list("parking_lots"))
	road_surface.visualize_mesh(road_surface_triangle_list)
	# Road Marker
	var road_marker = get_node("RoadMarkerMesh")
	var road_marker_verts = Array()
	road_marker_verts.append_array(vector_map.get_polygon_triangle_list("pedestrian_marking"))
	road_marker_verts.append_array(vector_map.get_linestring_triangle_list("shared_white_line", 0.05))
	#road_marker_verts.append_array(vector_map.get_linestring_triangle_list("white_line", 0.05))
	road_marker_verts.append_array(vector_map.get_linestring_triangle_list("stop_line", 0.5))
	road_marker.visualize_mesh(road_marker_verts)
	# Traffic Light
	var traffic_light = get_node("TrafficLightMesh")
	traffic_light.set_traffic_light_map(vector_map.get_traffic_light_list())
	#traffic_light.visualize_mesh(vector_map.get_triangle_list("traffic_light_triangle"), vector_map.get_color_spheres("traffic_light"))
	traffic_light.visualize()

	vector_map.set_old()



extends MeshInstance3D

var traffic_light_recognition = TrafficLights.new()
var traffic_light_group_map: Array

@export var light_bulb_offset = 0.0

func _ready():
	traffic_light_recognition.subscribe("/perception/traffic_light_recognition/traffic_signals", false)

# Function to generate a light bulb, setting up its mesh and material
func generate_light_bulb(light_bulb) -> MeshInstance3D:
	var plane_mesh = PlaneMesh.new()
	plane_mesh.size = Vector2(light_bulb["radius"] * 2, light_bulb["radius"] * 2)

	# Creating mesh instance
	var mesh_instance = MeshInstance3D.new()

	# Creating ArrayMesh
	var array_mesh = ArrayMesh.new()
	array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, plane_mesh.get_mesh_arrays())

	# Setting material
	var material = StandardMaterial3D.new()
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	if light_bulb["color"] == "red":
		material.albedo_texture = load("res://TrafficLight/traffic_light_red.png")
	elif light_bulb["color"] == "yellow":
		material.albedo_texture = load("res://TrafficLight/traffic_light_yellow.png")
	elif light_bulb["color"] == "green":
		if light_bulb["arrow"] == "none":
			material.albedo_texture = load("res://TrafficLight/traffic_light_green.png")
		elif light_bulb["arrow"] == "right":
			material.albedo_texture = load("res://TrafficLight/traffic_light_green_right.png")
		elif light_bulb["arrow"] == "left":
			material.albedo_texture = load("res://TrafficLight/traffic_light_green_left.png")
		elif light_bulb["arrow"] == "up":
			material.albedo_texture = load("res://TrafficLight/traffic_light_green_up.png")
		elif light_bulb["arrow"] == "down":
			material.albedo_texture = load("res://TrafficLight/traffic_light_green_down.png")
	mesh_instance.material_override = material

	# Setting mesh
	mesh_instance.mesh = array_mesh

	# Setting mesh instance transformation and position
	var target_normal = light_bulb["normal"].normalized()
	var angle_y = atan2(target_normal.x, target_normal.z)
	var rotation_y = Transform3D(Basis().rotated(Vector3.UP, angle_y), Vector3.ZERO)
	var angle_z = atan2(target_normal.y, Vector2(target_normal.x, target_normal.z).length()) + PI * 0.5
	var rotation_z = Transform3D(Basis().rotated(Vector3.LEFT, angle_z), Vector3.ZERO)
	var rotation_flip = Transform3D(Basis().rotated(Vector3.UP, PI), Vector3.ZERO)

	# Applying final transformation
	mesh_instance.transform = Transform3D().translated(light_bulb["position"]) * rotation_y * rotation_z * rotation_flip * Transform3D().translated(Vector3(0,light_bulb_offset,0))

	return mesh_instance

func set_traffic_light_map(traffic_light_group_list):
	traffic_light_group_map = traffic_light_group_list

# Function to generate the visuals for light bulbs
func visualize():
	mesh.clear_surfaces()

	var boards_arr = []
	boards_arr.resize(Mesh.ARRAY_MAX)
	var boards_verts = PackedVector3Array()
	var boards_normals = PackedVector3Array()
	var boards_colors = PackedColorArray()
	
	# Boards
	for traffic_light_group in traffic_light_group_map:
		for traffic_light in traffic_light_group["traffic_lights"]:
			boards_verts.append(traffic_light["board"]["left_top_position"])
			boards_normals.append(traffic_light["board"]["normal"])
			boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))
			boards_verts.append(traffic_light["board"]["right_top_position"])
			boards_normals.append(traffic_light["board"]["normal"])
			boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))
			boards_verts.append(traffic_light["board"]["right_bottom_position"])
			boards_normals.append(traffic_light["board"]["normal"])
			boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))

			boards_verts.append(traffic_light["board"]["left_bottom_position"])
			boards_normals.append(traffic_light["board"]["normal"])
			boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))
			boards_verts.append(traffic_light["board"]["left_top_position"])
			boards_normals.append(traffic_light["board"]["normal"])
			boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))
			boards_verts.append(traffic_light["board"]["right_bottom_position"])
			boards_normals.append(traffic_light["board"]["normal"])
			boards_colors.append(Color(0.1, 0.1, 0.1, 1.0))
			
	boards_arr[Mesh.ARRAY_VERTEX] = boards_verts
	boards_arr[Mesh.ARRAY_NORMAL] = boards_normals
	boards_arr[Mesh.ARRAY_COLOR] = boards_colors

	# Light Bulbs
	var light_bulbs_arr = []
	light_bulbs_arr.resize(Mesh.ARRAY_MAX)
	var light_bulbs_verts = PackedVector3Array()
	var light_bulbs_normals = PackedVector3Array()
	var light_bulbs_colors = PackedColorArray()

	var segments = 16 # Number of segments to form a circle. Increase this number for a smoother circle.
	var angle_step = 2.0 * PI / segments  # Angle per segment
	for traffic_light_group in traffic_light_group_map:
		for traffic_light in traffic_light_group["traffic_lights"]:
			for light_bulb in traffic_light["light_bulbs"]:
				var center = light_bulb["position"]  # Center of the circle
				var radius = light_bulb["radius"]  # Radius of the circle
				var normal = light_bulb["normal"].normalized()  # Normal vector of the circle
				var angle_y = atan2(normal.x, normal.z)
				var rotation_y = Transform3D(Basis().rotated(Vector3.UP, angle_y), Vector3.ZERO)
				var angle_z = atan2(normal.y, Vector2(normal.x, normal.z).length()) + PI * 0.5
				var rotation_z = Transform3D(Basis().rotated(Vector3.LEFT, angle_z), Vector3.ZERO)
				for i in range(segments):
					# Calculate two points (current point and next point)
					var current_angle = angle_step * i
					var next_angle = angle_step * (i + 1)
					var current_point_local = Vector3(cos(current_angle), 0.0, sin(current_angle)) * radius
					var next_point_local = Vector3(cos(next_angle), 0.0, sin(next_angle)) * radius

					# Apply rotation and transform to world coordinates
					var current_point = center + rotation_y * rotation_z * current_point_local
					var next_point = center + rotation_y * rotation_z * next_point_local
		
					# Add vertices
					light_bulbs_verts.append(center)
					light_bulbs_verts.append(current_point)
					light_bulbs_verts.append(next_point)
		
					# Add normals
					light_bulbs_normals.append(normal)
					light_bulbs_normals.append(normal)
					light_bulbs_normals.append(normal)
		
					# Add colors
					light_bulbs_colors.append(Color(0.0, 0.0, 0.0, 1.0))
					light_bulbs_colors.append(Color(0.0, 0.0, 0.0, 1.0))
					light_bulbs_colors.append(Color(0.0, 0.0, 0.0, 1.0))

	light_bulbs_arr[Mesh.ARRAY_VERTEX] = light_bulbs_verts
	light_bulbs_arr[Mesh.ARRAY_NORMAL] = light_bulbs_normals
	light_bulbs_arr[Mesh.ARRAY_COLOR] = light_bulbs_colors
	
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, light_bulbs_arr)
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, boards_arr)


func _process(_delta):
	if !traffic_light_recognition.has_new():
		return

	var traffic_light_status_list = traffic_light_recognition.get_traffic_light_status()
	
	# Dictionary to mark existing meshes
	var existing_meshes = {}
	for child in get_children():
		if child is MeshInstance3D:
			existing_meshes[child.name] = child

	for traffic_light_group in traffic_light_group_map:
		for traffic_light_status in traffic_light_status_list:
			if traffic_light_status["group_id"] != traffic_light_group["group_id"]:
				continue

			for traffic_light_status_element in traffic_light_status["status_elements"]:
				
				for traffic_light in traffic_light_group["traffic_lights"]:
					for light_bulb in traffic_light["light_bulbs"]:
						if 	light_bulb["color"] != traffic_light_status_element["color"]:
							continue
						if 	light_bulb["arrow"] != traffic_light_status_element["arrow"]:
							continue
						var mesh_name = "Mesh_" + str(traffic_light_status["group_id"]) + "_" + light_bulb["color"] + "_" + light_bulb["arrow"]
						var mesh_instance
						if mesh_name in existing_meshes:
							mesh_instance = existing_meshes[mesh_name]
						else:
							mesh_instance = generate_light_bulb(light_bulb)
							mesh_instance.name = mesh_name
							add_child(mesh_instance)
						# Exclude this mesh from the deletion list since it's being reused
						existing_meshes.erase(mesh_name)

	# Delete meshes that were not used
	for mesh_instance in existing_meshes.values():
		remove_child(mesh_instance)
		mesh_instance.queue_free()
	traffic_light_recognition.set_old()

extends MeshInstance3D

var dynamic_objects = DynamicObjects.new()
var only_known_object = true

func _ready():
	dynamic_objects.subscribe("/perception/object_recognition/objects", false)

func _process(_delta):
	if !dynamic_objects.has_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PackedVector3Array()
#	var uvs = PoolVector2Array()
	var normals = PackedVector3Array()
#	var indices = PoolIntArray()

	var triangle_list = dynamic_objects.get_triangle_list(only_known_object)

	for point in triangle_list:
		verts.append(point["position"])
		normals.append(point["normal"])

	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
#	arr[Mesh.ARRAY_INDEX] = indices	
#	arr[Mesh.ARRAY_TEX_UV] = uvs

	if !verts.is_empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	dynamic_objects.set_old()

func _on_OnlyKnownObjectCheckButton_toggled(button_pressed):
	only_known_object = button_pressed

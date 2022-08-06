extends MeshInstance

var dynamic_objects = DynamicObjects.new()
var only_known_object = true

func cross_product(a, b):
	return Vector3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x)

func _ready():
	dynamic_objects.subscribe("/perception/object_recognition/objects", false)

func _process(_delta):
	if !dynamic_objects.has_new():
		return

	var arr = []
	arr.resize(Mesh.ARRAY_MAX)
	var verts = PoolVector3Array()
#	var uvs = PoolVector2Array()
	var normals = PoolVector3Array()
#	var indices = PoolIntArray()

	verts = dynamic_objects.get_triangle_points(only_known_object)

	for i in verts.size():
		if(i % 3 ==2):
			normals.append(cross_product(verts[i] - verts[i-2], verts[i-1] - verts[i-2]).normalized())
			normals.append(normals[normals.size()-1])
			normals.append(normals[normals.size()-1])

	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
#	arr[Mesh.ARRAY_INDEX] = indices	
#	arr[Mesh.ARRAY_TEX_UV] = uvs

	if !verts.empty():
		mesh.clear_surfaces()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arr)
	dynamic_objects.set_old()


func _on_OnlyKnownObjectCheckButton_toggled(button_pressed):
	only_known_object = button_pressed

class_name MeshUtils
extends RefCounted
## Static helpers that convert ROS-message point lists (dictionaries produced
## by the C++ modules Trajectory / BehaviorPath / VectorMap) into ArrayMesh
## surface arrays, so each mesh script does not rebuild the same boilerplate.


## Wraps pre-built packed arrays into a Mesh.ARRAY_MAX sized surface array.
## Empty optional arrays (colors / uvs) are skipped.
static func make_surface_arrays(
	verts: PackedVector3Array,
	normals: PackedVector3Array,
	colors: PackedColorArray = PackedColorArray(),
	uvs: PackedVector2Array = PackedVector2Array()
) -> Array:
	var arr: Array = []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = verts
	arr[Mesh.ARRAY_NORMAL] = normals
	if !colors.is_empty():
		arr[Mesh.ARRAY_COLOR] = colors
	if !uvs.is_empty():
		arr[Mesh.ARRAY_TEX_UV] = uvs
	return arr


## Strip/list of points with "position" + "normal" keys, painted a single color.
static func strip_to_surface_arrays(points: Array, color: Color) -> Array:
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var colors := PackedColorArray()
	for point in points:
		verts.append(point["position"])
		normals.append(point["normal"])
		colors.append(color)
	return make_surface_arrays(verts, normals, colors)


## Strip/list of points with "position" + "normal" + per-point "color" keys.
static func colored_strip_to_surface_arrays(points: Array) -> Array:
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var colors := PackedColorArray()
	for point in points:
		verts.append(point["position"])
		normals.append(point["normal"])
		colors.append(point["color"])
	return make_surface_arrays(verts, normals, colors)


## Flat (ground) triangle list of points with only a "position" key.
## Normals are forced up; an optional constant vertex color can be written.
static func triangle_list_to_surface_arrays(
	points: Array, color: Color = Color(0.0, 0.0, 0.0, 0.0)
) -> Array:
	var write_color := color.a > 0.0
	var verts := PackedVector3Array()
	var normals := PackedVector3Array()
	var colors := PackedColorArray()
	for point in points:
		verts.append(point["position"])
		normals.append(Vector3.UP)
		if write_color:
			colors.append(color)
	return make_surface_arrays(verts, normals, colors)


## True when the surface array set holds at least one vertex.
static func has_vertices(arrays: Array) -> bool:
	var verts: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
	return !verts.is_empty()

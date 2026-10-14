extends Control

# Mini map of the HUD: the roads around the vehicle, the route and its goal. At the bottom right,
# or in a portrait window (where the speed is below the vehicle) at the top right under the clock.
# Its background is the vector map's road surface ("lanelet") or OpenStreetMap tiles over it
# ("osm", see the setting hud/minimap): without a connection, the road surface shows where there
# are no tiles. It follows the vehicle heading up; a click opens it over the whole window with the
# whole route north up, another one brings it back.
# The map is drawn by a 2D world in its own viewport, in meters: a point is (x, z) of Godot's
# coordinates. The vehicle and the goal are drawn over it, upright.

@export var map_size: float = 300.0  # [px] at a 1080 px high window, scaled with it
@export var margin: float = 32.0  # [px] from the window's corner
@export var portrait_width: float = 0.18  # of the window's width at most, in a portrait window
@export var portrait_top: float = 88.0  # [px] under the clock (UI/Hud.gd)
@export var follow_view_width: float = 250.0  # [m] across the map while following the vehicle
@export var follow_ahead: float = 0.2  # of the map's height: the vehicle is below its middle
@export var overview_margin: float = 0.12  # of the map's size around the whole route
@export var view_smoothing: float = 6.0  # [1/s] how fast the view moves to its target
@export var expand_time: float = 0.3  # [s] to open over the whole window and back
@export var build_budget_msec: float = 2.0  # time spent on the road meshes per frame

const GROUND := Color(0.035, 0.05, 0.085)
const ROAD := Color(0.24, 0.28, 0.37)
const ROUTE := Color(0.45, 0.68, 1.0)  # the HUD's accent blue
const EGO := Color(0.16, 0.42, 0.95)
const GOAL := Color(0.9, 0.0, 0.07)  # as the goal marker (3DViewer/GoalMarker.gd)
const FRAME := Color(1, 1, 1, 0.14)
# Sizes [px] at a 1080 px high window, scaled with the map while it is smaller than the default
const CORNER_RADIUS := 18.0
const ROUTE_WIDTH := 5.0
const MIN_FONT_SIZE := 9  # [px] the texts stay readable on a small map
const GOAL_INSET := 14.0  # the goal shows as an arrow at the map's edge beyond this

var _mode := "off"
var _expanded := false  # over the whole window, showing the whole route
var _expansion := 0.0  # 0: in the corner .. 1: over the whole window (animated)
var _corner_rect := Rect2()
var _window_rect := Rect2()
var _ui_scale := 1.0  # as the HUD (UI/Hud.gd)
var _viewport := SubViewport.new()
var _view := TextureRect.new()
var _overlay := Control.new()
var _roads := Node2D.new()  # one MeshInstance2D per tile of the road surface
var _tiles: Node2D = preload("res://UI/OsmTiles.gd").new()
var _route := Line2D.new()
var _pending_road_tiles: Array = []

# Current view: center [m], rotation of the world [rad], zoom [px/m] (smoothed toward the target)
var _center := Vector2.ZERO
var _rotation := 0.0
var _zoom := 1.0
var _view_initialized := false

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	_viewport.disable_3d = true
	_viewport.transparent_bg = true  # the ground color is put behind it by the rounded mask
	_viewport.render_target_update_mode = SubViewport.UPDATE_WHEN_VISIBLE
	_viewport.msaa_2d = Viewport.MSAA_4X
	add_child(_viewport)
	_roads.z_index = -100  # under the tiles (see OsmTiles.gd)
	_viewport.add_child(_roads)
	_viewport.add_child(_tiles)
	_route.default_color = ROUTE
	_route.joint_mode = Line2D.LINE_JOINT_ROUND
	_route.begin_cap_mode = Line2D.LINE_CAP_ROUND
	_route.end_cap_mode = Line2D.LINE_CAP_ROUND
	_viewport.add_child(_route)

	var mask := ShaderMaterial.new()
	mask.shader = preload("res://UI/Shaders/rounded_map.gdshader")
	mask.set_shader_parameter("ground", GROUND)
	_view.material = mask
	_view.texture = _viewport.get_texture()
	_view.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_view)
	_overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_overlay.clip_contents = true
	_overlay.draw.connect(_draw_overlay)
	add_child(_overlay)

	RosBridge.map_ready.connect(_on_map_ready)
	RosBridge.route_changed.connect(_on_route_changed)
	if RosBridge.is_map_ready:
		_on_map_ready()
	_on_route_changed()
	get_viewport().size_changed.connect(_layout)
	_layout()
	Settings.bind("hud/minimap", _set_mode)

func _set_mode(mode: String) -> void:
	_mode = mode
	visible = mode != "off"
	_tiles.visible = mode == "osm"

func _layout() -> void:
	var window := get_viewport_rect().size
	_ui_scale = window.y / 1080.0
	var portrait := window.x < window.y
	var side := map_size * _ui_scale
	if portrait:
		side = minf(side, window.x * portrait_width)
	side = roundf(side)
	var gap := margin * _ui_scale
	if portrait:
		_corner_rect = Rect2(window.x - side - gap, portrait_top * _ui_scale, side, side)
	else:
		_corner_rect = Rect2(window - Vector2.ONE * (side + gap), Vector2(side, side))
	_window_rect = Rect2(Vector2.ONE * gap, window - Vector2.ONE * 2.0 * gap)
	_apply_rect()

# Places the map between the corner and the whole window, as far as it is expanded
func _apply_rect() -> void:
	var e := smoothstep(0.0, 1.0, _expansion)
	position = _corner_rect.position.lerp(_window_rect.position, e).round()
	size = _corner_rect.size.lerp(_window_rect.size, e).round()
	_viewport.size = Vector2i(size)
	_view.size = size
	_overlay.size = size
	var mask := _view.material as ShaderMaterial
	mask.set_shader_parameter("size", size)
	mask.set_shader_parameter("radius", CORNER_RADIUS * _scale())

# Scale of the drawing (1 at the default size in a 1080 px high window): smaller with a smaller map
func _scale() -> float:
	return lerpf(_corner_rect.size.y / map_size, _ui_scale, smoothstep(0.0, 1.0, _expansion))

func _gui_input(event: InputEvent) -> void:
	var button := event as InputEventMouseButton
	if button != null and button.pressed and button.button_index == MOUSE_BUTTON_LEFT:
		_expanded = not _expanded
		accept_event()

func _process(delta: float) -> void:
	if not visible:
		return
	_build_road_tiles()
	var expansion := move_toward(_expansion, 1.0 if _expanded else 0.0, delta / expand_time)
	if expansion != _expansion:
		_expansion = expansion
		_apply_rect()
	_update_view(delta)
	if _mode == "osm":
		_tiles.update_view(_center, _zoom, size.length() * 0.5 / _zoom)
	_overlay.queue_redraw()

# --- View

func _update_view(delta: float) -> void:
	var ego := _to_2d(RosBridge.get_ego_position())
	var target_center := ego
	var target_rotation := 0.0
	var target_zoom := size.x / follow_view_width
	if _expanded and RosBridge.route_line.size() >= 2:
		var bounds := Rect2(ego, Vector2.ZERO)
		for point in RosBridge.route_line:
			bounds = bounds.expand(_to_2d(point))
		target_center = bounds.get_center()
		var fit := size * (1.0 - 2.0 * overview_margin)
		target_zoom = minf(fit.x / maxf(bounds.size.x, 1.0), fit.y / maxf(bounds.size.y, 1.0))
	else:
		# Heading up: the vehicle's forward, at the angle -yaw in the world, points up on the map
		var yaw := RosBridge.get_ego_yaw()
		target_rotation = PI * 0.5 - yaw
		target_center = ego + Vector2.from_angle(-yaw) * follow_ahead * size.y / target_zoom
	var t := 1.0 - exp(-view_smoothing * delta)
	if not _view_initialized:
		t = 1.0
		_view_initialized = RosBridge.is_ego_pose_ready()
	_center = _center.lerp(target_center, t)
	_rotation = lerp_angle(_rotation, target_rotation, t)
	_zoom = exp(lerpf(log(_zoom), log(target_zoom), t))
	_viewport.canvas_transform = _world_to_map()
	_route.width = ROUTE_WIDTH * _scale() / _zoom

# Transform from the 2D world [m] to the map [px]
func _world_to_map() -> Transform2D:
	return Transform2D.IDENTITY.translated(-_center).rotated(-_rotation) \
		.scaled(Vector2(_zoom, _zoom)).translated(size * 0.5)

static func _to_2d(position_3d: Vector3) -> Vector2:
	return Vector2(position_3d.x, position_3d.z)

# --- Content

func _on_route_changed() -> void:
	var points := PackedVector2Array()
	for point in RosBridge.route_line:
		points.append(_to_2d(point))
	_route.points = points

func _on_map_ready() -> void:
	for child in _roads.get_children():
		child.queue_free()
	_pending_road_tiles = RosBridge.map_geometry["layers"].get("road_surface", []).duplicate()

# Flattens the tiles of the 3D road surface into 2D meshes, a few per frame
func _build_road_tiles() -> void:
	var start := Time.get_ticks_usec()
	while not _pending_road_tiles.is_empty() and \
			Time.get_ticks_usec() - start < build_budget_msec * 1000.0:
		var tile: Dictionary = _pending_road_tiles.pop_back()
		var vertices_3d: PackedVector3Array = tile["vertices"]
		var vertices := PackedVector2Array()
		vertices.resize(vertices_3d.size())
		for i in vertices_3d.size():
			vertices[i] = _to_2d(vertices_3d[i])
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = vertices
		var mesh := ArrayMesh.new()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
		var instance := MeshInstance2D.new()
		instance.mesh = mesh
		instance.position = _to_2d(tile["center"])
		instance.modulate = ROAD
		_roads.add_child(instance)

# --- Overlay: the vehicle and the goal upright, the attribution and the frame

func _draw_overlay() -> void:
	var s := _scale()
	var to_map := _world_to_map()
	if RosBridge.has_route:
		var goal := to_map * _to_2d(RosBridge.goal_position)
		if Rect2(Vector2.ZERO, size).grow(-GOAL_INSET * s).has_point(goal):
			_draw_goal(goal, s)
		else:
			_draw_goal_direction(goal, s)
	if RosBridge.is_ego_pose_ready():
		var heading := -RosBridge.get_ego_yaw() - _rotation
		_draw_ego(to_map * _to_2d(RosBridge.get_ego_position()), heading, s)
	if _mode == "osm" and _tiles.is_showing_tiles():
		_draw_attribution(s)
	if _expansion > 0.0:
		_draw_close(s)
	var frame := StyleBoxFlat.new()
	frame.draw_center = false
	frame.border_color = FRAME
	frame.set_border_width_all(maxi(1, roundi(s)))
	frame.set_corner_radius_all(roundi(CORNER_RADIUS * s))
	_overlay.draw_style_box(frame, Rect2(Vector2.ZERO, size))

# Goal beyond the map: an arrow at the map's edge pointing to it, with the distance to it
func _draw_goal_direction(goal: Vector2, s: float) -> void:
	var middle := size * 0.5
	var direction := (goal - middle).normalized()
	var half := middle - Vector2.ONE * GOAL_INSET * s
	var at := middle + direction * minf(half.x / maxf(absf(direction.x), 1e-6),
		half.y / maxf(absf(direction.y), 1e-6))
	var side := direction.orthogonal()
	var arrow := PackedVector2Array([
		at + direction * 9 * s, at - direction * 5 * s + side * 8 * s,
		at - direction * 2 * s, at - direction * 5 * s - side * 8 * s])
	_overlay.draw_colored_polygon(arrow, GOAL)
	_overlay.draw_polyline(arrow + PackedVector2Array([arrow[0]]), Color.WHITE, 1.5 * s, true)

	var meters := RosBridge.get_ego_position().distance_to(RosBridge.goal_position)
	var text := "%d m" % roundi(meters) if meters < 1000.0 else "%.1f km" % (meters / 1000.0)
	var font := get_theme_default_font()
	var font_size := maxi(roundi(12 * s), MIN_FONT_SIZE + 2)
	var text_size := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size)
	# Next to the arrow, toward the middle
	var reach := text_size.x * 0.5 * absf(direction.x) + text_size.y * 0.5 * absf(direction.y)
	var center := at - direction * (14 * s + reach)
	var box := Rect2(center - text_size * 0.5 - Vector2(5, 1) * s, text_size + Vector2(10, 2) * s)
	_overlay.draw_rect(box, Color(0.03, 0.05, 0.09, 0.75))
	_overlay.draw_string(font, Vector2(box.position.x + 5 * s, center.y + text_size.y * 0.3), text,
		HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color.WHITE)

# Arrow of the vehicle, pointing along `heading` (on the map, 0: to the right)
func _draw_ego(at: Vector2, heading: float, s: float) -> void:
	var forward := Vector2.from_angle(heading)
	var side := forward.orthogonal()
	var points := PackedVector2Array([
		at + forward * 11 * s,
		at - forward * 7 * s + side * 8 * s,
		at - forward * 3 * s,
		at - forward * 7 * s - side * 8 * s])
	_overlay.draw_circle(at, 15 * s, Color(ROUTE, 0.25))
	# Blue in a white outline, to be seen on both the dark roads and the light tiles
	_overlay.draw_colored_polygon(points, EGO)
	_overlay.draw_polyline(points + PackedVector2Array([points[0]]), Color.WHITE, 1.5 * s, true)

# Map pin with its tip on the goal: a round head and the sides tangent to it, a white dot inside
func _draw_goal(tip: Vector2, s: float) -> void:
	var head := tip - Vector2(0, 16 * s)
	var radius := 7.5 * s
	var tangent := acos(radius / (tip - head).length())  # angle at the head between tip and side
	var points := PackedVector2Array([tip])
	var steps := 24
	for i in steps + 1:
		var angle := PI * 0.5 + tangent + (TAU - 2.0 * tangent) * i / steps
		points.append(head + Vector2.from_angle(angle) * radius)
	_overlay.draw_colored_polygon(points, GOAL)
	_overlay.draw_circle(head, radius * 0.42, Color.WHITE)

# A cross at the top right of the expanded map: a click anywhere brings the map back
func _draw_close(s: float) -> void:
	var center := Vector2(size.x - 34 * s, 34 * s)
	var alpha := smoothstep(0.5, 1.0, _expansion)
	_overlay.draw_circle(center, 18 * s, Color(0.03, 0.05, 0.09, 0.75 * alpha))
	for direction in [Vector2(1, 1), Vector2(1, -1)]:
		_overlay.draw_line(center - direction * 6 * s, center + direction * 6 * s,
			Color(1, 1, 1, alpha), 2.0 * s, true)

# OpenStreetMap's tiles must be credited
func _draw_attribution(s: float) -> void:
	var font := get_theme_default_font()
	var text := "© OpenStreetMap contributors"
	var font_size := maxi(roundi(10 * s), MIN_FONT_SIZE)
	var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
	var bottom := size.y - CORNER_RADIUS * 0.5 * s
	_overlay.draw_string(font, Vector2((size.x - width) * 0.5, bottom), text,
		HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(0.15, 0.15, 0.2, 0.9))

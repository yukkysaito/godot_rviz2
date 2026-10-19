extends Control

# Progress along the route at the top of the HUD, with the remaining distance
# (Core/RouteTracker.gd): a thin track to the goal and the travelled part as a band of smoke
# (Shaders/route_progress.gdshader) with a white-hot head at the vehicle. Shown while there is a
# route the vehicle is on, until it arrives.
# Sized for a 1080 px high window and scaled like the HUD's top row (UI/Hud.gd). It
# is centered in the top row between the HUD's mode and clock (shorter in a narrow window) or,
# where they leave too little room, in a row under them, beside the mini map (UI/MiniMap.gd) if
# that is there.

@export var bar_length: float = 620.0  # [px]
@export var max_width: float = 0.6  # of the window's width at most
@export var top: float = 46.0  # [px] of the bar's middle, level with the HUD's top status bar
@export var clearance: float = 40.0  # [px] from the HUD's mode and clock
@export var min_length: float = 120.0  # [px] in the top row; shorter, it goes under it
@export var second_row_top: float = 104.0  # [px] of the bar's middle under the top row
@export var hud_path: NodePath = ^"../Hud"
@export var mini_map_path: NodePath = ^"../MiniMap"
@export var band_height: float = 56.0  # [px] of its drawing, the band and its light
@export var fade_time: float = 0.5  # [s]

const TEXT := Color(0.93, 0.95, 1.0)
const TRACK := Color(1, 1, 1, 0.22)
const GOAL := Color(0.9, 0.0, 0.07)  # as the goal marker (3DViewer/GoalMarker.gd)
const FONT_BOLD := preload("res://UI/Fonts/Barlow-SemiBold.ttf")

var _band := ColorRect.new()
var _progress := 0.0
var _visibility := 0.0
var _enabled := true
var _distance_text := ""  # drawn under the bar

@onready var _hud: Hud = get_node_or_null(hud_path)
@onready var _mini_map: Control = get_node_or_null(mini_map_path)

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	var material := ShaderMaterial.new()
	material.shader = preload("res://UI/Shaders/route_progress.gdshader")
	_band.material = material
	_band.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_band)
	visible = false  # until _process() shows it, with its size
	Settings.bind("hud/route_progress", func(on): _enabled = on)

func _process(delta: float) -> void:
	var tracker := RosBridge.route_tracker
	var shown := _enabled and tracker.valid and not RosBridge.routing_state.is_arrived() \
		and not RosBridge.routing_state.has_no_route()
	_visibility = move_toward(_visibility, 1.0 if shown else 0.0, delta / fade_time)
	visible = _visibility > 0.0
	if not visible:
		return
	_progress = lerpf(_progress, tracker.progress, 1.0 - exp(-4.0 * delta))  # eases jumps

	var window := get_viewport_rect().size
	var s := _hud.top_row_scale() if _hud != null else window.y / 1080.0
	# In the top row: centered, as far from both edges, clear of the HUD's mode (left) and clock
	# (right)
	var insets := _hud.status_bar_insets() if _hud != null else Vector2.ZERO
	var side := maxf(insets.x, insets.y) + clearance * s
	var left := side
	var right := window.x - side
	var middle := top * s
	if right - left < min_length * s:
		# In a row under it, from the edge to the mini map where that is beside the row
		middle = second_row_top * s
		left = clearance * s
		right = window.x - clearance * s
		if _mini_map != null and _mini_map.visible:
			var map: Rect2 = _mini_map.corner_rect()
			var row := Rect2(0.0, middle - band_height * s * 0.5, window.x, band_height * s)
			if map.intersects(row):
				right = minf(right, map.position.x - clearance * s)
	var length := minf(minf(bar_length * s, window.x * max_width), right - left)
	var material := _band.material as ShaderMaterial
	material.set_shader_parameter("progress", _progress)
	modulate.a = _visibility
	# The track and the text are drawn again only when they change (the band flows in its shader)
	var distance_text := _format_distance(tracker.remaining_distance)
	var new_size := Vector2(length, band_height * s)
	var new_position := Vector2((left + right - length) * 0.5, middle - new_size.y * 0.5)
	if new_size != size or new_position != position or distance_text != _distance_text:
		size = new_size
		position = new_position
		_band.size = size
		material.set_shader_parameter("length_px", length)
		_distance_text = distance_text
		queue_redraw()

# The track (a thin line from a diamond at the start to a small goal pin) and, under its goal end,
# the remaining distance
func _draw() -> void:
	var s := size.y / band_height
	var middle := size.y * 0.5
	draw_line(Vector2(0, middle), Vector2(size.x - 10 * s, middle), TRACK, maxf(1.0, 1.2 * s), true)
	var diamond := PackedVector2Array([Vector2(-6, 0), Vector2(0, -6), Vector2(6, 0), Vector2(0, 6)])
	for i in diamond.size():
		diamond[i] = diamond[i] * s + Vector2(0, middle)
	draw_polyline(diamond + PackedVector2Array([diamond[0]]), Color(1, 1, 1, 0.5), maxf(1.0, 1.2 * s),
		true)
	_draw_goal_pin(Vector2(size.x - 4 * s, middle), s)

	var baseline := middle + 26 * s
	var distance := _distance_text
	var font_size := int(20 * s)
	var width := FONT_BOLD.get_string_size(distance, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
	draw_string(FONT_BOLD, Vector2(size.x - width, baseline), distance, HORIZONTAL_ALIGNMENT_LEFT, -1,
		font_size, TEXT)

# Map pin with its tip on the track's end, as the goal marker
func _draw_goal_pin(tip: Vector2, s: float) -> void:
	var head := tip - Vector2(0, 11 * s)
	var radius := 5.0 * s
	var tangent := acos(radius / (tip - head).length())  # angle at the head between tip and side
	var points := PackedVector2Array([tip])
	var steps := 20
	for i in steps + 1:
		var angle := PI * 0.5 + tangent + (TAU - 2.0 * tangent) * i / steps
		points.append(head + Vector2.from_angle(angle) * radius)
	draw_colored_polygon(points, GOAL)
	draw_circle(head, radius * 0.42, Color.WHITE)

static func _format_distance(meters: float) -> String:
	if meters < 1000.0:
		return "%d m" % roundi(meters)
	return "%.1f km" % (meters / 1000.0)

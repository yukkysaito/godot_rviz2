extends Control

# Progress along the route at the top of the HUD, with the remaining distance
# (Core/RouteTracker.gd): a thin track to the goal and the travelled part as a band of smoke
# (Shaders/route_progress.gdshader) with a white-hot head at the vehicle. Shown while there is a
# route the vehicle is on, until it arrives.
# Sized for a 1080 px high window and scaled with the window height, like the HUD (UI/Hud.gd).

@export var bar_length: float = 620.0  # [px]
@export var max_width: float = 0.6  # of the window's width at most
@export var top: float = 46.0  # [px] of the bar's middle, level with the HUD's top status bar
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
	var s := window.y / 1080.0
	var length := minf(bar_length * s, window.x * max_width)
	size = Vector2(length, band_height * s)
	position = Vector2((window.x - length) * 0.5, top * s - size.y * 0.5)
	_band.size = size
	var material := _band.material as ShaderMaterial
	material.set_shader_parameter("progress", _progress)
	material.set_shader_parameter("length_px", length)
	modulate.a = _visibility
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
	var distance := _format_distance(RosBridge.route_tracker.remaining_distance)
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

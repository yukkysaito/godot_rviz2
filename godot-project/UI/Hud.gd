extends Control
class_name Hud

# Heads-up display drawn over the 3D view, crisp at any resolution (vector drawing, no images):
# - top left: the operation mode (AUTO / MANUAL)
# - top right: the clock (with the route progress of UI/RouteProgress.gd between them, the top row)
# - bottom center: the speed, the turn signals, the steering angle and, when the vehicle is planned
#   to stop ahead, the distance to the stop
# Sized for a 1080 px high window and scaled with the window height; the top row also shrinks in a
# narrow window (e.g. a portrait one), so that it stays in one row.
# What it shows is gathered every frame, and it is drawn again only when that changed (drawing it,
# with its texts, is the main cost of a frame on the CPU).

@export var trajectory_mesh_path: NodePath  # for the distance to the planned stop
@export var stop_display_range: float = 150.0  # [m] the stop shows within this distance
@export var max_steering_angle: float = 0.5  # [rad] at the end of the steering arc
@export var top_row_width: float = 620.0  # [px] of a 1080 px high window the top row needs at full size
# Small angles are shown larger (angle^STEERING_CURVE), as most driving is within a few degrees
const STEERING_CURVE := 0.5

const FONT_LIGHT := preload("res://UI/Fonts/Barlow-Light.ttf")
const FONT_BOLD := preload("res://UI/Fonts/Barlow-SemiBold.ttf")

const TEXT := Color(0.93, 0.95, 1.0)
const MUTED := Color(0.62, 0.68, 0.8)
const ACCENT := Color(0.45, 0.68, 1.0)  # the blue of the road borders and the trajectory
const SIGNAL := Color(0.3, 0.95, 0.55)  # turn signals
const STOP := Color(1.0, 0.3, 0.26)
const PANEL := Color(0.03, 0.05, 0.09, 0.55)

var _autonomous := false
var _show_status := true  # mode and clock
var _show_cluster := true  # speed, signals, steering, stop
var _labels: FontVariation  # letter-spaced, for the small capitals
var _clock := ""
# What is shown (see _process), and what was drawn last
var _kmh := 0
var _steer := 0.0  # -1..1 along the steering arc, > 0: to the right
var _signals_lit := [false, false]  # left, right
var _stop_text := ""
var _drawn := []

@onready var _trajectory_mesh: Node = get_node_or_null(trajectory_mesh_path)

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_labels = FontVariation.new()
	_labels.base_font = FONT_BOLD
	_labels.spacing_glyph = 2
	Settings.bind("hud/top_status_bar", func(on): _show_status = on)
	Settings.bind("hud/vehicle_actuator_status", func(on): _show_cluster = on)

# Called by OperationControl
func set_autonomous(enabled: bool) -> void:
	_autonomous = enabled

func _process(_delta: float) -> void:
	_clock = Time.get_time_string_from_system().substr(0, 5)
	_kmh = int(round(absf(RosBridge.velocity) * 3.6))
	var ratio := clampf(absf(RosBridge.steering_angle) / max_steering_angle, 0.0, 1.0)
	_steer = snappedf(-signf(RosBridge.steering_angle) * pow(ratio, STEERING_CURVE), 0.005)
	var blink := fmod(Time.get_ticks_msec() / 1000.0, 0.8) < 0.5
	_signals_lit = [RosBridge.turn_left and blink, RosBridge.turn_right and blink]
	var stop_distance: float = _trajectory_mesh.stop_distance if _trajectory_mesh != null else -1.0
	var stop_shown := stop_distance > 1.0 and stop_distance < stop_display_range
	_stop_text = "STOP  %d m" % int(round(stop_distance)) if stop_shown else ""
	var shown := [size, _show_status, _show_cluster, _autonomous, _clock, _kmh, _steer, _signals_lit,
		_stop_text]
	if shown != _drawn:
		_drawn = shown
		queue_redraw()

# Scale of the top row (the mode, the route progress and the clock): the HUD's, smaller in a window
# narrower than top_row_width at that scale
func top_row_scale() -> float:
	return minf(size.y / 1080.0, size.x / top_row_width)

# How far the top status bar reaches in from the left and from the right edge [px] (the mode and
# the clock), for the route progress between them. Zero while it is hidden.
func status_bar_insets() -> Vector2:
	if not _show_status:
		return Vector2.ZERO
	var t := top_row_scale()
	var origin := Vector2(28, 28) * t
	# The wider of the two modes, so that the row does not move when the mode changes
	var mode := maxf(_mode_rect(origin, t, "AUTO").end.x, _mode_rect(origin, t, "MANUAL").end.x)
	var clock := FONT_LIGHT.get_string_size(_clock, HORIZONTAL_ALIGNMENT_LEFT, -1, int(30 * t)).x
	return Vector2(mode, 32 * t + clock)

func _draw() -> void:
	var s := size.y / 1080.0
	if _show_status:
		var t := top_row_scale()
		_draw_mode(Vector2(28, 28) * t, t)
		_draw_text_right(FONT_LIGHT, _clock, Vector2(size.x - 32 * t, 56 * t), 30 * t, TEXT)
	if _show_cluster:
		_draw_cluster(Vector2(size.x * 0.5, size.y - 92 * s), s)

# A pill with a glowing dot and AUTO / MANUAL: blue while autonomous, gray while manual
func _draw_mode(origin: Vector2, s: float) -> void:
	var text := _mode_text()
	var font_size := int(15 * s)
	var rect := _mode_rect(origin, s, text)
	var color := ACCENT if _autonomous else MUTED
	_draw_pill(rect, PANEL, Color(ACCENT, 0.5) if _autonomous else Color(1, 1, 1, 0.12), s)
	var dot := origin + Vector2(20, 18) * s
	for i in 3:  # the glow, fading out
		_draw_dot(dot, (5.5 + 2.0 * i) * s, Color(color, 0.16 - 0.04 * i))
	_draw_dot(dot, 4.0 * s, color)
	_draw_dot(dot, 2.2 * s, color.lerp(Color.WHITE, 0.55))
	draw_string(_labels, origin + Vector2(36 * s, 23.5 * s), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size,
		TEXT if _autonomous else MUTED)

func _mode_text() -> String:
	return "AUTO" if _autonomous else "MANUAL"

func _mode_rect(origin: Vector2, s: float, text: String) -> Rect2:
	var width := _labels.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(15 * s)).x
	return Rect2(origin, Vector2(width + 52 * s, 36 * s))

func _draw_cluster(center: Vector2, s: float) -> void:
	# A soft shade under it, so that it reads over any scene
	_draw_shade(Rect2(center.x - 420 * s, size.y - 260 * s, 840 * s, 260 * s), 0.5)

	# Speed
	_draw_text_center(FONT_BOLD, str(_kmh), center + Vector2(0, 18 * s), 76 * s, TEXT)
	_draw_text_center(_labels, "KM/H", center + Vector2(0, 46 * s), 13 * s, MUTED)

	# Steering: a thin arc over the speed with a mark at the current angle
	var radius := 78 * s
	var arc_center := center + Vector2(0, 8 * s)
	var half_span := 0.55
	draw_arc(arc_center, radius, -PI * 0.5 - half_span, -PI * 0.5 + half_span, 48, Color(1, 1, 1, 0.14), 2 * s, true)
	var angle := -PI * 0.5 + _steer * half_span
	if absf(_steer) > 0.01:  # the arc from the middle to the mark, colored
		draw_arc(arc_center, radius, minf(-PI * 0.5, angle), maxf(-PI * 0.5, angle), 32, ACCENT, 2.5 * s, true)
	var mark := arc_center + Vector2(cos(angle), sin(angle)) * radius
	_draw_dot(mark, 6 * s, Color(ACCENT, 0.18))
	_draw_dot(mark, 3.5 * s, ACCENT)

	# Turn signals, blinking
	_draw_signal(center + Vector2(-150 * s, -2 * s), -1.0, _signals_lit[0], s)
	_draw_signal(center + Vector2(150 * s, -2 * s), 1.0, _signals_lit[1], s)

	# Distance to the planned stop
	if not _stop_text.is_empty():
		var text := _stop_text
		var font_size := int(14 * s)
		var width := _labels.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
		var rect := Rect2(center + Vector2(-width * 0.5 - 18 * s, -126 * s), Vector2(width + 36 * s, 30 * s))
		_draw_pill(rect, Color(STOP, 0.16), Color(STOP, 0.7), s)
		_draw_text_center(_labels, text, rect.get_center() + Vector2(0, 5 * s), 14 * s, Color(1.0, 0.82, 0.8))

# Darker toward the bottom middle of `rect`, fading out to its top and sides
func _draw_shade(rect: Rect2, strength: float) -> void:
	const COLUMNS := 16
	for i in COLUMNS:
		var x0 := float(i) / COLUMNS
		var x1 := float(i + 1) / COLUMNS
		var a0 := strength * (1.0 - pow(absf(x0 * 2.0 - 1.0), 2.0))
		var a1 := strength * (1.0 - pow(absf(x1 * 2.0 - 1.0), 2.0))
		var left := rect.position.x + rect.size.x * x0
		var right := rect.position.x + rect.size.x * x1
		draw_polygon(PackedVector2Array([Vector2(left, rect.position.y), Vector2(right, rect.position.y),
			Vector2(right, rect.end.y), Vector2(left, rect.end.y)]),
			PackedColorArray([Color(0, 0, 0, 0), Color(0, 0, 0, 0), Color(0, 0, 0, a1), Color(0, 0, 0, a0)]))

# An arrow pointing left (direction -1) or right (1)
func _draw_signal(at: Vector2, direction: float, on: bool, s: float) -> void:
	var w := 16 * s
	var h := 19 * s
	var points := PackedVector2Array([
		at + Vector2(direction * w, 0), at + Vector2(0, -h * 0.5), at + Vector2(0, -h * 0.2),
		at + Vector2(-direction * w * 0.7, -h * 0.2), at + Vector2(-direction * w * 0.7, h * 0.2),
		at + Vector2(0, h * 0.2), at + Vector2(0, h * 0.5)])
	_draw_shape(points, Color(SIGNAL, 0.85) if on else Color(1, 1, 1, 0.07))

# Filled shapes with anti-aliased edges (the fill alone is drawn without anti-aliasing)
func _draw_dot(center: Vector2, radius: float, color: Color) -> void:
	draw_circle(center, radius - 0.5, color)
	draw_arc(center, radius - 0.5, 0.0, TAU, 32, color, 1.0, true)

func _draw_shape(points: PackedVector2Array, color: Color) -> void:
	draw_colored_polygon(points, color)
	var outline := points.duplicate()
	outline.append(points[0])
	draw_polyline(outline, color, 1.0, true)

func _draw_pill(rect: Rect2, fill: Color, border: Color, s: float) -> void:
	var box := StyleBoxFlat.new()
	box.bg_color = fill
	box.border_color = border
	box.set_border_width_all(maxi(1, int(round(1.2 * s))))
	box.set_corner_radius_all(int(rect.size.y * 0.5))
	box.anti_aliasing = true
	draw_style_box(box, rect)

# `baseline` is the text's baseline
func _draw_text_center(font: Font, text: String, baseline: Vector2, font_size: float, color: Color) -> void:
	var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size)).x
	draw_string(font, baseline - Vector2(width * 0.5, 0), text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size), color)

func _draw_text_right(font: Font, text: String, baseline: Vector2, font_size: float, color: Color) -> void:
	var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size)).x
	draw_string(font, baseline - Vector2(width, 0), text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size), color)

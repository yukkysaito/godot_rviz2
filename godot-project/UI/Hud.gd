extends Control
class_name Hud

# Heads-up display drawn over the 3D view, crisp at any resolution (vector drawing, no images):
# - top left: the operation mode (autonomous / manual)
# - top right: the clock
# - bottom center: the speed, the turn signals, the steering angle and, when the vehicle is planned
#   to stop ahead, the distance to the stop
# Sized for a 1080 px high window and scaled with the window height.

@export var trajectory_mesh_path: NodePath  # for the distance to the planned stop
@export var stop_display_range: float = 150.0  # [m] the stop shows within this distance
@export var max_steering_angle: float = 0.5  # [rad] at the end of the steering arc
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
	queue_redraw()

func _draw() -> void:
	var s := size.y / 1080.0
	if _show_status:
		_draw_mode(Vector2(28, 28) * s, s)
		_draw_text_right(FONT_LIGHT, _clock, Vector2(size.x - 32 * s, 56 * s), 30 * s, TEXT)
	if _show_cluster:
		_draw_cluster(Vector2(size.x * 0.5, size.y - 92 * s), s)

# A pill with a dot and AUTONOMOUS / MANUAL
func _draw_mode(origin: Vector2, s: float) -> void:
	var text := "AUTONOMOUS" if _autonomous else "MANUAL"
	var font_size := int(15 * s)
	var width := _labels.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
	var rect := Rect2(origin, Vector2(width + 52 * s, 36 * s))
	_draw_pill(rect, PANEL, Color(ACCENT, 0.5) if _autonomous else Color(1, 1, 1, 0.12), s)
	_draw_dot(origin + Vector2(20, 18) * s, 3.5 * s, ACCENT if _autonomous else MUTED)
	draw_string(_labels, origin + Vector2(36 * s, 23.5 * s), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size,
		TEXT if _autonomous else MUTED)

func _draw_cluster(center: Vector2, s: float) -> void:
	# A soft shade under it, so that it reads over any scene
	_draw_shade(Rect2(center.x - 420 * s, size.y - 260 * s, 840 * s, 260 * s), 0.5)

	# Speed
	var kmh := int(round(absf(RosBridge.velocity) * 3.6))
	_draw_text_center(FONT_BOLD, str(kmh), center + Vector2(0, 18 * s), 76 * s, TEXT)
	_draw_text_center(_labels, "KM/H", center + Vector2(0, 46 * s), 13 * s, MUTED)

	# Steering: a thin arc over the speed with a mark at the current angle
	var radius := 78 * s
	var arc_center := center + Vector2(0, 8 * s)
	var half_span := 0.55
	draw_arc(arc_center, radius, -PI * 0.5 - half_span, -PI * 0.5 + half_span, 48, Color(1, 1, 1, 0.14), 2 * s, true)
	var ratio := clampf(absf(RosBridge.steering_angle) / max_steering_angle, 0.0, 1.0)
	var steer := -signf(RosBridge.steering_angle) * pow(ratio, STEERING_CURVE)  # > 0: to the right
	var angle := -PI * 0.5 + steer * half_span
	if absf(steer) > 0.01:  # the arc from the middle to the mark, colored
		draw_arc(arc_center, radius, minf(-PI * 0.5, angle), maxf(-PI * 0.5, angle), 32, ACCENT, 2.5 * s, true)
	var mark := arc_center + Vector2(cos(angle), sin(angle)) * radius
	_draw_dot(mark, 6 * s, Color(ACCENT, 0.18))
	_draw_dot(mark, 3.5 * s, ACCENT)

	# Turn signals, blinking
	var blink := fmod(Time.get_ticks_msec() / 1000.0, 0.8) < 0.5
	_draw_signal(center + Vector2(-150 * s, -2 * s), -1.0, RosBridge.turn_left and blink, s)
	_draw_signal(center + Vector2(150 * s, -2 * s), 1.0, RosBridge.turn_right and blink, s)

	# Distance to the planned stop
	var stop_distance: float = _trajectory_mesh.stop_distance if _trajectory_mesh != null else -1.0
	if stop_distance > 1.0 and stop_distance < stop_display_range:
		var text := "STOP  %d m" % int(round(stop_distance))
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

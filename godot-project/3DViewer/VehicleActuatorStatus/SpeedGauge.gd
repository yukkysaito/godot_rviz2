# SpeedGauge.gd
# Vector-drawn 270-degree holographic speed gauge (replaces the PNG console meter).
# Subscribes to /vehicle/status/velocity_status via VelocityReport.
extends Control
class_name SpeedGauge

const FONT: Font = preload("res://SourceCodePro-Light.otf")

const COLOR_CYAN := Color(0.0, 0.898039, 1.0)
const COLOR_TIP := Color(0.92, 0.99, 1.0)
const COLOR_AMBER := Color(1.0, 0.701961, 0.0)
const COLOR_TRACK := Color(1.0, 1.0, 1.0, 0.08)
const COLOR_TICK := Color(1.0, 1.0, 1.0, 0.28)
const COLOR_TICK_TEXT := Color(1.0, 1.0, 1.0, 0.38)

const ARC_START_DEG := 135.0
const ARC_SPAN_DEG := 270.0
const TRACK_WIDTH := 10.0
const REDRAW_EPSILON_KMH := 0.05

@export var topic: String = "/vehicle/status/velocity_status"
@export var max_kmh: float = 60.0
@export var warn_kmh: float = 50.0
@export var tick_step_kmh: float = 10.0
@export_range(1.0, 30.0, 0.5) var smoothing_speed: float = 8.0

var _velocity_report := VelocityReport.new()
var _target_kmh := 0.0
var _display_kmh := 0.0
var _drawn_kmh := -1.0

@onready var _speed_label: Label = $SpeedLabel

func _ready() -> void:
	_velocity_report.subscribe(topic, false)
	_update_readout()

func _process(delta: float) -> void:
	if _velocity_report.has_new():
		_target_kmh = absf(_velocity_report.get_velocity()) * 3.6
		_velocity_report.set_old()

	_display_kmh = lerpf(_display_kmh, _target_kmh, clampf(smoothing_speed * delta, 0.0, 1.0))
	if absf(_display_kmh - _target_kmh) < 0.01:
		_display_kmh = _target_kmh

	# Redraw only when the value moved meaningfully.
	if absf(_display_kmh - _drawn_kmh) > REDRAW_EPSILON_KMH:
		_drawn_kmh = _display_kmh
		_update_readout()
		queue_redraw()

func _accent_color() -> Color:
	if _display_kmh > warn_kmh:
		return COLOR_AMBER
	return COLOR_CYAN.lerp(COLOR_TIP, clampf(_display_kmh / max_kmh, 0.0, 1.0))

func _update_readout() -> void:
	if _speed_label == null:
		return
	_speed_label.text = str(int(roundf(_display_kmh)))
	_speed_label.add_theme_color_override("font_color", _accent_color())

func _draw() -> void:
	var center := size * 0.5
	var radius := minf(size.x, size.y) * 0.5 - 14.0
	if radius <= 20.0:
		return

	var start := deg_to_rad(ARC_START_DEG)
	var span := deg_to_rad(ARC_SPAN_DEG)
	var frac := clampf(_display_kmh / max_kmh, 0.0, 1.0)
	var over_warn := _display_kmh > warn_kmh
	var tip_color := COLOR_AMBER if over_warn else COLOR_TIP
	var glow := COLOR_AMBER if over_warn else COLOR_CYAN

	# Background track.
	draw_arc(center, radius, start, start + span, 96, COLOR_TRACK, TRACK_WIDTH, true)

	# Tick marks + numbers every tick_step_kmh.
	var step := maxf(tick_step_kmh, 1.0)
	var v := 0.0
	while v <= max_kmh + 0.001:
		var a := start + (v / max_kmh) * span
		var dir := Vector2(cos(a), sin(a))
		draw_line(center + dir * (radius - 10.0), center + dir * (radius - 16.0), COLOR_TICK, 1.5, true)
		var text := str(int(roundf(v)))
		var text_size := FONT.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, 10)
		var text_pos := center + dir * (radius - 27.0) - Vector2(text_size.x * 0.5, -text_size.y * 0.3)
		draw_string(FONT, text_pos, text, HORIZONTAL_ALIGNMENT_LEFT, -1, 10, COLOR_TICK_TEXT)
		v += step

	if frac <= 0.004:
		return

	var value_end := start + frac * span

	# Soft outer glow (wider translucent arcs under the value arc).
	draw_arc(center, radius, start, value_end, 96,
		Color(glow.r, glow.g, glow.b, 0.14), TRACK_WIDTH + 12.0, true)
	draw_arc(center, radius, start, value_end, 96,
		Color(glow.r, glow.g, glow.b, 0.06), TRACK_WIDTH + 26.0, true)

	# Value arc with a per-point gradient (cyan -> tip color).
	var segments := maxi(int(ceilf(frac * 64.0)), 2)
	var points := PackedVector2Array()
	var colors := PackedColorArray()
	for i in segments + 1:
		var t := float(i) / float(segments)
		var a := start + t * frac * span
		points.push_back(center + Vector2(cos(a), sin(a)) * radius)
		colors.push_back(COLOR_CYAN.lerp(tip_color, t * frac))
	draw_polyline_colors(points, colors, TRACK_WIDTH, true)

	# Bright cap at the tip.
	var tip: Vector2 = points[points.size() - 1]
	draw_circle(tip, TRACK_WIDTH * 0.85, Color(glow.r, glow.g, glow.b, 0.35))
	draw_circle(tip, TRACK_WIDTH * 0.5, tip_color)

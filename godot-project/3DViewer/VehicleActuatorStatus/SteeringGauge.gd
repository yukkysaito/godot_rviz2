# SteeringGauge.gd
# Vector-drawn steering-angle dial (replaces the PNG steering wheel).
# Subscribes to /vehicle/status/steering_status via SteeringReport.
# Positive angle (radians) = steering to the LEFT, matching the sign
# convention used by the legacy Steering.gd (which rotates -angle).
extends Control
class_name SteeringGauge

const COLOR_CYAN := Color(0.0, 0.898039, 1.0)
const COLOR_TIP := Color(0.92, 0.99, 1.0)
const COLOR_TRACK := Color(1.0, 1.0, 1.0, 0.08)
const COLOR_TICK := Color(1.0, 1.0, 1.0, 0.28)
const COLOR_TICK_CENTER := Color(1.0, 1.0, 1.0, 0.5)
const COLOR_NEEDLE := Color(0.92, 0.99, 1.0)

const TRACK_WIDTH := 6.0
const REDRAW_EPSILON_DEG := 0.05

@export var topic: String = "/vehicle/status/steering_status"
## Tire angle (degrees) that deflects the needle to the end of the arc.
@export var max_angle_deg: float = 40.0
## Half of the arc span; needle sweeps [-half, +half] around straight-up.
@export var arc_half_span_deg: float = 60.0
@export_range(1.0, 30.0, 0.5) var smoothing_speed: float = 10.0

var _steering_report := SteeringReport.new()
var _target_deg := 0.0
var _display_deg := 0.0
var _drawn_deg := 999.0

@onready var _angle_label: Label = $AngleLabel

func _ready() -> void:
	_steering_report.subscribe(topic, false)
	_update_readout()

func _process(delta: float) -> void:
	if _steering_report.has_new():
		_target_deg = rad_to_deg(_steering_report.get_angle())
		_steering_report.set_old()

	_display_deg = lerpf(_display_deg, _target_deg, clampf(smoothing_speed * delta, 0.0, 1.0))
	if absf(_display_deg - _target_deg) < 0.005:
		_display_deg = _target_deg

	# Redraw only when the value moved meaningfully.
	if absf(_display_deg - _drawn_deg) > REDRAW_EPSILON_DEG:
		_drawn_deg = _display_deg
		_update_readout()
		queue_redraw()

func _update_readout() -> void:
	if _angle_label == null:
		return
	_angle_label.text = "%+.1f°" % _display_deg

func _draw() -> void:
	var pivot := Vector2(size.x * 0.5, size.y - 22.0)
	var radius := minf(size.x * 0.5 - 12.0, size.y - 36.0)
	if radius <= 10.0:
		return

	var up := -PI / 2.0
	var half := deg_to_rad(arc_half_span_deg)
	var frac := clampf(_display_deg / max_angle_deg, -1.0, 1.0)
	# Positive (left) steering deflects the needle to the left of straight-up.
	var needle_theta := up - frac * half

	# Background track.
	draw_arc(pivot, radius, up - half, up + half, 64, COLOR_TRACK, TRACK_WIDTH, true)

	# Tick marks at -1, -0.5, 0, +0.5, +1 of the span (inside the track).
	for i in 5:
		var f := -1.0 + 0.5 * float(i)
		var a := up - f * half
		var dir := Vector2(cos(a), sin(a))
		var is_center := absf(f) < 0.01
		var col := COLOR_TICK_CENTER if is_center else COLOR_TICK
		var inner := radius - (13.0 if is_center else 9.0)
		draw_line(pivot + dir * inner, pivot + dir * (radius - 4.0), col, 1.5, true)

	# Value arc from center to the needle, with glow and gradient.
	if absf(frac) > 0.004:
		draw_arc(pivot, radius, up, needle_theta, 48,
			Color(0, 0.898039, 1, 0.15), TRACK_WIDTH + 8.0, true)
		var segments := maxi(int(ceilf(absf(frac) * 32.0)), 2)
		var points := PackedVector2Array()
		var colors := PackedColorArray()
		for i in segments + 1:
			var t := float(i) / float(segments)
			var a := lerpf(up, needle_theta, t)
			points.push_back(pivot + Vector2(cos(a), sin(a)) * radius)
			colors.push_back(COLOR_CYAN.lerp(COLOR_TIP, t * absf(frac)))
		draw_polyline_colors(points, colors, TRACK_WIDTH, true)

	# Needle (soft cyan glow line under a bright core) + hub.
	var ndir := Vector2(cos(needle_theta), sin(needle_theta))
	var n_from := pivot + ndir * 14.0
	var n_to := pivot + ndir * (radius - 5.0)
	draw_line(n_from, n_to, Color(0, 0.898039, 1, 0.25), 5.0, true)
	draw_line(n_from, n_to, COLOR_NEEDLE, 2.0, true)
	draw_circle(pivot, 3.5, COLOR_CYAN)

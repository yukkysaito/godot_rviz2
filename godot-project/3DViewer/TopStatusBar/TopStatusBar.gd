extends PanelContainer
class_name TopStatusBar

# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------
enum DrivingMode { MANUAL, AUTONOMOUS }

# Backing field to avoid setter recursion
var _driving_mode: DrivingMode = DrivingMode.MANUAL

## Optional read access (public)
func get_driving_mode() -> DrivingMode:
	return _driving_mode

func set_driving_mode(mode: DrivingMode) -> void:
	_set_driving_mode_internal(mode)

func set_autonomous(enabled: bool) -> void:
	set_driving_mode(DrivingMode.AUTONOMOUS if enabled else DrivingMode.MANUAL)

func set_weather_icon(tex: Texture2D) -> void:
	if _weather_icon == null:
		return
	_weather_icon.texture = tex
	_weather_icon.visible = (tex != null)

func set_temperature_text(text: String) -> void:
	if _temp_label == null:
		return
	_temp_label.text = text
	_temp_label.visible = (text != "")

# -----------------------------------------------------------------------------
# Inspector (customizable for public repo)
# -----------------------------------------------------------------------------
@export_group("Clock")
@export var clock_update_interval_sec: float = 1.0:
	set(value):
		clock_update_interval_sec = max(value, 0.1)
		_apply_clock_timer_settings()

@export var show_seconds: bool = false:
	set(value):
		show_seconds = value
		_refresh_time()

@export_group("Text")
@export var manual_text: String = "Manual":
	set(value):
		manual_text = value
		_apply_driving_mode_style()

@export var autonomous_text: String = "Autonomous":
	set(value):
		autonomous_text = value
		_apply_driving_mode_style()

@export_group("Mode Badge Colors (Holo)")
## Manual = amber pill (warning accent of the holographic cockpit palette).
@export var manual_badge_bg: Color = Color(1.0, 0.701961, 0.0) # amber #FFB300
@export var manual_badge_fg: Color = Color(1.0, 0.85, 0.45) # bright amber

## Autonomous = cyan glow pill.
@export var autonomous_badge_bg: Color = Color(0.0, 0.898039, 1.0) # cyan #00E5FF
@export var autonomous_badge_fg: Color = Color(0.62, 0.97, 1.0) # bright cyan

## Alpha applied to the pill background (kept low for the glass look).
@export_range(0.0, 1.0, 0.01) var badge_alpha: float = 0.18

## Subtle pulse animation on the badge while in autonomous mode.
@export var autonomous_pulse_enabled: bool = true
@export_range(0.2, 5.0, 0.1) var pulse_period_sec: float = 1.8
@export_range(0.0, 1.0, 0.01) var pulse_min_alpha: float = 0.72

@export_group("Nodes")
## NodePath is more robust than hard-coded $ paths for public repos.
@export var time_label_path: NodePath = NodePath("Margin/Row/Left/TimeLabel")
@export var weather_icon_path: NodePath = NodePath("Margin/Row/Left/WeatherIcon")
@export var temp_label_path: NodePath = NodePath("Margin/Row/Left/TempLabel")

# Mode badge:
# Right/ModeBadge (PanelContainer)
#   └ ModeLabel (Label)
@export var mode_badge_path: NodePath = NodePath("Margin/Row/Right/ModeBadge")
@export var mode_label_path: NodePath = NodePath("Margin/Row/Right/ModeBadge/ModeLabel")

# Visible Toggle
@export var top_status_bar_toggle: BaseButton
# -----------------------------------------------------------------------------
# Private state
# -----------------------------------------------------------------------------
var _clock_timer: Timer

var _time_label: Label
var _weather_icon: TextureRect
var _temp_label: Label

var _mode_badge: PanelContainer
var _mode_label: Label
var _badge_style: StyleBoxFlat # cached duplicated stylebox

const _WDAYS: Array[String] = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"]
const _MONTHS: Array[String] = ["Jan", "Feb", "Mar", "Apr", "May", "Jun",
	"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]

# -----------------------------------------------------------------------------
# Lifecycle
# -----------------------------------------------------------------------------
func _ready() -> void:
	visible = top_status_bar_toggle.button_pressed
	_bind_nodes()

	# Initial render (order matters)
	_refresh_time()
	_apply_driving_mode_style()

	_ensure_clock_timer()
	_apply_clock_timer_settings()

# -----------------------------------------------------------------------------
# Private: node binding
# -----------------------------------------------------------------------------
func _bind_nodes() -> void:
	_time_label = get_node_or_null(time_label_path) as Label
	_weather_icon = get_node_or_null(weather_icon_path) as TextureRect
	_temp_label = get_node_or_null(temp_label_path) as Label

	_mode_badge = get_node_or_null(mode_badge_path) as PanelContainer
	_mode_label = get_node_or_null(mode_label_path) as Label

	if _time_label == null:
		push_warning("TopStatusBar: TimeLabel not found at '%s'." % [String(time_label_path)])

	if _mode_badge == null:
		push_warning("TopStatusBar: ModeBadge not found at '%s'." % [String(mode_badge_path)])
	if _mode_label == null:
		push_warning("TopStatusBar: ModeLabel not found at '%s'." % [String(mode_label_path)])

	_prepare_badge_style()

func _prepare_badge_style() -> void:
	if _mode_badge == null:
		return

	# Ensure we have a StyleBoxFlat to edit without affecting global theme
	var base: StyleBox = _mode_badge.get_theme_stylebox("panel")
	if base == null:
		_badge_style = StyleBoxFlat.new()
	else:
		_badge_style = base.duplicate(true) as StyleBoxFlat

	if _badge_style == null:
		_badge_style = StyleBoxFlat.new()

	# Rounded pill (you tuned it to 8; keep as-is)
	_badge_style.corner_radius_top_left = 8
	_badge_style.corner_radius_top_right = 8
	_badge_style.corner_radius_bottom_left = 8
	_badge_style.corner_radius_bottom_right = 8

	# Padding (you tuned top/bottom to 0)
	_badge_style.content_margin_left = 12
	_badge_style.content_margin_right = 12
	_badge_style.content_margin_top = 0
	_badge_style.content_margin_bottom = 0

	# Soft glow behind the pill; color is set per driving mode.
	_badge_style.shadow_size = 6
	_badge_style.shadow_offset = Vector2.ZERO
	_badge_style.shadow_color = Color(0.0, 0.898039, 1.0, 0.35)

	# Thin luminous border; color is set per driving mode.
	_badge_style.border_width_left = 1
	_badge_style.border_width_right = 1
	_badge_style.border_width_top = 1
	_badge_style.border_width_bottom = 1
	_badge_style.border_color = Color(0.0, 0.898039, 1.0, 0.55)

	_badge_style.anti_aliasing = true

	_mode_badge.add_theme_stylebox_override("panel", _badge_style)

# -----------------------------------------------------------------------------
# Private: clock timer
# -----------------------------------------------------------------------------
func _ensure_clock_timer() -> void:
	if _clock_timer != null:
		return

	_clock_timer = Timer.new()
	_clock_timer.one_shot = false
	_clock_timer.autostart = false
	_clock_timer.timeout.connect(_on_clock_tick)
	add_child(_clock_timer)

func _apply_clock_timer_settings() -> void:
	if not is_inside_tree():
		return
	_ensure_clock_timer()

	_clock_timer.wait_time = max(clock_update_interval_sec, 0.1)
	_clock_timer.start()

func _on_clock_tick() -> void:
	_refresh_time()

# -----------------------------------------------------------------------------
# Private: driving mode (badge)
# -----------------------------------------------------------------------------
func _set_driving_mode_internal(mode: DrivingMode) -> void:
	if _driving_mode == mode:
		return
	_driving_mode = mode
	_apply_driving_mode_style()

func _apply_driving_mode_style() -> void:
	if _mode_label == null or _mode_badge == null:
		return
	if _badge_style == null:
		_prepare_badge_style()
		if _badge_style == null:
			return

	match _driving_mode:
		DrivingMode.AUTONOMOUS:
			_mode_label.text = autonomous_text
			_apply_badge_colors(autonomous_badge_bg, autonomous_badge_fg)
			_start_badge_pulse()
		DrivingMode.MANUAL:
			_mode_label.text = manual_text
			_apply_badge_colors(manual_badge_bg, manual_badge_fg)
			_stop_badge_pulse()

func _apply_badge_colors(bg: Color, fg: Color) -> void:
	# Text
	_mode_label.modulate = fg

	# Background (translucent glass pill)
	var base: Color = bg
	base.a = badge_alpha
	_badge_style.bg_color = base

	# Luminous border + glow matching the mode color
	_badge_style.border_color = Color(fg.r, fg.g, fg.b, 0.55)
	_badge_style.shadow_color = Color(bg.r, bg.g, bg.b, 0.35)

	_mode_badge.queue_redraw()

# -----------------------------------------------------------------------------
# Private: badge pulse (subtle breathing glow while autonomous)
# -----------------------------------------------------------------------------
var _pulse_tween: Tween

func _start_badge_pulse() -> void:
	if not autonomous_pulse_enabled:
		return
	if _mode_badge == null or not is_inside_tree():
		return
	_stop_badge_pulse()
	var half: float = pulse_period_sec * 0.5
	_pulse_tween = create_tween().set_loops().set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)
	_pulse_tween.tween_property(_mode_badge, "modulate:a", pulse_min_alpha, half)
	_pulse_tween.tween_property(_mode_badge, "modulate:a", 1.0, half)

func _stop_badge_pulse() -> void:
	if _pulse_tween != null and _pulse_tween.is_valid():
		_pulse_tween.kill()
	_pulse_tween = null
	if _mode_badge != null:
		_mode_badge.modulate.a = 1.0

# -----------------------------------------------------------------------------
# Private: time formatting
# -----------------------------------------------------------------------------
func _refresh_time() -> void:
	if _time_label == null:
		return
	_time_label.text = _format_datetime_en_like_bar(show_seconds)

func _format_datetime_en_like_bar(include_seconds: bool) -> String:
	var d: Dictionary = Time.get_datetime_dict_from_system()

	var weekday: int = int(d.get("weekday", 0))
	if weekday < 0 or weekday > 6:
		weekday = 0

	var month_index: int = int(d.get("month", 1)) - 1
	if month_index < 0 or month_index > 11:
		month_index = 0

	var wd: String = _WDAYS[weekday]
	var mo: String = _MONTHS[month_index]

	var dd: int = int(d.get("day", 1))
	var hh: int = int(d.get("hour", 0))
	var mm: int = int(d.get("minute", 0))

	if include_seconds:
		var ss: int = int(d.get("second", 0))
		return "%s %s %d %02d:%02d:%02d" % [wd, mo, dd, hh, mm, ss]

	return "%s %s %d %02d:%02d" % [wd, mo, dd, hh, mm]


func _on_top_bar_toggle_toggled(toggled_on):
	visible = toggled_on

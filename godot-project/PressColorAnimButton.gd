# PressColorAnimButton.gd
class_name PressColorAnimButton
extends Button

@export var hover_time := 0.20
@export var press_time := 0.18
@export var release_time := 0.22
@export var easing := Tween.EASE_OUT
@export var trans := Tween.TRANS_SINE

var _tween: Tween
var _sb_anim: StyleBoxFlat
var _col_normal: Color
var _col_hover: Color
var _col_pressed: Color
var _is_down := false

func _ready() -> void:
	# Get the StyleBoxFlat defined in the GUI (use fallback if missing)
	var sb_n_any := get_theme_stylebox("normal")
	var sb_h_any := get_theme_stylebox("hover")
	var sb_p_any := get_theme_stylebox("pressed")

	var sb_n: StyleBoxFlat = null
	var sb_h: StyleBoxFlat = null
	var sb_p: StyleBoxFlat = null
	if sb_n_any is StyleBoxFlat:
		sb_n = sb_n_any as StyleBoxFlat
	if sb_h_any is StyleBoxFlat:
		sb_h = sb_h_any as StyleBoxFlat
	if sb_p_any is StyleBoxFlat:
		sb_p = sb_p_any as StyleBoxFlat

	# Determine target colors (with fallback)
	if sb_n != null:
		_col_normal = sb_n.bg_color
	else:
		_col_normal = Color(0.2, 0.2, 0.2)
	if sb_h != null:
		_col_hover = sb_h.bg_color
	else:
		_col_hover = _col_normal
	if sb_p != null:
		_col_pressed = sb_p.bg_color
	else:
		_col_pressed = _col_hover

	# Prepare one shared StyleBox used for all states
	if sb_n != null:
		_sb_anim = sb_n.duplicate()
	else:
		_sb_anim = StyleBoxFlat.new()

	add_theme_stylebox_override("normal",  _sb_anim)
	add_theme_stylebox_override("hover",   _sb_anim)
	add_theme_stylebox_override("pressed", _sb_anim)

	# Initial color (hover color if already hovered, otherwise normal)
	if is_hovered():
		_sb_anim.bg_color = _col_hover
	else:
		_sb_anim.bg_color = _col_normal

	# Connect signals to handle color interpolation only
	mouse_entered.connect(_on_mouse_entered)
	mouse_exited.connect(_on_mouse_exited)
	button_down.connect(_on_button_down)
	button_up.connect(_on_button_up)
func _on_mouse_entered() -> void:
	if _is_down: return
	_tween_to(_col_hover, hover_time)

func _on_mouse_exited() -> void:
	if _is_down: return
	_tween_to(_col_normal, hover_time)

func _on_button_down() -> void:
	_is_down = true
	_tween_to(_col_pressed, press_time)

func _on_button_up() -> void:
	_is_down = false
	if is_hovered():
		_tween_to(_col_hover, release_time)
	else:
		_tween_to(_col_normal, release_time)

func _tween_to(target: Color, dur: float) -> void:
	if _tween != null and _tween.is_running():
		_tween.kill()
	_tween = create_tween().set_ease(easing).set_trans(trans)
	_tween.tween_property(_sb_anim, "bg_color", target, dur)

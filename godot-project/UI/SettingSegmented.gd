extends Control
class_name SettingSegmented

# A segmented choice that shows and changes a setting with a few fixed values (see
# Core/Settings.gd), for choices short enough to show them all: one click (or tap) picks one,
# instead of opening a list. values[i] is shown as labels[i]. The chosen one is lit in the accent
# blue, and the light slides to a new choice. Drawn in the HUD's colors (see Menu/MenuTheme.gd).

const HEIGHT := 30.0
const ANIM_TIME := 0.18

@export var key: String
@export var values: PackedStringArray
@export var labels: PackedStringArray

var _selected := 0
var _shown := 0.0  # the lit segment's index, animated between them
var _hovered := -1
var _tween: Tween

func _ready() -> void:
	custom_minimum_size = Vector2(MenuTheme.RIGHT_COLUMN, HEIGHT)
	size_flags_vertical = Control.SIZE_SHRINK_CENTER
	mouse_filter = Control.MOUSE_FILTER_STOP
	mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	Settings.bind(key, func(value): _select(maxi(values.find(str(value)), 0), false))
	mouse_exited.connect(func():
		_hovered = -1
		queue_redraw())

func _gui_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		var index := _index_at(event.position.x)
		if index != _hovered:
			_hovered = index
			queue_redraw()
	elif event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		var index := _index_at(event.position.x)
		if index != _selected:
			_select(index, true)
			Settings.set_value(key, values[index])
		accept_event()

func _index_at(x: float) -> int:
	return clampi(int(x / _segment_width()), 0, values.size() - 1)

func _segment_width() -> float:
	return size.x / maxf(values.size(), 1)

func _select(index: int, animate: bool) -> void:
	_selected = index
	if _tween:
		_tween.kill()
	if not animate:
		_shown = index
		queue_redraw()
		return
	_tween = create_tween().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	_tween.tween_method(_set_shown, _shown, float(index), ANIM_TIME)

func _set_shown(value: float) -> void:
	_shown = value
	queue_redraw()

func _draw() -> void:
	var accent := MenuTheme.ACCENT
	var track := Rect2(Vector2.ZERO, size)
	var box := StyleBoxFlat.new()
	box.bg_color = Color(1, 1, 1, 0.05)
	box.border_color = Color(1, 1, 1, 0.12)
	box.set_border_width_all(1)
	box.set_corner_radius_all(8)
	box.corner_detail = 6
	draw_style_box(box, track)

	var width := _segment_width()
	# Dividers between the segments not lit
	for i in range(1, values.size()):
		if absf(i - 0.5 - _shown) > 0.6:
			var x := roundf(width * i)
			draw_line(Vector2(x, 8), Vector2(x, size.y - 8), Color(1, 1, 1, 0.1), 1.0)
	# The lit one
	var lit := Rect2(Vector2(width * _shown, 0), Vector2(width, size.y)).grow(-2.0)
	var glow := StyleBoxFlat.new()
	glow.bg_color = Color(accent, 0.24)
	glow.border_color = Color(accent, 0.9)
	glow.set_border_width_all(1)
	glow.set_corner_radius_all(6)
	glow.corner_detail = 6
	glow.shadow_color = Color(accent, 0.25)
	glow.shadow_size = 6
	draw_style_box(glow, lit)

	var font := get_theme_font("font", "Label")
	var font_size := 14
	for i in values.size():
		var text := labels[i] if i < labels.size() else values[i]
		var color := MenuTheme.MUTED
		if i == _selected:
			color = Color.WHITE
		elif i == _hovered:
			color = MenuTheme.TEXT
		var text_size := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size)
		var pos := Vector2(width * i + (width - text_size.x) * 0.5,
			(size.y - text_size.y) * 0.5 + font.get_ascent(font_size))
		draw_string(font, pos, text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, color)

extends CheckButton
class_name SettingToggle

# A toggle that shows and changes a boolean setting (see Core/Settings.gd), drawn as a switch in
# the HUD's colors: its knob slides over and its track lights up in the accent blue when on (the
# theme, Menu/MenuTheme.gd, keeps the room of the check box for it).

const SWITCH_SIZE := Vector2i(42, 24)
const ANIM_TIME := 0.16

@export var key: String

var _on := 0.0  # 0 off .. 1 on, animated
var _tween: Tween

func _ready() -> void:
	Settings.bind(key, func(value):
		set_pressed_no_signal(bool(value))
		_slide(false))
	toggled.connect(func(on: bool):
		Settings.set_value(key, on)
		_slide(true))
	mouse_entered.connect(queue_redraw)
	mouse_exited.connect(queue_redraw)

func _slide(animate: bool) -> void:
	var target := 1.0 if button_pressed else 0.0
	if _tween:
		_tween.kill()
	if not animate or not is_inside_tree():
		_set_on(target)
		return
	_tween = create_tween().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	_tween.tween_method(_set_on, _on, target, ANIM_TIME)

func _set_on(value: float) -> void:
	_on = value
	queue_redraw()

func _draw() -> void:
	var track := Rect2(Vector2(size.x - SWITCH_SIZE.x, (size.y - SWITCH_SIZE.y) * 0.5), Vector2(SWITCH_SIZE))
	var hover := is_hovered()
	var accent := MenuTheme.ACCENT
	# Track: dim when off, the accent with a soft glow when on
	if _on > 0.0:
		_pill(track.grow(3.0), Color(accent, 0.18 * _on), Color(0, 0, 0, 0))
	var off_fill := Color(1, 1, 1, 0.1 if hover else 0.07)
	var off_border := Color(1, 1, 1, 0.22 if hover else 0.16)
	_pill(track, off_fill.lerp(Color(accent, 0.85), _on), off_border.lerp(Color(accent.lightened(0.25), 1.0), _on))
	# Knob
	var radius := SWITCH_SIZE.y * 0.5 - 3.0
	var x := lerpf(track.position.x + 3.0 + radius, track.end.x - 3.0 - radius, _on)
	var center := Vector2(x, track.get_center().y)
	draw_circle(center + Vector2(0, 1), radius, Color(0, 0, 0, 0.25), true, -1.0, true)
	draw_circle(center, radius, Color(0.7, 0.74, 0.82).lerp(Color.WHITE, _on), true, -1.0, true)

func _pill(rect: Rect2, fill: Color, border: Color) -> void:
	var box := PillStyleBox.new()
	box.bg_color = fill
	box.border_color = border
	box.border_width = 1.0 if border.a > 0.0 else 0.0
	draw_style_box(box, rect)

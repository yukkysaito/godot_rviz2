extends Control
class_name GameSlide

# Opens the mini game (Game/PlannerGame.gd) with a swipe from the right edge of the view to the
# left: the game slides in following the finger (or the mouse) and, on release, settles open or
# closed by where it is and how fast it moves. It closes with its close button and with Esc.
# While the game covers the view, the viewer's 3D view is not drawn (it is hidden anyway), so that
# the game costs little; the viewer keeps receiving its data. Made by Game/MiniGame.gd.

const EDGE := 24.0  # [px] from the right edge, where the swipe starts
const DRAG_START := 10.0  # [px]
const FLING_SPEED := 600.0  # [px/s]
const ANIM_TIME := 0.3

var _game: PlannerGame
var _open := false
var _press := false
var _dragging := false
var _press_x := 0.0
var _last_x := 0.0
var _last_time := 0.0
var _speed := 0.0
var _tween: Tween
var _viewer_3d_was_disabled := false
var _covering := false  # the 3D view is turned off for the game

func _ready() -> void:
	set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_game = PlannerGame.new()
	_game.name = "PlannerGame"
	add_child(_game)
	_game.close_requested.connect(func(): _settle(false))
	resized.connect(_fit)
	_fit()
	_set_shown(false)

func _fit() -> void:
	_game.size = size
	if not _dragging and (_tween == null or not _tween.is_running()):
		_game.position = Vector2(0.0 if _open else size.x, 0.0)

func _input(event: InputEvent) -> void:
	if _open:
		if event.is_action_pressed("ui_cancel"):
			_settle(false)
			get_viewport().set_input_as_handled()
		return
	var position := Vector2.ZERO
	var phase := ""
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		phase = "press" if event.pressed else "release"
		position = event.position
	elif event is InputEventScreenTouch and event.index == 0:
		phase = "press" if event.pressed else "release"
		position = event.position
	elif event is InputEventMouseMotion and event.button_mask & MOUSE_BUTTON_MASK_LEFT:
		phase = "move"
		position = event.position
	elif event is InputEventScreenDrag and event.index == 0:
		phase = "move"
		position = event.position
	else:
		return
	match phase:
		"press":
			if _press or position.x < size.x - EDGE:
				return
			_press = true
			_dragging = false
			_press_x = position.x
			_last_x = position.x
			_last_time = Time.get_ticks_msec() / 1000.0
			_speed = 0.0
		"move":
			if not _press:
				return
			if not _dragging and _press_x - position.x >= DRAG_START:
				_dragging = true
				if _tween:
					_tween.kill()
				_game.prepare()
				_set_shown(true)
			if _dragging:
				var now := Time.get_ticks_msec() / 1000.0
				if now > _last_time:
					_speed = lerpf(_speed, (position.x - _last_x) / (now - _last_time), 0.5)
				_last_x = position.x
				_last_time = now
				_game.position.x = clampf(position.x, 0.0, size.x)
		"release":
			if not _press:
				return
			_press = false
			if _dragging:
				_dragging = false
				if absf(_speed) >= FLING_SPEED:
					_settle(_speed < 0.0)
				else:
					_settle(_game.position.x < size.x * 0.6)
	get_viewport().set_input_as_handled()

func _settle(open: bool) -> void:
	_open = open
	if _tween:
		_tween.kill()
	_game.prepare()
	_set_shown(true)
	if open:
		_game.open()
	else:
		_game.close()
		_cover_viewer(false)
	_tween = create_tween().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	_tween.tween_property(_game, "position:x", 0.0 if open else size.x, ANIM_TIME)
	if open:
		_tween.tween_callback(_cover_viewer.bind(true))
	else:
		_tween.tween_callback(_set_shown.bind(false))

# Turns the viewer's 3D drawing off while the game covers it, and back as it was
func _cover_viewer(covered: bool) -> void:
	if covered == _covering:
		return
	_covering = covered
	var viewport := get_viewport()
	if covered:
		_viewer_3d_was_disabled = viewport.disable_3d
		viewport.disable_3d = true
	else:
		viewport.disable_3d = _viewer_3d_was_disabled

func _set_shown(shown: bool) -> void:
	_game.visible = shown

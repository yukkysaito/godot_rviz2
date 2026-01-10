extends Control
class_name MenuSlide

@export var menu_panel_path: NodePath
@export var edge_handle_path: NodePath
@export var window_settings_path: NodePath  # WindowSettings から通知を受ける

@export var open_hidden_px: float = 0.0     # 0なら全部見える
@export var anim_time: float = 0.22
@export var easing: Tween.EaseType = Tween.EASE_OUT
@export var transition: Tween.TransitionType = Tween.TRANS_QUAD

@export var handle_closed_x: float = 0.0
@export var handle_outset: float = 0.0

@onready var menu_panel: Control = get_node(menu_panel_path)
@onready var edge_handle: Button = get_node(edge_handle_path)
@onready var window_settings: Node = get_node(window_settings_path)

var _is_open := false
var _tween: Tween

func _ready() -> void:
	edge_handle.pressed.connect(_toggle)
	edge_handle.focus_mode = Control.FOCUS_NONE

	await get_tree().process_frame
	_apply_layout(true)

	# WindowSettings 側でモード変更したら再レイアウト
	if window_settings.has_signal("window_changed"):
		window_settings.connect("window_changed", Callable(self, "_on_window_changed"))

func _on_window_changed() -> void:
	# 次フレームでサイズが確定することがある
	call_deferred("_relayout_deferred")

func _relayout_deferred() -> void:
	menu_panel.queue_sort()
	await get_tree().process_frame
	_apply_layout(true)

func _toggle() -> void:
	_is_open = !_is_open
	_apply_layout(false)

func _apply_layout(immediate: bool) -> void:
	var w := menu_panel.size.x
	if w <= 0.0:
		return

	var closed_x := -w
	var open_x := -open_hidden_px
	var menu_x := open_x if _is_open else closed_x

	var handle_x := handle_closed_x
	if _is_open:
		handle_x = (menu_x + w) + handle_outset

	_kill_tween()

	if immediate:
		menu_panel.position.x = menu_x
		edge_handle.position.x = handle_x
	else:
		_tween = create_tween().set_ease(easing).set_trans(transition)
		_tween.tween_property(menu_panel, "position:x", menu_x, anim_time)
		_tween.parallel().tween_property(edge_handle, "position:x", handle_x, anim_time)

	edge_handle.text = "<" if _is_open else ">"

func _kill_tween() -> void:
	if is_instance_valid(_tween):
		_tween.kill()

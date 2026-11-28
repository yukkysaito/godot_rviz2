extends Control
class_name MenuSlide

# The settings menu, sliding in from the left edge. Its panel is frosted glass in the HUD's colors
# (Menu/menu_backdrop.gdshader: the view behind it blurred and tinted dark blue) with rounded right
# corners and a thin outline; while it is open the view darkens a little towards it, and its
# contents fade in as it slides.
# Its handle is a slim grabber (a short vertical pill) at the left edge of the view, quiet until
# hovered, when it grows and lights up in the accent blue; it rides along the panel's right edge
# while the menu is open. Its button is much larger than the pill, to be easy to hit.
# It opens and closes with a click (or tap) on the grabber, and with a drag (or swipe) of the
# grabber, of the open menu's title, or, while closed, from the view's left edge: the panel follows
# the pointer and, on release, settles open or closed by where it is and how fast it moves. Esc
# (ui_cancel) closes it too.
# The backdrop, the outline and the shade are drawn behind the panel by nodes made here, which
# follow it; they are hidden while it is closed, so that the blur costs nothing then.
# Like the HUD, the menu is sized for a 1080 px high window and scaled with the window's height
# (the text is drawn at the scaled size, so it stays sharp), but not shrunk below its size.

@export var menu_panel_path: NodePath
@export var edge_handle_path: NodePath

@export var anim_time: float = 0.28
@export var easing: Tween.EaseType = Tween.EASE_OUT
@export var transition: Tween.TransitionType = Tween.TRANS_CUBIC

const CORNER := 24.0  # [px] radius of the panel's right corners
const LINE := Color(1, 1, 1, 0.12)  # the panel's outline
const SHADE := 0.35  # the view's darkening at the panel's edge, fading out across the view
const HANDLE_SIZE := Vector2(28, 120)  # [px] the grabber's button
const HANDLE_GAP := 2.0  # [px] between the open panel and the grabber's button
const GRABBER := Vector2(4, 40)  # [px] the pill, and hovered:
const GRABBER_HOVERED := Vector2(5, 56)
const EDGE_SWIPE := 24.0  # [px] from the view's left edge, where a drag opens the closed menu
const DRAG_START := 8.0  # [px] the pointer moves before a press becomes a drag
const FLING_SPEED := 400.0  # [px/s] a release faster than this settles the way it moves
const REFERENCE_HEIGHT := 1080.0  # [px] of the window the menu is sized for

@onready var menu_panel: Control = get_node(menu_panel_path)
@onready var edge_handle: Button = get_node(edge_handle_path)

var _is_open := false
var _tween: Tween
var _shade: TextureRect
var _backdrop: Control
var _edge: Control
var _shown := 0.0  # 0 closed .. 1 open, animated: the shade and the contents' opacity
var _hover := 0.0  # of the grabber, animated
var _hover_tween: Tween
# Dragging the panel: a press on the grabber, the title or the left edge, then its motion
var _press := false  # a press that may become a drag
var _press_on_handle := false  # a click on the grabber toggles the menu
var _dragging := false
var _press_position := Vector2.ZERO
var _press_panel_x := 0.0
var _last_x := 0.0
var _last_time := 0.0
var _speed := 0.0  # [px/s] of the pointer along x, smoothed

func _ready() -> void:
	menu_panel.theme = MenuTheme.build()
	var margins := StyleBoxEmpty.new()
	margins.content_margin_left = 24
	margins.content_margin_top = 22
	margins.content_margin_right = 20
	margins.content_margin_bottom = 20
	menu_panel.add_theme_stylebox_override("panel", margins)
	_align_choices(menu_panel)
	_add_backdrop()
	_style_handle()

	# Wait one frame so Control sizes are initialized by layout
	await get_tree().process_frame
	_apply_layout(true)

	# Re-layout when the window changes (e.g. its mode in the display settings)
	get_viewport().size_changed.connect(_on_window_changed)

func _process(_delta: float) -> void:
	_follow_panel()

# Clicks and drags on the grabber, the title and the left edge (a touch, or the mouse emulated from
# it, comes as either; the two copies of an event do the same)
func _input(event: InputEvent) -> void:
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
			if _press or not _starts_drag(position):
				return
			_press = true
			_dragging = false
			_press_on_handle = edge_handle.get_global_rect().has_point(position)
			_press_position = position
			_press_panel_x = menu_panel.position.x
			_last_x = position.x
			_last_time = Time.get_ticks_msec() / 1000.0
			_speed = 0.0
		"move":
			if not _press:
				return
			if not _dragging and absf(position.x - _press_position.x) >= DRAG_START:
				_dragging = true
				_kill_tween()
				_set_layers_visible(true)
			if _dragging:
				_drag_to(position.x)
		"release":
			if not _press:
				return
			_press = false
			if _dragging:
				_dragging = false
				_settle()
			elif _press_on_handle:
				_toggle()
	get_viewport().set_input_as_handled()

func _starts_drag(position: Vector2) -> bool:
	if edge_handle.get_global_rect().has_point(position):
		return true
	if not _is_open:
		return position.x <= EDGE_SWIPE
	var title := menu_panel.find_child("MenuTitle", true, false) as Control
	return title != null and title.get_global_rect().has_point(position)

func _closed_x() -> float:
	return -menu_panel.size.x * menu_panel.scale.x - 2.0  # just off-screen, with its outline

# The menu's scale for the window: as the HUD's, but not below 1
func _ui_scale() -> float:
	return clampf(get_viewport_rect().size.y / REFERENCE_HEIGHT, 1.0, 2.0)  # not smaller: its text stays legible

# Scales the menu and fits the panel to the window's height
func _apply_scale() -> void:
	var s := _ui_scale()
	menu_panel.anchor_bottom = 0.0
	menu_panel.grow_vertical = Control.GROW_DIRECTION_END
	menu_panel.scale = Vector2(s, s)
	menu_panel.size.y = get_viewport_rect().size.y / s
	menu_panel.position.y = 0.0
	edge_handle.pivot_offset = Vector2(0, edge_handle.size.y * 0.5)
	edge_handle.scale = Vector2(s, s)
	_scale_popups(menu_panel, s)

func _scale_popups(node: Node, s: float) -> void:
	if node is OptionButton:
		(node as OptionButton).get_popup().content_scale_factor = s
	for child in node.get_children():
		_scale_popups(child, s)

func _drag_to(x: float) -> void:
	var now := Time.get_ticks_msec() / 1000.0
	if now > _last_time:
		_speed = lerpf(_speed, (x - _last_x) / (now - _last_time), 0.5)
	_last_x = x
	_last_time = now
	var closed_x := _closed_x()
	menu_panel.position.x = clampf(_press_panel_x + x - _press_position.x, closed_x, 0.0)
	_set_shown(inverse_lerp(closed_x, 0.0, menu_panel.position.x))

# After a drag: open or closed, the way it was flung or else the nearer
func _settle() -> void:
	if absf(_speed) >= FLING_SPEED:
		_is_open = _speed > 0.0
	else:
		_is_open = _shown >= 0.5
	_apply_layout(false)

func _unhandled_input(event: InputEvent) -> void:
	if _is_open and event.is_action_pressed("ui_cancel"):
		_toggle()
		get_viewport().set_input_as_handled()

# The backdrop, the outline and the grabber follow the panel
func _follow_panel() -> void:
	var rect := Rect2(menu_panel.position, menu_panel.size)  # unscaled (get_rect() is scaled)
	var s := menu_panel.scale.x
	edge_handle.position.x = maxf(rect.position.x + rect.size.x * s + HANDLE_GAP, 0.0)
	if _backdrop.position != rect.position or _backdrop.size != rect.size or _backdrop.scale != menu_panel.scale:
		for node: Control in [_backdrop, _edge]:
			node.position = rect.position
			node.size = rect.size
			node.scale = menu_panel.scale
			node.queue_redraw()
		(_backdrop.material as ShaderMaterial).set_shader_parameter("height", rect.size.y)

func _add_backdrop() -> void:
	# A shade over the view, darkest at the left, behind everything of the menu
	var gradient := Gradient.new()
	gradient.set_color(0, Color(0.01, 0.02, 0.04, SHADE))
	gradient.set_color(1, Color(0.01, 0.02, 0.04, 0.0))
	var texture := GradientTexture2D.new()
	texture.gradient = gradient
	texture.fill_to = Vector2(0.7, 0)
	_shade = TextureRect.new()
	_shade.name = "Shade"
	_shade.texture = texture
	_shade.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_shade.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_shade.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_shade)
	move_child(_shade, 0)

	var material := ShaderMaterial.new()
	material.shader = preload("res://Menu/menu_backdrop.gdshader")
	_backdrop = Control.new()
	_backdrop.name = "Backdrop"
	_backdrop.material = material
	_backdrop.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_backdrop.draw.connect(func():
		# White: the shader takes the color from the view, and the alpha (the rounded corners'
		# coverage) from here
		_backdrop.draw_style_box(_panel_box(Color.WHITE, true), Rect2(Vector2.ZERO, _backdrop.size)))
	add_child(_backdrop)
	move_child(_backdrop, 1)

	_edge = Control.new()
	_edge.name = "Edge"
	_edge.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_edge.draw.connect(func():
		# The outline, not along the screen's edge
		_edge.draw_style_box(_panel_box(LINE, false), Rect2(Vector2(-2, 0), _edge.size + Vector2(2, 0))))
	add_child(_edge)
	move_child(_edge, 2)

static func _panel_box(color: Color, filled: bool) -> StyleBoxFlat:
	var box := StyleBoxFlat.new()
	box.draw_center = filled
	box.bg_color = color
	if not filled:
		box.border_color = color
		box.border_width_top = 1
		box.border_width_right = 1
		box.border_width_bottom = 1
	box.corner_radius_top_right = int(CORNER)
	box.corner_radius_bottom_right = int(CORNER)
	box.corner_detail = 12
	box.anti_aliasing = true
	return box

func _style_handle() -> void:
	edge_handle.text = ""
	edge_handle.focus_mode = Control.FOCUS_NONE
	edge_handle.tooltip_text = "Settings"
	edge_handle.mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	edge_handle.custom_minimum_size = HANDLE_SIZE
	edge_handle.offset_top = -HANDLE_SIZE.y * 0.5
	edge_handle.offset_bottom = HANDLE_SIZE.y * 0.5
	edge_handle.offset_right = edge_handle.offset_left + HANDLE_SIZE.x
	for state in ["normal", "hover", "pressed", "hover_pressed", "focus", "disabled"]:
		edge_handle.add_theme_stylebox_override(state, StyleBoxEmpty.new())
	edge_handle.draw.connect(_draw_grabber)
	edge_handle.mouse_entered.connect(_set_hovered.bind(true))
	edge_handle.mouse_exited.connect(_set_hovered.bind(false))

func _set_hovered(hovered: bool) -> void:
	if _hover_tween:
		_hover_tween.kill()
	_hover_tween = create_tween().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	_hover_tween.tween_method(_set_hover, _hover, 1.0 if hovered else 0.0, 0.15)

func _set_hover(value: float) -> void:
	_hover = value
	edge_handle.queue_redraw()

# The grabber: a short vertical pill, faint, growing and lit in the accent blue when hovered
func _draw_grabber() -> void:
	var accent := MenuTheme.ACCENT
	var pill_size := GRABBER.lerp(GRABBER_HOVERED, _hover)
	var center := edge_handle.size * 0.5
	if not _is_open:
		center.x = 10.0  # near the view's edge
	var rect := Rect2(center - pill_size * 0.5, pill_size)
	var box := PillStyleBox.new()
	box.border_width = 0.0
	if _hover > 0.0:
		box.bg_color = Color(accent, 0.22 * _hover)
		edge_handle.draw_style_box(box, rect.grow(3.0 * _hover))
	box.bg_color = Color(1, 1, 1, 0.3).lerp(Color(accent.lightened(0.1), 1.0), _hover)
	edge_handle.draw_style_box(box, rect)

func _on_window_changed() -> void:
	# Sometimes Control sizes finalize on the next frame
	call_deferred("_relayout_deferred")

func _relayout_deferred() -> void:
	# Ensure layout pass happens, then apply positions using finalized sizes
	menu_panel.queue_sort()
	await get_tree().process_frame
	_apply_layout(true)

func _toggle() -> void:
	_is_open = !_is_open
	_apply_layout(false)

func _apply_layout(immediate: bool) -> void:
	if menu_panel.size.x <= 0.0:
		return
	_apply_scale()

	# Closed: the panel (with its outline) just off-screen to the left
	var menu_x := 0.0 if _is_open else _closed_x()

	_kill_tween()
	edge_handle.queue_redraw()
	var shown := 1.0 if _is_open else 0.0
	if _is_open:
		_set_layers_visible(true)

	if immediate:
		menu_panel.position.x = menu_x
		_set_shown(shown)
		_set_layers_visible(_is_open)
	else:
		_tween = create_tween().set_ease(easing).set_trans(transition)
		_tween.tween_property(menu_panel, "position:x", menu_x, anim_time)
		_tween.parallel().tween_method(_set_shown, _shown, shown, anim_time)
		if not _is_open:
			_tween.tween_callback(_set_layers_visible.bind(false))

func _set_shown(value: float) -> void:
	_shown = value
	_shade.modulate.a = value
	menu_panel.modulate.a = lerpf(0.3, 1.0, value)

func _set_layers_visible(shown: bool) -> void:
	_shade.visible = shown
	_backdrop.visible = shown
	_edge.visible = shown
	_follow_panel()
	set_process(shown)

func _kill_tween() -> void:
	# Prevent stacking tweens (e.g., repeated clicks)
	if is_instance_valid(_tween):
		_tween.kill()

# The choices all as wide, aligned on the right
func _align_choices(node: Node) -> void:
	if node is OptionButton:
		(node as OptionButton).fit_to_longest_item = false
		(node as OptionButton).custom_minimum_size.x = MenuTheme.RIGHT_COLUMN
	for child in node.get_children():
		_align_choices(child)

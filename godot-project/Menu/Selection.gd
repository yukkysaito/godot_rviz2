extends VBoxContainer

# A section of the settings menu: a header that folds its body in and out. The body unrolls (its
# height grows from 0) and fades in. The header draws a thin line above it, the section's icon (a
# few strokes in the accent blue) and a chevron that turns down when it is open.

@export var title: String = "Section"
@export_enum("display", "data", "view", "hud") var icon: String = ""
@export var default_open := false

const ANIM_TIME := 0.22

@onready var header: Control = $Header
@onready var title_label: Label = $Header/Title
@onready var chevron_label: Label = $Header/Chevron
@onready var body: Control = $Body

var _open := false
var _clip: Control  # holds the body, clipped to its animated height
var _tween: Tween
var _turn := 0.0  # of the chevron: 0 pointing right (closed) .. 1 down (open)
var _hover := false

func _ready() -> void:
	title_label.text = title.to_upper()
	title_label.theme_type_variation = &"SectionTitle"
	title_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
	title_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	title_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	chevron_label.text = ""  # drawn by the header
	chevron_label.custom_minimum_size.x = 20
	header.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	header.add_theme_constant_override("separation", 0)
	size_flags_horizontal = Control.SIZE_EXPAND_FILL

	header.mouse_filter = Control.MOUSE_FILTER_STOP
	header.mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	header.gui_input.connect(_on_header_input)
	header.mouse_entered.connect(func():
		_hover = true
		header.queue_redraw())
	header.mouse_exited.connect(func():
		_hover = false
		header.queue_redraw())
	header.draw.connect(_draw_header)

	# The body goes into a plain Control (not a container), whose height can be animated
	_clip = Control.new()
	_clip.name = "Clip"
	_clip.clip_contents = true
	_clip.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_clip.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	add_child(_clip)
	move_child(_clip, body.get_index())
	body.reparent(_clip, false)
	body.position = Vector2.ZERO
	_clip.resized.connect(_fit_body)
	body.minimum_size_changed.connect(_fit_body)

	_set_open(default_open, false)

func _on_header_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		_set_open(not _open, true)

func _fit_body() -> void:
	body.size = Vector2(_clip.size.x, body.get_combined_minimum_size().y)
	if _open and (_tween == null or not _tween.is_running()):
		_clip.custom_minimum_size.y = body.size.y

func _set_open(open: bool, animate: bool) -> void:
	_open = open
	var height := body.get_combined_minimum_size().y if open else 0.0
	var alpha := 1.0 if open else 0.0
	if _tween:
		_tween.kill()
	if not animate:
		_clip.custom_minimum_size.y = height
		body.modulate.a = alpha
		_clip.visible = open
		_set_turn(1.0 if open else 0.0)
		return
	_clip.visible = true
	_tween = create_tween().set_parallel().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	_tween.tween_method(_set_turn, _turn, 1.0 if open else 0.0, ANIM_TIME)
	_tween.tween_property(_clip, "custom_minimum_size:y", height, ANIM_TIME)
	_tween.tween_property(body, "modulate:a", alpha, ANIM_TIME * (1.0 if open else 0.6))
	if not open:
		_tween.chain().tween_callback(func(): _clip.visible = false)

func _set_turn(value: float) -> void:
	_turn = value
	header.queue_redraw()

# The line above the section, its icon before the title and the chevron at the end, all centered on
# the title's text
func _draw_header() -> void:
	var accent := MenuTheme.ACCENT
	var width := header.size.x
	header.draw_line(Vector2(0, 0.5), Vector2(width, 0.5), Color(1, 1, 1, 0.08), 1.0)
	var box := title_label.get_theme_stylebox("normal")
	var top := box.get_margin(SIDE_TOP)
	var y := top + (header.size.y - top - box.get_margin(SIDE_BOTTOM)) * 0.5
	var color := accent if _hover or _turn > 0.5 else Color(accent, 0.75)
	_draw_icon(Vector2(8, y), color)
	# Chevron: right when closed, turned down when open
	var center := Vector2(width - 8, y)
	var angle := _turn * PI * 0.5
	var points := PackedVector2Array([Vector2(-2.5, -5), Vector2(2.5, 0), Vector2(-2.5, 5)])
	for i in points.size():
		points[i] = center + points[i].rotated(angle)
	header.draw_polyline(points, MenuTheme.TEXT if _hover else MenuTheme.MUTED, 1.6, true)

# The section's icon, about 16 px, centered at `c`
func _draw_icon(c: Vector2, color: Color) -> void:
	var w := 1.5
	match icon:
		"display":  # a screen on a stand
			_rect(Rect2(c + Vector2(-8, -6.5), Vector2(16, 10.5)), color, w)
			header.draw_line(c + Vector2(0, 4), c + Vector2(0, 7), color, w, true)
			header.draw_line(c + Vector2(-4, 7.5), c + Vector2(4, 7.5), color, w, true)
		"data":  # stacked layers
			for dy in [-4.0, 0.0, 4.0]:
				var o := c + Vector2(0, dy)
				header.draw_polyline(PackedVector2Array([o + Vector2(-7.5, 0), o + Vector2(0, -3.5),
					o + Vector2(7.5, 0), o + Vector2(0, 3.5), o + Vector2(-7.5, 0)]), color, w, true)
		"view":  # a cube
			var top := PackedVector2Array([c + Vector2(0, -8), c + Vector2(7, -4), c + Vector2(0, 0),
				c + Vector2(-7, -4), c + Vector2(0, -8)])
			header.draw_polyline(top, color, w, true)
			header.draw_line(c + Vector2(-7, -4), c + Vector2(-7, 4), color, w, true)
			header.draw_line(c + Vector2(7, -4), c + Vector2(7, 4), color, w, true)
			header.draw_line(c, c + Vector2(0, 8), color, w, true)
			header.draw_polyline(PackedVector2Array([c + Vector2(-7, 4), c + Vector2(0, 8), c + Vector2(7, 4)]),
				color, w, true)
		"hud":  # a gauge
			header.draw_arc(c + Vector2(0, 3), 7.5, PI * 0.85, PI * 2.15, 24, color, w, true)
			header.draw_line(c + Vector2(0, 3), c + Vector2(4, -2), color, w, true)
			header.draw_circle(c + Vector2(0, 3), 1.6, color, true, -1.0, true)

func _rect(rect: Rect2, color: Color, width: float) -> void:
	var box := StyleBoxFlat.new()
	box.draw_center = false
	box.border_color = color
	box.set_border_width_all(int(width))
	box.set_corner_radius_all(2)
	box.anti_aliasing = true
	header.draw_style_box(box, rect)

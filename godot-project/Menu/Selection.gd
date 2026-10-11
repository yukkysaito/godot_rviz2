extends VBoxContainer

# A section of the settings menu: a header that folds its body in and out. The body unrolls (its
# height grows from 0) and fades in; the chevron shows whether it is open.

@export var title: String = "Section"
@export var default_open := false

const ANIM_TIME := 0.22

@onready var header: Control = $Header
@onready var title_label: Label = $Header/Title
@onready var chevron_label: Label = $Header/Chevron
@onready var body: Control = $Body

var _open := false
var _clip: Control  # holds the body, clipped to its animated height
var _tween: Tween

func _ready() -> void:
	title_label.text = title.to_upper()
	title_label.theme_type_variation = &"SectionTitle"
	chevron_label.theme_type_variation = &"SectionTitle"
	title_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
	title_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	title_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	chevron_label.custom_minimum_size.x = 28
	chevron_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	chevron_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	header.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	size_flags_horizontal = Control.SIZE_EXPAND_FILL

	header.mouse_filter = Control.MOUSE_FILTER_STOP
	header.gui_input.connect(_on_header_input)
	header.mouse_entered.connect(func(): header.modulate = Color(1, 1, 1, 1))
	header.mouse_exited.connect(func(): header.modulate = Color(0.9, 0.9, 0.9, 1))

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
	chevron_label.text = "⌄" if open else "›"
	if _tween:
		_tween.kill()
	if not animate:
		_clip.custom_minimum_size.y = height
		body.modulate.a = alpha
		_clip.visible = open
		return
	_clip.visible = true
	_tween = create_tween().set_parallel().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_CUBIC)
	_tween.tween_property(_clip, "custom_minimum_size:y", height, ANIM_TIME)
	_tween.tween_property(body, "modulate:a", alpha, ANIM_TIME * (1.0 if open else 0.6))
	if not open:
		_tween.chain().tween_callback(func(): _clip.visible = false)

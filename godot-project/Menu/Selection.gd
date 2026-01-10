extends VBoxContainer

@export var title: String = "Section"
@export var default_open := false

@onready var header: Control = $Header
@onready var title_label: Label = $Header/Title
@onready var chevron_label: Label = $Header/Chevron
@onready var body: Control = $Body

var _open := true

func _ready() -> void:
	_open = default_open
	body.visible = _open

	title_label.text = title
	title_label.custom_minimum_size.y = 28
	title_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	title_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	title_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	chevron_label.custom_minimum_size.x = 28
	chevron_label.custom_minimum_size.y = 28
	chevron_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	chevron_label.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	header.size_flags_horizontal = Control.SIZE_EXPAND_FILL	
	size_flags_horizontal = Control.SIZE_EXPAND_FILL

	header.mouse_filter = Control.MOUSE_FILTER_STOP
	header.gui_input.connect(_on_header_input)

	header.mouse_entered.connect(
		func(): header.modulate = Color(1,1,1,1)
	)
	header.mouse_exited.connect(
		func(): header.modulate = Color(0.9,0.9,0.9,1)
	)

	_update_chevron()

func _on_header_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed:
		_toggle()

func _toggle() -> void:
	_open = !_open
	body.visible = _open
	_update_chevron()

func _update_chevron() -> void:
	chevron_label.text = "⌄" if _open else "›"

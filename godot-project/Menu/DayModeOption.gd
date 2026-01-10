extends OptionButton

@export var world_path: NodePath
@onready var world: Node = get_node(world_path) as Node

enum Preset { DAY, NIGHT }

var _suppress := false

func _ready() -> void:
	_setup_items()
	item_selected.connect(_on_item_selected)

	# 初期状態反映
	_suppress = true
	select(Preset.DAY)
	_apply_preset(Preset.DAY)
	_suppress = false

func _setup_items() -> void:
	clear()
	add_item("Day", Preset.DAY)
	add_item("Night", Preset.NIGHT)

func _on_item_selected(index: int) -> void:
	if _suppress:
		return
	var preset := get_item_id(index)
	_apply_preset(preset)

func _apply_preset(preset: int) -> void:
	if world == null:
		return

	match preset:
		Preset.DAY:
			world.set_night_mode(false)
		Preset.NIGHT:
			world.set_night_mode(true)

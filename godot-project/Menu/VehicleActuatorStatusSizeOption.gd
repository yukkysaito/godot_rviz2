extends OptionButton

const SIZE_LABELS := ["50%", "75%", "100%", "125%", "150%"]
const DEFAULT_SIZE_PRESET := 2

func _ready() -> void:
	clear()
	for label in SIZE_LABELS:
		add_item(label)
	select(DEFAULT_SIZE_PRESET)
	call_deferred("_emit_initial_selection")

func _emit_initial_selection() -> void:
	item_selected.emit(DEFAULT_SIZE_PRESET)

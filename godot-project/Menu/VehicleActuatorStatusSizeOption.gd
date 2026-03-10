extends OptionButton

enum Preset {
	PERCENT_50,
	PERCENT_75,
	PERCENT_100,
	PERCENT_125,
	PERCENT_150,
}

func _ready() -> void:
	clear()
	add_item("50%", Preset.PERCENT_50)
	add_item("75%", Preset.PERCENT_75)
	add_item("100%", Preset.PERCENT_100)
	add_item("125%", Preset.PERCENT_125)
	add_item("150%", Preset.PERCENT_150)
	select(_find_index_by_id(Preset.PERCENT_100))
	call_deferred("_emit_initial_selection")

func _emit_initial_selection() -> void:
	item_selected.emit(_find_index_by_id(Preset.PERCENT_100))

func _find_index_by_id(id: int) -> int:
	for i in range(item_count):
		if get_item_id(i) == id:
			return i
	return 0

extends OptionButton
class_name SettingOption

# An option button that shows and changes a setting with a fixed set of values (see
# Core/Settings.gd). values[i] is shown as labels[i].

@export var key: String
@export var values: PackedStringArray
@export var labels: PackedStringArray

func _ready() -> void:
	clear()
	for i in values.size():
		add_item(labels[i] if i < labels.size() else values[i], i)
	Settings.bind(key, func(value): select(maxi(values.find(str(value)), 0)))
	item_selected.connect(func(index: int): Settings.set_value(key, values[index]))

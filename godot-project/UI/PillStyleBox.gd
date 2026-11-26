@tool
extends StyleBox
class_name PillStyleBox

# A pill (a box with fully round ends), smooth at any size: drawn as one outline, which a
# StyleBoxFlat with corner radii of half its height is not (its few-sided corners look jagged, and
# where the top and bottom corners meet at the ends, their anti-aliased edges overlap into a dot).
# Its fill, an anti-aliased border inside its edge and a shadow fading out around it.

@export var bg_color := Color(0, 0, 0, 0.6):
	set(value):
		bg_color = value
		emit_changed()
@export var border_color := Color(1, 1, 1, 0.3):
	set(value):
		border_color = value
		emit_changed()
@export var border_width := 1.0:  # [px], 0: none
	set(value):
		border_width = value
		emit_changed()
@export var shadow_color := Color(0, 0, 0, 0.25):
	set(value):
		shadow_color = value
		emit_changed()
@export var shadow_size := 0.0:  # [px] the shadow fades out over, 0: none
	set(value):
		shadow_size = value
		emit_changed()

func _draw(to_canvas_item: RID, rect: Rect2) -> void:
	if shadow_size > 0.0 and shadow_color.a > 0.0:
		_draw_shadow(to_canvas_item, rect)
	var outline := _outline(rect)
	RenderingServer.canvas_item_add_polygon(to_canvas_item, outline, PackedColorArray([bg_color]))
	if border_width > 0.0:
		var border := _outline(rect.grow(-border_width * 0.5))
		border.append(border[0])
		RenderingServer.canvas_item_add_polyline(to_canvas_item, border, PackedColorArray([border_color]),
			border_width, true)
	else:  # the fill's edge, anti-aliased
		outline.append(outline[0])
		RenderingServer.canvas_item_add_polyline(to_canvas_item, outline, PackedColorArray([bg_color]),
			1.0, true)

# A ring from the pill's edge (shadow_color) to shadow_size around it (transparent)
func _draw_shadow(to_canvas_item: RID, rect: Rect2) -> void:
	var inner := _outline(rect)
	var outer := _outline(rect.grow(shadow_size), inner.size())
	var points := PackedVector2Array()
	var colors := PackedColorArray()
	var indices := PackedInt32Array()
	var clear := Color(shadow_color, 0.0)
	for i in inner.size():
		points.append_array([inner[i], outer[i]])
		colors.append_array([shadow_color, clear])
		var next := (i + 1) % inner.size()
		indices.append_array([2 * i, 2 * i + 1, 2 * next, 2 * next, 2 * i + 1, 2 * next + 1])
	RenderingServer.canvas_item_add_triangle_array(to_canvas_item, indices, points, colors)

# The pill's edge, clockwise from the top of its right end; `count` points (by default enough for
# its size)
static func _outline(rect: Rect2, count: int = 0) -> PackedVector2Array:
	var radius := minf(rect.size.x, rect.size.y) * 0.5
	if count <= 0:
		count = 2 * clampi(roundi(radius * 0.8), 8, 64)
	var half := count / 2
	var center := rect.get_center()
	var axis := Vector2(rect.size.x * 0.5 - radius, 0.0) if rect.size.x >= rect.size.y \
		else Vector2(0.0, rect.size.y * 0.5 - radius)
	var start := -PI * 0.5 if rect.size.x >= rect.size.y else 0.0
	var points := PackedVector2Array()
	for end in 2:  # the far end (+axis), then the near one (-axis)
		var end_center := center + axis * (1.0 - 2.0 * end)
		for i in half:
			var angle := start + PI * (end + float(i) / (half - 1))
			points.append(end_center + Vector2(cos(angle), sin(angle)) * radius)
	return points

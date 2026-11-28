extends Control
class_name PlannerGame

# "Planner Challenge", a mini game opened by a swipe from the right edge (Game/GameSlide.gd): the
# player is the velocity planner of an automated vehicle on a course of Game/PlannerCourse.gd.
# Touch (or click) and drag up to accelerate, down to brake, release to coast; reach the goal fast,
# keep the passengers comfortable, stay safe and stop on the goal line. A rival ghost (a rule based
# planner, Game/PlannerCourse.gd) drives the same course at the same time: its time is the course's
# target time and its score the one to beat. A daily course (the same for everyone on a day) and
# random ones; the best scores are kept.
# Drawn from above as vectors in a cute, pop style: a pastel town of rooftops, parks and trees along
# the road, little rounded cars and people, and white cards for the HUD. The ego's stopping distance
# shows ahead of it as a planned path. It is a game only: it never talks to ROS.

signal close_requested

const FONT_LIGHT := preload("res://UI/Fonts/Barlow-Light.ttf")
const FONT_BOLD := preload("res://UI/Fonts/Barlow-SemiBold.ttf")
const FONT := preload("res://UI/Fonts/Barlow-Medium.ttf")
const INK := Color(0.16, 0.19, 0.32)  # text
const INK_SOFT := Color(0.46, 0.51, 0.65)
const ACCENT := Color(0.33, 0.58, 1.0)  # the ego, the planned path, buttons
const GOOD := Color(0.25, 0.78, 0.5)
const WARN := Color(1.0, 0.7, 0.18)
const BAD := Color(1.0, 0.36, 0.4)
const CARD := Color(1, 1, 1, 0.95)
const SHADOW := Color(0.2, 0.25, 0.5, 0.16)
const GROUND := Color(0.83, 0.92, 0.85)
const SIDEWALK := Color(0.99, 0.97, 0.93)
const ROAD := Color(0.47, 0.51, 0.64)
const NONE := Color(0, 0, 0, 0)
# The town's roofs and the cars
const PASTELS := [Color(1.0, 0.76, 0.72), Color(0.79, 0.76, 1.0), Color(0.66, 0.89, 0.82), Color(0.66, 0.82, 1.0),
	Color(1.0, 0.89, 0.6), Color(1.0, 0.72, 0.86)]
const CAR_COLORS := [Color(1.0, 0.52, 0.5), Color(1.0, 0.8, 0.3), Color(0.4, 0.82, 0.75), Color(0.72, 0.6, 1.0),
	Color(1.0, 0.6, 0.8)]
const SHIRTS := [Color(1.0, 0.45, 0.55), Color(0.35, 0.7, 1.0), Color(1.0, 0.75, 0.2), Color(0.45, 0.8, 0.55),
	Color(0.7, 0.55, 1.0)]
const SAVE_FILE := "user://planner_game.cfg"
const VIEW_METERS := 68.0  # [m] across the view
const EGO_X := 0.17  # of the width, where the ego is drawn
const PEDAL_TRAVEL := 0.12  # of the height, a drag this far is the full command
const COMFORT_DECEL := 2.0  # [m/s^2] of the stopping distance shown ahead of the ego
const MAX_STEPS := 6  # per frame, so that a slow frame does not pile up

enum State { TITLE, COUNTDOWN, RUN, RESULT }

var _state := State.TITLE
var _daily := true
var _course: PlannerCourse
var _run: PlannerCourse.Run
var _ghost_index := 0
var _accumulator := 0.0
var _countdown := 0.0
var _pedal_anchor := Vector2.ZERO
var _pedal_at := Vector2.ZERO
var _pedaling := false
var _keys := 0.0  # the command from the keyboard (up / down), when not touching
var _messages: Array = []  # [text, color, time left]
var _best := ConfigFile.new()
var _new_best := false
var _buttons := {}  # name -> Rect2, of the overlay shown
var _s := 1.0  # the drawing scale (for a 1080 px high view)
var _boxes := {}  # StyleBoxFlat by its look, made once
# The town is drawn once per block, by a node of its own behind the game's drawing, which then only
# moves with the view (block index -> Node2D); the ground under it, by another
var _ground: Control
var _blocks := {}
var _blocks_for := Vector3.ZERO  # the size and course they were drawn for

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	focus_mode = Control.FOCUS_ALL
	# Nothing outside its rect (the town and the road go on past its edges): while it slides in, the
	# viewer beside it stays clear
	clip_contents = true
	set_process(false)
	_ground = Control.new()
	_ground.show_behind_parent = true
	_ground.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_ground.draw.connect(_draw_ground)
	add_child(_ground)

# The course and the saved scores, made the first time the game shows (not when the viewer starts)
func prepare() -> void:
	if _course == null:
		_best.load(SAVE_FILE)
		_new_course()
	_update_town()

# Shown: back to the title, with the course kept
func open() -> void:
	prepare()
	_state = State.TITLE
	_pedaling = false
	set_process(true)
	grab_focus()
	queue_redraw()

func close() -> void:
	set_process(false)
	_pedaling = false

func _new_course() -> void:
	if _daily:
		var date := Time.get_date_dict_from_system()
		var key: int = date.year * 10000 + date.month * 100 + date.day
		_course = PlannerCourse.make(key, "Daily  %d/%02d/%02d" % [date.year, date.month, date.day])
	else:
		var seed := randi() % 100000
		_course = PlannerCourse.make(seed, "Course #%05d" % seed)
	_run = _course.new_run()
	_ghost_index = 0

func _course_key() -> String:
	return "daily_%d" % _course.seed_value if _daily else "course_%d" % _course.seed_value

func _start() -> void:
	_run = _course.new_run()
	_ghost_index = 0
	_accumulator = 0.0
	_messages.clear()
	_new_best = false
	_countdown = 2.4
	_state = State.COUNTDOWN

func _process(delta: float) -> void:
	_s = size.y / 1080.0
	match _state:
		State.COUNTDOWN:
			_countdown -= delta
			if _countdown <= 0.0:
				_state = State.RUN
		State.RUN:
			_accumulator += delta
			var steps := 0
			while _accumulator >= PlannerCourse.DT and steps < MAX_STEPS:
				_accumulator -= PlannerCourse.DT
				steps += 1
				_run.command = _command()
				_run.step()
				_ghost_index += 1
				for event in _run.events:
					_on_event(event)
				if _run.finished:
					_finish()
					break
			_accumulator = minf(_accumulator, PlannerCourse.DT)
	for m in _messages:
		m[2] -= delta
	_messages = _messages.filter(func(m): return m[2] > 0.0)
	_update_town()
	queue_redraw()

func _command() -> float:
	if _pedaling:
		var travel := maxf(size.y * PEDAL_TRAVEL, 1.0)
		var dy := (_pedal_anchor.y - _pedal_at.y) / travel  # up is positive
		if absf(dy) < 0.05:
			return 0.0
		if dy > 0.0:
			return PlannerCourse.MAX_ACCEL * minf(dy, 1.0)
		return PlannerCourse.MAX_DECEL * minf(-dy, 1.0)
	if _keys != 0.0:
		return _keys
	return PlannerCourse.COAST

func _on_event(event: String) -> void:
	match event:
		"aeb":
			_message("AEB  −60", BAD)
		"close":
			_message("Too close", WARN)
		"speeding":
			_message("Over the limit", WARN)

func _message(text: String, color: Color) -> void:
	for m in _messages:
		if m[0] == text:
			m[2] = 1.6
			return
	_messages.append([text, color, 1.6])

func _finish() -> void:
	_pedaling = false
	_state = State.RESULT
	var points := _run.total()
	var key := _course_key()
	if _run.failed == "" and points > float(_best.get_value("best", key, 0.0)):
		_best.set_value("best", key, points)
		_best.save(SAVE_FILE)
		_new_best = true

# --- Input ---

func _gui_input(event: InputEvent) -> void:
	var button := event as InputEventMouseButton
	if button != null and button.button_index == MOUSE_BUTTON_LEFT:
		if button.pressed:
			_press(button.position)
		else:
			_pedaling = false
		accept_event()
	elif event is InputEventMouseMotion and _pedaling:
		_pedal_at = event.position
		accept_event()

func _press(at: Vector2) -> void:
	if _hit("close", at):
		close_requested.emit()
		return
	match _state:
		State.TITLE:
			if _hit("daily", at):
				if not _daily:
					_daily = true
					_new_course()
			elif _hit("random", at):
				_daily = false
				_new_course()
			else:
				_start()
		State.RESULT:
			if _hit("retry", at):
				_start()
			elif _hit("new", at):
				_daily = false
				_new_course()
				_start()
			elif _hit("title", at):
				_state = State.TITLE
		State.COUNTDOWN, State.RUN:
			_pedaling = true
			_pedal_anchor = at
			_pedal_at = at

func _hit(name: String, at: Vector2) -> bool:
	return _buttons.has(name) and (_buttons[name] as Rect2).has_point(at)

func _unhandled_key_input(event: InputEvent) -> void:
	if not is_processing():
		return
	var key := event as InputEventKey
	if key == null or key.echo:
		return
	match key.keycode:
		KEY_UP, KEY_W:
			_keys = 1.5 if key.pressed else 0.0
		KEY_DOWN, KEY_S:
			_keys = -2.5 if key.pressed else 0.0
		KEY_SPACE, KEY_ENTER:
			if key.pressed and _state != State.RUN and _state != State.COUNTDOWN:
				_start()
		_:
			return
	accept_event()

# --- Drawing helpers ---

func _box(fill: Color, border: Color, radius: float, shadow: float = 0.0) -> StyleBoxFlat:
	var key := [fill, border, int(radius), int(shadow)]
	var box: StyleBoxFlat = _boxes.get(key)
	if box == null:
		box = StyleBoxFlat.new()
		box.bg_color = fill
		box.border_color = border
		box.set_border_width_all(2 if border.a > 0.0 else 0)
		box.set_corner_radius_all(int(radius))
		box.corner_detail = 8
		box.anti_aliasing = true
		if shadow > 0.0:
			box.shadow_color = SHADOW
			box.shadow_size = int(shadow)
			box.shadow_offset = Vector2(0, shadow * 0.35)
		_boxes[key] = box
	return box

func _rounded(rect: Rect2, fill: Color, border: Color, radius: float, shadow: float = 0.0) -> void:
	draw_style_box(_box(fill, border, radius, shadow), rect)

func _text(font: Font, text: String, at: Vector2, size_1080: int, color: Color,
		align: HorizontalAlignment = HORIZONTAL_ALIGNMENT_LEFT) -> void:
	var font_size := maxi(1, int(size_1080 * _s))
	var x := at.x
	if align != HORIZONTAL_ALIGNMENT_LEFT:
		var width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
		x -= width * (0.5 if align == HORIZONTAL_ALIGNMENT_CENTER else 1.0)
	draw_string(font, Vector2(x, at.y), text, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, color)

func _text_width(font: Font, text: String, size_1080: int) -> float:
	return font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, maxi(1, int(size_1080 * _s))).x

# A pill with a label, centered at `at`: filled with the color (solid) and white text, or white with
# the color's text (not solid)
func _chip(at: Vector2, text: String, color: Color, solid: bool = true, size_1080: int = 14) -> void:
	var s := _s
	var w := _text_width(FONT_BOLD, text, size_1080) + 26 * s
	var h := (size_1080 + 16) * s
	var rect := Rect2(at - Vector2(w, h) * 0.5, Vector2(w, h))
	_rounded(rect, color if solid else Color(1, 1, 1, 0.95 * color.a), NONE, h * 0.5, 6 * s)
	_text(FONT_BOLD, text, at + Vector2(0, size_1080 * 0.36 * s), size_1080, Color(1, 1, 1, color.a) if solid else color,
		HORIZONTAL_ALIGNMENT_CENTER)

func _ppm() -> float:
	return size.x / VIEW_METERS

func _road_y() -> float:
	return size.y * 0.6

# World (along the road, across it) to the screen, the ego at EGO_X
func _to_screen(s_world: float, y_world: float) -> Vector2:
	var ppm := _ppm()
	return Vector2(size.x * EGO_X + (s_world - _run.s) * ppm, _road_y() + y_world * ppm)

func _visible(s_world: float, margin: float = 10.0) -> bool:
	var x := _to_screen(s_world, 0).x
	var ppm := _ppm()
	return x > -margin * ppm and x < size.x + margin * ppm

# --- Drawing ---

func _draw() -> void:
	if _course == null:
		return
	_s = size.y / 1080.0
	_buttons.clear()
	_draw_road()
	_draw_traffic()
	_draw_hud()
	if _state == State.RUN or _state == State.COUNTDOWN:
		_draw_pedal()
	_draw_messages()
	match _state:
		State.TITLE:
			_draw_title()
		State.COUNTDOWN:
			_draw_countdown()
		State.RESULT:
			_draw_result()
	_draw_close()

const SIDEWALK_WIDTH := 3.0  # [m]
const BLOCK := 36.0  # [m] the town is made in blocks along the road, each from the seed

# The town's blocks in view: made when they come into it (and when the size or the course changed),
# freed when they leave it, moved with the view
func _update_town() -> void:
	if _course == null:
		return
	_s = size.y / 1080.0
	var key := Vector3(size.x, size.y, _course.seed_value)
	if key != _blocks_for:
		for node in _blocks.values():
			node.queue_free()
		_blocks.clear()
		_blocks_for = key
		_ground.size = size
		_ground.queue_redraw()
	var first := int(floor((_run.s - VIEW_METERS * EGO_X - 20.0) / BLOCK))
	var last := int(floor((_run.s + VIEW_METERS + 10.0) / BLOCK))
	for block in _blocks.keys():
		if block < first or block > last:
			_blocks[block].queue_free()
			_blocks.erase(block)
	for block in range(first, last + 1):
		if not _blocks.has(block):
			var node := Node2D.new()
			node.show_behind_parent = true
			node.draw.connect(_draw_block.bind(node, block))
			add_child(node)
			_blocks[block] = node
		(_blocks[block] as Node2D).position.x = _to_screen(block * BLOCK, 0).x

# The ground and the sidewalks (all along the view: they do not move)
func _draw_ground() -> void:
	var ppm := _ppm()
	var lane := PlannerCourse.LANE_WIDTH
	_ground.draw_rect(Rect2(Vector2.ZERO, size), GROUND)
	var top := _road_y() - (lane + SIDEWALK_WIDTH) * ppm
	_ground.draw_rect(Rect2(0, top, size.x, 2.0 * (lane + SIDEWALK_WIDTH) * ppm), SIDEWALK)

# One block of the town on both sides of the road, from the seed: lots of rooftops (with a darker
# side, a soft shadow and things on top) and parks, and trees along the sidewalks. Drawn on its node,
# whose x is the block's start.
func _draw_block(c: Node2D, block: int) -> void:
	var ppm := _ppm()
	var lane := PlannerCourse.LANE_WIDTH
	var rng := RandomNumberGenerator.new()
	rng.seed = hash([_course.seed_value, block])
	for side in [-1, 1]:
		var s0 := 1.0
		while s0 < BLOCK - 4.0:
			var length := minf(rng.randf_range(7.0, 14.0), BLOCK - 1.0 - s0)
			var depth := rng.randf_range(7.0, 12.0)
			var near := (lane + SIDEWALK_WIDTH + 1.2) * ppm
			var y: float = _road_y() + side * near - (depth * ppm if side < 0 else 0.0)
			var lot := Rect2(s0 * ppm, y, length * ppm, depth * ppm)
			if rng.randf() < 0.72:
				_draw_building(c, lot, rng, ppm)
			else:
				_draw_park(c, lot, rng, ppm)
			s0 += length + rng.randf_range(1.5, 3.0)
	for side in [-1, 1]:
		var t := rng.randf_range(3.0, 9.0)
		while t < BLOCK:
			_draw_tree(c, Vector2(t * ppm, _road_y() + side * (lane + SIDEWALK_WIDTH * 0.6) * ppm), 0.9 * ppm, rng)
			t += rng.randf_range(9.0, 14.0)

func _draw_building(c: CanvasItem, lot: Rect2, rng: RandomNumberGenerator, ppm: float) -> void:
	var color: Color = PASTELS[rng.randi_range(0, PASTELS.size() - 1)]
	var r := 0.9 * ppm
	# Its shadow and its side (below the roof, as seen a little from the front)
	c.draw_style_box(_box(Color(0.2, 0.3, 0.3, 0.12), NONE, r), Rect2(lot.position + Vector2(0.5, 1.0) * ppm, lot.size))
	c.draw_style_box(_box(color.darkened(0.18), NONE, r), Rect2(lot.position + Vector2(0, 0.7 * ppm), lot.size))
	c.draw_style_box(_box(color, NONE, r), lot)
	c.draw_style_box(_box(color.lightened(0.18), NONE, r * 0.7), lot.grow(-0.5 * ppm))
	# Things on the roof: a box or two, a round tank, a garden
	for i in rng.randi_range(1, 3):
		var w := rng.randf_range(1.4, 3.0) * ppm
		var h := rng.randf_range(1.2, 2.4) * ppm
		var p := lot.position + Vector2(rng.randf_range(0.8 * ppm, maxf(lot.size.x - w - 0.8 * ppm, 0.8 * ppm)),
			rng.randf_range(0.8 * ppm, maxf(lot.size.y - h - 0.8 * ppm, 0.8 * ppm)))
		match rng.randi_range(0, 2):
			0:
				c.draw_style_box(_box(color.darkened(0.25), NONE, 0.4 * ppm), Rect2(p + Vector2(0, 0.25 * ppm), Vector2(w, h)))
				c.draw_style_box(_box(Color(1, 1, 1, 0.85), NONE, 0.4 * ppm), Rect2(p, Vector2(w, h)))
			1:
				var center := p + Vector2(w, h) * 0.5
				c.draw_circle(center + Vector2(0, 0.2 * ppm), minf(w, h) * 0.45, color.darkened(0.25), true, -1.0, true)
				c.draw_circle(center, minf(w, h) * 0.45, Color(1, 1, 1, 0.9), true, -1.0, true)
			2:
				c.draw_style_box(_box(Color(0.55, 0.82, 0.55), NONE, 0.6 * ppm), Rect2(p, Vector2(w * 1.3, h)))

func _draw_park(c: CanvasItem, lot: Rect2, rng: RandomNumberGenerator, ppm: float) -> void:
	c.draw_style_box(_box(Color(0.62, 0.86, 0.62), NONE, 1.2 * ppm), lot)
	c.draw_style_box(_box(Color(0.7, 0.9, 0.68), NONE, 0.9 * ppm), lot.grow(-0.6 * ppm))
	for i in rng.randi_range(2, 4):
		var at := lot.position + Vector2(rng.randf_range(1.5, maxf(lot.size.x / ppm - 1.5, 1.5)),
			rng.randf_range(1.5, maxf(lot.size.y / ppm - 1.5, 1.5))) * ppm
		_draw_tree(c, at, rng.randf_range(0.9, 1.4) * ppm, rng)

# A round tree from above: its shadow, the crown and a highlight
func _draw_tree(canvas: CanvasItem, c: Vector2, r: float, rng: RandomNumberGenerator) -> void:
	var green := Color(0.35, 0.72, 0.45).lerp(Color(0.5, 0.8, 0.4), rng.randf())
	canvas.draw_circle(c + Vector2(r * 0.25, r * 0.35), r, Color(0.2, 0.35, 0.25, 0.18), true, -1.0, true)
	canvas.draw_circle(c, r, green, true, -1.0, true)
	canvas.draw_circle(c - Vector2(r * 0.3, r * 0.3), r * 0.35, green.lightened(0.3), true, -1.0, true)

func _draw_road() -> void:
	var ppm := _ppm()
	var lane := PlannerCourse.LANE_WIDTH
	var top := _to_screen(0, -lane).y
	var bottom := _to_screen(0, lane).y
	draw_rect(Rect2(0, top - 0.3 * ppm, size.x, 0.3 * ppm), Color(0.85, 0.83, 0.8))
	draw_rect(Rect2(0, bottom, size.x, 0.3 * ppm), Color(0.85, 0.83, 0.8))
	draw_rect(Rect2(0, top, size.x, bottom - top), ROAD)
	# Speed zones: a colored road with their signs
	for zone in _course.zones:
		var x0 := _to_screen(zone.s0, 0).x
		var x1 := _to_screen(zone.s1, 0).x
		if x1 < 0 or x0 > size.x:
			continue
		draw_rect(Rect2(x0, top, x1 - x0, bottom - top), Color(0.6, 0.53, 0.68))
		_draw_limit_sign(_to_screen(zone.s0, lane + 1.6), zone.limit, 1.0 * ppm)
		_draw_limit_sign(_to_screen(zone.s1, lane + 1.6), PlannerCourse.DEFAULT_LIMIT, 1.0 * ppm)
	for y in [top + 0.35 * ppm, bottom - 0.35 * ppm]:
		draw_line(Vector2(0, y), Vector2(size.x, y), Color(1, 1, 1, 0.85), 0.18 * ppm)
	var d := floorf((_run.s - VIEW_METERS) / 6.0) * 6.0
	while d < _run.s + VIEW_METERS:
		var a := _to_screen(d, 0)
		_rounded(Rect2(a - Vector2(0, 0.12 * ppm), Vector2(3.0 * ppm, 0.24 * ppm)), Color(1, 0.92, 0.6), NONE, 0.12 * ppm)
		d += 6.0
	# Crosswalks
	for crosswalk in _run.crosswalks:
		if not _visible(crosswalk.s):
			continue
		var stripe := -lane + 0.3
		while stripe < lane - 0.3:
			var p := _to_screen(crosswalk.s, stripe)
			_rounded(Rect2(p, Vector2(crosswalk.width * ppm, 0.55 * ppm)), Color(1, 1, 1, 0.92), NONE, 0.2 * ppm)
			stripe += 1.0
		var yielding := crosswalk.triggered() and crosswalk.pending(_run.t)
		_draw_stop_line(crosswalk.stop_line, WARN if yielding else Color(1, 1, 1, 0.9))
	# Lights
	for light in _course.lights:
		if _visible(light.s):
			var state := light.state(_run.t)
			_draw_stop_line(light.s, BAD if state == 2 else (WARN if state == 1 else Color(1, 1, 1, 0.9)))
	# Goal
	if _visible(_course.length):
		var g := _to_screen(_course.length, 0)
		var cell := lane * ppm / 6.0
		for i in 6:
			for j in 2:
				draw_rect(Rect2(g.x + j * cell, g.y + i * cell, cell, cell), Color.WHITE if (i + j) % 2 == 0 else INK)

func _draw_traffic() -> void:
	var ppm := _ppm()
	var t := _run.t
	var lane := PlannerCourse.LANE_WIDTH
	for crosswalk in _run.crosswalks:
		if not _visible(crosswalk.s):
			continue
		var i := 0
		for pedestrian in crosswalk.pedestrians:
			var py := pedestrian.y(t) if pedestrian.start < INF else pedestrian.from_y
			var at := _to_screen(crosswalk.s + crosswalk.width * 0.5 + (i - 1) * 0.9, py)
			_draw_person(at, SHIRTS[(i + crosswalk.pedestrians.size()) % SHIRTS.size()], pedestrian.in_lane(t),
				pedestrian.walking(t), ppm)
			i += 1
	for light in _course.lights:
		if _visible(light.s):
			_draw_light(light.s, light.state(t), light.remaining(t), ppm)
	if _visible(_course.length):
		_chip(_to_screen(_course.length + 1.0, lane + 1.8), "GOAL", INK, true, 14)
	var k := 0
	for o in _course.oncoming:
		k += 1
		if t < o.start:
			continue
		var os := o.s(t)
		if _visible(os):
			_draw_car(_to_screen(os, -PlannerCourse.EGO_Y), PI, CAR_COLORS[k % CAR_COLORS.size()], false, ppm)
	k = 2
	for car in _run.cars:
		k += 1
		if not car.active(t):
			continue
		var cs := car.s(t)
		if not _visible(cs):
			continue
		var heading := atan2((car.y(t + 0.1) - car.y(t - 0.1)) / 0.2, maxf(car.v(t), 1.0))
		_draw_car(_to_screen(cs, car.y(t)), heading, CAR_COLORS[k % CAR_COLORS.size()], car.accel(t) < -0.5, ppm)
	var ghost := _course.reference
	if ghost != null and ghost.trace.size() > 0:
		var gs := ghost.trace[mini(_ghost_index, ghost.trace.size() - 1)]
		if _visible(gs):
			_draw_ghost(_to_screen(gs, PlannerCourse.EGO_Y), ppm)
	_draw_planned_path(ppm)
	var ego := _to_screen(_run.s, PlannerCourse.EGO_Y)
	if _run.aeb and fmod(Time.get_ticks_msec() / 120.0, 2.0) < 1.0:
		draw_circle(ego, 3.4 * ppm, Color(BAD, 0.3), true, -1.0, true)
	_draw_car(ego, 0.0, ACCENT, _run.a < -0.4 or _run.aeb, ppm, true)

func _draw_stop_line(s_world: float, color: Color) -> void:
	var ppm := _ppm()
	var a := _to_screen(s_world, 0.3)
	_rounded(Rect2(a - Vector2(0.25 * ppm, 0), Vector2(0.5 * ppm, (PlannerCourse.LANE_WIDTH - 0.6) * ppm)), color, NONE, 0.25 * ppm)

# A friendly light beside the road: a rounded head with the lit lamp, and the time to its next
# change (V2I) as a ring and a number
func _draw_light(s_world: float, state: int, remaining: float, ppm: float) -> void:
	var colors := [GOOD, WARN, BAD]
	var lit: Color = colors[state]
	var at := _to_screen(s_world, PlannerCourse.LANE_WIDTH + 2.2)
	var r := 0.6 * ppm
	var head := Rect2(at - Vector2(r * 3.8, r * 1.4), Vector2(r * 7.6, r * 2.8))
	_rounded(head, INK, NONE, r * 1.4, 8 * _s)
	for i in 3:
		var c := at + Vector2((i - 1) * r * 2.5, 0)
		if i == state:
			draw_circle(c, r * 1.05, lit, true, -1.0, true)
			draw_circle(c - Vector2(r * 0.3, r * 0.3), r * 0.3, Color(1, 1, 1, 0.6), true, -1.0, true)
			draw_arc(c, r * 1.55, -PI * 0.5, -PI * 0.5 + TAU * clampf(remaining / 12.0, 0.0, 1.0), 40, lit,
				0.22 * ppm, true)
		else:
			draw_circle(c, r * 0.8, Color(1, 1, 1, 0.12), true, -1.0, true)
	_chip(at + Vector2(r * 6.0, 0), "%ds" % ceili(remaining), lit, true, 15)

func _draw_limit_sign(at: Vector2, limit: float, r: float) -> void:
	draw_circle(at + Vector2(0, r * 0.18), r, SHADOW, true, -1.0, true)
	draw_circle(at, r, Color.WHITE, true, -1.0, true)
	draw_arc(at, r * 0.84, 0, TAU, 40, BAD, r * 0.24, true)
	_text(FONT_BOLD, "%d" % int(limit), at + Vector2(0, r * 0.3), int(r * 0.85 / _s), INK, HORIZONTAL_ALIGNMENT_CENTER)

# A little rounded car from above: a soft shadow, the body and its window (and a red glow at the
# back while it brakes)
func _draw_car(at: Vector2, heading: float, color: Color, braking: bool, ppm: float, _ego: bool = false) -> void:
	var l := PlannerCourse.EGO_LENGTH * ppm
	var w := 2.0 * ppm
	draw_set_transform_matrix(Transform2D(heading, at))
	_rounded(Rect2(-l * 0.5 + 0.3 * ppm, -w * 0.5 + 0.45 * ppm, l, w), Color(0.15, 0.2, 0.35, 0.22), NONE, w * 0.5)
	if braking:
		draw_circle(Vector2(-l * 0.5, 0), w * 0.55, Color(BAD, 0.35), true, -1.0, true)
	_rounded(Rect2(-l * 0.5, -w * 0.5, l, w), color, NONE, w * 0.5)
	_rounded(Rect2(-l * 0.12, -w * 0.3, l * 0.36, w * 0.6), Color(1, 1, 1, 0.9), NONE, w * 0.26)
	draw_set_transform_matrix(Transform2D.IDENTITY)

func _draw_ghost(at: Vector2, ppm: float) -> void:
	var l := PlannerCourse.EGO_LENGTH * ppm
	var w := 2.0 * ppm
	_rounded(Rect2(at - Vector2(l, w) * 0.5, Vector2(l, w)), Color(1, 1, 1, 0.3), Color(1, 1, 1, 0.9), w * 0.5)

# A little person from above: shoulders in a colorful shirt and a round head, with a pulsing ring
# while in the ego's lane
func _draw_person(at: Vector2, shirt: Color, in_lane: bool, walking: bool, ppm: float) -> void:
	if in_lane:
		var pulse := 0.5 + 0.5 * sin(Time.get_ticks_msec() / 160.0)
		draw_circle(at, (1.2 + 0.25 * pulse) * ppm, Color(BAD, 0.18 + 0.1 * pulse), true, -1.0, true)
	ppm *= 1.5  # drawn larger than life, to be seen
	var bob := sin(Time.get_ticks_msec() / 110.0) * 0.06 * ppm if walking else 0.0
	draw_circle(at + Vector2(0.15, 0.25) * ppm, 0.55 * ppm, Color(0.15, 0.2, 0.35, 0.2), true, -1.0, true)
	draw_set_transform_matrix(Transform2D(0.0, Vector2(1.0, 0.7), 0.0, at + Vector2(0, bob)))
	draw_circle(Vector2.ZERO, 0.55 * ppm, shirt, true, -1.0, true)
	draw_set_transform_matrix(Transform2D.IDENTITY)
	draw_circle(at + Vector2(0, bob), 0.3 * ppm, Color(1.0, 0.86, 0.72), true, -1.0, true)
	draw_circle(at + Vector2(0, bob - 0.08 * ppm), 0.24 * ppm, Color(0.35, 0.25, 0.2), true, -1.0, true)

# Ahead of the ego, the distance it needs to stop comfortably, as a planned path with moving
# chevrons: blue, or red when an obstacle, a red light or a crosswalk to yield at is within it
func _draw_planned_path(ppm: float) -> void:
	var front := _run.front()
	var reach := _run.v * _run.v / (2.0 * COMFORT_DECEL) + _run.v * PlannerCourse.LAG
	if reach < 0.5:
		return
	var limit := INF
	var threat := _run.obstacle_ahead(_run.t, front, _run.v)
	if threat.x < INF:
		limit = threat.x
	for light in _course.lights:
		var d := light.s - front
		if d > 0.0 and light.state(_run.t) != 0:
			limit = minf(limit, d)
	for crosswalk in _run.crosswalks:
		var d := crosswalk.stop_line - front
		if d > 0.0 and crosswalk.triggered() and crosswalk.pending(_run.t):
			limit = minf(limit, d)
	var color := BAD if reach > limit else Color(0.55, 0.85, 1.0)
	var a := _to_screen(front, PlannerCourse.EGO_Y - 0.8)
	var b := _to_screen(front + reach, PlannerCourse.EGO_Y + 0.8)
	draw_polygon(PackedVector2Array([a, Vector2(b.x, a.y), b, Vector2(a.x, b.y)]),
		PackedColorArray([Color(color, 0.55), Color(color, 0.0), Color(color, 0.0), Color(color, 0.55)]))
	var spacing := 2.5
	var k := spacing - fposmod(_run.s, spacing)
	while k < reach:
		var c := _to_screen(front + k, PlannerCourse.EGO_Y)
		var fade := 1.0 - k / reach
		draw_polyline(PackedVector2Array([c + Vector2(-0.3 * ppm, -0.45 * ppm), c + Vector2(0.15 * ppm, 0),
			c + Vector2(-0.3 * ppm, 0.45 * ppm)]), Color(1, 1, 1, 0.9 * fade), 0.18 * ppm, true)
		k += spacing

# --- HUD ---

func _draw_hud() -> void:
	var s := _s
	# The speed card
	var card := Rect2(Vector2(28, 28) * s, Vector2(300, 170) * s)
	_rounded(card, CARD, NONE, 26 * s, 16 * s)
	var kmh := _run.v * 3.6
	var limit := _course.limit_at(_run.s)
	var over := kmh > limit + 3.0
	_text(FONT_BOLD, "%d" % roundi(kmh), card.position + Vector2(26, 96) * s, 76, BAD if over else INK)
	_text(FONT_BOLD, "km/h", card.position + Vector2(30, 124) * s, 14, INK_SOFT)
	_draw_limit_sign(card.position + Vector2(240, 66) * s, limit, 30 * s)
	var bar := Rect2(card.position + Vector2(26, 144) * s, Vector2(248, 10) * s)
	_rounded(bar, Color(INK, 0.08), NONE, 5 * s)
	var mid := bar.position.x + bar.size.x * (3.5 / 5.5)
	var ax := mid + _run.a / 5.5 * bar.size.x
	var a_color := GOOD if _run.a >= 0.0 else (BAD if _run.a < -2.0 else WARN)
	_rounded(Rect2(minf(mid, ax), bar.position.y, maxf(absf(ax - mid), 2.0), bar.size.y), a_color, NONE, 5 * s)
	draw_line(Vector2(mid, bar.position.y - 4 * s), Vector2(mid, bar.end.y + 4 * s), Color(INK, 0.4), 2.0 * s)
	# The badge (a simulation), the progress with the ghost, the course and the time
	_chip(Vector2(size.x * 0.5, 34 * s), "MINI GAME  ·  SIMULATION", WARN, true, 13)
	var track := Rect2(Vector2(size.x * 0.33, 74 * s), Vector2(size.x * 0.34, 10 * s))
	_rounded(track.grow(12 * s), CARD, NONE, 17 * s, 10 * s)
	_rounded(track, Color(INK, 0.08), NONE, 5 * s)
	var done := clampf(_run.s / _course.length, 0, 1)
	_rounded(Rect2(track.position, Vector2(maxf(track.size.x * done, 10 * s), track.size.y)), ACCENT, NONE, 5 * s)
	for crosswalk in _course.crosswalks:
		var x := track.position.x + track.size.x * crosswalk.s / _course.length
		draw_circle(Vector2(x, track.get_center().y), 4 * s, Color.WHITE, true, -1.0, true)
		draw_arc(Vector2(x, track.get_center().y), 4 * s, 0, TAU, 16, INK_SOFT, 1.5 * s, true)
	for light in _course.lights:
		var x := track.position.x + track.size.x * light.s / _course.length
		draw_circle(Vector2(x, track.get_center().y), 6 * s, [GOOD, WARN, BAD][light.state(_run.t)], true, -1.0, true)
	var ghost := _course.reference
	if ghost != null and ghost.trace.size() > 0:
		var gs := ghost.trace[mini(_ghost_index, ghost.trace.size() - 1)]
		var gx := track.position.x + track.size.x * clampf(gs / _course.length, 0, 1)
		draw_circle(Vector2(gx, track.get_center().y), 9 * s, Color.WHITE, true, -1.0, true)
		draw_arc(Vector2(gx, track.get_center().y), 9 * s, 0, TAU, 32, INK_SOFT, 2.0 * s, true)
	var ex := track.position.x + track.size.x * done
	draw_circle(Vector2(ex, track.get_center().y), 10 * s, ACCENT, true, -1.0, true)
	draw_circle(Vector2(ex, track.get_center().y), 5 * s, Color.WHITE, true, -1.0, true)
	_chip(Vector2(size.x * 0.5, 124 * s), "%s   ·   %.1f s" % [_course.name, _run.t], INK_SOFT, false, 14)
	# Comfort and safety
	var right := Rect2(Vector2(size.x - 410 * s, 28 * s), Vector2(300, 170) * s)
	_rounded(right, CARD, NONE, 26 * s, 16 * s)
	_draw_ring(right.position + Vector2(82, 76) * s, "comfort", _run.comfort / PlannerCourse.COMFORT_MAX, Color(1.0, 0.55, 0.7))
	_draw_ring(right.position + Vector2(218, 76) * s, "safety", _run.safety / PlannerCourse.SAFETY_MAX, Color(0.35, 0.75, 1.0))

func _draw_ring(c: Vector2, label: String, ratio: float, color: Color) -> void:
	var s := _s
	var r := 42 * s
	if ratio < 0.4:
		color = BAD
	draw_arc(c, r, 0, TAU, 64, Color(color, 0.18), 12 * s, true)
	if ratio > 0.0:
		draw_arc(c, r, -PI * 0.5, -PI * 0.5 + TAU * clampf(ratio, 0, 1), 64, color, 12 * s, true)
	_text(FONT_BOLD, "%d" % roundi(ratio * 100.0), c + Vector2(0, 9 * s), 26, INK, HORIZONTAL_ALIGNMENT_CENTER)
	_text(FONT_BOLD, label, c + Vector2(0, r + 30 * s), 13, INK_SOFT, HORIZONTAL_ALIGNMENT_CENTER)

# The pedal: where the drag started, how far, and the command
func _draw_pedal() -> void:
	var s := _s
	if not _pedaling:
		_chip(Vector2(size.x * 0.5, size.y - 48 * s), "drag  ↑ go   ↓ brake   ·   let go to coast", INK_SOFT, false, 16)
		return
	var travel := size.y * PEDAL_TRAVEL
	var c := _pedal_anchor
	_rounded(Rect2(c - Vector2(18 * s, travel + 18 * s), Vector2(36 * s, travel * 2.0 + 36 * s)), CARD, NONE, 18 * s, 10 * s)
	var command := _command()
	# Up for accelerating, down for braking, as far as the drag
	var ratio := command / PlannerCourse.MAX_ACCEL if command > 0 else command / -PlannerCourse.MAX_DECEL
	var end := c + Vector2(0, -travel * ratio)
	var color := GOOD if command > 0 else (WARN if command > -2.0 else BAD)
	draw_line(c, end, color, 12 * s, true)
	draw_circle(c, 7 * s, Color(INK, 0.3), true, -1.0, true)
	draw_circle(end, 16 * s, color, true, -1.0, true)
	draw_circle(end, 7 * s, Color.WHITE, true, -1.0, true)
	_chip(end + Vector2(90 * s, 0), "%+.1f m/s²" % command, color, true, 15)

func _draw_messages() -> void:
	var y := size.y * 0.3
	for m in _messages:
		var alpha := clampf(m[2] / 0.4, 0.0, 1.0)
		_chip(Vector2(size.x * 0.5, y), m[0], Color(m[1], alpha), true, 22)
		y += 56 * _s

# --- Overlays ---

func _card(height: float) -> Rect2:
	var s := _s
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.2, 0.25, 0.45, 0.35))
	var card := Rect2(Vector2(size.x * 0.5 - 340 * s, size.y * 0.5 - height * 0.5 * s), Vector2(680, height) * s)
	_rounded(card, Color.WHITE, NONE, 34 * s, 36 * s)
	return card

func _button(name: String, rect: Rect2, label: String, primary: bool) -> void:
	var s := _s
	var fill := ACCENT if primary else Color(INK, 0.06)
	_rounded(Rect2(rect.position + Vector2(0, 4 * s), rect.size), fill.darkened(0.2) if primary else Color(INK, 0.1), NONE, rect.size.y * 0.5)
	_rounded(rect, fill, NONE, rect.size.y * 0.5)
	_text(FONT_BOLD, label, rect.get_center() + Vector2(0, 7 * s), 18, Color.WHITE if primary else INK, HORIZONTAL_ALIGNMENT_CENTER)
	_buttons[name] = rect

func _draw_title() -> void:
	var s := _s
	var card := _card(610)
	var x := card.position.x + 50 * s
	var y := card.position.y
	_chip(Vector2(x + 52 * s, y + 52 * s), "MINI GAME", Color(1.0, 0.55, 0.7), true, 13)
	_text(FONT_BOLD, "Planner Challenge", Vector2(x, y + 116 * s), 46, INK)
	_text(FONT, "You are the planner! Take everyone to the goal, smooth and safe.", Vector2(x, y + 152 * s), 17, INK_SOFT)
	var rules := [
		[ACCENT, "drag up to go, down to brake, let go to coast"],
		[BAD, "stop for red lights (the ring is the time left, V2I)"],
		[WARN, "let people cross first"],
		[Color(0.4, 0.82, 0.75), "keep your distance, or AEB brakes for you"],
		[Color(0.72, 0.6, 1.0), "stop right on the goal line"],
	]
	var ry := y + 200 * s
	for r in rules:
		draw_circle(Vector2(x + 12 * s, ry - 6 * s), 11 * s, r[0], true, -1.0, true)
		draw_circle(Vector2(x + 12 * s, ry - 6 * s), 4 * s, Color.WHITE, true, -1.0, true)
		_text(FONT, r[1], Vector2(x + 40 * s, ry), 17, INK)
		ry += 38 * s
	# The course: daily or random, the best and the rival's score
	var by := y + 404 * s
	var seg := Rect2(Vector2(x, by), Vector2(250, 46) * s)
	_rounded(seg, Color(INK, 0.06), NONE, 23 * s)
	var half := Rect2(seg.position, Vector2(seg.size.x * 0.5, seg.size.y))
	var other := Rect2(seg.position + Vector2(seg.size.x * 0.5, 0), half.size)
	_rounded((half if _daily else other).grow(-4 * s), Color.WHITE, NONE, 19 * s, 6 * s)
	_text(FONT_BOLD, "Daily", half.get_center() + Vector2(0, 6 * s), 16, INK if _daily else INK_SOFT, HORIZONTAL_ALIGNMENT_CENTER)
	_text(FONT_BOLD, "Random", other.get_center() + Vector2(0, 6 * s), 16, INK_SOFT if _daily else INK, HORIZONTAL_ALIGNMENT_CENTER)
	_buttons["daily"] = half
	_buttons["random"] = other
	var best := float(_best.get_value("best", _course_key(), 0.0))
	var ghost_points := _course.reference.total()
	_text(FONT, _course.name, Vector2(x + 274 * s, by + 18 * s), 14, INK_SOFT)
	_text(FONT_BOLD, "best  %s" % ("%d · %s" % [best, PlannerCourse.Run.rank(best)] if best > 0 else "-"),
		Vector2(x + 274 * s, by + 40 * s), 16, INK)
	_text(FONT_BOLD, "rival  %d · %s" % [ghost_points, PlannerCourse.Run.rank(ghost_points)],
		Vector2(x + 410 * s, by + 40 * s), 16, INK_SOFT)
	_button("start", Rect2(Vector2(x, card.end.y - 104 * s), Vector2(580, 60) * s), "Let's go!", true)

func _draw_countdown() -> void:
	var n := ceili(_countdown / 0.8)
	var label := str(n) if n > 0 else "GO!"
	var c := Vector2(size.x * 0.5, size.y * 0.36)
	var phase := fposmod(_countdown, 0.8) / 0.8
	var r := (84 + 10 * phase) * _s
	draw_circle(c + Vector2(0, 8 * _s), r, SHADOW, true, -1.0, true)
	draw_circle(c, r, Color.WHITE, true, -1.0, true)
	draw_arc(c, r - 8 * _s, -PI * 0.5, -PI * 0.5 + TAU * phase, 64, ACCENT, 10 * _s, true)
	_text(FONT_BOLD, label, c + Vector2(0, 28 * _s), 80, INK, HORIZONTAL_ALIGNMENT_CENTER)

func _draw_result() -> void:
	var s := _s
	var card := _card(630)
	var x := card.position.x + 50 * s
	var y := card.position.y
	var points := _run.total()
	var failed := _run.failed != ""
	var ghost := _course.reference
	if failed:
		_chip(Vector2(x + 44 * s, y + 52 * s), "OOPS", BAD, true, 13)
		_text(FONT_BOLD, "Failed", Vector2(x, y + 116 * s), 46, INK)
		_text(FONT, _run.failed, Vector2(x, y + 152 * s), 18, BAD)
	else:
		_chip(Vector2(x + 52 * s, y + 52 * s), "CLEAR!", GOOD, true, 13)
		_text(FONT_BOLD, "Nice drive!", Vector2(x, y + 116 * s), 46, INK)
		_text(FONT, "%.1f s   ·   rival %.1f s" % [_run.t, ghost.t], Vector2(x, y + 152 * s), 17, INK_SOFT)
		var rank := PlannerCourse.Run.rank(points)
		var rank_color: Color = {"S": Color(1.0, 0.72, 0.15), "A": Color(1.0, 0.5, 0.65), "B": ACCENT, "C": GOOD, "D": INK_SOFT}[rank]
		var c := Vector2(card.end.x - 104 * s, y + 104 * s)
		draw_circle(c + Vector2(0, 6 * s), 64 * s, Color(rank_color.darkened(0.25)), true, -1.0, true)
		draw_circle(c, 64 * s, rank_color, true, -1.0, true)
		draw_circle(c - Vector2(20, 22) * s, 14 * s, Color(1, 1, 1, 0.35), true, -1.0, true)
		_text(FONT_BOLD, rank, c + Vector2(0, 26 * s), 74, Color.WHITE, HORIZONTAL_ALIGNMENT_CENTER)
	var stop := "%+.2f m" % _run.stop_error
	if not failed and absf(_run.stop_error) <= PlannerCourse.PERFECT_STOP:
		stop = "PERFECT!  " + stop
	var rows := [
		["Time", "", _run.efficiency(), PlannerCourse.EFFICIENCY_MAX, ACCENT],
		["Comfort", "", _run.comfort if not failed else 0.0, PlannerCourse.COMFORT_MAX, Color(1.0, 0.55, 0.7)],
		["Safety", "AEB %d" % _run.aeb_count, _run.safety if not failed else 0.0, PlannerCourse.SAFETY_MAX, Color(0.35, 0.75, 1.0)],
		["Stop", stop if not failed else "", _run.stop_points(), PlannerCourse.STOP_MAX, Color(0.72, 0.6, 1.0)],
	]
	var ry := y + 206 * s
	var bar_x := x + 120 * s
	var bar_w := card.size.x - 100 * s - 120 * s - 80 * s
	for r in rows:
		_text(FONT_BOLD, r[0], Vector2(x, ry + 6 * s), 17, INK)
		_text(FONT_BOLD, r[1], Vector2(bar_x, ry - 12 * s), 12, INK_SOFT)
		_rounded(Rect2(bar_x, ry - 5 * s, bar_w, 14 * s), Color(INK, 0.07), NONE, 7 * s)
		var ratio := clampf(float(r[2]) / float(r[3]), 0, 1)
		if ratio > 0.0:
			_rounded(Rect2(bar_x, ry - 5 * s, maxf(bar_w * ratio, 14 * s), 14 * s), r[4], NONE, 7 * s)
		_text(FONT_BOLD, "%d" % r[2], Vector2(card.end.x - 50 * s, ry + 7 * s), 19, INK, HORIZONTAL_ALIGNMENT_RIGHT)
		ry += 50 * s
	_text(FONT_BOLD, "total", Vector2(x, ry + 22 * s), 17, INK_SOFT)
	_text(FONT_BOLD, "%d" % points, Vector2(card.end.x - 50 * s, ry + 32 * s), 48, INK, HORIZONTAL_ALIGNMENT_RIGHT)
	if not failed:
		var ghost_points := ghost.total()
		var beat := points > ghost_points
		var verdict := "You beat the rival!" if beat else "Rival %d, so close!" % ghost_points
		var vw := _text_width(FONT_BOLD, verdict, 15) + 26 * s
		_chip(Vector2(x + vw * 0.5, ry + 72 * s), verdict, GOOD if beat else INK_SOFT, true, 15)
		if _new_best:
			_chip(Vector2(x + vw + 70 * s, ry + 72 * s), "NEW BEST", Color(1.0, 0.72, 0.15), true, 15)
	var by := card.end.y - 96 * s
	_button("retry", Rect2(Vector2(x, by), Vector2(190, 56) * s), "Retry", true)
	_button("new", Rect2(Vector2(x + 204 * s, by), Vector2(180, 56) * s), "New course", false)
	_button("title", Rect2(Vector2(x + 398 * s, by), Vector2(182, 56) * s), "Title", false)

func _draw_close() -> void:
	var s := _s
	var c := Vector2(size.x - 58 * s, 58 * s)
	var r := 26 * s
	draw_circle(c + Vector2(0, 4 * s), r, SHADOW, true, -1.0, true)
	draw_circle(c, r, Color.WHITE, true, -1.0, true)
	var k := 7 * s
	draw_line(c + Vector2(-k, -k), c + Vector2(k, k), INK, 3.0 * s, true)
	draw_line(c + Vector2(-k, k), c + Vector2(k, -k), INK, 3.0 * s, true)
	_buttons["close"] = Rect2(c - Vector2(r, r) * 1.4, Vector2(r, r) * 2.8)

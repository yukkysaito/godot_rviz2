# ParentControl.gd
extends Control

var operation_mode_state := OperationModeState.new()
var navidation_state := NavigationState.new()

@export var start_button_scene: PackedScene = preload("res://UI/StartButton.tscn")
@export var message_panel_scene: PackedScene = preload("res://UI/MessagePanel.tscn")

# --- Internal UI caches ---
var _start_root: Control = null         # Start button root (e.g., CenterContainer)
var _msg_root: Control = null   # Message panel root

# --- Route / Op enums for clarity ---
enum RouteState { NO_ROUTE, HAS_ROUTE, ARRIVED, UNKNOWN }
enum UiState { NEED_DEST, ARRIVED_MSG, READY_TO_START, HIDE_ALL }

# --- Cached last UI state to avoid redundant work ---
var _last_ui_state: int = -1

func _ready() -> void:
	operation_mode_state.subscribe("/api/operation_mode/state", true)
	navidation_state.subscribe("/api/routing/state", true)

	# First paint even if has_new() is false on first frame
	_update_and_apply_ui()
	
func _process(_delta: float) -> void:

	var changed := false

	if navidation_state.has_new():
		navidation_state.set_old()
		changed = true

	if operation_mode_state.has_new():
		operation_mode_state.set_old()
		changed = true

	if changed:
		_update_and_apply_ui()

# ---------------- Core loop: read -> decide -> apply ----------------
func _update_and_apply_ui() -> void:
	var route := _read_route_state()
	var ui := _compute_ui_state(route)
	_apply_ui_state(ui)

# Read current navigation into enum
func _read_route_state() -> int:
	# Order matters: more specific first
	if navidation_state.has_no_route():
		#print("- NO_ROUTE")
		return RouteState.NO_ROUTE
	if navidation_state.is_arrived():
		#print("- ARRIVED")
		return RouteState.ARRIVED
	if navidation_state.has_route():
		#print("- HAS_ROUTE")
		return RouteState.HAS_ROUTE
	#print("- UNKNOWN")
	return RouteState.UNKNOWN

# Decide what the UI should look like *only* from state
func _compute_ui_state(route: int) -> int:
	# Derive op flags once
	var can_auto := operation_mode_state.is_autonomous_mode_available()
	var is_auto  := operation_mode_state.is_autonomous_mode()

	match route:
		RouteState.NO_ROUTE:
			return UiState.NEED_DEST
		RouteState.ARRIVED:
			return UiState.ARRIVED_MSG
		RouteState.HAS_ROUTE:
			# Show start only when available, not already running
			if can_auto and not is_auto:
				#print("  - READY_TO_START")
				return UiState.READY_TO_START
			else:
				#print("  - HIDE_ALL")
				return UiState.HIDE_ALL
		_:
			return UiState.HIDE_ALL

# Mutate UI in one place; ignore if nothing changes
func _apply_ui_state(ui: int) -> void:
	if ui == _last_ui_state:
		return
	_last_ui_state = ui

	match ui:
		UiState.NEED_DEST:
			_hide_start_button()
			_show_message("Hi there! Please set your destination.")
		UiState.ARRIVED_MSG:
			_hide_start_button()
			_show_message("We’ve arrived! Hope you enjoyed the ride.")
		UiState.READY_TO_START:
			_hide_message()
			_show_start_button()
		UiState.HIDE_ALL:
			_hide_message()
			_hide_start_button()

# ===============================================================
# Message Panel Handling
# ===============================================================
func _show_message(text: String) -> bool:
	# Already shown? → just update the message
	if _msg_root != null and is_instance_valid(_msg_root):
		_msg_root.show_message(text)
		return false

	# Instantiate MessagePanel.tscn
	var inst := message_panel_scene.instantiate()
	if inst is Control:
		var c := inst as Control
		c.anchor_left = 0.0
		c.anchor_top = 0.0
		c.anchor_right = 1.0
		c.anchor_bottom = 0.3
		c.offset_left = 0
		c.offset_top = 0
		c.offset_right = 0
		c.offset_bottom = 0

	add_child(inst)
	_msg_root = inst

	# Show message with fade-in animation
	_msg_root.show_message(text)
	return true

func _hide_message() -> void:
	if _msg_root == null or not is_instance_valid(_msg_root):
		return
	# MessagePanel.gd handles fade-out and free by itself
	await _msg_root.hide_message()
	_msg_root = null

# ===============================================================
# Start Button Handling (Reconciliation-style)
# ===============================================================
var _start_tween: Tween = null
var _is_start_visible_target := false  # The desired final visibility state (true = should be visible)

func _show_start_button() -> void:
	# Request to show the start button
	_is_start_visible_target = true
	_reconcile_start_button()

func _hide_start_button() -> void:
	# Request to hide the start button
	_is_start_visible_target = false
	_reconcile_start_button()

func _reconcile_start_button() -> void:
	# 1. Kill any existing tween to prevent animation conflicts
	if _start_tween and _start_tween.is_running():
		_start_tween.kill()

	# 2. If we need to show but no node exists → create one
	if _is_start_visible_target and (_start_root == null or not is_instance_valid(_start_root)):
		var inst := start_button_scene.instantiate()
		if inst is Control:
			var c := inst as Control
			c.anchor_left = 0.0
			c.anchor_top = 0.0
			c.anchor_right = 1.0
			c.anchor_bottom = 1.0
			c.offset_left = 0
			c.offset_top = 0
			c.offset_right = 0
			c.offset_bottom = 0
		add_child(inst)
		_start_root = inst
		_start_root.modulate.a = 0.0  # Start fully transparent

	# 3. If we need to hide but node doesn't exist → do nothing
	if not _is_start_visible_target and (_start_root == null or not is_instance_valid(_start_root)):
		return

	# 4. Show mode → fade in
	if _is_start_visible_target:
		_start_root.visible = true
		_start_tween = create_tween().set_ease(Tween.EASE_OUT).set_trans(Tween.TRANS_SINE)
		_start_tween.tween_property(_start_root, "modulate:a", 1.0, 0.5)
	else:
		# Hide mode → fade out and remove when done
		_start_tween = create_tween().set_ease(Tween.EASE_IN).set_trans(Tween.TRANS_SINE)
		_start_tween.tween_property(_start_root, "modulate:a", 0.0, 0.5)
		_start_tween.finished.connect(func ():
			# If another show request came during fade-out → re-show instead of deleting
			if _is_start_visible_target:
				_reconcile_start_button()
			elif is_instance_valid(_start_root):
				_start_root.queue_free()
				_start_root = null
		)

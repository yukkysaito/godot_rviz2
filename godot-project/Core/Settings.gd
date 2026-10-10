extends Node

# Application settings (autoload).
#
# Values are layered: DEFAULTS < project settings (res://Config/project.cfg, optional: lets a
# derived project, e.g. one with its own vehicles, change the defaults by adding this file
# instead of editing this script) < preset (res://Config/Presets/<name>.cfg, chosen with
# "-- --preset=<name>", e.g. for a real vehicle or a Jetson) < the user's settings
# (user://settings.cfg, saved whenever a value changes from the UI) < command line overrides
# (not saved): "-- --vehicle=<Name>" selects res://3DViewer/Vehicle/<Name>/<Name>.tres.
#
# The UI only changes values here (see UI/SettingToggle.gd, UI/SettingOption.gd) and the 3D view
# and HUD follow them with bind(), so they do not need to know each other.
# The display settings (window mode, anti-aliasing) are applied here.

const DEFAULTS := {
	"display/window_mode": "windowed",  # windowed, maximized, borderless_maximized, fullscreen
	"display/always_on_top": false,
	"display/anti_aliasing": "msaa_8x",  # off, fxaa, taa, msaa_2x, msaa_4x, msaa_8x
	"view/day_mode": "day",  # day, night
	"view/object_mode": "model",  # model, geometry
	"view/object_icons": true,
	"view/predicted_paths": false,
	"view/ignore_unknown_objects": false,
	"view/high_contrast": false,
	"view/camera_auto_return": true,
	"view/adaptive_camera_work": true,
	"data/pointcloud_map": true,
	"data/obstacle_segmentation": true,
	"hud/top_status_bar": true,
	"hud/operation_control": true,
	"hud/vehicle_actuator_status": true,
	"vehicle/profile": "res://3DViewer/Vehicle/RX450h/RX450h.tres",
}

const PROJECT_FILE := "res://Config/project.cfg"
const USER_FILE := "user://settings.cfg"
const PRESET_DIR := "res://Config/Presets/"

signal changed(key: String, value: Variant)

var _values := {}
var _user := ConfigFile.new()  # only the values the user changed
var _listeners := {}  # key -> Array[Callable]
var _save_pending := false

func _enter_tree() -> void:
	_values = DEFAULTS.duplicate()
	if FileAccess.file_exists(PROJECT_FILE):
		_merge(PROJECT_FILE)
	var preset := _cmdline_value("--preset=")
	if not preset.is_empty():
		_merge(PRESET_DIR + preset + ".cfg")
	if _user.load(USER_FILE) == OK:
		_merge_config(_user)
	var vehicle := _cmdline_value("--vehicle=")
	if not vehicle.is_empty():
		_values["vehicle/profile"] = "res://3DViewer/Vehicle/%s/%s.tres" % [vehicle, vehicle]
	_apply_display()

func get_value(key: String) -> Variant:
	return _values.get(key)

# Changes a value (and remembers it as the user's choice)
func set_value(key: String, value: Variant) -> void:
	if not DEFAULTS.has(key):
		push_warning("Unknown setting: %s" % key)
		return
	if _values.get(key) == value:
		return
	_values[key] = value
	var parts := key.split("/", true, 1)
	_user.set_value(parts[0], parts[1], value)
	_save_soon()
	_notify(key, value)

# Calls callback(value) now and whenever the value changes. A callback whose object was freed
# is dropped.
func bind(key: String, callback: Callable) -> void:
	if not _listeners.has(key):
		_listeners[key] = []
	_listeners[key].append(callback)
	callback.call(get_value(key))

func _notify(key: String, value: Variant) -> void:
	var callbacks: Array = _listeners.get(key, [])
	for callback in callbacks.duplicate():
		if callback.is_valid():
			callback.call(value)
		else:
			callbacks.erase(callback)
	if key.begins_with("display/"):
		_apply_display()
	changed.emit(key, value)

func _cmdline_value(prefix: String) -> String:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with(prefix):
			return arg.trim_prefix(prefix)
	return ""

func _merge(path: String) -> void:
	var config := ConfigFile.new()
	var err := config.load(path)
	if err != OK:
		push_warning("Cannot load settings %s (%s)" % [path, error_string(err)])
		return
	_merge_config(config)

func _merge_config(config: ConfigFile) -> void:
	for section in config.get_sections():
		for name in config.get_section_keys(section):
			var key := "%s/%s" % [section, name]
			if DEFAULTS.has(key):
				_values[key] = config.get_value(section, name)

func _save_soon() -> void:
	if _save_pending:
		return
	_save_pending = true
	_save.call_deferred()

func _save() -> void:
	_save_pending = false
	var err := _user.save(USER_FILE)
	if err != OK:
		push_warning("Cannot save settings (%s)" % error_string(err))

# --- Display ---

func _apply_display() -> void:
	match str(get_value("display/window_mode")):
		"maximized":
			_set_window(DisplayServer.WINDOW_MODE_MAXIMIZED, false)
		"borderless_maximized":
			_set_window(DisplayServer.WINDOW_MODE_MAXIMIZED, true)
		"fullscreen":
			_set_window(DisplayServer.WINDOW_MODE_FULLSCREEN, false)
		_:
			_set_window(DisplayServer.WINDOW_MODE_WINDOWED, false)
	DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_ALWAYS_ON_TOP, bool(get_value("display/always_on_top")))

	var viewport := get_viewport()
	viewport.msaa_3d = Viewport.MSAA_DISABLED
	viewport.use_taa = false
	viewport.screen_space_aa = Viewport.SCREEN_SPACE_AA_DISABLED
	match str(get_value("display/anti_aliasing")):
		"fxaa":
			viewport.screen_space_aa = Viewport.SCREEN_SPACE_AA_FXAA
		"taa":
			viewport.use_taa = true
		"msaa_2x":
			viewport.msaa_3d = Viewport.MSAA_2X
		"msaa_4x":
			viewport.msaa_3d = Viewport.MSAA_4X
		"msaa_8x":
			viewport.msaa_3d = Viewport.MSAA_8X

func _set_window(mode: DisplayServer.WindowMode, borderless: bool) -> void:
	if DisplayServer.get_name() == "headless":
		return
	DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, borderless)
	if DisplayServer.window_get_mode() != mode:
		DisplayServer.window_set_mode(mode)

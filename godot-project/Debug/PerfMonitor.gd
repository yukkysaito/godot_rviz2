extends Node

# Performance / startup instrumentation, enabled with the user argument `--perf`
# (e.g. `godot --path godot-project -- --perf --perf-duration=60`).
#
# - mark(name): logs the time since engine start of a startup milestone (first time only)
# - measure_begin(name) / measure_end(name): logs the duration of a block
# - every `report_interval` seconds: frame time statistics and render counters
# - `--perf-duration=N`: quits after N seconds and prints a summary
# - `--perf-screenshot=T1,T2,...`: saves screenshots at these times [s] to user://perf/

var enabled := false
var report_interval := 5.0

var _marks := {}
var _measure_start := {}
var _frame_ms: PackedFloat32Array = []
var _all_frame_ms: PackedFloat32Array = []
var _since_report := 0.0
var _duration := 0.0
var _screenshot_times: Array[float] = []

func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	for arg in OS.get_cmdline_user_args():
		if arg == "--perf":
			enabled = true
		elif arg.begins_with("--perf-duration="):
			enabled = true
			_duration = arg.get_slice("=", 1).to_float()
		elif arg.begins_with("--perf-screenshot="):
			enabled = true
			for t in arg.get_slice("=", 1).split(","):
				_screenshot_times.append(t.to_float())
	set_process(enabled)
	if enabled:
		_log("enabled (renderer: %s, %s)" % [
			RenderingServer.get_video_adapter_name(), ProjectSettings.get_setting("rendering/renderer/rendering_method")])

func mark(event_name: String) -> void:
	if not enabled or _marks.has(event_name):
		return
	_marks[event_name] = Time.get_ticks_msec()
	_log("mark %-28s t=%7d ms" % [event_name, _marks[event_name]])

func measure_begin(block_name: String) -> void:
	if enabled:
		_measure_start[block_name] = Time.get_ticks_usec()

func measure_end(block_name: String) -> void:
	if not enabled or not _measure_start.has(block_name):
		return
	var ms: float = (Time.get_ticks_usec() - _measure_start[block_name]) / 1000.0
	_measure_start.erase(block_name)
	_log("time %-28s %9.1f ms" % [block_name, ms])

func _process(delta: float) -> void:
	var ms := delta * 1000.0
	_frame_ms.append(ms)
	_all_frame_ms.append(ms)
	_since_report += delta
	if _since_report >= report_interval:
		_report("frames", _frame_ms)
		_frame_ms.clear()
		_since_report = 0.0
	var now := Time.get_ticks_msec() / 1000.0
	if not _screenshot_times.is_empty() and now >= _screenshot_times[0]:
		_save_screenshot(_screenshot_times.pop_front())
	if _duration > 0.0 and now >= _duration:
		_report("summary", _all_frame_ms)
		_log("summary marks %s" % JSON.stringify(_marks))
		get_tree().quit()

func _report(label: String, samples: PackedFloat32Array) -> void:
	if samples.is_empty():
		return
	var sorted := samples.duplicate()
	sorted.sort()
	var total := 0.0
	for v in samples:
		total += v
	_log("%s n=%d avg=%.2f p95=%.2f max=%.2f ms | process=%.1f physics=%.1f ms | draw_calls=%d objects=%d primitives=%d vram=%.0fMB" % [
		label, samples.size(), total / samples.size(), sorted[int(sorted.size() * 0.95)], sorted[-1],
		Performance.get_monitor(Performance.TIME_PROCESS) * 1000.0,
		Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS) * 1000.0,
		Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME),
		Performance.get_monitor(Performance.RENDER_TOTAL_OBJECTS_IN_FRAME),
		Performance.get_monitor(Performance.RENDER_TOTAL_PRIMITIVES_IN_FRAME),
		Performance.get_monitor(Performance.RENDER_VIDEO_MEM_USED) / 1048576.0])

func _save_screenshot(at: float) -> void:
	DirAccess.make_dir_recursive_absolute("user://perf")
	var path := "user://perf/screenshot_%05.1f.png" % at
	get_viewport().get_texture().get_image().save_png(path)
	_log("screenshot %s" % ProjectSettings.globalize_path(path))

func _log(text: String) -> void:
	print("[perf] ", text)

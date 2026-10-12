extends Control
class_name Orb

# Orb, the voice copilot (a first sample): it listens to the microphone, recognizes Japanese with
# whisper.cpp (SpeechRecognizer, on a worker thread) and, once woken up by its name ("オーブ"),
# presses the start button when told to start ("スタート", "発進", "出発") and stops the vehicle
# when told to stop ("ストップ", "止まって", "停止"), as the STOP button of Autoware's RViz panel
# does. It shows as a small sphere of smoke in the bottom right corner, with what it heard and
# what it does as a caption.
#
# Utterances are cut out of the microphone input by their loudness (the level adapts to the noise),
# so recognition only runs while someone speaks.

@export var operation_control_path: NodePath  # to press the start button
# The whisper model: $GODOT_RVIZ2_WHISPER_MODEL, or the first of these that exists (relative to the
# project): kotoba-whisper (trained for Japanese) is much more accurate than the multilingual small
@export var model_paths: PackedStringArray = [
	"../models/ggml-kotoba-whisper-v2.0-q5_0.bin", "../models/ggml-small-q5_1.bin"]
# Recognitions are dropped when whisper thinks it was no speech, or is unsure of the words
@export var max_no_speech: float = 0.5
@export var min_confidence: float = 0.4
# Silero VAD (relative to the project): whisper alone "hears" words in noise, which woke Orb up
# and made false commands; with it, sound without speech gives no text
@export var vad_model_path: String = "../models/ggml-silero-v6.2.0.bin"

# The wake word must start what was said ("オーブ、スタート"), so that it is not found in the
# middle of other talk
const WAKE_WORDS := ["オーブ", "おーぶ", "オウブ"]
const WAKE_WITHIN := 2  # characters before it (e.g. "あ、")
# Commands, matched anywhere in what was said (so "スタートして", "出発して", "発進して" match too)
const START_WORDS := ["スタート", "すたーと", "発進", "出発", "しゅっぱつ", "はっしん"]
const STOP_WORDS := ["ストップ", "すとっぷ", "止まって", "とまって", "止めて", "とめて", "停止", "ていし"]

const AWAKE_TIME := 8.0  # [s] after the wake word, a command is taken without it
const SAMPLE_RATE := 16000
const FRAME := 320  # 20 ms at 16 kHz
const PRE_ROLL := 15  # frames kept before the speech starts
const END_SILENCE := 35  # frames of silence that end an utterance (0.7 s)
const MAX_FRAMES := 400  # 8 s
const MIN_FRAMES := 15  # shorter sounds (0.3 s) are ignored

enum State { OFF, IDLE, AWAKE, THINKING }

var _state := State.OFF
var _recognizer: SpeechRecognizer
var _capture: AudioEffectCapture
var _player: AudioStreamPlayer
var _awake_until := 0.0
# Stops the vehicle as the STOP button of Autoware's RViz panel does (the stop operation mode)
var _stopper: OperationModeChanger = RosBridge.create_operation_mode_changer("change_to_stop")
var _caption := ""
var _caption_until := 0.0

# Voice activity
var _resampled := PackedFloat32Array()  # 16 kHz input not yet cut into frames
var _resample_phase := 0.0
var _noise := 0.01  # running noise level (RMS)
var _frames: Array[PackedFloat32Array] = []  # the utterance, or the pre-roll before one
var _speaking := false
var _silence := 0
var _level := 0.0
var _heard_awake := false  # the utterance being recognized began while awake

var _sphere := ColorRect.new()  # drawn by the orb shader
var _material := ShaderMaterial.new()
var _spin := 0.0
var _activity := 0.0
var _labels: FontVariation

@onready var _operation_control: Node = get_node_or_null(operation_control_path)

func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_material.shader = preload("res://UI/Shaders/orb_mini.gdshader")
	_sphere.material = _material
	_sphere.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_sphere)
	_labels = FontVariation.new()
	_labels.base_font = preload("res://UI/Fonts/Barlow-Medium.ttf")
	Settings.bind("hud/orb", func(on): visible = on)
	if not SpeechRecognizer.is_available():
		push_warning("Orb: built without whisper.cpp (scripts/build_whisper.sh); voice is off")
		return
	_recognizer = SpeechRecognizer.new()
	var path := OS.get_environment("GODOT_RVIZ2_WHISPER_MODEL")
	for candidate in model_paths:
		if path.is_empty() or not FileAccess.file_exists(path):
			path = ProjectSettings.globalize_path("res://").path_join(candidate).simplify_path()
	var vad_path := ProjectSettings.globalize_path("res://").path_join(vad_model_path).simplify_path()
	if not FileAccess.file_exists(vad_path):
		push_warning("Orb: no VAD model %s; noise may be taken for words" % vad_path)
		vad_path = ""
	if not _recognizer.load_model(path, vad_path):
		push_warning("Orb: cannot load the speech model %s; voice is off" % path)
		return
	_start_microphone()
	_state = State.IDLE

func _start_microphone() -> void:
	var bus := AudioServer.bus_count
	AudioServer.add_bus(bus)
	AudioServer.set_bus_name(bus, "OrbMicrophone")
	AudioServer.set_bus_volume_db(bus, -80.0)  # not played back (the capture is before the fader)
	_capture = AudioEffectCapture.new()
	_capture.buffer_length = 0.5
	AudioServer.add_bus_effect(bus, _capture)
	_player = AudioStreamPlayer.new()
	_player.stream = AudioStreamMicrophone.new()
	_player.bus = "OrbMicrophone"
	add_child(_player)
	_player.play()

func _process(delta: float) -> void:
	if _state != State.OFF:
		_read_microphone()
		if _state == State.THINKING and _recognizer.is_done():
			_on_result(_recognizer.take_result())
		if _state == State.AWAKE and Time.get_ticks_msec() / 1000.0 > _awake_until:
			_state = State.IDLE
	var target := 1.0 if _state in [State.AWAKE, State.THINKING] else 0.0
	_activity = move_toward(_activity, target, delta * 3.0)
	_spin += delta * (4.0 if _state == State.THINKING else 0.3)
	_material.set_shader_parameter("activity", _activity)
	_material.set_shader_parameter("level", _level if _state == State.AWAKE else _level * 0.3)
	_material.set_shader_parameter("spin", _spin)
	queue_redraw()

# --- microphone and voice activity

func _read_microphone() -> void:
	var available := _capture.get_frames_available()
	if available == 0:
		return
	var stereo := _capture.get_buffer(available)
	var step := AudioServer.get_mix_rate() / SAMPLE_RATE
	# Down to 16 kHz mono, averaging the input samples of each output sample
	var sum := 0.0
	var count := 0
	for frame in stereo:
		sum += (frame.x + frame.y) * 0.5
		count += 1
		_resample_phase += 1.0
		if _resample_phase >= step:
			_resample_phase -= step
			_resampled.append(sum / count)
			sum = 0.0
			count = 0
	while _resampled.size() >= FRAME:
		_on_frame(_resampled.slice(0, FRAME))
		_resampled = _resampled.slice(FRAME)

func _on_frame(frame: PackedFloat32Array) -> void:
	var energy := 0.0
	for v in frame:
		energy += v * v
	var rms := sqrt(energy / frame.size())
	_level = clampf(rms / maxf(_noise * 8.0, 0.02), 0.0, 1.0)
	var loud := rms > maxf(_noise * 4.0, 0.015)
	if not _speaking:
		_noise = lerpf(_noise, rms, 0.02)  # the noise level follows the quiet input
		_frames.append(frame)
		if _frames.size() > PRE_ROLL:
			_frames.pop_front()
		if loud:
			_speaking = true
			_silence = 0
		return
	_frames.append(frame)
	_silence = 0 if loud else _silence + 1
	if _silence >= END_SILENCE or _frames.size() >= MAX_FRAMES:
		_speaking = false
		var utterance := PackedFloat32Array()
		for f in _frames:
			utterance.append_array(f)
		var spoken := _frames.size() - _silence
		_frames.clear()
		if spoken >= MIN_FRAMES and _state != State.THINKING and _recognizer.start(utterance, "ja"):
			_heard_awake = _state == State.AWAKE
			_state = State.THINKING

# --- what was said

func _on_result(result: Dictionary) -> void:
	var heard := str(result["text"]).strip_edges()
	print("Orb heard: \"%s\" (no speech %.2f, confidence %.2f)" % [heard, result["no_speech"], result["confidence"]])
	if heard.is_empty() or result["no_speech"] > max_no_speech or result["confidence"] < min_confidence:
		_state = State.AWAKE if _heard_awake else State.IDLE
		return
	var plain := _normalize(heard)
	var woke := _starts_with_any(plain, WAKE_WORDS, WAKE_WITHIN)
	if not (woke or _heard_awake):
		_state = State.IDLE  # not for Orb
		return
	# Stop first: it is always taken, and wins if both were heard
	if _contains_any(plain, STOP_WORDS):
		_say(heard, "停止します" if _stopper.send_request() else "停止できません")
		_state = State.IDLE
		_awake_until = 0.0
		return
	if _contains_any(plain, START_WORDS):
		var pressed: bool = _operation_control != null and _operation_control.call("press_start")
		_say(heard, "発進します" if pressed else "今は発進できません")
		_state = State.IDLE
		_awake_until = 0.0
		return
	if woke:
		_say(heard, "はい、どうぞ")
		_state = State.AWAKE
		_awake_until = Time.get_ticks_msec() / 1000.0 + AWAKE_TIME
		return
	_say(heard, "もう一度お願いします")
	_state = State.AWAKE
	_awake_until = Time.get_ticks_msec() / 1000.0 + AWAKE_TIME

func _say(heard: String, reply: String) -> void:
	_caption = "「%s」\n%s" % [heard, reply] if heard != "" else reply
	_caption_until = Time.get_ticks_msec() / 1000.0 + 4.0

static func _normalize(text: String) -> String:
	var out := ""
	for c in text:
		if not c in " 　、。,.!?！？「」":
			out += c
	return out

static func _starts_with_any(text: String, words: Array, within: int) -> bool:
	for w in words:
		var at := text.find(w)
		if at >= 0 and at <= within:
			return true
	return false

static func _contains_any(text: String, words: Array) -> bool:
	for w in words:
		if text.contains(w):
			return true
	return false

# --- drawing

func _draw() -> void:
	var s := size.y / 1080.0
	var orb_size := 92.0 * s
	var rect := Rect2(size - Vector2(orb_size + 28 * s, orb_size + 28 * s), Vector2(orb_size, orb_size))
	_sphere.position = rect.position
	_sphere.size = rect.size
	_sphere.modulate.a = 1.0 if _state != State.OFF else 0.35
	var now := Time.get_ticks_msec() / 1000.0
	if _caption != "" and now < _caption_until:
		var alpha := clampf((_caption_until - now) / 0.5, 0.0, 1.0)
		var font_size := int(16 * s)
		var lines := _caption.split("\n")
		var y := rect.get_center().y - (lines.size() - 1) * 11 * s
		for line in lines:
			var width := _labels.get_string_size(line, HORIZONTAL_ALIGNMENT_LEFT, -1, font_size).x
			draw_string(_labels, Vector2(rect.position.x - 16 * s - width, y + 6 * s), line,
				HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color(0.93, 0.95, 1.0, alpha))
			y += 22 * s

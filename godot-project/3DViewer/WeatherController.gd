extends Node
class_name WeatherController

@export var status_bar_path: NodePath

@export_group("Behavior")
@export var enable_weather: bool = true
@export var geo_refresh_hours: float = 6.0
@export var weather_refresh_minutes: float = 60.0

@export_group("Icons")
@export var icon_clear: Texture2D
@export var icon_cloudy: Texture2D
@export var icon_fog: Texture2D
@export var icon_rain: Texture2D
@export var icon_snow: Texture2D
@export var icon_thunder: Texture2D
@export var icon_unknown: Texture2D

@onready var _status_bar: Node = get_node(status_bar_path)
@onready var _geo_req: HTTPRequest = $GeoRequest
@onready var _weather_req: HTTPRequest = $WeatherRequest

var _lat: float = 0.0
var _lon: float = 0.0

var _geo_timer: Timer
var _weather_timer: Timer

func _ready() -> void:
	if not enable_weather:
		_publish_weather(icon_unknown, "")
		return

	_request_geo_by_ip()

	_weather_timer = Timer.new()
	_weather_timer.wait_time = max(weather_refresh_minutes, 1.0) * 60.0
	_weather_timer.autostart = true
	_weather_timer.timeout.connect(_on_weather_tick)
	add_child(_weather_timer)

	if geo_refresh_hours > 0.0:
		_geo_timer = Timer.new()
		_geo_timer.wait_time = geo_refresh_hours * 3600.0
		_geo_timer.autostart = true
		_geo_timer.timeout.connect(_on_geo_tick)
		add_child(_geo_timer)

func _on_geo_tick() -> void:
	_request_geo_by_ip()

func _on_weather_tick() -> void:
	if _lat == 0.0 and _lon == 0.0:
		return
	_request_weather(_lat, _lon)

func _request_geo_by_ip() -> void:
	var url: String = "https://ipapi.co/json/"
	_geo_req.request_completed.connect(_on_geo_completed, CONNECT_ONE_SHOT)
	var err: int = _geo_req.request(url)
	if err != OK:
		push_warning("Geo request() failed: %s" % err)
		_publish_weather(icon_unknown, "")

func _on_geo_completed(_result: int, response_code: int, _headers: PackedStringArray, body: PackedByteArray) -> void:
	if response_code != 200:
		push_warning("Geo HTTP %d" % response_code)
		_publish_weather(icon_unknown, "")
		return

	var text: String = body.get_string_from_utf8()
	var parsed: Variant = JSON.parse_string(text)
	if typeof(parsed) != TYPE_DICTIONARY:
		push_warning("Geo JSON parse failed")
		_publish_weather(icon_unknown, "")
		return

	var data: Dictionary = parsed as Dictionary

	# Dictionary.get() returns Variant -> cast explicitly (warning-as-error friendly)
	var lat_v: Variant = data.get("latitude", 0.0)
	var lon_v: Variant = data.get("longitude", 0.0)
	var lat: float = float(lat_v)
	var lon: float = float(lon_v)

	if lat == 0.0 and lon == 0.0:
		push_warning("Geo lat/lon missing")
		_publish_weather(icon_unknown, "")
		return

	_lat = lat
	_lon = lon
	_request_weather(_lat, _lon)

func _request_weather(lat: float, lon: float) -> void:
	var url: String = "https://api.open-meteo.com/v1/forecast?latitude=%f&longitude=%f&current_weather=true&daily=weathercode&timezone=auto" % [lat, lon]
	_weather_req.request_completed.connect(_on_weather_completed, CONNECT_ONE_SHOT)
	var err: int = _weather_req.request(url)
	if err != OK:
		push_warning("Weather request() failed: %s" % err)
		_publish_weather(icon_unknown, "")

func _on_weather_completed(_result: int, response_code: int, _headers: PackedStringArray, body: PackedByteArray) -> void:
	if response_code != 200:
		push_warning("Weather HTTP %d" % response_code)
		_publish_weather(icon_unknown, "")
		return

	var text: String = body.get_string_from_utf8()
	var parsed: Variant = JSON.parse_string(text)
	if typeof(parsed) != TYPE_DICTIONARY:
		push_warning("Weather JSON parse failed")
		_publish_weather(icon_unknown, "")
		return

	var data: Dictionary = parsed as Dictionary

	# current_weather (Variant -> Dictionary)
	var cw_v: Variant = data.get("current_weather", {})
	var cw: Dictionary = cw_v as Dictionary

	# temperature (Variant)
	var temp_v: Variant = cw.get("temperature", null)
	var has_temp: bool = temp_v != null
	var temp_text: String = "%d°C" % int(round(float(temp_v))) if has_temp else ""

	# daily (Variant -> Dictionary)
	var daily_v: Variant = data.get("daily", {})
	var daily: Dictionary = daily_v as Dictionary

	# weathercode array (Variant -> Array)
	var wcode_v: Variant = daily.get("weathercode", [])
	var wcode_arr: Array = wcode_v as Array

	var wcode: int = -1
	if wcode_arr.size() > 0:
		wcode = int(wcode_arr[0])

	var icon: Texture2D = _icon_from_weathercode(wcode)
	_publish_weather(icon, temp_text)

func _publish_weather(icon: Texture2D, temp_text: String) -> void:
	if _status_bar == null:
		return

	if _status_bar.has_method("set_weather_icon"):
		_status_bar.call("set_weather_icon", icon)

	if _status_bar.has_method("set_temperature_text"):
		_status_bar.call("set_temperature_text", temp_text)

func _icon_from_weathercode(code: int) -> Texture2D:
	match code:
		0:
			return icon_clear
		1, 2, 3:
			return icon_cloudy
		45, 48:
			return icon_fog
		51, 53, 55, 61, 63, 65, 80, 81, 82:
			return icon_rain
		71, 73, 75, 85, 86:
			return icon_snow
		95, 96, 99:
			return icon_thunder
		_:
			return icon_unknown

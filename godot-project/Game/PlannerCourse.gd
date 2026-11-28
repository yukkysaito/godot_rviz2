extends RefCounted
class_name PlannerCourse

# The world of the mini game (Game/PlannerGame.gd), a straight road with what a velocity planner has
# to handle: traffic lights (with their remaining time, as from V2I), crosswalks with pedestrians,
# cars that cut in from a side road and brake, and speed limit zones, then the goal to stop at.
# A course made from a seed is the same in every run. The lights run on their own clocks; the
# pedestrians start crossing, and the cars come out of their side roads, when the ego gets within
# their trigger distance, always far enough to stop for them, and then move on their own schedules.
# So every run, the player's and the reference planner's (rule based, plan()), meets the same
# situations, at the times it brings them about. The reference run's time is the course's target
# time.
# A run (Run) is simulated in fixed steps: the ego follows an acceleration command with a short lag,
# and is scored for its efficiency (time against the target time), its comfort (acceleration and jerk),
# its safety (headway, emergency braking (AEB)) and how close to the goal line it stops. Running a
# red light or a collision fails the run.
# Nothing here, nor in the game, talks to ROS: it is a game, not a control of the vehicle.

const DT := 1.0 / 60.0  # [s] the simulation step
const EGO_LENGTH := 4.5  # [m]
const LANE_WIDTH := 3.5  # [m] the ego's lane is y in [0, LANE_WIDTH], the opposite one below 0
const EGO_Y := LANE_WIDTH * 0.5
const IN_LANE_Y := LANE_WIDTH + 0.8  # [m] a car or a pedestrian above this is clear of the ego's lane
const LAG := 0.25  # [s] of the acceleration following its command
const MAX_ACCEL := 2.0
const MAX_DECEL := -3.5
const COAST := -0.3  # [m/s^2] without a command (released)
const AEB_DECEL := -6.0
const AEB_NEEDED := 4.0  # [m/s^2] the deceleration a stop before the obstacle needs, that triggers AEB
const TIME_LIMIT := 180.0  # [s]
const DEFAULT_LIMIT := 50.0  # [km/h]

# Scores
const EFFICIENCY_MAX := 400.0
const EFFICIENCY_AT_TARGET := 350.0  # at the target time
const COMFORT_MAX := 250.0
const SAFETY_MAX := 250.0
const STOP_MAX := 100.0
const STOP_TOLERANCE := 2.5  # [m] off the line, no stop points
const PERFECT_STOP := 0.25  # [m]

class Light:
	var s: float  # [m] its stop line
	var green: float
	var yellow := 3.0
	var red: float
	var offset: float

	func cycle() -> float:
		return green + yellow + red

	# 0 green, 1 yellow, 2 red
	func state(t: float) -> int:
		var phase := fposmod(t + offset, cycle())
		if phase < green:
			return 0
		return 1 if phase < green + yellow else 2

	# [s] until the state changes
	func remaining(t: float) -> float:
		var phase := fposmod(t + offset, cycle())
		if phase < green:
			return green - phase
		if phase < green + yellow:
			return green + yellow - phase
		return cycle() - phase

class Pedestrian:
	var delay: float  # [s] after its crosswalk's trigger, when it steps on the road
	var start := INF  # [s] in a run, once triggered
	var speed: float  # [m/s]
	var from_y: float  # [m] its side of the road (it walks to the other)

	func y(t: float) -> float:
		var to_y := -from_y
		var d := absf(to_y - from_y)
		var k := clampf((t - start) * speed / d, 0.0, 1.0)
		return lerpf(from_y, to_y, k)

	func walking(t: float) -> bool:
		return t >= start and t <= start + absf(2.0 * from_y) / speed

	# Stepping into the ego's lane next (not yet in it)
	func about_to_enter(t: float) -> bool:
		if t < start:
			return false
		var py := y(t)
		return (py >= IN_LANE_Y and from_y > 0.0) or (py <= -0.8 and from_y < 0.0 and walking(t))

	# In the ego's lane (with a margin)
	func in_lane(t: float) -> bool:
		if not walking(t):
			return false
		var py := y(t)
		return py > -0.8 and py < IN_LANE_Y

class Crosswalk:
	var s: float  # [m] its near edge
	var width := 4.0
	var stop_line: float
	var trigger: float  # [m] the ego's front this far before it starts the pedestrians
	var pedestrians: Array[Pedestrian] = []

	func copy() -> Crosswalk:
		var other := Crosswalk.new()
		other.s = s
		other.width = width
		other.stop_line = stop_line
		other.trigger = trigger
		for p in pedestrians:
			var q := Pedestrian.new()
			q.delay = p.delay
			q.speed = p.speed
			q.from_y = p.from_y
			other.pedestrians.append(q)
		return other

	func triggered() -> bool:
		return pedestrians.is_empty() or pedestrians[0].start < INF

	func start(t: float) -> void:
		for p in pedestrians:
			p.start = t + p.delay

	# All crossed (or none yet triggered): false
	func pending(t: float) -> bool:
		if not triggered():
			return true
		for p in pedestrians:
			if t <= p.start + absf(2.0 * p.from_y) / p.speed:
				return true
		return false

	func occupied(t: float) -> bool:
		for p in pedestrians:
			if p.in_lane(t):
				return true
		return false

# A car entering the ego's lane from a side road, driving its own schedule, then turning off
class Car:
	var times := PackedFloat32Array()  # sampled every SAMPLE, from its start
	var positions := PackedFloat32Array()  # [m] along the road
	var speeds := PackedFloat32Array()
	var lateral := PackedFloat32Array()  # [m] y
	var trigger: float  # [m] the ego's front, past which it comes out
	var start := INF  # [s] in a run, once triggered
	const SAMPLE := 0.05

	func copy() -> Car:
		var other := Car.new()
		other.times = times
		other.positions = positions
		other.speeds = speeds
		other.lateral = lateral
		other.trigger = trigger
		return other

	func active(t: float) -> bool:
		return t >= start and t <= start + (times.size() - 1) * SAMPLE

	func _index(t: float) -> float:
		return clampf((t - start) / SAMPLE, 0.0, times.size() - 1.0)

	func _at(values: PackedFloat32Array, t: float) -> float:
		var i := _index(t)
		var i0 := int(floor(i))
		var i1 := mini(i0 + 1, values.size() - 1)
		return lerpf(values[i0], values[i1], i - i0)

	func s(t: float) -> float:
		return _at(positions, t)

	func v(t: float) -> float:
		return _at(speeds, t)

	func y(t: float) -> float:
		return _at(lateral, t)

	func accel(t: float) -> float:
		return (v(t + 0.1) - v(t - 0.1)) / 0.2

	func in_lane(t: float) -> bool:
		return active(t) and y(t) < IN_LANE_Y

class Zone:
	var s0: float
	var s1: float
	var limit: float  # [km/h]

class Oncoming:
	var start: float
	var s0: float
	var speed: float

	func s(t: float) -> float:
		return s0 - (t - start) * speed

var seed_value: int
var name: String
var length: float  # [m] to the goal line
var lights: Array[Light] = []
var crosswalks: Array[Crosswalk] = []
var cars: Array[Car] = []
var zones: Array[Zone] = []
var oncoming: Array[Oncoming] = []
var reference: Run  # the reference planner's run: its time is the target time, it is the ghost

static func make(seed: int, course_name: String) -> PlannerCourse:
	var course := PlannerCourse.new()
	course.seed_value = seed
	course.name = course_name
	course._generate()
	course.reference = course.plan()
	return course

func _generate() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = seed_value
	var s := 80.0
	var count := 4
	var last := ""
	for i in count:
		var kinds := ["light", "crosswalk", "car", "zone"]
		kinds.erase(last)  # not the same twice in a row
		var kind: String = kinds[rng.randi_range(0, kinds.size() - 1)]
		last = kind
		match kind:
			"light":
				var light := Light.new()
				light.s = s
				light.green = rng.randf_range(7.0, 13.0)
				light.red = rng.randf_range(6.0, 11.0)
				light.offset = rng.randf_range(0.0, light.cycle())
				lights.append(light)
			"crosswalk":
				var crosswalk := Crosswalk.new()
				crosswalk.s = s
				crosswalk.stop_line = s - 2.0
				crosswalk.trigger = s - rng.randf_range(45.0, 85.0)
				for j in rng.randi_range(1, 3):
					var p := Pedestrian.new()
					p.delay = j * rng.randf_range(0.5, 3.0)
					p.speed = rng.randf_range(1.0, 1.6)
					p.from_y = 6.0 if rng.randf() < 0.5 else -6.0
					crosswalk.pedestrians.append(p)
				crosswalks.append(crosswalk)
			"car":
				cars.append(_make_car(rng, s))
			"zone":
				var zone := Zone.new()
				zone.s0 = s
				zone.s1 = s + rng.randf_range(40.0, 70.0)
				zone.limit = 30.0 if rng.randf() < 0.6 else 20.0
				zones.append(zone)
				s = zone.s1
		s += rng.randf_range(70.0, 100.0)
	length = s + 20.0
	# Cars in the opposite lane, for life
	var t := 0.0
	while t < TIME_LIMIT:
		var o := Oncoming.new()
		o.start = t
		o.s0 = rng.randf_range(80.0, length + 60.0)
		o.speed = rng.randf_range(9.0, 14.0)
		oncoming.append(o)
		t += rng.randf_range(3.0, 9.0)

func _make_car(rng: RandomNumberGenerator, s: float) -> Car:
	var car := Car.new()
	car.trigger = s - rng.randf_range(55.0, 90.0)
	var cruise := rng.randf_range(6.0, 10.0)
	var brakes := []  # [time after start, low speed]
	for k in rng.randi_range(1, 2):
		brakes.append([rng.randf_range(4.0, 9.0) * (k + 1), rng.randf_range(0.0, 3.0)])
	var exit_s := s + rng.randf_range(90.0, 150.0)
	var pos := s
	var v := 4.0
	var y := 6.5
	var tt := 0.0
	var braking_to := -1.0
	var exiting := -1.0
	while true:
		# Merging in over the first seconds, turning off at the end
		if exiting < 0.0:
			y = maxf(EGO_Y, 6.5 - (6.5 - EGO_Y) * clampf(tt / 2.5, 0.0, 1.0))
		else:
			y = minf(7.0, EGO_Y + (7.0 - EGO_Y) * clampf((tt - exiting) / 2.5, 0.0, 1.0))
		var a := 1.0 if v < cruise else 0.0
		for b in brakes:
			if tt >= b[0] and tt < b[0] + 0.1:
				braking_to = b[1]
		if braking_to >= 0.0:
			a = -2.5
			if v <= braking_to:
				braking_to = -1.0
				a = 0.0
		if exiting < 0.0 and pos >= exit_s:
			exiting = tt
		if exiting >= 0.0:
			a = minf(a, -0.5) if v > 5.0 else 0.0
		car.times.append(tt)
		car.positions.append(pos)
		car.speeds.append(v)
		car.lateral.append(y)
		if exiting >= 0.0 and tt - exiting > 2.6:
			break
		v = maxf(0.0, v + a * Car.SAMPLE)
		pos += v * Car.SAMPLE
		tt += Car.SAMPLE
	return car

# --- A run ---

# One drive of the course, the player's or the reference planner's: its own copies of the crosswalks and the
# cars (triggered by its ego), its ego's motion and its scores
class Run:
	var course: PlannerCourse
	var crosswalks: Array[Crosswalk] = []
	var cars: Array[Car] = []
	var t := 0.0
	var s := 0.0  # [m] the ego's middle
	var v := 0.0
	var a := 0.0
	var jerk := 0.0
	var command := COAST
	var aeb := false
	var aeb_count := 0
	var aeb_end := -INF  # [s] when the last AEB let go
	var comfort := COMFORT_MAX
	var safety := SAFETY_MAX
	var finished := false
	var failed := ""  # why, if it failed
	var stop_error := 0.0  # [m] of the front from the goal line, at the end
	var stopped_for := 0.0
	var trace := PackedFloat32Array()  # s every step (the reference run's is drawn as the ghost)
	var events: Array[String] = []  # since the last step, for the game to show

	func front() -> float:
		return s + EGO_LENGTH * 0.5

	func step() -> void:
		events.clear()
		if finished:
			return
		_trigger()
		var prev_front := front()
		var prev_a := a
		# AEB: the nearest obstacle ahead so close for the closing speed that not hitting it needs a
		# harder deceleration than a driver's (or a planner's) normal hard braking
		var threat := obstacle_ahead(t, front(), v)
		var needed := 0.0
		if threat.x < INF and threat.y > 0.0:
			needed = threat.y * threat.y / (2.0 * maxf(threat.x - 0.5, 0.05))
		var danger := needed > AEB_NEEDED
		if danger and not aeb:
			# A new intervention (not the same one again right after letting go)
			if t - aeb_end > 2.0:
				aeb_count += 1
				safety -= 60.0
				events.append("aeb")
			aeb = true
		elif aeb and (v <= 0.0 or needed < AEB_NEEDED * 0.5):
			# It holds until stopped, or until the danger is clearly gone
			aeb = false
			aeb_end = t
		if aeb:
			a = AEB_DECEL
		else:
			a += (command - a) * minf(1.0, DT / LAG)
		jerk = (a - prev_a) / DT
		v = maxf(0.0, v + a * DT)
		if v == 0.0 and a < 0.0:
			a = 0.0
		s += v * DT
		t += DT
		trace.append(s)
		_score(prev_front)

	func _trigger() -> void:
		for crosswalk in crosswalks:
			if not crosswalk.triggered() and front() >= crosswalk.trigger:
				crosswalk.start(t)
			# A pedestrian does not step in front of a car close to the crosswalk and still moving:
			# it waits (its crossing starts later)
			var gap := crosswalk.s - front()
			if gap > -crosswalk.width and gap < maxf(8.0, v * 1.5) and v > 1.0:
				for p in crosswalk.pedestrians:
					if p.about_to_enter(t):
						p.start += DT
		for car in cars:
			if car.start == INF and front() >= car.trigger:
				car.start = t

	func _score(prev_front: float) -> void:
		# Comfort: strong acceleration and jerk
		if absf(a) > 2.0 and not aeb:
			comfort -= (absf(a) - 2.0) * 25.0 * DT
		if absf(jerk) > 2.5 and not aeb:
			comfort -= (absf(jerk) - 2.5) * 3.0 * DT
		comfort = maxf(comfort, 0.0)
		# Safety: headway, speeding
		var lead := lead_gap(t, front())
		if lead.x < INF and v > 2.0 and lead.x / v < 1.0:
			safety -= 25.0 * DT
			if fposmod(t, 1.0) < DT:
				events.append("close")
		var over := v * 3.6 - course.limit_at(s) - 3.0
		if over > 0.0:
			safety -= over * 2.0 * DT
			if fposmod(t, 1.0) < DT:
				events.append("speeding")
		safety = maxf(safety, 0.0)
		# Failures
		for light in course.lights:
			if prev_front < light.s and front() >= light.s and light.state(t) == 2:
				_fail("Ran a red light")
				return
		if collides(t, s):
			_fail("Collision")
			return
		if t >= TIME_LIMIT:
			_fail("Time up")
			return
		# The end: stopped near the goal (or past it)
		var error := front() - course.length
		if v < 0.05 and error > -8.0:
			stopped_for += DT
			if stopped_for >= 0.4:
				finished = true
				stop_error = error
				events.append("finish")
		else:
			stopped_for = 0.0

	func _fail(reason: String) -> void:
		failed = reason
		finished = true
		events.append("fail")

	# The nearest obstacle in the ego's lane ahead of `front`: Vector2(gap [m], closing speed [m/s]),
	# INF gap if none. A car in the lane, or a crosswalk with a pedestrian on it.
	func obstacle_ahead(at: float, ego_front: float, speed: float) -> Vector2:
		var best := Vector2(INF, 0.0)
		var lead := lead_gap(at, ego_front)
		if lead.x < best.x:
			best = Vector2(lead.x, speed - lead.y)
		for crosswalk in crosswalks:
			var gap := crosswalk.s - ego_front
			if gap > -0.5 and crosswalk.occupied(at) and gap < best.x:
				best = Vector2(maxf(gap, 0.0), speed)
		return best

	# The car ahead in the ego's lane: Vector2(gap [m], its speed), INF gap if none
	func lead_gap(at: float, ego_front: float) -> Vector2:
		var best := Vector2(INF, 0.0)
		for car in cars:
			if not car.in_lane(at):
				continue
			var gap := car.s(at) - EGO_LENGTH * 0.5 - ego_front
			if gap > -EGO_LENGTH * 0.5 and gap < best.x:
				best = Vector2(gap, car.v(at))
		return best

	func collides(at: float, ego_s: float) -> bool:
		var rear := ego_s - EGO_LENGTH * 0.5
		var ego_front := ego_s + EGO_LENGTH * 0.5
		for car in cars:
			if car.in_lane(at) and absf(car.s(at) - ego_s) < EGO_LENGTH - 0.2:
				return true
		for crosswalk in crosswalks:
			if ego_front > crosswalk.s and rear < crosswalk.s + crosswalk.width and crosswalk.occupied(at):
				return true
		return false

	func efficiency() -> float:
		if failed != "":
			return 0.0
		return clampf(EFFICIENCY_AT_TARGET * course.target_time() / maxf(t, 0.01), 0.0, EFFICIENCY_MAX)

	func stop_points() -> float:
		if failed != "":
			return 0.0
		return STOP_MAX * clampf(1.0 - absf(stop_error) / STOP_TOLERANCE, 0.0, 1.0)

	func total() -> float:
		if failed != "":
			return 0.0
		return efficiency() + comfort + safety + stop_points()

	static func rank(points: float) -> String:
		if points >= 900.0:
			return "S"
		if points >= 800.0:
			return "A"
		if points >= 650.0:
			return "B"
		if points >= 500.0:
			return "C"
		return "D"

	# --- The reference planner: rule based ---

	# Its acceleration command: the lowest of cruising at the limit (slowing for a lower one ahead),
	# following the car ahead (IDM) and stopping where needed (a light that will be red or yellow when
	# it gets there, a crosswalk that will be occupied then, the goal). It knows what a planner would:
	# the lights' timing (V2I), and the pedestrians and cars once they appear.
	func plan_command() -> float:
		var ego_front := front()
		var target := course.limit_at(s) / 3.6 - 0.4
		for zone in course.zones:
			var d := zone.s0 - ego_front
			if d > 0.0 and d < 80.0:
				target = minf(target, sqrt(pow(zone.limit / 3.6 - 0.4, 2.0) + 2.0 * 0.8 * d))
		var result := clampf(0.8 * (target - v), -1.5, 1.0)
		# Following (IDM)
		var lead := lead_gap(t, ego_front)
		if lead.x < 120.0:
			var gap := maxf(lead.x, 0.1)
			var desired := 4.0 + v * 1.6 + v * (v - lead.y) / (2.0 * sqrt(1.2 * 1.5))
			result = minf(result, 1.2 * (1.0 - pow(v / maxf(target, 1.0), 4.0) - pow(maxf(desired, 0.0) / gap, 2.0)))
		# Stops
		for light in course.lights:
			var d := light.s - ego_front
			if d > -0.5 and d < 100.0:
				var arrival := t + d / maxf(v, 4.0)
				var now := light.state(t)
				var must := light.state(arrival) != 0 or now == 2 or (now == 1 and d > v * v / (2.0 * 3.0))
				if must:
					result = minf(result, PlannerCourse._stop_command(v, d - 1.0, 1.6))
		for crosswalk in crosswalks:
			# Yields to the pedestrians: stops at the stop line (or right before the crosswalk once
			# past the line) while any of them is still to cross; once on it, it goes on across
			var d := maxf(crosswalk.stop_line - ego_front, crosswalk.s - 1.0 - ego_front)
			if crosswalk.s - ego_front > 0.0 and d < 80.0 and crosswalk.triggered() and crosswalk.pending(t):
				result = minf(result, PlannerCourse._stop_command(v, d, 1.6))
		result = minf(result, PlannerCourse._stop_command(v, course.length - ego_front - 0.4, 1.2))
		return clampf(result, MAX_DECEL, MAX_ACCEL)

# [s] the reference planner's time (its own, while it is being planned)
func target_time() -> float:
	return reference.t if reference != null else 0.0

func new_run() -> Run:
	var run := Run.new()
	run.course = self
	for crosswalk in crosswalks:
		run.crosswalks.append(crosswalk.copy())
	for car in cars:
		run.cars.append(car.copy())
	return run

# [km/h] at s
func limit_at(s: float) -> float:
	for zone in zones:
		if s >= zone.s0 and s < zone.s1:
			return zone.limit
	return DEFAULT_LIMIT

# Following a velocity profile that stops in `d` with deceleration `decel` (shortened by the way
# covered while the acceleration follows its command)
static func _stop_command(v: float, d: float, decel: float) -> float:
	var allowed := sqrt(2.0 * decel * maxf(d - v * LAG, 0.0))
	return clampf(2.5 * (allowed - v), MAX_DECEL, 1.0)

func plan() -> Run:
	var run := new_run()
	while not run.finished:
		run.command = run.plan_command()
		run.step()
	return run

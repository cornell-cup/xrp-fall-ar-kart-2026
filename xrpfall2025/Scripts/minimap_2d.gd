extends Control

## Space between the panel's edge and the track, in pixels.
@export var padding: float = 26.0
## How far in from the panel's edge the off-map arrow sits.
@export var edge_margin: float = 16.0
## The road never gets thinner than this on screen, so small maps stay readable.
@export var min_road_width: float = 10.0
## Show the rival karts as well as the player.
@export var show_rivals: bool = true
## Free play (no track): the minimap shows a square this many world units across,
## centred on the start, with a grid instead of a road. 0 = a race (draw the track).
@export var free_area: float = 0.0
## Free play grid spacing in world units (50 = 1 m on the race tracks' scale).
@export var grid_spacing: float = 50.0

const NAVY := Color("#161C28")
const DARKEST := Color("#0B0F17")
const ROAD := Color("#2A3242")
const LIGHT_GREY := Color("#C3CCDA")
const WHITE := Color("#FFFFFF")
const XRP_RED := Color("#D0112B")
const XRP_BLUE := Color("#0070FF")

const EDGE_DEPTH := 6.0      # the thick bottom edge, like the buttons
const CORNER := 28

var _loader: Node
var _car_manager: Node
var _track_world := PackedVector2Array()   # centre line in world X/Z
var _track_map := PackedVector2Array()     # the same points on the minimap
var _closed := false
var _half_width := 0.0
var _scale := 1.0
var _offset := Vector2.ZERO
var _panel: StyleBoxFlat
var _panel_edge: StyleBoxFlat
# The karts are drawn on their own layer every frame; the panel and track underneath
# only redraw when they change
var _markers: Control


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	clip_contents = true

	_panel = StyleBoxFlat.new()
	_panel.bg_color = Color(NAVY, 0.9)
	_panel.border_color = LIGHT_GREY
	_panel.set_border_width_all(3)
	_panel.set_corner_radius_all(CORNER)

	_panel_edge = StyleBoxFlat.new()
	_panel_edge.bg_color = DARKEST
	_panel_edge.set_corner_radius_all(CORNER)

	var scene := get_tree().current_scene
	if scene:
		_loader = scene.get_node_or_null("TrackLoader")
		_car_manager = scene.get_node_or_null("CarManager")
	_markers = Control.new()
	_markers.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_markers.set_anchors_preset(Control.PRESET_FULL_RECT)
	_markers.draw.connect(_draw_markers)
	add_child(_markers)
	resized.connect(_fit)
	if free_area > 0.0:
		_fit()


func _process(_delta: float) -> void:
	if free_area <= 0.0 and _track_world.is_empty():
		_try_read_track()
	_markers.queue_redraw()


# The track only exists after TrackLoader.Init() runs, so keep trying until it does
func _try_read_track() -> void:
	if _loader == null:
		return
	var path := _loader.get("innerPath") as Path3D
	if path == null or path.curve == null or path.curve.point_count < 2:
		return
	var baked := path.curve.get_baked_points()
	if baked.size() < 2:
		return
	for p in baked:
		var g := path.to_global(p)
		_track_world.append(Vector2(g.x, g.z))
	_half_width = float(_loader.get("roadHalfWidth"))
	# join the ends if the loop comes back to the start
	_closed = _track_world[0].distance_to(_track_world[-1]) < _half_width * 4.0
	if _closed:
		_track_world.append(_track_world[0])
	_fit()



func _fit() -> void:
	if free_area > 0.0:
		var free_rect := _map_rect().grow(-padding)
		_scale = minf(free_rect.size.x, free_rect.size.y) / free_area
		_offset = free_rect.get_center()
		queue_redraw()
		return
	if _track_world.is_empty():
		return
	var lo := _track_world[0]
	var hi := _track_world[0]
	for p in _track_world:
		lo = Vector2(minf(lo.x, p.x), minf(lo.y, p.y))
		hi = Vector2(maxf(hi.x, p.x), maxf(hi.y, p.y))
	lo -= Vector2.ONE * _half_width
	hi += Vector2.ONE * _half_width
	var area := _map_rect().grow(-padding)
	var extent := hi - lo
	_scale = minf(area.size.x / maxf(extent.x, 0.001), area.size.y / maxf(extent.y, 0.001))
	_offset = area.get_center() - (lo + hi) * 0.5 * _scale
	_track_map.clear()
	for p in _track_world:
		_track_map.append(p * _scale + _offset)
	queue_redraw()


func _map_rect() -> Rect2:
	return Rect2(Vector2.ZERO, size - Vector2(0, EDGE_DEPTH))


func _to_map(world: Vector3) -> Vector2:
	return Vector2(world.x, world.z) * _scale + _offset


func _draw() -> void:
	var rect := _map_rect()
	draw_style_box(_panel_edge, Rect2(rect.position + Vector2(0, EDGE_DEPTH), rect.size))
	draw_style_box(_panel, rect)
	if free_area > 0.0:
		_draw_free_area()
	elif _track_map.size() < 2:
		return
	else:
		var road_w := maxf(_half_width * 2.0 * _scale, min_road_width)
		_draw_round_line(_track_map, LIGHT_GREY, road_w + 6.0)   # white-grey outline
		_draw_round_line(_track_map, ROAD, road_w)               # road
		_draw_dashes(_track_map, maxf(road_w * 0.12, 1.5), road_w * 0.6)
		_draw_start_line(road_w)


func _draw_markers() -> void:
	if free_area <= 0.0 and _track_map.size() < 2:
		return
	var rect := _map_rect()
	var player_pos := Vector2.INF
	var player_heading := Vector2.UP
	for kart in _karts():
		var body: Node3D = kart["body"]
		var pos := _to_map(body.global_position)
		var fwd := -body.global_basis.z
		var heading := Vector2(fwd.x, fwd.z)
		heading = heading.normalized() if heading.length() > 0.001 else Vector2.UP
		if kart["player"]:
			player_pos = pos
			player_heading = heading
		elif show_rivals:
			_draw_rival(pos)
	if player_pos != Vector2.INF:
		if rect.grow(-edge_margin).has_point(player_pos):
			_draw_player(player_pos, player_heading)
		else:
			_draw_offmap_arrow(rect, player_pos)


func _draw_round_line(points: PackedVector2Array, color: Color, width: float) -> void:
	draw_polyline(points, color, width, true)
	var step := maxi(1, points.size() / 120)
	for i in range(0, points.size(), step):
		draw_circle(points[i], width * 0.5, color)


func _draw_dashes(points: PackedVector2Array, width: float, dash: float) -> void:
	var gap := dash
	var phase := 0.0
	var color := Color(WHITE, 0.8)
	for i in points.size() - 1:
		var a := points[i]
		var b := points[i + 1]
		var seg := a.distance_to(b)
		if seg < 0.001:
			continue
		var d := 0.0
		while d < seg:
			var limit := dash if phase < dash else dash + gap
			var step := minf(seg - d, limit - phase)
			if phase < dash:
				draw_line(a.lerp(b, d / seg), a.lerp(b, (d + step) / seg), color, width, true)
			d += step
			phase += step
			if phase >= dash + gap:
				phase -= dash + gap


func _draw_start_line(road_w: float) -> void:
	var p0 := _track_map[0]
	var tangent := (_track_map[1] - p0).normalized()
	var normal := tangent.orthogonal()
	var cols := 4
	var s := road_w / cols
	for col in cols:
		for row in 2:
			var c := p0 + normal * (-road_w * 0.5 + s * (col + 0.5)) + tangent * (s * (row - 0.5))
			var h := s * 0.5
			var quad := PackedVector2Array([
				c + (-tangent - normal) * h, c + (tangent - normal) * h,
				c + (tangent + normal) * h, c + (-tangent + normal) * h,
			])
			draw_colored_polygon(quad, WHITE if (col + row) % 2 == 0 else NAVY)


# Free play mode
func _draw_free_area() -> void:
	var half := free_area / 2.0
	var top_left := _to_map(Vector3(-half, 0, -half))
	var bottom_right := _to_map(Vector3(half, 0, half))
	draw_rect(Rect2(top_left, bottom_right - top_left), ROAD)
	var lines := int(round(free_area / grid_spacing))
	for i in lines + 1:
		var w := -half + i * grid_spacing
		var through_start := absf(w) < 0.001
		var color := Color(LIGHT_GREY, 0.35) if through_start else Color(LIGHT_GREY, 0.12)
		var width := 2.0 if through_start else 1.0
		draw_line(_to_map(Vector3(w, 0, -half)), _to_map(Vector3(w, 0, half)), color, width)
		draw_line(_to_map(Vector3(-half, 0, w)), _to_map(Vector3(half, 0, w)), color, width)
	draw_rect(Rect2(top_left, bottom_right - top_left), LIGHT_GREY, false, 2.0)
	var c := _to_map(Vector3.ZERO)
	var s := 6.0
	for col in 2:
		for row in 2:
			draw_rect(Rect2(c + Vector2(col - 1, row - 1) * s, Vector2(s, s)), WHITE if (col + row) % 2 == 0 else NAVY)


func _draw_rival(pos: Vector2) -> void:
	_markers.draw_circle(pos, 9.0, WHITE)
	_markers.draw_circle(pos, 6.5, XRP_BLUE)


func _draw_player(pos: Vector2, heading: Vector2) -> void:
	# slow pulse for player
	var t := fmod(Time.get_ticks_msec() / 1000.0, 1.4) / 1.4
	_markers.draw_arc(pos, 13.0 + 9.0 * t, 0.0, TAU, 40, Color(XRP_RED, 0.55 * (1.0 - t)), 3.0, true)
	_markers.draw_circle(pos + Vector2(0, 2), 13.0, Color(DARKEST, 0.6))
	_markers.draw_circle(pos, 13.0, WHITE)
	_markers.draw_circle(pos, 10.0, XRP_RED)
	var side := heading.orthogonal()
	_markers.draw_colored_polygon(PackedVector2Array([
		pos + heading * 7.0, pos - heading * 4.0 + side * 5.0, pos - heading * 4.0 - side * 5.0,
	]), WHITE)

func _draw_offmap_arrow(rect: Rect2, target: Vector2) -> void:
	var center := rect.get_center()
	var dir := (target - center).normalized()
	var half := rect.size * 0.5 - Vector2.ONE * edge_margin
	var reach_x := half.x / absf(dir.x) if dir.x != 0.0 else INF
	var reach_y := half.y / absf(dir.y) if dir.y != 0.0 else INF
	var tip := center + dir * minf(reach_x, reach_y)
	var side := dir.orthogonal()
	var outline := PackedVector2Array([tip + dir * 4.0, tip - dir * 15.0 + side * 12.0, tip - dir * 15.0 - side * 12.0])
	var fill := PackedVector2Array([tip, tip - dir * 12.0 + side * 8.0, tip - dir * 12.0 - side * 8.0])
	_markers.draw_colored_polygon(outline, WHITE)
	_markers.draw_colored_polygon(fill, XRP_RED)

func _karts() -> Array[Dictionary]:
	var out: Array[Dictionary] = []
	if _car_manager == null:
		return out
	for car in _car_manager.get_children():
		var bodies := car.find_children("*", "CharacterBody3D", true, false)
		if bodies.is_empty():
			continue
		var script := car.get_script() as Script
		var is_player := script != null and script.resource_path.ends_with("Player.cs")
		out.append({"body": bodies[0], "player": is_player})
	return out

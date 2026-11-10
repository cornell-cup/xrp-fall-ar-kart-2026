extends Control
## Animated XRP speed-streak background (the "parallax slide" from the design).
## Add a Control, set it to Full Rect, attach this script, and keep it as the
## first child so your menu draws on top of it.

@export var streak_count: int = 30
@export var min_speed: float = 230.0   # pixels per second
@export var max_speed: float = 430.0
@export var angle_degrees: float = -12.0
@export var background_color: Color = Color("#161C28")

const COLORS: Array[Color] = [
	Color("#D0112B"), Color("#0070FF"), Color("#FFFFFF"), Color("#4D9BFF"),
]
const HEIGHTS: Array[int] = [6, 8, 10, 14, 18, 22]

var _streaks: Array[Dictionary] = []


func _ready() -> void:
	clip_contents = true
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	for i in streak_count:
		var panel := Panel.new()
		var style := StyleBoxFlat.new()
		var c: Color = COLORS.pick_random()
		c.a = randf_range(0.3, 0.45) if c == Color.WHITE else randf_range(0.55, 0.85)
		style.bg_color = c
		var h: int = HEIGHTS.pick_random()
		style.set_corner_radius_all(h / 2)
		panel.add_theme_stylebox_override("panel", style)
		panel.size = Vector2(randf_range(180.0, 520.0), h)
		panel.rotation = deg_to_rad(angle_degrees)
		panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
		add_child(panel)
		_streaks.append({"node": panel, "speed": randf_range(min_speed, max_speed)})
		_respawn(panel, true)


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), background_color)


func _process(delta: float) -> void:
	var dir := Vector2.LEFT.rotated(deg_to_rad(angle_degrees))
	for s in _streaks:
		var panel: Panel = s["node"]
		panel.position += dir * float(s["speed"]) * delta
		if panel.position.x + panel.size.x < -60.0 or panel.position.y > size.y + 120.0:
			_respawn(panel, false)


func _respawn(panel: Panel, anywhere: bool) -> void:
	var x := randf_range(-panel.size.x, size.x) if anywhere else size.x + randf_range(20.0, 240.0)
	panel.position = Vector2(x, randf_range(-160.0, size.y))

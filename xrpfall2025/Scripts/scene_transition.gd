extends CanvasLayer

# Animated wipe between screens: red, blue and navy bands, slanted like the menu
# streaks, sweep in from the right to cover the screen, the scene changes while
# it's covered, then they sweep off to the left. Use change_scene() (or
# change_to_node() for a scene that needs data first) instead of the SceneTree's

const COVER_S := 0.6
const REVEAL_S := 0.35
# How far each band trails the one before it, as a fraction of the sweep
const STAGGER := 0.2
# Horizontal lean of the bands' edges across the screen's height
const SLANT := 260.0
const COLORS: Array[Color] = [Color("#D0112B"), Color("#0070FF"), Color("#161C28")]

var _wipe := Control.new()
# Where the sweep is: 0..1 covering, then 1..2 revealing
var _progress := 0.0
var _busy := false


func _ready() -> void:
	layer = 100   # above every scene's UI
	process_mode = Node.PROCESS_MODE_ALWAYS   # works from the pause menu too
	_wipe.set_anchors_preset(Control.PRESET_FULL_RECT)
	_wipe.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_wipe.draw.connect(_draw_bands)
	_wipe.visible = false
	add_child(_wipe)


func change_scene(path: String) -> void:
	await _run(func(): get_tree().change_scene_to_file(path))


# Swaps in a scene that's already been made (e.g. the results screen, which
# gets its lap times before it's shown)
func change_to_node(node: Node) -> void:
	await _run(func():
		var old = get_tree().current_scene
		get_tree().root.add_child(node)
		get_tree().current_scene = node
		if old:
			old.queue_free())


func _run(swap: Callable) -> void:
	if _busy:
		return
	_busy = true
	_wipe.visible = true
	# no clicks get through while the screen is changing
	_wipe.mouse_filter = Control.MOUSE_FILTER_STOP

	var tween = create_tween()
	tween.tween_method(_set_progress, 0.0, 1.0, COVER_S)
	await tween.finished

	swap.call()
	# a new screen never starts paused (leaving from the pause menu)
	get_tree().paused = false
	# let the new scene load and draw once before uncovering it
	await get_tree().process_frame
	await get_tree().process_frame

	tween = create_tween()
	tween.tween_method(_set_progress, 1.0, 2.0, REVEAL_S)
	await tween.finished

	_wipe.visible = false
	_wipe.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_busy = false


func _set_progress(value: float) -> void:
	_progress = value
	_wipe.queue_redraw()


func _draw_bands() -> void:
	var size = _wipe.size
	var span = size.x + SLANT   # a band's left edge goes from size.x to -SLANT to cover
	for i in COLORS.size():
		var left: float
		if _progress <= 1.0:
			# covering: red leads, navy (drawn last, on top) arrives last
			var t = _band_t(_progress, i)
			left = lerpf(size.x, -SLANT, t)
		else:
			# revealing: navy leaves first, the stripes trail behind it
			var t = _band_t(_progress - 1.0, COLORS.size() - 1 - i)
			left = lerpf(-SLANT, -SLANT - span - SLANT, t)
		var width = span + SLANT
		draw_band(left, width, size.y, COLORS[i])


func _band_t(p: float, order: int) -> float:
	var start = order * STAGGER
	var t = clampf((p - start) / (1.0 - (COLORS.size() - 1) * STAGGER), 0.0, 1.0)
	return ease(t, -2.2)   # ease in and out


func draw_band(left: float, width: float, height: float, color: Color) -> void:
	# a parallelogram leaning right, like the menu streaks
	_wipe.draw_colored_polygon(PackedVector2Array([
		Vector2(left + SLANT, 0), Vector2(left + SLANT + width, 0),
		Vector2(left + width, height), Vector2(left, height),
	]), color)

extends Control

# Hover effect for the menu buttons: a brighter colour, a thick white outline
# that pops out and then pulses, and a small grow. Works on Buttons (pills and
# cards), TextureButtons (the round back arrow) and PanelContainers (the logo cards)

# Only the main buttons (Start, Quick Start, Track Builder, RACE!) also play the hover sound
@export var hover_sound := false
# Corner radius of the button's shape, for image buttons whose shape the script
# can't read (-1 = from the button's style, or round for image buttons)
@export var outline_corner := -1

const OUTLINE_WIDTH := 7
# How far outside the button the outline sits
const OUTLINE_GAP := 7.0
const GROW := 1.04
const IN_S := 0.18
const OUT_S := 0.12
# White buttons can't get brighter, so they get a light blue tint instead
const LIGHT_TINT := Color("#D6E6FF")
# The outline pulses between white and this. Colour rather than transparency,
# since a see-through outline shows a seam where its two halves meet
const PULSE_COLOR := Color("#9CC6FF")

var _outline := Panel.new()
var _outline_style := StyleBoxFlat.new()
# The button's own corner radius, so the outline follows its shape (pill or card)
var _corner := 1000
# Panels have no hover state, so their style is swapped by hand
var _panel_normal: StyleBox
var _panel_hover: StyleBox
# A PanelContainer fits the outline inside its padding; this pushes it back out
# to the panel's edge (left, top, right, bottom)
var _padding := [0.0, 0.0, 0.0, 0.0]
var _tween: Tween
var _pulse: Tween


func _ready() -> void:
	var normal = get_theme_stylebox("normal") if is_class("Button") else null
	if normal is StyleBoxFlat:
		_corner = normal.corner_radius_top_left
		# a stronger hover colour than the theme's
		var hover = get_theme_stylebox("hover")
		if hover is StyleBoxFlat:
			hover = hover.duplicate()
			if normal.bg_color.v > 0.9:
				hover.bg_color = normal.bg_color.lerp(LIGHT_TINT, 0.6)
			else:
				hover.bg_color = normal.bg_color.lightened(0.22)
			add_theme_stylebox_override("hover", hover)
	elif is_class("PanelContainer"):
		var panel = get_theme_stylebox("panel")
		if panel is StyleBoxFlat:
			_corner = panel.corner_radius_top_left
			_panel_normal = panel
			_panel_hover = panel.duplicate()
			_panel_hover.bg_color = panel.bg_color.lerp(LIGHT_TINT, 0.6) if panel.bg_color.v > 0.9 \
				else panel.bg_color.lightened(0.22)
			_padding = [panel.get_margin(SIDE_LEFT), panel.get_margin(SIDE_TOP),
				panel.get_margin(SIDE_RIGHT), panel.get_margin(SIDE_BOTTOM)]

	# the outline, hidden until hovered. First child, so it's under the button's
	# own children (like the kart cards' tick)
	_outline_style.draw_center = false
	_outline_style.border_color = Color.WHITE
	_outline_style.corner_detail = 20
	_set_outline(0.0)
	_outline.add_theme_stylebox_override("panel", _outline_style)
	_outline.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_outline.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(_outline)
	move_child(_outline, 0)

	resized.connect(_on_resized)
	_on_resized()
	mouse_entered.connect(_on_hover)
	mouse_exited.connect(_on_unhover)


func _on_resized() -> void:
	pivot_offset = size / 2
	# a radius bigger than half the button leaves notches where the ends meet
	var corner = outline_corner if outline_corner >= 0 else _corner
	var radius = mini(corner, int(minf(size.x, size.y) / 2))
	_outline_style.set_corner_radius_all(radius + int(OUTLINE_GAP))


# 0 = no outline, 1 = full width and gap
func _set_outline(amount: float) -> void:
	amount = maxf(amount, 0.0)   # the springy ease dips below 0 on the way out
	_outline_style.set_border_width_all(int(round(OUTLINE_WIDTH * amount)))
	for side in 4:
		_outline_style.set_expand_margin(side, _padding[side] + OUTLINE_GAP * amount)


func _on_hover() -> void:
	if get("disabled"):
		return
	if hover_sound:
		SoundManager.play_hover()
	if _panel_hover:
		add_theme_stylebox_override("panel", _panel_hover)
	_animate(1.0, GROW, IN_S)
	# a slow pulse of the outline while the mouse stays on it
	if _pulse:
		_pulse.kill()
	_pulse = create_tween().set_loops()
	_pulse.tween_property(_outline_style, "border_color", PULSE_COLOR, 0.5).set_trans(Tween.TRANS_SINE)
	_pulse.tween_property(_outline_style, "border_color", Color.WHITE, 0.5).set_trans(Tween.TRANS_SINE)


func _on_unhover() -> void:
	if _pulse:
		_pulse.kill()
	_outline_style.border_color = Color.WHITE
	if _panel_normal:
		add_theme_stylebox_override("panel", _panel_normal)
	_animate(0.0, 1.0, OUT_S)


func _animate(outline: float, grow: float, seconds: float) -> void:
	if _tween:
		_tween.kill()
	var start := float(_outline_style.border_width_left) / OUTLINE_WIDTH
	_tween = create_tween().set_parallel().set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)
	_tween.tween_method(_set_outline, start, outline, seconds)
	_tween.tween_property(self, "scale", Vector2.ONE * grow, seconds)
	# texture buttons have no hover style to swap, so they're tinted instead,
	# unless they come with their own hover image
	if is_class("TextureButton") and get("texture_hover") == null:
		_tween.tween_property(self, "self_modulate", LIGHT_TINT if outline > 0.0 else Color.WHITE, seconds)

extends Control

# One device on the loading screen (XRP robot, camera or gamepad). The card's
# border, status icon and status text all follow its state

enum State { SEARCHING, CONNECTED, NOT_FOUND, SIMULATED }

const CARDS := {
	State.SEARCHING: preload("res://Assets/ar_kart_ui/loading/device_card_searching@2x.png"),
	State.CONNECTED: preload("res://Assets/ar_kart_ui/loading/device_card_connected@2x.png"),
	State.NOT_FOUND: preload("res://Assets/ar_kart_ui/loading/device_card_not_found@2x.png"),
	State.SIMULATED: preload("res://Assets/ar_kart_ui/loading/device_card_simulated@2x.png"),
}
const ICONS := {
	State.SEARCHING: preload("res://Assets/ar_kart_ui/loading/status_searching_ring@2x.png"),
	State.CONNECTED: preload("res://Assets/ar_kart_ui/loading/status_connected@2x.png"),
	State.NOT_FOUND: preload("res://Assets/ar_kart_ui/loading/status_not_found@2x.png"),
	State.SIMULATED: preload("res://Assets/ar_kart_ui/loading/status_simulated@2x.png"),
}
const TEXTS := {
	State.SEARCHING: "Searching",
	State.CONNECTED: "Connected",
	State.NOT_FOUND: "Not found",
	State.SIMULATED: "Simulated",
}
const COLORS := {
	State.SEARCHING: Color("#AEB6C4"),
	State.CONNECTED: Color("#3DDC84"),
	State.NOT_FOUND: Color("#FF8A92"),
	State.SIMULATED: Color("#4D9BFF"),
}

# Spinner speed, in radians per second
const SPIN_SPEED := 6.0

# A connected (green) or simulated (blue) card gets a thicker border than the
# one in its image
const THICK_BORDER := 8
const BORDER_COLORS := {
	State.CONNECTED: Color("#3DDC84"),
	State.SIMULATED: Color("#0070FF"),
}
# How much the card grows for a moment when its device connects or is simulated
const POP_SCALE := 1.06

@export var device_name := "DEVICE"
@export var icon: Texture2D

var state := State.SEARCHING
var _pop: Tween
var _border_style := StyleBoxFlat.new()

@onready var card: TextureRect = $Card
@onready var status_icon: TextureRect = $StatusIcon
@onready var status_label: Label = $StatusLabel
@onready var thick_border: Panel = $ThickBorder


func _ready() -> void:
	$Icon.texture = icon
	$NameLabel.text = device_name
	# spin around the ring's centre
	status_icon.pivot_offset = status_icon.size / 2.0
	# grow from the middle of the card face (the image's bottom edge is its shadow)
	pivot_offset = Vector2(size.x / 2.0, thick_border.size.y / 2.0)
	_border_style.draw_center = false
	_border_style.set_border_width_all(THICK_BORDER)
	_border_style.set_corner_radius_all(43)
	_border_style.corner_detail = 16
	thick_border.add_theme_stylebox_override("panel", _border_style)
	set_state(State.SEARCHING)


func _process(delta: float) -> void:
	if state == State.SEARCHING:
		status_icon.rotation += SPIN_SPEED * delta


# play_sound: false when a louder sound is about to cover it (devices ready)
func set_state(new_state: State, play_sound := true) -> void:
	var changed := new_state != state
	state = new_state
	card.texture = CARDS[state]
	status_icon.texture = ICONS[state]
	status_icon.rotation = 0.0
	status_label.text = TEXTS[state]
	status_label.add_theme_color_override("font_color", COLORS[state])
	thick_border.visible = is_ready()
	if is_ready():
		_border_style.border_color = BORDER_COLORS[state]
		if changed:
			_play_connected(play_sound)


# The card pops out and back, the check pops in, with the button press sound
# (for both a real connection and a simulated one)
func _play_connected(play_sound: bool) -> void:
	if play_sound:
		SoundManager.play_click()
	if _pop:
		_pop.kill()
	scale = Vector2.ONE
	status_icon.scale = Vector2.ZERO
	_pop = create_tween().set_parallel()
	_pop.tween_property(self, "scale", Vector2.ONE * POP_SCALE, 0.12) \
		.set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_OUT)
	_pop.tween_property(self, "scale", Vector2.ONE, 0.25) \
		.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT).set_delay(0.12)
	_pop.tween_property(status_icon, "scale", Vector2.ONE, 0.35) \
		.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)


func is_ready() -> bool:
	return state == State.CONNECTED or state == State.SIMULATED

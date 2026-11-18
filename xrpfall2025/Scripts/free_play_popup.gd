extends Control

signal start_pressed

# Intro timings, in seconds
const CARD_IN_S := 0.45
const FIRST_PART_DELAY_S := 0.18
const PART_STAGGER_S := 0.09
const PART_IN_S := 0.4
# How small the card and its parts start before popping to full size
const CARD_START_SCALE := 0.8
const PART_START_SCALE := 0.5
# The steering wheel icon turns this far each way once it lands
const WIGGLE_DEG := 12.0

@onready var dim: ColorRect = $Dim
@onready var card: PanelContainer = $Card
@onready var icon: TextureRect = $Card/Column/Icon
@onready var title: Label = $Card/Column/Title
@onready var rows: Array = $Card/Column/Rows.get_children()
@onready var button_row: Control = $Card/Column/ButtonRow
@onready var start_button: Button = $Card/Column/ButtonRow/StartButton


func _ready() -> void:
	start_button.pressed.connect(_on_start_pressed)
	_play_intro()


func _play_intro() -> void:
	# hidden until the card's containers have laid out, so the pivots are right
	dim.modulate.a = 0.0
	card.modulate.a = 0.0
	for part in _parts():
		part.modulate.a = 0.0
	start_button.mouse_filter = Control.MOUSE_FILTER_IGNORE
	await get_tree().process_frame

	card.pivot_offset = card.size / 2.0
	card.scale = Vector2.ONE * CARD_START_SCALE
	var intro := create_tween().set_parallel()
	intro.tween_property(dim, "modulate:a", 1.0, 0.25)
	intro.tween_property(card, "modulate:a", 1.0, 0.2)
	intro.tween_property(card, "scale", Vector2.ONE, CARD_IN_S) \
		.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)

	# the icon, title, rows and button pop in one after another
	var delay := FIRST_PART_DELAY_S
	for part in _parts():
		# rows grow out from their left edge, everything else from its centre
		part.pivot_offset = Vector2(0.0, part.size.y / 2.0) if part in rows else part.size / 2.0
		part.scale = Vector2.ONE * PART_START_SCALE
		intro.tween_property(part, "modulate:a", 1.0, 0.2).set_delay(delay)
		intro.tween_property(part, "scale", Vector2.ONE, PART_IN_S) \
			.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT).set_delay(delay)
		delay += PART_STAGGER_S
	await intro.finished

	start_button.mouse_filter = Control.MOUSE_FILTER_STOP
	_wiggle_icon()


# The order the parts appear in
func _parts() -> Array:
	return [icon, title] + rows + [button_row]


# A quick turn left and right, like a steering wheel
func _wiggle_icon() -> void:
	icon.pivot_offset = icon.size / 2.0
	var wiggle := create_tween().set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)
	wiggle.tween_property(icon, "rotation", deg_to_rad(-WIGGLE_DEG), 0.12)
	wiggle.tween_property(icon, "rotation", deg_to_rad(WIGGLE_DEG), 0.22)
	wiggle.tween_property(icon, "rotation", deg_to_rad(-WIGGLE_DEG * 0.5), 0.18)
	wiggle.tween_property(icon, "rotation", 0.0, 0.14)


func _on_start_pressed() -> void:
	start_button.disabled = true
	# the card shrinks away before the countdown starts
	var outro := create_tween().set_parallel()
	outro.tween_property(card, "scale", Vector2.ONE * 0.9, 0.2) \
		.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_IN)
	outro.tween_property(card, "modulate:a", 0.0, 0.2)
	outro.tween_property(dim, "modulate:a", 0.0, 0.25)
	await outro.finished
	visible = false
	start_pressed.emit()

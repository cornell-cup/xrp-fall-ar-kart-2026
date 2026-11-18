extends Node

const START_POSITION := Vector3.ZERO
const START_FACING := Vector3.FORWARD

@onready var player = $CarManager/Player
@onready var hud: Control = $CanvasLayer/HUD
@onready var popup: Control = $PopupLayer/FreePlayPopup


func _ready() -> void:
	player.Init(START_POSITION, null, 0, START_FACING)
	hud.countdown_finished.connect(Callable(player, "RaceStarted"))
	hud.set_process_input(false)
	popup.start_pressed.connect(_on_start_pressed)


func _on_start_pressed() -> void:
	hud.set_process_input(true)
	hud.start_countdown()

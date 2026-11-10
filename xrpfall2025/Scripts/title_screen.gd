extends Control

@onready var background: Control = $Background

const CORNELL_CUP_URL := "https://www.cornellcuprobotics.com/"
const XRP_URL := "https://www.experiential.bot/xrp"

func _ready():
	SoundManager.play_menu_music()
	# Clicking the logo cards opens their websites in the browser
	$CornellCard.gui_input.connect(_on_logo_card_input.bind(CORNELL_CUP_URL))
	$XrpCard.gui_input.connect(_on_logo_card_input.bind(XRP_URL))

func _on_logo_card_input(event: InputEvent, url: String) -> void:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		OS.shell_open(url)

func _on_start_button_pressed() -> void:
	SceneTransition.change_scene("res://Scenes/GameplayScenes/customization_screen.tscn")
	pass


func _on_quickstart_pressed() -> void:
	SceneTransition.change_scene("res://Scenes/GameplayScenes/main.tscn")
	pass
	

func _on_quit_pressed() -> void:
	NetworkReceiver.quit_game()


func _on_track_editor_pressed() -> void:
	SceneTransition.change_scene("res://Scenes/GameplayScenes/track builder.tscn")

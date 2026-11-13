extends Control

@onready var car_button_group: ButtonGroup = $CarOptions/Car1.button_group
@onready var circuit_button: Button = $MapPanel/MapOptions/Map1
@onready var custom_button: Button = $MapPanel/MapOptions/FileSelect
@onready var custom_info: Label = $MapPanel/MapOptions/FileSelect/Info
@onready var file_dialog: FileDialog = $FileDialog

var selected_color: Color = Color.AQUA

func _ready():
	SoundManager.play_select_music()
	$CarOptions/Car1.pressed.connect(_on_car_pressed.bind(Color.AQUA))
	$CarOptions/Car2.pressed.connect(_on_car_pressed.bind(Color.RED))
	$CarOptions/Car3.pressed.connect(_on_car_pressed.bind(Color.GREEN))
	car_button_group.pressed.connect(func(_button): _update_checks())
	_update_checks()

	if LevelData.level_path != LevelData.DEFAULT_LEVEL_PATH:
		_show_custom_track(LevelData.level_path)
	file_dialog.file_selected.connect(_on_custom_track_selected)
	file_dialog.canceled.connect(_on_custom_track_canceled)

func _on_back_button_pressed() -> void:
	SceneTransition.change_scene("res://Scenes/GameplayScenes/title_screen.tscn")
	pass #

func _on_confirm_button_pressed() -> void:
	SceneTransition.change_scene("res://Scenes/GameplayScenes/main.tscn")
	pass 

func _update_checks() -> void:
	for car in $CarOptions.get_children():
		car.get_node("Check").visible = car.button_pressed

func _on_car_pressed(color: Color) -> void:
	pass


func _on_circuit_pressed() -> void:
	LevelData.level_path = LevelData.DEFAULT_LEVEL_PATH
	custom_info.text = "Load from Track\nBuilder"

func _on_file_select_pressed() -> void:
	file_dialog.popup_centered_clamped()

func _on_custom_track_selected(path: String) -> void:
	LevelData.level_path = path
	_show_custom_track(path)

func _on_custom_track_canceled() -> void:
	if LevelData.level_path == LevelData.DEFAULT_LEVEL_PATH:
		circuit_button.button_pressed = true
	else:
		custom_button.button_pressed = true

func _show_custom_track(path: String) -> void:
	custom_button.button_pressed = true
	custom_info.text = path.get_file().get_basename()


func save_customization():
	var save_data = { "car_color": selected_color.to_html() } # e.g. "#00ffff"
	var file = FileAccess.open("user://customization.json", FileAccess.WRITE)
	file.store_string(JSON.stringify(save_data))
	file.close()

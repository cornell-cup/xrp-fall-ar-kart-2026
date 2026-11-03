extends Control

@onready var background: TextureRect = $BackgroundImage

func _ready():
	SoundManager.play_menu_music()
	# bounce_background()

func _on_start_button_pressed() -> void:
	get_tree().change_scene_to_file("res://Scenes/GameplayScenes/customization_screen.tscn")
	
	pass # Replace with function body.


func _on_quickstart_pressed() -> void:
	get_tree().change_scene_to_file("res://Scenes/GameplayScenes/main.tscn")
	pass # Replace with function body.
	
func bounce_background():
	var tween = create_tween()
	tween.set_loops() # loops forever
	tween.tween_property(background, "position:y", background.position.y - 20, 5.0).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_OUT)
	tween.tween_property(background, "position:y", background.position.y + 20, 5.0).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN)


func _on_track_editor_pressed() -> void:
	get_tree().change_scene_to_file("res://Scenes/GameplayScenes/track builder.tscn")

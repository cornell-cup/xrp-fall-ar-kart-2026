extends Control

var lap_times: Array = []
var total_time: float = 0.0

# I should put all the buttons in a container so it's more efficient to turn them visible/invis
func _ready() -> void:
	SoundManager.play_match_finish()
	$HomeButton.visible = true
	$RestartButton.visible = true
	$Background.visible = true

# Set lap times
func set_lap_times(times: Array, overall_time: float):
	lap_times = times
	total_time = overall_time
	
	$ResultsLabel.text = "Lap 1: %s\nLap 2: %s\nLap 3: %s" % [
		format_time(lap_times[0]),
		format_time(lap_times[1]),
		format_time(lap_times[2])
	]
	
	$TotalTimeLabel.text = "Total Time: %s" % format_time(total_time)
	
# Return home. Moves to title screen
func _on_home_button_pressed() -> void:
	
	switch_screens()
	
	get_tree().change_scene_to_file("res://Scenes/GameplayScenes/title_screen.tscn")
	pass # Replace with function body.

# Restart game. Goes back to main scene
func _on_restart_button_pressed() -> void:
	
	switch_screens() # Used to reset things mainly
	
	get_tree().change_scene_to_file("res://Scenes/GameplayScenes/main.tscn")
	
	pass # Replace with function body.

# this is copied from hud which is sloppy, but a temporary easier fix
func format_time(time_sec: float) -> String:
	var minutes = int(time_sec) / 60
	var seconds = int(time_sec) % 60
	var milliseconds = int((time_sec - int(time_sec)) * 100)
	return "%02d:%02d.%02d" % [minutes, seconds, milliseconds]
	
	# Helper function to toggle visibility of end screen labels when screen switches
func switch_screens() -> void:
	if $ResultsLabel:
		$ResultsLabel.text = ""
	if $TotalTimeLabel:
		$TotalTimeLabel.text = ""

	lap_times.clear()
	total_time = 0.0
	
	$RestartButton.visible = false
	$HomeButton.visible = false
	$Background.visible = false
	

extends Control

var lap_times: Array = []
var total_time: float = 0.0

const BEST_ROW_COLOR := Color("#E6F0FF")
const BEST_ROW_BORDER := Color("#0070FF")

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

	# The fastest finished lap gets highlighted
	var best := -1
	for i in lap_times.size():
		if lap_times[i] > 0.0 and (best == -1 or lap_times[i] < lap_times[best]):
			best = i

	for i in 3:
		var row: PanelContainer = get_node("LapCard/Rows/Lap%d" % (i + 1))
		var time = lap_times[i] if i < lap_times.size() else 0.0
		row.get_node("Row/Time").text = format_time(time) if time > 0.0 else "--:--.--"
		row.get_node("Row/BestTag").visible = i == best
		if i == best:
			var style: StyleBoxFlat = row.get_theme_stylebox("panel").duplicate()
			style.bg_color = BEST_ROW_COLOR
			style.border_color = BEST_ROW_BORDER
			row.add_theme_stylebox_override("panel", style)

	$LapCard/Rows/FinalTime/Row/TotalTimeLabel.text = format_time(total_time)

# Finishing place, shown in the badge
func set_place(place: int):
	$PlaceBadge/Number.text = str(place)
	match place:
		1:
			$PlaceBadge/Suffix.text = "st"
		2:
			$PlaceBadge/Suffix.text = "nd"
		3:
			$PlaceBadge/Suffix.text = "rd"
		_:
			$PlaceBadge/Suffix.text = "th"

# Return home. Moves to title screen
func _on_home_button_pressed() -> void:

	switch_screens()

	SceneTransition.change_scene("res://Scenes/GameplayScenes/title_screen.tscn")
	pass # Replace with function body.

# Restart game. Goes back to main scene
func _on_restart_button_pressed() -> void:

	switch_screens() # Used to reset things mainly

	SceneTransition.change_scene("res://Scenes/GameplayScenes/main.tscn")

	pass # Replace with function body.

# this is copied from hud which is sloppy, but a temporary easier fix
func format_time(time_sec: float) -> String:
	var minutes = int(time_sec) / 60
	var seconds = int(time_sec) % 60
	var milliseconds = int((time_sec - int(time_sec)) * 100)
	return "%02d:%02d.%02d" % [minutes, seconds, milliseconds]

# The end screen is the current scene, so changing scenes frees it
func switch_screens() -> void:
	lap_times.clear()
	total_time = 0.0

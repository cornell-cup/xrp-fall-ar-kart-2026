extends Control

# Connecting player node with player script
# There are possibly better solutions for this connection but it does work for now
@onready var player_node = get_node("../../CarManager/Player")

# Countdown variables
@onready var countdown_label: Label = $CountdownLabel  
var counter: int = 3
var countdown_active: bool = false
signal countdown_finished

var use_tween := false # Toggle tween if wanted

# Item Bar variables
@onready var item_icon: TextureRect = $CurrentItem
@onready var stored_item_icon: TextureRect = $StoredItem

var item_textures = {
	"Fast": preload("res://UI/ar_kart_ui/items/item_fast@2x.png"),
	"Slow": preload("res://UI/ar_kart_ui/items/item_slow@2x.png"),
	"Inverted": preload("res://UI/ar_kart_ui/items/item_inverted@2x.png")

}

var last_item_type: String = ""

# Item effect timer and popups
@onready var effect_timer_label: Label = $EffectTimerLabel
@onready var item_popup_label: Label = $ItemPopupLabel
@onready var off_track_banner: Control = $OffTrackBanner
@onready var off_track_label: Label = $OffTrackBanner/Row/OffTrackLabel
# Whether the warning banner was up last frame, so its sound plays once as it appears
var warning_was_shown := false

# Place badge and lap counter
@onready var place_number: Label = $PlaceBadge/Badge/Number
@onready var place_suffix: Label = $PlaceBadge/Badge/Suffix
@onready var lap_number: Label = $LapPanel/Row/LapNumber
var item_popup_tween: Tween = null

# Pause menu
@onready var pause_menu: Control = $PauseMenu

# Stopwatch variables
var race_time: float = 0.0
var lap_start_time: float = 0.0
var lap_times: Array = [0.0, 0.0, 0.0]
var current_lap: int = 1
var stopwatch_running: bool = false

@onready var stopwatch_label: Label = $Stopwatch/StopwatchLabel

# On game start
func _ready():
	# The race music starts when the countdown ends; the countdown itself is quiet
	SoundManager.stop_music()
	# The countdown is started by SceneManager once the race is set up

	
	#create a default texture for item
	if not item_icon.texture:
		item_icon.texture = ImageTexture.new()  # empty texture
	item_icon.visible = false
	
	# Make sure Pause Menu isn't visible
	pause_menu.visible = false
	
	bounce_icon()
	
	# Popups for when the player uses an item or gets hit by one
	get_node("../../CarManager").connect("ItemUsed", Callable(self, "_on_item_used"))
	

# Constantly updating
func _process(delta: float) -> void:
	if player_node:
		# Set speed to absolute value
		var current_speed = abs(player_node.Speed)
		
		# Displaying live data taken from Player and updating text
		$TelemetryWindow/State.text = " State: " + str(state_name(player_node.Current))
		$TelemetryWindow/Position.text = " Position: (%.2f, %.2f)" % [player_node.CurrentPosition.x, player_node.CurrentPosition.z]
		$TelemetryWindow/Speed.text = " Speed: %.2f" % current_speed
		place_number.text = str(player_node.Place)
		place_suffix.text = get_place_suffix(player_node.Place).trim_prefix(str(player_node.Place))
		lap_number.text = str(player_node.Lap)
		
		# Item Bar. Constantly checks if the state has changed
		var current_state_name = state_name(player_node.Current) 
		if current_state_name != last_item_type:
			update_item_icon(current_state_name)
			last_item_type = current_state_name
			
			# Update stored item icon
		var stored_state_name = state_name(player_node.StoredItem)  
		update_stored_item_icon(stored_state_name)
		
		# Warnings: driving the wrong way, or off the road
		off_track_banner.visible = (player_node.IsWrongWay or player_node.IsOffTrack) and not player_node.FinishedRace
		if off_track_banner.visible and not warning_was_shown:
			SoundManager.play_off_track()
		warning_was_shown = off_track_banner.visible
		if off_track_banner.visible:
			if player_node.IsWrongWay:
				off_track_label.text = "Wrong way! Turn around"
			else:
				off_track_label.text = "Off track! Drive back to the road"
		
		# Time left on the active item effect
		if current_state_name != "Regular":
			effect_timer_label.text = "%.1fs" % player_node.EffectTimeRemaining
			effect_timer_label.visible = true
		else:
			effect_timer_label.visible = false
		
		# Stopwatch
		if stopwatch_running and not get_tree().paused:
			race_time += delta
			stopwatch_label.text = format_time(race_time)
			
			# TEST: End the game if the timer hits 10 seconds
			#if race_time >= 10.0:
				#finished_race()
				#return
		
			# FinishedRace is checked too because the final crossing doesn't change Lap
			if player_node.Lap != current_lap or player_node.FinishedRace:
				record_lap_time()

# Helper functions:

#return the cooresponding name of the State enum because to_string() is being silly
func state_name(current: int) -> String:
	match current:
		0:
			return "Inverted"
		1:
			return "Slow"
		2:
			return "Fast"
		3:
			return "Regular"	
		_:
			return "Other"
			
		
# Function to add the right suffix to the place number
func get_place_suffix(place_number: int) -> String:
	match place_number:
		1:
			return str(place_number) + "st"
		2:
			return str(place_number) + "nd"
		3:
			return str(place_number) + "rd"
		_:
			return str(place_number) + "th"
			
# Function to set up/start countdown
func start_countdown():
	counter = 3
	countdown_label.visible = true
	countdown_active = true
	
	# I need to condense this better if I like it like that
	$LapPanel.visible = false
	$PlaceBadge.visible = false
	$TelemetryWindow.visible = false
	$Stopwatch.visible = false
	$Images.visible = false
	
	
	race_time = 0.0
	stopwatch_running = false
	stopwatch_label.text = "00:00.00"
	
	_show_number()
	
# Function that displays the number for countdow until it reaches 0, then go
func _show_number():
	if counter > 0:
		countdown_label.text = str(counter)
		SoundManager.play_countdown_beep()
		countdown_label.scale = Vector2(0.5, 0.5)
		_animate_label()
	else:
		_show_go()

# Use if/else to control tween because I don't know if I like it.
# Toggleable T/F in variables
func _animate_label():
	if use_tween:	
		var tween := create_tween()
		countdown_label.modulate = Color(1, 1, 1, 0)
		tween.parallel().tween_property(countdown_label, "modulate:a", 1.0, 0.3)
		tween.tween_property(countdown_label, "scale", Vector2(1.5, 1.5), 0.8).set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)
		tween.tween_callback(Callable(self, "_next_step"))
	else:
		countdown_label.modulate = Color(1, 1, 1, 1)
		countdown_label.scale = Vector2.ONE
		await get_tree().create_timer(1.0).timeout
		_next_step()
		
# Decrements counter 
func _next_step(): 
	counter -= 1
	if counter > 0:
		_show_number()
	else:
		_show_go()

# Actually displays "Go" and calls to end the countdown
func _show_go():
	countdown_label.text = "Go!"
	SoundManager.play_countdown_go()
	if use_tween:
		countdown_label.scale = Vector2(0.5, 0.5)
		var tween := create_tween()
		tween.tween_property(countdown_label, "scale", Vector2(1.5, 1.5), 0.8).set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)
		tween.tween_callback(Callable(self, "_end_countdown"))
	else:
		await get_tree().create_timer(1.0).timeout
		_end_countdown()

# Ends the countdown and returns everything to its normal states.
# Sends out signal that can be used for Player class to indicate car can move again.
func _end_countdown():
	await get_tree().create_timer(0.5).timeout
	countdown_label.visible = false
	countdown_active = false
	
	# Again could be done better if I want to keep this
	$LapPanel.visible = true
	$PlaceBadge.visible = true
	$Stopwatch.visible = true
	$Images.visible = true
	
	# Start Stopwatch
	race_time = 0.0
	lap_start_time = 0.0
	current_lap = player_node.Lap
	stopwatch_running = true
	SoundManager.play_race_music()
	
	emit_signal("countdown_finished")

# Item Bar function

func update_item_icon(item_type: String) -> void:
	if item_type in item_textures:
		item_icon.texture = item_textures[item_type]
		item_icon.visible = true
	else:
		item_icon.visible = false  # Hide if no item
		
# Pause Menu

func _input(event):
	if event.is_action_pressed("ui_cancel"):  # Default Escape key
		if get_tree().paused:
			_resume_game()
		else:
			_pause_game()

func _pause_game():
	get_tree().paused = true
	pause_menu.visible = true
	SoundManager.play_click()
	SoundManager.pause_race_music()

# Resume the game
func _resume_game():
	get_tree().paused = false
	pause_menu.visible = false
	SoundManager.play_back()
	SoundManager.resume_race_music()
	

func _on_resume_button_pressed() -> void:
	_resume_game()
	pass # Replace with function body.


func _on_home_pressed() -> void:
	# the transition unpauses once the screen is covered
	SceneTransition.change_scene("res://Scenes/GameplayScenes/title_screen.tscn")
	pass # Replace with function body.
	
	# STOPWATCH
	
# Function to get individual lap times and store them
# Also used to display the times, both in list and popup
func record_lap_time():
	# Calculate the time for this lap
	var lap_time = race_time - lap_start_time
	lap_start_time = race_time 

	# Store the lap time in array
	if current_lap - 1 < lap_times.size():
		lap_times[current_lap - 1] = lap_time
	else:
		lap_times.append(lap_time)
		
	# Pop up label each lap
	var popup_label = $Stopwatch/LapTimePopupLabel  
	popup_label.text = "Lap %d: %s" % [current_lap, format_time(lap_time)]
	popup_label.visible = true
	popup_label.modulate.a = 1.0
	
	# Fade out
	var tween := create_tween()
	tween.tween_interval(0.8)
	tween.tween_property(popup_label, "modulate:a", 0.0, 0.5)
	tween.tween_callback(Callable(self, "_hide_lap_popup"))

	# List of times. Likely temporary and will only be shown on end screen
	$Stopwatch/LapTimesLabel.text = "Lap 1: %s\nLap 2: %s\nLap 3: %s" % [
		format_time(lap_times[0]),
		format_time(lap_times[1]),
		format_time(lap_times[2])
	]
	# Update to new lap number
	current_lap = player_node.Lap
	
	# Uses player end condition
	if player_node.FinishedRace:
		finished_race()
	elif current_lap == 3:
		SoundManager.play_final_lap()
	else:
		SoundManager.play_lap_complete()
	
# Simple helper function to format the time properly
func format_time(time_sec: float) -> String:
	var minutes = int(time_sec) / 60
	var seconds = int(time_sec) % 60
	var milliseconds = int((time_sec - int(time_sec)) * 100)
	return "%02d:%02d.%02d" % [minutes, seconds, milliseconds]

# Lap button for testing purposes that calls players "IncrementLap"
func _on_lap_button_pressed() -> void:
	if player_node:
		player_node.call("IncrementLap", player_node)
		
# Simple helper (possibly unnecessary) to hide and reset label
func _hide_lap_popup():
	$Stopwatch/LapTimePopupLabel.visible = false
	$Stopwatch/LapTimePopupLabel.modulate.a = 1.0 

# Function for when the race is ended, triggering end screen and stopping the clock
func finished_race():
	stopwatch_running = false
	
	var end_screen = preload("res://Scenes/GameplayScenes/end_screen.tscn").instantiate()
	end_screen.set_lap_times(lap_times, race_time)
	end_screen.set_place(player_node.Place)
	SceneTransition.change_to_node(end_screen)
	
# Called by CarManager's ItemUsed signal whenever any car uses an item
func _on_item_used(user, item: int, target) -> void:
	var item_name = state_name(item)
	if user == player_node and target == player_node:
		show_item_popup("%s boost!" % item_name)
		SoundManager.play_item_boost()
	elif user == player_node:
		show_item_popup("Hit %s place with %s!" % [get_place_suffix(target.Place), item_name])
	elif target == player_node:
		show_item_popup("Hit by %s!" % item_name)
		if item_name == "Slow":
			SoundManager.play_item_slow()
		elif item_name == "Inverted":
			SoundManager.play_item_inverted()

func show_item_popup(text: String) -> void:
	if item_popup_tween:
		item_popup_tween.kill()
	item_popup_label.text = text
	item_popup_label.visible = true
	item_popup_label.modulate.a = 1.0
	
	item_popup_tween = create_tween()
	item_popup_tween.tween_interval(1.2)
	item_popup_tween.tween_property(item_popup_label, "modulate:a", 0.0, 0.5)
	item_popup_tween.tween_callback(Callable(item_popup_label, "hide"))
	
func update_stored_item_icon(item_type: String) -> void:
	if item_type != "Regular" and item_type in item_textures:
		stored_item_icon.texture = item_textures[item_type]
		stored_item_icon.visible = true
	else:
		stored_item_icon.visible = false  # Hide if no stored item
		
func bounce_icon():
	var tween = create_tween()
	tween.set_loops() # loops forever
	tween.tween_property(stored_item_icon, "position:y", stored_item_icon.position.y - 3, 0.7).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_OUT)
	tween.tween_property(stored_item_icon, "position:y", stored_item_icon.position.y + 3, 0.7).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN)

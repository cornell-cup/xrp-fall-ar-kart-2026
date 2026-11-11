extends Control

# First screen of the game: waits for the XRP robot, the camera and a gamepad.
# - each device's card turns green as soon as it's detected
# - after a timeout the missing ones turn red, with RETRY and PLAY OFFLINE
# - PLAY OFFLINE fills in the missing devices (blue) straight away
# - once all three are ready, CONTINUE goes to the title screen

const TITLE_SCENE := "res://Scenes/GameplayScenes/title_screen.tscn"

# Same as the status bar: fusion only sends poses while it hears the XRP
const XRP_TIMEOUT_S := 1.5

# Seconds the loading music takes to hand over to the title music
const MUSIC_CROSSFADE_S := 1

const HEADING_WHITE := Color("#FFFFFF")
const HEADING_RED := Color("#FF8A92")
const HEADING_GREEN := Color("#3DDC84")

const CONNECTING_HEADING := "CONNECTING DEVICES..."
const CONNECTING_SUBTITLE := "Make sure the robot is on and the gamepad is paired"

# Seconds to search before showing which devices weren't found
@export var timeout_duration: float = 15.0

var time_elapsed: float = 0.0
var has_timed_out: bool = false
var is_offline_mode: bool = false
var all_ready: bool = false

@onready var robot_card = $Cards/RobotCard
@onready var camera_card = $Cards/CameraCard
@onready var gamepad_card = $Cards/GamepadCard
@onready var heading: Label = $Heading
@onready var subtitle: Label = $Subtitle
@onready var retry_button: TextureButton = $Buttons/RetryButton
@onready var play_offline_button: TextureButton = $Buttons/PlayOfflineButton
@onready var continue_button: TextureButton = $Buttons/ContinueButton

func _ready() -> void:
	get_window().mode = Window.MODE_FULLSCREEN
	SoundManager.play_loading_music()
	retry_button.pressed.connect(_on_retry_pressed)
	play_offline_button.pressed.connect(_on_play_offline_pressed)
	continue_button.pressed.connect(_on_continue_pressed)
	_show_connecting()


func _process(delta: float) -> void:
	# Live hardware polling, while searching (not timed out, not offline, not done)
	if is_offline_mode or all_ready or has_timed_out:
		return
	_poll_live_hardware()
	if all_ready:
		return
	time_elapsed += delta
	if time_elapsed >= timeout_duration:
		_on_timeout()


func _poll_live_hardware() -> void:
	if _xrp_connected() and robot_card.state == robot_card.State.SEARCHING:
		_set_connected(robot_card)
	if NetworkReceiver.camera_connected() and camera_card.state == camera_card.State.SEARCHING:
		_set_connected(camera_card)
	if not Input.get_connected_joypads().is_empty() and gamepad_card.state == gamepad_card.State.SEARCHING:
		_set_connected(gamepad_card)


func _xrp_connected() -> bool:
	var now := Time.get_ticks_msec() / 1000.0
	return NetworkReceiver.xrp_connected and now - NetworkReceiver.pose_last_received < XRP_TIMEOUT_S


func _show_connecting() -> void:
	_set_heading(CONNECTING_HEADING, HEADING_WHITE, CONNECTING_SUBTITLE)
	retry_button.visible = false
	play_offline_button.visible = true
	continue_button.visible = false


func _on_timeout() -> void:
	has_timed_out = true
	var missing = []
	for card in _cards():
		if not card.is_ready():
			card.set_state(card.State.NOT_FOUND)
			missing.append(card.device_name.to_lower())
	_set_heading("CONNECTION TIMED OUT", HEADING_RED, "Could not find: " + ", ".join(missing))
	retry_button.visible = true


func _on_retry_pressed() -> void:
	has_timed_out = false
	time_elapsed = 0.0
	for card in _cards():
		if card.state == card.State.NOT_FOUND:
			card.set_state(card.State.SEARCHING)
	_show_connecting()


# Offline mode: devices that aren't connected are filled in straight away
func _on_play_offline_pressed() -> void:
	if is_offline_mode or all_ready:
		return
	is_offline_mode = true
	for card in _cards():
		if not card.is_ready():
			card.set_state(card.State.SIMULATED)
	_check_all_ready()


func _set_connected(card) -> void:
	card.set_state(card.State.CONNECTED)
	_check_all_ready()


func _check_all_ready() -> void:
	if all_ready:
		return
	for card in _cards():
		if not card.is_ready():
			return
	all_ready = true
	if is_offline_mode:
		_set_heading("OFFLINE MODE READY!", HEADING_GREEN, "You can connect devices later from the title screen")
	else:
		_set_heading("ALL DEVICES CONNECTED!", HEADING_GREEN, "Ready to race")
	retry_button.visible = false
	play_offline_button.visible = false
	continue_button.visible = true


func _on_continue_pressed() -> void:
	SoundManager.slow_next_crossfade(MUSIC_CROSSFADE_S)
	SceneTransition.change_scene(TITLE_SCENE)


func _set_heading(text: String, color: Color, sub: String) -> void:
	heading.text = text
	heading.add_theme_color_override("font_color", color)
	subtitle.text = sub


func _cards() -> Array:
	return [robot_card, camera_card, gamepad_card]

extends Control

# Spinner node references
@onready var robot_spinner: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Spinner
@onready var camera_spinner: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Spinner
@onready var controller_spinner: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Spinner

# Checkmark node references
@onready var robot_checkmark: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Checkmark
@onready var camera_checkmark: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Checkmark
@onready var controller_checkmark: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Checkmark

# UI labels & buttons
@onready var footer_label: Label = $FooterLabel
@onready var play_offline_button: Button = get_node_or_null("PlayOfflineButton")

# Rotation speed: 0.5 turns per second (180 deg/s)
@export var spin_speed: float = TAU * 0.5

# Manual adjustment in pixels to center the spindle
@export var pivot_adjustment: Vector2 = Vector2(0, -18)

# State management
var is_offline_mode: bool = false
var transition_started: bool = false

func _ready() -> void:
	_reset_indicators()

	# Wait one frame for containers to finish computing layout dimensions
	await get_tree().process_frame
	_recenter_all_pivots()

	for spinner in [robot_spinner, camera_spinner, controller_spinner]:
		if spinner:
			spinner.resized.connect(_recenter_all_pivots)

	# Connect Play Offline button if present
	if play_offline_button and not play_offline_button.pressed.is_connected(_on_play_offline_pressed):
		play_offline_button.pressed.connect(_on_play_offline_pressed)

func _process(delta: float) -> void:
	# 1. Rotate active spinners
	var step = delta * spin_speed
	if robot_spinner and robot_spinner.visible:
		robot_spinner.rotation += step
	if camera_spinner and camera_spinner.visible:
		camera_spinner.rotation += step
	if controller_spinner and controller_spinner.visible:
		controller_spinner.rotation += step

	# 2. Mode A: Live Hardware Polling (only active while NOT in offline mode)
	if not is_offline_mode and not transition_started:
		_poll_live_hardware()

func _recenter_all_pivots() -> void:
	_center_pivot(robot_spinner)
	_center_pivot(camera_spinner)
	_center_pivot(controller_spinner)

func _center_pivot(spinner: TextureRect) -> void:
	if spinner and is_instance_valid(spinner):
		spinner.pivot_offset = (spinner.size / 2.0) + pivot_adjustment

func _reset_indicators() -> void:
	if robot_checkmark: robot_checkmark.visible = false
	if camera_checkmark: camera_checkmark.visible = false
	if controller_checkmark: controller_checkmark.visible = false

	if robot_spinner: robot_spinner.visible = true
	if camera_spinner: camera_spinner.visible = true
	if controller_spinner: controller_spinner.visible = true

	if footer_label:
		footer_label.text = "Verifying Kart Peripherals..."

# Mode A: Checks real-world connectivity via NetworkReceiver and Input API
func _poll_live_hardware() -> void:
	var net = get_node_or_null("/root/NetworkReceiver")
	if net:
		# Robot: true when telemetry packets stream from EKF
		if net.xrp_connected and not robot_checkmark.visible:
			_set_connected(robot_spinner, robot_checkmark)

		# Camera: true when video frames are actively confirmed over TCP
		if (net.get("_video_confirmed") or net.tcp_connected) and not camera_checkmark.visible:
			_set_connected(camera_spinner, camera_checkmark)

		# Controller: true when gamepad detected by NetworkReceiver or Godot Input
		if (net.gamepad_connected or Input.get_connected_joypads().size() > 0) and not controller_checkmark.visible:
			_set_connected(controller_spinner, controller_checkmark)
	else:
		# Fallback check for gamepad if running scene without NetworkReceiver
		if Input.get_connected_joypads().size() > 0 and not controller_checkmark.visible:
			_set_connected(controller_spinner, controller_checkmark)

# Mode B: Triggered when user clicks "Play Offline"
func _on_play_offline_pressed() -> void:
	if is_offline_mode or transition_started:
		return

	is_offline_mode = true
	if play_offline_button:
		play_offline_button.disabled = true

	if footer_label:
		footer_label.text = "Starting Offline Mode..."

	# Trigger staggered mock delays for remaining unconnected devices
	if not robot_checkmark.visible:
		get_tree().create_timer(3.0).timeout.connect(func(): _set_connected(robot_spinner, robot_checkmark))
	if not camera_checkmark.visible:
		get_tree().create_timer(4.0).timeout.connect(func(): _set_connected(camera_spinner, camera_checkmark))
	if not controller_checkmark.visible:
		get_tree().create_timer(5.0).timeout.connect(func(): _set_connected(controller_spinner, controller_checkmark))

# Unified handler: hides spinner, shows checkmark, checks completion
func _set_connected(spinner: TextureRect, checkmark: TextureRect) -> void:
	if spinner: spinner.visible = false
	if checkmark: checkmark.visible = true
	_check_all_connected()

func _check_all_connected() -> void:
	var robot_ready = robot_checkmark and robot_checkmark.visible
	var camera_ready = camera_checkmark and camera_checkmark.visible
	var controller_ready = controller_checkmark and controller_checkmark.visible

	if robot_ready and camera_ready and controller_ready:
		if transition_started:
			return
		transition_started = true

		if footer_label:
			footer_label.text = "All Devices connected!" if not is_offline_mode else "Offline Mode Ready!"

		# Hide the skip button once ready
		if play_offline_button:
			play_offline_button.visible = false

		# Pause for 1.2 seconds so player sees completion, then load title screen
		await get_tree().create_timer(1.2).timeout
		get_tree().change_scene_to_file("res://Scenes/GameplayScenes/title_screen.tscn")

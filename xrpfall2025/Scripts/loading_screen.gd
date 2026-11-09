extends Control

# Spinner references
@onready var robot_spinner: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Spinner
@onready var camera_spinner: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Spinner
@onready var controller_spinner: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Spinner

# Checkmark references
@onready var robot_checkmark: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Checkmark
@onready var camera_checkmark: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Checkmark
@onready var controller_checkmark: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Checkmark

# Footer label
@onready var footer_label: Label = $FooterLabel

# Rotation speed: 0.5 turns per second (180 deg/s)
@export var spin_speed: float = TAU * 0.5

# Manual pivot adjustment working from your setup
@export var pivot_adjustment: Vector2 = Vector2(0, -18)

# Simulation / Mock delays (in seconds)
@export_group("Simulation Delays")
@export var mock_mode: bool = true
@export var robot_delay: float = 3.0
@export var camera_delay: float = 4.0
@export var controller_delay: float = 5.0

func _ready() -> void:
	# Ensure checkmarks start hidden and spinners start visible
	_reset_indicators()

	# Wait one frame for containers to finalize dimensions
	await get_tree().process_frame
	_recenter_all_pivots()

	for spinner in [robot_spinner, camera_spinner, controller_spinner]:
		if spinner:
			spinner.resized.connect(_recenter_all_pivots)

	# Run mock connection sequence if enabled
	if mock_mode:
		_start_simulation()

func _process(delta: float) -> void:
	var step = delta * spin_speed
	# Only rotate spinners that are still actively searching
	if robot_spinner and robot_spinner.visible:
		robot_spinner.rotation += step
	if camera_spinner and camera_spinner.visible:
		camera_spinner.rotation += step
	if controller_spinner and controller_spinner.visible:
		controller_spinner.rotation += step

func _reset_indicators() -> void:
	if robot_checkmark: robot_checkmark.visible = false
	if camera_checkmark: camera_checkmark.visible = false
	if controller_checkmark: controller_checkmark.visible = false

	if robot_spinner: robot_spinner.visible = true
	if camera_spinner: camera_spinner.visible = true
	if controller_spinner: controller_spinner.visible = true

	if footer_label:
		footer_label.text = "Verifying Kart Peripherals..."

func _start_simulation() -> void:
	get_tree().create_timer(robot_delay).timeout.connect(_on_robot_connected)
	get_tree().create_timer(camera_delay).timeout.connect(_on_camera_connected)
	get_tree().create_timer(controller_delay).timeout.connect(_on_controller_connected)

func _on_robot_connected() -> void:
	if robot_spinner: robot_spinner.visible = false
	if robot_checkmark: robot_checkmark.visible = true
	_check_all_connected()

func _on_camera_connected() -> void:
	if camera_spinner: camera_spinner.visible = false
	if camera_checkmark: camera_checkmark.visible = true
	_check_all_connected()

func _on_controller_connected() -> void:
	if controller_spinner: controller_spinner.visible = false
	if controller_checkmark: controller_checkmark.visible = true
	_check_all_connected()

func _check_all_connected() -> void:
	var robot_ready = robot_checkmark and robot_checkmark.visible
	var camera_ready = camera_checkmark and camera_checkmark.visible
	var controller_ready = controller_checkmark and controller_checkmark.visible

	if robot_ready and camera_ready and controller_ready:
		if footer_label:
			footer_label.text = "All Devices Connected!"
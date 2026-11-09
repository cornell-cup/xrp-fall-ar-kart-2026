extends Control

# Spinner node references
@onready var robot_spinner: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Spinner
@onready var camera_spinner: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Spinner
@onready var controller_spinner: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Spinner

# Rotation speed: 0.5 turns per second (180 deg/s) - slow, readable arcade pace
@export var spin_speed: float = TAU * 0.5

# Manual adjustment in pixels to shift the spindle into the exact center hole of the ring
# (Positive Y moves the pivot DOWN; Negative Y moves the pivot UP towards the hole)
@export var pivot_adjustment: Vector2 = Vector2(0, -18)

func _ready() -> void:
	# Wait one frame for containers to finish computing layout dimensions
	await get_tree().process_frame
	_recenter_all_pivots()

	for spinner in [robot_spinner, camera_spinner, controller_spinner]:
		if spinner:
			spinner.resized.connect(_recenter_all_pivots)

func _process(delta: float) -> void:
	# Rotate continuously around the calibrated spindle
	var step = delta * spin_speed
	if robot_spinner and robot_spinner.visible:
		robot_spinner.rotation += step
	if camera_spinner and camera_spinner.visible:
		camera_spinner.rotation += step
	if controller_spinner and controller_spinner.visible:
		controller_spinner.rotation += step

func _recenter_all_pivots() -> void:
	_center_pivot(robot_spinner)
	_center_pivot(camera_spinner)
	_center_pivot(controller_spinner)

func _center_pivot(spinner: TextureRect) -> void:
	if spinner and is_instance_valid(spinner):
		spinner.pivot_offset = (spinner.size / 2.0) + pivot_adjustment
extends Control

# Spinner node references
@onready var robot_spinner: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Spinner
@onready var camera_spinner: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Spinner
@onready var controller_spinner: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Spinner

# Rotation speed: 1.5 full rotations per second (in radians)
const SPIN_SPEED: float = TAU * 1.5

# Fine-tune offset in pixels if the PNG itself has uneven transparent margins
@export var pivot_adjustment: Vector2 = Vector2.ZERO

func _ready() -> void:
	# Wait one frame so containers finish calculating true pixel sizes
	await get_tree().process_frame
	_recenter_all_pivots()

	# Recalculate center automatically if window or container resizes
	for spinner in [robot_spinner, camera_spinner, controller_spinner]:
		if spinner:
			spinner.resized.connect(_recenter_all_pivots)

func _process(delta: float) -> void:
	# Rotate continuously around the true center
	if robot_spinner and robot_spinner.visible:
		robot_spinner.rotation += delta * SPIN_SPEED
	if camera_spinner and camera_spinner.visible:
		camera_spinner.rotation += delta * SPIN_SPEED
	if controller_spinner and controller_spinner.visible:
		controller_spinner.rotation += delta * SPIN_SPEED

func _recenter_all_pivots() -> void:
	_setup_spinner_pivot(robot_spinner)
	_setup_spinner_pivot(camera_spinner)
	_setup_spinner_pivot(controller_spinner)

func _setup_spinner_pivot(spinner: TextureRect) -> void:
	if spinner and is_instance_valid(spinner):
		spinner.pivot_offset = (spinner.size / 2.0) + pivot_adjustment
extends Control

# Spinner node references
@onready var robot_spinner: TextureRect = $IconsHBox/RobotColumn/IndicatorAnchor/Spinner
@onready var camera_spinner: TextureRect = $IconsHBox/CameraColumn/IndicatorAnchor/Spinner
@onready var controller_spinner: TextureRect = $IconsHBox/ControllerColumn/IndicatorAnchor/Spinner

# Rotation speed: 1.5 full rotations per second (in radians)
const SPIN_SPEED: float = TAU * 1.5

func _ready() -> void:
	# Center the pivot on each spinner so they rotate smoothly without wobbling
	_setup_spinner_pivot(robot_spinner)
	_setup_spinner_pivot(camera_spinner)
	_setup_spinner_pivot(controller_spinner)

func _process(delta: float) -> void:
	# Continuously rotate each spinner
	robot_spinner.rotation += delta * SPIN_SPEED
	camera_spinner.rotation += delta * SPIN_SPEED
	controller_spinner.rotation += delta * SPIN_SPEED

func _setup_spinner_pivot(spinner: TextureRect) -> void:
	if spinner:
		spinner.pivot_offset = spinner.size / 2.0
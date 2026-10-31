extends Node

@export var click_sfx: AudioStream = preload("res://Assets/Sound/SFX/YOUR_CLICK_FILE.mp3")
@export var race_start_sfx: AudioStream = preload("res://Assets/Sound/SFX/YOUR_RACE_FILE.mp3")
@export var match_finish_sfx: AudioStream = preload("res://Assets/Sound/SFX/YOUR_FINISH_FILE.mp3")

var _click_player: AudioStreamPlayer
var _event_player: AudioStreamPlayer

func _ready() -> void:
	_click_player = AudioStreamPlayer.new()
	_click_player.bus = "Master"
	add_child(_click_player)
	
	_event_player = AudioStreamPlayer.new()
	_event_player.bus = "Master"
	add_child(_event_player)
	
	get_tree().node_added.connect(_on_node_added)

func _on_node_added(node: Node) -> void:
	if node is BaseButton:
		# If a button has custom sound behavior, skip the generic click
		if node.is_in_group("custom_sfx"):
			return
		if not node.pressed.is_connected(play_click):
			node.pressed.connect(play_click)

func play_click() -> void:
	if click_sfx != null:
		_click_player.stream = click_sfx
		_click_player.play()

func play_race_start() -> void:
	if race_start_sfx != null:
		_event_player.stream = race_start_sfx
		_event_player.play()

func play_match_finish() -> void:
	if match_finish_sfx != null:
		_event_player.stream = match_finish_sfx
		_event_player.play()

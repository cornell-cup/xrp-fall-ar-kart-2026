extends Node

@export var click_sfx: AudioStream = preload("res://Assets/Sound/SFX/UI-Click.mp3")

var _player: AudioStreamPlayer

func _ready() -> void:
	_player = AudioStreamPlayer.new()
	_player.bus = "Master"
	add_child(_player)
	
	# Listen for every node added to the game
	get_tree().node_added.connect(_on_node_added)

func _on_node_added(node: Node) -> void:
	# Automatically wire click audio to every Button and TextureButton
	if node is BaseButton:
		if not node.pressed.is_connected(play_click):
			node.pressed.connect(play_click)

func play_click() -> void:
	if click_sfx != null:
		_player.stream = click_sfx
		_player.play()

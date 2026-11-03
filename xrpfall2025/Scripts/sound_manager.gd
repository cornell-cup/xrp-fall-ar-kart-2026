extends Node

@export var click_sfx: AudioStream = preload("res://Assets/Sound/SFX/UIClick.mp3")
@export var race_start_sfx: AudioStream = preload("res://Assets/Sound/SFX/RaceButtonClick.mp3")
@export var match_finish_sfx: AudioStream = preload("res://Assets/Sound/SFX/GameComplete.mp3")

# Background Music
@export var menu_music: AudioStream = preload("res://Assets/Sound/BackgroundMusic/Option_1_Menu_maksymmalko-game-game-music-581017-OOG.ogg")
@export var race_music: AudioStream = preload("res://Assets/Sound/BackgroundMusic/Option_1_Race_sub_clair-game-music-628077-OOG.ogg")

var _click_player: AudioStreamPlayer
var _event_player: AudioStreamPlayer
var _music_player: AudioStreamPlayer

func _ready() -> void:
	_click_player = AudioStreamPlayer.new()
	_click_player.bus = "Master"
	add_child(_click_player)
	
	_event_player = AudioStreamPlayer.new()
	_event_player.bus = "Master"
	add_child(_event_player)
	
	_music_player = AudioStreamPlayer.new()
	_music_player.bus = "Master"
	add_child(_music_player)
	
	get_tree().node_added.connect(_on_node_added)

func _on_node_added(node: Node) -> void:
	if node is BaseButton:
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
	_music_player.stop() # Stops race music so GameComplete SFX Activates
	if match_finish_sfx != null:
		_event_player.stream = match_finish_sfx
		_event_player.play()

func play_menu_music() -> void:
	if menu_music == null:
		return
	# If this track is already playing, do NOT restart it from second 0!
	if _music_player.stream == menu_music and _music_player.playing:
		return
	_music_player.stream = menu_music
	_music_player.play()

func play_race_music() -> void:
	if race_music == null:
		return
	if _music_player.stream == race_music and _music_player.playing:
		return
	_music_player.stream = race_music
	_music_player.play()

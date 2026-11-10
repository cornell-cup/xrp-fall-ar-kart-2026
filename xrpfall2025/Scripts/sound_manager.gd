extends Node

@export var click_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/ui_press.wav")
# For buttons in the "back_sfx" group (going back to a previous screen)
@export var back_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/ui_back.wav")
@export var hover_sfx: AudioStream = preload("res://Assets/Sound/SFX/UI/hover.wav")
# Buttons in the "confirm_sfx" group (picking a kart or map)
@export var race_start_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/ui_confirm.wav")
@export var match_finish_sfx: AudioStream = preload("res://Assets/Sound/SFX/GameComplete.mp3")

# In-race sounds
@export var countdown_beep_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/countdown_beep.wav")
@export var countdown_go_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/countdown_go.wav")
@export var lap_complete_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/lap_complete.wav")
@export var final_lap_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/final_lap.wav")
@export var off_track_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/off_track.wav")
@export var item_box_break_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/item_box_break.wav")
@export var item_boost_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/item_boost.wav")
@export var item_slow_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/item_slow.wav")
@export var item_inverted_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/item_inverted.wav")
# For things the player can't do (not used yet)
@export var error_sfx: AudioStream = preload("res://Assets/Sound/SFX/Kart/ui_error.wav")

# Background Music (all set to loop in their import settings)
@export var menu_music: AudioStream = preload("res://Assets/Sound/BackgroundMusic/KARTS.mp3")
@export var select_music: AudioStream = preload("res://Assets/Sound/BackgroundMusic/Choose_Your_Racer.mp3")
# Each race plays one of these, picked at random
@export var race_music: Array[AudioStream] = [
	preload("res://Assets/Sound/BackgroundMusic/Flowey_Speedway.mp3"),
	preload("res://Assets/Sound/BackgroundMusic/Highway_Hustle.mp3"),
	preload("res://Assets/Sound/BackgroundMusic/Stadium_64.mp3"),
	preload("res://Assets/Sound/BackgroundMusic/Rainbow_Way.mp3"),
	preload("res://Assets/Sound/BackgroundMusic/Bouncing_Pyramids.mp3"),
	preload("res://Assets/Sound/BackgroundMusic/Retro_Roundabout.mp3"),
]
# Plays over the pause menu while the race song waits
@export var pause_music: AudioStream = preload("res://Assets/Sound/BackgroundMusic/Ghost_Trials.mp3")
@export var results_music: AudioStream = preload("res://Assets/Sound/BackgroundMusic/Race_Results.mp3")

# How long one song takes to fade into the next, in seconds
const CROSSFADE_S := 1.0
# Switching between the race song and the pause music
const PAUSE_FADE_S := 0.4
# Fading out the race music under the finish fanfare is quicker
const FINISH_FADE_S := 0.4
# The results music starts this long before the fanfare ends, over its quiet
# ring-out (below -30 dB for its last ~1.1 s), so the two run together without a pause
const FANFARE_LEAD_S := 0.8
# The pause music sits under the other music's volume (-6 dB is about half as loud)
const PAUSE_MUSIC_DB := -6.0
# How many in-race sounds can play over each other
const SFX_VOICES := 4
const SILENT_DB := -60.0

var _click_player: AudioStreamPlayer
# Separate, so a press doesn't cut off the hover sound or the other way round
var _hover_player: AudioStreamPlayer
var _event_player: AudioStreamPlayer
var _sfx_players: Array[AudioStreamPlayer] = []
var _next_sfx := 0
# Two music players, so the old song can fade out while the new one fades in
var _music_players: Array[AudioStreamPlayer] = []
var _music_tweens: Array = [null, null]
var _active := 0
var _last_race_music: AudioStream = null
var _pause_player: AudioStreamPlayer
var _pause_tween: Tween
# Bumped on every music change, so a song queued behind the finish fanfare is
# dropped if the player has already moved on to another screen
var _music_request := 0

func _ready() -> void:
	# keeps running while the game is paused, for the pause menu's music and fades
	process_mode = Node.PROCESS_MODE_ALWAYS

	_click_player = AudioStreamPlayer.new()
	_click_player.bus = "Master"
	add_child(_click_player)

	_hover_player = AudioStreamPlayer.new()
	_hover_player.bus = "Master"
	_hover_player.volume_db = linear_to_db(0.25)   # half as loud as the other sounds
	add_child(_hover_player)

	_event_player = AudioStreamPlayer.new()
	_event_player.bus = "Master"
	add_child(_event_player)

	for i in SFX_VOICES:
		var player = AudioStreamPlayer.new()
		player.bus = "Master"
		add_child(player)
		_sfx_players.append(player)

	for i in 2:
		var player = AudioStreamPlayer.new()
		player.bus = "Master"
		add_child(player)
		_music_players.append(player)

	_pause_player = AudioStreamPlayer.new()
	_pause_player.bus = "Master"
	add_child(_pause_player)

	get_tree().node_added.connect(_on_node_added)

func _on_node_added(node: Node) -> void:
	if node is BaseButton:
		if node.is_in_group("custom_sfx"):
			return
		var press_sound = play_click
		if node.is_in_group("back_sfx"):
			press_sound = play_back
		elif node.is_in_group("confirm_sfx"):
			press_sound = play_race_start
		if not node.pressed.is_connected(press_sound):
			node.pressed.connect(press_sound)

func play_click() -> void:
	if click_sfx != null:
		_click_player.stream = click_sfx
		_click_player.play()

func play_back() -> void:
	if back_sfx != null:
		_click_player.stream = back_sfx
		_click_player.play()

# Only the main menu buttons hover with a sound, see hover_button.gd
func play_hover() -> void:
	if hover_sfx != null:
		_hover_player.stream = hover_sfx
		_hover_player.play()

func play_race_start() -> void:
	if race_start_sfx != null:
		_event_player.stream = race_start_sfx
		_event_player.play()

# In-race sounds take turns on a few players, so one doesn't cut another off.
# pitch_jitter varies the pitch a little, for sounds heard over and over
func play_sfx(stream: AudioStream, pitch_jitter := false) -> void:
	if stream == null:
		return
	var player = _sfx_players[_next_sfx]
	_next_sfx = (_next_sfx + 1) % _sfx_players.size()
	player.stream = stream
	player.pitch_scale = randf_range(0.92, 1.08) if pitch_jitter else 1.0
	player.play()

func play_countdown_beep() -> void:
	play_sfx(countdown_beep_sfx)

func play_countdown_go() -> void:
	play_sfx(countdown_go_sfx)

func play_lap_complete() -> void:
	play_sfx(lap_complete_sfx)

func play_final_lap() -> void:
	play_sfx(final_lap_sfx)

func play_off_track() -> void:
	play_sfx(off_track_sfx)

# Driving through an item box. Called from Item.cs
func play_item_pickup() -> void:
	play_sfx(item_box_break_sfx, true)

func play_item_boost() -> void:
	play_sfx(item_boost_sfx)

func play_item_slow() -> void:
	play_sfx(item_slow_sfx)

func play_item_inverted() -> void:
	play_sfx(item_inverted_sfx)

func play_error() -> void:
	play_sfx(error_sfx)

# Race music fades out under the finish fanfare, then the results music fades in
func play_match_finish() -> void:
	_music_request += 1
	var request = _music_request
	_fade(_active, SILENT_DB, FINISH_FADE_S, true)
	if match_finish_sfx != null:
		_event_player.stream = match_finish_sfx
		_event_player.play()
		# go by the fanfare's own playback position, so the handover stays in sync
		var handover = maxf(match_finish_sfx.get_length() - FANFARE_LEAD_S, 0.0)
		while _event_player.playing and _event_player.get_playback_position() < handover:
			await get_tree().process_frame
	# straight in at full volume, so the track's opening isn't lost to a fade
	if request == _music_request:
		_play_music(results_music, 0.0)

func play_menu_music() -> void:
	_play_music(menu_music)

func play_select_music() -> void:
	_play_music(select_music)

# Fades out whatever's playing, e.g. the kart select music while a race counts down
func stop_music() -> void:
	_music_request += 1
	_stop_pause_music()
	_fade(_active, SILENT_DB, CROSSFADE_S, true)

# Called when the race countdown ends; the track starts at full volume, as nothing
# else is playing by then
func play_race_music() -> void:
	if race_music.is_empty():
		return
	# a different song from last race when there's a choice
	var choices = race_music.filter(func(s): return s != _last_race_music)
	if choices.is_empty():
		choices = race_music
	_last_race_music = choices.pick_random()
	_play_music(_last_race_music, 0.0)

func play_results_music() -> void:
	_play_music(results_music)

# The race song fades down and holds its place while the pause music fades in
func pause_race_music() -> void:
	var race = _music_players[_active]
	if _music_tweens[_active]:
		_music_tweens[_active].kill()
	if race.playing:
		var tween = _tween_volume(race, SILENT_DB, PAUSE_FADE_S)
		tween.tween_callback(func(): race.stream_paused = true)
		_music_tweens[_active] = tween
	if pause_music != null:
		if _pause_tween:
			_pause_tween.kill()
		_pause_player.stream = pause_music
		_pause_player.volume_db = SILENT_DB
		_pause_player.play()
		_pause_tween = _tween_volume(_pause_player, PAUSE_MUSIC_DB, PAUSE_FADE_S)

# ...and picks up again where it left off
func resume_race_music() -> void:
	_stop_pause_music()
	var race = _music_players[_active]
	if _music_tweens[_active]:
		_music_tweens[_active].kill()
	if race.stream != null and (race.stream_paused or race.playing):
		race.stream_paused = false
		_music_tweens[_active] = _tween_volume(race, 0.0, PAUSE_FADE_S)

func _stop_pause_music() -> void:
	if _pause_tween:
		_pause_tween.kill()
	if _pause_player.playing:
		_pause_tween = _tween_volume(_pause_player, SILENT_DB, PAUSE_FADE_S)
		_pause_tween.tween_callback(_pause_player.stop)

# fade_in_s: how long the new song takes to come in (0 = starts at full volume)
func _play_music(stream: AudioStream, fade_in_s := CROSSFADE_S) -> void:
	if stream == null:
		return
	_music_request += 1
	_stop_pause_music()
	var current = _music_players[_active]
	# a song held for the pause menu isn't coming back (e.g. Home from the pause menu)
	if current.stream_paused:
		current.stream_paused = false
		current.stop()
	# If this track is already playing, do NOT restart it from second 0!
	if current.stream == stream and current.playing:
		_fade(_active, 0.0, CROSSFADE_S, false)
		return
	# fade the current song out on its player while the new one fades in on the other
	_fade(_active, SILENT_DB, CROSSFADE_S, true)
	_active = 1 - _active
	var next = _music_players[_active]
	next.stream = stream
	if fade_in_s > 0.0:
		next.volume_db = SILENT_DB
		next.play()
		_fade(_active, 0.0, fade_in_s, false)
	else:
		next.volume_db = 0.0
		next.play()

func _fade(index: int, to_db: float, seconds: float, stop_after: bool) -> void:
	var player = _music_players[index]
	if _music_tweens[index]:
		_music_tweens[index].kill()
	if not player.playing:
		return
	var tween = _tween_volume(player, to_db, seconds)
	if stop_after:
		tween.tween_callback(player.stop)
	_music_tweens[index] = tween

# Fades a player's volume, in linear volume rather than decibels so it sounds even
func _tween_volume(player: AudioStreamPlayer, to_db: float, seconds: float) -> Tween:
	var tween = create_tween()
	tween.tween_method(func(v): player.volume_db = linear_to_db(v),
		db_to_linear(player.volume_db), db_to_linear(to_db), seconds)
	return tween

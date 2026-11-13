extends Node

# real_track.tscn is the physical track from Car Map.pdf; basic_track.tscn is the older test track
const DEFAULT_LEVEL_PATH : String = "res://real_track.tscn"

# The track the race loads. The kart select screen points this at a Track Builder save
var level_path : String = DEFAULT_LEVEL_PATH

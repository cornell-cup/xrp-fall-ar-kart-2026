extends PanelContainer

# Menu bar showing what the race needs: the XRP (pose data from laptop_fusion),
# a controller plugged into this laptop, and the Pi camera's video

# Fusion only sends poses while it's hearing the XRP, so a gap this long means
# the robot is gone (NetworkReceiver.xrp_connected waits 8 s, too slow for this)
const XRP_TIMEOUT_S := 1.5

const DOT_CONNECTED = preload("res://Assets/ar_kart_ui/status/dot_connected@2x.png")
const DOT_DISCONNECTED = preload("res://Assets/ar_kart_ui/status/dot_disconnected@2x.png")

@onready var xrp_dot: TextureRect = $Row/XrpDot
@onready var controller_dot: TextureRect = $Row/ControllerDot
@onready var camera_dot: TextureRect = $Row/CameraDot


func _process(_delta: float) -> void:
	xrp_dot.texture = DOT_CONNECTED if _xrp_connected() else DOT_DISCONNECTED
	controller_dot.texture = DOT_CONNECTED if _controller_connected() else DOT_DISCONNECTED
	camera_dot.texture = DOT_CONNECTED if _camera_connected() else DOT_DISCONNECTED


func _xrp_connected() -> bool:
	var now := Time.get_ticks_msec() / 1000.0
	return NetworkReceiver.xrp_connected and now - NetworkReceiver.pose_last_received < XRP_TIMEOUT_S


func _controller_connected() -> bool:
	return NetworkReceiver.controller_connected()


# The same check that switches AR mode on, so the dot and the mode always agree
func _camera_connected() -> bool:
	return NetworkReceiver.camera_connected()

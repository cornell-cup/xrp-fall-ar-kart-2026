extends CanvasLayer

@onready var video_rect: TextureRect = $VideoRect

func _ready() -> void:
	layer = 10  # on top of everything
	
	var window_size = Vector2(1152, 648)
	var pip_size = Vector2(400, 300)
	
	video_rect.position = Vector2(15, window_size.y - pip_size.y - 15)
	video_rect.size = pip_size
	video_rect.flip_v = true
	video_rect.flip_h = true
	video_rect.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	video_rect.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED

func _process(_delta: float) -> void:
	video_rect.visible = not NetworkReceiver.ar_enabled
	if NetworkReceiver.video_texture != null:
		video_rect.texture = NetworkReceiver.video_texture

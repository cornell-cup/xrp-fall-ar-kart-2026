extends Node

# ============================================================
# Combined network receiver — fusion + AR UI.
#
# Launches the bundled laptop_fusion executable on startup. From it:
#   - UDP 6000: fused robot pose JSON {x, y, angle}
#   - TCP 6001: raw RGB video frames
#   - UDP 6002: ar_tags JSON   (NEW — for AR overlays)
#
# Sends to Python:
#   - UDP 4003 (localhost): gamepad packets, Python forwards to XRP
#
# AR mode: at startup, prompts "Enable AR Camera?". If yes, the live
# camera feed becomes the world backdrop and labels float over each
# detected AprilTag. The kart pose / gamepad / fusion logic runs the
# same way regardless.
#
# Variable names xrp_x/y/angle/connected/trail kept for backwards-compat
# with Player.cs.
# ============================================================

# Passed to laptop_fusion, which tries it first and then falls back to
# cornellcup2.local / 10.42.0.1 / 192.168.4.95 on its own. So the kart
# works on the Pi hotspot or CornellCup-Web without changing this.
const PI_IP: String = "cornellcup2.local"

const POSE_PORT: int = 6000
const VIDEO_PORT: int = 6001
const AR_TAGS_PORT: int = 6002
const GAMEPAD_RELAY_PORT: int = 4003
const GAMEPAD_RELAY_HOST: String = "127.0.0.1"

# ---- Fused pose ----
var udp_pose_in: PacketPeerUDP = PacketPeerUDP.new()
var pose_last_received: float = 0.0
const POSE_TIMEOUT: float = 8.0

# ---- Gamepad ----
var udp_gamepad: PacketPeerUDP = PacketPeerUDP.new()
var gamepad_connected: bool = false
const GAMEPAD_SEND_HZ: float = 20.0
var gamepad_send_timer: float = 0.0
const GAMEPAD_HEADER: int = 0x55
const DEADZONE: float = 0.08

# ---- Video TCP ----
var tcp_client: StreamPeerTCP = StreamPeerTCP.new()
var tcp_connected: bool = false
var tcp_buffer: PackedByteArray = PackedByteArray()
var tcp_retry_timer: float = 0.0

# ---- AR tags UDP ----
var udp_ar_tags: PacketPeerUDP = PacketPeerUDP.new()
var ar_frame_w: int = 640
var ar_frame_h: int = 480
const AR_LABEL_PIXEL_SIZE: float = 0.2

# ---- Fusion process ----
var fusion_pid: int = -1

# ---- Latest video frame ----
var video_texture: ImageTexture = null
var _video_confirmed: bool = false

# ---- Robot state (legacy names) ----
var xrp_x: float = 0.0
var xrp_y: float = 0.0
var xrp_angle: float = 0.0
var xrp_connected: bool = false
var xrp_trail: Array[Vector2] = []
const TRAIL_MAX: int = 300

# ---- AR mode ----
var ar_enabled: bool = false
var ar_backdrop_mesh: MeshInstance3D = null
var ar_camera: Camera3D = null
# Size of the backdrop quad, set so it exactly covers the real camera's view
var ar_backdrop_size: Vector2 = Vector2.ZERO
# How far in front of the kart camera the backdrop sits. Anything virtual further
# away than this is hidden behind the video, so keep it past the far side of the track
const AR_BACKDROP_DIST: float = 1000.0
# Map of tag_id -> Label3D overlay node.
var ar_tag_labels: Dictionary = {}
var _ar_attach_timer: float = 0.0

# Real camera intrinsics in pixels, at the resolution they were calibrated at.
# Must match Camera_params in laptop_fusion.py
const CAM_FX: float = 1268.28550
const CAM_FY: float = 1254.14697
const CAM_CX: float = 361.108041
const CAM_CY: float = 297.975274
const CAM_CALIB_W: float = 640.0
const CAM_CALIB_H: float = 480.0
# The Pi camera is mounted upside down (the picture-in-picture preview flips it
# both ways too), so the AR backdrop rotates the feed 180°
const CAM_UPSIDE_DOWN: bool = true
# How far the real camera is tilted up from level (negative = down). The 25° is
# _theta_mount in laptop_fusion.py; it points up so it can see the tags on the gates
const CAM_TILT_UP_DEG: float = 25.0

# In AR mode the real floor and gates are in the video, so these virtual pieces
# are only drawn on the minimap (render layer 2) and not on the kart camera (layer 1)
const AR_HIDDEN_NODES: Array[String] = ["Ground", "TrackLoader", "Finishline"]
const MINIMAP_ONLY_LAYER: int = 2

# ---- Debug ----
var debug_timer: float = 0.0
var _frames_dropped_this_period: int = 0
var _frames_shown_this_period: int = 0


# ============================================================
# Setup
# ============================================================

func _get_base_dir() -> String:
	if OS.has_feature("editor"):
		return ProjectSettings.globalize_path("res://")
	else:
		return OS.get_executable_path().get_base_dir()


func _get_fusion_path() -> String:
	var base = _get_base_dir()
	if OS.get_name() == "Windows":
		return base.path_join("laptop_fusion.exe")
	else:
		return base.path_join("laptop_fusion")


func _ready() -> void:
	# Pose listener
	var err_pose = udp_pose_in.bind(POSE_PORT, "0.0.0.0")
	if err_pose != OK:
		push_error("[NetworkReceiver] Failed to bind pose port %d (error %d)" % [POSE_PORT, err_pose])
	else:
		print("[NetworkReceiver] Listening for fused pose on UDP %d" % POSE_PORT)

	# AR tags listener
	var err_ar = udp_ar_tags.bind(AR_TAGS_PORT, "0.0.0.0")
	if err_ar != OK:
		push_error("[NetworkReceiver] Failed to bind AR tags port %d (error %d)" % [AR_TAGS_PORT, err_ar])
	else:
		print("[NetworkReceiver] Listening for AR tags on UDP %d" % AR_TAGS_PORT)

	# Gamepad output
	udp_gamepad.set_dest_address(GAMEPAD_RELAY_HOST, GAMEPAD_RELAY_PORT)
	print("[NetworkReceiver] Gamepad relayed via %s:%d at %d Hz" % [
		GAMEPAD_RELAY_HOST, GAMEPAD_RELAY_PORT, int(GAMEPAD_SEND_HZ)])

	Input.joy_connection_changed.connect(_on_joy_connection_changed)
	var pads = Input.get_connected_joypads()
	if not pads.is_empty():
		print("[NetworkReceiver] Controller already connected: %s" % Input.get_joy_name(0))

	_start_fusion()
	await get_tree().create_timer(4.0).timeout
	_connect_video_tcp()

	# Show the AR popup after fusion + video are wired up
	call_deferred("_show_ar_dialog")


func _process(_delta: float) -> void:
	_process_tcp_status()
	_process_video()
	_process_pose()
	_process_ar_tags()
	_process_gamepad_send(_delta)

	# AR mode attaches to the kart camera of whichever race scene is loaded.
	# Scene changes (title screen -> race, restart) free the old backdrop, so reattach
	if ar_enabled:
		if not is_instance_valid(ar_backdrop_mesh):
			_ar_attach_timer += _delta
			if _ar_attach_timer > 0.5:
				_ar_attach_timer = 0.0
				_try_attach_ar()
		else:
			_update_ar_camera_fov()

	if not tcp_connected:
		tcp_retry_timer += _delta
		if tcp_retry_timer > 3.0:
			tcp_retry_timer = 0.0
			tcp_client.disconnect_from_host()
			tcp_client = StreamPeerTCP.new()
			var err = tcp_client.connect_to_host("127.0.0.1", VIDEO_PORT)
			if err != OK:
				print("[NetworkReceiver] TCP connect attempt failed (error %d)" % err)
			else:
				print("[NetworkReceiver] Retrying TCP connection to %d..." % VIDEO_PORT)

	debug_timer += _delta
	if debug_timer > 5.0:
		debug_timer = 0.0
		var tcp_status = tcp_client.get_status()
		var tcp_status_name = "NONE"
		match tcp_status:
			StreamPeerTCP.STATUS_NONE: tcp_status_name = "NONE"
			StreamPeerTCP.STATUS_CONNECTING: tcp_status_name = "CONNECTING"
			StreamPeerTCP.STATUS_CONNECTED: tcp_status_name = "CONNECTED"
			StreamPeerTCP.STATUS_ERROR: tcp_status_name = "ERROR"
		print("[NetworkReceiver] Status - TCP: %s | Pose: %s | Gamepad: %s | Video: %s | AR: %s | Frames: %d/%d" % [
			tcp_status_name,
			"active" if xrp_connected else "waiting",
			"connected" if gamepad_connected else "none",
			"yes" if _video_confirmed else "no",
			"on" if ar_enabled else "off",
			_frames_shown_this_period,
			_frames_dropped_this_period,
		])
		_frames_shown_this_period = 0
		_frames_dropped_this_period = 0


# ============================================================
# Fusion process launcher
# ============================================================

func _start_fusion() -> void:
	_kill_old_fusion()
	await get_tree().create_timer(1.0).timeout
	var fusion_path = _get_fusion_path()
	if not FileAccess.file_exists(fusion_path):
		push_error("[NetworkReceiver] Fusion binary not found: %s" % fusion_path)
		return
	if OS.get_name() == "macOS":
		OS.execute("xattr", ["-rd", "com.apple.quarantine", fusion_path])
		OS.execute("chmod", ["+x", fusion_path])
	fusion_pid = OS.create_process(fusion_path, [PI_IP])
	if fusion_pid > 0:
		print("[NetworkReceiver] Fusion started (PID: %d)" % fusion_pid)
	else:
		push_error("[NetworkReceiver] Failed to start fusion binary")


func _kill_old_fusion() -> void:
	if OS.get_name() == "Windows":
		OS.execute("taskkill", ["/F", "/IM", "laptop_fusion.exe"])
	else:
		OS.execute("pkill", ["-f", "laptop_fusion"])


func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_CLOSE_REQUEST:
		_cleanup()
		get_tree().quit()


func _cleanup() -> void:
	if fusion_pid > 0:
		OS.kill(fusion_pid)
		print("[NetworkReceiver] Fusion stopped (PID: %d)" % fusion_pid)
		fusion_pid = -1
	_kill_old_fusion()
	tcp_client.disconnect_from_host()
	tcp_connected = false
	udp_pose_in.close()
	udp_ar_tags.close()
	udp_gamepad.close()


# ============================================================
# AR mode setup
# ============================================================

func _show_ar_dialog() -> void:
	var dialog = ConfirmationDialog.new()
	dialog.title = "AR Camera"
	dialog.dialog_text = "Enable AR Camera?\n\nThis will use the kart's live camera as the game background\nwith floating labels over detected tags."
	dialog.ok_button_text = "Yes, use camera"
	dialog.cancel_button_text = "No, normal mode"
	dialog.size = Vector2i(420, 170)
	dialog.initial_position = Window.WINDOW_INITIAL_POSITION_CENTER_MAIN_WINDOW_SCREEN
	dialog.confirmed.connect(_on_ar_yes)
	dialog.canceled.connect(_on_ar_no)

	var canvas = CanvasLayer.new()
	canvas.layer = 100
	canvas.name = "ARDialogLayer"
	add_child(canvas)
	canvas.add_child(dialog)
	dialog.popup_centered()


func _on_ar_yes() -> void:
	ar_enabled = true
	var dl = get_node_or_null("ARDialogLayer")
	if dl: dl.queue_free()
	print("[AR] AR mode on, waiting for a race to attach to")
	_try_attach_ar()


func _on_ar_no() -> void:
	ar_enabled = false
	var dl = get_node_or_null("ARDialogLayer")
	if dl: dl.queue_free()
	print("[AR] Normal mode")


# Sets up AR on the current race scene if it has a kart camera. Returns false
# if there isn't one yet (e.g. still on the title screen); _process retries
func _try_attach_ar() -> bool:
	var cam = _find_node_by_name(get_tree().root, "fpsCamera") as Camera3D
	if cam == null:
		return false
	ar_camera = cam
	ar_tag_labels.clear()  # old labels were freed along with the old backdrop
	_match_real_camera(cam)
	_setup_camera_backdrop(cam)
	_update_ar_camera_fov()
	_hide_world()
	print("[AR] AR mode active on %s" % cam.get_path())
	return true


# Where the real camera's optical center lands in the displayed image, 0..1 across and down
func _principal_point() -> Vector2:
	var pp = Vector2(CAM_CX / CAM_CALIB_W, CAM_CY / CAM_CALIB_H)
	return Vector2.ONE - pp if CAM_UPSIDE_DOWN else pp


# Tilts the kart camera to match the real one, keeping its position on the kart model
func _match_real_camera(cam: Camera3D) -> void:
	var forward = -cam.global_transform.basis.z
	forward.y = 0
	forward = forward.normalized()
	var tilt = deg_to_rad(CAM_TILT_UP_DEG)
	var look = forward * cos(tilt) + Vector3.UP * sin(tilt)
	cam.global_basis = Basis.looking_at(look, Vector3.UP)


func _setup_camera_backdrop(cam: Camera3D) -> void:
	# Size the quad to the real camera's view at AR_BACKDROP_DIST, so a pixel
	# in the video lines up with the same direction in the game
	ar_backdrop_size = Vector2(
		AR_BACKDROP_DIST * CAM_CALIB_W / CAM_FX,
		AR_BACKDROP_DIST * CAM_CALIB_H / CAM_FY)
	var mesh = QuadMesh.new()
	mesh.size = ar_backdrop_size

	# Use the existing live video texture if it's been created already,
	# or a placeholder if not (will get updated on first frame).
	if video_texture == null:
		var img = Image.create(ar_frame_w, ar_frame_h, false, Image.FORMAT_RGB8)
		img.fill(Color(0.15, 0.15, 0.25))
		video_texture = ImageTexture.create_from_image(img)

	var mat = StandardMaterial3D.new()
	mat.albedo_texture = video_texture
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	if CAM_UPSIDE_DOWN:
		# uv -> 1 - uv rotates the picture 180°
		mat.uv1_scale = Vector3(-1, -1, 1)
		mat.uv1_offset = Vector3(1, 1, 0)

	ar_backdrop_mesh = MeshInstance3D.new()
	ar_backdrop_mesh.mesh = mesh
	ar_backdrop_mesh.material_override = mat
	ar_backdrop_mesh.name = "ARCameraBackdrop"
	cam.add_child(ar_backdrop_mesh)
	# Shift the quad so the camera's optical center sits straight ahead of the game camera
	var pp = _principal_point()
	ar_backdrop_mesh.position = Vector3(
		(0.5 - pp.x) * ar_backdrop_size.x,
		(pp.y - 0.5) * ar_backdrop_size.y,
		-AR_BACKDROP_DIST)


# Zooms the kart camera so the screen is filled by the video with no gaps,
# using the same field of view as the real camera so virtual objects line up
func _update_ar_camera_fov() -> void:
	if not is_instance_valid(ar_camera):
		return
	var pp = _principal_point()
	# the widest the view can be on each side before running off the edge of the video
	var half_w = min(pp.x, 1.0 - pp.x) * ar_backdrop_size.x
	var half_h = min(pp.y, 1.0 - pp.y) * ar_backdrop_size.y
	var screen = get_viewport().get_visible_rect().size
	half_w = min(half_w, half_h * screen.x / screen.y)
	ar_camera.keep_aspect = Camera3D.KEEP_WIDTH
	ar_camera.fov = rad_to_deg(2.0 * atan(half_w / AR_BACKDROP_DIST))


func _hide_world() -> void:
	var scene = get_tree().current_scene
	if scene == null:
		return
	for node_name in AR_HIDDEN_NODES:
		var node = scene.get_node_or_null(node_name)
		if node: _move_to_minimap_layer(node)


func _move_to_minimap_layer(node: Node) -> void:
	if node is VisualInstance3D:
		node.layers = MINIMAP_ONLY_LAYER
	for child in node.get_children():
		_move_to_minimap_layer(child)


# ============================================================
# AR overlay updates from incoming tag packets
# ============================================================

func _process_ar_tags() -> void:
	while udp_ar_tags.get_available_packet_count() > 0:
		var pkt = udp_ar_tags.get_packet()
		var msg = pkt.get_string_from_utf8().strip_edges()
		var json = JSON.new()
		var err = json.parse(msg)
		if err != OK:
			continue
		var data = json.data
		if not (data is Dictionary): continue
		if str(data.get("type", "")) != "ar_tags": continue

		ar_frame_w = int(data.get("frame_w", ar_frame_w))
		ar_frame_h = int(data.get("frame_h", ar_frame_h))

		var tags: Array = data.get("tags", [])
		_update_ar_labels(tags)


func _update_ar_labels(tags: Array) -> void:
	if not ar_enabled or not is_instance_valid(ar_backdrop_mesh):
		# Even if AR isn't active visually, still consume the packets
		return

	# Track which IDs are present this frame so we can hide stale ones.
	var seen_ids := {}

	for tag in tags:
		if not (tag is Dictionary): continue
		var id := int(tag.get("id", -1))
		if id < 0: continue
		seen_ids[id] = true

		var corners: Array = tag.get("corners", [])
		var center: Array = tag.get("center", [0.0, 0.0])
		var name: String = str(tag.get("name", "TAG %d" % id))

		var label: Label3D = ar_tag_labels.get(id, null)
		if label == null:
			label = _create_tag_label(name)
			ar_tag_labels[id] = label

		_position_tag_label(label, center, corners)
		label.text = name
		label.visible = true

	# Hide labels for tags we didn't see this frame
	for existing_id in ar_tag_labels.keys():
		if not seen_ids.has(existing_id):
			(ar_tag_labels[existing_id] as Label3D).visible = false


func _create_tag_label(text: String) -> Label3D:
	var label = Label3D.new()
	label.text = text
	label.font_size = 64
	label.modulate = Color(1, 0.9, 0.2)
	label.outline_modulate = Color(0.3, 0.0, 0.0)
	label.outline_size = 8
	label.billboard = BaseMaterial3D.BILLBOARD_DISABLED
	label.no_depth_test = true
	label.pixel_size = AR_LABEL_PIXEL_SIZE
	# Parented to the backdrop so positions are in backdrop-local coords
	ar_backdrop_mesh.add_child(label)
	return label


func _pixel_to_backdrop(px: float, py: float) -> Vector3:
	# Convert 2D pixel coordinates (0..frame_w, 0..frame_h) into
	# 3D positions on the backdrop plane, in the backdrop's local frame.
	var u = px / float(ar_frame_w)
	var v = py / float(ar_frame_h)
	if CAM_UPSIDE_DOWN:
		# same 180° rotation as the backdrop's video
		u = 1.0 - u
		v = 1.0 - v
	var x = (u - 0.5) * ar_backdrop_size.x
	var y = (0.5 - v) * ar_backdrop_size.y
	# Tiny z offset so the label sits in front of the backdrop, not behind
	return Vector3(x, y, 0.5)


func _position_tag_label(label: Label3D, center: Array, corners: Array) -> void:
	var cx = float(center[0]) if center.size() > 0 else 0.0
	var cy = float(center[1]) if center.size() > 1 else 0.0
	var pos = _pixel_to_backdrop(cx, cy)

	# Lift the label above the tag by a fraction of its height so it
	# doesn't cover the marker
	var tag_h_px := 0.0
	if corners.size() == 4:
		var top_y = min(float(corners[0][1]), float(corners[1][1]))
		var bot_y = max(float(corners[2][1]), float(corners[3][1]))
		tag_h_px = bot_y - top_y
	var lift = (tag_h_px / float(ar_frame_h)) * ar_backdrop_size.y * 0.6 + 8.0

	label.position = Vector3(pos.x, pos.y + lift, pos.z)


# ============================================================
# TCP video
# ============================================================

func _connect_video_tcp() -> void:
	tcp_client.disconnect_from_host()
	tcp_client = StreamPeerTCP.new()
	var err = tcp_client.connect_to_host("127.0.0.1", VIDEO_PORT)
	if err != OK:
		push_error("[NetworkReceiver] Failed to initiate TCP connection (error %d)" % err)
	else:
		print("[NetworkReceiver] Connecting to video TCP %d..." % VIDEO_PORT)


func _process_tcp_status() -> void:
	tcp_client.poll()
	var status = tcp_client.get_status()
	if status == StreamPeerTCP.STATUS_CONNECTED and not tcp_connected:
		tcp_connected = true
		tcp_buffer.clear()
		tcp_client.set_no_delay(true)
	elif (status == StreamPeerTCP.STATUS_ERROR or status == StreamPeerTCP.STATUS_NONE) and tcp_connected:
		tcp_connected = false
		tcp_buffer.clear()
		_video_confirmed = false
		print("[NetworkReceiver] Video TCP disconnected")


func _read_uint32_be(data: PackedByteArray, offset: int) -> int:
	return (data[offset] << 24) | (data[offset + 1] << 16) | (data[offset + 2] << 8) | data[offset + 3]


func _process_video() -> void:
	if not tcp_connected:
		return

	var available = tcp_client.get_available_bytes()
	if available > 0:
		var result = tcp_client.get_data(available)
		if result[0] == OK:
			tcp_buffer.append_array(result[1])

	if tcp_buffer.size() > 32 * 1024 * 1024:
		print("[NetworkReceiver] TCP buffer overflow, clearing")
		tcp_buffer.clear()
		return

	var latest_w: int = 0
	var latest_h: int = 0
	var latest_data: PackedByteArray = PackedByteArray()
	var frames_in_this_batch: int = 0

	const HEADER_SIZE = 12

	while tcp_buffer.size() >= HEADER_SIZE:
		var w = _read_uint32_be(tcp_buffer, 0)
		var h = _read_uint32_be(tcp_buffer, 4)
		var frame_len = _read_uint32_be(tcp_buffer, 8)

		if w <= 0 or h <= 0 or w > 7680 or h > 4320:
			print("[NetworkReceiver] Bad frame dimensions: %dx%d, clearing buffer" % [w, h])
			tcp_buffer.clear()
			break

		var expected_len = w * h * 3
		if frame_len != expected_len:
			print("[NetworkReceiver] Frame length mismatch: got %d, expected %d, clearing" % [frame_len, expected_len])
			tcp_buffer.clear()
			break

		if tcp_buffer.size() < HEADER_SIZE + frame_len:
			break

		latest_w = w
		latest_h = h
		latest_data = tcp_buffer.slice(HEADER_SIZE, HEADER_SIZE + frame_len)
		tcp_buffer = tcp_buffer.slice(HEADER_SIZE + frame_len)
		frames_in_this_batch += 1

	if frames_in_this_batch == 0:
		return

	if frames_in_this_batch > 1:
		_frames_dropped_this_period += frames_in_this_batch - 1

	var img = Image.create_from_data(latest_w, latest_h, false, Image.FORMAT_RGB8, latest_data)
	if img == null:
		return

	if video_texture == null:
		video_texture = ImageTexture.create_from_image(img)
	else:
		video_texture.update(img)

	# Keep the AR side's frame size up to date so pixel->backdrop math is correct
	ar_frame_w = latest_w
	ar_frame_h = latest_h

	_frames_shown_this_period += 1

	if not _video_confirmed:
		_video_confirmed = true
		print("[NetworkReceiver] Video stream active! (%dx%d RGB)" % [latest_w, latest_h])


# ============================================================
# Gamepad sending
# ============================================================

func _encode_axis(value: float) -> int:
	return clampi(int((value + 1.0) * 127.5), 0, 255)

func _apply_deadzone(value: float) -> float:
	if absf(value) < DEADZONE:
		return 0.0
	return value

func _process_gamepad_send(delta: float) -> void:
	if Input.get_connected_joypads().is_empty():
		if gamepad_connected:
			gamepad_connected = false
			print("[NetworkReceiver] Gamepad disconnected")
		return

	if not gamepad_connected:
		gamepad_connected = true
		var pad_name = Input.get_joy_name(0)
		print("[NetworkReceiver] Gamepad connected: %s" % pad_name)

	gamepad_send_timer += delta
	if gamepad_send_timer < 1.0 / GAMEPAD_SEND_HZ:
		return
	gamepad_send_timer = 0.0

	var ly = _apply_deadzone(Input.get_joy_axis(0, JOY_AXIS_LEFT_Y))
	var rx = _apply_deadzone(Input.get_joy_axis(0, JOY_AXIS_RIGHT_X))
	# Hold the real kart still during the race countdown by sending a centered stick
	if _in_race_countdown():
		ly = 0.0
		rx = 0.0

	var packet := PackedByteArray()
	packet.append(GAMEPAD_HEADER)
	packet.append(4)
	packet.append(1); packet.append(_encode_axis(ly))
	packet.append(2); packet.append(_encode_axis(rx))

	udp_gamepad.put_packet(packet)


# True while a race is loaded and its countdown hasn't finished yet
func _in_race_countdown() -> bool:
	var scene = get_tree().current_scene
	if scene == null:
		return false
	var player = scene.get_node_or_null("CarManager/Player")
	return player != null and not player.HasRaceStarted


# ============================================================
# Fused pose receiver
# ============================================================

func _process_pose() -> void:
	while udp_pose_in.get_available_packet_count() > 0:
		var pkt = udp_pose_in.get_packet()
		var msg = pkt.get_string_from_utf8().strip_edges()
		var json = JSON.new()
		var err = json.parse(msg)
		if err != OK:
			print("[NetworkReceiver] Pose parse error: ", msg)
			continue
		var data = json.data
		if data is Dictionary:
			if not xrp_connected:
				xrp_connected = true
				print("[NetworkReceiver] Robot pose active!")
			pose_last_received = Time.get_ticks_msec() / 1000.0
			xrp_x = float(data.get("x", 0.0))
			xrp_y = float(data.get("y", 0.0))
			xrp_angle = float(data.get("angle", 0.0))
			xrp_trail.append(Vector2(xrp_x, xrp_y))
			if xrp_trail.size() > TRAIL_MAX:
				xrp_trail.remove_at(0)

	if xrp_connected:
		var now = Time.get_ticks_msec() / 1000.0
		if now - pose_last_received > POSE_TIMEOUT:
			xrp_connected = false
			print("[NetworkReceiver] Pose stream timed out")


func _on_joy_connection_changed(device_id: int, connected: bool) -> void:
	if connected:
		print("[NetworkReceiver] Controller %d connected: %s" % [device_id, Input.get_joy_name(device_id)])
	else:
		print("[NetworkReceiver] Controller %d disconnected" % device_id)
		if device_id == 0:
			gamepad_connected = false


# ============================================================
# Helpers
# ============================================================

func _find_node_by_name(node: Node, target: String) -> Node:
	if node.name == target:
		return node
	for child in node.get_children():
		var r = _find_node_by_name(child, target)
		if r:
			return r
	return null

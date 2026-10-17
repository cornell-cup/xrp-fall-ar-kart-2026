# ar_manager.gd - AR Manager
# On start: popup "Enable AR Camera?" -> Yes: auto-launch Python, show camera backdrop
# Hides: killPlane ground | Keeps: track edges, finishline, items, HUD, minimap
# Ports: UDP 6000 (tags), TCP 6001 (frames) - aligned with team convention

extends Node

var python_path: String = ""
var script_path: String = ""

var udp_tags := PacketPeerUDP.new()
var tcp_client := StreamPeerTCP.new()
var tcp_connected := false
var tcp_buffer := PackedByteArray()
var tcp_retry_timer := 0.0

var player_node = null
var player_found := false
var gate_cooldown := 0.0
var backdrop_mesh: MeshInstance3D = null
var camera_texture: ImageTexture = null
var frames_received := 0
var process_count := 0
var ar_enabled := false
var python_pid := -1

# Gate overlay
var gate_banner: Label3D = null
var gate_sub_label: Label3D = null
var gate_visible := false
var gate_fade_timer := 0.0


func _ready() -> void:
	_detect_paths()
	call_deferred("_show_camera_dialog")


func _detect_paths() -> void:
	var project_dir = ProjectSettings.globalize_path("res://")

	for p in [
		project_dir + "../venv/bin/python3",
		project_dir + "venv/bin/python3",
		"/usr/bin/python3",
		"/usr/local/bin/python3",
	]:
		if FileAccess.file_exists(p):
			python_path = p
			break

	var s = project_dir + "Scripts/ar_stream.py"
	if FileAccess.file_exists(s):
		script_path = s

	print("[AR] Python: %s | Script: %s" % [python_path, script_path])


func _show_camera_dialog() -> void:
	var dialog = ConfirmationDialog.new()
	dialog.title = "AR Camera"
	dialog.dialog_text = "Enable AR Camera?\n\nThis will use your webcam as the game background."
	dialog.ok_button_text = "Yes, use camera"
	dialog.cancel_button_text = "No, normal mode"
	dialog.size = Vector2i(400, 160)
	dialog.initial_position = Window.WINDOW_INITIAL_POSITION_CENTER_MAIN_WINDOW_SCREEN
	dialog.confirmed.connect(_on_camera_yes)
	dialog.canceled.connect(_on_camera_no)

	var canvas = CanvasLayer.new()
	canvas.layer = 100
	canvas.name = "ARDialogLayer"
	add_child(canvas)
	canvas.add_child(dialog)
	dialog.popup_centered()


func _on_camera_yes() -> void:
	ar_enabled = true
	var dl = get_node_or_null("ARDialogLayer")
	if dl: dl.queue_free()

	_launch_python()
	udp_tags.bind(6000, "0.0.0.0")
	_connect_tcp()
	_setup_camera_backdrop()
	_setup_gate_overlay()
	_hide_world()

	await get_tree().create_timer(1.0).timeout
	_find_player()
	print("[AR] AR mode active")


func _on_camera_no() -> void:
	ar_enabled = false
	var dl = get_node_or_null("ARDialogLayer")
	if dl: dl.queue_free()
	print("[AR] Normal game mode")


func _launch_python() -> void:
	if python_path == "" or script_path == "":
		print("[AR] ERROR: Python/script not found. Run manually:")
		print("  python3 Scripts/ar_stream.py --webcam")
		return

	var cmd = "OPENCV_AVFOUNDATION_SKIP_AUTH=1 '%s' '%s' --webcam --headless" % [python_path, script_path]
	python_pid = OS.create_process("/bin/bash", ["-c", cmd])
	print("[AR] Python PID: %d" % python_pid if python_pid > 0 else "[AR] ERROR: Python failed to start")


func _setup_camera_backdrop() -> void:
	var cam = _find_node_by_name(get_tree().root, "fpsCamera") as Camera3D
	if cam == null:
		print("[AR] fpsCamera not found")
		return

	var mesh = QuadMesh.new()
	mesh.size = Vector2(500, 350)

	var img = Image.create(960, 540, false, Image.FORMAT_RGB8)
	img.fill(Color(0.15, 0.15, 0.25))
	camera_texture = ImageTexture.create_from_image(img)

	var mat = StandardMaterial3D.new()
	mat.albedo_texture = camera_texture
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED

	backdrop_mesh = MeshInstance3D.new()
	backdrop_mesh.mesh = mesh
	backdrop_mesh.material_override = mat
	backdrop_mesh.name = "ARCameraBackdrop"
	cam.add_child(backdrop_mesh)
	backdrop_mesh.position = Vector3(0, 0, -90)


func _setup_gate_overlay() -> void:
	var cam = _find_node_by_name(get_tree().root, "fpsCamera") as Camera3D
	if cam == null: return

	gate_banner = Label3D.new()
	gate_banner.text = "MARIO KART AR"
	gate_banner.font_size = 72
	gate_banner.pixel_size = 0.25
	gate_banner.modulate = Color(1, 0.2, 0.2)
	gate_banner.outline_modulate = Color(1, 0.85, 0)
	gate_banner.outline_size = 12
	gate_banner.billboard = BaseMaterial3D.BILLBOARD_DISABLED
	gate_banner.no_depth_test = true
	gate_banner.visible = false
	gate_banner.name = "GateBanner"
	cam.add_child(gate_banner)

	gate_sub_label = Label3D.new()
	gate_sub_label.text = ""
	gate_sub_label.font_size = 48
	gate_sub_label.pixel_size = 0.2
	gate_sub_label.modulate = Color(1, 0.85, 0)
	gate_sub_label.outline_modulate = Color(0.6, 0, 0)
	gate_sub_label.outline_size = 8
	gate_sub_label.billboard = BaseMaterial3D.BILLBOARD_DISABLED
	gate_sub_label.no_depth_test = true
	gate_sub_label.visible = false
	gate_sub_label.name = "GateSubLabel"
	cam.add_child(gate_sub_label)


func _pixel_to_backdrop(px: float, py: float) -> Vector3:
	var x = (px / 960.0 - 0.5) * 500.0
	var y = (0.5 - py / 540.0) * 350.0
	return Vector3(x, y, -89.5)


func _show_gate(center: Array, corners: Array, gate_name: String) -> void:
	if gate_banner == null: return

	var cx = float(center[0])
	var cy = float(center[1])
	var pos = _pixel_to_backdrop(cx, cy)

	var tag_h_px := 0.0
	if corners.size() == 4:
		var top_y = min(float(corners[0][1]), float(corners[1][1]))
		var bot_y = max(float(corners[2][1]), float(corners[3][1]))
		tag_h_px = bot_y - top_y

	var banner_offset = (tag_h_px / 540.0) * 350.0 * 0.6 + 8.0

	# Scale based on tag size in frame (closer = bigger, farther = smaller)
	var scale_factor = tag_h_px / 100.0
	scale_factor = clamp(scale_factor, 0.3, 3.0)

	gate_banner.position = Vector3(pos.x, pos.y + banner_offset, pos.z)
	gate_banner.scale = Vector3(scale_factor, scale_factor, scale_factor)
	gate_banner.visible = true

	gate_sub_label.text = gate_name
	gate_sub_label.position = Vector3(pos.x, pos.y - banner_offset * 0.7, pos.z)
	gate_sub_label.scale = Vector3(scale_factor, scale_factor, scale_factor)
	gate_sub_label.visible = true

	gate_visible = true
	gate_fade_timer = 0.15


func _hide_gate() -> void:
	if gate_banner: gate_banner.visible = false
	if gate_sub_label: gate_sub_label.visible = false
	gate_visible = false


func _hide_world() -> void:
	var root = get_tree().root

	var kp = get_node_or_null("../killPlane")
	if kp == null: kp = _find_node_by_name(root, "killPlane")
	if kp: _hide_visual_children(kp)

	var cp = _find_node_by_name(root, "CameraPivot")
	if cp and cp is Node3D: cp.visible = false


func _hide_visual_children(node: Node) -> void:
	for child in node.get_children():
		if child is VisualInstance3D: child.visible = false
		if child is Light3D: child.visible = false
		_hide_visual_children(child)


func _connect_tcp() -> void:
	tcp_client.connect_to_host("127.0.0.1", 6001)


func _process(delta: float) -> void:
	if not ar_enabled: return

	process_count += 1
	if gate_cooldown > 0: gate_cooldown -= delta
	if not player_found: _find_player()

	if gate_visible:
		gate_fade_timer -= delta
		if gate_fade_timer <= 0:
			_hide_gate()

	tcp_client.poll()
	var status = tcp_client.get_status()

	if status == StreamPeerTCP.STATUS_CONNECTED:
		if not tcp_connected:
			tcp_connected = true
			print("[AR] TCP connected")
		_receive_tcp_frames()
	elif status == StreamPeerTCP.STATUS_NONE or status == StreamPeerTCP.STATUS_ERROR:
		tcp_connected = false
		tcp_retry_timer += delta
		if tcp_retry_timer >= 2.0:
			tcp_retry_timer = 0.0
			tcp_client = StreamPeerTCP.new()
			_connect_tcp()

	_receive_tags()


func _receive_tcp_frames() -> void:
	var avail = tcp_client.get_available_bytes()
	if avail > 0:
		var chunk = tcp_client.get_data(avail)
		if chunk[0] == OK: tcp_buffer.append_array(chunk[1])

	while tcp_buffer.size() >= 4:
		var size = (tcp_buffer[0] << 24) | (tcp_buffer[1] << 16) | (tcp_buffer[2] << 8) | tcp_buffer[3]
		if size <= 0 or size > 2000000:
			tcp_buffer.clear()
			break
		if tcp_buffer.size() < 4 + size: break

		var jpeg = tcp_buffer.slice(4, 4 + size)
		tcp_buffer = tcp_buffer.slice(4 + size)

		var img = Image.new()
		if img.load_jpg_from_buffer(jpeg) == OK:
			camera_texture.update(img)
			frames_received += 1


func _receive_tags() -> void:
	while udp_tags.get_available_packet_count() > 0:
		var data = JSON.parse_string(udp_tags.get_packet().get_string_from_utf8())
		if data == null or data.get("type", "") != "ar_tags": continue
		for tag in data.get("tags", []):
			var center = tag.get("center", [0, 0])
			var corners = tag.get("corners", [])
			var gate_name = tag.get("name", "")
			var event = tag.get("event", "")

			_show_gate(center, corners, gate_name)
			_trigger_gate(event, gate_name)


func _trigger_gate(event: String, name: String) -> void:
	if gate_cooldown > 0 or player_node == null: return
	gate_cooldown = 2.0
	match event:
		"checkpoint":    print("[AR] Passed: %s" % name)
		"speed_boost":   player_node.Current = 2; player_node.StartTimer(player_node); print("[AR] SPEED BOOST!")
		"mystery_box":   player_node.StoreItem(randi() % 3); print("[AR] ITEM BOX!")
		"banana_trap":   player_node.Current = 1; player_node.StartTimer(player_node); print("[AR] HAZARD!")


func _find_player() -> void:
	player_node = get_node_or_null("../CarManager/Player")
	if player_node == null:
		player_node = _find_node_by_name(get_tree().root, "Player")
	if player_node:
		player_found = true


func _find_node_by_name(node: Node, target: String) -> Node:
	if node.name == target: return node
	for child in node.get_children():
		var r = _find_node_by_name(child, target)
		if r: return r
	return null


func _exit_tree() -> void:
	udp_tags.close()
	tcp_client.disconnect_from_host()
	if python_pid > 0: OS.kill(python_pid)

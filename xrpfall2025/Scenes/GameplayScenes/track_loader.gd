extends Node

@export var checkpoints : Array[Vector3]
@export var innerPath : Path3D
@export var startingPoint : Vector3
# half the road's width in world units, read from the track's road polygon
@export var roadHalfWidth : float

@export var filePath : String

# adjustable in order to make sure tracking and placements are accurate.
# Tracks built in metres (like real_track.tscn) make this the world units per metre
@export var scale : float

# overhead minimap camera, framed around whichever track gets loaded
@export var minimapCamera : Camera3D

func Init():
	var scene = load(LevelData.level_path).instantiate()
	add_child(scene)
	print(get_children())
	$Track.scale *= scale
	innerPath = $"Track/Final Path"
	startingPoint = innerPath.curve.get_point_position(0)
	roadHalfWidth = get_road_half_width()
	generate_checkpoints()
	fit_minimap_camera()

# all pieces are (currently) only 3 points long, so taking every other point
# gives us the joints between them
func generate_checkpoints():
	var c = innerPath.curve
	for i in c.point_count:
		if i % 2 == 0:
			checkpoints.append(c.get_point_position(i) * scale)
	print(checkpoints)

# the road is the CSGPolygon3D swept along the track's path; its cross-section is the road width
func get_road_half_width() -> float:
	var road = $Track.get_node_or_null("CSGPolygon3D") as CSGPolygon3D
	if road == null:
		return 10.0
	var half = 0.0
	for p in road.polygon:
		half = max(half, abs(p.x))
	return half * scale

# put the minimap camera over the middle of the track, high enough to see all of it
func fit_minimap_camera():
	if minimapCamera == null:
		return
	var lo = Vector3(INF, 0, INF)
	var hi = Vector3(-INF, 0, -INF)
	for p in innerPath.curve.get_baked_points():
		var g = innerPath.to_global(p)
		lo = Vector3(min(lo.x, g.x), 0, min(lo.z, g.z))
		hi = Vector3(max(hi.x, g.x), 0, max(hi.z, g.z))
	lo -= Vector3(roadHalfWidth, 0, roadHalfWidth)
	hi += Vector3(roadHalfWidth, 0, roadHalfWidth)
	var center = (lo + hi) / 2
	var half = (hi - lo) / 2
	# look straight down with the start direction (-Z) as screen up, the same way
	# up as the track drawing in Car Map.pdf
	var view = minimapCamera.get_viewport().get_visible_rect().size
	var tan_half_fov = tan(deg_to_rad(minimapCamera.fov) / 2)
	var height = max(half.z / tan_half_fov, half.x / (tan_half_fov * view.x / view.y)) * 1.1
	minimapCamera.global_position = Vector3(center.x, height, center.z)
	minimapCamera.global_basis = Basis.looking_at(Vector3.DOWN, Vector3.FORWARD)

func request_track() -> void:
	var fileDialog = $FileDialog
	fileDialog.popup_centered_clamped()
	
	var path = await fileDialog.file_selected
	print("finished await1")
	if !path.ends_with(".tscn"):
		pass # deal with later
	
	filePath = path

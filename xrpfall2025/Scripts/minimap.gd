extends Node

# Keeps the player findable on the minimap: while their kart is outside the
# area the minimap shows (it's framed around the track), an arrow is pinned to
# the minimap's edge, pointing the way the kart is

@onready var container: SubViewportContainer = $MarginContainer/SubViewportContainer
@onready var minimap_camera: Camera3D = $MarginContainer/SubViewportContainer/SubViewport/minimapCamera

# how far in from the minimap's edge the arrow sits, in pixels
const EDGE_MARGIN: float = 12.0
const ARROW_COLOR: Color = Color(0.7, 1.0, 0.1)

var arrow: Polygon2D


func _ready() -> void:
	arrow = Polygon2D.new()
	arrow.polygon = PackedVector2Array([Vector2(10, 0), Vector2(-7, -7), Vector2(-7, 7)])
	arrow.color = ARROW_COLOR
	arrow.visible = false
	container.add_child(arrow)


func _process(_delta: float) -> void:
	var body = get_node_or_null("../CarManager/Player/Node3D/Player") as Node3D
	if body == null:
		arrow.visible = false
		return

	var view: Vector2 = container.size
	var on_map: Vector2 = minimap_camera.unproject_position(body.global_position)
	var shown_area = Rect2(Vector2.ZERO, view).grow(-EDGE_MARGIN)
	arrow.visible = not shown_area.has_point(on_map)
	if not arrow.visible:
		return

	# from the middle of the minimap towards the kart, stopped at the edge
	var center = view / 2
	var direction = (on_map - center).normalized()
	var half = center - Vector2(EDGE_MARGIN, EDGE_MARGIN)
	var reach_x = half.x / abs(direction.x) if direction.x != 0 else INF
	var reach_y = half.y / abs(direction.y) if direction.y != 0 else INF
	arrow.position = center + direction * min(reach_x, reach_y)
	arrow.rotation = direction.angle()

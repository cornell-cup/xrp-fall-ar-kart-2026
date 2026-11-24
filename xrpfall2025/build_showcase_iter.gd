@tool
extends EditorScript

func _run():
	# We will create a brand new clean scene for your design iteration
	var path = "res://real_track_design.tscn" 
	
	# Start from the clean base track
	var packed_scene = load("res://real_track.tscn")
	var root = packed_scene.instantiate()

	# 1. Ensure it's clean (just in case)
	for child in root.get_children():
		if child.name not in ["Final Path", "CSGPolygon3D", "InnerEdge", "OuterEdge", "StartLine"]:
			child.free()

	# 2. Add Grass
	var grass = MeshInstance3D.new()
	grass.name = "Grass"
	var plane = PlaneMesh.new()
	plane.size = Vector2(100, 100)
	grass.mesh = plane
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color("#68C151")
	grass.material_override = mat
	grass.position = Vector3(0, -0.01, 0)
	root.add_child(grass)
	grass.owner = root

	# 3. Add Sky
	var we = WorldEnvironment.new()
	we.name = "WorldEnvironment"
	var env = Environment.new()
	var sky = Sky.new()
	var sky_mat = ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color("#6AAEFF")
	sky_mat.sky_horizon_color = Color("#FFFFFF")
	sky_mat.ground_bottom_color = Color("#68C151")
	sky_mat.ground_horizon_color = Color("#68C151")
	sky.sky_material = sky_mat
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	we.environment = env
	root.add_child(we)
	we.owner = root

	# 4. Add the EnvironmentProps container
	var props = Node3D.new()
	props.name = "EnvironmentProps"
	root.add_child(props)
	props.owner = root

	# Helper function to easily add and position props
	var add_prop = func(file_name: String, pos: Vector3, rot_y_deg: float):
		var scene = load("res://Assets/3dmodels/KenneyRacing/" + file_name + ".glb")
		if not scene:
			scene = load("res://Assets/3dmodels/KenneyRacing/" + file_name + ".gltf")
		if scene:
			var inst = scene.instantiate()
			inst.name = file_name
			props.add_child(inst)
			inst.owner = root
			inst.scale = Vector3(0.444, 0.444, 0.444)
			inst.position = pos
			inst.rotation_degrees = Vector3(0, rot_y_deg, 0)

	# --- YOUR DESIGN PLACEMENTS ---
	# The start line is at (0, 0, 0). The road is ~0.44 wide.
	
	# Gate at Start Line (Rotated 90 degrees to span across the track)
	add_prop.call("overhead", Vector3(0, 0, 0), 90)
	
	# Checkered Flags on the left and right of the start line
	add_prop.call("flagCheckers", Vector3(-0.25, 0, 0), 0)
	add_prop.call("flagCheckers", Vector3(0.25, 0, 0), 0)

	# Grandstands in the infield (x = 0.5 is the middle of the oval grass island)
	# Placed just "before" the gate at z = 0.3
	add_prop.call("grandStandAwning", Vector3(0.32, 0, 0.3), 90)  # Facing the left straightaway
	add_prop.call("grandStandAwning", Vector3(0.68, 0, 0.3), -90) # Facing the right straightaway

	# A few trees scattered on the outside edges to frame it
	add_prop.call("treeLarge", Vector3(-0.4, 0, 0.4), 0)
	add_prop.call("treeSmall", Vector3(-0.35, 0, -0.4), 0)
	add_prop.call("treeLarge", Vector3(1.4, 0, 0.2), 0)
	add_prop.call("treeSmall", Vector3(1.35, 0, -0.3), 0)

	# Save the new scene
	var new_scene = PackedScene.new()
	new_scene.pack(root)
	ResourceSaver.save(new_scene, path)
	print("Design V1 generated successfully! Please open real_track_design.tscn.")

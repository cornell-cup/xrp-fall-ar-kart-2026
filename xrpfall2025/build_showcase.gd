@tool
extends EditorScript

func _run():
	# Make sure this matches your current scene name!
	var path = "res://real_track-Asset-mash-v2.tscn" 
	var packed_scene = load(path)
	var root = packed_scene.instantiate()

	# 1. Clean up old stuff
	for child in root.get_children():
		if child.name not in ["Final Path", "CSGPolygon3D", "InnerEdge", "OuterEdge", "StartLine"]:
			child.free()

	# 2. Add Clean Vibrant Grass
	var grass = MeshInstance3D.new()
	grass.name = "Grass"
	var plane = PlaneMesh.new()
	plane.size = Vector2(100, 100)
	grass.mesh = plane
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color("#68C151") # Vibrant Kenney Green
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

	# 4. Generate positions wrapping around the track
	var valid_positions = []
	var spacing = 0.8 # Space between props
	for ix in range(-10, 11):
		for iz in range(-10, 11):
			var px = 0.5 + ix * spacing
			var pz = 0.0 + iz * spacing
			# Track bounds approx: x from -0.5 to 1.5, z from -0.8 to 0.8
			if px > -0.6 and px < 1.6 and pz > -1.0 and pz < 1.0:
				continue
			valid_positions.append(Vector3(px, 0, pz))
	
	# Sort positions by distance to the track center so they wrap tightly
	valid_positions.sort_custom(func(a, b): return a.distance_to(Vector3(0.5, 0, 0)) < b.distance_to(Vector3(0.5, 0, 0)))

	# 5. Add Grid
	var grid = Node3D.new()
	grid.name = "KenneyAssetGrid"
	root.add_child(grid)
	grid.owner = root

	var dir = DirAccess.open("res://Assets/3dmodels/KenneyRacing")
	if dir:
		dir.list_dir_begin()
		var file_name = dir.get_next()
		var index = 0
		while file_name != "":
			if file_name.ends_with(".glb") or file_name.ends_with(".gltf"):
				var model_scene = load("res://Assets/3dmodels/KenneyRacing/" + file_name)
				if model_scene:
					var instance = model_scene.instantiate()
					instance.name = file_name.get_basename()
					grid.add_child(instance)
					instance.owner = root
					
					if index < valid_positions.size():
						var pos = valid_positions[index]
						var target = Vector3(0.5, 0, 0)
						# Safely calculate position, rotation, and scale in memory!
						instance.transform = Transform3D(Basis(), pos).looking_at(target, Vector3.UP).scaled_local(Vector3(0.444, 0.444, 0.444))
					else:
						instance.scale = Vector3(0.444, 0.444, 0.444)
					
					index += 1
			file_name = dir.get_next()

	# Save the scene
	var new_scene = PackedScene.new()
	new_scene.pack(root)
	ResourceSaver.save(new_scene, path)
	print("Showcase generated successfully! Please reload the scene.")

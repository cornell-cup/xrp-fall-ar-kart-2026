using Godot;
using Vector3 = Godot.Vector3;
using System.IO;
using System.Text.Json;

/// <summary>
/// Blueprint for a player controlled car. Handles inputs
/// </summary>
public partial class Player : Car
{
	/// <summary>
	/// Any additional set up logic here outside of Init and parent field set up
	/// </summary>
	private Vector3 xrpOffset = Vector3.Zero;
	// The way the kart faces on the start line, which is the way the real XRP faces when fusion starts
	private Vector3 xrpForward = Vector3.Forward;
	private float xrpStartYaw = 0;
	// Last pose target, for working out how fast the real XRP is going
	private Vector3 lastXrpTarget = Vector3.Zero;
	private double xrpSpeed = 0;

	public override void Init(Vector3 startingPosition, Path3D track, int totalCheckpoints, Vector3 facingDirection)
	{
		base.Init(startingPosition, track, totalCheckpoints, facingDirection);
		// Save where the car spawns (the start line) — XRP (0,0) maps to this position
		xrpOffset = startingPosition;
		xrpForward = facingDirection.Normalized();
		xrpStartYaw = charbody3d.Rotation.Y;
	}
	
	public override void _Ready()
	{
		base._Ready();
		
		// For customization, not yet applied
		ApplySavedCarColor();
		
	}

	/// <summary>
	/// Any logic to ensure no memory leaks (particularly with any events)
	/// when this Player leaves the scene tree during runtime for whatever reason
	/// </summary>
	public override void _ExitTree()
	{
	}

	/// <summary>
	/// General game logic being run every frame.
	/// If you're modifying anything relating to the Player's physics,
	/// please use _PhysicsProcess()
	/// </summary>
	/// <param name="delta">delta time</param>
	public override void _Process(double delta)
	{
		// Items only affect the game, never the real XRP
		if (Input.IsActionJustPressed("use_item") && raceStarted && !finishedRace)
		{
			(GetParent() as CarManager)?.UseItem(this);
		}

		// Put the kart back on the road. The real XRP's position can't be reset, so only without it
		if (Input.IsActionJustPressed("reset_kart") && !IsXrpActive())
		{
			ResetToTrack();
		}

		//draw gizmos
		//DebugDraw3D.DrawBox(AABB.Position, Godot.Quaternion.Identity, Vector3.One, color);
		//GD.Print("Player State: " + Current);

		//closest point
		/*DebugDraw3D.DrawBox(
			GetClosestAbsolutePosition(track, GlobalPosition),
			Godot.Quaternion.Identity,
			Vector3.One,
			color
			);*/

		//check the timer
		//GD.Print("Player Timer: " + timer.ElapsedMilliseconds);

		//when collision event, run rng and start stopwatch. unsubscribe collision event and start stopwatch event
		//after certain time, resubscribe collision event, reset stop watch.
	}

	/// <summary>
	/// Called every frame to process how this object will move by what
	/// Will rotate this object left or right, move forwards or backwards locally
	/// and fall if necessary
	/// </summary>
	/// <param name="delta">delta time</param>
	public override void _PhysicsProcess(double delta)
	{
		// ---- Try XRP control ----
		var receiver = GetNode("/root/NetworkReceiver");

		if (IsXrpActive())
		{
			float xrpX = (float)receiver.Get("xrp_x");
			float xrpY = (float)receiver.Get("xrp_y");
			float xrpAngle = (float)receiver.Get("xrp_angle");

			// Fusion sends the robot's pose in cm relative to where it started: +x is the way it
			// faced at the start (sent negated), +y is to its left, and the angle is counter-clockwise
			// in degrees. In Car Map.pdf's frame that's +y forward and +x right of the start line.
			float forwardMetres = -xrpX / 100f;
			float leftMetres = xrpY / 100f;

			// Metres to world units: tracks are built in metres and scaled up by the track loader
			float unitsPerMetre = track.GlobalTransform.Basis.Scale.X;
			Vector3 xrpRight = xrpForward.Cross(Vector3.Up);
			Vector3 targetPos = xrpOffset
				+ xrpForward * forwardMetres * unitsPerMetre
				- xrpRight * leftMetres * unitsPerMetre;
			targetPos.Y = charbody3d.GlobalPosition.Y;

			// Smooth position
			charbody3d.GlobalPosition = charbody3d.GlobalPosition.Lerp(targetPos, 0.3f);

			// Counter-clockwise from the start heading, which is also counter-clockwise in Godot
			float targetRotY = xrpStartYaw + Mathf.DegToRad(xrpAngle);
			charbody3d.Rotation = new Vector3(
				charbody3d.Rotation.X,
				Mathf.LerpAngle(charbody3d.Rotation.Y, targetRotY, 0.3f),
				charbody3d.Rotation.Z
			);

			// The XRP's pose decides where the kart is, so the base movement must not push it
			// forward too. Speed is zeroed for that, then set to the measured speed for the HUD
			speed = 0;
			base._PhysicsProcess(delta);
			if (lastXrpTarget != Vector3.Zero)
			{
				double measured = (targetPos - lastXrpTarget).Length() / delta;
				xrpSpeed = Mathf.Lerp(xrpSpeed, measured, 0.1);
			}
			lastXrpTarget = targetPos;
			speed = xrpSpeed;
			return;
		}
		else
		{
			// ---- Fallback: keyboard ----
			// No steering or throttle during the countdown, only gravity
			if (!raceStarted)
			{
				speed = 0;
				base._PhysicsProcess(delta);
				return;
			}

			if (((Input.IsActionPressed("right") && current != States.Inverted)
				|| (Input.IsActionPressed("left") && current == States.Inverted)) && charbody3d.IsOnFloor())
			{
				charbody3d.RotateObjectLocal(new Vector3(0, 1, 0), -(float)rotationIncrement);
			}
			else if (((Input.IsActionPressed("left") && current != States.Inverted)
				|| (Input.IsActionPressed("right") && current == States.Inverted)) && charbody3d.IsOnFloor())
			{
				charbody3d.RotateObjectLocal(new Vector3(0, 1, 0), (float)rotationIncrement);
			}

			if (Input.IsActionPressed("back") && charbody3d.IsOnFloor())
			{
				if (speed > -maxSpeed) speed -= acceleration * delta;
			}
			else if (Input.IsActionPressed("forward") && charbody3d.IsOnFloor())
			{
				if (speed < maxSpeed) speed += acceleration * delta;
			}
			else
			{
				speed *= 0.9 * delta;
			}
			// Process movement (gravity, state speed, move-and-slide)
			base._PhysicsProcess(delta);
		}

		
	}
	
	/// <summary>
	/// Is the kart being driven by the real XRP's position right now?
	/// </summary>
	public bool IsXrpActive()
	{
		var receiver = GetNodeOrNull("/root/NetworkReceiver");
		return receiver != null && (bool)receiver.Get("xrp_connected");
	}

	//Customization file reading, saves the color to be applied to the car
	protected override void ApplySavedCarColor()
	{
		string path = ProjectSettings.GlobalizePath("user://customization.json");
		if (!File.Exists(path))
		{
			GD.Print("No customization file found.");
			return;
		}
		else 
		{
			GD.Print("File Found.");
		}

		string json = File.ReadAllText(path);
		var data = JsonSerializer.Deserialize<CustomizationData>(json);

		if (data != null && !string.IsNullOrEmpty(data.CarColor))
		{
			Color color = new Color(data.CarColor); // Godot parses "#RRGGBB"
			
			if (color != null){
				GD.Print(color);
			}
			
			SetCarColor(color);
		}
	}
	
	// Sets the material of the car to the proper color
	protected override void SetCarColor(Color color)
	{
		if (carMesh == null)
		{
			GD.PrintErr("Car mesh not found!");
			return;
		}

		var mat = carMesh.GetActiveMaterial(0) as StandardMaterial3D;
		if (mat != null)
		{
			var newMat = (StandardMaterial3D)mat.Duplicate();
			newMat.AlbedoColor = color;
			carMesh.SetSurfaceOverrideMaterial(0, newMat);
		}
		else
		{
			var newMat = new StandardMaterial3D();
			newMat.AlbedoColor = color;
			carMesh.MaterialOverride = newMat;
		}
	}

	private class CustomizationData
	{
		[System.Text.Json.Serialization.JsonPropertyName("car_color")]
		public string CarColor { get; set; }
	}
	 
}

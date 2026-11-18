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
	// World units per real metre without a track (free play): the TrackLoader's scale in main.tscn
	private const float FREEPLAYUNITSPERMETRE = 50f;
	// The kart model's front wheels, turned to match the steering and rolling as it moves.
	// Each is a pivot at the wheel's axle (the model's own wheel origin is at the bottom of the tyre)
	private Node3D frontWheelL = null;
	private Node3D frontWheelR = null;
	// Front wheel radius in world units, and how far round they've rolled (radians)
	private float frontWheelRadius = 1f;
	private float wheelRoll = 0f;
	// Where the kart was last frame, for how far it has moved
	private Vector3 lastWheelPos = Vector3.Zero;
	private static readonly Shader WheelMarksShader = GD.Load<Shader>("res://Assets/wheel_marks.gdshader");
	// Stripes round the wheel (marks in wheel_marks.gdshader). Turning more than half the gap
	// between two stripes in one frame makes the wheel look still or spinning backwards, so each
	// frame's roll is capped at this fraction of the gap
	private const float WHEELMARKS = 8f;
	private const float WHEELMAXROLLPERFRAME = 0.3f;
	// How far the front wheels turn at full lock, in degrees
	private const float WHEELMAXDEG = 30f;
	// How quickly the wheels follow the steering (higher = snappier)
	private const float WHEELFOLLOW = 15f;
	// Current steering shown on the wheels, -1 (full left) .. 1 (full right)
	private float wheelSteer = 0f;

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

		// The wheels sit beside the chassis in the kart model
		Node model = carMesh?.GetParent();
		frontWheelL = PivotWheel(model, "FrontWheel_L");
		frontWheelR = PivotWheel(model, "FrontWheel_R");
		lastWheelPos = charbody3d.GlobalPosition;

		// The kart camera sits beside the front wheels, so it can't see them: show
		// the front of the kart along the bottom of its view instead
		var receiver = GetNodeOrNull("/root/NetworkReceiver");
		var camera = model?.GetParent()?.GetNodeOrNull<Camera3D>("fpsCamera");
		if (receiver != null && camera != null)
			receiver.CallDeferred("show_kart_nose", camera);
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

		TurnFrontWheels(delta);

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

			// Fusion (lalo's map-frame estimator) sends the robot's position in cm relative to the
			// start line: x = forward (the way it faced at the start), y = to its left. The angle is
			// its heading in Car Map.pdf's frame, counter-clockwise in degrees, 90 when facing
			// forward from the start line.
			float forwardMetres = xrpX / 100f;
			float leftMetres = xrpY / 100f;

			// Metres to world units: tracks are built in metres and scaled up by the track loader.
			// Free play has no track, so it uses the same scale as the race tracks
			float unitsPerMetre = track != null ? track.GlobalTransform.Basis.Scale.X : FREEPLAYUNITSPERMETRE;
			Vector3 xrpRight = xrpForward.Cross(Vector3.Up);
			Vector3 targetPos = xrpOffset
				+ xrpForward * forwardMetres * unitsPerMetre
				- xrpRight * leftMetres * unitsPerMetre;
			targetPos.Y = charbody3d.GlobalPosition.Y;

			// Smooth position
			charbody3d.GlobalPosition = charbody3d.GlobalPosition.Lerp(targetPos, 0.3f);

			// Turning left (counter-clockwise) is also counter-clockwise in Godot; 90 = start heading
			float targetRotY = xrpStartYaw + Mathf.DegToRad(xrpAngle - 90f);
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

			// A steering wheel turns the kart in proportion to how far it's turned,
			// with the same ratio as the real kart (full wheel lock = full turning rate)
			if ((bool)receiver.Call("wheel_connected"))
			{
				float steering = (float)receiver.Call("wheel_steering");
				if (current == States.Inverted) steering = -steering;
				if (steering != 0 && charbody3d.IsOnFloor())
				{
					charbody3d.RotateObjectLocal(new Vector3(0, 1, 0), -steering * (float)rotationIncrement);
				}
			}
			else if (((Input.IsActionPressed("right") && current != States.Inverted)
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
	/// Puts a wheel under a new node at its axle, named like the wheel, so it can
	/// steer and roll around its centre. Returns that node
	/// </summary>
	private Node3D PivotWheel(Node model, string name)
	{
		var mesh = model?.GetNodeOrNull<MeshInstance3D>(name);
		if (mesh == null)
			return null;
		Vector3 centre = mesh.GetAabb().GetCenter();
		float scale = (model as Node3D)?.GlobalBasis.Scale.Y ?? 1f;
		frontWheelRadius = mesh.GetAabb().Size.Y / 2f * scale;
		mesh.Name = name + "Mesh";
		// stripes round the tyre so the rolling shows
		var marks = new ShaderMaterial { Shader = WheelMarksShader };
		marks.SetShaderParameter("centre", centre);
		marks.SetShaderParameter("radius", mesh.GetAabb().Size.Y / 2f);
		mesh.MaterialOverlay = marks;
		var pivot = new Node3D { Name = name, Position = mesh.Position + centre };
		model.AddChild(pivot);
		model.RemoveChild(mesh);
		pivot.AddChild(mesh);
		mesh.Position = -centre;
		return pivot;
	}

	/// <summary>
	/// Turns the model's front wheels to the current steering input and rolls them
	/// by how far the kart has moved forward or back
	/// </summary>
	private void TurnFrontWheels(double delta)
	{
		if (frontWheelL == null || frontWheelR == null)
			return;
		float target = SteeringInput();
		wheelSteer = Mathf.Lerp(wheelSteer, target, Mathf.Min(1f, (float)delta * WHEELFOLLOW));

		Vector3 pos = charbody3d.GlobalPosition;
		float moved = (pos - lastWheelPos).Dot(-charbody3d.GlobalBasis.Z);
		lastWheelPos = pos;
		// a big jump is a respawn or restart, not driving
		if (Mathf.Abs(moved) < frontWheelRadius * 4f)
		{
			float maxRoll = Mathf.Tau / WHEELMARKS * WHEELMAXROLLPERFRAME;
			float roll = Mathf.Clamp(-moved / frontWheelRadius, -maxRoll, maxRoll);
			wheelRoll = Mathf.Wrap(wheelRoll + roll, -Mathf.Tau, Mathf.Tau);
		}

		// The model faces +X with its right side towards +Z, so turning right is a negative Y
		// rotation, and rolling forward is a negative Z rotation (applied after the steering)
		float angle = -wheelSteer * Mathf.DegToRad(WHEELMAXDEG);
		frontWheelL.Rotation = new Vector3(0, angle, wheelRoll);
		frontWheelR.Rotation = new Vector3(0, angle, wheelRoll);
	}

	/// <summary>
	/// Steering from the wheel, gamepad or keyboard, -1 (full left) .. 1 (full right):
	/// the same input that steers the kart or is sent to the XRP
	/// </summary>
	private float SteeringInput()
	{
		var receiver = GetNodeOrNull("/root/NetworkReceiver");
		float steering;
		if (receiver != null && (bool)receiver.Call("wheel_connected"))
		{
			steering = (float)receiver.Call("wheel_steering");
		}
		else
		{
			steering = Input.GetActionStrength("right") - Input.GetActionStrength("left");
			if (receiver != null && Input.GetConnectedJoypads().Count > 0)
			{
				float stick = (float)receiver.Call("_apply_deadzone", Input.GetJoyAxis(0, JoyAxis.RightX));
				if (stick != 0) steering = stick;
			}
		}
		// Inverted controls turn the game kart the other way (the real XRP isn't affected)
		if (current == States.Inverted && !IsXrpActive()) steering = -steering;
		return Mathf.Clamp(steering, -1f, 1f);
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

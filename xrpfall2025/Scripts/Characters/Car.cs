using Godot;
using System.Collections.Generic;

#region Enums
/// <summary>
/// Enum for defining the car's current 
/// state given an item effect from impact/use
/// </summary>
public enum States
{
	Inverted,
	Slow,
	Fast,
	Regular
};
#endregion

/// <summary>
/// Parent class that all car types will inherit from.
/// In this case: Player and Enemy AI
/// </summary>
public partial class Car : Node
{
	#region Fields
	protected States current; //current state
	protected States storedItem = States.Regular;

	//Seconds left before the current item effect wears off.
	//Counted down in _PhysicsProcess so it pauses along with the game
	protected double effectTimeRemaining = 0;

	//How long each item effect lasts, in seconds
	protected const double FASTDURATION = 5;
	protected const double SLOWDURATION = 10;
	protected const double INVERTEDDURATION = 10;

	#region racing related fields
	// Temporary values
	protected int place = 1;
	protected int lap = 1;
	protected bool finishedRace = false;
	//flipped to true by the hud's "countdown_finished" signal (hooked up in SceneManager)
	protected bool raceStarted = false; 
	protected Path3D track;

	//checkpoints passed for the onFinishline Collision check --> lap increment
	protected List<Checkpoint> passedCheckpoints = new List<Checkpoint>();

	protected int totalCheckpoints = 0;
	#endregion

	#region movement related fields
	protected const double MAXSPEED = 30;
	protected double maxSpeed = 30;

	//if the car gets a speedup or speed debuff, 
	//multiply/divide speed by cooresponding multiplier
	//has to be int because Vectors are made up by ints
	[Export] protected double slowMultiplier = 0.5;
	[Export] protected double fastMultiplier = 2;

	// How fast the car moves in meters per second.
	protected double speed = 0;
	protected double acceleration = 10;
	//Rotation speed
	protected double rotationIncrement = Mathf.DegToRad(2); //per delta
	
	// The downward acceleration when in the air, in meters per second squared.
	[Export] protected int fallAcceleration = 10;
	#endregion

	#region track position related fields
	//Half the width of the road in world units, read from the loaded track by the track loader.
	//A car whose center is further than this from the middle of the road is off the track
	public static float RoadHalfWidth = 10;
	//how high above the track cars are dropped in when spawning or resetting.
	//Clears the real-size kart's collision box so it settles onto the road
	protected const float SPAWNHEIGHT = 4;

	//Total distance driven along the track (in the curve's own units), counting every lap. 
	//Goes down when driving backwards. Used for placements
	protected float trackProgress = 0;
	//where along the track the car was last physics frame
	private float lastTrackOffset = 0;
	#endregion

	#region car mesh and size related fields
	//Gizmos for debugging
	protected Color color;
	[Export] protected CsgBox3D csgBox3D = null;
	[Export] protected CharacterBody3D charbody3d = null;
	// Used for customization of car mesh color
	[Export] protected MeshInstance3D carMesh = null;

	protected float radius = 1;
	protected Vector3 halflength;
	#endregion
	#endregion

	#region Properties
	// Expose properties for HUD
	public double Speed => CurrentSpeed;
	public double Acceleration => acceleration;
	public Vector3 CurrentPosition => charbody3d.Position;
	public Vector3 GlobalPosition => charbody3d.GlobalPosition;
	public int Place => place;
	public int Lap => lap;

	/// <summary>
	/// The current state of the car. Get/Set
	/// Relevant when we eventually add items that modify
	/// the car's state (defined by enum State)
	/// </summary>
	public States Current
	{
		get { return current; }
		set { current = value; }
	}

	/// <summary>
	/// How fast is the car currently going
	/// </summary>
	public double CurrentSpeed
	{
		get
		{
			return current switch
			{
				States.Slow => speed * slowMultiplier,
				States.Fast => speed * fastMultiplier,
				_ => speed,
			};
		}
	}

	/// <summary>
	/// Makes the current stored item publc to be able to access in hud.gd
	/// </summary>
	public States StoredItem
	{
		get { return storedItem; }
		set { storedItem = value; }
	}

	/// <summary>
	/// Seconds left before the current item effect wears off. 0 when in States.Regular
	/// </summary>
	public double EffectTimeRemaining
	{
		get { return effectTimeRemaining; }
	}

	/// <summary>
	/// Total length in seconds of the current item effect. Used for the HUD's effect timer
	/// </summary>
	public double EffectDuration
	{
		get { return GetEffectDuration(current); }
	}

	/// <summary>
	/// How many checkpoints has this car passed this lap?
	/// </summary>
	public int NumPassedCheckpoints
	{
		get { return passedCheckpoints.Count; }
	}

	/// <summary>
	/// Property to give the car's aabb centered on the car,
	/// used for collision calculations happening in the collided item
	/// </summary>
	public Aabb AABB
	{
		get
		{
			//move the aabb to the actual object's transform
			//otherwise the aabb sits in the origin
			Aabb temp = csgBox3D.GetAabb();
			temp.Position = charbody3d.GlobalPosition - temp.Size / 2;
			return temp;
		}
	}

	/// <summary>
	/// Refers to the radius of the bounding circle 
	/// used in the checkpoint collision checks
	/// </summary>
	public float Radius
	{
		get { return radius; }
	}

	/// <summary>
	/// Refers to the halflength of the bounding box in 
	/// the x, y, and z direction. Used for aabb collision checks
	/// </summary>
	public Vector3 Halflength
	{
		get { return halflength; }
	}

	/// <summary>
	/// Total distance driven along the track, counting every lap. Used for placements
	/// </summary>
	public float TrackProgress
	{
		get { return trackProgress; }
	}

	/// <summary>
	/// Is the car further from the middle of the road than the road is wide?
	/// </summary>
	public bool IsOffTrack
	{
		get
		{
			if (track == null) return false;
			Vector3 closest = Utils.SamplePathGlobal(track, Utils.GetClosestOffset(track, GlobalPosition));
			Vector2 gap = new Vector2(closest.X - GlobalPosition.X, closest.Z - GlobalPosition.Z);
			return gap.Length() > RoadHalfWidth;
		}
	}

	/// <summary>
	/// Has the race countdown finished? Cars can't move or turn until it has
	/// </summary>
	public bool HasRaceStarted
	{
		get { return raceStarted; }
	}

	// making end condition public so it can be reached in HUD
	public bool FinishedRace
	{
		get { return finishedRace; }
	}
	#endregion

	#region Methods

	/// <summary>
	/// Initialize any values upon load. If you're initializing this 
	/// through another script, please utilize the Init() method.
	/// </summary>
	public override void _Ready()
	{
		color = new Godot.Color("CYAN");
		current = States.Regular;
		halflength = new Vector3(radius, radius, radius); //TODO
	}

	/// <summary>
	/// Handle any forces or causes of speed change. 
	/// Call this method base._PhysicsProcess(delta) 
	/// in children AFTER handling input
	/// </summary>
	/// <param name="delta">Delta time</param>
	public override void _PhysicsProcess(double delta)
	{
		UpdateTrackProgress();

		// Vertical velocity
		if (!charbody3d.IsOnFloor()) // If in the air, fall towards the floor. Literally gravity
		{
			charbody3d.Position -= charbody3d.GetTransform().Basis.Y * (float)(delta * fallAcceleration);
			speed = 0;
		}

		if (current != States.Regular)
		{
			RevertState(delta);
		}

		// Moving the character
		charbody3d.Position += charbody3d.GetTransform().Basis.Z * (float)(delta * UpdateStateSpeed(current, speed)) * -1;

		charbody3d.MoveAndSlide();
	}


	/// <summary>
	/// Initialize the spawning position of the car 
	/// and set the reference to the stage's track
	/// </summary>
	/// <param name="startingPosition">Where is the car going to spawn on the track</param>
	/// <param name="track">Reference to the underlying path for the current stage's track</param>
	/// <param name="totalCheckpoints">How many checkpoints exist in the current track</param>
	/// <param name="facingDirection">Which way the car should face, along the track's direction of flow</param>
	public virtual void Init(Vector3 startingPosition, Path3D track, int totalCheckpoints, Vector3 facingDirection)
	{
		//spawn just above the track and let the car settle onto it
		charbody3d.GlobalPosition = startingPosition + new Vector3(0, SPAWNHEIGHT, 0);
		charbody3d.LookAt(charbody3d.GlobalPosition + facingDirection, Vector3.Up);
		this.track = track;
		this.totalCheckpoints = totalCheckpoints;

		//cars on the grid behind the start line are at the very end of the loop,
		//so count them as just behind the start instead of almost a full lap ahead
		float length = track.Curve.GetBakedLength();
		lastTrackOffset = Utils.GetClosestOffset(track, charbody3d.GlobalPosition);
		trackProgress = lastTrackOffset > length / 2 ? lastTrackOffset - length : lastTrackOffset;
	}

	/// <summary>
	/// Adds however far the car moved along the track since last frame to its track progress
	/// </summary>
	private void UpdateTrackProgress()
	{
		if (track == null) return;

		float length = track.Curve.GetBakedLength();
		float offset = Utils.GetClosestOffset(track, charbody3d.GlobalPosition);
		float change = offset - lastTrackOffset;
		//crossing the start line wraps the offset around between the end and 0
		if (change > length / 2) change -= length;
		else if (change < -length / 2) change += length;

		trackProgress += change;
		lastTrackOffset = offset;
	}

	/// <summary>
	/// Puts the car back in the middle of the road at the closest point on the 
	/// track, facing the direction the track flows. For when a car leaves the track
	/// </summary>
	public void ResetToTrack()
	{
		if (track == null) return;

		float offset = Utils.GetClosestOffset(track, charbody3d.GlobalPosition);
		Vector3 point = Utils.SamplePathGlobal(track, offset);
		//a little further along the track, to know which way it flows here
		Vector3 forward = Utils.SamplePathGlobal(track, offset + 2 / track.GlobalTransform.Basis.Scale.X) - point;
		forward.Y = 0;

		charbody3d.GlobalPosition = point + new Vector3(0, SPAWNHEIGHT, 0);
		charbody3d.LookAt(charbody3d.GlobalPosition + forward.Normalized(), Vector3.Up);
		speed = 0;
	}

	#region methods regarding the car's state
	/// <summary>
	/// Calculates the speed value for the final position change 
	/// without modifying the original speed value. 
	/// </summary>
	/// <param name="state">What state is the car in currently</param>
	/// <param name="speed">How fast is the car supposedly going</param>
	/// <returns>The final speed value to be fed to the car</returns>
	protected virtual double UpdateStateSpeed(States state, double speed)
	{
		switch (state)
		{
			//multiply speed
			case States.Fast:
				speed *= fastMultiplier;
				break;
			//divide speed
			case States.Slow:
				speed *= slowMultiplier;
				break;
			//flip flop controls
			case States.Inverted:
				speed *= -1;
				break;
		}
		//stop moving if you've completed all three laps
		//or ignore all movement while the race has not started 
		if (finishedRace || !raceStarted) speed *= 0;
		//GD.Print("speed: " + speed);
		return speed;
	}

	/// <summary>
	/// Counts down the current item effect. Once it runs 
	/// out, the car's state reverts back to States.Regular.
	/// </summary>
	/// <param name="delta">deltatime</param>
	protected void RevertState(double delta)
	{
		effectTimeRemaining -= delta;
		if (effectTimeRemaining <= 0)
		{
			current = States.Regular;
			ClearTimer();
		}
	}

	/// <summary>
	/// How long the given item effect lasts in seconds
	/// </summary>
	protected static double GetEffectDuration(States state)
	{
		return state switch
		{
			States.Fast => FASTDURATION,
			States.Slow => SLOWDURATION,
			States.Inverted => INVERTEDDURATION,
			_ => 0,
		};
	}

	/// <summary>
	/// Resets the timer associated with the state transition logic
	/// </summary>
	public void ClearTimer()
	{
		effectTimeRemaining = 0;
	}
	#endregion

	#region methods regarding the car's checkpoint tracking
	/// <summary>
	/// If the car has crossed one of the checkpoints placed 
	/// around the track, add that specific checkpoint to the 
	/// car's list of passed checkpoints this lap
	/// </summary>
	/// <param name="chpt">The current checkpoint passed</param>
	public void AddCheckpoint(Checkpoint chpt, Car c)
	{
		if (c != this) return;
		//ensure the checkpoint does not exist in the list before adding 
		if (!passedCheckpoints.Contains(chpt)) 
		{
			passedCheckpoints.Add(chpt);
			GD.Print("Adding Checkpoint: " + chpt);
		}
	}

	/// <summary>
	/// Checks if this car has gone through all checkpoints on the track
	/// </summary>
	/// <returns>True if all checkpoints exist in this "passedCheckpoints" list</returns>
	public bool CheckCheckpoints()
	{
		if (passedCheckpoints.Count >= totalCheckpoints) return true;
		else return false;
	}

	/// <summary>
	/// Clears the list of checkpoints passed. Called after CheckCheckpoints() returns true
	/// </summary>
	public void ClearCheckpoints(Car c)
	{
		if(c != this) return;
		passedCheckpoints.Clear();
		GD.Print("Clearing Checkpoints");
	}

	#endregion

	/// <summary>
	/// Increment the lap counter. This is called upon
	/// passing the finishline collision check
	/// </summary>
	public void IncrementLap(Car c)
	{
		if (c != this) return;
		if (lap + 1 > 3) finishedRace = true;
		else lap++;

	}

	/// <summary>
	/// Changes the placement of the car given a int
	/// </summary>
	/// <param name="i">What place is the car in overall</param>
	public void PlacementChanged(int i)
	{
		place = i;
	}

	virtual protected void ApplySavedCarColor() { }
	virtual protected void SetCarColor(Color color) { }

	/// <summary>
	/// Applies the chosen item effect to this car and starts the effect timer.
	/// Overrides any effect that is already active.
	/// Fast/Slow are applied through UpdateStateSpeed, so maxSpeed is left alone
	/// (scaling both used to double the effect and wiped the AI's difficulty speed)
	/// </summary>
	public void ApplyItemEffect(States item)
	{
		if (item == States.Regular)
		{
			//GD.Print("Invalid item state.");
			return;
		}

		current = item;
		effectTimeRemaining = GetEffectDuration(item);
		//GD.Print($"{Name}: activated {item} item");
	}

	/// <summary>
	/// Takes the stored item out of this car's item slot.
	/// </summary>
	/// <returns>The item that was stored, or States.Regular if the slot was empty</returns>
	public States TakeStoredItem()
	{
		States item = storedItem;
		storedItem = States.Regular;
		return item;
	}

	/// <summary>
	/// Stores an item. Overrides any existing stored item.
	/// </summary>
	public void StoreItem(States newItem)
	{
		storedItem = newItem;
		//GD.Print($"Stored new item: {storedItem}");
	}

	#endregion

	/// <summary>
	/// Flips internal boolean that the race has started, enabling cars the ability to move. 
	/// Call for all cars in car manager when the countdown has reached 0
	/// </summary>
	public void RaceStarted()
	{
		raceStarted = true;
	}

}

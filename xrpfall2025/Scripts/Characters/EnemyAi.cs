using System;
using Godot;
using Vector3 = Godot.Vector3;

public enum Difficulty
{
	Easy,
	Medium,
	Hard
};

/// <summary>
/// Class for Enemy cars. Inherits from parent Car class. 
/// Only difference between enemy and player is how movement
/// is handled and preset decision making
/// </summary>
public partial class EnemyAi : Car
{
	/// <summary>
	/// Different driving modes the Enemy AI would be in
	/// </summary>
	private enum RacingState
	{
		NormalDriving,
		Overtake,
		Attack,
		Defend,
		Recover

	}

	//will be used to simulate vision for more sophisticated driving behaviors
	[Export] RayCast3D raycast3d;
	//track what the car's raycast is "seeing" currently
	private GodotObject collided; 
	
	//player AI is rubberbanding off of
	private Player rubberbandingTo; 
	private RacingState racingState = RacingState.NormalDriving;
	private Difficulty difficulty;
	//the point in the middle of the road the ai is steering towards
	private Vector3 seekPoint;
	//how far ahead along the middle of the road (in world units) the ai aims.
	//Following the road's curve instead of cutting between track points keeps it on the track
	//Tuned on real_track.tscn so the real-size kart stays inside the 0.44 m lane, including
	//the tighter inside lane of the turns (a longer look ahead cuts across the corner)
	[Export] private float lookAhead = 12;
	//fraction of the angle to the look ahead point the ai turns each physics frame
	[Export] private float steeringGain = 0.2f;
	//how far right (negative = left) of the middle of the road this car drives, in world units.
	//Each AI keeps the side of the road it starts on in the grid, so AI karts never drive
	//through each other (cars have no collision with each other)
	private float laneOffset = 0;
	//furthest a lane can be from the middle of the road, as a fraction of a kart's width.
	//0.7 keeps two karts in opposite lanes apart while leaving room on the inside of turns
	private const float MAXLANEOFFSET = 0.7f;

	// is this car going to rubberband to the player? 
	// currently assumes only one player in the scene, 
	// cooresponding logic will need to be modified if 
	// multiplayer ever gets implemented
	private bool isRubberbanding = false;

	//should this car use the items it picks up
	[Export] private bool useItems = true;
	//seconds left before this car uses its stored item. Negative when not holding one
	private double itemHoldTime = -1;

	/// <summary>
	/// Get/Set if this enemy ai car is rubberbanding to the player
	/// </summary>
	public bool IsRubberbanding 
	{ 
		get { return isRubberbanding;}
		set { isRubberbanding = value;}
	}

	/// <summary>
	/// Initialize a Enemy AI 
	/// </summary>
	/// <param name="startingPosition">Where are we first spawning</param>
	/// <param name="track">Reference to the track's path</param>
	/// <param name="totalCheckpoints">How many checkpoints are on the track</param>
	/// <param name="facingDirection">Which way the car should face, along the track's direction of flow</param>
	public void Init(Vector3 startingPosition, Path3D track, int totalCheckpoints, Vector3 facingDirection, Difficulty difficulty = Difficulty.Medium)
	{
		base.Init(startingPosition, track, totalCheckpoints, facingDirection);
		laneOffset = GetStartingLaneOffset();
		seekPoint = GetLookAheadPoint();
		racingState = RacingState.NormalDriving;
		this.difficulty = difficulty;
		SetSpeed(difficulty);
		raycast3d.Show();
	}


	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		base._Ready();
	}

	// Handle any in-game movement and effecting physics on this EnemyAI body
	public override void _PhysicsProcess(double delta)
	{
		//car cannot drive while the race has not started, but still lets gravity
		//settle it onto the track (UpdateStateSpeed zeroes speed until then)
		if (!raceStarted)
		{
			base._PhysicsProcess(delta);
			return;
		}

		HandleStoredItem(delta);
		// //get what the ray cast is looking at
		// if(raycast3d.IsColliding()) 
		// {
		// 	collided = raycast3d.GetCollider();
		// 	GD.Print("I'm seeing " + collided.Get(Name));
		// }

		// if rubberbanding is on, depending on the distance from 
		// the player in terms of placements and physical gap distance,
		// adjust speed accordingly. This boolean is adjusted in the 
		// Car manager. Not every car needs to be rubberbanding to the player
		if (isRubberbanding)
		{
			//dot product to determine whether the target is in front or behind
			float dot = charbody3d.GlobalTransform.Basis.Z.Dot(rubberbandingTo.GlobalPosition);
			//float angleToTarget = charbody3d.GlobalTransform.Basis.Z.SignedAngleTo(rubberbandingTo.GlobalPosition, Vector3.Forward);
			//if is significantly in front of the player, slow down a bit to close the gap
			if (dot > 0)
			{
				if (difficulty != Difficulty.Easy) difficulty--;
			}
			//if is significantly behind the player, speed up to close the gap
			else
			{
				if (difficulty != Difficulty.Hard) difficulty++;
			}

			isRubberbanding = false;
			rubberbandingTo = null;
		}

		//get input based on racing state behavior
		switch (racingState)
		{
			case RacingState.NormalDriving: 
				NormalDriving(delta); 
				break;
			case RacingState.Overtake: 
				Overtake(delta); 
				break;
			case RacingState.Attack: 
				Attack(delta); 
				break;
			case RacingState.Defend: 
				Defend(delta); 
				break;
			case RacingState.Recover: 
				Recover(delta); 
				break;
		}

		//process input
		base._PhysicsProcess(delta);
	}

	/// <summary>
	/// Mimic a car just driving along the path with 
	/// no other intentions like sabotoge or defense
	/// </summary>
	/// <param name="delta">Deltatime</param>
	private void NormalDriving(double delta)
	{
		seekPoint = GetLookAheadPoint();
		DebugDraw3D.DrawSphere(seekPoint, 2, color);
		//need to get the correct angle for the car to turn
		Vector3 targetVector = seekPoint - GlobalPosition;
		targetVector.Y = 0;
		//measured from the kart's forward (-Z) axis: 0 when facing the target, positive when it's to the left.
		//(Measuring from Basis.Z, the backward axis, flipped between +180 and -180 degrees every frame and made the kart jitter)
		Vector3 forward = -charbody3d.GlobalTransform.Basis.Z;
		forward.Y = 0;
		float angle = forward.SignedAngleTo(targetVector, Vector3.Up);
		//GD.Print("EnemyAI angle: ", angle);
		rotationIncrement = angle * steeringGain;
		rotationIncrement = Mathf.Clamp(rotationIncrement, -1, 1);
		//when hit by an Inverted item, swerve back and forth instead of driving backwards
		if (current == States.Inverted)
		{
			rotationIncrement += Mathf.Sin(Time.GetTicksMsec() * 0.008f) * 0.04f;
		}

		//rotate to look at the next node 
		charbody3d.RotateObjectLocal(
			new Vector3(0, 1, 0), 
			(float)rotationIncrement
		);
		
		// //move towards the node when the car is facing the node well enough
		//if (raycast3d.CollideWithAreas) speed *= 0.9 * delta; 
		if (speed < maxSpeed ) 
		{
			speed += acceleration * delta;
		}
		//apply to movement -> call in base.physicsprocess
	}

	/// <summary>
	/// The point in this car's lane lookAhead units further along 
	/// the track from wherever this car currently is
	/// </summary>
	private Vector3 GetLookAheadPoint()
	{
		//the curve is in the track's local units, which are scaled up in the world
		float ahead = Utils.GetClosestOffset(track, GlobalPosition) + lookAhead / track.GlobalTransform.Basis.Scale.X;
		return Utils.SamplePathGlobal(track, ahead) + GetTrackRight(ahead) * laneOffset;
	}

	/// <summary>
	/// Direction pointing to the right of the road at the given offset along the track
	/// </summary>
	private Vector3 GetTrackRight(float offset)
	{
		Vector3 here = Utils.SamplePathGlobal(track, offset);
		Vector3 further = Utils.SamplePathGlobal(track, offset + 1 / track.GlobalTransform.Basis.Scale.X);
		Vector3 forward = further - here;
		forward.Y = 0;
		return forward.Normalized().Cross(Vector3.Up);
	}

	/// <summary>
	/// How far right of the middle of the road this car spawned, limited so the whole kart stays on the road
	/// </summary>
	private float GetStartingLaneOffset()
	{
		float offset = Utils.GetClosestOffset(track, GlobalPosition);
		Vector3 fromMiddle = GlobalPosition - Utils.SamplePathGlobal(track, offset);
		//the kart's collision box is ready right away, unlike the CSG box behind AABB
		float kartWidth = (charbody3d.GetNode<CollisionShape3D>("CollisionShape3D").Shape as BoxShape3D)?.Size.X ?? 0;
		float maxOffset = Mathf.Min(kartWidth * MAXLANEOFFSET, Mathf.Max(0, RoadHalfWidth - kartWidth / 2 - 1));
		return Mathf.Clamp(fromMiddle.Dot(GetTrackRight(offset)), -maxOffset, maxOffset);
	}

	/// <summary>
	/// When the car is at a bend and a car is in front of this car, attempt a tight angle
	/// Only implement behavior if on hard
	/// </summary>
	/// <param name="delta"></param>
	private void Overtake(double delta)
	{
		
	}

	/// <summary>
	/// If the car is holding a projectile item and a car is ahead, attempt to hit
	/// </summary>
	/// <param name="delta"></param>
	private void Attack(double delta)
	{
		//projectile items have not been implemented yet
	}

	/// <summary>
	/// If a car is behind and attempting to overtaking this car, try to block the incoming car
	/// Only implement behavior if on hard
	/// </summary>
	/// <param name="delta"></param>
	private void Defend(double delta)
	{
		
	}

	/// <summary>
	/// If the car has veered significantly off course, find its way back to the main track
	/// This behavior assumes there is no risk of falling off a track 
	/// </summary>
	/// <param name="delta"></param>
	private void Recover(double delta)
	{
		
	}

	/// <summary>
	/// adjusts the max speed of a ai car according to their set difficulty
	/// </summary>
	/// <param name="difficulty">what difficulty is the car set to</param>
	private void SetSpeed(Difficulty difficulty)
	{
		switch (difficulty)
		{
			case Difficulty.Easy:
			maxSpeed = 10;
			break;

			case Difficulty.Medium:
			maxSpeed = 20;
			break;

			case Difficulty.Hard:
			maxSpeed = 30;
			break;
		}
	}

	/// <summary>
	/// Holds on to a picked up item for a few seconds, then uses it
	/// </summary>
	/// <param name="delta">Deltatime</param>
	private void HandleStoredItem(double delta)
	{
		if (!useItems || finishedRace || storedItem == States.Regular)
		{
			itemHoldTime = -1;
			return;
		}

		//just picked up an item, decide how long to hold it
		if (itemHoldTime < 0) itemHoldTime = GD.RandRange(2.0, 6.0);

		itemHoldTime -= delta;
		if (itemHoldTime <= 0)
		{
			itemHoldTime = -1;
			(GetParent() as CarManager)?.UseItem(this);
		}
	}

	/// <summary>
	/// AI cars don't have controls to flip, so Inverted 
	/// makes them swerve (see NormalDriving) rather than reverse
	/// </summary>
	protected override double UpdateStateSpeed(States state, double speed)
	{
		if (state == States.Inverted) state = States.Regular;
		return base.UpdateStateSpeed(state, speed);
	}

	/// <summary>
	/// Sets the player the enemy ai is rubberbanding to
	/// </summary>
	/// <param name="p">Which player</param>
	public void IsRubberbandingTo(Player p)
	{
		rubberbandingTo = p;
	}

}

using Godot;
using System.Collections.Generic;
using System.Linq;

public partial class CarManager : Node
{
	private PackedScene playerPrefab = ResourceLoader.Load<PackedScene>("res://Scenes/Prefabs/player.tscn");
	private PackedScene enemyPrefab = ResourceLoader.Load<PackedScene>("res://Scenes/Prefabs/enemy_ai.tscn");
	
	private List<Car> cars = new List<Car>();
	private int numberOfEnemies = 0;
	private const int TOTALCARS = 3;
	private int numberOfPlayers = 1;
	//starting grid spacing: how far apart karts are side to side, and how far back each row is
	//Karts are real size (5.7 x 11.4 units on the 22 unit wide road), so the side slots
	//stay inside the lane and each row starts a kart length plus a gap behind
	private const float GRIDSIDESPACING = 7;
	private const float GRIDROWSPACING = 14;
	//keep track of the only player. will need to rewrite 
	// cooresponding logic if multiplayer is implemented
	private Player player; 
	//cars in the order they crossed the finish line on their last lap
	private List<Car> finishOrder = new List<Car>();

	/// <summary>
	/// Emitted whenever a car uses its stored item. 
	/// target is the car the effect was applied to (the user itself for Fast)
	/// </summary>
	[Signal] public delegate void ItemUsedEventHandler(Car user, int item, Car target);

	/// <summary>
	/// Get a reference to the list of cars in the game
	/// </summary>
	public List<Car> Cars
	{
		get {return cars;}
	}

	/// <summary>
	/// How many players are in the race
	/// </summary>
	public int TotalPlayers
	{
		get {return numberOfPlayers;}
	}

	public override void _Process(double delta)
	{
		//adjust the placements of each car
		List<Car> placements = GetPlacements();
		for (int i = 0; i < placements.Count; i++)
		{
			placements[i].PlacementChanged(i + 1);
		}

		//figure out which car is rubber banding to the player
		if (player.Place == 1) //first place
		{
			foreach (Car c in Cars)
			{
				if (c.Place == 2 && c is EnemyAi)
				{
					EnemyAi e = (EnemyAi)c;
					e.IsRubberbanding = true;
					e.IsRubberbandingTo(player);
					return;
				}
			}
		}
		else if (player.Place == Cars.Count) //last place
		{
			//have all cars rubber to the player for now
			foreach (Car c in Cars)
			{
				if (c is EnemyAi)
				{
					EnemyAi e = (EnemyAi)c;
					e.IsRubberbanding = true;
					e.IsRubberbandingTo(player);
				}
			}
		}
		else //middle of the pack
		{
			int i = 0;
			//grab a car in front and behind the player
			foreach (Car c in Cars)
			{
				if (
					(c.Place == player.Place + 1 || c.Place == player.Place - 1) 
					&& c is EnemyAi
				)
				{
					EnemyAi e = (EnemyAi)c;
					e.IsRubberbanding = true;
					e.IsRubberbandingTo(player);
					i++;
				}
				if (i >= 2) return;
			}
		}
	}

	/// <summary>
	/// Initializes all cars for the new track
	/// </summary>
	/// <param name="startingPosition">Where will the cars be spawning</param>
	/// <param name="track">Reference to the path underlying the current track</param>
	/// <param name="totalCheckpoints">How many checkpoints exist on the current track</param>
	public void Init(Vector3 startingPosition, Path3D track, int totalCheckpoints)
	{
		numberOfEnemies = TOTALCARS - numberOfPlayers;

		//direction the track runs out of the start line, flattened onto the floor
		Vector3 forward = track.ToGlobal(track.Curve.GetPointPosition(1)) - track.ToGlobal(track.Curve.GetPointPosition(0));
		forward.Y = 0;
		forward = forward.Normalized();

		//spawn all players. Grid slot 0 is on the start line, which is
		//where the real XRP's (0,0) gets mapped to
		for (int i = 0; i < numberOfPlayers; i++)
		{
			cars.Add((Car)playerPrefab.Instantiate());
			AddChild(cars[^1]);
			cars[^1].Init(GetGridPosition(i, startingPosition, forward), track, totalCheckpoints, forward);
			player = (Player)cars[^1];
		}

		//spawn all enemies in the slots behind the players
		for(int i = 0; i < numberOfEnemies; i++)
		{
			cars.Add((Car)enemyPrefab.Instantiate());
			AddChild(cars[^1]);
			((EnemyAi)cars[^1]).Init(GetGridPosition(numberOfPlayers + i, startingPosition, forward), track, totalCheckpoints, forward);
		}

	}

	/// <summary>
	/// Where a kart starts on the starting grid. Slot 0 is centered on the start line,
	/// then each row behind it has a kart on the left and right
	/// </summary>
	/// <param name="slot">Which grid slot, starting at 0</param>
	/// <param name="startingPosition">Center of the start line</param>
	/// <param name="forward">Direction the track runs out of the start line</param>
	private static Vector3 GetGridPosition(int slot, Vector3 startingPosition, Vector3 forward)
	{
		if (slot == 0) return startingPosition;

		Vector3 right = forward.Cross(Vector3.Up);
		int row = (slot + 1) / 2;
		float side = slot % 2 == 1 ? -1 : 1;
		return startingPosition + right * side * GRIDSIDESPACING - forward * row * GRIDROWSPACING;
	}

	/// <summary>
	/// Uses the car's stored item. Fast boosts the user, Slow and 
	/// Inverted hit the car one place ahead (or second place if the user is in first)
	/// </summary>
	/// <param name="user">The car using its item</param>
	public void UseItem(Car user)
	{
		if (user.StoredItem == States.Regular) return;

		States item = user.StoredItem;
		Car target = item == States.Fast ? user : GetItemTarget(user);
		//nobody to hit, keep the item for later
		if (target == null) return;

		user.TakeStoredItem();
		target.ApplyItemEffect(item);
		EmitSignal(SignalName.ItemUsed, user, (int)item, target);
	}

	/// <summary>
	/// Which car an attacking item used by this car should hit
	/// </summary>
	private Car GetItemTarget(Car user)
	{
		int targetPlace = user.Place == 1 ? 2 : user.Place - 1;
		foreach (Car c in cars)
		{
			if (c != user && c.Place == targetPlace && !c.FinishedRace) return c;
		}
		return null;
	}

	/// <summary>
	/// Sorts the cars in order of their placement. Cars that finished come first 
	/// in the order they finished, then by laps, and finally by how far 
	/// they have actually driven along the track
	/// </summary>
	private List<Car> GetPlacements()
	{
		foreach (Car c in cars)
		{
			if (c.FinishedRace && !finishOrder.Contains(c)) finishOrder.Add(c);
		}

		return cars
			.OrderBy(c => c.FinishedRace ? finishOrder.IndexOf(c) : finishOrder.Count)
			.ThenByDescending(c => c.Lap)
			.ThenByDescending(c => c.TrackProgress)
			.ToList();
	}

}

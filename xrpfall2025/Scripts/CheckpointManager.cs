using Godot;
using System.Collections.Generic;
using Godot.Collections;


public delegate void CheckpointCollisionDelegate(Checkpoint chpt, Car car);

/// <summary>
/// Handles creating and deleting all checkpoints on a track
/// </summary>
public partial class CheckpointManager : Node
{
	private PackedScene checkpointPrefab = ResourceLoader.Load<PackedScene>("res://Scenes/Prefabs/checkpoint.tscn");
	private List<Car> cars;
	private Array<Checkpoint> checkpointList = new Array<Checkpoint>();

	/// <summary>
	/// How many checkpoints are on the track.
	/// </summary>
	public int TotalCheckpoints
	{
		get{ return checkpointList.Count; }
	}

	/// <summary>
	/// Need to create all checkpoints in the scene first, 
	/// then init the car manager in the scene manager, 
	/// and finally init checkpoint manager
	/// </summary>
	/// <param name="checkpoints">Global locations of where checkpoints will spawn at.</param>
	public void SpawnCheckpoints(Array<Vector3> checkpoints)
	{
		for (int i = 0; i < checkpoints.Count; i++)
		{
			checkpointList.Add((Checkpoint)checkpointPrefab.Instantiate());
			AddChild(checkpointList[^1]);
		}
	}

	/// <summary>
	/// Removes all existing checkpoints in the scene
	/// </summary>
	public void DeleteAllCheckpoints()
	{
		while(checkpointList.Count > 0)
		{
			checkpointList[0].Dispose();
		}
	}

	/// <summary>
	/// Initialize the references to players and checkpoint 
	/// positions based on track points and generate the checkpoints
	/// </summary>
	/// <param name="cars">Reference to list of all existing cars</param>
	/// <param name="checkpoints">List of points where we want to instantiate our checkpoints</param>
	public void Init(List<Car> cars, Array<Vector3> checkpoints)
	{
		this.cars = cars;
		Checkpoint temp;

		for (int i = 0; i < checkpoints.Count; i++)
		{
			temp = checkpointList[i];
			temp.Init(checkpoints[i], cars, 20);
			foreach (Car c in cars)
			{
				temp.OnCheckpointCollision += c.AddCheckpoint;
			}
			
			GD.Print("making checkpoint: " + temp);
		}
	}
}

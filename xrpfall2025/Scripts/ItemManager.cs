using Godot;
using System;
using System.Collections.Generic;

//delegate declaration
public delegate void ItemCollisionDelegate(Car c);

/// <summary>
/// Handles spawning all items on the track 
/// </summary>
public partial class ItemManager : Node
{
	//grab a reference to all cars in scene
	//Every item generated will get a reference to all cars' bounds 
	//Used in collision detection & item event declaration
	private List<Car> cars;
	private PackedScene itemPrefab = ResourceLoader.Load<PackedScene>("res://Scenes/Prefabs/item.tscn");

	//list of all items generated
	private List<Node> items = null;
	private Path3D trackPath = null;

	[Export] private int itemLocations = 1;
	[Export] private int itemsPerLocation = 1;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		//set up all the item nodes before the scene starts
		items = new List<Node>();
		itemLocations = trackPath.Curve.PointCount;

		GenerateItems(/*player*/cars);
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	/// <summary>
	/// Set references to all cars and track
	/// </summary>
	/// <param name="cars">List of all cars</param>
	/// <param name="t">Reference to the track</param>
	public void Init(List<Car> cars, Path3D tp)
	{
		this.cars = cars;
		trackPath = tp;

	}

	/// <summary>
	/// Instantiates items scattered throughout the track 
	/// </summary>
	/// <param name="cars">Necessary for each car to be hooked 
	/// up to a collision event with each item spawned </param>
	public void GenerateItems(List<Car> cars)
	{
		for (int i = 0; i < itemLocations-1; i++)
		{
			//calculate where on the track we want to generate this row of items
			//double theta = Mathf.DegToRad(i * 360 / itemLocations);
			Vector3 spawnPos = trackPath.ToGlobal(trackPath.Curve.GetPointPosition(i + 1)) + new Vector3(0, 1, 0);
			//the last point of a closed track sits back on the start line, skip it
			Vector3 start = trackPath.ToGlobal(trackPath.Curve.GetPointPosition(0));
			if (new Vector2(spawnPos.X - start.X, spawnPos.Z - start.Z).Length() < 1) continue;

			//get direction vector towards the origin from this point, flat along the ground
			Vector3 dir = new Vector3(-spawnPos.X, 0, -spawnPos.Z).Normalized();
			//determine the direction to generate the items in
			for (int j = 0; j < itemsPerLocation; j++)
			{
				Item temp;
				items.Add(itemPrefab.Instantiate()); //create the new item
				AddChild(items[^1]); //add to the scene tree
				temp = (Item)items[^1]; //grab reference to call its init()
				temp.CustomInit(cars, spawnPos + (5 * dir));

			}			
		}
	}

}

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

	//boxes in each row, spread evenly across the road
	[Export] private int itemsPerRow = 4;
	//how far past the finish line the first row sits, in road widths
	private const float ROWPASTLINE = 0.5f;
	//how much of the road's width a row covers (1 = edge to edge)
	private const float ROWSPREAD = 0.7f;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		//set up all the item nodes before the scene starts
		items = new List<Node>();

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
	/// Instantiates two rows of items across the road: one just past the
	/// finish line and one at the halfway point of the lap
	/// </summary>
	/// <param name="cars">Necessary for each car to be hooked 
	/// up to a collision event with each item spawned </param>
	public void GenerateItems(List<Car> cars)
	{
		Curve3D curve = trackPath.Curve;
		float length = curve.GetBakedLength();
		//the track is scaled up after it's built, so world distances are converted
		//back to the path's own units
		float pathScale = trackPath.GlobalTransform.Basis.Scale.X;
		float pastLine = ROWPASTLINE * 2 * Car.RoadHalfWidth / pathScale;

		SpawnRow(cars, curve, pastLine, length);
		SpawnRow(cars, curve, length / 2, length);
	}

	/// <summary>
	/// A row of itemsPerRow boxes across the road, offset along the path
	/// </summary>
	private void SpawnRow(List<Car> cars, Curve3D curve, float offset, float length)
	{
		Vector3 centre = trackPath.ToGlobal(curve.SampleBaked(offset));
		Vector3 ahead = trackPath.ToGlobal(curve.SampleBaked(Mathf.Min(offset + length * 0.002f, length)));
		Vector3 forward = new Vector3(ahead.X - centre.X, 0, ahead.Z - centre.Z).Normalized();
		Vector3 right = forward.Cross(Vector3.Up);
		float halfSpread = Car.RoadHalfWidth * ROWSPREAD;

		for (int j = 0; j < itemsPerRow; j++)
		{
			//evenly spaced from one side of the row to the other
			float across = itemsPerRow == 1 ? 0 : -halfSpread + 2 * halfSpread * j / (itemsPerRow - 1);
			Item temp = (Item)itemPrefab.Instantiate();
			items.Add(temp);
			AddChild(temp);
			temp.CustomInit(cars, centre + right * across + new Vector3(0, 1, 0));
		}
	}

}

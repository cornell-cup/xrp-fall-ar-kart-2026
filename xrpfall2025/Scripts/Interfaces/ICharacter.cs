using Godot;
using System;

/// <summary>
/// Interface for any characters. In this case, player, enemy ai
/// </summary>
interface ICharacter
{
    
    public States Current { get; set; } //current state
    
    //speed after taking into account Current state
    public double CurrentSpeed { get; }
    public bool FinishedRace { get; }

    void Init(Vector3 startingPosition, Path3D track, int totalCheckpoints);

    double UpdateStateSpeed(States state, double speed);

    void RevertState(States prevState, double speed, double delta) { }

    void StartTimer();
    void ClearTimer();
    void ResetToTrack();
    void IncrementLap();
    void AddCheckpoint(Checkpoint chpt);
    bool CheckCheckpoints();
    void ClearCheckpoints();

}

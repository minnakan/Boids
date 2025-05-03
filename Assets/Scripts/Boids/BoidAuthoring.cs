using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

// The IComponentData for our Boid
public struct BoidComponent : IComponentData
{
    // Current velocity of the boid
    public float3 Velocity;

    public float MaxSpeed;

    public float MaxSteeringForce;

    // Weights for the three boid rules
    public float SeparationWeight;
    public float AlignmentWeight;
    public float CohesionWeight;

    // Perception radius for detecting other boids
    public float PerceptionRadius;
}

// This MonoBehaviour is used to author Boid entities in the Editor
public class BoidAuthoring : MonoBehaviour
{
    public Vector3 InitialVelocity = new Vector3(0, 0, 1);

    public float MaxSpeed = 5f;

    public float MaxSteeringForce = 0.5f;

    // Weights for the three boid rules
    public float SeparationWeight = 1.5f;
    public float AlignmentWeight = 1.0f;
    public float CohesionWeight = 1.0f;

    public float PerceptionRadius = 5f;

    class BoidBaker : Baker<BoidAuthoring>
    {
        public override void Bake(BoidAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity,new BoidComponent
            {
                Velocity = new float3(authoring.InitialVelocity.x, authoring.InitialVelocity.y, authoring.InitialVelocity.z),
                MaxSpeed = authoring.MaxSpeed,
                MaxSteeringForce = authoring.MaxSteeringForce,
                SeparationWeight = authoring.SeparationWeight,
                AlignmentWeight = authoring.AlignmentWeight,
                CohesionWeight = authoring.CohesionWeight,
                PerceptionRadius = authoring.PerceptionRadius
            });
        }
    }
}
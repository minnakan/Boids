using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

// Component to store boundary configuration
public struct BoidBoundaryData : IComponentData
{
    public float3 MinBounds;
    public float3 MaxBounds;
    public float BoundaryForce;
    public float BoundaryDistance;
}

// Authoring component for the boundary configuration
public class BoidBoundaryAuthoring : MonoBehaviour
{
    // The bounds of the area where boids can fly
    public Vector3 MinBounds = new Vector3(-20f, -20f, -20f);
    public Vector3 MaxBounds = new Vector3(20f, 20f, 20f);

    // Force applied when approaching boundaries
    public float BoundaryForce = 1.0f;

    // Distance from boundary where force starts being applied
    public float BoundaryDistance = 5.0f;

    // Display the boundaries in the editor
    private void OnDrawGizmos()
    {
        // Draw a wireframe box to represent the boundaries
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireCube(
            (MaxBounds + MinBounds) * 0.5f,
            MaxBounds - MinBounds
        );

        // Draw the inner boundary where force starts being applied
        Gizmos.color = new Color(1f, 0.5f, 0f, 0.3f);
        Vector3 innerSize = MaxBounds - MinBounds - new Vector3(BoundaryDistance, BoundaryDistance, BoundaryDistance) * 2f;
        Gizmos.DrawWireCube(
            (MaxBounds + MinBounds) * 0.5f,
            innerSize
        );
    }

    // Baker to convert to entity component
    class BoidBoundaryBaker : Baker<BoidBoundaryAuthoring>
    {
        public override void Bake(BoidBoundaryAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.None);
            AddComponent(entity, new BoidBoundaryData
            {
                MinBounds = new float3(authoring.MinBounds),
                MaxBounds = new float3(authoring.MaxBounds),
                BoundaryForce = authoring.BoundaryForce,
                BoundaryDistance = authoring.BoundaryDistance
            });
        }
    }
}
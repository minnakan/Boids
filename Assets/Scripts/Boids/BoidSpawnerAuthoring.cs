using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Random = Unity.Mathematics.Random;

// Component tag to identify boid spawners
public struct BoidSpawnerTag : IComponentData
{
}

// Data component for spawner configuration
public struct BoidSpawnerProperties : IComponentData
{
    public Entity BoidPrefab;
    public int NumberToSpawn;
    public float3 SpawnBounds;
    public bool RandomizeVelocity;
    public float MinSpeed;
    public float MaxSpeed;
    public uint RandomSeed;
    public bool HasSpawned;
}

// MonoBehaviour to author spawner configuration
public class BoidSpawnerAuthoring : MonoBehaviour
{
    // The prefab with BoidAuthoring component
    public GameObject BoidPrefab;

    // Number of boids to spawn
    public int NumberOfBoids = 100;

    // Bounds for spawning boids
    public Vector3 SpawnBounds = new Vector3(10f, 10f, 10f);

    // Apply random velocity
    public bool RandomizeVelocity = true;

    // Range of initial velocity magnitude
    public Vector2 InitialSpeedRange = new Vector2(1f, 3f);

    // Random seed for reproducible results
    public uint RandomSeed = 1;

    // Baker to convert to entity component
    class BoidSpawnerBaker : Baker<BoidSpawnerAuthoring>
    {
        public override void Bake(BoidSpawnerAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);

            // Add tag component
            AddComponent<BoidSpawnerTag>(entity);

            // Add properties component
            AddComponent(entity, new BoidSpawnerProperties
            {
                BoidPrefab = GetEntity(authoring.BoidPrefab, TransformUsageFlags.Dynamic),
                NumberToSpawn = authoring.NumberOfBoids,
                SpawnBounds = new float3(authoring.SpawnBounds),
                RandomizeVelocity = authoring.RandomizeVelocity,
                MinSpeed = authoring.InitialSpeedRange.x,
                MaxSpeed = authoring.InitialSpeedRange.y,
                RandomSeed = authoring.RandomSeed,
                HasSpawned = false
            });
        }
    }
}

// System to spawn the boids
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial class BoidSpawnerSystem : SystemBase
{
    protected override void OnCreate()
    {
        // Require both tag and properties
        RequireForUpdate<BoidSpawnerTag>();
        RequireForUpdate<BoidSpawnerProperties>();
    }

    protected override void OnUpdate()
    {
        // Get an ECB for entity creation
        var ecbSystem = World.GetOrCreateSystemManaged<BeginInitializationEntityCommandBufferSystem>();
        var ecb = ecbSystem.CreateCommandBuffer();

        // Process each spawner entity
        Entities
            .WithoutBurst()
            .ForEach((Entity spawnerEntity, ref BoidSpawnerProperties spawnerProps) =>
            {
                // Skip if already spawned or prefab is invalid
                if (spawnerProps.HasSpawned || !EntityManager.Exists(spawnerProps.BoidPrefab))
                    return;

                // Create random number generator
                var random = Random.CreateFromIndex(spawnerProps.RandomSeed);

                // Spawn the boids
                for (int i = 0; i < spawnerProps.NumberToSpawn; i++)
                {
                    // Create entity from prefab
                    Entity boidEntity = ecb.Instantiate(spawnerProps.BoidPrefab);

                    // Generate random position within bounds
                    float3 position = new float3(
                        random.NextFloat(-spawnerProps.SpawnBounds.x, spawnerProps.SpawnBounds.x),
                        random.NextFloat(-spawnerProps.SpawnBounds.y, spawnerProps.SpawnBounds.y),
                        random.NextFloat(-spawnerProps.SpawnBounds.z, spawnerProps.SpawnBounds.z)
                    );

                    // Set the transform
                    ecb.SetComponent(boidEntity, new LocalTransform
                    {
                        Position = position,
                        Rotation = quaternion.Euler(0, random.NextFloat(0, math.PI * 2), 0),
                        Scale = 1f
                    });

                    // Add random velocity if needed
                    if (spawnerProps.RandomizeVelocity && EntityManager.HasComponent<BoidComponent>(spawnerProps.BoidPrefab))
                    {
                        // Get the base component from the prefab
                        var baseBoidComponent = EntityManager.GetComponentData<BoidComponent>(spawnerProps.BoidPrefab);

                        // Generate random direction
                        float3 direction = new float3(
                            random.NextFloat(-1f, 1f),
                            random.NextFloat(-1f, 1f),
                            random.NextFloat(-1f, 1f)
                        );

                        // Normalize and scale by random speed
                        direction = math.normalize(direction) *
                            random.NextFloat(spawnerProps.MinSpeed, spawnerProps.MaxSpeed);

                        // Update velocity
                        baseBoidComponent.Velocity = direction;

                        // Set the component on the new entity
                        ecb.SetComponent(boidEntity, baseBoidComponent);
                    }
                }

                // Mark as spawned
                spawnerProps.HasSpawned = true;

                // Log completion
                Debug.Log($"BoidSpawnerSystem: Spawned {spawnerProps.NumberToSpawn} boid entities!");
            }).Run();
    }
}
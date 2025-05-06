using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst.Intrinsics;
using Unity.Collections.LowLevel.Unsafe;

public struct SpatialGrid : IComponentData
{
    public float CellSize;
    public int3 GridDimensions;
    public int TotalCells;
}

// System to manage the spatial grid
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial class SpatialGridSystem : SystemBase
{
    protected override void OnCreate()
    {
        // Create a singleton entity with spatial grid data
        var entity = EntityManager.CreateEntity();
        EntityManager.AddComponentData(entity, new SpatialGrid
        {
            CellSize = 5f,
            GridDimensions = new int3(20, 20, 20), 
            TotalCells = 20 * 20 * 20
        });
    }

    protected override void OnUpdate() { }
}

public struct BoidCellData : IComponentData
{
    public int CellIndex;
}

[BurstCompile]
public partial struct BoidSystem : ISystem
{
    
    private NativeArray<int> _cellCounts;
    private NativeArray<int> _cellStartIndices;
    private NativeArray<int> _boidIndices;
    private EntityQuery _boidQuery;
    private int _previousCapacity;

    
    [BurstCompile]
    private static int GetHashKey(in float3 position, float cellSize, in int3 gridDimensions)
    {
        int3 cellPos = new int3(
            math.clamp((int)math.floor(position.x / cellSize), 0, gridDimensions.x - 1),
            math.clamp((int)math.floor(position.y / cellSize), 0, gridDimensions.y - 1),
            math.clamp((int)math.floor(position.z / cellSize), 0, gridDimensions.z - 1)
        );

        return cellPos.x + cellPos.y * gridDimensions.x + cellPos.z * gridDimensions.x * gridDimensions.y;
    }

    [BurstCompile]
    private struct UpdateCellsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<LocalTransform> Transforms;
        [ReadOnly] public float CellSize;
        [ReadOnly] public int3 GridDimensions;

        public NativeArray<BoidCellData> CellData;

        public void Execute(int index)
        {
            float3 position = Transforms[index].Position;
            int cellIndex = GetHashKey(in position, CellSize, in GridDimensions);

            CellData[index] = new BoidCellData { CellIndex = cellIndex };
        }
    }

    [BurstCompile]
    private struct BuildCellLookupJob : IJob
    {
        [ReadOnly] public NativeArray<BoidCellData> CellData;
        [ReadOnly] public NativeArray<Entity> Entities;
        [ReadOnly] public int TotalCells;

        public NativeArray<int> CellCounts;
        public NativeArray<int> CellStartIndices;
        public NativeArray<int> BoidIndices;

        public void Execute()
        {
            // Reset cell counts - simple loop instead of unsafe memory operations
            for (int i = 0; i < CellCounts.Length; i++)
            {
                CellCounts[i] = 0;
            }

            // First pass: count boids per cell
            for (int i = 0; i < CellData.Length; i++)
            {
                int cellIndex = CellData[i].CellIndex;
                CellCounts[cellIndex]++;
            }

            // Second pass: calculate start indices
            int startIndex = 0;
            for (int i = 0; i < TotalCells; i++)
            {
                CellStartIndices[i] = startIndex;
                startIndex += CellCounts[i];

                // Reset counts for next phase
                CellCounts[i] = 0;
            }

            // Third pass: populate boid indices
            for (int i = 0; i < CellData.Length; i++)
            {
                int cellIndex = CellData[i].CellIndex;
                int index = CellStartIndices[cellIndex] + CellCounts[cellIndex];
                BoidIndices[index] = i;
                CellCounts[cellIndex]++;
            }
        }
    }

    // Main job to update boid behavior
    [BurstCompile]
    private struct BoidJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<BoidComponent> Boids;
        [ReadOnly] public NativeArray<LocalTransform> Transforms;
        [ReadOnly] public NativeArray<BoidCellData> CellData;
        [ReadOnly] public NativeArray<int> CellCounts;
        [ReadOnly] public NativeArray<int> CellStartIndices;
        [ReadOnly] public NativeArray<int> BoidIndices;
        [ReadOnly] public float CellSize;
        [ReadOnly] public int3 GridDimensions;
        [ReadOnly] public float DeltaTime;
        [ReadOnly] public float3 MinBounds;
        [ReadOnly] public float3 MaxBounds;
        [ReadOnly] public float BoundaryForce;
        [ReadOnly] public float BoundaryDistance;

        public NativeArray<BoidComponent> UpdatedBoids;
        public NativeArray<LocalTransform> UpdatedTransforms;

        public void Execute(int index)
        {
            var boid = Boids[index];
            var transform = Transforms[index];

            // Combine all force calculations to reduce redundant cell lookups
            CalculateForces(index, transform.Position, boid, out float3 separation, out float3 alignment, out float3 cohesion);

            // Apply weights with SIMD-friendly approach
            float3 steering = separation * boid.SeparationWeight + alignment * boid.AlignmentWeight + cohesion * boid.CohesionWeight;

            // Add boundary avoidance force
            steering += CalculateBoundaryForce(transform.Position);

            // Limit steering force efficiently
            float steeringSqrLen = math.lengthsq(steering);
            float maxSteeringForceSq = boid.MaxSteeringForce * boid.MaxSteeringForce;

            if (steeringSqrLen > maxSteeringForceSq)
            {
                steering *= math.rsqrt(steeringSqrLen) * boid.MaxSteeringForce;
            }

            boid.Velocity += steering;

            float speedSqrLen = math.lengthsq(boid.Velocity);
            float maxSpeedSq = boid.MaxSpeed * boid.MaxSpeed;
            if (speedSqrLen > maxSpeedSq)
            {
                boid.Velocity *= math.rsqrt(speedSqrLen) * boid.MaxSpeed;
            }

            transform.Position += boid.Velocity * DeltaTime;

            if (speedSqrLen > 0.001f)
            {
                float3 normalizedVelocity = boid.Velocity * math.rsqrt(speedSqrLen);
                transform.Rotation = quaternion.LookRotation(normalizedVelocity, new float3(0, 1, 0));
            }

            UpdatedBoids[index] = boid;
            UpdatedTransforms[index] = transform;
        }

        private float3 CalculateBoundaryForce(float3 position)
        {
            float3 boundarySteer = float3.zero;
            bool needsSteeringForce = false;

            if (position.x < MinBounds.x + BoundaryDistance)
            {
                boundarySteer.x = BoundaryForce * ((MinBounds.x + BoundaryDistance) - position.x) / BoundaryDistance;
                needsSteeringForce = true;
            }
            else if (position.x > MaxBounds.x - BoundaryDistance)
            {
                boundarySteer.x = BoundaryForce * ((MaxBounds.x - BoundaryDistance) - position.x) / BoundaryDistance;
                needsSteeringForce = true;
            }
            if (position.y < MinBounds.y + BoundaryDistance)
            {
                boundarySteer.y = BoundaryForce * ((MinBounds.y + BoundaryDistance) - position.y) / BoundaryDistance;
                needsSteeringForce = true;
            }
            else if (position.y > MaxBounds.y - BoundaryDistance)
            {
                boundarySteer.y = BoundaryForce * ((MaxBounds.y - BoundaryDistance) - position.y) / BoundaryDistance;
                needsSteeringForce = true;
            }
            if (position.z < MinBounds.z + BoundaryDistance)
            {
                boundarySteer.z = BoundaryForce * ((MinBounds.z + BoundaryDistance) - position.z) / BoundaryDistance;
                needsSteeringForce = true;
            }
            else if (position.z > MaxBounds.z - BoundaryDistance)
            {
                boundarySteer.z = BoundaryForce * ((MaxBounds.z - BoundaryDistance) - position.z) / BoundaryDistance;
                needsSteeringForce = true;
            }

            return needsSteeringForce ? boundarySteer : float3.zero;
        }

        private void CalculateForces(int currentIndex, float3 position, BoidComponent boid, out float3 separation, out float3 alignment, out float3 cohesion)
        {
            separation = float3.zero;
            alignment = float3.zero;
            cohesion = float3.zero;
            float3 centerOfMass = float3.zero;
            int separationCount = 0;
            int alignmentCount = 0;
            int cohesionCount = 0;
            float perceptionRadiusSq = boid.PerceptionRadius * boid.PerceptionRadius;

            // Get cell of current boid and only check neighboring cells once
            int currentCellIndex = CellData[currentIndex].CellIndex;
            int3 cellPos = new int3(
                currentCellIndex % GridDimensions.x,
                (currentCellIndex / GridDimensions.x) % GridDimensions.y,
                currentCellIndex / (GridDimensions.x * GridDimensions.y)
            );

            // Pre-compute cell neighbor indices to avoid triple nested loop
            for (int neighbor = 0; neighbor < 27; neighbor++)
            {
                // Convert linear index to 3D offset (-1, 0, 1) in each dimension
                int dx = neighbor % 3 - 1;
                int dy = (neighbor / 3) % 3 - 1;
                int dz = neighbor / 9 - 1;

                int3 checkCell = cellPos + new int3(dx, dy, dz);

                // Skip if outside grid
                if (checkCell.x < 0 || checkCell.x >= GridDimensions.x ||
                    checkCell.y < 0 || checkCell.y >= GridDimensions.y ||
                    checkCell.z < 0 || checkCell.z >= GridDimensions.z)
                {
                    continue;
                }

                int neighborCellIndex = checkCell.x + checkCell.y * GridDimensions.x + checkCell.z * GridDimensions.x * GridDimensions.y;

                int startIdx = CellStartIndices[neighborCellIndex];
                int cellCount = CellCounts[neighborCellIndex];

                // Check all boids in this cell
                for (int i = 0; i < cellCount; i++)
                {
                    int boidIdx = BoidIndices[startIdx + i];

                    if (boidIdx == currentIndex) continue;

                    float3 offset = position - Transforms[boidIdx].Position;
                    float distSq = math.lengthsq(offset);

                    // Combined force calculation to reduce duplicate checks
                    if (distSq < perceptionRadiusSq)
                    {
                        // Separation (only if non-zero distance)
                        if (distSq > 0.0001f)
                        {
                            // More efficient math: normalize and divide in one step
                            float invDist = math.rsqrt(distSq);
                            separation += offset * invDist * invDist; // Scale by 1/distSq for closer agents to have stronger effect
                            separationCount++;
                        }

                        // Alignment
                        alignment += Boids[boidIdx].Velocity;
                        alignmentCount++;

                        // Cohesion
                        centerOfMass += Transforms[boidIdx].Position;
                        cohesionCount++;
                    }
                }
            }

            // Process results with vectorized approach
            if (separationCount > 0)
                separation /= separationCount;

            if (alignmentCount > 0)
            {
                alignment /= alignmentCount;
                float alignmentMagSq = math.lengthsq(alignment);
                if (alignmentMagSq > 0.0001f)
                    alignment *= math.rsqrt(alignmentMagSq); // Normalize using rsqrt for better performance
            }

            if (cohesionCount > 0)
            {
                centerOfMass /= cohesionCount;
                cohesion = centerOfMass - position;
                float cohesionMagSq = math.lengthsq(cohesion);
                if (cohesionMagSq > 0.0001f)
                    cohesion *= math.rsqrt(cohesionMagSq); // Normalize
            }
        }
    }

    // Boundary parameters
    private float3 _minBounds;
    private float3 _maxBounds;
    private float _boundaryForce;
    private float _boundaryDistance;

    // Spatial grid parameters
    private SpatialGrid _spatialGrid;

    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<BoidComponent>();
        state.RequireForUpdate<SpatialGrid>();

        // Default boundary values
        _minBounds = new float3(-20f, -20f, -20f);
        _maxBounds = new float3(20f, 20f, 20f);
        _boundaryForce = 1.0f;
        _boundaryDistance = 5.0f;

        // Create the entity query directly with individual component types
        _boidQuery = state.GetEntityQuery(
            ComponentType.ReadWrite<BoidComponent>(),
            ComponentType.ReadWrite<LocalTransform>(),
            ComponentType.ReadWrite<BoidCellData>());

        _previousCapacity = 0;
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Check if boundary configuration exists and update values
        if (SystemAPI.TryGetSingleton<BoidBoundaryData>(out var boundaryConfig))
        {
            _minBounds = boundaryConfig.MinBounds;
            _maxBounds = boundaryConfig.MaxBounds;
            _boundaryForce = boundaryConfig.BoundaryForce;
            _boundaryDistance = boundaryConfig.BoundaryDistance;
        }

        // Get spatial grid configuration
        if (!SystemAPI.TryGetSingleton<SpatialGrid>(out _spatialGrid))
        {
            return;
        }

        int boidCount = _boidQuery.CalculateEntityCount();
        if (boidCount == 0)
            return;

        if (boidCount > _previousCapacity)
        {
            if (_previousCapacity > 0)
            {
                _cellCounts.Dispose();
                _cellStartIndices.Dispose();
                _boidIndices.Dispose();
            }

            int newCapacity = math.max(boidCount * 2, 1000);
            _cellCounts = new NativeArray<int>(_spatialGrid.TotalCells, Allocator.Persistent);
            _cellStartIndices = new NativeArray<int>(_spatialGrid.TotalCells, Allocator.Persistent);
            _boidIndices = new NativeArray<int>(newCapacity, Allocator.Persistent);
            _previousCapacity = newCapacity;
        }

        float deltaTime = SystemAPI.Time.DeltaTime;

        // Create efficient component type handles to process by chunk
        var transformTypeHandle = state.GetComponentTypeHandle<LocalTransform>(true); // ReadOnly
        var boidTypeHandle = state.GetComponentTypeHandle<BoidComponent>(true); // ReadOnly
        var cellDataTypeHandle = state.GetComponentTypeHandle<BoidCellData>(false); // ReadWrite
        var entityTypeHandle = state.GetEntityTypeHandle();

        // Create temporary arrays for job results
        var boidComponents = _boidQuery.ToComponentDataArray<BoidComponent>(Allocator.TempJob);
        var boidTransforms = _boidQuery.ToComponentDataArray<LocalTransform>(Allocator.TempJob);
        var boidCellData = _boidQuery.ToComponentDataArray<BoidCellData>(Allocator.TempJob);
        var entities = _boidQuery.ToEntityArray(Allocator.TempJob);

        // Create output arrays
        var updatedBoids = new NativeArray<BoidComponent>(boidCount, Allocator.TempJob);
        var updatedTransforms = new NativeArray<LocalTransform>(boidCount, Allocator.TempJob);

        // Step 1: Update cell assignments for each boid
        var updateCellsJob = new UpdateCellsJob
        {
            Transforms = boidTransforms,
            CellSize = _spatialGrid.CellSize,
            GridDimensions = _spatialGrid.GridDimensions,
            CellData = boidCellData
        };


        var updateCellsHandle = updateCellsJob.Schedule(boidCount, 64, state.Dependency);

        // Step 2: Build the cell lookup data structure
        var buildCellLookupJob = new BuildCellLookupJob
        {
            CellData = boidCellData,
            Entities = entities,
            TotalCells = _spatialGrid.TotalCells,
            CellCounts = _cellCounts,
            CellStartIndices = _cellStartIndices,
            BoidIndices = _boidIndices
        };

        var buildCellLookupHandle = buildCellLookupJob.Schedule(updateCellsHandle);

        // Step 3: Update boid behavior using spatial partitioning
        var boidJob = new BoidJob
        {
            Boids = boidComponents,
            Transforms = boidTransforms,
            CellData = boidCellData,
            CellCounts = _cellCounts,
            CellStartIndices = _cellStartIndices,
            BoidIndices = _boidIndices,
            CellSize = _spatialGrid.CellSize,
            GridDimensions = _spatialGrid.GridDimensions,
            DeltaTime = deltaTime,
            MinBounds = _minBounds,
            MaxBounds = _maxBounds,
            BoundaryForce = _boundaryForce,
            BoundaryDistance = _boundaryDistance,
            UpdatedBoids = updatedBoids,
            UpdatedTransforms = updatedTransforms
        };

        // Schedule with optimal batch size (tune this value for your hardware)
        var batchSize = 64; // Try values between 32-128 to find best performance
        var jobHandle = boidJob.Schedule(boidCount, batchSize, buildCellLookupHandle);
        jobHandle.Complete();

        // Write back the results
        for (int i = 0; i < boidCount; i++)
        {
            state.EntityManager.SetComponentData(entities[i], updatedBoids[i]);
            state.EntityManager.SetComponentData(entities[i], updatedTransforms[i]);
            state.EntityManager.SetComponentData(entities[i], boidCellData[i]);
        }

        // Dispose of temporary arrays
        entities.Dispose();
        boidComponents.Dispose();
        boidTransforms.Dispose();
        boidCellData.Dispose();
        updatedBoids.Dispose();
        updatedTransforms.Dispose();

        // Update state dependency
        state.Dependency = jobHandle;
    }


    public void OnDestroy(ref SystemState state)
    {
        if (_previousCapacity > 0)
        {
            _cellCounts.Dispose();
            _cellStartIndices.Dispose();
            _boidIndices.Dispose();
        }
    }
}
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;

[BurstCompile]
public partial struct BoidSystem : ISystem
{
    // Boundary parameters - can be made configurable later
    private float3 _minBounds;
    private float3 _maxBounds;
    private float _boundaryForce;
    private float _boundaryDistance;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<BoidComponent>();

        // Default boundary values
        _minBounds = new float3(-20f, -20f, -20f);
        _maxBounds = new float3(20f, 20f, 20f);
        _boundaryForce = 1.0f;
        _boundaryDistance = 5.0f;
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

        float deltaTime = SystemAPI.Time.DeltaTime;

        // Create entity query for all boids
        var allBoids = SystemAPI.QueryBuilder().WithAll<BoidComponent, LocalTransform>().Build();
        var boidEntities = allBoids.ToEntityArray(Allocator.Temp);
        var boidComponents = allBoids.ToComponentDataArray<BoidComponent>(Allocator.Temp);
        var boidTransforms = allBoids.ToComponentDataArray<LocalTransform>(Allocator.Temp);

        // Process each boid
        for (int i = 0; i < boidEntities.Length; i++)
        {
            var entity = boidEntities[i];
            var boid = boidComponents[i];
            var transform = boidTransforms[i];

            // Calculate the three classic boid forces
            float3 separation = CalculateSeparation(i, boidComponents, boidTransforms, boid.PerceptionRadius);
            float3 alignment = CalculateAlignment(i, boidComponents, boidTransforms, boid.PerceptionRadius);
            float3 cohesion = CalculateCohesion(i, boidComponents, boidTransforms, boid.PerceptionRadius);

            // Apply weights
            separation *= boid.SeparationWeight;
            alignment *= boid.AlignmentWeight;
            cohesion *= boid.CohesionWeight;

            // Calculate combined steering force
            float3 steering = separation + alignment + cohesion;

            // Add boundary avoidance force
            float3 boundaryForce = CalculateBoundaryForce(transform.Position);
            steering += boundaryForce;

            // Limit steering force
            if (math.lengthsq(steering) > boid.MaxSteeringForce * boid.MaxSteeringForce)
            {
                steering = math.normalize(steering) * boid.MaxSteeringForce;
            }

            // Update velocity
            boid.Velocity += steering;

            // Limit speed
            if (math.lengthsq(boid.Velocity) > boid.MaxSpeed * boid.MaxSpeed)
            {
                boid.Velocity = math.normalize(boid.Velocity) * boid.MaxSpeed;
            }

            // Update position
            transform.Position += boid.Velocity * deltaTime;

            // If velocity is not zero, update rotation to face direction of movement
            if (math.lengthsq(boid.Velocity) > 0.001f)
            {
                quaternion targetRotation = quaternion.LookRotation(math.normalize(boid.Velocity), new float3(0, 1, 0));
                transform.Rotation = targetRotation;
            }

            // Update the components
            state.EntityManager.SetComponentData(entity, boid);
            state.EntityManager.SetComponentData(entity, transform);
        }

        // Dispose of temporary arrays
        boidEntities.Dispose();
        boidComponents.Dispose();
        boidTransforms.Dispose();
    }

    private float3 CalculateBoundaryForce(float3 position)
    {
        float3 boundarySteer = float3.zero;
        bool needsSteeringForce = false;

        // Check X boundaries
        if (position.x < _minBounds.x + _boundaryDistance)
        {
            boundarySteer.x = _boundaryForce * ((_minBounds.x + _boundaryDistance) - position.x) / _boundaryDistance;
            needsSteeringForce = true;
        }
        else if (position.x > _maxBounds.x - _boundaryDistance)
        {
            boundarySteer.x = _boundaryForce * ((_maxBounds.x - _boundaryDistance) - position.x) / _boundaryDistance;
            needsSteeringForce = true;
        }

        // Check Y boundaries
        if (position.y < _minBounds.y + _boundaryDistance)
        {
            boundarySteer.y = _boundaryForce * ((_minBounds.y + _boundaryDistance) - position.y) / _boundaryDistance;
            needsSteeringForce = true;
        }
        else if (position.y > _maxBounds.y - _boundaryDistance)
        {
            boundarySteer.y = _boundaryForce * ((_maxBounds.y - _boundaryDistance) - position.y) / _boundaryDistance;
            needsSteeringForce = true;
        }

        // Check Z boundaries
        if (position.z < _minBounds.z + _boundaryDistance)
        {
            boundarySteer.z = _boundaryForce * ((_minBounds.z + _boundaryDistance) - position.z) / _boundaryDistance;
            needsSteeringForce = true;
        }
        else if (position.z > _maxBounds.z - _boundaryDistance)
        {
            boundarySteer.z = _boundaryForce * ((_maxBounds.z - _boundaryDistance) - position.z) / _boundaryDistance;
            needsSteeringForce = true;
        }

        // Only return a steering force if needed
        return needsSteeringForce ? boundarySteer : float3.zero;
    }

    private float3 CalculateSeparation(int currentIndex, NativeArray<BoidComponent> boids, NativeArray<LocalTransform> transforms, float perceptionRadius)
    {
        float3 steering = float3.zero;
        int count = 0;
        float perceptionRadiusSq = perceptionRadius * perceptionRadius;

        for (int i = 0; i < boids.Length; i++)
        {
            if (i == currentIndex) continue;

            float3 offset = transforms[currentIndex].Position - transforms[i].Position;
            float distSq = math.lengthsq(offset);

            if (distSq < perceptionRadiusSq && distSq > 0)
            {
                // The closer the boid, the stronger the separation force
                float3 diff = math.normalize(offset) / math.sqrt(distSq);
                steering += diff;
                count++;
            }
        }

        if (count > 0)
        {
            steering /= count;
        }

        return steering;
    }

    private float3 CalculateAlignment(int currentIndex, NativeArray<BoidComponent> boids, NativeArray<LocalTransform> transforms, float perceptionRadius)
    {
        float3 steering = float3.zero;
        int count = 0;
        float perceptionRadiusSq = perceptionRadius * perceptionRadius;

        for (int i = 0; i < boids.Length; i++)
        {
            if (i == currentIndex) continue;

            float distSq = math.lengthsq(transforms[currentIndex].Position - transforms[i].Position);

            if (distSq < perceptionRadiusSq)
            {
                steering += boids[i].Velocity;
                count++;
            }
        }

        if (count > 0)
        {
            steering /= count;
            steering = math.normalize(steering);
        }

        return steering;
    }

    private float3 CalculateCohesion(int currentIndex, NativeArray<BoidComponent> boids, NativeArray<LocalTransform> transforms, float perceptionRadius)
    {
        float3 steering = float3.zero;
        float3 centerOfMass = float3.zero;
        int count = 0;
        float perceptionRadiusSq = perceptionRadius * perceptionRadius;

        for (int i = 0; i < boids.Length; i++)
        {
            if (i == currentIndex) continue;

            float distSq = math.lengthsq(transforms[currentIndex].Position - transforms[i].Position);

            if (distSq < perceptionRadiusSq)
            {
                centerOfMass += transforms[i].Position;
                count++;
            }
        }

        if (count > 0)
        {
            centerOfMass /= count;
            float3 desired = centerOfMass - transforms[currentIndex].Position;
            if (math.lengthsq(desired) > 0)
            {
                steering = math.normalize(desired);
            }
        }

        return steering;
    }
}
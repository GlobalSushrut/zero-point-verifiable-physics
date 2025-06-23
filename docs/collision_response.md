# Collision Response System

## Overview

The collision response system handles how objects react after a collision is detected. The Zero Point Physics Engine provides multiple collision response methods optimized for different scenarios, from simple elastic bounces to complex military-grade simulations with conservation guarantees.

## Response Types

The engine supports several collision response methods:

```cpp
enum class CollisionResponseType {
    NONE,       // No response, just detection
    DISCRETE,   // Standard discrete collision response
    CONTINUOUS, // Continuous collision detection with response
    SPECULATIVE // Speculative contacts for stability
};
```

## Basic Impulse-Based Response

The standard collision response uses an impulse-based model:

```cpp
void resolveCollision(Node3D& node1, Node3D& node2, const CollisionInfo& info) {
    // Skip if both objects are static
    if (!node1.is_dynamic && !node2.is_dynamic) {
        return;
    }
    
    // Calculate relative velocity
    Vec3 relative_velocity = node2.velocity - node1.velocity;
    
    // Velocity along normal
    float vel_along_normal = relative_velocity.dot(info.collision_normal);
    
    // Skip if objects are separating
    if (vel_along_normal > 0) {
        return;
    }
    
    // Calculate restitution (bounciness)
    float restitution = std::min(node1.restitution, node2.restitution);
    
    // Calculate impulse scalar
    float inv_mass1 = node1.is_dynamic ? 1.0f / node1.mass : 0.0f;
    float inv_mass2 = node2.is_dynamic ? 1.0f / node2.mass : 0.0f;
    float impulse_scalar = -(1.0f + restitution) * vel_along_normal / 
                         (inv_mass1 + inv_mass2);
    
    // Apply impulse
    Vec3 impulse = info.collision_normal * impulse_scalar;
    
    if (node1.is_dynamic) {
        node1.velocity -= impulse * inv_mass1;
    }
    if (node2.is_dynamic) {
        node2.velocity += impulse * inv_mass2;
    }
}
```

## Friction Model

Collision response can include friction effects:

```cpp
void applyFriction(Node3D& node1, Node3D& node2, const CollisionInfo& info, 
                  float normal_impulse) {
    // Calculate relative velocity
    Vec3 relative_velocity = node2.velocity - node1.velocity;
    
    // Remove normal component to get tangential velocity
    Vec3 tangent_velocity = relative_velocity - 
                         (info.collision_normal * relative_velocity.dot(info.collision_normal));
    
    // Skip if negligible tangential velocity
    if (tangent_velocity.lengthSquared() < 0.0001f) {
        return;
    }
    
    // Calculate tangent vector
    Vec3 tangent = tangent_velocity.normalized();
    
    // Calculate friction impulse scalar
    float inv_mass1 = node1.is_dynamic ? 1.0f / node1.mass : 0.0f;
    float inv_mass2 = node2.is_dynamic ? 1.0f / node2.mass : 0.0f;
    float friction_impulse = -tangent_velocity.dot(tangent) / (inv_mass1 + inv_mass2);
    
    // Coulomb friction model
    float mu = std::sqrt(node1.static_friction * node2.static_friction);
    float max_friction = mu * normal_impulse;
    
    // Clamp friction impulse
    friction_impulse = std::min(max_friction, std::max(-max_friction, friction_impulse));
    
    // Apply friction impulse
    Vec3 friction_force = tangent * friction_impulse;
    
    if (node1.is_dynamic) {
        node1.velocity -= friction_force * inv_mass1;
    }
    if (node2.is_dynamic) {
        node2.velocity += friction_force * inv_mass2;
    }
}
```

## Position Correction

To prevent objects from sinking into each other due to numerical errors:

```cpp
void correctPosition(Node3D& node1, Node3D& node2, const CollisionInfo& info) {
    const float percent = 0.2f;  // Penetration resolution percentage
    const float slop = 0.01f;    // Penetration allowance
    
    float inv_mass1 = node1.is_dynamic ? 1.0f / node1.mass : 0.0f;
    float inv_mass2 = node2.is_dynamic ? 1.0f / node2.mass : 0.0f;
    
    Vec3 correction = std::max(info.penetration_depth - slop, 0.0f) * 
                    percent * info.collision_normal / (inv_mass1 + inv_mass2);
    
    if (node1.is_dynamic) {
        node1.position -= correction * inv_mass1;
    }
    if (node2.is_dynamic) {
        node2.position += correction * inv_mass2;
    }
}
```

## Continuous Collision Response

For fast-moving objects to prevent tunneling:

```cpp
void resolveContinuousCollision(Node3D& node1, Node3D& node2, 
                              const ContinuousCollisionInfo& info) {
    // Save original velocities
    Vec3 v1_original = node1.velocity;
    Vec3 v2_original = node2.velocity;
    
    // Calculate positions at time of impact
    Vec3 p1_at_impact = node1.prev_position + 
                      info.time_of_impact * (node1.position - node1.prev_position);
    Vec3 p2_at_impact = node2.prev_position + 
                      info.time_of_impact * (node2.position - node2.prev_position);
    
    // Set positions to impact point
    node1.position = p1_at_impact;
    node2.position = p2_at_impact;
    
    // Resolve collision at impact point
    CollisionInfo discrete_info;
    discrete_info.collision_normal = info.collision_normal;
    discrete_info.penetration_depth = 0.0f;
    discrete_info.contact_point = info.contact_point;
    
    resolveCollision(node1, node2, discrete_info);
    
    // Calculate remaining time
    float remaining_time = 1.0f - info.time_of_impact;
    
    // Continue movement with new velocities
    node1.position += node1.velocity * remaining_time * info.dt;
    node2.position += node2.velocity * remaining_time * info.dt;
    
    // Store updated velocities
    node1.prev_velocity = v1_original;
    node2.prev_velocity = v2_original;
}
```

## Military-Grade Verification

For military applications, collision responses are verified for physical correctness:

```cpp
bool verifyCollisionResponse(const Node3D& node1_before, const Node3D& node2_before,
                           const Node3D& node1_after, const Node3D& node2_after) {
    // Verify conservation of momentum
    Vec3 momentum_before = node1_before.mass * node1_before.velocity + 
                         node2_before.mass * node2_before.velocity;
                         
    Vec3 momentum_after = node1_after.mass * node1_after.velocity + 
                        node2_after.mass * node2_after.velocity;
    
    float momentum_error = (momentum_after - momentum_before).length();
    
    // Verify energy (allowing for some dissipation but no increase)
    float energy_before = 0.5f * node1_before.mass * node1_before.velocity.lengthSquared() + 
                        0.5f * node2_before.mass * node2_before.velocity.lengthSquared();
                        
    float energy_after = 0.5f * node1_after.mass * node1_after.velocity.lengthSquared() + 
                       0.5f * node2_after.mass * node2_after.velocity.lengthSquared();
    
    // Energy can decrease but not increase significantly
    bool energy_valid = energy_after <= energy_before * 1.001f;
    
    // Combined verification
    return momentum_error < 0.001f && energy_valid;
}
```

## Collision Response Configuration

```cpp
// Configure collision response settings
PhysicsSolverConfig config;
config.collision_response = CollisionResponseType::CONTINUOUS;
config.restitution = 0.5f;  // Global restitution (bounciness)
config.friction = 0.3f;     // Global friction coefficient
config.position_correction = true;  // Enable position correction
solver->setConfig(config);

// Set per-node properties
node1->restitution = 0.8f;  // More bouncy
node1->static_friction = 0.2f;
node1->dynamic_friction = 0.1f;
```

## Performance Considerations

- Basic collision response is fast but may suffer from stability issues
- Continuous methods prevent tunneling but require more computation
- Position correction helps stability but adds overhead
- For large simulations, collision islands can be processed in parallel

# Numerical Stability Guide

## Overview

The Zero Point Physics Engine employs various techniques to ensure numerical stability in simulations, even under extreme conditions. This document outlines these techniques and provides guidance on avoiding common numerical issues.

## Common Numerical Problems

Physics simulations face several numerical challenges:

1. **Floating-point precision errors**: Accumulation of roundoff errors
2. **Stiff systems**: Interactions with widely varying time scales
3. **Energy drift**: Non-physical energy changes over time
4. **Collision jitter**: Unstable behavior during collisions
5. **Tunneling**: Fast objects passing through thin objects

## Stability Techniques

### Time Step Selection

```cpp
float calculateStableTimestep(const std::vector<Node3D*>& nodes) {
    float max_velocity = 0.0f;
    float min_radius = FLT_MAX;
    
    // Find maximum velocity and minimum radius
    for (const auto* node : nodes) {
        max_velocity = std::max(max_velocity, node->velocity.length());
        min_radius = std::min(min_radius, node->radius);
    }
    
    // CFL condition for stability
    if (max_velocity > 0.0001f) {
        return 0.5f * min_radius / max_velocity;
    }
    
    return 0.016f;  // Default for 60Hz
}
```

### Velocity Clamping

```cpp
void clampExtremeVelocities(Node3D& node) {
    const float MAX_VELOCITY = 1000.0f;
    
    float vel_length = node.velocity.length();
    if (vel_length > MAX_VELOCITY) {
        node.velocity = node.velocity * (MAX_VELOCITY / vel_length);
    }
}
```

### Position Integration

Using semi-implicit Euler for improved stability:

```cpp
void semiImplicitEuler(Node3D& node, float dt) {
    // Update velocity first
    Vec3 acceleration = node.accumulated_force / node.mass;
    node.velocity += acceleration * dt;
    
    // Then update position using new velocity
    node.position += node.velocity * dt;
    
    node.accumulated_force = Vec3(0, 0, 0);
}
```

### Collision Stabilization

Position correction to prevent sinking:

```cpp
void applyPositionCorrection(Node3D& node1, Node3D& node2, const Vec3& normal, float penetration) {
    const float percent = 0.2f;  // Correction factor
    const float slop = 0.01f;    // Penetration allowance
    
    float correction = std::max(penetration - slop, 0.0f) * percent;
    
    float inv_mass1 = node1.is_dynamic ? 1.0f / node1.mass : 0.0f;
    float inv_mass2 = node2.is_dynamic ? 1.0f / node2.mass : 0.0f;
    
    Vec3 correction_vector = normal * correction / (inv_mass1 + inv_mass2);
    
    if (node1.is_dynamic) node1.position -= correction_vector * inv_mass1;
    if (node2.is_dynamic) node2.position += correction_vector * inv_mass2;
}
```

## Military-Grade Enhancements

For military applications, additional techniques are employed:

1. **Double-precision calculations** for critical operations
2. **Algorithmic robustness checks** to detect instability
3. **Conservation enforcement** to maintain physical laws
4. **Adaptive time stepping** based on local conditions

## Recommendations

1. Use appropriate time steps based on simulation characteristics
2. Implement continuous collision detection for fast-moving objects
3. Consider higher-order integration methods for critical simulations
4. Monitor energy conservation as an indicator of numerical health
5. Adjust error tolerances based on application requirements

## Diagnostic Tools

The engine provides tools to monitor numerical stability:

```cpp
// Check energy conservation
float initialEnergy = solver->getTotalEnergy();
// ... run simulation ...
float finalEnergy = solver->getTotalEnergy();
float drift = std::abs(finalEnergy - initialEnergy) / initialEnergy;
```

## Configuration Options

```cpp
// Configure numerical stability options
PhysicsSolverConfig config;
config.use_double_precision = true;  // Use double precision
config.max_velocity = 1000.0f;       // Maximum allowed velocity
config.position_correction = 0.2f;    // Collision correction factor
config.adaptive_timestep = true;      // Enable adaptive time stepping
solver->setConfig(config);
```

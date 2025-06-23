# Numerical Integration Methods

## Overview

The Zero Point Physics Engine employs advanced numerical integration techniques to accurately simulate physical motion over time. This document details the integration methods used, their mathematical foundations, error characteristics, and implementation details.

## Integration Methods Summary

| Method | Order | Stability | Computational Cost | Use Case |
|--------|-------|-----------|-------------------|----------|
| Explicit Euler | 1st | Low | Very Low | Fast prototyping |
| Semi-implicit Euler | 1st | Medium | Low | Default method |
| Verlet | 2nd | High | Medium | Energy conservation |
| RK4 | 4th | Very High | High | Military-grade precision |
| Symplectic | 2nd | Very High | Medium | Long-term stability |

## Explicit Euler Integration

The simplest integration method, used primarily for testing and debugging:

```cpp
void explicitEulerIntegration(Node3D& node, float dt) {
    // Update position based on current velocity
    node.position += node.velocity * dt;
    
    // Calculate acceleration from force and mass
    Vec3 acceleration = node.accumulated_force / node.mass;
    
    // Update velocity based on calculated acceleration
    node.velocity += acceleration * dt;
    
    // Clear accumulated forces
    node.accumulated_force = Vec3(0, 0, 0);
}
```

### Characteristics:
- **Accuracy**: 1st order
- **Stability**: Conditionally stable for small time steps
- **Energy**: Tends to add energy to the system over time
- **Error**: O(dt²)

## Semi-implicit Euler Integration

The default integration method for standard simulations:

```cpp
void semiImplicitEulerIntegration(Node3D& node, float dt) {
    // Calculate acceleration from force and mass
    Vec3 acceleration = node.accumulated_force / node.mass;
    
    // Update velocity based on acceleration
    node.velocity += acceleration * dt;
    
    // Update position based on updated velocity
    node.position += node.velocity * dt;
    
    // Clear accumulated forces
    node.accumulated_force = Vec3(0, 0, 0);
}
```

### Characteristics:
- **Accuracy**: 1st order
- **Stability**: Better than explicit Euler
- **Energy**: Better energy conservation than explicit Euler
- **Error**: O(dt²)

## Velocity Verlet Integration

Used for systems requiring better energy conservation:

```cpp
void velocityVerletIntegration(Node3D& node, float dt, PhysicsSolver& solver) {
    // Save current acceleration
    Vec3 acc_old = node.accumulated_force / node.mass;
    
    // Update position using velocity and half-step acceleration
    node.position += node.velocity * dt + 0.5f * acc_old * dt * dt;
    
    // Clear accumulated forces to prepare for force recalculation
    node.accumulated_force = Vec3(0, 0, 0);
    
    // Recalculate forces at new position (using physics solver)
    solver.calculateForcesForNode(node);
    
    // Calculate new acceleration
    Vec3 acc_new = node.accumulated_force / node.mass;
    
    // Update velocity using average acceleration
    node.velocity += 0.5f * (acc_old + acc_new) * dt;
}
```

### Characteristics:
- **Accuracy**: 2nd order
- **Stability**: Very good, symplectic properties
- **Energy**: Excellent energy conservation
- **Error**: O(dt³)

## Runge-Kutta 4th Order (RK4)

The highest precision integration method used for military-grade simulations:

```cpp
void rungeKutta4Integration(Node3D& node, float dt, PhysicsSolver& solver) {
    // Store original state
    Vec3 original_pos = node.position;
    Vec3 original_vel = node.velocity;
    
    // Calculate k1 (acceleration at start position)
    Vec3 k1_acc = node.accumulated_force / node.mass;
    Vec3 k1_vel = node.velocity;
    
    // Calculate k2 (acceleration at position + 0.5*dt*k1_vel)
    node.position = original_pos + 0.5f * dt * k1_vel;
    node.accumulated_force = Vec3(0, 0, 0);
    solver.calculateForcesForNode(node);
    Vec3 k2_acc = node.accumulated_force / node.mass;
    Vec3 k2_vel = original_vel + 0.5f * dt * k1_acc;
    
    // Calculate k3 (acceleration at position + 0.5*dt*k2_vel)
    node.position = original_pos + 0.5f * dt * k2_vel;
    node.accumulated_force = Vec3(0, 0, 0);
    solver.calculateForcesForNode(node);
    Vec3 k3_acc = node.accumulated_force / node.mass;
    Vec3 k3_vel = original_vel + 0.5f * dt * k2_acc;
    
    // Calculate k4 (acceleration at position + dt*k3_vel)
    node.position = original_pos + dt * k3_vel;
    node.accumulated_force = Vec3(0, 0, 0);
    solver.calculateForcesForNode(node);
    Vec3 k4_acc = node.accumulated_force / node.mass;
    Vec3 k4_vel = original_vel + dt * k3_acc;
    
    // Final update using weighted average
    node.position = original_pos + (dt/6.0f) * (k1_vel + 2.0f*k2_vel + 2.0f*k3_vel + k4_vel);
    node.velocity = original_vel + (dt/6.0f) * (k1_acc + 2.0f*k2_acc + 2.0f*k3_acc + k4_acc);
    
    // Reset forces
    node.accumulated_force = Vec3(0, 0, 0);
}
```

### Characteristics:
- **Accuracy**: 4th order
- **Stability**: Excellent
- **Energy**: Very good energy conservation
- **Error**: O(dt⁵)
- **Computational Cost**: High (requires multiple force evaluations)

## Symplectic Integration

Used for very long simulations requiring excellent energy conservation:

```cpp
void symplecticIntegration(Node3D& node, float dt) {
    // Half-step velocity update
    Vec3 acceleration = node.accumulated_force / node.mass;
    node.velocity += acceleration * (dt * 0.5f);
    
    // Full-step position update
    node.position += node.velocity * dt;
    
    // Recalculate forces at new position (requires external force calculation)
    // This would be done by the physics solver
    
    // Half-step velocity update with new forces
    acceleration = node.accumulated_force / node.mass;
    node.velocity += acceleration * (dt * 0.5f);
    
    // Clear accumulated forces
    node.accumulated_force = Vec3(0, 0, 0);
}
```

### Characteristics:
- **Accuracy**: 2nd order
- **Stability**: Excellent for long-term simulations
- **Energy**: Excellent energy conservation
- **Error**: O(dt³)

## Integration Method Selection

The physics solver selects the appropriate integration method based on several factors:

1. **Verification Level**: Higher verification levels use higher-order methods
2. **Simulation Duration**: Longer simulations benefit from symplectic methods
3. **Energy Conservation**: When energy conservation is critical, Verlet or symplectic methods are preferred
4. **Performance Requirements**: Real-time applications may use semi-implicit Euler
5. **Military Requirements**: Military-grade simulations always use RK4

```cpp
IntegrationMethod selectIntegrationMethod(VerificationLevel level) {
    switch (level) {
        case VerificationLevel::NONE:
            return IntegrationMethod::EXPLICIT_EULER;
        case VerificationLevel::BASIC:
            return IntegrationMethod::SEMI_IMPLICIT_EULER;
        case VerificationLevel::STANDARD:
            return IntegrationMethod::VELOCITY_VERLET;
        case VerificationLevel::MILITARY:
            return IntegrationMethod::RK4;
        case VerificationLevel::PARANOID:
            return IntegrationMethod::RK4;
        default:
            return IntegrationMethod::SEMI_IMPLICIT_EULER;
    }
}
```

## Error Analysis

### Local Error

Local truncation error for each method:

| Method | Local Error |
|--------|-------------|
| Explicit Euler | O(dt²) |
| Semi-implicit Euler | O(dt²) |
| Verlet | O(dt³) |
| Symplectic | O(dt³) |
| RK4 | O(dt⁵) |

### Global Error

Global accumulation error for each method:

| Method | Global Error |
|--------|------------|
| Explicit Euler | O(dt) |
| Semi-implicit Euler | O(dt) |
| Verlet | O(dt²) |
| Symplectic | O(dt²) |
| RK4 | O(dt⁴) |

## Adaptive Time-Stepping

The engine supports adaptive time-stepping to automatically adjust the integration step size based on error estimates:

```cpp
float calculateAdaptiveTimeStep(const std::vector<Node3D*>& nodes, float desired_error) {
    float max_velocity = 0.0f;
    float min_node_size = FLT_MAX;
    
    // Calculate maximum velocity and minimum node size
    for (const auto& node : nodes) {
        max_velocity = std::max(max_velocity, node->velocity.length());
        min_node_size = std::min(min_node_size, node->getRadius());
    }
    
    // Calculate stable time step based on CFL condition
    float stable_dt = 0.5f * min_node_size / max_velocity;
    
    // Ensure time step doesn't exceed desired error threshold
    float max_dt = std::sqrt(desired_error);
    
    // Return minimum of the two constraints
    return std::min(stable_dt, max_dt);
}
```

## Military-Grade Enhancements

For military-grade simulations, several enhancements are applied to the integration methods:

1. **Higher Precision Arithmetic**: Using extended precision floating-point operations
2. **Error Bounded Integration**: Guaranteeing maximum error bounds
3. **Verification After Each Step**: Validating integration results
4. **Conservation Enforcement**: Explicitly enforcing conservation laws
5. **Cryptographic State Tracking**: Ensuring state integrity

## Implementation Notes

- All integration methods are implemented using template specialization for different verification levels
- Force calculations are separated from integration steps for better modularity
- Military-grade methods use additional checks and balances
- Thread safety is ensured for parallel integration operations

## References

1. Hairer, E., Lubich, C., & Wanner, G. (2006). Geometric numerical integration: structure-preserving algorithms for ordinary differential equations.
2. Leimkuhler, B., & Reich, S. (2004). Simulating Hamiltonian dynamics.
3. Verlet, L. (1967). Computer "Experiments" on Classical Fluids. I. Thermodynamical Properties of Lennard-Jones Molecules.
4. Butcher, J. C. (2016). Numerical methods for ordinary differential equations.

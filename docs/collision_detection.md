# Collision Detection System

## Overview

The collision detection system in the Zero Point Physics Engine provides efficient identification of object interactions in 3D space. It employs hierarchical methods to balance performance with accuracy, supporting both discrete and continuous collision detection approaches.

## Architecture

```
┌─────────────────────────────────────────┐
│         Collision Detection Pipeline    │
└───────────────────┬─────────────────────┘
                    │
        ┌───────────▼────────────┐
        │     Broad Phase        │
        │  (Spatial Partitioning)│
        └───────────┬────────────┘
                    │
        ┌───────────▼────────────┐
        │    Mid Phase           │
        │ (AABB Tests)           │
        └───────────┬────────────┘
                    │
        ┌───────────▼────────────┐
        │    Narrow Phase        │
        │ (Exact Collision Tests)│
        └───────────┬────────────┘
                    │
        ┌───────────▼────────────┐
        │ Collision Resolution   │
        │ (Response Calculation) │
        └────────────────────────┘
```

## Broad Phase

The broad phase uses spatial partitioning to efficiently identify potential collision pairs:

### Binary Space Tree Implementation

```cpp
std::vector<CollisionPair> BinarySpaceTree::findPotentialCollisions() {
    std::vector<CollisionPair> pairs;
    std::stack<std::pair<BSTNode*, BSTNode*>> node_pairs;
    
    // Start with root node against itself
    if (root) {
        node_pairs.push({root, root});
    }
    
    // Traverse tree pairs
    while (!node_pairs.empty()) {
        auto [node1, node2] = node_pairs.top();
        node_pairs.pop();
        
        // Skip if AABBs don't overlap
        if (!node1->bounds.intersects(node2->bounds)) {
            continue;
        }
        
        // Leaf node handling - add contained objects as collision pairs
        if (node1->isLeaf() && node2->isLeaf()) {
            // Add all pairs of objects between these nodes
            for (auto& obj1 : node1->objects) {
                for (auto& obj2 : node2->objects) {
                    // Avoid self-collision and duplicate pairs
                    if (obj1->id != obj2->id && obj1->id < obj2->id) {
                        pairs.push_back({obj1, obj2});
                    }
                }
            }
        }
        // Recursive subdivision
        else if (node1->isLeaf()) {
            node_pairs.push({node1, node2->left});
            node_pairs.push({node1, node2->right});
        }
        else if (node2->isLeaf()) {
            node_pairs.push({node1->left, node2});
            node_pairs.push({node1->right, node2});
        }
        else {
            node_pairs.push({node1->left, node2->left});
            node_pairs.push({node1->left, node2->right});
            node_pairs.push({node1->right, node2->left});
            node_pairs.push({node1->right, node2->right});
        }
    }
    
    return pairs;
}
```

### Performance Optimizations

- **Dynamic Tree Rebalancing**: Rebalances the tree when objects move significantly
- **Node Expansion Heuristics**: Optimizes tree structure based on object distribution
- **Node Capacity Tuning**: Adjusts leaf capacity based on object count and distribution
- **Predictive Caching**: Maintains collision pairs between frames for temporal coherence

## Mid Phase

The mid phase uses Axis-Aligned Bounding Box (AABB) tests to further refine potential collision pairs:

```cpp
bool AABB::intersects(const AABB& other) const {
    // Early exit for non-overlapping bounds
    if (max.x < other.min.x || min.x > other.max.x) return false;
    if (max.y < other.min.y || min.y > other.max.y) return false;
    if (max.z < other.min.z || min.z > other.max.z) return false;
    
    // All axes overlap
    return true;
}
```

## Narrow Phase

The narrow phase performs exact collision tests between object geometries:

### Sphere-Sphere Collision

```cpp
bool testSphereSphereCollision(const Node3D& node1, const Node3D& node2, 
                              CollisionInfo& result) {
    Vec3 direction = node2.position - node1.position;
    float distance_sq = direction.lengthSquared();
    float radius_sum = node1.radius + node2.radius;
    
    // No collision if distance > sum of radii
    if (distance_sq > radius_sum * radius_sum) {
        return false;
    }
    
    // Calculate collision info
    float distance = std::sqrt(distance_sq);
    result.collision_normal = distance > 0.0001f ? direction / distance : Vec3(1, 0, 0);
    result.penetration_depth = radius_sum - distance;
    result.contact_point = node1.position + result.collision_normal * node1.radius;
    
    return true;
}
```

### Continuous Collision Detection

For high-speed objects, the engine employs continuous collision detection:

```cpp
bool detectContinuousCollision(const Node3D& node1, const Node3D& node2, 
                              float dt, CollisionInfo& result) {
    // Calculate relative motion during timestep
    Vec3 relative_motion = (node2.position - node2.prev_position) - 
                          (node1.position - node1.prev_position);
    
    // Collision geometry
    float radius_sum = node1.radius + node2.radius;
    
    // Solve quadratic equation for time of impact
    Vec3 start_pos_diff = node1.prev_position - node2.prev_position;
    
    float a = relative_motion.lengthSquared();
    float b = 2.0f * start_pos_diff.dot(relative_motion);
    float c = start_pos_diff.lengthSquared() - radius_sum * radius_sum;
    
    // Test if collision is possible
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0 || a < 0.0001f) {
        return false;
    }
    
    // Find earliest time of impact in [0, dt]
    float t = (-b - std::sqrt(discriminant)) / (2 * a);
    
    if (t < 0 || t > dt) {
        return false;
    }
    
    // Calculate collision state at time of impact
    Vec3 pos1_at_t = node1.prev_position + t * (node1.position - node1.prev_position);
    Vec3 pos2_at_t = node2.prev_position + t * (node2.position - node2.prev_position);
    
    result.collision_normal = (pos2_at_t - pos1_at_t).normalized();
    result.contact_point = pos1_at_t + result.collision_normal * node1.radius;
    result.collision_time = t;
    
    return true;
}
```

## Collision Response

Once collisions are detected, the physics solver applies appropriate responses:

### Impulse-Based Response

```cpp
void resolveCollision(Node3D& node1, Node3D& node2, const CollisionInfo& info) {
    // Skip if either object is static
    if (!node1.is_dynamic && !node2.is_dynamic) {
        return;
    }
    
    // Calculate relative velocity
    Vec3 relative_velocity = node2.velocity - node1.velocity;
    
    // Project relative velocity onto collision normal
    float vel_along_normal = relative_velocity.dot(info.collision_normal);
    
    // Early exit if objects are separating
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
    
    // Positional correction to prevent sinking
    const float percent = 0.2f; // penetration resolution percentage
    const float slop = 0.01f;   // penetration allowance
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

## Military-Grade Enhancements

For military applications, the collision system has several enhancements:

1. **Ultra-Precise Contact Generation**: Enhanced accuracy for critical simulations
2. **Cryptographic Collision Verification**: Ensures collision integrity
3. **Conservation Law Enforcement**: Guarantees physical correctness of collisions
4. **Predictive Collision Avoidance**: Uses intent heap to identify critical future collisions

## Performance Considerations

- The collision system uses batch processing to optimize cache usage
- Multi-threading is employed for broad and narrow phases
- Time-critical collisions are prioritized
- Adaptive precision reduces calculations for distant or non-critical collisions

## Verification Integration

Collisions are verified by the formal verification system:

```cpp
VerificationResult FormalVerify::verifyCollision(const Node3D& node1, const Node3D& node2,
                                              const Node3D& pre_collision1,
                                              const Node3D& pre_collision2) {
    // Initialize verification result
    VerificationResult result;
    
    // Verify momentum conservation
    Vec3 pre_momentum = pre_collision1.mass * pre_collision1.velocity +
                       pre_collision2.mass * pre_collision2.velocity;
    
    Vec3 post_momentum = node1.mass * node1.velocity +
                        node2.mass * node2.velocity;
    
    Vec3 momentum_diff = post_momentum - pre_momentum;
    double momentum_error = momentum_diff.length();
    
    // Verify energy conservation
    double pre_energy = 0.5 * pre_collision1.mass * pre_collision1.velocity.lengthSquared() +
                       0.5 * pre_collision2.mass * pre_collision2.velocity.lengthSquared();
    
    double post_energy = 0.5 * node1.mass * node1.velocity.lengthSquared() +
                        0.5 * node2.mass * node2.velocity.lengthSquared();
    
    // Energy can decrease but not increase (non-negative dissipation)
    double energy_error = std::max(0.0, post_energy - pre_energy);
    
    // Tiered error thresholds for verification success
    const double MOMENTUM_THRESHOLD_LOW = 1e-3;
    const double MOMENTUM_THRESHOLD_MED = 1e-2;
    const double MOMENTUM_THRESHOLD_HIGH = 1e-1;
    
    const double ENERGY_THRESHOLD_LOW = 1e-3;
    const double ENERGY_THRESHOLD_MED = 1e-2;
    const double ENERGY_THRESHOLD_HIGH = 1e-1;
    
    // Calculate verification confidence
    if (momentum_error < MOMENTUM_THRESHOLD_LOW && energy_error < ENERGY_THRESHOLD_LOW) {
        result.verified = true;
        result.confidence = 1.0;
    } else if (momentum_error < MOMENTUM_THRESHOLD_MED || energy_error < ENERGY_THRESHOLD_MED) {
        result.verified = true;
        result.confidence = 0.7;
    } else if (momentum_error < MOMENTUM_THRESHOLD_HIGH || energy_error < ENERGY_THRESHOLD_HIGH) {
        result.verified = true;
        result.confidence = 0.4;
    } else {
        result.verified = false;
        result.confidence = 0.0;
    }
    
    // Record error metrics
    result.max_error = std::max(momentum_error, energy_error);
    return result;
}
```

## Future Work

- GPU-accelerated broad phase using compute shaders
- Machine learning-based collision prediction
- Adaptable collision response based on material properties
- Enhanced continuous collision detection for deformable objects

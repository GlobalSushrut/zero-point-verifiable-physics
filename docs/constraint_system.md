# Constraint System

## Overview

The Constraint System in the Zero Point Physics Engine allows for the enforcement of relationships between physics objects. Constraints can restrict relative movement, maintain distances, or enforce specific orientations between nodes, enabling complex physical behaviors such as joints, ropes, and mechanical linkages.

## Constraint Types

The engine implements several constraint types:

```cpp
enum class ConstraintType {
    DISTANCE,      // Maintain a specific distance between nodes
    POINT_TO_POINT, // Connect two points with a ball joint
    HINGE,         // Connect two bodies with a hinge/revolute joint
    SLIDER,        // Allow movement along a single axis
    FIXED,         // Prevent all relative movement
    CONE_TWIST,    // Restrict rotation within a cone
    SPRING         // Connect nodes with an elastic spring force
};
```

## Core Implementation

The base constraint class defines the interface:

```cpp
class Constraint {
public:
    Constraint(Node3D* node_a, Node3D* node_b);
    virtual ~Constraint() = default;
    
    virtual void solve() = 0;
    virtual bool isValid() const;
    
    // Common settings
    void setEnabled(bool enabled);
    void setBreakingThreshold(float threshold);
    
protected:
    Node3D* node_a_;
    Node3D* node_b_;
    bool enabled_ = true;
    float breaking_threshold_ = FLT_MAX;
};
```

## Distance Constraint

Maintains a specific distance between two nodes:

```cpp
class DistanceConstraint : public Constraint {
public:
    DistanceConstraint(Node3D* node_a, Node3D* node_b, float distance);
    
    void solve() override {
        if (!enabled_) return;
        
        Vec3 delta = node_b_->position - node_a_->position;
        float current_distance = delta.length();
        
        // Skip if very small distance to avoid division by zero
        if (current_distance < 0.0001f) {
            return;
        }
        
        // Calculate correction
        float diff = (current_distance - distance_) / current_distance;
        
        // Check if constraint should break
        if (std::abs(diff) > breaking_threshold_) {
            enabled_ = false;
            return;
        }
        
        // Calculate position corrections
        Vec3 correction = delta * diff * 0.5f;
        
        // Apply correction based on mass ratio
        float total_inv_mass = node_a_->inv_mass + node_b_->inv_mass;
        if (total_inv_mass > 0.0f) {
            float node_a_factor = node_a_->inv_mass / total_inv_mass;
            float node_b_factor = node_b_->inv_mass / total_inv_mass;
            
            node_a_->position += correction * node_a_factor;
            node_b_->position -= correction * node_b_factor;
        }
    }
    
    void setDistance(float distance) { distance_ = distance; }
    float getDistance() const { return distance_; }
    
private:
    float distance_;
};
```

## Spring Constraint

Implements an elastic spring force:

```cpp
class SpringConstraint : public Constraint {
public:
    SpringConstraint(Node3D* node_a, Node3D* node_b, 
                   float rest_length, float stiffness, float damping);
    
    void solve() override {
        if (!enabled_) return;
        
        // Calculate displacement vector
        Vec3 delta = node_b_->position - node_a_->position;
        float current_length = delta.length();
        
        if (current_length < 0.0001f) {
            return; // Avoid division by zero
        }
        
        // Calculate spring force (Hooke's Law)
        float displacement = current_length - rest_length_;
        Vec3 direction = delta / current_length;
        
        // F = -k*x (spring force)
        Vec3 spring_force = direction * (-stiffness_ * displacement);
        
        // Calculate damping force
        Vec3 relative_velocity = node_b_->velocity - node_a_->velocity;
        Vec3 damping_force = direction * 
            (-damping_ * relative_velocity.dot(direction));
        
        // Combined force
        Vec3 total_force = spring_force + damping_force;
        
        // Apply forces
        node_a_->applyForce(-total_force);
        node_b_->applyForce(total_force);
    }
    
    void setParameters(float rest_length, float stiffness, float damping) {
        rest_length_ = rest_length;
        stiffness_ = stiffness;
        damping_ = damping;
    }
    
private:
    float rest_length_;
    float stiffness_;
    float damping_;
};
```

## Fixed Constraint

Locks all relative movement between nodes:

```cpp
class FixedConstraint : public Constraint {
public:
    FixedConstraint(Node3D* node_a, Node3D* node_b);
    
    void solve() override {
        if (!enabled_) return;
        
        // Store original relative transform on first solve
        if (!initialized_) {
            // Calculate relative position
            relative_position_ = node_b_->position - node_a_->position;
            
            // Calculate relative orientation
            relative_orientation_ = node_b_->orientation * 
                                  node_a_->orientation.inverse();
            
            initialized_ = true;
        }
        
        // Calculate target position for node_b based on node_a
        Vec3 target_position = node_a_->position + 
                            node_a_->orientation.rotate(relative_position_);
        
        // Calculate target orientation for node_b based on node_a
        Quaternion target_orientation = node_a_->orientation * relative_orientation_;
        
        // Correct position
        Vec3 position_error = target_position - node_b_->position;
        
        // Apply position correction
        float total_inv_mass = node_a_->inv_mass + node_b_->inv_mass;
        if (total_inv_mass > 0.0f) {
            float node_a_factor = node_a_->inv_mass / total_inv_mass;
            float node_b_factor = node_b_->inv_mass / total_inv_mass;
            
            node_a_->position -= position_error * node_a_factor;
            node_b_->position += position_error * node_b_factor;
        }
        
        // Correct orientation
        Quaternion orientation_error = target_orientation * 
                                    node_b_->orientation.inverse();
        
        // Apply orientation correction
        // (Simplified - actual implementation would use more complex orientation correction)
    }
    
private:
    bool initialized_ = false;
    Vec3 relative_position_;
    Quaternion relative_orientation_;
};
```

## Military-Grade Enhancements

For military applications, constraints include additional features:

```cpp
class MilitaryConstraint : public Constraint {
public:
    MilitaryConstraint(Node3D* node_a, Node3D* node_b);
    
    void setVerification(bool enabled) { verify_conservation_ = enabled; }
    void setFailureMode(FailureMode mode) { failure_mode_ = mode; }
    
protected:
    bool verify(const Vec3& force) {
        if (!verify_conservation_) return true;
        
        // Store pre-state for verification
        Vec3 p1_before = node_a_->position;
        Vec3 p2_before = node_b_->position;
        Vec3 v1_before = node_a_->velocity;
        Vec3 v2_before = node_b_->velocity;
        
        // Apply force
        if (node_a_->is_dynamic) {
            node_a_->applyForce(-force);
        }
        if (node_b_->is_dynamic) {
            node_b_->applyForce(force);
        }
        
        // Verify conservation laws
        Vec3 momentum_before = node_a_->mass * v1_before + node_b_->mass * v2_before;
        Vec3 momentum_after = node_a_->mass * node_a_->velocity + node_b_->mass * node_b_->velocity;
        
        float momentum_error = (momentum_after - momentum_before).length();
        bool is_valid = momentum_error < 0.001f;
        
        // Handle verification failure
        if (!is_valid) {
            handleVerificationFailure();
        }
        
        return is_valid;
    }
    
    void handleVerificationFailure() {
        switch (failure_mode_) {
            case FailureMode::DISABLE:
                enabled_ = false;
                break;
            case FailureMode::RESTORE:
                // Restore prior state
                break;
            case FailureMode::LOG:
                // Log failure
                break;
        }
    }
    
private:
    bool verify_conservation_ = false;
    FailureMode failure_mode_ = FailureMode::LOG;
};
```

## Usage Example

```cpp
// Create nodes
auto node1 = solver->createNode(Vec3(0, 0, 0), 1.0f);
auto node2 = solver->createNode(Vec3(0, 2, 0), 1.0f);

// Create a spring constraint
float rest_length = 2.0f;
float stiffness = 100.0f;
float damping = 0.5f;

auto spring = std::make_shared<SpringConstraint>(node1, node2, rest_length, stiffness, damping);
solver->addConstraint(spring);

// Create distance constraint with breaking threshold
auto rope = std::make_shared<DistanceConstraint>(node1, node2, 3.0f);
rope->setBreakingThreshold(0.2f);  // Breaks if stretched more than 20%
solver->addConstraint(rope);
```

## Performance Considerations

- Constraints are solved iteratively for stability
- Grouped constraints can be solved more efficiently
- Prioritized constraints ensure critical relationships are maintained
- Constraint islands can be processed in parallel

## Military Applications

In military simulations, constraints are used for:

- Vehicle suspension systems
- Aircraft control surfaces
- Robotic joint mechanics 
- Structural integrity analysis
- Soft-body tethering
- Damage modeling and failure analysis

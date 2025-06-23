#include "physics_solver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>

namespace zero_point {
namespace core {

PhysicsSolver::PhysicsSolver(size_t max_nodes, size_t max_threads)
    : max_nodes_(max_nodes),
      max_threads_(std::min(max_threads, static_cast<size_t>(std::thread::hardware_concurrency()))),
      gravity_(0.0f, -9.81f, 0.0f),
      solver_type_(SolverType::PARTICLE),
      energy_budget_(1.0f),
      simulation_time_(0.0f),
      enabled_(true),
      running_(false),
      next_constraint_id_(1),
      intent_heap_(max_nodes),
      zk_validator_(5) {
    
    // Initialize performance metrics
    performance_metrics_["fps"] = 0.0f;
    performance_metrics_["step_time_ms"] = 0.0f;
    performance_metrics_["collision_time_ms"] = 0.0f;
    performance_metrics_["constraint_time_ms"] = 0.0f;
    performance_metrics_["integration_time_ms"] = 0.0f;
    performance_metrics_["verification_time_ms"] = 0.0f;
    performance_metrics_["intent_update_time_ms"] = 0.0f;
    performance_metrics_["active_nodes"] = 0.0f;
    performance_metrics_["energy_efficiency"] = 1.0f;
    
    // Set default curvature function (flat space)
    setCurvatureFunction([](const Vec3& pos) -> float {
        return 0.0f;
    });
    
    // Set default intent function (uniform)
    setIntentFunction([](const Vec3& pos) -> float {
        return 0.5f;
    });
    
    // Initialize space tree with default depth
    space_tree_.initialize(8);
}

PhysicsSolver::~PhysicsSolver() {
    // Stop all worker threads
    enabled_ = false;
    
    for (auto& thread : worker_threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

bool PhysicsSolver::initialize(SolverType solver_type) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    solver_type_ = solver_type;
    
    // Initialize the binary space tree with appropriate depth based on solver type
    int depth = 8;
    switch (solver_type) {
        case SolverType::FLUID:
        case SolverType::SOFT_BODY:
            depth = 10;  // More depth for fine-grained simulations
            break;
        case SolverType::RIGID_BODY:
        case SolverType::ARTICULATED_BODY:
            depth = 6;   // Less depth for rigid body
            break;
        default:
            depth = 8;   // Default for particle and other types
    }
    
    // Initialize space tree
    if (!space_tree_.initialize(depth)) {
        return false;
    }
    
    // Clear any existing constraints
    constraints_.clear();
    
    // Reset simulation time
    simulation_time_ = 0.0f;
    
    // Start worker threads if needed
    if (worker_threads_.empty() && max_threads_ > 1) {
        for (size_t i = 0; i < max_threads_ - 1; ++i) {
            worker_threads_.emplace_back(&PhysicsSolver::workerThread_, this, i);
        }
    }
    
    return true;
}

void PhysicsSolver::setGravity(const Vec3& gravity) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    gravity_ = gravity;
}

void PhysicsSolver::setCurvatureFunction(const CurvatureFunction& func) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    // Set in space tree
    space_tree_.setCurvatureFunction(func);
}

void PhysicsSolver::setIntentFunction(const IntentFunction& func) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    // Set in space tree
    space_tree_.setIntentFunction(func);
    
    // Update all node priorities
    updateNodePriorities_();
}

void PhysicsSolver::setEnergyBudget(float budget) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    energy_budget_ = std::max(0.1f, std::min(1.0f, budget));
    
    // Propagate to subsystems
    space_tree_.setEnergyBudget(energy_budget_);
    intent_heap_.setEnergyBudget(energy_budget_);
}

void PhysicsSolver::registerCollisionCallback(const CollisionCallback& callback) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    collision_callbacks_.push_back(callback);
}

uint32_t PhysicsSolver::addNode(const Vec3& position, float mass, 
                                const PhysicsMaterial& material, bool is_static) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    // Create a new node
    auto node = std::make_shared<Node3D>(position, mass);
    node->is_dynamic = !is_static;
    node->density = material.density;
    
    // Add to space tree
    uint32_t node_id = space_tree_.addNode(node);
    
    // Add an intent node with medium priority
    IntentNode intent(0.5f);
    intent.entity_id = node_id;
    intent.energy_cost = 0.01f;  // Base energy cost
    intent_heap_.push(intent);
    
    return node_id;
}

uint32_t PhysicsSolver::addNode(const Vec3& position, bool is_dynamic, float mass) {
    // Create default material
    PhysicsMaterial material;
    
    // Call the main implementation with the dynamic flag inverted (is_static = !is_dynamic)
    return addNode(position, mass, material, !is_dynamic);
}

std::vector<uint32_t> PhysicsSolver::addParticleGroup(
    const std::vector<Vec3>& positions,
    const std::vector<float>& masses,
    const PhysicsMaterial& material,
    bool connected) {
    
    std::lock_guard<std::mutex> lock(solver_mutex_);
    std::vector<uint32_t> node_ids;
    
    // Validate inputs
    if (positions.empty() || positions.size() != masses.size()) {
        return node_ids;
    }
    
    // Create nodes
    node_ids.reserve(positions.size());
    std::vector<std::shared_ptr<Node3D>> nodes;
    nodes.reserve(positions.size());
    
    for (size_t i = 0; i < positions.size(); ++i) {
        auto node = std::make_shared<Node3D>(positions[i], masses[i]);
        node->density = material.density;
        
        // Add to space tree
        uint32_t node_id = space_tree_.addNode(node);
        node_ids.push_back(node_id);
        nodes.push_back(node);
        
        // Add an intent node
        IntentNode intent(0.5f);
        intent.entity_id = node_id;
        intent.energy_cost = 0.01f;
        intent_heap_.push(intent);
    }
    
    // Create connections between particles if requested
    if (connected && positions.size() > 1) {
        for (size_t i = 0; i < positions.size() - 1; ++i) {
            float distance = Vec3::distance(positions[i], positions[i + 1]);
            createDistanceConstraint(node_ids[i], node_ids[i + 1], distance, 0.8f);
        }
    }
    
    return node_ids;
}

bool PhysicsSolver::step(float dt, int substeps) {
    if (!enabled_) return false;
    
    // Set running flag
    running_ = true;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Adjust dt by energy budget
    float adjusted_dt = dt * std::min(1.0f, energy_budget_);
    
    // Calculate substep dt
    float substep_dt = adjusted_dt / static_cast<float>(substeps);
    
    // Process each substep
    for (int i = 0; i < substeps; ++i) {
        processStep_(substep_dt);
    }
    
    // Decay intent priorities
    intent_heap_.decay(0.99f);
    
    // Update simulation time
    simulation_time_ += adjusted_dt;
    
    // Calculate timing
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    float step_time_ms = static_cast<float>(duration.count()) / 1000.0f;
    
    // Update performance metrics
    performance_metrics_["step_time_ms"] = step_time_ms;
    performance_metrics_["fps"] = 1000.0f / std::max(1.0f, step_time_ms);
    performance_metrics_["active_nodes"] = static_cast<float>(space_tree_.getNodeCount());
    performance_metrics_["energy_efficiency"] = energy_budget_;
    
    // Clear running flag
    running_ = false;
    
    return true;
}

void PhysicsSolver::setEnabled(bool enabled) {
    enabled_ = enabled;
}

bool PhysicsSolver::isRunning() const {
    return running_;
}

Vec3 PhysicsSolver::getNodePosition(uint32_t node_id) const {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node) {
        return node->position;
    }
    
    return Vec3(0.0f, 0.0f, 0.0f);
}

Vec3 PhysicsSolver::getNodeVelocity(uint32_t node_id) const {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node) {
        return node->velocity;
    }
    
    return Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsSolver::setNodePosition(uint32_t node_id, const Vec3& position) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node) {
        node->position = position;
    }
}

void PhysicsSolver::setNodeVelocity(uint32_t node_id, const Vec3& velocity) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node) {
        node->velocity = velocity;
    }
}

void PhysicsSolver::applyForce(uint32_t node_id, const Vec3& force, bool world_space) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node && node->is_dynamic) {
        Vec3 effective_force = force;
        
        // Convert to world space if needed
        if (!world_space) {
            // For now, we assume local and world space are the same
            // This would be extended for objects with rotation
        }
        
        node->applyForce(effective_force, 0.016f);  // Apply with a small dt
    }
}

void PhysicsSolver::applyImpulse(uint32_t node_id, const Vec3& impulse, bool world_space) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node && node->is_dynamic && node->mass > 0.0f) {
        Vec3 effective_impulse = impulse;
        
        // Convert to world space if needed
        if (!world_space) {
            // For now, we assume local and world space are the same
        }
        
        // Impulse = F*dt = m*dv, so dv = impulse/m
        node->velocity += effective_impulse / node->mass;
    }
}

uint32_t PhysicsSolver::createDistanceConstraint(uint32_t node1_id, uint32_t node2_id, 
                                               float distance, float stiffness) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    // Find the nodes
    auto node1 = space_tree_.findNode(node1_id);
    auto node2 = space_tree_.findNode(node2_id);
    
    if (!node1 || !node2) {
        return 0;  // Invalid node IDs
    }
    
    // Create the constraint
    Constraint constraint;
    constraint.id = next_constraint_id_++;
    constraint.node1_id = node1_id;
    constraint.node2_id = node2_id;
    
    // Use provided distance or calculate from current positions
    if (distance <= 0.0f) {
        constraint.rest_length = Vec3::distance(node1->position, node2->position);
    } else {
        constraint.rest_length = distance;
    }
    
    constraint.stiffness = std::max(0.01f, std::min(1.0f, stiffness));
    constraint.enabled = true;
    
    constraints_.push_back(constraint);
    
    return constraint.id;
}

void PhysicsSolver::setNodePriority(uint32_t node_id, float priority) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    
    auto node = space_tree_.findNode(node_id);
    if (node) {
        // Set node priority directly
        node->intent_priority = std::max(0.0f, std::min(1.0f, priority));
        
        // Create a new intent node for this entity
        IntentNode intent(priority);
        intent.entity_id = node_id;
        intent.energy_cost = 0.01f;
        intent.requires_reflex = (priority > 0.8f);  // High priority nodes need reflexive processing
        
        // Add to intent heap
        intent_heap_.push(intent);
    }
}

std::unordered_map<std::string, float> PhysicsSolver::getPerformanceMetrics() const {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    return performance_metrics_;
}

const BinarySpaceTree& PhysicsSolver::getSpaceTree() const {
    return space_tree_;
}

const IntentHeap& PhysicsSolver::getIntentHeap() const {
    return intent_heap_;
}

const ZKCoromLight& PhysicsSolver::getZKValidator() const {
    return zk_validator_;
}

void PhysicsSolver::setVerificationMode(const std::string& mode) {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    zk_validator_.setVerificationMode(mode);
}

SolverType PhysicsSolver::getSolverType() const {
    return solver_type_;
}

void PhysicsSolver::workerThread_(size_t thread_id) {
    while (enabled_) {
        // Only do work if the solver is running
        if (running_) {
            // Worker threads just help with particle simulation for now
            // This would be expanded for more complex multi-threaded operations
            
            // Sleep to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            // Sleep when idle to avoid consuming CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void PhysicsSolver::processStep_(float dt) {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto integration_start_time = start_time;
    
    // Integration phase - update positions based on velocities
    space_tree_.integrateFrame(dt);
    
    auto collision_start_time = std::chrono::high_resolution_clock::now();
    performance_metrics_["integration_time_ms"] = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            collision_start_time - integration_start_time).count() / 1000.0f;
    
    // Collision detection and response
    detectCollisions_(dt);
    
    auto constraint_start_time = std::chrono::high_resolution_clock::now();
    performance_metrics_["collision_time_ms"] = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            constraint_start_time - collision_start_time).count() / 1000.0f;
    
    // Solve constraints
    solveConstraints_(dt);
    
    auto intent_start_time = std::chrono::high_resolution_clock::now();
    performance_metrics_["constraint_time_ms"] = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            intent_start_time - constraint_start_time).count() / 1000.0f;
    
    // Update node priorities
    updateNodePriorities_();
    
    auto verification_start_time = std::chrono::high_resolution_clock::now();
    performance_metrics_["intent_update_time_ms"] = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            verification_start_time - intent_start_time).count() / 1000.0f;
    
    // Verify physics (if enabled)
    verifyPhysics_(dt);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    performance_metrics_["verification_time_ms"] = 
        std::chrono::duration_cast<std::chrono::microseconds>(
            end_time - verification_start_time).count() / 1000.0f;
}

void PhysicsSolver::solveConstraints_(float dt) {
    // Skip if no constraints or low energy
    if (constraints_.empty() || energy_budget_ < 0.1f) {
        return;
    }
    
    // Apply a fraction of constraint correction based on stiffness
    for (const auto& constraint : constraints_) {
        if (!constraint.enabled) continue;
        
        // Get the nodes
        auto node1 = space_tree_.findNode(constraint.node1_id);
        auto node2 = space_tree_.findNode(constraint.node2_id);
        
        if (!node1 || !node2) continue;
        
        // Calculate current distance
        Vec3 delta = node2->position - node1->position;
        float current_distance = delta.length();
        
        if (current_distance < 0.0001f) continue; // Prevent division by zero
        
        // Calculate displacement
        float displacement = current_distance - constraint.rest_length;
        
        // Skip if already satisfied
        if (std::abs(displacement) < 0.0001f) continue;
        
        // Normalize direction
        Vec3 direction = delta / current_distance;
        
        // Calculate correction based on constraint stiffness
        float correction_factor = displacement * constraint.stiffness;
        Vec3 correction = direction * correction_factor;
        
        // Apply correction based on mass ratio
        float total_mass = node1->mass + node2->mass;
        
        if (node1->is_dynamic && total_mass > 0.0f) {
            float mass_ratio = node1->mass / total_mass;
            node1->position += correction * mass_ratio;
        }
        
        if (node2->is_dynamic && total_mass > 0.0f) {
            float mass_ratio = node2->mass / total_mass;
            node2->position -= correction * mass_ratio;
        }
    }
}

void PhysicsSolver::detectCollisions_(float dt) {
    // Skip if energy budget is too low
    if (energy_budget_ < 0.1f) {
        return;
    }
    
    // Simple collision detection between all dynamic nodes
    // For a real system, this would use spatial partitioning for efficiency
    if (solver_type_ == SolverType::PARTICLE) {
        // For particle system, use basic sphere-sphere collision
        calculateParticleForces_(dt);
    } else if (solver_type_ == SolverType::RIGID_BODY) {
        // For rigid bodies, delegate to specialized method
        calculateRigidBodyForces_(dt);
    } else if (solver_type_ == SolverType::SOFT_BODY || solver_type_ == SolverType::CLOTH) {
        // For soft bodies and cloth
        calculateSoftBodyForces_(dt);
    } else if (solver_type_ == SolverType::ARTICULATED_BODY) {
        // For humanoid/articulated simulation
        calculateArticulatedBodyForces_(dt);
    }
}

void PhysicsSolver::updateNodePriorities_() {
    // Update priorities based on intent function
    // This will be called periodically to update computational resources
    
    // We don't have a direct way to iterate all nodes, so we'll use node IDs
    // Assuming IDs are consecutive starting from 1
    size_t node_count = space_tree_.getNodeCount();
    
    for (uint32_t i = 1; i <= node_count; ++i) {
        auto node = space_tree_.findNode(i);
        if (node) {
            // Apply intent function to get priority
            // Calculate priority based on node position using the intent function
            float priority = 0.5f; // Default priority if no function set
            if (space_tree_.getIntentFunction()) {
                priority = space_tree_.getIntentFunction()(node->position);
            }
            node->intent_priority = priority;
            
            // Adjust intent for high-velocity objects
            float speed = node->velocity.length();
            if (speed > 10.0f) {
                node->intent_priority = std::min(1.0f, node->intent_priority + 0.2f);
            }
        }
    }
}

bool PhysicsSolver::verifyPhysics_(float dt) {
    // Use ZK validator to verify the physics simulation
    return zk_validator_.verifyFrame(space_tree_, dt);
}

void PhysicsSolver::calculateParticleForces_(float dt) {
    // Get all nodes
    size_t node_count = space_tree_.getNodeCount();
    
    // For a real system, this would use spatial partitioning for efficiency
    // For now, we'll use a simple O(n²) approach
    for (uint32_t i = 1; i <= node_count; ++i) {
        auto node1 = space_tree_.findNode(i);
        if (!node1 || !node1->is_dynamic) continue;
        
        // Apply gravity
        node1->applyForce(gravity_ * node1->mass, dt);
        
        // Apply spacetime curvature effect
        float curvature = node1->curvature;
        if (std::abs(curvature) > 0.0001f) {
            // In curved space, objects follow geodesics
            Vec3 curve_force = node1->position.normalized() * (-curvature * node1->mass * 9.81f);
            node1->applyForce(curve_force, dt);
        }
        
        // Check collisions with other nodes
        for (uint32_t j = i + 1; j <= node_count; ++j) {
            auto node2 = space_tree_.findNode(j);
            if (!node2) continue;
            
            // Calculate distance between particles
            Vec3 delta = node2->position - node1->position;
            float dist_sq = delta.lengthSquared();
            
            // Simple radius based on mass (could be more sophisticated)
            float radius1 = std::cbrt(node1->mass / (node1->density * 4.0f/3.0f * M_PI));
            float radius2 = std::cbrt(node2->mass / (node2->density * 4.0f/3.0f * M_PI));
            float min_dist = radius1 + radius2;
            float min_dist_sq = min_dist * min_dist;
            
            // Check for collision
            if (dist_sq < min_dist_sq) {
                float dist = std::sqrt(dist_sq);
                
                // Normalized direction from node1 to node2
                Vec3 normal = dist > 0.0001f ? delta / dist : Vec3(1.0f, 0.0f, 0.0f);
                
                // Calculate overlap
                float overlap = min_dist - dist;
                
                // Position correction to prevent sinking
                if (node1->is_dynamic && node2->is_dynamic) {
                    float mass_sum = node1->mass + node2->mass;
                    if (mass_sum > 0.0001f) {
                        float ratio1 = node2->mass / mass_sum;
                        float ratio2 = node1->mass / mass_sum;
                        
                        node1->position -= normal * (overlap * ratio1);
                        node2->position += normal * (overlap * ratio2);
                    }
                } else if (node1->is_dynamic) {
                    node1->position -= normal * overlap;
                } else if (node2->is_dynamic) {
                    node2->position += normal * overlap;
                }
                
                // Velocity correction (bounce)
                Vec3 relative_velocity = node2->velocity - node1->velocity;
                float normal_velocity = relative_velocity.dot(normal);
                
                // Only resolve if objects are moving toward each other
                if (normal_velocity < 0) {
                    // Calculate impulse scalar
                    float restitution = 0.5f;  // Coefficient of restitution
                    float impulse_scalar = -(1.0f + restitution) * normal_velocity;
                    
                    if (node1->is_dynamic && node2->is_dynamic) {
                        impulse_scalar /= (1.0f / node1->mass + 1.0f / node2->mass);
                        
                        node1->velocity -= normal * (impulse_scalar / node1->mass);
                        node2->velocity += normal * (impulse_scalar / node2->mass);
                    } else if (node1->is_dynamic) {
                        node1->velocity -= normal * normal_velocity * (1.0f + restitution);
                    } else if (node2->is_dynamic) {
                        node2->velocity -= normal * normal_velocity * (1.0f + restitution);
                    }
                }
                
                // Trigger callbacks if any
                Vec3 collision_point = node1->position + normal * radius1;
                float impulse = std::abs(normal_velocity) * std::min(node1->mass, node2->mass);
                
                for (const auto& callback : collision_callbacks_) {
                    callback(node1->id, node2->id, collision_point, impulse);
                }
            }
        }
    }
}

void PhysicsSolver::calculateRigidBodyForces_(float dt) {
    // Placeholder for rigid body physics
    // Would implement full rigid body dynamics with orientation
    // For now, just use particle approach
    calculateParticleForces_(dt);
}

void PhysicsSolver::calculateSoftBodyForces_(float dt) {
    // Placeholder for soft body physics
    // Would implement stress and strain calculations
    // For now, just use particle approach with constraints
    calculateParticleForces_(dt);
    solveConstraints_(dt);
}

void PhysicsSolver::calculateArticulatedBodyForces_(float dt) {
    // Placeholder for articulated/humanoid body physics
    // Would implement joint constraints and hierarchical rigid bodies
    // For now, just use rigid body approach
    calculateRigidBodyForces_(dt);
}

} // namespace core
} // namespace zero_point

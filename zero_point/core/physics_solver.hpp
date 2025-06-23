#pragma once

#include <vector>
#include <memory>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <future>
#include <unordered_map>

#include "binary_space_tree.hpp"
#include "intent_heap.hpp"
#include "zk_corom_light.hpp"

namespace zero_point {
namespace core {

/**
 * @brief Solver types for different physical phenomena
 */
enum class SolverType {
    PARTICLE,           // Simple particle physics
    RIGID_BODY,         // Rigid body dynamics
    SOFT_BODY,          // Deformable soft body
    FLUID,              // Fluid dynamics
    ARTICULATED_BODY,   // Articulated body with joints (humanoid)
    CLOTH,              // Cloth simulation
    CUSTOM              // Custom solver
};

/**
 * @brief Physics material properties
 */
struct PhysicsMaterial {
    float density;              // Density in kg/m³
    float restitution;          // Coefficient of restitution (0-1)
    float friction;             // Coefficient of friction
    float drag;                 // Drag coefficient for air resistance
    float roughness;            // Surface roughness
    float viscosity;            // Internal viscosity for soft bodies/fluids
    
    PhysicsMaterial() : 
        density(1000.0f), 
        restitution(0.5f), 
        friction(0.5f), 
        drag(0.01f), 
        roughness(0.1f), 
        viscosity(0.0f) {}
};

/**
 * @brief Unified physics solver supporting various simulation types
 * 
 * The PhysicsSolver integrates with the binary space tree, intent heap,
 * and ZK validation to provide a complete physics simulation framework
 * with non-Euclidean spacetime support and intent-driven computation.
 */
class PhysicsSolver {
public:
    using CurvatureFunction = std::function<float(const Vec3&)>;
    using IntentFunction = std::function<float(const Vec3&)>;
    using CollisionCallback = std::function<void(uint32_t, uint32_t, const Vec3&, float)>;
    
    /**
     * @brief Constructor
     * @param max_nodes Maximum number of nodes supported
     * @param max_threads Maximum number of threads to use
     */
    PhysicsSolver(size_t max_nodes = 10000, size_t max_threads = 4);
    
    /**
     * @brief Destructor ensures all threads are stopped
     */
    ~PhysicsSolver();
    
    /**
     * @brief Initialize the physics solver
     * @param solver_type Type of solver to initialize
     * @return True if successful
     */
    bool initialize(SolverType solver_type);
    
    /**
     * @brief Set the gravity vector
     * @param gravity Gravity vector
     */
    void setGravity(const Vec3& gravity);
    
    /**
     * @brief Set the spacetime curvature function
     * @param func Curvature function
     */
    void setCurvatureFunction(const CurvatureFunction& func);
    
    /**
     * @brief Set the computational intent function
     * @param func Intent function
     */
    void setIntentFunction(const IntentFunction& func);
    
    /**
     * @brief Set the energy budget for physics calculations
     * @param budget Energy budget from 0.0 to 1.0
     */
    void setEnergyBudget(float budget);
    
    /**
     * @brief Register a collision callback
     * @param callback Function to call on collision
     */
    void registerCollisionCallback(const CollisionCallback& callback);
    
    /**
     * @brief Add a node to the physics system
     * @param position Initial position
     * @param mass Mass in kg
     * @param material Material properties
     * @param is_static Whether the node is static (immovable)
     * @return ID of the added node
     */
    uint32_t addNode(const Vec3& position, 
                    float mass, 
                    const PhysicsMaterial& material,
                    bool is_static = false);
                    
    /**
     * @brief Add a node to the physics system (simplified version)
     * @param position Initial position
     * @param is_dynamic Whether the node is dynamic (movable)
     * @param mass Mass in kg
     * @return ID of the added node
     */
    uint32_t addNode(const Vec3& position, bool is_dynamic = true, float mass = 1.0f);
    
    /**
     * @brief Add a particle group (for particle-based simulations)
     * @param positions Vector of positions
     * @param masses Vector of masses
     * @param material Material for all particles
     * @param connected Whether particles are connected
     * @return Vector of node IDs
     */
    std::vector<uint32_t> addParticleGroup(
        const std::vector<Vec3>& positions,
        const std::vector<float>& masses,
        const PhysicsMaterial& material,
        bool connected = false);
    
    /**
     * @brief Simulate one frame of physics
     * @param dt Time step in seconds
     * @param substeps Number of substeps for stability
     * @return True if simulation was successful
     */
    bool step(float dt, int substeps = 1);
    
    /**
     * @brief Enable or disable the solver
     * @param enabled Whether the solver is enabled
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Check if the solver is currently running
     * @return True if running
     */
    bool isRunning() const;
    
    /**
     * @brief Get the position of a node
     * @param node_id ID of the node
     * @return Position vector
     */
    Vec3 getNodePosition(uint32_t node_id) const;
    
    /**
     * @brief Get the velocity of a node
     * @param node_id ID of the node
     * @return Velocity vector
     */
    Vec3 getNodeVelocity(uint32_t node_id) const;
    
    /**
     * @brief Set the position of a node
     * @param node_id ID of the node
     * @param position New position
     */
    void setNodePosition(uint32_t node_id, const Vec3& position);
    
    /**
     * @brief Set the velocity of a node
     * @param node_id ID of the node
     * @param velocity New velocity
     */
    void setNodeVelocity(uint32_t node_id, const Vec3& velocity);
    
    /**
     * @brief Apply a force to a node
     * @param node_id ID of the node
     * @param force Force vector
     * @param world_space Whether the force is in world space (vs. local)
     */
    void applyForce(uint32_t node_id, const Vec3& force, bool world_space = true);
    
    /**
     * @brief Apply an impulse to a node
     * @param node_id ID of the node
     * @param impulse Impulse vector
     * @param world_space Whether the impulse is in world space (vs. local)
     */
    void applyImpulse(uint32_t node_id, const Vec3& impulse, bool world_space = true);
    
    /**
     * @brief Create a constraint between two nodes
     * @param node1_id First node ID
     * @param node2_id Second node ID
     * @param distance Reference distance for the constraint
     * @param stiffness Stiffness coefficient (0-1)
     * @return ID of the created constraint
     */
    uint32_t createDistanceConstraint(
        uint32_t node1_id, 
        uint32_t node2_id, 
        float distance,
        float stiffness);
    
    /**
     * @brief Set intent priority for a specific node
     * @param node_id Node ID
     * @param priority Priority value (0-1)
     */
    void setNodePriority(uint32_t node_id, float priority);
    
    /**
     * @brief Get the overall simulation performance metrics
     * @return Map of metric name to value
     */
    std::unordered_map<std::string, float> getPerformanceMetrics() const;
    
    /**
     * @brief Get the internal binary space tree
     * @return Reference to the binary space tree
     */
    const BinarySpaceTree& getSpaceTree() const;
    
    /**
     * @brief Get the internal intent heap
     * @return Reference to the intent heap
     */
    const IntentHeap& getIntentHeap() const;
    
    /**
     * @brief Get the internal ZK validator
     * @return Reference to the ZK validator
     */
    const ZKCoromLight& getZKValidator() const;
    
    /**
     * @brief Set the verification mode for physics
     * @param mode Verification mode ("strict", "relaxed", or "off")
     */
    void setVerificationMode(const std::string& mode);
    
    /**
     * @brief Get the current solver type
     * @return Current solver type
     */
    SolverType getSolverType() const;
    
protected:
    struct Constraint {
        uint32_t id;
        uint32_t node1_id;
        uint32_t node2_id;
        float rest_length;
        float stiffness;
        bool enabled;
        
        Constraint() : id(0), node1_id(0), node2_id(0), 
                     rest_length(1.0f), stiffness(0.5f), enabled(true) {}
    };
    
    BinarySpaceTree space_tree_;
    IntentHeap intent_heap_;
    ZKCoromLight zk_validator_;
    
    size_t max_nodes_;
    size_t max_threads_;
    Vec3 gravity_;
    SolverType solver_type_;
    float energy_budget_;
    float simulation_time_;
    bool enabled_;
    std::atomic<bool> running_;
    
    std::vector<std::thread> worker_threads_;
    std::vector<Constraint> constraints_;
    std::unordered_map<std::string, float> performance_metrics_;
    std::vector<CollisionCallback> collision_callbacks_;
    
    std::atomic<uint32_t> next_constraint_id_;
    mutable std::mutex solver_mutex_;
    
    /**
     * @brief Worker thread function
     * @param thread_id ID of the thread
     */
    void workerThread_(size_t thread_id);
    
    /**
     * @brief Process a single physics step
     * @param dt Time step
     */
    void processStep_(float dt);
    
    /**
     * @brief Solve distance constraints
     * @param dt Time step
     */
    void solveConstraints_(float dt);
    
    /**
     * @brief Detect and resolve collisions
     * @param dt Time step
     */
    void detectCollisions_(float dt);
    
    /**
     * @brief Update node priorities based on intent function
     */
    void updateNodePriorities_();
    
    /**
     * @brief Verify physics simulation with ZK proof
     * @param dt Time step
     * @return True if verification passed
     */
    bool verifyPhysics_(float dt);
    
    /**
     * @brief Calculate forces for particle-based simulation
     * @param dt Time step
     */
    void calculateParticleForces_(float dt);
    
    /**
     * @brief Calculate forces for rigid body simulation
     * @param dt Time step
     */
    void calculateRigidBodyForces_(float dt);
    
    /**
     * @brief Calculate forces for soft body simulation
     * @param dt Time step
     */
    void calculateSoftBodyForces_(float dt);
    
    /**
     * @brief Calculate forces for articulated body simulation
     * @param dt Time step
     */
    void calculateArticulatedBodyForces_(float dt);
};

} // namespace core
} // namespace zero_point

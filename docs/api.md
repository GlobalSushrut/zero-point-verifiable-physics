# API Reference

## Core Classes

### PhysicsSolver

The main interface for physics simulation operations.

```cpp
class PhysicsSolver {
public:
    // Constructors and initialization
    PhysicsSolver();
    explicit PhysicsSolver(const PhysicsSolverConfig& config);
    virtual ~PhysicsSolver();
    
    // Core simulation methods
    virtual void step(float dt);
    virtual void reset();
    virtual bool initialize();
    
    // Node management
    virtual uint32_t addNode(const Vec3& position, float mass);
    virtual void removeNode(uint32_t id);
    virtual Node3D* findNode(uint32_t id);
    
    // Force application
    virtual void applyForce(uint32_t node_id, const Vec3& force);
    virtual void applyGravity(const Vec3& gravity = Vec3(0, -9.81f, 0));
    
    // Constraint management
    virtual uint32_t addConstraint(uint32_t node1_id, uint32_t node2_id, 
                                 const ConstraintParams& params);
    virtual void removeConstraint(uint32_t constraint_id);
    
    // Collisions
    virtual void setCollisionDetection(bool enabled);
    virtual void setCollisionResponseType(CollisionResponseType type);
    
    // Query methods
    virtual uint32_t getNodeCount() const;
    virtual uint32_t getConstraintCount() const;
    virtual uint32_t getCollisionCount() const;
    
    // Performance metrics
    virtual std::unordered_map<std::string, double> getPerformanceMetrics() const;
    
    // Configuration
    virtual void setConfig(const PhysicsSolverConfig& config);
    virtual PhysicsSolverConfig getConfig() const;
};
```

### PhysicsSolverMilitary

Extended solver with military-grade features.

```cpp
class PhysicsSolverMilitary : public PhysicsSolver {
public:
    // Constructors
    PhysicsSolverMilitary();
    explicit PhysicsSolverMilitary(VerificationLevel level);
    
    // Overridden methods with enhanced precision and verification
    void step(float dt) override;
    bool initialize() override;
    
    // Military-grade features
    void setVerificationLevel(VerificationLevel level);
    VerificationLevel getVerificationLevel() const;
    std::unordered_map<std::string, double> getVerificationStats() const;
    
    // State integrity
    void saveState(const std::string& file_path);
    bool loadState(const std::string& file_path);
    std::string getStateSignature() const;
    
    // Extended metrics
    std::unordered_map<std::string, double> getPerformanceMetrics() const override;
    float getMilitaryConfidence() const;
};
```

### RTScheduler

Real-time task scheduler for physics operations.

```cpp
class RTScheduler {
public:
    // Constructor and initialization
    explicit RTScheduler(uint32_t worker_count = 1);
    ~RTScheduler();
    
    // Task management
    uint64_t addTask(std::function<void()> function, 
                    const TaskParameters& params);
    bool removeTask(uint64_t task_id);
    bool modifyTask(uint64_t task_id, const TaskParameters& new_params);
    
    // Scheduler control
    void start();
    void stop();
    void pause();
    void resume();
    
    // Configuration
    void setCpuAffinity(const std::vector<int>& core_ids);
    void setQueueCapacity(uint32_t capacity);
    
    // Statistics and metrics
    std::unordered_map<std::string, TaskStatistics> getAllTaskStatistics() const;
    TaskStatistics getTaskStatistics(const std::string& task_name) const;
    double getUtilization() const;
    double getDeadlineMissRate() const;
    uint64_t getTotalTasksExecuted() const;
    uint64_t getTotalDeadlineMisses() const;
    double getAverageExecutionTime() const;
};
```

### FormalVerify

Verification engine for physics operations.

```cpp
class FormalVerify {
public:
    // Constructor and configuration
    explicit FormalVerify(VerificationLevel level = VerificationLevel::STANDARD);
    void setLevel(VerificationLevel level);
    VerificationLevel getLevel() const;
    
    // Verification methods
    VerificationResult verifyNodeIntegration(const Node3D& node, float dt,
                                          const Vec3& original_position, 
                                          const Vec3& original_velocity);
    
    VerificationResult verifyCollision(const Node3D& node1, const Node3D& node2,
                                     const Node3D& pre_collision1,
                                     const Node3D& pre_collision2);
    
    // Statistics
    std::unordered_map<std::string, double> getStatistics() const;
    double getSuccessRate() const;
    double getAverageConfidence() const;
    double getMaxError() const;
};
```

### BinarySpaceTree

Spatial partitioning for efficient physics calculations.

```cpp
class BinarySpaceTree {
public:
    // Constructors
    BinarySpaceTree();
    explicit BinarySpaceTree(const AABB& bounds);
    
    // Node management
    uint32_t insertNode(std::shared_ptr<Node3D> node);
    bool removeNode(uint32_t node_id);
    std::shared_ptr<Node3D> findNode(uint32_t node_id);
    
    // Tree operations
    void rebalance();
    void clear();
    AABB getBounds() const;
    
    // Queries
    std::vector<std::shared_ptr<Node3D>> query(const AABB& area);
    std::vector<std::shared_ptr<Node3D>> queryRadius(const Vec3& center, float radius);
    std::vector<CollisionPair> findPotentialCollisions();
    
    // Statistics
    uint32_t getNodeCount() const;
    uint32_t getTreeDepth() const;
    double getAverageChildCount() const;
};
```

## Data Structures

### Vec3

3D vector type with mathematical operations.

```cpp
class Vec3 {
public:
    float x, y, z;
    
    // Constructors
    Vec3();
    Vec3(float x, float y, float z);
    Vec3(const Vec3& other);
    
    // Assignment
    Vec3& operator=(const Vec3& other);
    
    // Vector operations
    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(float scalar) const;
    Vec3 operator/(float scalar) const;
    
    // Vector products
    float dot(const Vec3& other) const;
    Vec3 cross(const Vec3& other) const;
    
    // Length operations
    float length() const;
    float lengthSquared() const;
    Vec3 normalized() const;
    void normalize();
    
    // Utilities
    std::string toString() const;
    bool isZero() const;
};
```

### AABB

Axis-aligned bounding box for spatial queries.

```cpp
struct AABB {
    Vec3 min;
    Vec3 max;
    
    // Constructors
    AABB();
    AABB(const Vec3& min, const Vec3& max);
    
    // Query methods
    bool contains(const Vec3& point) const;
    bool intersects(const AABB& other) const;
    bool intersectsSphere(const Vec3& center, float radius) const;
    
    // Manipulation
    void expand(float amount);
    void expandToInclude(const Vec3& point);
    void expandToInclude(const AABB& other);
    
    // Properties
    Vec3 getCenter() const;
    Vec3 getExtents() const;
    float getVolume() const;
    float getSurfaceArea() const;
};
```

### Node3D

Physical entity in the simulation.

```cpp
class Node3D {
public:
    Vec3 position;
    Vec3 velocity;
    Vec3 accumulated_force;
    float mass;
    float curvature;
    bool is_dynamic;
    float intent_priority;
    uint32_t id;
    float density;
    
    // Constructors
    Node3D(const Vec3& pos = Vec3(), float m = 1.0f);
    
    // Node operations
    void integrate(float dt);
    void applyForce(const Vec3& force);
    void clearForces();
    
    // Derived properties
    float getInverseMass() const;
    bool isStatic() const;
    
    // Serialization
    std::string serialize() const;
    static Node3D deserialize(const std::string& data);
};
```

### TaskParameters

Configuration for scheduled tasks.

```cpp
struct TaskParameters {
    std::string task_name;
    int32_t priority;
    uint64_t period_us;
    uint64_t deadline_us;
    bool is_periodic;
    
    // Constructors
    TaskParameters();
    TaskParameters(const std::string& name, int32_t pri, uint64_t period,
                 uint64_t deadline, bool periodic);
};
```

### VerificationResult

Result of a physics verification operation.

```cpp
struct VerificationResult {
    bool verified;
    double confidence;
    double max_error;
    std::string error_location;
    uint32_t verified_elements;
    uint32_t total_elements;
    std::chrono::microseconds duration;
    
    // Constructor
    VerificationResult();
    
    // Utility methods
    std::string toString() const;
};
```

## Enumerations

### VerificationLevel

```cpp
enum class VerificationLevel {
    NONE,       // No verification
    BASIC,      // Basic conservation laws
    STANDARD,   // Standard verification
    MILITARY,   // High precision military grade
    PARANOID    // Ultra-high precision, zero tolerance
};
```

### CollisionResponseType

```cpp
enum class CollisionResponseType {
    NONE,              // No collision response
    DISCRETE,          // Discrete collision handling
    CONTINUOUS,        // Continuous collision detection
    SPECULATIVE        // Speculative contacts
};
```

### TaskPriority

```cpp
enum class TaskPriority {
    CRITICAL = 0,      // Highest priority
    HIGH = 10,
    NORMAL = 20,
    LOW = 30,
    BACKGROUND = 40    // Lowest priority
};
```

## Configuration Objects

### PhysicsSolverConfig

```cpp
struct PhysicsSolverConfig {
    Vec3 gravity;
    float damping;
    bool detect_collisions;
    CollisionResponseType collision_response;
    float collision_restitution;
    float collision_friction;
    uint32_t max_substeps;
    uint32_t simulation_threads;
    
    // Constructor with defaults
    PhysicsSolverConfig();
};
```

### ConstraintParams

```cpp
struct ConstraintParams {
    float rest_length;
    float stiffness;
    float damping;
    bool breakable;
    float break_threshold;
    ConstraintType type;
    
    // Constructor with defaults
    ConstraintParams();
};
```

## Function Signatures

### Mathematical Utilities

```cpp
namespace math_utils {
    // Vector operations
    Vec3 lerp(const Vec3& a, const Vec3& b, float t);
    Vec3 slerp(const Vec3& a, const Vec3& b, float t);
    
    // Collision detection
    bool rayIntersectsAABB(const Vec3& ray_origin, const Vec3& ray_dir,
                          const AABB& box, float& t_near, float& t_far);
    bool sphereIntersectsSphere(const Vec3& center1, float radius1,
                              const Vec3& center2, float radius2);
    
    // Numerical methods
    Vec3 rungeKutta4(const Vec3& pos, const Vec3& vel, const Vec3& acc, float dt);
    float calculateStableTimestep(float max_velocity, float min_node_size);
}
```

### Cryptographic Functions

```cpp
namespace crypto {
    // Hash generation
    std::string computeStateHash(const std::vector<Node3D>& nodes);
    std::string signState(const std::string& state_hash, const std::string& private_key);
    bool verifyStateSignature(const std::string& state_hash, const std::string& signature,
                             const std::string& public_key);
    
    // Zero-knowledge proofs
    std::vector<uint8_t> generateStateProof(const std::vector<Node3D>& nodes);
    bool verifyStateTransition(const std::vector<uint8_t>& proof,
                              const std::string& initial_hash,
                              const std::string& final_hash);
}
```

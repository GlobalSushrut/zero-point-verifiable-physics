#pragma once

#include <vector>
#include <memory>
#include <functional>
#include <array>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <string>
#include <memory>

// Enable ARM NEON optimizations when available
#ifdef USE_NEON
#include <arm_neon.h>
#endif

namespace zero_point {
namespace core {

/**
 * @brief 3D vector class with NEON optimization
 */
class Vec3 {
public:
    float x, y, z;
    
    // Constructors
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3(const Vec3& other) : x(other.x), y(other.y), z(other.z) {}
    
    // Explicit assignment operator to fix deprecated-copy warning
    Vec3& operator=(const Vec3& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }
    
    // Basic vector operations
    inline Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    
    inline Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    
    inline Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
    
    inline Vec3 operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Vec3(x * inv, y * inv, z * inv);
    }
    
    // In-place operations
    inline Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    
    inline Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    
    inline Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }
    
    // Vector operations
    inline float dot(const Vec3& other) const {
#ifdef USE_NEON
        float32x4_t v1 = {x, y, z, 0};
        float32x4_t v2 = {other.x, other.y, other.z, 0};
        float32x4_t result = vmulq_f32(v1, v2);
        float32x2_t sum = vadd_f32(vget_low_f32(result), vget_high_f32(result));
        return vget_lane_f32(vpadd_f32(sum, sum), 0);
#else
        return x * other.x + y * other.y + z * other.z;
#endif
    }
    
    inline float length() const {
        return std::sqrt(dot(*this));
    }
    
    inline float lengthSquared() const {
        return dot(*this);
    }
    
    inline Vec3 normalized() const {
        float len = length();
        if (len < 1e-6f) {
            return Vec3();
        }
        return *this / len;
    }
    
    inline Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    // Utility methods
    static float distance(const Vec3& a, const Vec3& b) {
        return (b - a).length();
    }
    
    static float distanceSquared(const Vec3& a, const Vec3& b) {
        return (b - a).lengthSquared();
    }
    
    // Array accessor for compatibility
    float& operator[](int index) {
        switch (index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Vec3 index out of range");
        }
    }
    
    const float& operator[](int index) const {
        switch (index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::out_of_range("Vec3 index out of range");
        }
    }
    
    // Conversion to array
    std::array<float, 3> toArray() const {
        return {x, y, z};
    }
    
    // Debug string
    std::string toString() const {
        return "(" + std::to_string(x) + ", " + 
               std::to_string(y) + ", " + 
               std::to_string(z) + ")";
    }
};

/**
 * @brief 3D node for binary space tree
 */
struct Node3D : public std::enable_shared_from_this<Node3D> {
    Vec3 position;           // Current position
    Vec3 velocity;           // Current velocity
    Vec3 accumulated_force;  // Accumulated force for this step
    float mass;              // Mass
    float curvature;         // Local spacetime curvature
    bool is_dynamic;         // Whether this node moves
    float intent_priority;   // Computational priority
    uint32_t id;             // Unique identifier
    float density;           // Mass density
    
    std::vector<std::shared_ptr<Node3D>> children;
    std::weak_ptr<Node3D> parent;
    
    // Constructor
    Node3D(const Vec3& pos = Vec3(), float m = 1.0f)
        : position(pos), velocity(Vec3()), accumulated_force(Vec3()), mass(m), curvature(0.0f),
          is_dynamic(true), intent_priority(0.5f), id(0), density(1000.0f),
          children() {}
    
    // Add a child node
    void addChild(const std::shared_ptr<Node3D>& child) {
        if (child) {
            children.push_back(child);
            child->parent = std::weak_ptr<Node3D>(shared_from_this());
        }
    }
    
    // Check if this node is a leaf (no children)
    bool isLeaf() const {
        return children.empty();
    }
    
    // Apply a force to this node
    void applyForce(const Vec3& force, float dt) {
        if (is_dynamic && mass > 0.0f) {
            // Accumulate forces
            accumulated_force += force;
            
            // Convert to acceleration
            Vec3 acceleration = force / mass;
            velocity += acceleration * dt;
        }
    }
    
    // Update position based on velocity
    void integrate(float dt) {
        if (is_dynamic) {
            position += velocity * dt;
            // Reset accumulated force after integration
            accumulated_force = Vec3(0.0f, 0.0f, 0.0f);
        }
    }
    
    // Calculate total mass including children
    float calculateTotalMass() const {
        float total = mass;
        for (const auto& child : children) {
            total += child->calculateTotalMass();
        }
        return total;
    }
    
    // Calculate center of mass including children
    Vec3 calculateCenterOfMass() const {
        Vec3 com = position * mass;
        float total_mass = mass;
        
        for (const auto& child : children) {
            float child_mass = child->calculateTotalMass();
            com += child->calculateCenterOfMass() * child_mass;
            total_mass += child_mass;
        }
        
        return com / total_mass;
    }
    
    // Recursively update curvature based on a function
    void updateCurvature(const std::function<float(const Vec3&)>& curvatureFunc) {
        curvature = curvatureFunc(position);
        for (auto& child : children) {
            child->updateCurvature(curvatureFunc);
        }
    }
};

/**
 * @brief Binary space tree for efficient 3D physics
 */
class BinarySpaceTree {
public:
    using CurvatureFunction = std::function<float(const Vec3&)>;
    using IntentFunction = std::function<float(const Vec3&)>;
    
    BinarySpaceTree();
    ~BinarySpaceTree() = default;
    
    /**
     * @brief Initialize the tree with a maximum depth
     * @param maxDepth Maximum depth of the tree
     * @return True if initialization succeeded
     */
    bool initialize(int maxDepth);
    
    /**
     * @brief Set the curvature function for non-Euclidean physics
     * @param func Function that returns curvature at a given position
     */
    void setCurvatureFunction(const CurvatureFunction& func);
    
    /**
     * @brief Set the intent function for computational prioritization
     * @param func Function that returns intent priority at a given position
     */
    void setIntentFunction(const IntentFunction& func);
    
    /**
     * @brief Simulate physics for a time step
     * @param dt Time step in seconds
     */
    void integrateFrame(float dt);
    
    /**
     * @brief Apply spacetime curvature effects
     * @param observer Position of the observer
     */
    void applyCurvature(const Vec3& observer);
    
    /**
     * @brief Render nodes with level-of-detail based on focus
     * @param focus_point Point of focus (e.g., camera position)
     * @param max_nodes Maximum number of nodes to render
     * @return Vector of nodes to render
     */
    std::vector<std::shared_ptr<Node3D>> renderLOD(const Vec3& focus_point, size_t max_nodes = 1000);
    
    /**
     * @brief Add a new node to the tree
     * @param node Node to add
     * @return ID of the added node
     */
    uint32_t addNode(const std::shared_ptr<Node3D>& node);
    
    /**
     * @brief Find a node by its ID
     * @param id ID of the node to find
     * @return Shared pointer to the node or nullptr if not found
     */
    std::shared_ptr<Node3D> findNode(uint32_t id) const;
    
    /**
     * @brief Find the nearest node to a position
     * @param position Position to search near
     * @param max_distance Maximum search distance
     * @return Nearest node or nullptr if none found
     */
    std::shared_ptr<Node3D> findNearestNode(const Vec3& position, float max_distance = INFINITY);
    
    /**
     * @brief Find nodes within a radius
     * @param position Center position
     * @param radius Search radius
     * @return Vector of nodes within the radius
     */
    std::vector<std::shared_ptr<Node3D>> findNodesInRadius(const Vec3& position, float radius);
    
    /**
     * @brief Set the energy budget for physics calculations
     * @param energy_budget Energy budget from 0.0 to 1.0
     */
    void setEnergyBudget(float energy_budget);
    
    /**
     * @brief Get the current node count
     * @return Number of nodes in the tree
     */
    size_t getNodeCount() const;
    
    /**
     * @brief Get a random node from the tree
     * @return Shared pointer to a random node or nullptr if empty
     */
    std::shared_ptr<Node3D> getRandomNode() const;
    
    /**
     * @brief Get the maximum tree depth
     * @return Maximum tree depth
     */
    int getMaxDepth() const;
    
    /**
     * @brief Get the current intent function
     * @return The intent function
     */
    IntentFunction getIntentFunction() const;
    
    /**
     * @brief Get the curvature at the root node
     * @return Root node curvature value
     */
    float getRootCurvature() const;
    
private:
    std::shared_ptr<Node3D> root_;
    CurvatureFunction curvatureFunc_;
    IntentFunction intentFunc_;
    int maxDepth_;
    float energyBudget_;
    std::atomic<uint32_t> nextNodeId_;
    std::unordered_map<uint32_t, std::shared_ptr<Node3D>> nodeMap_;
    mutable std::mutex mutex_;
    float simulationTime_;
    
    /**
     * @brief Insert a node at the appropriate position in the tree
     * @param parent Parent node
     * @param node Node to insert
     * @param depth Current depth
     */
    void insertNode_(std::shared_ptr<Node3D> parent, 
                     const std::shared_ptr<Node3D>& node, 
                     int depth);
    
    /**
     * @brief Find nodes for LOD rendering recursively
     * @param node Current node
     * @param focus_point Focus point
     * @param result Result vector
     * @param remaining_budget Remaining node budget
     * @param depth Current depth
     */
    void findLODNodes_(std::shared_ptr<Node3D> node, 
                       const Vec3& focus_point,
                       std::vector<std::shared_ptr<Node3D>>& result,
                       size_t& remaining_budget,
                       int depth);
                       
    /**
     * @brief Calculate forces between nodes
     * @param node1 First node
     * @param node2 Second node
     * @param dt Time step
     */
    void calculateForces_(std::shared_ptr<Node3D> node1, 
                          std::shared_ptr<Node3D> node2,
                          float dt);
                          
    /**
     * @brief Find nodes recursively within a radius
     * @param node Current node
     * @param position Center position
     * @param radius Search radius
     * @param result Result vector
     * @param depth Current depth
     */
    void findNodesInRadius_(std::shared_ptr<Node3D> node,
                           const Vec3& position,
                           float radius,
                           std::vector<std::shared_ptr<Node3D>>& result,
                           int depth);
};

} // namespace core
} // namespace zero_point

# Data Structures

## Overview

The Zero Point Physics Engine utilizes specialized data structures optimized for physics simulation performance, memory efficiency, and numerical stability. This document describes the core data structures and their implementations.

## Vec3 - 3D Vector

The foundational vector class for positions, velocities, and forces:

```cpp
class Vec3 {
public:
    float x, y, z;
    
    // Constructors
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    // Vector operations
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    
    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }
    
    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }
    
    // Length calculations
    float lengthSquared() const {
        return x*x + y*y + z*z;
    }
    
    float length() const {
        return std::sqrt(lengthSquared());
    }
    
    // Normalization
    Vec3 normalized() const {
        float len = length();
        if (len > 0.0001f) {
            return Vec3(x / len, y / len, z / len);
        }
        return *this;
    }
    
    void normalize() {
        float len = length();
        if (len > 0.0001f) {
            x /= len;
            y /= len;
            z /= len;
        }
    }
    
    // Dot product
    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    // Cross product
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
};
```

## AABB - Axis-Aligned Bounding Box

Used for broad-phase collision detection:

```cpp
struct AABB {
    Vec3 min;
    Vec3 max;
    
    AABB() : min(0,0,0), max(0,0,0) {}
    
    AABB(const Vec3& min_, const Vec3& max_) : min(min_), max(max_) {}
    
    // Create AABB from center and half-extents
    AABB(const Vec3& center, float half_width, float half_height, float half_depth) {
        min = Vec3(center.x - half_width, center.y - half_height, center.z - half_depth);
        max = Vec3(center.x + half_width, center.y + half_height, center.z + half_depth);
    }
    
    // Check if this AABB overlaps another
    bool overlaps(const AABB& other) const {
        return (min.x <= other.max.x && max.x >= other.min.x) &&
               (min.y <= other.max.y && max.y >= other.min.y) &&
               (min.z <= other.max.z && max.z >= other.min.z);
    }
    
    // Check if a point is inside this AABB
    bool contains(const Vec3& point) const {
        return (point.x >= min.x && point.x <= max.x) &&
               (point.y >= min.y && point.y <= max.y) &&
               (point.z >= min.z && point.z <= max.z);
    }
    
    // Get the center point
    Vec3 getCenter() const {
        return Vec3(
            (min.x + max.x) * 0.5f,
            (min.y + max.y) * 0.5f,
            (min.z + max.z) * 0.5f
        );
    }
    
    // Get the dimensions
    Vec3 getExtents() const {
        return Vec3(
            (max.x - min.x) * 0.5f,
            (max.y - min.y) * 0.5f,
            (max.z - min.z) * 0.5f
        );
    }
    
    // Union with another AABB
    AABB merge(const AABB& other) const {
        return AABB(
            Vec3(
                std::min(min.x, other.min.x),
                std::min(min.y, other.min.y),
                std::min(min.z, other.min.z)
            ),
            Vec3(
                std::max(max.x, other.max.x),
                std::max(max.y, other.max.y),
                std::max(max.z, other.max.z)
            )
        );
    }
};
```

## Node3D - Physics Node

The primary object for physics simulation:

```cpp
class Node3D {
public:
    // Core properties
    uint64_t id;
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    float mass;
    float radius;  // For sphere collision
    
    // Simulation properties
    bool is_dynamic = true;
    Vec3 accumulated_force;
    float restitution = 0.5f;
    float static_friction = 0.5f;
    float dynamic_friction = 0.3f;
    
    // Material properties
    uint32_t material_id = 0;
    
    // Verification data
    Vec3 prev_position;
    Vec3 prev_velocity;
    
    // Optional properties
    Quaternion orientation;
    Vec3 angular_velocity;
    Vec3 accumulated_torque;
    AABB bounding_box;
    
    // Constructor
    Node3D(uint64_t id_, const Vec3& position_, float mass_, float radius_)
        : id(id_),
          position(position_),
          velocity(0, 0, 0),
          acceleration(0, 0, 0),
          mass(mass_),
          radius(radius_),
          accumulated_force(0, 0, 0),
          prev_position(position_),
          prev_velocity(0, 0, 0),
          orientation(),
          angular_velocity(0, 0, 0),
          accumulated_torque(0, 0, 0) {
        
        updateBoundingBox();
    }
    
    // Apply force to node
    void applyForce(const Vec3& force) {
        if (is_dynamic) {
            accumulated_force += force;
        }
    }
    
    // Apply impulse (instantaneous change in velocity)
    void applyImpulse(const Vec3& impulse) {
        if (is_dynamic) {
            velocity += impulse * (1.0f / mass);
        }
    }
    
    // Update bounding box from position and radius
    void updateBoundingBox() {
        bounding_box.min = Vec3(position.x - radius, position.y - radius, position.z - radius);
        bounding_box.max = Vec3(position.x + radius, position.y + radius, position.z + radius);
    }
    
    // Get inverse mass (0 for static objects)
    float getInverseMass() const {
        return is_dynamic ? (1.0f / mass) : 0.0f;
    }
};
```

## Quaternion - Rotation

Used for representing rotations and orientations:

```cpp
class Quaternion {
public:
    float w, x, y, z;
    
    // Constructors
    Quaternion() : w(1), x(0), y(0), z(0) {}  // Identity quaternion
    
    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}
    
    // Create from axis-angle
    static Quaternion fromAxisAngle(const Vec3& axis, float angle) {
        float half_angle = angle * 0.5f;
        float s = std::sin(half_angle);
        
        Vec3 normalized_axis = axis.normalized();
        
        return Quaternion(
            std::cos(half_angle),
            normalized_axis.x * s,
            normalized_axis.y * s,
            normalized_axis.z * s
        );
    }
    
    // Normalization
    Quaternion normalized() const {
        float len = std::sqrt(w*w + x*x + y*y + z*z);
        if (len > 0.0001f) {
            float inv_len = 1.0f / len;
            return Quaternion(w * inv_len, x * inv_len, y * inv_len, z * inv_len);
        }
        return *this;
    }
    
    // Inverse quaternion
    Quaternion inverse() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    // Multiply quaternions (composition of rotations)
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }
    
    // Rotate a vector by this quaternion
    Vec3 rotate(const Vec3& v) const {
        // Implementation using quaternion-vector multiplication
        Quaternion q_vector(0, v.x, v.y, v.z);
        Quaternion q_result = *this * q_vector * inverse();
        return Vec3(q_result.x, q_result.y, q_result.z);
    }
};
```

## CollisionPair - Collision Information

Stores information about a potential collision:

```cpp
struct CollisionPair {
    Node3D* node_a;
    Node3D* node_b;
    
    CollisionPair(Node3D* a, Node3D* b)
        : node_a(a), node_b(b) {}
        
    // For sorting and containers
    bool operator==(const CollisionPair& other) const {
        return (node_a == other.node_a && node_b == other.node_b) ||
               (node_a == other.node_b && node_b == other.node_a);
    }
};

// Hash function for CollisionPair
namespace std {
    template<>
    struct hash<CollisionPair> {
        size_t operator()(const CollisionPair& pair) const {
            // Commutative hash to ensure a,b and b,a hash the same
            size_t a = reinterpret_cast<size_t>(pair.node_a);
            size_t b = reinterpret_cast<size_t>(pair.node_b);
            return a ^ b; // XOR is commutative
        }
    };
}
```

## CollisionInfo - Detailed Collision Data

Stores detailed information about a collision:

```cpp
struct CollisionInfo {
    Vec3 collision_normal;     // Direction of collision
    Vec3 contact_point;        // Point of contact
    float penetration_depth;   // How deeply objects are intersecting
    Node3D* node_a;            // First object in collision
    Node3D* node_b;            // Second object in collision
    
    CollisionInfo()
        : collision_normal(0, 1, 0),
          contact_point(0, 0, 0),
          penetration_depth(0),
          node_a(nullptr),
          node_b(nullptr) {}
};
```

## Military-Grade Extensions

For military applications, additional security and integrity features:

```cpp
struct VerificationResult {
    bool success;
    float confidence;
    std::string hash;
    std::unordered_map<std::string, double> metrics;
    
    VerificationResult()
        : success(false), confidence(0.0f) {}
};

struct CryptoSignature {
    std::string data_hash;
    std::string signature;
    uint64_t timestamp;
    
    CryptoSignature()
        : timestamp(0) {}
};
```

## Performance Considerations

- Data structures are designed for memory alignment and cache efficiency
- Minimal object overhead through careful member ordering
- Support for both single and double precision as needed
- Custom memory allocators for high-performance scenarios

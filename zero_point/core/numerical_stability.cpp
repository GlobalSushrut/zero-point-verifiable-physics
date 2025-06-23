#include "numerical_stability.hpp"
#include <algorithm>
#include <cmath>

namespace zero_point {
namespace core {

// Static member initialization
bool NumericalStability::use_compensated_summation_ = true;
bool NumericalStability::use_interval_arithmetic_ = true;

void NumericalStability::initialize(bool use_compensated_summation,
                                  bool use_interval_arithmetic) {
    use_compensated_summation_ = use_compensated_summation;
    use_interval_arithmetic_ = use_interval_arithmetic;
}

Vec3 NumericalStability::safeNormalize(const Vec3& v, float epsilon) {
    float length_sq = v.lengthSquared();
    
    if (length_sq < epsilon * epsilon) {
        // Vector is too small to normalize safely
        return Vec3(0.0f, 0.0f, 0.0f);
    }
    
    float inv_length = 1.0f / std::sqrt(length_sq);
    return v * inv_length;
}

double NumericalStability::stableDot(const Vec3& a, const Vec3& b) {
    // Use double precision for intermediate calculations
    double x = static_cast<double>(a.x) * static_cast<double>(b.x);
    double y = static_cast<double>(a.y) * static_cast<double>(b.y);
    double z = static_cast<double>(a.z) * static_cast<double>(b.z);
    
    // Use compensated summation if enabled
    if (use_compensated_summation_) {
        double values[3] = {x, y, z};
        return compensatedSum(values, 3);
    } else {
        return x + y + z;
    }
}

bool NumericalStability::validatePosition(Vec3& position, const Vec3& velocity, float dt) {
    // Check for NaN or infinity
    if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z) ||
        std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z)) {
        // Reset to origin - this is a critical error in a real system
        position = Vec3(0.0f, 0.0f, 0.0f);
        return false;
    }
    
    // Check for excessively large values
    constexpr float MAX_POSITION = 1.0e9f;  // 1 billion units
    if (std::abs(position.x) > MAX_POSITION ||
        std::abs(position.y) > MAX_POSITION ||
        std::abs(position.z) > MAX_POSITION) {
        // Cap at maximum bounds
        position.x = std::clamp(position.x, -MAX_POSITION, MAX_POSITION);
        position.y = std::clamp(position.y, -MAX_POSITION, MAX_POSITION);
        position.z = std::clamp(position.z, -MAX_POSITION, MAX_POSITION);
        return false;
    }
    
    // Check for potential tunneling (object moving too fast)
    constexpr float MAX_DISPLACEMENT = 1000.0f;  // Maximum safe displacement
    float displacement = velocity.length() * dt;
    if (displacement > MAX_DISPLACEMENT) {
        return false;
    }
    
    return true;
}

std::string NumericalStability::checkNodeStability(const Node3D& node) {
    // Check position
    if (std::isnan(node.position.x) || std::isnan(node.position.y) || std::isnan(node.position.z)) {
        return "Position contains NaN values";
    }
    
    if (std::isinf(node.position.x) || std::isinf(node.position.y) || std::isinf(node.position.z)) {
        return "Position contains infinite values";
    }
    
    // Check velocity
    if (std::isnan(node.velocity.x) || std::isnan(node.velocity.y) || std::isnan(node.velocity.z)) {
        return "Velocity contains NaN values";
    }
    
    if (std::isinf(node.velocity.x) || std::isinf(node.velocity.y) || std::isinf(node.velocity.z)) {
        return "Velocity contains infinite values";
    }
    
    // Check for excessively high velocity
    constexpr float MAX_SAFE_VELOCITY = 1.0e6f;
    float velocity_mag = node.velocity.length();
    if (velocity_mag > MAX_SAFE_VELOCITY) {
        return "Velocity magnitude exceeds safe limits: " + std::to_string(velocity_mag);
    }
    
    // Check for near-zero or negative mass
    if (node.mass <= 1.0e-6f) {
        return "Mass is too small or negative: " + std::to_string(node.mass);
    }
    
    // Check for excessive forces
    constexpr float MAX_SAFE_FORCE = 1.0e9f;
    float force_mag = node.accumulated_force.length();
    if (force_mag > MAX_SAFE_FORCE) {
        return "Force magnitude exceeds safe limits: " + std::to_string(force_mag);
    }
    
    // All checks passed
    return "";
}

Vec3 NumericalStability::regularizeVelocity(const Vec3& velocity, float max_speed) {
    float speed = velocity.length();
    
    if (speed > max_speed) {
        // Scale down velocity while preserving direction
        return velocity * (max_speed / speed);
    }
    
    return velocity;
}

double NumericalStability::conditionNumber(const float* matrix, size_t size) {
    // Simple estimation of condition number for small matrices
    // For a real implementation, use SVD or eigenvalues
    
    // For now, return a dummy value
    return 1.0;
    
    // In a real implementation:
    // 1. Calculate SVD of matrix
    // 2. Return ratio of largest to smallest singular value
}

} // namespace core
} // namespace zero_point

#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <array>
#include <vector>
#include "binary_space_tree.hpp"

namespace zero_point {
namespace core {

/**
 * @brief Military/NASA-grade numerical stability utilities
 * 
 * This class provides specialized numerical methods for ensuring extreme
 * precision and stability in physics calculations, especially in edge cases.
 */
class NumericalStability {
public:
    /**
     * @brief Initialize numerical stability system
     * @param use_compensated_summation Enable Kahan summation for floating point
     * @param use_interval_arithmetic Enable interval arithmetic for bounds checking
     */
    static void initialize(bool use_compensated_summation = true,
                          bool use_interval_arithmetic = true);
    
    /**
     * @brief Safe vector normalization that avoids division by zero
     * @param v Vector to normalize
     * @param epsilon Minimum length before considering zero
     * @return Normalized vector or zero vector if length < epsilon
     */
    static Vec3 safeNormalize(const Vec3& v, float epsilon = 1e-8f);
    
    /**
     * @brief Stable dot product using extended precision
     * @param a First vector
     * @param b Second vector
     * @return High precision dot product
     */
    static double stableDot(const Vec3& a, const Vec3& b);
    
    /**
     * @brief Detect and prevent numerical instability in position
     * @param position Position to check
     * @param velocity Velocity vector
     * @param dt Time step
     * @return True if position is numerically stable
     */
    static bool validatePosition(Vec3& position, const Vec3& velocity, float dt);
    
    /**
     * @brief Compensated summation (Kahan algorithm) for reducing floating point errors
     * @param values Array of values to sum
     * @param count Number of values
     * @return Precise sum with error compensation
     */
    template<typename T>
    static T compensatedSum(const T* values, size_t count) {
        T sum = 0;
        T compensation = 0;
        
        for (size_t i = 0; i < count; i++) {
            T y = values[i] - compensation;
            T t = sum + y;
            compensation = (t - sum) - y;
            sum = t;
        }
        
        return sum;
    }
    
    /**
     * @brief Compute solution bounds using interval arithmetic
     * @param value Nominal value
     * @param error_bound Error bound
     * @return Array containing [min, max] bounds
     */
    template<typename T>
    static std::array<T, 2> intervalBounds(T value, T error_bound) {
        return {value - error_bound, value + error_bound};
    }
    
    /**
     * @brief Check if a node is at risk of numerical instability
     * @param node Node to check
     * @return Empty string if stable, or error message if unstable
     */
    static std::string checkNodeStability(const Node3D& node);
    
    /**
     * @brief Regularize velocity to prevent excessive speed
     * @param velocity Velocity vector to regularize
     * @param max_speed Maximum allowable speed
     * @return Regularized velocity
     */
    static Vec3 regularizeVelocity(const Vec3& velocity, float max_speed = 1e6f);
    
    /**
     * @brief Test if a physical quantity exceeds safe bounds
     * @param value Value to check
     * @param min_safe Minimum safe value
     * @param max_safe Maximum safe value
     * @return True if value is in safe range
     */
    template<typename T>
    static bool isInSafeRange(T value, T min_safe, T max_safe) {
        return value >= min_safe && value <= max_safe && !std::isnan(value) && !std::isinf(value);
    }
    
    /**
     * @brief Get machine epsilon for float type
     * @return Epsilon value
     */
    static constexpr float epsilon() {
        return std::numeric_limits<float>::epsilon();
    }
    
    /**
     * @brief Calculate relative error between computed and reference values
     * @param computed Computed value
     * @param reference Reference value
     * @return Relative error or 0 if reference is zero
     */
    template<typename T>
    static T relativeError(T computed, T reference) {
        if (std::abs(reference) < std::numeric_limits<T>::min()) {
            return std::abs(computed);
        }
        return std::abs((computed - reference) / reference);
    }
    
    /**
     * @brief Generate condition number for a system (measure of numerical stability)
     * @param matrix System matrix
     * @param size Matrix size (NxN)
     * @return Condition number (higher means less stable)
     */
    static double conditionNumber(const float* matrix, size_t size);
    
private:
    static bool use_compensated_summation_;
    static bool use_interval_arithmetic_;
};

} // namespace core
} // namespace zero_point

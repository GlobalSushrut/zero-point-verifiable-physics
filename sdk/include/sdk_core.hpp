#pragma once
#include <memory>
#include <functional>
#include <array>
#include "tree_binary_env.hpp"

namespace physics {
namespace sdk {

/**
 * PhysicsSDK is the main interface for the physics engine.
 * It provides initialization, configuration and management of the physics environment.
 * This class aims to provide Unity-level physics capabilities on resource-constrained hardware.
 */
template <typename T = float, int Dimensions = 3>
class PhysicsSDK {
public:
    using VectorT = std::array<T, Dimensions>;
    using CurvatureFunction = std::function<T(const VectorT&)>;
    using IntentFunction = std::function<float(const VectorT&)>;
    using EnvPtr = std::shared_ptr<TreeBinaryEnvironment<T, Dimensions>>;
    
    /**
     * Initialize the SDK with the given parameters
     * @param maxDepth Maximum depth for spatial tree structure
     * @param enableHardwareAcceleration Whether to enable hardware acceleration
     * @return True if initialization succeeded, false otherwise
     */
    bool initialize(int maxDepth = 8, bool enableHardwareAcceleration = true) {
        environment_ = std::make_shared<TreeBinaryEnvironment<T, Dimensions>>();
        if (!environment_->initialize(maxDepth)) {
            return false;
        }
        
        hardwareAccelerationEnabled_ = enableHardwareAcceleration;
        initialized_ = true;
        return true;
    }
    
    /**
     * Set the curvature function for non-Euclidean physics
     * @param func Function that computes curvature at a given position
     */
    void setCurvatureFunction(const CurvatureFunction& func) {
        curvatureFunction_ = func;
        if (environment_) {
            environment_->setCurvatureFunction(func);
        }
    }
    
    /**
     * Set the intent function to prioritize computation
     * @param func Function that computes intent/importance at a given position
     */
    void setIntentFunction(const IntentFunction& func) {
        intentFunction_ = func;
        if (environment_) {
            environment_->setIntentFunction(func);
        }
    }
    
    /**
     * Check if hardware acceleration is enabled
     * @return True if hardware acceleration is enabled
     */
    bool isHardwareAccelerationEnabled() const {
        return hardwareAccelerationEnabled_;
    }
    
    /**
     * Get the environment instance
     * @return Shared pointer to the environment
     */
    EnvPtr getEnvironment() const {
        return environment_;
    }
    
    /**
     * Update the physics state
     * @param dt Time step in seconds
     */
    void update(float dt) {
        if (environment_) {
            environment_->update(dt);
        }
    }
    
    /**
     * Get the current simulation time
     * @return Current simulation time in seconds
     */
    float getSimulationTime() const {
        return environment_ ? environment_->getSimulationTime() : 0.0f;
    }
    
private:
    EnvPtr environment_;
    CurvatureFunction curvatureFunction_;
    IntentFunction intentFunction_;
    bool initialized_ = false;
    bool hardwareAccelerationEnabled_ = false;
};

// Function to get version information
extern "C" {
    const char* getSdkVersion();
}

} // namespace sdk
} // namespace physics

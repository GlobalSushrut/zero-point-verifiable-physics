#pragma once

#include "physics_solver.hpp"
#include "formal_verify.hpp"
#include "numerical_stability.hpp"
#include "integrity_check.hpp"
#include "rt_scheduler.hpp"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace zero_point {
namespace core {

/**
 * @brief Military/NASA-grade enhanced physics solver with formal verification
 */
class PhysicsSolverMilitary : public PhysicsSolver {
public:
    /**
     * @brief Enhanced constructor with military-grade components
     * @param solver_type Type of physics solver
     * @param verification_level Level of formal verification
     * @param use_strict_timing Enable strict deterministic timing
     */
    PhysicsSolverMilitary(
        SolverType solver_type = SolverType::PARTICLE,
        VerificationLevel verification_level = VerificationLevel::PARTIAL,
        bool use_strict_timing = true);
        
    /**
     * @brief Destructor
     */
    ~PhysicsSolverMilitary();
    
    /**
     * @brief Override to add formal verification and monitoring
     * @param dt Time step
     * @return True if step completed successfully
     */
    bool step(float dt);
    
    /**
     * @brief Add node with enhanced validation
     * @param position Initial position
     * @param is_dynamic Whether node is dynamic
     * @param mass Node mass
     * @return Node ID
     */
    uint32_t addNode(const Vec3& position, bool is_dynamic = true, 
                   float mass = 1.0f);
    
    /**
     * @brief Set gravity with numerical stability checks
     * @param gravity Gravity vector
     */
    void setGravity(const Vec3& gravity);
    
    /**
     * @brief Get enhanced diagnostics
     * @return Diagnostic information
     */
    std::unordered_map<std::string, float> getPerformanceMetrics() const;
    
    /**
     * @brief Set CPU cores for physics computation
     * @param cpu_cores List of CPU core IDs
     * @return True if affinity was set
     */
    bool setCpuAffinity(const std::vector<int>& cpu_cores);
    
    /**
     * @brief Set formal verification level
     * @param level Verification level
     */
    void setVerificationLevel(VerificationLevel level);
    
    /**
     * @brief Set numerical stability options
     * @param use_compensated_summation Use Kahan summation
     * @param use_interval_arithmetic Use interval arithmetic
     */
    void setNumericalStabilityOptions(
        bool use_compensated_summation, 
        bool use_interval_arithmetic);
    
    /**
     * @brief Generate cryptographic integrity signature for simulation state
     * @return Hex-encoded signature string
     */
    std::string generateStateSignature() const;
    
    /**
     * @brief Verify integrity of critical components
     * @param strict_checking Use stricter checking
     * @return True if integrity verified
     */
    bool verifySystemIntegrity(bool strict_checking = false) const;
    
    /**
     * @brief Get formal verification statistics
     * @return Statistics as key-value pairs
     */
    std::unordered_map<std::string, double> getVerificationStatistics() const;
    
    /**
     * @brief Get scheduler statistics
     * @return Statistics as key-value pairs
     */
    std::map<std::string, double> getSchedulerStatistics() const;

protected:
    /**
     * @brief Override processStep_ to add military-grade features
     * @param dt Time step
     */
    void processStep_(float dt);
    
private:
    // Military-grade components
    std::unique_ptr<FormalVerify> formal_verifier_;
    std::unique_ptr<IntegrityCheck> integrity_checker_;
    std::unique_ptr<RTScheduler> rt_scheduler_;
    
    // Pre-collision state for verification
    std::vector<Node3D> pre_collision_state_;
    
    // Timing and diagnostics
    std::chrono::high_resolution_clock::time_point sim_start_time_;
    double total_simulation_time_s_;
    uint64_t verification_count_;
    uint64_t integrity_checks_;
    double verification_time_ms_;
    VerificationLevel verification_level_;
    
    // Enhanced memory protection
    std::vector<uint8_t> hash_buffer_;
    
    // Thread sanitization
    void sanitizeThreadExecution_();
    
    // Schedule physics tasks with real-time guarantees
    void schedulePhysicsTasks_();
    
    // Formal verification of physics operations
    bool verifyPhysicsOperations_(float dt);
    
    // Memory integrity validation
    void validateComponentIntegrity_();
};

} // namespace core
} // namespace zero_point

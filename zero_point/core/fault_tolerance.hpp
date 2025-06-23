#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <stdexcept>

namespace zero_point {
namespace core {

// Error severity levels for proper categorization
enum class ErrorSeverity {
    INFO,       // Informational, non-critical
    WARNING,    // Potential issue, can continue
    ERROR,      // Serious issue, may need recovery
    CRITICAL,   // Critical failure, recovery may not be possible
    FATAL       // Fatal error, system shutdown required
};

// Error source for traceability
enum class ErrorSource {
    PHYSICS,            // Physics calculation error
    MEMORY,             // Memory allocation/access error
    NUMERICAL,          // Numerical stability error
    THREAD,             // Threading/concurrency error
    BOUNDARY,           // Boundary condition error
    VERIFICATION,       // Verification failure
    HARDWARE,           // Hardware-related error
    SECURITY,           // Security breach
    SYSTEM              // General system error
};

/**
 * @brief Structured error report for detailed error tracking
 */
struct ErrorReport {
    ErrorSeverity severity;
    ErrorSource source;
    std::string message;
    std::string location;  // File:line
    std::string function;
    std::string stack_trace;
    uint64_t timestamp;
    uint64_t error_id;
    double numerical_value;  // Optional numerical value related to error
    bool requires_immediate_action;
    
    ErrorReport() : 
        severity(ErrorSeverity::INFO),
        source(ErrorSource::SYSTEM),
        timestamp(std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now().time_since_epoch()).count()),
        error_id(0),
        numerical_value(0.0),
        requires_immediate_action(false) {}
        
    std::string toString() const;
};

/**
 * @brief Exception class for simulation errors with detailed diagnostics
 */
class SimulationException : public std::runtime_error {
public:
    SimulationException(const ErrorReport& report);
    
    const ErrorReport& getErrorReport() const { return report_; }
    
private:
    ErrorReport report_;
};

/**
 * @brief Class for managing fault tolerance and recovery
 */
class FaultTolerance {
public:
    using ErrorCallback = std::function<void(const ErrorReport&)>;
    using RecoveryFunction = std::function<bool(const ErrorReport&)>;
    
    /**
     * @brief Initialize fault tolerance system
     * @param log_file Path to error log file
     * @param enable_watchdog Enable watchdog monitoring
     * @return True if successfully initialized
     */
    static bool initialize(const std::string& log_file = "simulation_errors.log", 
                          bool enable_watchdog = true);
                          
    /**
     * @brief Report an error condition
     * @param severity Error severity
     * @param source Error source
     * @param message Error message
     * @param file Source file where error occurred
     * @param line Line number where error occurred
     * @param function Function name where error occurred
     * @return Error report ID for tracking
     */
    static uint64_t reportError(ErrorSeverity severity,
                               ErrorSource source,
                               const std::string& message,
                               const char* file,
                               int line,
                               const char* function);
                               
    /**
     * @brief Register error callback for notification
     * @param callback Function to call when error occurs
     * @param min_severity Minimum severity level to trigger callback
     */
    static void registerErrorCallback(const ErrorCallback& callback, 
                                     ErrorSeverity min_severity = ErrorSeverity::WARNING);
                                     
    /**
     * @brief Register recovery function for specific error source
     * @param source Error source
     * @param recovery Function to attempt recovery
     */
    static void registerRecoveryFunction(ErrorSource source, 
                                        const RecoveryFunction& recovery);
                                        
    /**
     * @brief Check system health status
     * @return True if system is healthy
     */
    static bool isHealthy();
    
    /**
     * @brief Reset error counts and status
     */
    static void resetErrorState();
    
    /**
     * @brief Enable or disable the watchdog
     * @param enabled Whether watchdog should be enabled
     * @param timeout_ms Watchdog timeout in milliseconds
     */
    static void setWatchdog(bool enabled, uint32_t timeout_ms = 1000);
    
    /**
     * @brief Pet the watchdog to prevent timeout
     */
    static void petWatchdog();
    
    /**
     * @brief Get count of errors by severity
     * @param severity Error severity
     * @return Number of errors of specified severity
     */
    static uint32_t getErrorCount(ErrorSeverity severity);
    
    /**
     * @brief Generate a system diagnostic report
     * @return JSON string with diagnostic information
     */
    static std::string generateDiagnosticReport();
    
    /**
     * @brief Check if a numerical value is within safe bounds
     * @param value Value to check
     * @param min_safe Minimum safe value
     * @param max_safe Maximum safe value
     * @param value_name Name of the value (for error reporting)
     * @param file Source file
     * @param line Line number
     * @param function Function name
     * @return True if value is safe
     */
    static bool checkNumericalSafety(double value, 
                                   double min_safe, 
                                   double max_safe,
                                   const char* value_name,
                                   const char* file,
                                   int line,
                                   const char* function);
                                   
    /**
     * @brief Verify memory integrity of a buffer
     * @param buffer Memory buffer
     * @param size Size in bytes
     * @param expected_checksum Expected checksum (0 to calculate new one)
     * @return True if memory is intact
     */
    static bool verifyMemoryIntegrity(const void* buffer, 
                                    size_t size, 
                                    uint32_t expected_checksum = 0);
                                    
private:
    static std::mutex mutex_;
    static std::atomic<bool> initialized_;
    static std::string log_file_;
    static std::atomic<uint64_t> next_error_id_;
    static std::vector<ErrorCallback> callbacks_;
    static std::vector<ErrorSeverity> callback_severities_;
    static std::unordered_map<ErrorSource, RecoveryFunction> recovery_functions_;
    static std::atomic<uint32_t> error_counts_[5];  // One for each severity
    
    // Watchdog related
    static std::atomic<bool> watchdog_enabled_;
    static std::atomic<uint32_t> watchdog_timeout_ms_;
    static std::atomic<uint64_t> last_pet_time_;
    static std::thread watchdog_thread_;
    static std::atomic<bool> watchdog_running_;
    
    static void watchdogThreadFunc_();
    static void logError_(const ErrorReport& report);
    static std::string getCurrentTimeString_();
    static std::string captureStackTrace_();
    static uint32_t calculateChecksum_(const void* data, size_t size);
};

/**
 * @brief Triple redundancy for critical calculations
 * @tparam T Type of value
 */
template<typename T>
class TripleRedundant {
public:
    TripleRedundant(const T& initial_value = T())
        : values_{initial_value, initial_value, initial_value},
          last_valid_value_(initial_value) {}
          
    /**
     * @brief Set all three values
     * @param value New value
     */
    void set(const T& value) {
        values_[0] = value;
        values_[1] = value;
        values_[2] = value;
        last_valid_value_ = value;
    }
    
    /**
     * @brief Update one of the redundant values
     * @param index Index to update (0-2)
     * @param value New value
     */
    void update(int index, const T& value) {
        if (index >= 0 && index < 3) {
            values_[index] = value;
        }
    }
    
    /**
     * @brief Get value with majority voting
     * @return Voted value
     */
    T get() const {
        // Find which values agree
        bool equal01 = values_[0] == values_[1];
        bool equal12 = values_[1] == values_[2];
        bool equal02 = values_[0] == values_[2];
        
        if (equal01 && equal12) {
            // All agree
            return values_[0];
        } else if (equal01) {
            // 0 and 1 agree
            return values_[0];
        } else if (equal12) {
            // 1 and 2 agree
            return values_[1];
        } else if (equal02) {
            // 0 and 2 agree
            return values_[0];
        }
        
        // No agreement, report error and return last valid value
        FaultTolerance::reportError(
            ErrorSeverity::ERROR,
            ErrorSource::NUMERICAL,
            "Triple redundancy check failed: no majority agreement",
            __FILE__, __LINE__, __func__);
            
        return last_valid_value_;
    }
    
    /**
     * @brief Check if all redundant values agree
     * @return True if all three values are equal
     */
    bool isConsistent() const {
        return (values_[0] == values_[1]) && (values_[1] == values_[2]);
    }
    
    /**
     * @brief Get redundancy status
     * @return Number of agreeing redundant systems (1-3)
     */
    int getConsensusCount() const {
        bool equal01 = values_[0] == values_[1];
        bool equal12 = values_[1] == values_[2];
        bool equal02 = values_[0] == values_[2];
        
        if (equal01 && equal12) return 3;
        if (equal01 || equal12 || equal02) return 2;
        return 1;
    }
    
    /**
     * @brief Assignment operator
     */
    TripleRedundant& operator=(const T& value) {
        set(value);
        return *this;
    }
    
    /**
     * @brief Conversion operator
     */
    operator T() const {
        return get();
    }
    
private:
    T values_[3];
    T last_valid_value_;
};

/**
 * @brief Macros for easier error reporting
 */
#define REPORT_INFO(source, message) \
    zero_point::core::FaultTolerance::reportError( \
        zero_point::core::ErrorSeverity::INFO, source, message, __FILE__, __LINE__, __func__)

#define REPORT_WARNING(source, message) \
    zero_point::core::FaultTolerance::reportError( \
        zero_point::core::ErrorSeverity::WARNING, source, message, __FILE__, __LINE__, __func__)

#define REPORT_ERROR(source, message) \
    zero_point::core::FaultTolerance::reportError( \
        zero_point::core::ErrorSeverity::ERROR, source, message, __FILE__, __LINE__, __func__)

#define REPORT_CRITICAL(source, message) \
    zero_point::core::FaultTolerance::reportError( \
        zero_point::core::ErrorSeverity::CRITICAL, source, message, __FILE__, __LINE__, __func__)

#define REPORT_FATAL(source, message) \
    zero_point::core::FaultTolerance::reportError( \
        zero_point::core::ErrorSeverity::FATAL, source, message, __FILE__, __LINE__, __func__)

#define CHECK_NUMERICAL_SAFETY(value, min, max, name) \
    zero_point::core::FaultTolerance::checkNumericalSafety( \
        value, min, max, name, __FILE__, __LINE__, __func__)

} // namespace core
} // namespace zero_point

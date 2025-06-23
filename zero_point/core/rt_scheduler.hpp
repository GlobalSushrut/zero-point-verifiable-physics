#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <condition_variable>
#include <string>
#include <map>

namespace zero_point {
namespace core {

/**
 * @brief Task priority levels for the real-time scheduler
 */
enum class TaskPriority {
    CRITICAL,   // Highest priority, must execute immediately (e.g., emergency stop)
    HIGH,       // High priority, deadline-sensitive (e.g., collision response)
    NORMAL,     // Normal priority (e.g., regular physics update)
    LOW,        // Low priority, background tasks (e.g., data logging)
    IDLE        // Lowest priority, only execute when system is idle
};

/**
 * @brief Task scheduling parameters
 */
struct TaskParameters {
    std::string task_name;               // Task identifier
    TaskPriority priority;               // Task priority
    uint32_t deadline_us;                // Deadline in microseconds (0 = no deadline)
    bool is_periodic;                    // True if task should be rescheduled automatically
    uint32_t period_us;                  // Period in microseconds (for periodic tasks)
    bool allow_parallel;                 // Whether task can run in parallel with itself
    
    TaskParameters() :
        priority(TaskPriority::NORMAL),
        deadline_us(0),
        is_periodic(false),
        period_us(0),
        allow_parallel(false) {}
};

/**
 * @brief Task execution statistics
 */
struct TaskStatistics {
    uint64_t execution_count;            // Number of times task has executed
    double average_execution_time_us;    // Average execution time in microseconds
    double max_execution_time_us;        // Maximum execution time in microseconds
    double min_execution_time_us;        // Minimum execution time in microseconds
    double deadline_miss_rate;           // Rate of deadline misses (0.0-1.0)
    uint64_t total_deadline_misses;      // Total number of deadline misses
    TaskParameters params;               // Reference to the task parameters
    
    TaskStatistics() : 
        execution_count(0),
        average_execution_time_us(0.0),
        max_execution_time_us(0.0),
        min_execution_time_us(std::numeric_limits<double>::max()),
        deadline_miss_rate(0.0),
        total_deadline_misses(0),
        params() {}
};

/**
 * @brief Task for scheduler
 */
struct Task {
    uint64_t id;                         // Unique task ID
    TaskParameters params;               // Task parameters
    std::function<void()> function;      // Task function to execute
    std::chrono::steady_clock::time_point schedule_time; // When to execute task
    std::chrono::steady_clock::time_point deadline_time; // Task deadline
    
    // For sorting in priority queue (highest priority first, then earliest time)
    bool operator>(const Task& other) const {
        if (params.priority != other.params.priority) {
            return params.priority > other.params.priority;
        }
        return schedule_time > other.schedule_time;
    }
};

/**
 * @brief Military-grade deterministic real-time scheduler
 * 
 * This scheduler ensures predictable execution timing for simulation events,
 * with support for priority-based preemption, deadline monitoring, and 
 * statistical analysis of timing jitter.
 */
class RTScheduler {
public:
    using TaskFunction = std::function<void()>;
    using TaskId = uint64_t;
    
    /**
     * @brief Initialize the scheduler
     * @param thread_count Number of worker threads
     * @param strict_timing Enable strict timing guarantees
     */
    RTScheduler(size_t thread_count = 0, bool strict_timing = false);
    
    /**
     * @brief Destroy scheduler and stop all threads
     */
    ~RTScheduler();
    
    /**
     * @brief Schedule a new task
     * @param function Task function to execute
     * @param params Task parameters
     * @param delay_us Delay before execution in microseconds
     * @return Task ID for reference
     */
    TaskId scheduleTask(const TaskFunction& function,
                      const TaskParameters& params,
                      uint64_t delay_us = 0);
    
    /**
     * @brief Cancel a previously scheduled task
     * @param task_id Task ID to cancel
     * @return True if task was found and cancelled
     */
    bool cancelTask(TaskId task_id);
    
    /**
     * @brief Get statistics for a task
     * @param task_name Task name
     * @return Task statistics
     */
    TaskStatistics getTaskStatistics(const std::string& task_name) const;
    
    /**
     * @brief Get overall scheduler statistics
     * @return Statistics as key-value pairs
     */
    std::map<std::string, double> getSchedulerStatistics() const;
    
    /**
     * @brief Set scheduler CPU affinity
     * @param cpu_cores Vector of CPU core IDs to use
     * @return True if affinity was set successfully
     */
    bool setCpuAffinity(const std::vector<int>& cpu_cores);
    
    /**
     * @brief Enable or disable strict timing mode
     * @param strict_timing True to enable strict timing
     */
    void setStrictTiming(bool strict_timing);
    
    /**
     * @brief Start the scheduler
     */
    void start();
    
    /**
     * @brief Stop the scheduler
     */
    void stop();
    
    /**
     * @brief Check if scheduler is running
     * @return True if scheduler is running
     */
    bool isRunning() const;
    
    /**
     * @brief Get current schedule utilization
     * @return Utilization as percentage (0.0-100.0)
     */
    double getUtilization() const;
    
private:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::steady_clock::time_point;
    
    // Task comparison for priority queue (higher priority first)
    struct TaskCompare {
        bool operator()(const Task& a, const Task& b) const {
            // Sort by priority first, then by schedule time
            if (a.params.priority != b.params.priority) {
                return a.params.priority < b.params.priority;
            }
            return a.schedule_time > b.schedule_time;
        }
    };
    
    // Scheduler state
    std::atomic<bool> running_;
    std::atomic<bool> strict_timing_;
    std::atomic<uint64_t> next_task_id_;
    std::vector<std::thread> worker_threads_;
    
    // Task queue
    std::priority_queue<Task, std::vector<Task>, TaskCompare> task_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable cv_;
    
    // Task statistics
    std::map<std::string, TaskStatistics> task_statistics_;
    mutable std::mutex stats_mutex_;
    
    // Overall scheduler statistics
    std::atomic<uint64_t> total_tasks_executed_;
    std::atomic<uint64_t> total_deadline_misses_;
    
    // Internal methods
    void workerThread_();
    void executeTask_(const Task& task);
    void reschedulePeriodicTask_(const Task& task);
    TimePoint calculateNextExecutionTime_(uint64_t period_us);
    void updateTaskStatistics_(const std::string& task_name,
                             double execution_time_us,
                             bool deadline_missed);
};

} // namespace core
} // namespace zero_point

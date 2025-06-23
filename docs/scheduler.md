# Real-Time Scheduler

## Overview

The Real-Time Scheduler is a critical component of the Zero Point Physics Engine, ensuring deterministic execution of physics operations with strict timing guarantees. This system manages task priorities, deadlines, and worker threads to achieve predictable performance in time-sensitive environments.

## Architecture

```
┌─────────────────────────────────────────────┐
│               RTScheduler                   │
└─┬───────────────────────────────────────────┘
  │
  ├─── Task Queue Management
  │    ├── Priority Queue
  │    └── Deadline Tracking
  │
  ├─── Worker Thread Pool
  │    ├── Thread Creation & Management
  │    └── CPU Affinity Control
  │
  ├─── Task Execution
  │    ├── Function Invocation
  │    └── Performance Timing
  │
  └─── Statistics Collection
       ├── Utilization Tracking
       ├── Deadline Miss Rate
       └── Execution Time Recording
```

## Key Components

### Task Structure

Each task in the scheduler represents a unit of work:

```cpp
struct Task {
    uint64_t id;                  // Unique task identifier
    TaskParameters params;        // Configuration parameters
    std::function<void()> function; // The actual work to be done
    Clock::time_point schedule_time;  // When to execute
    Clock::time_point deadline_time;  // Latest execution time
};
```

### Task Parameters

```cpp
struct TaskParameters {
    std::string task_name;        // Descriptive name
    int32_t priority;             // Execution priority
    uint64_t period_us;           // For periodic tasks
    uint64_t deadline_us;         // Maximum execution deadline
    bool is_periodic;             // Whether to reschedule
};
```

### Task Statistics

The scheduler tracks extensive performance metrics:

```cpp
struct TaskStatistics {
    std::string task_name;
    uint64_t execution_count;     // Number of executions
    uint64_t deadline_misses;     // Number of deadline violations
    double average_execution_time_us; // Average execution time
    double max_execution_time_us; // Maximum execution time
    double min_execution_time_us; // Minimum execution time
};
```

## Thread Management

### Worker Threads

The scheduler creates and manages a pool of worker threads:

1. Thread initialization with configurable count
2. CPU affinity settings for deterministic performance 
3. Work distribution across available cores
4. Controlled thread shutdown on exit

### Thread Safety

The scheduler implements several mechanisms for thread safety:

- Mutex protection for queue operations
- Atomic operations for counters
- Condition variables for thread signaling
- Memory barriers for synchronization

## Task Scheduling

### Priority-Based Scheduling

Tasks are ordered primarily by priority:

- Higher priority tasks execute before lower priority
- Same-priority tasks execute in order of schedule time
- Priority inversions are detected and mitigated

### Deadline Monitoring

Each task can specify a deadline:

- Absolute deadline = schedule time + deadline_us
- Scheduler tracks missed deadlines
- Statistical data on deadline misses is maintained
- Configurable actions on deadline miss

## Periodic Tasks

The scheduler has special handling for periodic tasks:

1. Task executes at its scheduled time
2. If task is periodic, it's rescheduled with:
   - New unique ID
   - New schedule time = now + period_us
   - New deadline time = schedule time + deadline_us
3. Rescheduled task is inserted into the queue

## Performance Metrics

### Utilization

The scheduler calculates CPU utilization based on:

- Actual execution time vs available time
- Task period-based theoretical utilization
- Combined utilization metric for accurate reporting

### Statistics

Comprehensive statistics are maintained:

- Per-task execution count and times
- Global deadline miss rate
- Overall scheduler utilization
- Task execution history

## Usage

### Creating the Scheduler

```cpp
// Create scheduler with 3 worker threads
auto scheduler = std::make_shared<RTScheduler>(3);

// Set CPU affinity for deterministic performance
std::vector<int> cpu_cores = {1, 2, 3}; // Use cores 1-3
scheduler->setCpuAffinity(cpu_cores);
```

### Adding Tasks

```cpp
// Create task parameters
TaskParameters params;
params.task_name = "PhysicsStep";
params.priority = 10;
params.period_us = 16666; // ~60Hz
params.deadline_us = 15000; // 15ms deadline
params.is_periodic = true;

// Add task to scheduler
scheduler->addTask([this]() { 
    this->performPhysicsStep(); 
}, params);
```

### Starting and Stopping

```cpp
// Start scheduler
scheduler->start();

// Run simulation...

// Stop scheduler
scheduler->stop();

// Get performance metrics
auto stats = scheduler->getAllTaskStatistics();
auto utilization = scheduler->getUtilization();
```

## Military-Grade Extensions

For defense and aerospace applications, the scheduler includes:

- Guaranteed determinism regardless of system load
- Cryptographic verification of execution order
- Formal guarantees of worst-case execution time
- Fault-tolerant task execution with recovery mechanisms

## Configuration Parameters

The scheduler can be configured with several parameters:

| Parameter | Description | Default |
|-----------|-------------|---------|
| worker_count | Number of worker threads | 1 |
| cpu_affinity | Specific cores to use | All cores |
| queue_capacity | Maximum queue size | 1000 |
| statistics_window | Window for rolling statistics | 1000 |
| performance_logging | Enable detailed logging | false |

## Implementation Details

### Time Representation

The scheduler uses C++17's `std::chrono` library for precise timing:

```cpp
using Clock = std::chrono::high_resolution_clock;
```

### Thread Pool Management

Worker threads are implemented as a managed pool:

1. Creation is batched during scheduler startup
2. Each thread has a main work loop with wait condition
3. Threads can be paused, resumed, or terminated
4. Work stealing is implemented for better load balancing

### Queue Implementation

The task queue is implemented as a priority queue with O(log n) complexity for insertion and extraction, optimized for:

- Fast priority-based extraction
- Thread-safe operations
- Minimal contention between worker threads

## Performance Considerations

- Lock-free algorithms are used where possible
- Cache-friendly data structures minimize memory stalls
- Thread affinity reduces context switching overhead
- Batched operations reduce synchronization overhead

## Debugging Features

- Detailed task execution tracing
- Deadline miss detection and reporting
- Thread starvation monitoring
- Lock contention analysis

## Future Enhancements

- Dynamic thread count based on system load
- Adaptive scheduling based on historical performance
- Hierarchical task dependencies
- Hardware acceleration for queue operations

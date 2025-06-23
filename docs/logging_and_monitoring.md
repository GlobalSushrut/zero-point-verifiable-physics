# Logging and Monitoring

## Overview

The Zero Point Physics Engine includes a comprehensive logging and monitoring system for debugging, performance analysis, and security auditing. This document outlines the available logging features, monitoring capabilities, and best practices for effective use.

## Logging System

### Core Logging Framework

The engine uses a flexible, multi-level logging system:

```cpp
class Logger {
public:
    enum class Level {
        TRACE = 0,
        DEBUG = 1,
        INFO = 2,
        WARN = 3,
        ERROR = 4,
        CRITICAL = 5,
        OFF = 6
    };
    
    static Logger& getInstance();
    
    void setLevel(Level level);
    void setOutput(std::shared_ptr<LogOutput> output);
    void addOutput(std::shared_ptr<LogOutput> output);
    
    void log(Level level, const std::string& message);
    
    // Convenience methods
    void trace(const std::string& message);
    void debug(const std::string& message);
    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);
    void critical(const std::string& message);
    
    // Format and log
    template<typename... Args>
    void logf(Level level, const std::string& format, Args... args);
    
private:
    Level level_ = Level::INFO;
    std::vector<std::shared_ptr<LogOutput>> outputs_;
    std::mutex mutex_;
};

// Global logger access
#define ZP_LOGGER Logger::getInstance()

// Convenience macros
#define ZP_TRACE(msg) ZP_LOGGER.trace(msg)
#define ZP_DEBUG(msg) ZP_LOGGER.debug(msg)
#define ZP_INFO(msg) ZP_LOGGER.info(msg)
#define ZP_WARN(msg) ZP_LOGGER.warn(msg)
#define ZP_ERROR(msg) ZP_LOGGER.error(msg)
#define ZP_CRITICAL(msg) ZP_LOGGER.critical(msg)
```

### Log Outputs

Multiple log outputs are supported:

```cpp
// Base log output interface
class LogOutput {
public:
    virtual ~LogOutput() = default;
    virtual void write(Logger::Level level, 
                     const std::string& message,
                     const std::chrono::system_clock::time_point& time) = 0;
};

// Console output
class ConsoleLogOutput : public LogOutput {
public:
    void write(Logger::Level level, 
              const std::string& message,
              const std::chrono::system_clock::time_point& time) override;
};

// File output
class FileLogOutput : public LogOutput {
public:
    FileLogOutput(const std::string& filename, bool append = true);
    ~FileLogOutput();
    
    void write(Logger::Level level, 
              const std::string& message,
              const std::chrono::system_clock::time_point& time) override;
    
private:
    std::ofstream file_stream_;
};

// Syslog output (Linux/Unix)
class SyslogOutput : public LogOutput {
public:
    SyslogOutput(const std::string& identifier = "ZeroPointEngine");
    ~SyslogOutput();
    
    void write(Logger::Level level, 
              const std::string& message,
              const std::chrono::system_clock::time_point& time) override;
};

// Network log server output
class NetworkLogOutput : public LogOutput {
public:
    NetworkLogOutput(const std::string& host, int port);
    
    void write(Logger::Level level, 
              const std::string& message,
              const std::chrono::system_clock::time_point& time) override;
    
private:
    std::string host_;
    int port_;
    // Network connection handling
};
```

### Usage Examples

```cpp
// Initialize logging system
auto& logger = Logger::getInstance();
logger.setLevel(Logger::Level::DEBUG);

// Add console output
logger.addOutput(std::make_shared<ConsoleLogOutput>());

// Add file output
auto file_output = std::make_shared<FileLogOutput>("physics_engine.log");
logger.addOutput(file_output);

// Log messages
ZP_INFO("Physics engine initialized");
ZP_DEBUG("Created " + std::to_string(node_count) + " nodes");

// Log structured data
logger.logf(Logger::Level::INFO, "Node %d collision with node %d at position (%f, %f, %f)", 
           node1->id, node2->id, contact_point.x, contact_point.y, contact_point.z);

// Log with context
PhysicsContext context;
context["timestamp"] = getCurrentTimeString();
context["solver"] = solver->getName();
context["frame"] = std::to_string(frame_number);
ZP_LOGGER.logWithContext(Logger::Level::INFO, "Simulation step complete", context);
```

## Monitoring System

### Performance Monitors

The engine includes specialized monitors for performance analysis:

```cpp
class PerformanceMonitor {
public:
    PerformanceMonitor();
    
    void begin(const std::string& section);
    void end(const std::string& section);
    
    void beginFrame();
    void endFrame();
    
    void markEvent(const std::string& event_name);
    
    // Get statistics
    double getAverageDuration(const std::string& section) const;
    double getMinDuration(const std::string& section) const;
    double getMaxDuration(const std::string& section) const;
    
    // Generate reports
    void generateReport(const std::string& filename);
    std::string getReportString() const;
    
    // Reset statistics
    void reset();
    
private:
    struct SectionStats {
        double total_time = 0.0;
        double min_time = std::numeric_limits<double>::max();
        double max_time = 0.0;
        size_t call_count = 0;
        std::chrono::high_resolution_clock::time_point start_time;
    };
    
    std::map<std::string, SectionStats> sections_;
    std::chrono::high_resolution_clock::time_point frame_start_time_;
    std::vector<double> frame_times_;
    std::mutex mutex_;
};

// Global monitor
#define ZP_PERF_MONITOR PerformanceMonitor::getInstance()

// Convenience macros
#define ZP_PROFILE_SCOPE(name) ScopedProfiler profiler(ZP_PERF_MONITOR, name)
#define ZP_PROFILE_FUNCTION() ScopedProfiler profiler(ZP_PERF_MONITOR, __FUNCTION__)
```

### Resource Monitors

Monitor system resource usage:

```cpp
class ResourceMonitor {
public:
    ResourceMonitor(int sampling_period_ms = 1000);
    ~ResourceMonitor();
    
    void start();
    void stop();
    
    // Get resource usage
    double getCpuUsage() const;
    size_t getMemoryUsage() const;
    size_t getPeakMemoryUsage() const;
    
    // Get thread statistics
    struct ThreadStats {
        std::string name;
        int id;
        double cpu_usage;
        size_t stack_size;
    };
    
    std::vector<ThreadStats> getThreadStats() const;
    
    // Memory allocation tracking
    size_t getTotalAllocations() const;
    size_t getCurrentLiveAllocations() const;
    
    // Generate reports
    void generateResourceReport(const std::string& filename);
    
private:
    int sampling_period_ms_;
    std::atomic<bool> running_;
    std::thread monitor_thread_;
    
    // Sampling data
    std::mutex data_mutex_;
    std::vector<double> cpu_samples_;
    std::vector<size_t> memory_samples_;
    
    // Worker function
    void monitorWorker_();
};
```

### Example Usage

```cpp
// Start performance monitoring
auto& perf_monitor = PerformanceMonitor::getInstance();

// Monitor specific section
perf_monitor.begin("physics_step");
solver->step(dt);
perf_monitor.end("physics_step");

// Use scoped profiling (automatically calls begin/end)
{
    ZP_PROFILE_SCOPE("collision_detection");
    solver->detectCollisions();
}

// Function-level profiling
void PhysicsSolver::integratePositions(float dt) {
    ZP_PROFILE_FUNCTION();
    // Integration code
}

// Begin/end frames for higher-level timing
perf_monitor.beginFrame();
// Run simulation frame
perf_monitor.endFrame();

// Generate performance report
perf_monitor.generateReport("performance_report.json");

// Start resource monitoring
ResourceMonitor resource_monitor(500);  // 500ms sampling period
resource_monitor.start();

// Run simulation
// ...

// Get resource usage
double cpu_usage = resource_monitor.getCpuUsage();
size_t memory_usage = resource_monitor.getMemoryUsage();

ZP_INFO("CPU usage: " + std::to_string(cpu_usage) + 
       "%, Memory: " + std::to_string(memory_usage / (1024*1024)) + " MB");

// Stop monitoring
resource_monitor.stop();

// Generate resource report
resource_monitor.generateResourceReport("resource_usage.json");
```

## Visualization Tools

### Performance Visualization

```cpp
// Generate visualization of performance data
void generatePerformanceVisualization(const std::string& data_file,
                                    const std::string& output_file) {
    PerformanceVisualizer visualizer;
    visualizer.loadDataFromFile(data_file);
    
    // Create timeline view
    visualizer.createTimeline();
    
    // Create flame graph
    visualizer.createFlameGraph();
    
    // Create performance heatmap
    visualizer.createHeatmap();
    
    // Save to HTML file with interactive charts
    visualizer.saveToFile(output_file);
}
```

### Real-Time Monitoring

```cpp
// Real-time monitoring server
class MonitoringServer {
public:
    MonitoringServer(int port = 8080);
    ~MonitoringServer();
    
    void start();
    void stop();
    
    void setDataUpdateInterval(int milliseconds);
    
    // Register data sources
    void registerPerformanceMonitor(std::shared_ptr<PerformanceMonitor> monitor);
    void registerResourceMonitor(std::shared_ptr<ResourceMonitor> monitor);
    void registerPhysicsSolver(std::shared_ptr<PhysicsSolver> solver);
    
private:
    int port_;
    std::atomic<bool> running_;
    std::thread server_thread_;
    
    // Data sources
    std::shared_ptr<PerformanceMonitor> perf_monitor_;
    std::shared_ptr<ResourceMonitor> resource_monitor_;
    std::shared_ptr<PhysicsSolver> solver_;
    
    // Server implementation
    void serverWorker_();
    void handleWebSocketConnection_(WebSocket* ws);
    void sendDataUpdate_();
};
```

## Military-Grade Auditing

The engine provides secure, tamper-evident logging for military applications:

```cpp
// Secure audit log for military applications
class SecureAuditLog : public LogOutput {
public:
    SecureAuditLog(const std::string& filename, 
                  const std::string& crypto_key);
    ~SecureAuditLog();
    
    void write(Logger::Level level, 
              const std::string& message,
              const std::chrono::system_clock::time_point& time) override;
    
    // Tamper detection
    bool verifyIntegrity();
    
    // Export signed log
    bool exportSignedLog(const std::string& output_file, 
                        const std::string& signing_key);
                        
private:
    std::string filename_;
    std::string crypto_key_;
    std::ofstream file_stream_;
    
    // Cryptographic hash chain for tamper evidence
    std::string last_hash_;
    
    std::string computeEntryHash(Logger::Level level,
                               const std::string& message,
                               const std::chrono::system_clock::time_point& time,
                               const std::string& previous_hash);
};
```

### Usage Example

```cpp
// Create secure audit log
auto secure_log = std::make_shared<SecureAuditLog>(
    "military_audit.log",
    getSecretKey());
    
Logger::getInstance().addOutput(secure_log);

// Log sensitive operations
ZP_LOGGER.logf(Logger::Level::INFO, 
              "Verification signature generated: %s", signature.c_str());
ZP_LOGGER.logf(Logger::Level::INFO,
              "User %s performed operation: %s", 
              user_id.c_str(), operation.c_str());

// Verify log integrity
if (!secure_log->verifyIntegrity()) {
    ZP_CRITICAL("Audit log integrity check failed - possible tampering detected");
}

// Export signed log for external verification
secure_log->exportSignedLog(
    "signed_audit_log.dat",
    getSigningKey());
```

## Configuration

Configure logging and monitoring through a central configuration:

```cpp
// Configure logging system
LoggingConfig log_config;
log_config.default_level = Logger::Level::INFO;
log_config.enable_console = true;
log_config.enable_file_logging = true;
log_config.log_file = "physics_engine.log";
log_config.enable_syslog = false;
log_config.rotation_size_mb = 10;
log_config.max_log_files = 5;

// Configure monitoring
MonitoringConfig monitor_config;
monitor_config.enable_performance_monitoring = true;
monitor_config.sampling_rate_ms = 100;
monitor_config.enable_resource_monitoring = true;
monitor_config.resource_sampling_rate_ms = 1000;
monitor_config.enable_monitoring_server = false;
monitor_config.server_port = 8080;

// Apply configuration
ConfigureLoggingAndMonitoring(log_config, monitor_config);
```

## Best Practices

### Performance Logging Best Practices

1. **Focus on Critical Sections**: Instrument the most computationally intensive parts of your code
2. **Avoid Overhead**: Minimize logging in tight loops; use sampling for high-frequency operations
3. **Categorize Performance Data**: Group related operations for easier analysis
4. **Set Appropriate Granularity**: Balance between detailed timings and system overhead

### Debugging Best Practices

1. **Use Log Levels Appropriately**: Use TRACE for detailed debugging, INFO for normal operation milestones
2. **Include Context**: Log relevant state information with error messages
3. **Structured Logging**: Use consistent formats for automated log analysis
4. **Correlation IDs**: Include unique identifiers to trace related operations across components

### Security Best Practices

1. **Sensitive Data Handling**: Never log passwords, encryption keys, or personal data
2. **Integrity Protection**: Use secure audit logs for critical operations
3. **Access Control**: Restrict access to log files, especially security audit logs
4. **Log Rotation**: Implement proper log rotation and archival

### Military-Grade Logging Considerations

1. **Data Classification**: Mark logs with appropriate classification levels
2. **Chain of Custody**: Maintain cryptographically-verified chain of custody for audit logs
3. **Side-Channel Protection**: Ensure logging operations don't leak timing information
4. **Compliance**: Ensure logging meets relevant military standards (e.g., Common Criteria)

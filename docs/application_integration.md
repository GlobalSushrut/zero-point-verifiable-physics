# Application Integration Guide

## Overview

This guide explains how to integrate the Zero Point Physics Engine into your own applications, whether for military simulations, game development, scientific visualization, or engineering analysis.

## Basic Integration Steps

### Step 1: Include the Engine

```cpp
#include "zero_point/physics_engine.hpp"
#include "zero_point/core/physics_solver.hpp"
#include "zero_point/core/binary_space_tree.hpp"
#include "zero_point/core/rt_scheduler.hpp"
```

### Step 2: Initialize the Engine

```cpp
// Create physics solver (standard or military-grade)
auto solver = std::make_shared<PhysicsSolver>();
// For military applications:
// auto solver = std::make_shared<PhysicsSolverMilitary>(VerificationLevel::MILITARY);

// Configure the solver
PhysicsSolverConfig config;
config.integration_method = IntegrationMethod::VELOCITY_VERLET;
config.gravity = Vec3(0, -9.81f, 0);
solver->setConfig(config);

// Initialize the solver
if (!solver->initialize()) {
    // Handle initialization failure
    return false;
}
```

### Step 3: Create Physics Objects

```cpp
// Create a dynamic sphere
auto sphere = solver->createNode(
    Vec3(0, 10, 0),  // position
    1.0f,            // mass
    0.5f             // radius
);

// Create a static ground plane
auto ground = solver->createNode(
    Vec3(0, 0, 0),   // position
    0.0f,            // mass of 0 indicates static
    10.0f            // radius (large for ground plane)
);
ground->is_dynamic = false;
```

### Step 4: Main Simulation Loop

```cpp
const float fixed_dt = 1.0f / 60.0f;  // 60Hz simulation
float accumulator = 0.0f;

while (application_running) {
    float frame_time = calculateFrameTime();
    accumulator += frame_time;
    
    // Physics update with fixed time step
    while (accumulator >= fixed_dt) {
        solver->step(fixed_dt);
        accumulator -= fixed_dt;
    }
    
    // Render the scene
    renderScene(solver->getAllNodes());
    
    // Process input, etc.
    processInput();
}
```

## Integration Patterns

### Model-View-Controller Pattern

```cpp
class PhysicsController {
private:
    std::shared_ptr<PhysicsSolver> solver_;
    std::shared_ptr<PhysicsView> view_;
    
public:
    PhysicsController() {
        solver_ = std::make_shared<PhysicsSolver>();
        solver_->initialize();
        
        view_ = std::make_shared<PhysicsView>();
    }
    
    void update(float dt) {
        solver_->step(dt);
        view_->render(solver_->getAllNodes());
    }
    
    void handleInput(const UserInput& input) {
        // Process input and modify physics state
    }
};
```

### Component-Based Architecture

```cpp
class PhysicsComponent {
private:
    std::shared_ptr<Node3D> physics_node_;
    
public:
    void initialize(std::shared_ptr<PhysicsSolver> solver, 
                   const Vec3& position, float mass, float radius) {
        physics_node_ = solver->createNode(position, mass, radius);
    }
    
    void synchronizeWithTransform(Transform& transform) {
        // Update game object transform from physics
        transform.position = physics_node_->position;
        transform.rotation = physics_node_->orientation;
    }
    
    void applyForce(const Vec3& force) {
        physics_node_->applyForce(force);
    }
};
```

## Multithreading Considerations

```cpp
// Initialize the scheduler with worker threads
RTSchedulerConfig scheduler_config;
scheduler_config.worker_count = 3;  // Use 3 worker threads
scheduler_config.set_affinity = true;  // Set CPU affinity

auto scheduler = std::make_shared<RTScheduler>(scheduler_config);
solver->setScheduler(scheduler);

// Handle thread synchronization
std::mutex physics_mutex;

void updatePhysics() {
    std::lock_guard<std::mutex> lock(physics_mutex);
    solver->step(dt);
}

void readPhysicsState() {
    std::lock_guard<std::mutex> lock(physics_mutex);
    // Read state safely
}
```

## Integrating with Graphics Engines

### OpenGL Integration

```cpp
// Synchronize physics and rendering
void updateRenderMeshes(std::vector<RenderMesh>& meshes, 
                       const std::vector<Node3D*>& nodes) {
    for (size_t i = 0; i < nodes.size(); i++) {
        // Update transform matrices
        glm::mat4 translation = glm::translate(
            glm::mat4(1.0f), 
            glm::vec3(nodes[i]->position.x, nodes[i]->position.y, nodes[i]->position.z)
        );
        
        // Convert quaternion to matrix
        glm::mat4 rotation = quaternionToMatrix(nodes[i]->orientation);
        
        meshes[i].model_matrix = translation * rotation;
    }
}
```

### Vulkan Integration

```cpp
// Update uniform buffers with physics data
void updateUniformBuffers(const std::vector<Node3D*>& nodes) {
    for (size_t i = 0; i < nodes.size(); i++) {
        UniformBufferObject ubo{};
        
        // Convert physics transform to render transform
        ubo.model = convertToVulkanMatrix(nodes[i]->position, nodes[i]->orientation);
        
        // Copy to GPU
        void* data;
        vkMapMemory(device, uniformBuffersMemory[i], 0, sizeof(ubo), 0, &data);
        memcpy(data, &ubo, sizeof(ubo));
        vkUnmapMemory(device, uniformBuffersMemory[i]);
    }
}
```

## Military Application Integration

```cpp
// Initialize military-grade solver with verification
auto solver = std::make_shared<PhysicsSolverMilitary>(VerificationLevel::MILITARY);

// Configure cryptographic verification
solver->enableCryptographicIntegrity(true);
solver->setVerificationKeyPath("/path/to/verification/key.pem");

// Monitor verification status
void monitorVerificationStatus() {
    auto metrics = solver->getPerformanceMetrics();
    float verification_rate = metrics["verification_success_rate"];
    
    if (verification_rate < 0.98f) {
        // Verification falling below acceptable threshold
        logSecurityWarning("Physics verification integrity compromised!");
    }
}
```

## Error Handling

```cpp
try {
    solver->step(dt);
} catch (const PhysicsException& e) {
    switch (e.getErrorCode()) {
        case ErrorCode::NUMERICAL_INSTABILITY:
            // Handle instability
            resetSimulation();
            break;
            
        case ErrorCode::VERIFICATION_FAILURE:
            // Handle verification failure
            logSecurityBreach();
            break;
            
        default:
            // Handle other errors
            logError(e.what());
            break;
    }
}
```

## Serialization and State Management

```cpp
// Save simulation state
void saveSimulationState(const std::string& filename) {
    SimulationState state = solver->serializeState();
    
    std::ofstream file(filename, std::ios::binary);
    file.write(reinterpret_cast<const char*>(&state.version), sizeof(uint32_t));
    file.write(reinterpret_cast<const char*>(&state.node_count), sizeof(uint32_t));
    
    for (auto node : state.nodes) {
        file.write(reinterpret_cast<const char*>(&node), sizeof(SerializedNode));
    }
    
    // Save verification data
    if (state.has_verification) {
        file.write(state.integrity_signature.data(), state.integrity_signature.size());
    }
}

// Load simulation state
void loadSimulationState(const std::string& filename) {
    SimulationState state;
    std::ifstream file(filename, std::ios::binary);
    
    file.read(reinterpret_cast<char*>(&state.version), sizeof(uint32_t));
    file.read(reinterpret_cast<char*>(&state.node_count), sizeof(uint32_t));
    
    state.nodes.resize(state.node_count);
    for (auto& node : state.nodes) {
        file.read(reinterpret_cast<char*>(&node), sizeof(SerializedNode));
    }
    
    solver->deserializeState(state);
}
```

## Cross-Platform Considerations

```cpp
// Platform detection
#if defined(_WIN32)
    // Windows-specific code
#elif defined(__linux__)
    // Linux-specific code
#elif defined(__APPLE__) && defined(__MACH__)
    // macOS-specific code
#endif

// Platform-specific optimizations
#ifdef USE_SSE
    // SSE vector math implementation
#else
    // Standard vector math implementation
#endif
```

## Performance Monitoring

```cpp
void monitorPerformance() {
    auto metrics = solver->getPerformanceMetrics();
    
    std::cout << "Physics Performance:" << std::endl;
    std::cout << "Step time: " << metrics["step_time_ms"] << " ms" << std::endl;
    std::cout << "Collision checks: " << metrics["collision_checks"] << std::endl;
    std::cout << "Active nodes: " << metrics["active_nodes"] << std::endl;
    std::cout << "Verification time: " << metrics["verification_time_ms"] << " ms" << std::endl;
    std::cout << "CPU utilization: " << metrics["utilization"] << "%" << std::endl;
}
```

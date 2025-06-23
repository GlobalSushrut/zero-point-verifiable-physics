# Visualization Tools

## Overview

The Zero Point Physics Engine includes a suite of visualization tools for rendering simulation results, debugging physical interactions, and analyzing verification data. These tools help users understand complex physics phenomena and validate simulation correctness.

## Rendering Components

### Core Renderer

The engine provides an OpenGL-based renderer for visualizing simulations:

```cpp
class PhysicsRenderer {
public:
    PhysicsRenderer();
    ~PhysicsRenderer();
    
    // Initialize with window dimensions
    bool initialize(int width, int height);
    
    // Update camera position/orientation
    void setCamera(const Vec3& position, const Vec3& target, const Vec3& up);
    
    // Render physics objects
    void renderNodes(const std::vector<Node3D*>& nodes);
    
    // Render constraints between objects
    void renderConstraints(const std::vector<Constraint*>& constraints);
    
    // Render collision information
    void renderCollisions(const std::vector<CollisionInfo>& collisions);
    
    // Render debug information
    void renderDebugInfo(const std::unordered_map<std::string, double>& metrics);
    
private:
    // OpenGL shader programs
    GLuint node_shader_program_;
    GLuint constraint_shader_program_;
    GLuint debug_shader_program_;
    
    // Camera information
    Mat4 view_matrix_;
    Mat4 projection_matrix_;
    
    // Rendering resources
    std::unordered_map<uint32_t, GLuint> material_textures_;
    std::vector<GLuint> vaos_;  // Vertex Array Objects
    std::vector<GLuint> vbos_;  // Vertex Buffer Objects
};
```

### Usage Example

```cpp
// Create and initialize renderer
PhysicsRenderer renderer;
renderer.initialize(1280, 720);

// Set camera position
renderer.setCamera(
    Vec3(10, 5, 10),  // Camera position
    Vec3(0, 0, 0),    // Look target
    Vec3(0, 1, 0)     // Up vector
);

// In render loop
void renderScene() {
    // Clear frame buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Render physics objects
    renderer.renderNodes(solver->getAllNodes());
    
    // Render constraints
    renderer.renderConstraints(solver->getAllConstraints());
    
    // Render collisions
    renderer.renderCollisions(solver->getCollisions());
    
    // Render performance metrics
    renderer.renderDebugInfo(solver->getPerformanceMetrics());
    
    // Swap buffers
    SDL_GL_SwapWindow(window);
}
```

## Visualization Modes

### Standard Visualization

The basic visualization mode shows:
- Nodes as spheres with appropriate size and material
- Velocity vectors as directional arrows
- Constraints as connecting lines
- Collision points as highlighted contacts

### Debug Visualization

The debug mode includes additional information:
- Bounding boxes for spatial partitioning
- Force vectors showing applied forces
- Contact normals at collision points
- Coordinate axes at world origin

### Military-Grade Visualization

For military applications, additional visualization features:
- Verification confidence as color gradient
- Cryptographic integrity status indicators
- Error propagation visualization
- Stress analysis heat maps

## Heat Maps and Analysis Tools

```cpp
// Generate heat map based on node velocity
void generateVelocityHeatMap() {
    std::vector<float> velocity_magnitudes;
    
    for (const auto* node : solver->getAllNodes()) {
        velocity_magnitudes.push_back(node->velocity.length());
    }
    
    // Find min/max for normalization
    float min_vel = *std::min_element(velocity_magnitudes.begin(), 
                                    velocity_magnitudes.end());
    float max_vel = *std::max_element(velocity_magnitudes.begin(), 
                                    velocity_magnitudes.end());
    
    // Generate color mapping
    std::vector<Vec3> color_map;
    for (float vel : velocity_magnitudes) {
        float normalized = (vel - min_vel) / (max_vel - min_vel);
        
        // Blue to red gradient
        Vec3 color(normalized, 0.0f, 1.0f - normalized);
        color_map.push_back(color);
    }
    
    // Render with color map
    renderer.renderNodesWithColorMap(solver->getAllNodes(), color_map);
}
```

## Graph Tools

The engine provides real-time graphing for physics metrics:

```cpp
// Create graph visualizer
GraphVisualizer graph;
graph.initialize(400, 200);  // Width, height
graph.setPosition(10, 10);   // Screen position

// Add data series
graph.addSeries("Performance", Vec3(0, 1, 0));  // Green line
graph.addSeries("Verification", Vec3(1, 0, 0)); // Red line

// Update graph during simulation
void updateGraph() {
    auto metrics = solver->getPerformanceMetrics();
    
    graph.addDataPoint("Performance", metrics["step_time_ms"]);
    graph.addDataPoint("Verification", metrics["verification_time_ms"]);
    
    graph.render();
}
```

## Verification Visualization

Special visualization tools for verification confidence:

```cpp
// Create verification visualizer
VerificationVisualizer verifier_vis;
verifier_vis.initialize();

// Update visualization with verification data
void updateVerificationVis() {
    const auto* verifier = solver->getVerifier();
    auto stats = verifier->getStatistics();
    
    // Set up confidence visualization
    std::vector<float> confidence_values;
    for (const auto& node_confidence : stats.node_confidence) {
        confidence_values.push_back(node_confidence.second);
    }
    
    // Render confidence visualization
    verifier_vis.renderConfidenceMap(solver->getAllNodes(), confidence_values);
    verifier_vis.renderVerificationGraph(stats.historical_confidence);
}
```

## Exporting Visualization Data

The engine supports exporting visuals for analysis and reports:

```cpp
// Export a screenshot
void takeScreenshot(const std::string& filename) {
    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    
    // Allocate memory for pixel data
    std::vector<uint8_t> pixels(width * height * 4);
    
    // Read pixels from framebuffer
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    
    // Flip image vertically (OpenGL has origin at bottom-left)
    for (int y = 0; y < height / 2; y++) {
        for (int x = 0; x < width; x++) {
            for (int c = 0; c < 4; c++) {
                std::swap(
                    pixels[(y * width + x) * 4 + c],
                    pixels[((height - 1 - y) * width + x) * 4 + c]
                );
            }
        }
    }
    
    // Save to file using stb_image_write
    stbi_write_png(filename.c_str(), width, height, 4, pixels.data(), width * 4);
}

// Export metrics to CSV for further analysis
void exportMetricsCSV(const std::string& filename) {
    std::ofstream file(filename);
    
    // Write header
    file << "Step,StepTime,CollisionChecks,ActiveNodes,VerificationSuccess,VerificationTime\n";
    
    // Write data rows
    for (const auto& record : metric_history) {
        file << record.step << ","
             << record.step_time_ms << ","
             << record.collision_checks << ","
             << record.active_nodes << ","
             << record.verification_success_rate << ","
             << record.verification_time_ms << "\n";
    }
}
```

## VR Integration

For immersive visualization, the engine supports VR output:

```cpp
// Initialize VR visualization
VRVisualizer vr;
if (!vr.initialize()) {
    std::cerr << "Failed to initialize VR system" << std::endl;
    return false;
}

// Render to VR headset
void renderVR() {
    // Get VR headset pose
    VRPose headset_pose = vr.getHeadsetPose();
    
    // Convert to camera view matrix
    Mat4 view_matrix = headset_pose.getViewMatrix();
    
    // Render left eye
    vr.beginEyeRender(VREye::Left);
    renderer.setViewMatrix(view_matrix * vr.getEyeTransform(VREye::Left));
    renderScene();
    vr.endEyeRender(VREye::Left);
    
    // Render right eye
    vr.beginEyeRender(VREye::Right);
    renderer.setViewMatrix(view_matrix * vr.getEyeTransform(VREye::Right));
    renderScene();
    vr.endEyeRender(VREye::Right);
    
    // Submit to VR compositor
    vr.submitFrame();
}
```

## Performance Considerations

- Rendering can be offloaded to a separate thread
- Level of detail can be adjusted based on node distance
- Instanced rendering for large numbers of similar objects
- Culling techniques for large scenes
- GPU acceleration for visualization computations

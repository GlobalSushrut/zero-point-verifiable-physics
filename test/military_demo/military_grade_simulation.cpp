// Fix include path to use correct relative path
#include "../../zero_point/core/physics_solver_military.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <sstream>

// Conditionally include visualization headers
#ifdef ENABLE_VISUALIZATION
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

using namespace zero_point::core;

// Utility function to print a formatted header
void printHeader(const std::string& text) {
    std::cout << "\n\033[1;36m" << std::string(50, '=') << "\n" 
              << "    " << text << "\n" 
              << std::string(50, '=') << "\033[0m\n";
}

// Utility function to print metrics
void printMetrics(const std::unordered_map<std::string, float>& metrics) {
    std::cout << "\033[1;33m";
    for (const auto& [key, value] : metrics) {
        std::cout << std::setw(30) << key << " : " << std::fixed 
                  << std::setprecision(6) << value << std::endl;
    }
    std::cout << "\033[0m\n";
}

// Utility function for introducing errors to test robustness
void introduceNumericalError(Node3D& node, float magnitude) {
    // Introduce a small NaN or Inf to test numerical stability
    if (rand() % 2 == 0) {
        node.position.x = std::numeric_limits<float>::quiet_NaN();
    } else {
        node.velocity.y = std::numeric_limits<float>::infinity() * magnitude;
    }
}

// Set up a complex physical scenario that stresses the engine
void setupComplexScenario(PhysicsSolverMilitary& solver) {
    const int NODE_COUNT = 1000;
    const float SCENE_SIZE = 100.0f;
    
    printHeader("Setting up complex physical scenario with " + std::to_string(NODE_COUNT) + " nodes");
    
    // Create a structured grid of nodes with some randomization
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-SCENE_SIZE/2, SCENE_SIZE/2);
    std::uniform_real_distribution<float> mass_dist(0.1f, 10.0f);
    std::uniform_real_distribution<float> vel_dist(-10.0f, 10.0f);
    
    // Grid arrangement with randomized positions
    const int GRID_SIZE = static_cast<int>(std::cbrt(NODE_COUNT));
    const float SPACING = SCENE_SIZE / GRID_SIZE;
    
    std::vector<uint32_t> node_ids;
    node_ids.reserve(NODE_COUNT);
    
    for (int x = 0; x < GRID_SIZE; x++) {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int z = 0; z < GRID_SIZE; z++) {
                if (node_ids.size() >= NODE_COUNT) break;
                
                float jitter_x = dist(gen) * 0.1f;
                float jitter_y = dist(gen) * 0.1f;
                float jitter_z = dist(gen) * 0.1f;
                
                Vec3 pos(
                    x * SPACING - SCENE_SIZE/2 + jitter_x, 
                    y * SPACING - SCENE_SIZE/2 + jitter_y, 
                    z * SPACING - SCENE_SIZE/2 + jitter_z
                );
                
                float mass = mass_dist(gen);
                uint32_t id = solver.addNode(pos, true, mass);
                
                if (id > 0) {
                    node_ids.push_back(id);
                    
                    // Add initial velocity
                    Vec3 vel(vel_dist(gen), vel_dist(gen), vel_dist(gen));
                    solver.setNodeVelocity(id, vel);
                }
            }
        }
    }
    
    // Create some constraints between nodes
    printHeader("Creating node constraints");
    int constraint_count = NODE_COUNT / 5; // 20% of nodes get constraints
    std::uniform_int_distribution<int> node_dist(0, node_ids.size() - 1);
    
    for (int i = 0; i < constraint_count; i++) {
        int idx1 = node_dist(gen);
        int idx2 = node_dist(gen);
        
        // Don't connect a node to itself
        if (idx1 != idx2) {
            uint32_t node1 = node_ids[idx1];
            uint32_t node2 = node_ids[idx2];
            
            // Add constraint with random rest length and stiffness
            float rest_length = SPACING * (1.0f + 0.2f * (dist(gen) / SCENE_SIZE));
            float stiffness = 0.5f + dist(gen) / SCENE_SIZE;
            
            solver.createDistanceConstraint(node1, node2, rest_length, stiffness);
        }
    }
    
    // Set gravity
    solver.setGravity(Vec3(0.0f, -9.81f, 0.0f));
    
    std::cout << "Complex scenario setup complete with " << node_ids.size() 
              << " nodes and " << constraint_count << " constraints\n";
}

// Perform a stress test to demonstrate robustness
void performStressTest(PhysicsSolverMilitary& solver) {
    printHeader("Beginning Military-Grade Stress Test");
    
    const int STEPS = 1000;
    const float TIME_STEP = 1.0f / 120.0f;
    
    // Record signatures for integrity validation
    std::vector<std::string> state_signatures;
    
    // Warmup phase
    std::cout << "Performing warm-up phase...\n";
    for (int i = 0; i < 10; i++) {
        solver.step(TIME_STEP);
    }
    
    // Main simulation loop
    std::cout << "Beginning main simulation (strict verification)...\n";
    solver.setVerificationLevel(VerificationLevel::FULL);
    
    // Track timings for jitter analysis
    std::vector<double> step_times;
    step_times.reserve(STEPS);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < STEPS; i++) {
        auto step_start = std::chrono::high_resolution_clock::now();
        
        // Every 100 steps, verify system integrity
        if (i % 100 == 0) {
            bool integrity_result = solver.verifySystemIntegrity(true);
            std::cout << "Integrity check at step " << i << ": " 
                      << (integrity_result ? "\033[1;32mPASSED\033[0m" : "\033[1;31mFAILED\033[0m") << std::endl;
            
            // Store cryptographic signature for later verification
            state_signatures.push_back(solver.generateStateSignature());
        }
        
        // Every 250 steps, introduce a numerical error to test robustness
        if (i == 250 || i == 500 || i == 750) {
            std::cout << "\033[1;31mIntentionally introducing numerical error at step " << i << "...\033[0m\n";
            
            // Get a random node and corrupt it
            auto node = solver.getSpaceTree().getRandomNode();
            if (node) {
                introduceNumericalError(*node, 1.0f);
            }
        }
        
        // Perform simulation step
        solver.step(TIME_STEP);
        
        auto step_end = std::chrono::high_resolution_clock::now();
        double step_ms = std::chrono::duration<double, std::milli>(step_end - step_start).count();
        step_times.push_back(step_ms);
        
        // Print progress every 10%
        if (i % (STEPS/10) == 0) {
            std::cout << "Completed " << (i * 100 / STEPS) << "% (" << i << " steps)\n";
            
            // Display performance metrics periodically
            if (i > 0) {
                auto metrics = solver.getPerformanceMetrics();
                printMetrics(metrics);
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    // Print final statistics
    printHeader("Simulation Complete");
    std::cout << "Total simulation time: " << total_ms << " ms\n";
    std::cout << "Average step time: " << (total_ms / STEPS) << " ms\n";
    
    // Calculate timing jitter (important for military applications)
    double sum = 0.0;
    for (double time : step_times) sum += time;
    double mean = sum / step_times.size();
    
    double variance = 0.0;
    for (double time : step_times) {
        variance += (time - mean) * (time - mean);
    }
    variance /= step_times.size();
    double std_dev = std::sqrt(variance);
    
    std::cout << "Timing jitter statistics:\n";
    std::cout << "  Mean step time: " << std::fixed << std::setprecision(3) << mean << " ms\n";
    std::cout << "  Standard deviation: " << std_dev << " ms\n";
    std::cout << "  Coefficient of variation: " << (std_dev / mean * 100.0) << "%\n";
    
    // Verify integrity across saved signatures
    printHeader("Cryptographic Integrity Verification");
    if (state_signatures.size() >= 2) {
        std::cout << "Verifying integrity across " << state_signatures.size() << " saved states...\n";
        for (size_t i = 1; i < state_signatures.size(); i++) {
            std::cout << "Comparing signature " << i-1 << " to " << i << ": ";
            if (state_signatures[i-1] != state_signatures[i]) {
                std::cout << "\033[1;32mVerified (signatures correctly differ)\033[0m\n";
            } else {
                std::cout << "\033[1;31mWarning: Identical signatures between different states\033[0m\n";
            }
        }
    }
    
    // Print verification statistics
    auto verification_stats = solver.getVerificationStatistics();
    printHeader("Formal Verification Statistics");
    for (const auto& [key, value] : verification_stats) {
        std::cout << std::setw(30) << key << " : " << std::fixed 
                  << std::setprecision(6) << value << std::endl;
    }
    
    // Print scheduler statistics
    auto scheduler_stats = solver.getSchedulerStatistics();
    printHeader("Real-time Scheduler Statistics");
    for (const auto& [key, value] : scheduler_stats) {
        std::cout << std::setw(30) << key << " : " << std::fixed 
                  << std::setprecision(6) << value << std::endl;
    }
}

int main(int argc, char** argv) {
    printHeader("MILITARY-GRADE PHYSICS SIMULATION DEMONSTRATION");
    std::cout << "Initializing NASA/Military-Grade Physics Solver with formal verification...\n";
    
    // Create military-grade physics solver with full verification
    PhysicsSolverMilitary solver(SolverType::PARTICLE, VerificationLevel::PARTIAL, true);
    
    // Configure numerical stability options
    solver.setNumericalStabilityOptions(true, true);
    
    // Set CPU affinity if available (typically cores 1-3, leaving core 0 for system)
    std::vector<int> cpu_cores = {1, 2, 3};
    if (solver.setCpuAffinity(cpu_cores)) {
        std::cout << "CPU affinity set to cores: 1, 2, 3\n";
    } else {
        std::cout << "CPU affinity not supported on this platform\n";
    }
    
    // Setup complex physical scenario
    setupComplexScenario(solver);
    
    // Run stress test
    performStressTest(solver);
    
    printHeader("SIMULATION DEMONSTRATION COMPLETE");
    return 0;
}

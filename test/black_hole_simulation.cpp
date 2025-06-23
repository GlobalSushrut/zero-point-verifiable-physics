#include "../include/physics_engine.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include <omp.h>
#include <cmath>

using namespace physics;

// Save trajectory data to file for visualization
void saveTrajectories(const std::string& filename, const std::vector<std::vector<VectorX>>& trajectories) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    
    file << "# Black Hole Simulation Trajectory Data" << std::endl;
    file << "# Format: trajectory_id x y z" << std::endl;
    
    for (size_t i = 0; i < trajectories.size(); ++i) {
        for (const auto& point : trajectories[i]) {
            file << i << " ";
            for (int d = 0; d < point.size(); ++d) {
                file << point(d) << " ";
            }
            file << std::endl;
        }
        file << std::endl; // Empty line between trajectories for gnuplot
    }
    
    file.close();
    std::cout << "Trajectory data saved to " << filename << std::endl;
}

// Configuration for black hole simulation
struct BlackHoleConfig {
    int dimensions = 3;                 // Spatial dimensions
    Scalar blackHoleMass = 1.0;         // Mass of black hole (normalized)
    Scalar schwarzschildRadius = 1.0;   // Event horizon radius
    int particleCount = 1000;           // Number of test particles
    Scalar simulationRadius = 20.0;     // Simulation boundary radius
    Scalar timeStep = 0.01;             // Time step for simulation
    int steps = 1000;                   // Number of simulation steps
    int threads = 13;                   // Number of threads to use
    int optimizationLevel = 5;          // Entropic gateway optimization level (1-5)
    bool saveData = true;               // Whether to save trajectory data
    std::string outputFile = "black_hole_trajectories.dat"; // Output file for trajectories
};

// Setup black hole in physics engine
void setupBlackHole(PhysicsEngine& engine, const BlackHoleConfig& config) {
    std::cout << "Setting up black hole with Schwarzschild radius " 
              << config.schwarzschildRadius << std::endl;
    
    // Create the black hole singularity at the origin
    VectorX center = VectorX::Zero(config.dimensions);
    engine.addFieldSource(center, -config.blackHoleMass);
    
    // Set curved spacetime parameters
    // Access the non-Euclidean manifold and add a singularity
    auto nonEuclidean = std::make_shared<ManifoldMetric>(config.dimensions, 
                                                       0.1 * config.blackHoleMass);
    nonEuclidean->addSingularity(center, config.blackHoleMass);
    
    // Add test particles in orbital configurations
    std::cout << "Adding " << config.particleCount << " test particles..." << std::endl;
    
    std::vector<VectorX> positions;
    std::vector<VectorX> velocities;
    
    std::srand(12345); // For reproducibility
    
    for (int i = 0; i < config.particleCount; ++i) {
        // Create particles in random positions but outside the event horizon
        VectorX position = VectorX::Zero(config.dimensions);
        
        // Random spherical coordinates
        Scalar r = config.schwarzschildRadius * 1.5 + 
                 (config.simulationRadius - config.schwarzschildRadius * 1.5) * 
                 static_cast<Scalar>(std::rand()) / RAND_MAX;
                 
        Scalar theta = M_PI * static_cast<Scalar>(std::rand()) / RAND_MAX;
        Scalar phi = 2.0 * M_PI * static_cast<Scalar>(std::rand()) / RAND_MAX;
        
        // Convert to Cartesian
        position(0) = r * std::sin(theta) * std::cos(phi);
        position(1) = r * std::sin(theta) * std::sin(phi);
        if (config.dimensions > 2) {
            position(2) = r * std::cos(theta);
        }
        
        // Calculate orbital velocity (to demonstrate different orbits)
        // Some particles get orbital velocity, others get radial velocity
        VectorX velocity = VectorX::Zero(config.dimensions);
        
        // Direction perpendicular to radius
        VectorX direction = VectorX::Zero(config.dimensions);
        
        if (i % 3 == 0) { // Orbital motion
            // Cross product equivalent in the first two dimensions
            direction(0) = -position(1);
            direction(1) = position(0);
            if (config.dimensions > 2) {
                direction(2) = 0.0;
            }
            
            // Normalize
            direction /= direction.norm();
            
            // Keplerian orbital velocity scaled by mass and distance
            Scalar speed = std::sqrt(config.blackHoleMass / r);
            velocity = direction * speed;
            
        } else if (i % 3 == 1) { // Radial infall
            // Direction toward the black hole
            direction = -position / position.norm();
            
            // Random speed
            Scalar speed = 0.05 * std::sqrt(config.blackHoleMass / r) * 
                          static_cast<Scalar>(std::rand()) / RAND_MAX;
            velocity = direction * speed;
            
        } else { // Tangential + radial (spiral)
            // Tangential component
            VectorX tangent = VectorX::Zero(config.dimensions);
            tangent(0) = -position(1);
            tangent(1) = position(0);
            if (config.dimensions > 2) {
                tangent(2) = 0.0;
            }
            tangent /= tangent.norm();
            
            // Radial component (outward)
            VectorX radial = position / position.norm();
            
            // Combine with different weights
            Scalar tangentWeight = 0.7 + 0.3 * static_cast<Scalar>(std::rand()) / RAND_MAX;
            Scalar radialWeight = 0.1 * static_cast<Scalar>(std::rand()) / RAND_MAX;
            
            Scalar speed = std::sqrt(config.blackHoleMass / r);
            velocity = tangent * (tangentWeight * speed) + radial * (radialWeight * speed);
        }
        
        positions.push_back(position);
        velocities.push_back(velocity);
    }
    
    engine.addParticles(positions, &velocities);
}

// Run black hole simulation
std::vector<std::vector<VectorX>> runBlackHoleSimulation(const BlackHoleConfig& config) {
    std::cout << "===================================================" << std::endl;
    std::cout << "  BLACK HOLE SIMULATION WITH ENTROPIC OPTIMIZATION  " << std::endl;
    std::cout << "===================================================" << std::endl;
    
    // Create physics engine configuration
    PhysicsConfig engineConfig;
    engineConfig.dimensions = config.dimensions;
    engineConfig.dt = config.timeStep;
    engineConfig.adaptiveTimeStepping = true;
    engineConfig.numThreads = config.threads;
    engineConfig.shivaK = 1.0;  // Gravity equivalent
    engineConfig.shivaScale = 0.1;
    
    // Set number of OpenMP threads
    omp_set_num_threads(config.threads);
    
    // Create the physics engine
    PhysicsEngine engine(engineConfig);
    
    // Set up black hole and particles
    setupBlackHole(engine, config);
    
    // Apply entropic gateway optimization
    std::cout << "Applying entropic gateway optimization level " << config.optimizationLevel << std::endl;
    engine.applyEntropicOptimization(config.optimizationLevel);
    
    // Store particle trajectories
    std::vector<std::vector<VectorX>> trajectories(config.particleCount);
    for (int i = 0; i < config.particleCount; ++i) {
        trajectories[i].reserve(config.steps);
    }
    
    // Run simulation
    std::cout << "Starting simulation for " << config.steps << " steps..." << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    
    for (int step = 0; step < config.steps; ++step) {
        // Run one simulation step
        SimulationState state = engine.step();
        
        // Store particle positions for trajectory visualization
        for (size_t i = 0; i < std::min(config.particleCount, (int)state.systemState.positions.size()); ++i) {
            trajectories[i].push_back(state.systemState.positions[i]);
        }
        
        // Print progress (every 10%)
        if (step % (config.steps / 10) == 0) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<double>(currentTime - startTime).count();
            std::cout << "Step " << step << "/" << config.steps 
                      << " (" << (step * 100 / config.steps) << "%) - "
                      << "Time elapsed: " << elapsed << " seconds" << std::endl;
            
            // Predict total time
            if (step > 0) {
                double estimatedTotal = elapsed * config.steps / step;
                std::cout << "Estimated total time: " << estimatedTotal << " seconds" << std::endl;
            }
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(endTime - startTime).count();
    
    std::cout << "\nSimulation completed in " << duration << " seconds" << std::endl;
    std::cout << "Average time per step: " << (duration / config.steps * 1000) << " ms" << std::endl;
    
    // Get optimization statistics
    auto stats = engine.getOptimizationStats();
    std::cout << "\nOptimization statistics:" << std::endl;
    for (const auto& [key, value] : stats) {
        std::cout << "  " << key << ": " << value << std::endl;
    }
    
    std::cout << "\nTheoretical speedup: " << stats["total_speedup"] << "×" << std::endl;
    std::cout << "Using " << config.threads << " processor cores" << std::endl;
    
    return trajectories;
}

// Main function
int main(int argc, char** argv) {
    // Configure the black hole simulation
    BlackHoleConfig config;
    config.dimensions = 3;
    config.blackHoleMass = 1.0;
    config.schwarzschildRadius = 1.0;
    config.particleCount = 500;  // Reduced for faster demo
    config.simulationRadius = 20.0;
    config.timeStep = 0.01;
    config.steps = 500;  // Reduced for faster demo
    config.threads = 13;  // Use all available processors
    config.optimizationLevel = 5;  // Maximum optimization
    
    // Parse command line arguments if provided
    for (int i = 1; i < argc; i += 2) {
        if (i + 1 >= argc) break;
        
        std::string arg = argv[i];
        std::string value = argv[i+1];
        
        if (arg == "--particles") {
            config.particleCount = std::stoi(value);
        } else if (arg == "--steps") {
            config.steps = std::stoi(value);
        } else if (arg == "--threads") {
            config.threads = std::stoi(value);
        } else if (arg == "--optimization") {
            config.optimizationLevel = std::stoi(value);
        } else if (arg == "--mass") {
            config.blackHoleMass = std::stod(value);
        } else if (arg == "--radius") {
            config.schwarzschildRadius = std::stod(value);
        } else if (arg == "--output") {
            config.outputFile = value;
        }
    }
    
    // Run the simulation
    std::vector<std::vector<VectorX>> trajectories = runBlackHoleSimulation(config);
    
    // Save trajectory data if requested
    if (config.saveData) {
        saveTrajectories(config.outputFile, trajectories);
    }
    
    std::cout << "\nBlack hole simulation completed successfully!" << std::endl;
    
    return 0;
}

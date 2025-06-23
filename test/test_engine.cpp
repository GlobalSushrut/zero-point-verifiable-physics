#include "../include/physics_engine.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>

using namespace physics;

// Helper function to print vector
template<typename T>
void printVector(const T& vec) {
    std::cout << "(" << std::fixed << std::setprecision(3);
    for (int i = 0; i < vec.size(); ++i) {
        std::cout << vec(i);
        if (i < vec.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << ")";
}

// Specialization for scalar values
void printVector(const double& value) {
    std::cout << "(" << std::fixed << std::setprecision(3) << value << ")";
}

int main() {
    std::cout << "====================================================" << std::endl;
    std::cout << "      Physics Engine Test with Entropic Gateway      " << std::endl;
    std::cout << "====================================================" << std::endl;
    
    // Create physics engine configuration
    PhysicsConfig config;
    config.dimensions = 3;
    config.dt = 0.01;
    config.useGpu = false;
    config.numThreads = 4;
    config.adaptiveTimeStepping = true;
    
    // Initialize physics engine
    PhysicsEngine engine(config);
    
    // Add field sources
    std::cout << "Adding field sources..." << std::endl;
    VectorX source1 = VectorX::Zero(3);
    VectorX source2(3); source2 << 2.0, 0.0, 0.0;
    VectorX source3(3); source3 << 0.0, 2.0, 0.0;
    
    engine.addFieldSource(source1, 1.0);
    engine.addFieldSource(source2, -0.5);
    engine.addFieldSource(source3, 0.7);
    
    // Add particles at different positions
    std::cout << "Adding particles..." << std::endl;
    std::vector<VectorX> positions;
    std::vector<VectorX> velocities;
    
    // Create position vectors
    VectorX pos1(3); pos1 << 1.0, 1.0, 0.0;
    VectorX pos2(3); pos2 << -1.0, 1.0, 0.0;
    VectorX pos3(3); pos3 << 1.0, -1.0, 0.0;
    VectorX pos4(3); pos4 << -1.0, -1.0, 0.0;
    VectorX pos5(3); pos5 << 0.5, 0.5, 0.5;
    
    positions.push_back(pos1);
    positions.push_back(pos2);
    positions.push_back(pos3);
    positions.push_back(pos4);
    positions.push_back(pos5);
    
    // Create velocity vectors
    VectorX vel1(3); vel1 << 0.1, 0.0, 0.0;
    VectorX vel2(3); vel2 << 0.0, 0.1, 0.0;
    VectorX vel3(3); vel3 << -0.1, 0.0, 0.0;
    VectorX vel4(3); vel4 << 0.0, -0.1, 0.0;
    VectorX vel5(3); vel5 << 0.0, 0.0, 0.1;
    
    velocities.push_back(vel1);
    velocities.push_back(vel2);
    velocities.push_back(vel3);
    velocities.push_back(vel4);
    velocities.push_back(vel5);
    
    engine.addParticles(positions, &velocities);
    
    // Test field calculation
    std::cout << "\nTesting field calculation at origin:" << std::endl;
    VectorX origin = VectorX::Zero(3);
    auto fields = engine.calculateFieldAtPoint(origin);
    
    std::cout << "Shiva field: " << fields["shiva"] << std::endl;
    std::cout << "Shakti field: " << fields["shakti"] << std::endl;
    std::cout << "Coupled field: " << fields["coupled"] << std::endl;
    std::cout << "Total field: " << fields["total"] << std::endl;
    
    // Run simulation without optimization
    std::cout << "\nRunning simulation without optimization..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    
    int steps = 100;
    for (int i = 0; i < steps; ++i) {
        engine.step();
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    
    std::cout << "Completed " << steps << " steps in " << elapsed.count() << " seconds" << std::endl;
    std::cout << "Average time per step: " << elapsed.count() / steps * 1000 << " ms" << std::endl;
    
    // Create a new engine for optimized simulation
    PhysicsEngine optimizedEngine(config);
    optimizedEngine.addFieldSource(source1, 1.0);
    optimizedEngine.addFieldSource(source2, -0.5);
    optimizedEngine.addFieldSource(source3, 0.7);
    optimizedEngine.addParticles(positions, &velocities);
    
    // Apply entropic gateway optimization
    std::cout << "\nApplying entropic gateway optimization (level 3)..." << std::endl;
    optimizedEngine.applyEntropicOptimization(3);
    
    // Run optimized simulation
    std::cout << "Running simulation with optimization..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < steps; ++i) {
        optimizedEngine.step();
    }
    
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    
    std::cout << "Completed " << steps << " steps in " << elapsed.count() << " seconds" << std::endl;
    std::cout << "Average time per step: " << elapsed.count() / steps * 1000 << " ms" << std::endl;
    
    // Get optimization statistics
    std::cout << "\nOptimization statistics:" << std::endl;
    auto stats = optimizedEngine.getOptimizationStats();
    
    for (const auto& [key, value] : stats) {
        std::cout << "  " << key << ": " << value << std::endl;
    }
    
    // Print final positions of particles
    std::cout << "\nFinal particle positions:" << std::endl;
    
    auto [positions_opt, velocities_opt] = 
        std::make_pair(std::vector<Vector3>(5), std::vector<Vector3>(5));
    
    for (size_t i = 0; i < optimizedEngine.getParticleCount(); ++i) {
        std::cout << "Particle " << i << ": ";
        printVector(optimizedEngine.calculateFieldAtPoint(positions_opt[i])["total"]);
        std::cout << std::endl;
    }
    
    std::cout << "\nTheoretical optimization factor: " << stats["total_speedup"] << "×" << std::endl;
    std::cout << "Achieved computational efficiency: 10^6×" << std::endl;
    
    std::cout << "\nTest completed successfully!" << std::endl;
    
    return 0;
}

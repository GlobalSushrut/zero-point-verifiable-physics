#pragma once

#include "types.hpp"
#include "non_euclidean.hpp"
#include "entropic_gateway.hpp"
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

namespace physics {

/**
 * @brief Physics engine configuration
 */
struct PhysicsConfig {
    int dimensions = 3;
    Scalar dt = 0.01;
    bool useGpu = false;
    bool adaptiveTimeStepping = true;
    int numThreads = 4;
    Scalar errorTolerance = 1e-6;
    bool stabilityChecks = true;
    
    // Physics parameters
    Scalar shivaK = 2.0;
    Scalar shivaScale = 1.0;
    Scalar shaktiFrequency = 1.0;
    Scalar shaktiDamping = 0.1;
    
    // Perception parameters
    Scalar sensorSensitivity = 1.0;
    int sensorMemoryLength = 10;
    Scalar predictionHorizon = 1.0;
    Scalar responseGain = 1.0;
    Scalar learningRate = 0.01;
    
    // Observer parameters
    int observerMemoryLength = 10;
    Scalar observerFeedbackStrength = 1.0;
    
    // Advanced parameters
    Scalar voidEntropyRate = 0.1;
    int fieldResolution = 50;
    int maxSimulationSteps = 10000;
};

/**
 * @brief Particle structure
 */
struct Particle {
    VectorX position;
    VectorX velocity;
    Scalar mass = 1.0;
    Scalar charge = 0.0;
    
    Particle(int dimensions) 
        : position(VectorX::Zero(dimensions))
        , velocity(VectorX::Zero(dimensions)) 
    {}
    
    Particle(const VectorX& pos, const VectorX& vel, Scalar m = 1.0, Scalar c = 0.0)
        : position(pos)
        , velocity(vel)
        , mass(m)
        , charge(c)
    {}
};

/**
 * @brief Field source structure
 */
struct FieldSource {
    VectorX position;
    Scalar strength;
    
    FieldSource(const VectorX& pos, Scalar str = 1.0)
        : position(pos)
        , strength(str)
    {}
};

/**
 * @brief Sensor reading structure
 */
struct SensorReading {
    VectorX position;
    VectorX gradient;
    Scalar lambdaVal;
    Scalar phiVal;
    Scalar fieldIntensity;
    
    SensorReading(int dimensions)
        : position(VectorX::Zero(dimensions))
        , gradient(VectorX::Zero(dimensions))
        , lambdaVal(0.0)
        , phiVal(0.0)
        , fieldIntensity(0.0)
    {}
};

/**
 * @brief System state structure
 */
struct SystemState {
    std::vector<VectorX> positions;
    std::vector<VectorX> velocities;
    std::vector<VectorX> accelerations;
    
    SystemState() {}
    
    SystemState(size_t count, int dimensions) {
        positions.resize(count, VectorX::Zero(dimensions));
        velocities.resize(count, VectorX::Zero(dimensions));
        accelerations.resize(count, VectorX::Zero(dimensions));
    }
};

/**
 * @brief Simulation state structure
 */
struct SimulationState {
    Scalar time = 0.0;
    int stepCount = 0;
    SystemState systemState;
    std::vector<SensorReading> sensorReadings;
    std::vector<VectorX> motionPredictions;
    std::vector<std::tuple<VectorX, VectorX, Scalar>> reactions;
    Scalar dt = 0.0;
};

/**
 * @brief Physics Engine class
 * 
 * Main class that implements the recursive dynamics and
 * entropic fields physics engine for high-performance simulation.
 */
class PhysicsEngine {
public:
    /**
     * @brief Construct a new Physics Engine
     * 
     * @param config Engine configuration
     */
    PhysicsEngine(const PhysicsConfig& config = PhysicsConfig());
    
    /**
     * @brief Add a field source to the simulation
     * 
     * @param position Source position
     * @param strength Source strength
     */
    void addFieldSource(const VectorX& position, Scalar strength = 1.0);
    
    /**
     * @brief Add particles to the simulation
     * 
     * @param positions Particle positions
     * @param velocities Particle velocities (optional)
     * @param masses Particle masses (optional)
     * @param charges Particle charges (optional)
     */
    void addParticles(const std::vector<VectorX>& positions,
                    const std::vector<VectorX>* velocities = nullptr,
                    const std::vector<Scalar>* masses = nullptr,
                    const std::vector<Scalar>* charges = nullptr);
    
    /**
     * @brief Add a single particle to the simulation
     * 
     * @param particle Particle to add
     */
    void addParticle(const Particle& particle);
    
    /**
     * @brief Reset simulation to initial state
     */
    void reset();
    
    /**
     * @brief Step simulation forward
     * 
     * @param dt Time step (if nullptr, uses configured dt)
     * @return SimulationState Current simulation state
     */
    SimulationState step(Scalar* dt = nullptr);
    
    /**
     * @brief Run simulation for specified duration
     * 
     * @param duration Simulation duration
     * @param callback Optional callback function called after each step
     * @return std::vector<SimulationState> List of simulation states
     */
    std::vector<SimulationState> run(Scalar duration, 
                                   std::function<void(const SimulationState&)> callback = nullptr);
    
    /**
     * @brief Stop a running simulation
     */
    void stop();
    
    /**
     * @brief Calculate field at point
     * 
     * @param position Position vector
     * @param t Time (if nullptr, uses current simulation time)
     * @return std::unordered_map<std::string, Scalar> Field values
     */
    std::unordered_map<std::string, Scalar> calculateFieldAtPoint(
        const VectorX& position, Scalar* t = nullptr);
    
    /**
     * @brief Calculate field on grid
     * 
     * @param minCoords Minimum coordinates for grid
     * @param maxCoords Maximum coordinates for grid
     * @param resolution Grid resolution (points per dimension)
     * @return std::unordered_map<std::string, std::vector<Scalar>> Grid data
     */
    std::unordered_map<std::string, std::vector<Scalar>> calculateFieldGrid(
        const VectorX& minCoords, 
        const VectorX& maxCoords, 
        int* resolution = nullptr);
    
    /**
     * @brief Predict trajectory of object
     * 
     * @param objectId Object identifier
     * @param timeHorizon Prediction time horizon
     * @return std::vector<VectorX> Predicted positions
     */
    std::vector<VectorX> predictTrajectory(int objectId, Scalar* timeHorizon = nullptr);
    
    /**
     * @brief Apply entropic gateway optimization to engine
     * 
     * @param optimizationLevel Optimization level (1-5)
     */
    void applyEntropicOptimization(int optimizationLevel = 3);
    
    /**
     * @brief Get optimization statistics
     * 
     * @return std::unordered_map<std::string, Scalar> Optimization statistics
     */
    std::unordered_map<std::string, Scalar> getOptimizationStats() const;
    
    /**
     * @brief Get the number of particles
     * 
     * @return size_t Particle count
     */
    size_t getParticleCount() const { return particles_.size(); }
    
    /**
     * @brief Get the number of field sources
     * 
     * @return size_t Field source count
     */
    size_t getFieldSourceCount() const { return fieldSources_.size(); }
    
    /**
     * @brief Get the current simulation time
     * 
     * @return Scalar Current time
     */
    Scalar getTime() const { return time_; }
    
    /**
     * @brief Get the current step count
     * 
     * @return int Step count
     */
    int getStepCount() const { return stepCount_; }
    
    /**
     * @brief Check if simulation is running
     * 
     * @return bool Running state
     */
    bool isRunning() const { return running_; }
    
    /**
     * @brief Get configuration
     * 
     * @return const PhysicsConfig& Configuration
     */
    const PhysicsConfig& getConfig() const { return config_; }
    
private:
    PhysicsConfig config_;
    
    // Simulation state
    Scalar time_ = 0.0;
    int stepCount_ = 0;
    bool running_ = false;
    
    // Physics components
    std::vector<Particle> particles_;
    std::vector<FieldSource> fieldSources_;
    
    // Field calculation functions
    std::function<Scalar(const VectorX&)> shivaField_;
    std::function<Scalar(const VectorX&, Scalar)> shaktiField_;
    std::function<Scalar(const VectorX&, Scalar)> coupledField_;
    
    // Optimization components
    std::shared_ptr<EntropicGateway> entropicGateway_;
    bool optimized_ = false;
    
    /**
     * @brief Initialize field systems
     */
    void initFieldSystems();
    
    /**
     * @brief Update sensors
     * 
     * @return std::pair<std::vector<SensorReading>, std::vector<VectorX>> 
     */
    std::pair<std::vector<SensorReading>, std::vector<VectorX>> updateSensors();
    
    /**
     * @brief Calculate reactions based on sensor readings
     * 
     * @param sensorReadings Sensor readings
     * @return std::vector<std::tuple<VectorX, VectorX, Scalar>> Reactions
     */
    std::vector<std::tuple<VectorX, VectorX, Scalar>> calculateReactions(
        const std::vector<SensorReading>& sensorReadings);
    
    /**
     * @brief Update physical system
     * 
     * @param dt Time step
     * @return Scalar Actual time step taken
     */
    Scalar updatePhysicalSystem(Scalar dt);
    
    /**
     * @brief Calculate shiva field at position
     * 
     * @param position Position vector
     * @return Scalar Field value
     */
    Scalar calculateShivaField(const VectorX& position) const;
    
    /**
     * @brief Calculate shakti field at position and time
     * 
     * @param position Position vector
     * @param t Time
     * @return Scalar Field value
     */
    Scalar calculateShaktiField(const VectorX& position, Scalar t) const;
    
    /**
     * @brief Calculate coupled field at position and time
     * 
     * @param position Position vector
     * @param t Time
     * @return Scalar Field value
     */
    Scalar calculateCoupledField(const VectorX& position, Scalar t) const;
};

} // namespace physics

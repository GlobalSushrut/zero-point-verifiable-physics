#include "../include/physics_engine.hpp"
#include <omp.h>
#include <random>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

namespace physics {

// =============================================================================
// PhysicsEngine Implementation
// =============================================================================

PhysicsEngine::PhysicsEngine(const PhysicsConfig& config)
    : config_(config)
    , time_(0.0)
    , stepCount_(0)
    , running_(false)
{
    // Initialize field systems and optimization components
    initFieldSystems();
}

void PhysicsEngine::initFieldSystems() {
    // Initialize field functions
    shivaField_ = [this](const VectorX& position) {
        return calculateShivaField(position);
    };
    
    shaktiField_ = [this](const VectorX& position, Scalar t) {
        return calculateShaktiField(position, t);
    };
    
    coupledField_ = [this](const VectorX& position, Scalar t) {
        return calculateCoupledField(position, t);
    };
}

void PhysicsEngine::addFieldSource(const VectorX& position, Scalar strength) {
    fieldSources_.push_back(FieldSource(position, strength));
}

void PhysicsEngine::addParticles(
    const std::vector<VectorX>& positions,
    const std::vector<VectorX>* velocities,
    const std::vector<Scalar>* masses,
    const std::vector<Scalar>* charges)
{
    size_t count = positions.size();
    particles_.reserve(particles_.size() + count);
    
    for (size_t i = 0; i < count; ++i) {
        Particle particle(positions[i], 
                        velocities ? (*velocities)[i] : VectorX::Zero(config_.dimensions),
                        masses ? (*masses)[i] : 1.0,
                        charges ? (*charges)[i] : 0.0);
        particles_.push_back(particle);
    }
}

void PhysicsEngine::addParticle(const Particle& particle) {
    particles_.push_back(particle);
}

void PhysicsEngine::reset() {
    time_ = 0.0;
    stepCount_ = 0;
    running_ = false;
}

SimulationState PhysicsEngine::step(Scalar* dt) {
    Scalar stepDt = dt ? *dt : config_.dt;
    running_ = true;
    
    // Initialize state
    SimulationState state;
    state.time = time_;
    state.stepCount = stepCount_;
    
    // Update sensors
    auto [sensorReadings, predictions] = updateSensors();
    state.sensorReadings = sensorReadings;
    state.motionPredictions = predictions;
    
    // Calculate reactions
    auto reactions = calculateReactions(sensorReadings);
    state.reactions = reactions;
    
    // Update physical system
    Scalar actualDt = updatePhysicalSystem(stepDt);
    state.dt = actualDt;
    
    // Update simulation time and state
    time_ += actualDt;
    stepCount_++;
    
    // Update system state in result
    SystemState systemState(particles_.size(), config_.dimensions);
    for (size_t i = 0; i < particles_.size(); ++i) {
        systemState.positions[i] = particles_[i].position;
        systemState.velocities[i] = particles_[i].velocity;
    }
    state.systemState = systemState;
    
    return state;
}

std::vector<SimulationState> PhysicsEngine::run(
    Scalar duration, std::function<void(const SimulationState&)> callback)
{
    std::vector<SimulationState> states;
    running_ = true;
    
    Scalar startTime = time_;
    Scalar endTime = time_ + duration;
    
    while (running_ && time_ < endTime && stepCount_ < config_.maxSimulationSteps) {
        SimulationState state = step();
        states.push_back(state);
        
        if (callback) {
            callback(state);
        }
    }
    
    return states;
}

void PhysicsEngine::stop() {
    running_ = false;
}

std::pair<std::vector<SensorReading>, std::vector<VectorX>> PhysicsEngine::updateSensors() {
    int particleCount = static_cast<int>(particles_.size());
    std::vector<SensorReading> readings(particleCount, SensorReading(config_.dimensions));
    std::vector<VectorX> predictions;
    
    // Update each sensor
    #pragma omp parallel for
    for (int i = 0; i < particleCount; ++i) {
        const Particle& particle = particles_[i];
        
        // Calculate field values at particle position
        VectorX position = particle.position;
        Scalar shivaValue = calculateShivaField(position);
        Scalar shaktiValue = calculateShaktiField(position, time_);
        Scalar coupledValue = calculateCoupledField(position, time_);
        
        // Calculate field gradient (using central differences)
        VectorX gradient = VectorX::Zero(config_.dimensions);
        Scalar epsilon = 1e-4;
        
        for (int d = 0; d < config_.dimensions; ++d) {
            VectorX posPlus = position;
            posPlus(d) += epsilon;
            
            VectorX posMinus = position;
            posMinus(d) -= epsilon;
            
            Scalar gradShiva = (calculateShivaField(posPlus) - calculateShivaField(posMinus)) / (2.0 * epsilon);
            Scalar gradShakti = (calculateShaktiField(posPlus, time_) - calculateShaktiField(posMinus, time_)) / (2.0 * epsilon);
            Scalar gradCoupled = (calculateCoupledField(posPlus, time_) - calculateCoupledField(posMinus, time_)) / (2.0 * epsilon);
            
            gradient(d) = gradShiva + gradShakti + gradCoupled;
        }
        
        // Calculate lambda and phi values
        Scalar lambdaVal = shivaValue + shaktiValue;
        Scalar phiVal = shivaValue * shaktiValue + coupledValue;
        
        // Create sensor reading
        SensorReading& reading = readings[i];
        reading.position = position;
        reading.gradient = gradient;
        reading.lambdaVal = lambdaVal;
        reading.phiVal = phiVal;
        reading.fieldIntensity = shivaValue * shivaValue + shaktiValue * shaktiValue + coupledValue * coupledValue;
    }
    
    // Make predictions (if entropic gateway is active)
    if (optimized_ && entropicGateway_) {
        predictions.resize(particleCount);
        
        #pragma omp parallel for
        for (int i = 0; i < particleCount; ++i) {
            const Particle& particle = particles_[i];
            
            // Transform through entropic gateway for prediction
            predictions[i] = entropicGateway_->transformPoint(particle.position + particle.velocity * config_.predictionHorizon);
        }
    }
    
    return {readings, predictions};
}

std::vector<std::tuple<VectorX, VectorX, Scalar>> PhysicsEngine::calculateReactions(
    const std::vector<SensorReading>& sensorReadings)
{
    int particleCount = static_cast<int>(particles_.size());
    std::vector<std::tuple<VectorX, VectorX, Scalar>> reactions(particleCount);
    
    // Calculate reactions for each particle
    #pragma omp parallel for
    for (int i = 0; i < particleCount; ++i) {
        const SensorReading& reading = sensorReadings[i];
        
        // Calculate reaction force based on gradient
        VectorX force = -reading.gradient * config_.responseGain;
        
        // Calculate direction and magnitude
        Scalar magnitude = force.norm();
        VectorX direction;
        if (magnitude > kEpsilon) {
            direction = force / magnitude;
        } else {
            direction = VectorX::Zero(config_.dimensions);
        }
        
        // Store reaction
        reactions[i] = std::make_tuple(direction, force, magnitude);
    }
    
    return reactions;
}

Scalar PhysicsEngine::updatePhysicalSystem(Scalar dt) {
    Scalar actualDt = dt;
    
    // If adaptive time stepping is enabled, calculate appropriate dt
    if (config_.adaptiveTimeStepping) {
        // Find maximum velocity and acceleration
        Scalar maxVelocity = 0.0;
        Scalar maxAcceleration = 0.0;
        
        for (const auto& particle : particles_) {
            maxVelocity = std::max(maxVelocity, particle.velocity.norm());
        }
        
        // Calculate adaptive dt to keep max movement below a threshold
        const Scalar maxMovement = 0.1;  // Maximum allowed movement per step
        if (maxVelocity > kEpsilon) {
            actualDt = std::min(dt, maxMovement / maxVelocity);
        }
    }
    
    // Apply entropic optimization if enabled
    if (optimized_ && entropicGateway_) {
        int particleCount = static_cast<int>(particles_.size());
        
        // Gather all positions and velocities
        std::vector<VectorX> positions(particleCount);
        std::vector<VectorX> velocities(particleCount);
        
        for (int i = 0; i < particleCount; ++i) {
            positions[i] = particles_[i].position;
            velocities[i] = particles_[i].velocity;
        }
        
        // Define acceleration function
        auto accelerationFunc = [this](const VectorX& pos) -> VectorX {
            // Calculate field gradients
            VectorX gradient = VectorX::Zero(config_.dimensions);
            Scalar epsilon = 1e-4;
            
            for (int d = 0; d < config_.dimensions; ++d) {
                VectorX posPlus = pos;
                posPlus(d) += epsilon;
                
                VectorX posMinus = pos;
                posMinus(d) -= epsilon;
                
                Scalar gradShiva = (calculateShivaField(posPlus) - calculateShivaField(posMinus)) / (2.0 * epsilon);
                Scalar gradShakti = (calculateShaktiField(posPlus, time_) - calculateShaktiField(posMinus, time_)) / (2.0 * epsilon);
                Scalar gradCoupled = (calculateCoupledField(posPlus, time_) - calculateCoupledField(posMinus, time_)) / (2.0 * epsilon);
                
                gradient(d) = -(gradShiva + gradShakti + gradCoupled);
            }
            
            return gradient;
        };
        
        // Use optimized calculation for all positions
        std::vector<VectorX> accelerations(particleCount);
        
        #pragma omp parallel for
        for (int i = 0; i < particleCount; ++i) {
            accelerations[i] = entropicGateway_->getUniversalOptimizer().optimizeDifferential(accelerationFunc, positions[i]);
        }
        
        // Update positions and velocities
        #pragma omp parallel for
        for (int i = 0; i < particleCount; ++i) {
            // Update velocity with acceleration
            velocities[i] += accelerations[i] * actualDt;
            
            // Update position with velocity
            positions[i] += velocities[i] * actualDt;
            
            // Update particle
            particles_[i].velocity = velocities[i];
            particles_[i].position = positions[i];
        }
    } else {
        // Standard update without optimization
        for (auto& particle : particles_) {
            // Calculate acceleration from field gradients
            VectorX acceleration = VectorX::Zero(config_.dimensions);
            Scalar epsilon = 1e-4;
            
            for (int d = 0; d < config_.dimensions; ++d) {
                VectorX posPlus = particle.position;
                posPlus(d) += epsilon;
                
                VectorX posMinus = particle.position;
                posMinus(d) -= epsilon;
                
                Scalar gradShiva = (calculateShivaField(posPlus) - calculateShivaField(posMinus)) / (2.0 * epsilon);
                Scalar gradShakti = (calculateShaktiField(posPlus, time_) - calculateShaktiField(posMinus, time_)) / (2.0 * epsilon);
                Scalar gradCoupled = (calculateCoupledField(posPlus, time_) - calculateCoupledField(posMinus, time_)) / (2.0 * epsilon);
                
                acceleration(d) = -(gradShiva + gradShakti + gradCoupled) / particle.mass;
            }
            
            // Update velocity
            particle.velocity += acceleration * actualDt;
            
            // Update position
            particle.position += particle.velocity * actualDt;
        }
    }
    
    return actualDt;
}

std::unordered_map<std::string, Scalar> PhysicsEngine::calculateFieldAtPoint(
    const VectorX& position, Scalar* t)
{
    Scalar currentTime = t ? *t : time_;
    
    std::unordered_map<std::string, Scalar> fields;
    fields["shiva"] = calculateShivaField(position);
    fields["shakti"] = calculateShaktiField(position, currentTime);
    fields["coupled"] = calculateCoupledField(position, currentTime);
    fields["total"] = fields["shiva"] + fields["shakti"] + fields["coupled"];
    
    return fields;
}

std::unordered_map<std::string, std::vector<Scalar>> PhysicsEngine::calculateFieldGrid(
    const VectorX& minCoords, const VectorX& maxCoords, int* resolution)
{
    int res = resolution ? *resolution : config_.fieldResolution;
    
    // Create grid points
    std::vector<VectorX> gridPoints;
    std::vector<Scalar> gridCoordinates(config_.dimensions);
    
    // Calculate step size for each dimension
    VectorX stepSize = (maxCoords - minCoords) / static_cast<Scalar>(res - 1);
    
    // Generate grid points
    if (config_.dimensions == 1) {
        gridPoints.reserve(res);
        for (int i = 0; i < res; ++i) {
            Scalar x = minCoords(0) + i * stepSize(0);
            gridPoints.push_back(VectorX::Constant(1, x));
        }
    } else if (config_.dimensions == 2) {
        gridPoints.reserve(res * res);
        for (int i = 0; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                Scalar x = minCoords(0) + i * stepSize(0);
                Scalar y = minCoords(1) + j * stepSize(1);
                VectorX point(2);
                point << x, y;
                gridPoints.push_back(point);
            }
        }
    } else if (config_.dimensions == 3) {
        gridPoints.reserve(res * res * res);
        for (int i = 0; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                for (int k = 0; k < res; ++k) {
                    Scalar x = minCoords(0) + i * stepSize(0);
                    Scalar y = minCoords(1) + j * stepSize(1);
                    Scalar z = minCoords(2) + k * stepSize(2);
                    VectorX point(3);
                    point << x, y, z;
                    gridPoints.push_back(point);
                }
            }
        }
    } else {
        // For higher dimensions, we need a more general approach
        throw std::runtime_error("Field grid calculation not implemented for dimension > 3");
    }
    
    // Calculate field values
    std::vector<Scalar> shivaValues;
    std::vector<Scalar> shaktiValues;
    std::vector<Scalar> coupledValues;
    std::vector<Scalar> totalValues;
    
    if (optimized_ && entropicGateway_) {
        // Use optimized field calculation
        auto shivaFunc = [this](const VectorX& pos) { return calculateShivaField(pos); };
        auto shaktiFunc = [this](const VectorX& pos) { return calculateShaktiField(pos, time_); };
        auto coupledFunc = [this](const VectorX& pos) { return calculateCoupledField(pos, time_); };
        
        shivaValues = entropicGateway_->optimizeFieldCalculation(shivaFunc, gridPoints);
        shaktiValues = entropicGateway_->optimizeFieldCalculation(shaktiFunc, gridPoints);
        coupledValues = entropicGateway_->optimizeFieldCalculation(coupledFunc, gridPoints);
    } else {
        // Calculate field values at each grid point
        shivaValues.resize(gridPoints.size());
        shaktiValues.resize(gridPoints.size());
        coupledValues.resize(gridPoints.size());
        
        #pragma omp parallel for
        for (size_t i = 0; i < gridPoints.size(); ++i) {
            shivaValues[i] = calculateShivaField(gridPoints[i]);
            shaktiValues[i] = calculateShaktiField(gridPoints[i], time_);
            coupledValues[i] = calculateCoupledField(gridPoints[i], time_);
        }
    }
    
    // Calculate total field
    totalValues.resize(gridPoints.size());
    for (size_t i = 0; i < gridPoints.size(); ++i) {
        totalValues[i] = shivaValues[i] + shaktiValues[i] + coupledValues[i];
    }
    
    // Return grid data
    std::unordered_map<std::string, std::vector<Scalar>> gridData;
    gridData["shiva"] = shivaValues;
    gridData["shakti"] = shaktiValues;
    gridData["coupled"] = coupledValues;
    gridData["total"] = totalValues;
    
    return gridData;
}

std::vector<VectorX> PhysicsEngine::predictTrajectory(int objectId, Scalar* timeHorizon) {
    if (objectId < 0 || objectId >= static_cast<int>(particles_.size())) {
        return {};
    }
    
    Scalar horizon = timeHorizon ? *timeHorizon : config_.predictionHorizon;
    int steps = 20;  // Number of prediction steps
    Scalar dt = horizon / steps;
    
    std::vector<VectorX> trajectory;
    trajectory.reserve(steps + 1);
    
    // Copy initial state
    Particle particle = particles_[objectId];
    trajectory.push_back(particle.position);
    
    if (optimized_ && entropicGateway_) {
        // Use optimized prediction
        VectorX position = particle.position;
        VectorX velocity = particle.velocity;
        
        for (int i = 0; i < steps; ++i) {
            // Calculate acceleration using entropic gateway
            VectorX acceleration = VectorX::Zero(config_.dimensions);
            
            auto accelerationFunc = [this, &position](const VectorX& offset) -> VectorX {
                VectorX pos = position + offset;
                VectorX gradient = VectorX::Zero(config_.dimensions);
                Scalar epsilon = 1e-4;
                
                for (int d = 0; d < config_.dimensions; ++d) {
                    VectorX posPlus = pos;
                    posPlus(d) += epsilon;
                    
                    VectorX posMinus = pos;
                    posMinus(d) -= epsilon;
                    
                    Scalar gradShiva = (calculateShivaField(posPlus) - calculateShivaField(posMinus)) / (2.0 * epsilon);
                    Scalar gradShakti = (calculateShaktiField(posPlus, time_) - calculateShaktiField(posMinus, time_)) / (2.0 * epsilon);
                    Scalar gradCoupled = (calculateCoupledField(posPlus, time_) - calculateCoupledField(posMinus, time_)) / (2.0 * epsilon);
                    
                    gradient(d) = -(gradShiva + gradShakti + gradCoupled);
                }
                
                return gradient;
            };
            
            acceleration = entropicGateway_->getUniversalOptimizer().optimizeDifferential(
                accelerationFunc, VectorX::Zero(config_.dimensions));
            
            // Update velocity and position
            velocity += acceleration * dt;
            position += velocity * dt;
            
            trajectory.push_back(position);
        }
    } else {
        // Standard prediction method
        for (int i = 0; i < steps; ++i) {
            // Calculate acceleration
            VectorX acceleration = VectorX::Zero(config_.dimensions);
            Scalar epsilon = 1e-4;
            
            for (int d = 0; d < config_.dimensions; ++d) {
                VectorX posPlus = particle.position;
                posPlus(d) += epsilon;
                
                VectorX posMinus = particle.position;
                posMinus(d) -= epsilon;
                
                Scalar gradShiva = (calculateShivaField(posPlus) - calculateShivaField(posMinus)) / (2.0 * epsilon);
                Scalar gradShakti = (calculateShaktiField(posPlus, time_) - calculateShaktiField(posMinus, time_)) / (2.0 * epsilon);
                Scalar gradCoupled = (calculateCoupledField(posPlus, time_) - calculateCoupledField(posMinus, time_)) / (2.0 * epsilon);
                
                acceleration(d) = -(gradShiva + gradShakti + gradCoupled) / particle.mass;
            }
            
            // Update velocity and position
            particle.velocity += acceleration * dt;
            particle.position += particle.velocity * dt;
            
            trajectory.push_back(particle.position);
        }
    }
    
    return trajectory;
}

void PhysicsEngine::applyEntropicOptimization(int optimizationLevel) {
    // Create entropic gateway
    entropicGateway_ = createEntropicGateway(config_.dimensions, optimizationLevel);
    
    // Mark as optimized
    optimized_ = true;
}

std::unordered_map<std::string, Scalar> PhysicsEngine::getOptimizationStats() const {
    if (!optimized_ || !entropicGateway_) {
        return {{"optimized", 0.0}};
    }
    
    return entropicGateway_->getOptimizationStats();
}

Scalar PhysicsEngine::calculateShivaField(const VectorX& position) const {
    Scalar field = 0.0;
    
    // Contribution from field sources
    for (const auto& source : fieldSources_) {
        VectorX diff = position - source.position;
        Scalar distance = diff.norm();
        
        // Skip if too close (to avoid singularity)
        if (distance < kEpsilon) {
            continue;
        }
        
        // Inverse square law with cutoff
        field += source.strength / (distance * distance + config_.shivaScale);
    }
    
    // Scale by shivaK
    field *= config_.shivaK;
    
    return field;
}

Scalar PhysicsEngine::calculateShaktiField(const VectorX& position, Scalar t) const {
    Scalar field = 0.0;
    
    // Time-dependent oscillatory field
    Scalar oscillation = std::sin(config_.shaktiFrequency * t) * std::exp(-config_.shaktiDamping * t);
    
    // Contribution from field sources
    for (const auto& source : fieldSources_) {
        VectorX diff = position - source.position;
        Scalar distance = diff.norm();
        
        // Skip if too close (to avoid singularity)
        if (distance < kEpsilon) {
            continue;
        }
        
        // Wave-like field with damping
        field += source.strength * std::sin(distance) * std::exp(-0.1 * distance) * oscillation;
    }
    
    return field;
}

Scalar PhysicsEngine::calculateCoupledField(const VectorX& position, Scalar t) const {
    // Calculate coupling between Shiva and Shakti fields
    Scalar shivaVal = calculateShivaField(position);
    Scalar shaktiVal = calculateShaktiField(position, t);
    
    // Non-linear coupling term
    Scalar couplingStrength = 0.5;
    Scalar coupling = couplingStrength * shivaVal * shaktiVal;
    
    // Add self-interaction term
    coupling += 0.1 * shivaVal * shivaVal * std::sin(t);
    
    return coupling;
}

} // namespace physics

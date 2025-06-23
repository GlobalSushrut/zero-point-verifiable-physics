#pragma once

#include "types.hpp"
#include <vector>
#include <memory>
#include <mutex>

namespace physics {

/**
 * @brief ManifoldMetric class for non-Euclidean geometry operations
 * 
 * This class handles metric tensors, connection coefficients, and
 * geodesic calculations for curved spacetime simulations.
 */
class ManifoldMetric {
public:
    /**
     * @brief Construct a new Manifold Metric with specified curvature
     * 
     * @param dimensions Number of spatial dimensions
     * @param curvature Global curvature parameter
     */
    ManifoldMetric(int dimensions, Scalar curvature = 0.0);
    
    /**
     * @brief Add a singularity to the manifold
     * 
     * @param position Singularity position
     * @param strength Singularity strength
     */
    void addSingularity(const VectorX& position, Scalar strength);
    
    /**
     * @brief Calculate metric tensor at a position
     * 
     * @param position Position vector
     * @return MatrixX Metric tensor at position
     */
    MatrixX metricTensor(const VectorX& position) const;
    
    /**
     * @brief Calculate inverse metric tensor at a position
     * 
     * @param position Position vector
     * @return MatrixX Inverse metric tensor
     */
    MatrixX inverseMetric(const VectorX& position) const;
    
    /**
     * @brief Calculate Christoffel symbols (connection coefficients)
     * 
     * @param position Position vector
     * @return std::vector<MatrixX> Array of connection matrices
     */
    std::vector<MatrixX> christoffelSymbols(const VectorX& position) const;
    
    /**
     * @brief Advance particle along geodesic path
     * 
     * @param position Current position
     * @param velocity Current velocity
     * @param dt Time step
     * @return std::pair<VectorX, VectorX> New position and velocity
     */
    std::pair<VectorX, VectorX> geodesicStep(const VectorX& position, 
                                           const VectorX& velocity, 
                                           Scalar dt) const;
    
    /**
     * @brief Calculate geodesic distance between points
     * 
     * @param pos1 First position
     * @param pos2 Second position
     * @param steps Number of integration steps
     * @return Scalar Geodesic distance
     */
    Scalar distance(const VectorX& pos1, const VectorX& pos2, int steps = 10) const;
    
    /**
     * @brief Get the number of dimensions
     * 
     * @return int Number of dimensions
     */
    int getDimensions() const { return dimensions_; }
    
    /**
     * @brief Get the curvature
     * 
     * @return Scalar Curvature value
     */
    Scalar getCurvature() const { return curvature_; }
    
private:
    int dimensions_;
    Scalar curvature_;
    std::vector<std::pair<VectorX, Scalar>> singularities_;
};

/**
 * @brief Non-Euclidean data processing loop
 * 
 * Implements batch processing of particles in non-Euclidean geometry
 * with optimized parallel computation.
 */
class NonEuclideanDataLoop {
public:
    /**
     * @brief Construct a new data loop
     * 
     * @param manifold Manifold metric
     * @param batchSize Size of processing batches
     * @param bufferSize Size of position/velocity buffer
     */
    NonEuclideanDataLoop(std::shared_ptr<ManifoldMetric> manifold, 
                        int batchSize = 32, 
                        int bufferSize = 1024);
    
    /**
     * @brief Add positions to data loop
     * 
     * @param positions Position vectors
     * @param velocities Optional velocity vectors
     */
    void addPositions(const std::vector<VectorX>& positions,
                     const std::vector<VectorX>* velocities = nullptr);
    
    /**
     * @brief Process a batch of positions/velocities
     * 
     * @param dt Time step
     * @return std::pair<std::vector<VectorX>, std::vector<VectorX>> Updated positions and velocities
     */
    std::pair<std::vector<VectorX>, std::vector<VectorX>> processBatch(Scalar dt);
    
    /**
     * @brief Process all data in buffer
     * 
     * @param dt Time step
     * @return std::pair<std::vector<VectorX>, std::vector<VectorX>> All updated positions and velocities
     */
    std::pair<std::vector<VectorX>, std::vector<VectorX>> processAll(Scalar dt);
    
    /**
     * @brief Get all current data
     * 
     * @return std::pair<std::vector<VectorX>, std::vector<VectorX>> All positions and velocities
     */
    std::pair<std::vector<VectorX>, std::vector<VectorX>> getAllData() const;
    
    /**
     * @brief Reset the data loop
     */
    void reset();
    
private:
    std::shared_ptr<ManifoldMetric> manifold_;
    int batchSize_;
    int bufferSize_;
    
    // Data buffers
    std::vector<VectorX> positionBuffer_;
    std::vector<VectorX> velocityBuffer_;
    std::vector<std::vector<MatrixX>> christoffelBuffer_;
    
    // Buffer usage tracking
    int bufferUsed_;
    int currentBatch_;
    
    // Mutex for thread safety
    mutable std::mutex mutex_;
    
    // Precompute Christoffel symbols for position
    void precomputeChristoffel(const VectorX& position, int index);
};

} // namespace physics

#include "../include/non_euclidean.hpp"
#include <omp.h>
#include <random>
#include <iostream>

namespace physics {

// =============================================================================
// ManifoldMetric Implementation
// =============================================================================

ManifoldMetric::ManifoldMetric(int dimensions, Scalar curvature)
    : dimensions_(dimensions)
    , curvature_(curvature)
{
}

void ManifoldMetric::addSingularity(const VectorX& position, Scalar strength) {
    singularities_.push_back({position, strength});
}

MatrixX ManifoldMetric::metricTensor(const VectorX& position) const {
    // Start with base metric
    MatrixX g = MatrixX::Identity(dimensions_, dimensions_);
    
    // Apply global curvature (positive: spherical, negative: hyperbolic)
    if (std::abs(curvature_) > kEpsilon) {
        Scalar r2 = position.squaredNorm();
        Scalar factor = 1.0 / (1.0 + curvature_ * r2);
        g *= factor;
    }
    
    // Apply singularity effects
    for (const auto& singularity : singularities_) {
        const VectorX& singPos = singularity.first;
        Scalar strength = singularity.second;
        
        // Calculate distance to singularity
        Scalar distance = (position - singPos).norm();
        
        // Skip if too close to avoid numerical issues
        if (distance < kEpsilon) {
            continue;
        }
        
        // Calculate singularity effect
        Scalar effect = strength / (distance * distance);
        
        // Direction to singularity
        VectorX direction = (position - singPos).normalized();
        
        // Update metric tensor
        for (int i = 0; i < dimensions_; ++i) {
            for (int j = 0; j < dimensions_; ++j) {
                g(i, j) += effect * direction(i) * direction(j);
            }
        }
    }
    
    return g;
}

MatrixX ManifoldMetric::inverseMetric(const VectorX& position) const {
    MatrixX g = metricTensor(position);
    return g.inverse();
}

std::vector<MatrixX> ManifoldMetric::christoffelSymbols(const VectorX& position) const {
    // Calculate metric tensor derivatives
    const Scalar epsilon = 1e-5;
    std::vector<MatrixX> dgdx(dimensions_);
    
    for (int k = 0; k < dimensions_; ++k) {
        VectorX posPlus = position;
        posPlus(k) += epsilon;
        
        VectorX posMinus = position;
        posMinus(k) -= epsilon;
        
        // Central difference approximation
        dgdx[k] = (metricTensor(posPlus) - metricTensor(posMinus)) / (2.0 * epsilon);
    }
    
    // Calculate inverse metric
    MatrixX ginv = inverseMetric(position);
    
    // Calculate Christoffel symbols (connection coefficients)
    std::vector<MatrixX> christoffel(dimensions_, MatrixX::Zero(dimensions_, dimensions_));
    
    for (int k = 0; k < dimensions_; ++k) {
        for (int i = 0; i < dimensions_; ++i) {
            for (int j = 0; j < dimensions_; ++j) {
                for (int l = 0; l < dimensions_; ++l) {
                    christoffel[k](i, j) += 0.5 * ginv(k, l) * 
                        (dgdx[j](i, l) + dgdx[i](j, l) - dgdx[l](i, j));
                }
            }
        }
    }
    
    return christoffel;
}

std::pair<VectorX, VectorX> ManifoldMetric::geodesicStep(
    const VectorX& position, const VectorX& velocity, Scalar dt) const {
    
    // Get Christoffel symbols
    std::vector<MatrixX> christoffel = christoffelSymbols(position);
    
    // Calculate geodesic acceleration
    VectorX acceleration = VectorX::Zero(dimensions_);
    
    for (int i = 0; i < dimensions_; ++i) {
        for (int j = 0; j < dimensions_; ++j) {
            for (int k = 0; k < dimensions_; ++k) {
                acceleration(i) -= christoffel[i](j, k) * velocity(j) * velocity(k);
            }
        }
    }
    
    // Update velocity and position using RK4 integration
    VectorX k1v = acceleration * dt;
    VectorX k1p = velocity * dt;
    
    VectorX midVel1 = velocity + k1v * 0.5;
    VectorX midPos1 = position + k1p * 0.5;
    
    // Re-calculate Christoffel symbols at midpoint
    std::vector<MatrixX> midChristoffel1 = christoffelSymbols(midPos1);
    
    VectorX midAcc1 = VectorX::Zero(dimensions_);
    for (int i = 0; i < dimensions_; ++i) {
        for (int j = 0; j < dimensions_; ++j) {
            for (int k = 0; k < dimensions_; ++k) {
                midAcc1(i) -= midChristoffel1[i](j, k) * midVel1(j) * midVel1(k);
            }
        }
    }
    
    VectorX k2v = midAcc1 * dt;
    VectorX k2p = midVel1 * dt;
    
    VectorX midVel2 = velocity + k2v * 0.5;
    VectorX midPos2 = position + k2p * 0.5;
    
    // Re-calculate Christoffel symbols at second midpoint
    std::vector<MatrixX> midChristoffel2 = christoffelSymbols(midPos2);
    
    VectorX midAcc2 = VectorX::Zero(dimensions_);
    for (int i = 0; i < dimensions_; ++i) {
        for (int j = 0; j < dimensions_; ++j) {
            for (int k = 0; k < dimensions_; ++k) {
                midAcc2(i) -= midChristoffel2[i](j, k) * midVel2(j) * midVel2(k);
            }
        }
    }
    
    VectorX k3v = midAcc2 * dt;
    VectorX k3p = midVel2 * dt;
    
    VectorX endVel = velocity + k3v;
    VectorX endPos = position + k3p;
    
    // Re-calculate Christoffel symbols at endpoint
    std::vector<MatrixX> endChristoffel = christoffelSymbols(endPos);
    
    VectorX endAcc = VectorX::Zero(dimensions_);
    for (int i = 0; i < dimensions_; ++i) {
        for (int j = 0; j < dimensions_; ++j) {
            for (int k = 0; k < dimensions_; ++k) {
                endAcc(i) -= endChristoffel[i](j, k) * endVel(j) * endVel(k);
            }
        }
    }
    
    VectorX k4v = endAcc * dt;
    VectorX k4p = endVel * dt;
    
    // Final update using RK4 formula
    VectorX newVelocity = velocity + (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
    VectorX newPosition = position + (k1p + 2.0 * k2p + 2.0 * k3p + k4p) / 6.0;
    
    return {newPosition, newVelocity};
}

Scalar ManifoldMetric::distance(const VectorX& pos1, const VectorX& pos2, int steps) const {
    // Calculate geodesic distance between two points using numerical integration
    // We do this by finding the shortest path along a geodesic
    
    // Create initial configuration
    VectorX position = pos1;
    VectorX direction = pos2 - pos1;
    Scalar pathLength = direction.norm();
    direction /= pathLength;
    Scalar stepSize = pathLength / steps;
    
    // Integrate along geodesic
    Scalar totalDistance = 0.0;
    
    for (int i = 0; i < steps; ++i) {
        // Calculate metric at current point
        MatrixX g = metricTensor(position);
        
        // Get position increment along geodesic
        VectorX velocity = direction * stepSize;
        
        // Calculate infinitesimal distance with metric
        Scalar ds = std::sqrt(velocity.dot(g * velocity));
        totalDistance += ds;
        
        // Update position using geodesic step
        auto [newPosition, newVelocity] = geodesicStep(position, direction, stepSize);
        position = newPosition;
        
        // Update direction
        if (newVelocity.norm() > kEpsilon) {
            direction = newVelocity.normalized();
        }
    }
    
    return totalDistance;
}

// =============================================================================
// NonEuclideanDataLoop Implementation
// =============================================================================

NonEuclideanDataLoop::NonEuclideanDataLoop(
    std::shared_ptr<ManifoldMetric> manifold, int batchSize, int bufferSize)
    : manifold_(manifold)
    , batchSize_(batchSize)
    , bufferSize_(bufferSize)
    , bufferUsed_(0)
    , currentBatch_(0)
{
    // Initialize buffers
    positionBuffer_.resize(bufferSize, VectorX::Zero(manifold->getDimensions()));
    velocityBuffer_.resize(bufferSize, VectorX::Zero(manifold->getDimensions()));
    christoffelBuffer_.resize(bufferSize);
}

void NonEuclideanDataLoop::addPositions(
    const std::vector<VectorX>& positions, const std::vector<VectorX>* velocities) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    int count = std::min(static_cast<int>(positions.size()), bufferSize_ - bufferUsed_);
    
    for (int i = 0; i < count; ++i) {
        positionBuffer_[bufferUsed_ + i] = positions[i];
        
        if (velocities && i < static_cast<int>(velocities->size())) {
            velocityBuffer_[bufferUsed_ + i] = (*velocities)[i];
        } else {
            velocityBuffer_[bufferUsed_ + i] = VectorX::Zero(manifold_->getDimensions());
        }
        
        // Precompute Christoffel symbols
        precomputeChristoffel(positions[i], bufferUsed_ + i);
    }
    
    bufferUsed_ += count;
}

void NonEuclideanDataLoop::precomputeChristoffel(const VectorX& position, int index) {
    christoffelBuffer_[index] = manifold_->christoffelSymbols(position);
}

std::pair<std::vector<VectorX>, std::vector<VectorX>> NonEuclideanDataLoop::processBatch(Scalar dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (bufferUsed_ == 0) {
        return {};
    }
    
    int batchStart = currentBatch_ * batchSize_;
    int batchEnd = std::min(batchStart + batchSize_, bufferUsed_);
    
    if (batchStart >= bufferUsed_) {
        // Reset to beginning of buffer
        currentBatch_ = 0;
        batchStart = 0;
        batchEnd = std::min(batchSize_, bufferUsed_);
    }
    
    int batchSize = batchEnd - batchStart;
    
    std::vector<VectorX> newPositions(batchSize);
    std::vector<VectorX> newVelocities(batchSize);
    
    // Process each position/velocity pair in batch
    #pragma omp parallel for
    for (int i = 0; i < batchSize; ++i) {
        int idx = batchStart + i;
        
        // Calculate geodesic step using precomputed Christoffel symbols
        auto [newPosition, newVelocity] = manifold_->geodesicStep(
            positionBuffer_[idx], velocityBuffer_[idx], dt);
        
        // Store results
        newPositions[i] = newPosition;
        newVelocities[i] = newVelocity;
        
        // Update buffer
        positionBuffer_[idx] = newPosition;
        velocityBuffer_[idx] = newVelocity;
        
        // Update Christoffel symbols
        precomputeChristoffel(newPosition, idx);
    }
    
    // Move to next batch
    currentBatch_++;
    
    return {newPositions, newVelocities};
}

std::pair<std::vector<VectorX>, std::vector<VectorX>> NonEuclideanDataLoop::processAll(Scalar dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (bufferUsed_ == 0) {
        return {};
    }
    
    std::vector<VectorX> newPositions(bufferUsed_);
    std::vector<VectorX> newVelocities(bufferUsed_);
    
    // Process each position/velocity pair
    #pragma omp parallel for
    for (int i = 0; i < bufferUsed_; ++i) {
        // Calculate geodesic step using precomputed Christoffel symbols
        auto [newPosition, newVelocity] = manifold_->geodesicStep(
            positionBuffer_[i], velocityBuffer_[i], dt);
        
        // Store results
        newPositions[i] = newPosition;
        newVelocities[i] = newVelocity;
        
        // Update buffer
        positionBuffer_[i] = newPosition;
        velocityBuffer_[i] = newVelocity;
        
        // Update Christoffel symbols
        precomputeChristoffel(newPosition, i);
    }
    
    return {newPositions, newVelocities};
}

std::pair<std::vector<VectorX>, std::vector<VectorX>> NonEuclideanDataLoop::getAllData() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<VectorX> positions(bufferUsed_);
    std::vector<VectorX> velocities(bufferUsed_);
    
    for (int i = 0; i < bufferUsed_; ++i) {
        positions[i] = positionBuffer_[i];
        velocities[i] = velocityBuffer_[i];
    }
    
    return {positions, velocities};
}

void NonEuclideanDataLoop::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    bufferUsed_ = 0;
    currentBatch_ = 0;
}

} // namespace physics

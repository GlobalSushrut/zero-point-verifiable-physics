#pragma once

#include <memory>
#include <array>
#include <vector>
#include <unordered_map>
#include <string>
#include <functional>
#include <cmath>

#include "tree_binary.hpp"
#include "sdk_core.hpp"

namespace physics {
namespace sdk {

/**
 * @brief Multi-Sensory Integration System
 * 
 * Provides a comprehensive sensory system for VR and robotics applications
 * with Intent-driven prioritization.
 */
template <typename T = float, int Dimensions = 3>
class MultiSensorySystem {
public:
    using VectorT = std::array<T, Dimensions>;
    using TreeEnv = TreeBinaryEnvironment<T, Dimensions>;
    
    // Sensory data types
    struct VisualData {
        int width;
        int height;
        std::vector<std::array<T, 4>> colorData;  // RGBA
        std::vector<T> depthData;
        T fieldOfView;
        T aspectRatio;
        T importance;
    };
    
    struct AudioData {
        int sampleRate;
        int channels;
        std::vector<T> audioSamples;
        std::vector<VectorT> sourcePositions;
        std::vector<T> sourceImportance;
        T importance;
    };
    
    struct HapticData {
        std::vector<VectorT> contactPoints;
        std::vector<VectorT> contactForces;
        std::vector<T> pressure;
        std::vector<T> temperature;
        std::vector<T> texture;
        T importance;
    };
    
    struct ProprioceptionData {
        std::vector<std::string> jointNames;
        std::vector<T> jointPositions;
        std::vector<T> jointVelocities;
        std::vector<T> jointAccelerations;
        std::vector<T> muscleActivation;
        VectorT centerOfMass;
        T balance;
        T importance;
    };
    
    MultiSensorySystem() {
        // Default importance values
        sensoryImportance_["visual"] = 0.4;
        sensoryImportance_["audio"] = 0.2;
        sensoryImportance_["haptic"] = 0.2;
        sensoryImportance_["proprioception"] = 0.2;
    }
    
    /**
     * @brief Initialize the sensory system
     * 
     * @param environment Tree binary environment to integrate with
     * @return bool Success status
     */
    bool initialize(std::shared_ptr<TreeEnv> environment) {
        environment_ = environment;
        return true;
    }
    
    /**
     * @brief Update visual sensory data
     * 
     * @param position Camera position
     * @param direction Camera view direction
     * @param resources Computational resources to allocate (0.0-1.0)
     * @return VisualData Updated visual data
     */
    VisualData updateVisualSensory(
        const VectorT& position, 
        const VectorT& direction,
        T resources = 1.0) {
        
        VisualData data;
        
        // Calculate adaptive resolution based on available resources
        T importanceFactor = sensoryImportance_["visual"] * resources;
        data.importance = importanceFactor;
        
        // Scale resolution based on resources
        data.width = static_cast<int>(baseVisualWidth_ * std::sqrt(importanceFactor));
        data.height = static_cast<int>(baseVisualHeight_ * std::sqrt(importanceFactor));
        
        // Allocate color and depth buffers
        data.colorData.resize(data.width * data.height);
        data.depthData.resize(data.width * data.height);
        
        // Field of view and aspect ratio
        data.fieldOfView = baseFieldOfView_;
        data.aspectRatio = static_cast<T>(data.width) / data.height;
        
        // In a real implementation, this would render the scene
        // Here we just fill with placeholder data
        fillPlaceholderVisualData(data, position, direction);
        
        // Remember last visual data
        lastVisualData_ = data;
        
        return data;
    }
    
    /**
     * @brief Update audio sensory data
     * 
     * @param position Listener position
     * @param resources Computational resources to allocate (0.0-1.0)
     * @return AudioData Updated audio data
     */
    AudioData updateAudioSensory(
        const VectorT& position,
        T resources = 1.0) {
        
        AudioData data;
        
        // Calculate resources to allocate
        T importanceFactor = sensoryImportance_["audio"] * resources;
        data.importance = importanceFactor;
        
        // Scale audio quality based on resources
        data.sampleRate = static_cast<int>(baseSampleRate_ * importanceFactor);
        data.channels = baseAudioChannels_;
        
        // Determine number of samples based on resource allocation
        int sampleCount = static_cast<int>(baseAudioSamples_ * importanceFactor);
        data.audioSamples.resize(sampleCount * data.channels);
        
        // In a real implementation, this would process audio
        // Here we just fill with placeholder data
        fillPlaceholderAudioData(data, position);
        
        // Remember last audio data
        lastAudioData_ = data;
        
        return data;
    }
    
    /**
     * @brief Update haptic sensory data
     * 
     * @param bodyPoints Vector of body points to check for haptic contact
     * @param resources Computational resources to allocate (0.0-1.0)
     * @return HapticData Updated haptic data
     */
    HapticData updateHapticSensory(
        const std::vector<VectorT>& bodyPoints,
        T resources = 1.0) {
        
        HapticData data;
        
        // Calculate resources to allocate
        T importanceFactor = sensoryImportance_["haptic"] * resources;
        data.importance = importanceFactor;
        
        // Allocate contact points based on resources
        int maxContacts = static_cast<int>(baseMaxContacts_ * importanceFactor);
        
        // In a real implementation, this would detect collisions
        // Here we just fill with placeholder data
        fillPlaceholderHapticData(data, bodyPoints, maxContacts);
        
        // Remember last haptic data
        lastHapticData_ = data;
        
        return data;
    }
    
    /**
     * @brief Update proprioception sensory data
     * 
     * @param jointPositions Current joint positions
     * @param jointVelocities Current joint velocities
     * @param resources Computational resources to allocate (0.0-1.0)
     * @return ProprioceptionData Updated proprioception data
     */
    ProprioceptionData updateProprioceptionSensory(
        const std::vector<T>& jointPositions,
        const std::vector<T>& jointVelocities,
        T resources = 1.0) {
        
        ProprioceptionData data;
        
        // Calculate resources to allocate
        T importanceFactor = sensoryImportance_["proprioception"] * resources;
        data.importance = importanceFactor;
        
        // Copy input data
        data.jointPositions = jointPositions;
        data.jointVelocities = jointVelocities;
        
        // Allocate space for joint accelerations and muscle activation
        data.jointAccelerations.resize(jointPositions.size());
        data.muscleActivation.resize(jointPositions.size());
        
        // Generate joint names if not provided
        data.jointNames.resize(jointPositions.size());
        for (size_t i = 0; i < data.jointNames.size(); ++i) {
            data.jointNames[i] = "Joint_" + std::to_string(i);
        }
        
        // Calcuate joint accelerations based on current and previous velocities
        if (!lastProprioceptionData_.jointVelocities.empty() && 
            lastProprioceptionData_.jointVelocities.size() == jointVelocities.size()) {
            
            for (size_t i = 0; i < jointVelocities.size(); ++i) {
                data.jointAccelerations[i] = jointVelocities[i] - lastProprioceptionData_.jointVelocities[i];
            }
        }
        
        // Calculate center of mass and balance
        calculateCenterOfMassAndBalance(data);
        
        // Remember last proprioception data
        lastProprioceptionData_ = data;
        
        return data;
    }
    
    /**
     * @brief Set sensory importance values
     * 
     * @param visual Visual importance (0.0-1.0)
     * @param audio Audio importance (0.0-1.0)
     * @param haptic Haptic importance (0.0-1.0)
     * @param proprioception Proprioception importance (0.0-1.0)
     */
    void setSensoryImportance(T visual, T audio, T haptic, T proprioception) {
        // Calculate normalization factor
        T sum = visual + audio + haptic + proprioception;
        T normFactor = sum > 0 ? 1.0 / sum : 1.0;
        
        // Set normalized importance values
        sensoryImportance_["visual"] = visual * normFactor;
        sensoryImportance_["audio"] = audio * normFactor;
        sensoryImportance_["haptic"] = haptic * normFactor;
        sensoryImportance_["proprioception"] = proprioception * normFactor;
    }
    
    /**
     * @brief Set base visual parameters
     * 
     * @param width Base width resolution
     * @param height Base height resolution
     * @param fov Field of view in degrees
     */
    void setBaseVisualParams(int width, int height, T fov) {
        baseVisualWidth_ = width;
        baseVisualHeight_ = height;
        baseFieldOfView_ = fov;
    }
    
    /**
     * @brief Set base audio parameters
     * 
     * @param sampleRate Base sample rate
     * @param channels Number of audio channels
     * @param samples Base number of samples
     */
    void setBaseAudioParams(int sampleRate, int channels, int samples) {
        baseSampleRate_ = sampleRate;
        baseAudioChannels_ = channels;
        baseAudioSamples_ = samples;
    }
    
    /**
     * @brief Set base haptic parameters
     * 
     * @param maxContacts Maximum contact points
     */
    void setBaseHapticParams(int maxContacts) {
        baseMaxContacts_ = maxContacts;
    }

private:
    // Environment reference
    std::shared_ptr<TreeEnv> environment_;
    
    // Sensory importance allocation
    std::unordered_map<std::string, T> sensoryImportance_;
    
    // Last sensory data
    VisualData lastVisualData_;
    AudioData lastAudioData_;
    HapticData lastHapticData_;
    ProprioceptionData lastProprioceptionData_;
    
    // Base parameters for visual system
    int baseVisualWidth_ = 1920;
    int baseVisualHeight_ = 1080;
    T baseFieldOfView_ = 60.0;
    
    // Base parameters for audio system
    int baseSampleRate_ = 44100;
    int baseAudioChannels_ = 2;
    int baseAudioSamples_ = 4096;
    
    // Base parameters for haptic system
    int baseMaxContacts_ = 64;
    
    // Helper methods for placeholder data
    void fillPlaceholderVisualData(VisualData& data, const VectorT& position, const VectorT& direction) {
        // Simple placeholder: fill with gradient based on direction
        for (int y = 0; y < data.height; ++y) {
            for (int x = 0; x < data.width; ++x) {
                T nx = (x / static_cast<T>(data.width)) * 2.0 - 1.0;
                T ny = (y / static_cast<T>(data.height)) * 2.0 - 1.0;
                
                T dot = nx * direction[0] + ny * direction[1];
                T intensity = (dot + 1.0) * 0.5;
                
                int idx = y * data.width + x;
                data.colorData[idx] = {intensity, intensity, intensity, 1.0};
                data.depthData[idx] = 1.0 - intensity;
            }
        }
    }
    
    void fillPlaceholderAudioData(AudioData& data, const VectorT& position) {
        // Generate simple sine wave as placeholder
        for (int i = 0; i < data.audioSamples.size(); ++i) {
            T t = static_cast<T>(i) / data.sampleRate;
            data.audioSamples[i] = std::sin(t * 440.0 * 2.0 * 3.14159);
        }
        
        // Add a couple of audio sources
        data.sourcePositions = {
            {position[0] + 1.0, position[1], position[2]},
            {position[0], position[1] + 1.0, position[2]},
        };
        
        data.sourceImportance = {0.8, 0.6};
    }
    
    void fillPlaceholderHapticData(HapticData& data, 
                                  const std::vector<VectorT>& bodyPoints,
                                  int maxContacts) {
        // Choose a subset of body points as contacts
        int numContacts = std::min(static_cast<int>(bodyPoints.size()), maxContacts);
        
        for (int i = 0; i < numContacts; ++i) {
            data.contactPoints.push_back(bodyPoints[i]);
            
            // Generate random force vectors
            VectorT force;
            for (int d = 0; d < Dimensions; ++d) {
                force[d] = (std::sin(bodyPoints[i][d] * 10.0) + 1.0) * 0.5;
            }
            data.contactForces.push_back(force);
            
            // Generate random pressure, temperature and texture
            data.pressure.push_back((std::cos(bodyPoints[i][0] * 5.0) + 1.0) * 0.5);
            data.temperature.push_back((std::sin(bodyPoints[i][1] * 3.0) + 1.0) * 0.5 + 0.5);
            data.texture.push_back((std::cos(bodyPoints[i][2] * 7.0) + 1.0) * 0.5);
        }
    }
    
    void calculateCenterOfMassAndBalance(ProprioceptionData& data) {
        // Simple placeholder: average joint positions as COM
        if (data.jointPositions.size() >= 3 * Dimensions) {
            // Assume jointPositions are stored as [x1, y1, z1, x2, y2, z2, ...]
            data.centerOfMass.fill(0);
            int numJoints = data.jointPositions.size() / Dimensions;
            
            for (int i = 0; i < numJoints; ++i) {
                for (int d = 0; d < Dimensions; ++d) {
                    data.centerOfMass[d] += data.jointPositions[i * Dimensions + d] / numJoints;
                }
            }
            
            // Simple balance metric: how far COM is from "down" axis
            T comLength = 0;
            for (int d = 0; d < Dimensions; ++d) {
                comLength += data.centerOfMass[d] * data.centerOfMass[d];
            }
            comLength = std::sqrt(comLength);
            
            // Assume "up" is the Y axis (index 1)
            T verticalComponent = data.centerOfMass[1] / (comLength > 0 ? comLength : 1);
            
            // Balance is 1.0 when COM is directly above feet, 0.0 when far off
            data.balance = (verticalComponent + 1.0) * 0.5;
        } else {
            // Default values
            data.centerOfMass.fill(0);
            data.balance = 1.0;
        }
    }
};

} // namespace sdk
} // namespace physics

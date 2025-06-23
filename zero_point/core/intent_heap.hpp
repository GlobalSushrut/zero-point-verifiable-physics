#pragma once

#include <vector>
#include <queue>
#include <mutex>
#include <chrono>
#include <memory>
#include <algorithm>
#include <cstdint>

namespace zero_point {
namespace core {

/**
 * @brief Represents a node in the intent priority system
 * 
 * IntentNodes capture computational priority for different sensory inputs
 * and regions of interest, allowing the system to allocate resources based
 * on attention and importance.
 */
struct IntentNode {
    float priority;               // 0.0 (lowest) to 1.0 (highest)
    uint8_t sensor_id;            // ID of the sensor that generated this intent
    uint32_t entity_id;           // ID of the entity this intent is associated with
    uint64_t timestamp;           // When this intent was created
    bool requires_reflex;         // Whether this intent requires immediate processing
    float spatial_weight;         // Weight in spatial domain
    float temporal_weight;        // Weight in time domain
    float energy_cost;            // Computational cost of processing this intent
    
    // Constructor with defaults
    IntentNode(float p = 0.0f, uint8_t id = 0, uint32_t entity = 0, bool reflex = false,
               float spatial = 1.0f, float temporal = 1.0f, float cost = 1.0f)
        : priority(p), sensor_id(id), entity_id(entity), requires_reflex(reflex),
          spatial_weight(spatial), temporal_weight(temporal), energy_cost(cost) {
        timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
    // Compare based on priority
    bool operator<(const IntentNode& other) const {
        if (requires_reflex != other.requires_reflex) {
            return requires_reflex;  // Reflex intents are always higher priority
        }
        return priority > other.priority;  // Higher priority comes first
    }
};

/**
 * @brief Thread-safe priority queue for processing intents
 * 
 * The IntentHeap is the central prioritization mechanism that enables
 * computational focus on the most important aspects of simulation,
 * similar to how human attention works.
 */
class IntentHeap {
public:
    IntentHeap(size_t max_size = 1024);
    ~IntentHeap() = default;
    
    /**
     * @brief Add a new intent to the heap
     * @param intent The intent to add
     */
    void push(const IntentNode& intent);
    
    /**
     * @brief Get the highest priority intent without removing it
     * @return Reference to the top intent or nullptr if empty
     */
    std::shared_ptr<IntentNode> peek();
    
    /**
     * @brief Get and remove the highest priority intent
     * @return The highest priority intent
     */
    IntentNode pop();
    
    /**
     * @brief Apply time-based decay to all intents
     * @param decay_factor Factor to decay by (0.9 = 10% decay)
     */
    void decay(float decay_factor = 0.95f);
    
    /**
     * @brief Get the current size of the heap
     * @return Number of items in the heap
     */
    size_t size() const;
    
    /**
     * @brief Check if the heap is empty
     * @return True if empty, false otherwise
     */
    bool empty() const;
    
    /**
     * @brief Clear all intents from the heap
     */
    void clear();
    
    /**
     * @brief Update the priority of a specific sensor's intents
     * @param sensor_id The sensor ID to update
     * @param priority_delta Change in priority (-1.0 to 1.0)
     */
    void updateSensorPriority(uint8_t sensor_id, float priority_delta);
    
    /**
     * @brief Set energy budget for priority calculations
     * @param budget Available energy (computational resources)
     */
    void setEnergyBudget(float budget);
    
private:
    std::priority_queue<IntentNode> heap_;
    size_t max_size_;
    mutable std::mutex mutex_;
    float energy_budget_ = 1.0f;  // Default full energy
    
    // Rebalance priorities based on energy budget
    void rebalance_();
};

} // namespace core
} // namespace zero_point

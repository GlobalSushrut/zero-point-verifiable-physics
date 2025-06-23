#include "intent_heap.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace zero_point {
namespace core {

IntentHeap::IntentHeap(size_t max_size) : max_size_(max_size) {
    // Reserve space for efficiency
    heap_ = std::priority_queue<IntentNode>();
}

void IntentHeap::push(const IntentNode& intent) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Apply energy budget adjustments before inserting
    IntentNode adjusted_intent = intent;
    
    // Adjust priority based on energy constraints
    float energy_factor = std::min(energy_budget_ / adjusted_intent.energy_cost, 1.0f);
    adjusted_intent.priority *= energy_factor;
    
    // Don't let the heap grow beyond max size
    if (heap_.size() >= max_size_) {
        // For efficiency, we don't rebalance the entire heap
        // Instead, we check if this new intent deserves to be in the heap
        
        // Create a temporary copy of the top (lowest priority) element
        IntentNode lowest = heap_.top();
        
        if (adjusted_intent.priority > lowest.priority || adjusted_intent.requires_reflex) {
            // Pop the lowest priority element
            heap_.pop();
            // Add the new higher priority element
            heap_.push(adjusted_intent);
            
            // Rebalance the heap if energy constrained
            if (energy_budget_ < 0.8f) {
                rebalance_();
            }
        }
    } else {
        // Still have space, just add it
        heap_.push(adjusted_intent);
    }
}

std::shared_ptr<IntentNode> IntentHeap::peek() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (heap_.empty()) {
        return nullptr;
    }
    
    return std::make_shared<IntentNode>(heap_.top());
}

IntentNode IntentHeap::pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (heap_.empty()) {
        return IntentNode(0.0f);  // Return a zero-priority intent if empty
    }
    
    IntentNode top = heap_.top();
    heap_.pop();
    return top;
}

void IntentHeap::decay(float decay_factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // We need to rebuild the heap with decayed priorities
    std::vector<IntentNode> nodes;
    nodes.reserve(heap_.size());
    
    // Extract all nodes
    while (!heap_.empty()) {
        IntentNode node = heap_.top();
        heap_.pop();
        
        // Apply decay to non-reflex intents
        if (!node.requires_reflex) {
            node.priority *= decay_factor;
            
            // Apply temporal decay based on age
            uint64_t current_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            uint64_t age_us = current_time - node.timestamp;
            float age_factor = std::exp(-static_cast<float>(age_us) / 1000000.0f / node.temporal_weight);
            node.priority *= age_factor;
        }
        
        // Only keep nodes with reasonable priority
        if (node.priority > 0.01f) {
            nodes.push_back(node);
        }
    }
    
    // Rebuild heap with decayed priorities
    for (const auto& node : nodes) {
        heap_.push(node);
    }
}

size_t IntentHeap::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return heap_.size();
}

bool IntentHeap::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return heap_.empty();
}

void IntentHeap::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!heap_.empty()) {
        heap_.pop();
    }
}

void IntentHeap::updateSensorPriority(uint8_t sensor_id, float priority_delta) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // We need to rebuild the heap to adjust priorities
    std::vector<IntentNode> nodes;
    nodes.reserve(heap_.size());
    
    // Extract all nodes
    while (!heap_.empty()) {
        IntentNode node = heap_.top();
        heap_.pop();
        
        // Adjust priority for matching sensor_id
        if (node.sensor_id == sensor_id) {
            node.priority += priority_delta;
            // Clamp priority between 0 and 1
            node.priority = std::max(0.0f, std::min(1.0f, node.priority));
        }
        
        nodes.push_back(node);
    }
    
    // Rebuild heap with adjusted priorities
    for (const auto& node : nodes) {
        heap_.push(node);
    }
}

void IntentHeap::setEnergyBudget(float budget) {
    std::lock_guard<std::mutex> lock(mutex_);
    energy_budget_ = std::max(0.1f, std::min(1.0f, budget));  // Clamp between 0.1 and 1.0
    
    // Rebalance the entire heap when energy budget changes significantly
    rebalance_();
}

void IntentHeap::rebalance_() {
    // Only rebalance if we have nodes and limited energy
    if (heap_.empty() || energy_budget_ >= 0.99f) {
        return;
    }
    
    // Extract all nodes
    std::vector<IntentNode> nodes;
    nodes.reserve(heap_.size());
    
    while (!heap_.empty()) {
        nodes.push_back(heap_.top());
        heap_.pop();
    }
    
    // Sort by priority (highest first)
    std::sort(nodes.begin(), nodes.end(), [](const IntentNode& a, const IntentNode& b) {
        return a.priority > b.priority;
    });
    
    // Calculate total energy required
    float total_energy_required = 0.0f;
    for (const auto& node : nodes) {
        total_energy_required += node.energy_cost;
    }
    
    // Adjust priorities based on energy budget
    float energy_ratio = energy_budget_ / total_energy_required;
    if (energy_ratio < 1.0f) {
        float priority_scale = std::pow(energy_ratio, 0.7f); // Non-linear scaling
        
        // Adjust non-reflex nodes
        for (auto& node : nodes) {
            if (!node.requires_reflex) {
                node.priority *= priority_scale;
            }
        }
    }
    
    // Rebuild the heap with adjusted priorities
    for (const auto& node : nodes) {
        heap_.push(node);
    }
}

} // namespace core
} // namespace zero_point

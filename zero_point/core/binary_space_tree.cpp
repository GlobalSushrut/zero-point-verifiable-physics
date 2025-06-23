#include "binary_space_tree.hpp"
#include <queue>
#include <limits>
#include <iostream>

namespace zero_point {
namespace core {

BinarySpaceTree::BinarySpaceTree() 
    : maxDepth_(8), 
      energyBudget_(1.0f), 
      nextNodeId_(1), 
      simulationTime_(0.0f) {
    
    // Set default curvature function (flat space)
    curvatureFunc_ = [](const Vec3& pos) -> float {
        return 0.0f;
    };
    
    // Set default intent function (uniform)
    intentFunc_ = [](const Vec3& pos) -> float {
        return 0.5f;
    };
    
    // Create root node at origin
    root_ = std::make_shared<Node3D>(Vec3(0.0f, 0.0f, 0.0f), 0.0f);
    root_->id = nextNodeId_++;
    nodeMap_[root_->id] = root_;
}

bool BinarySpaceTree::initialize(int maxDepth) {
    std::lock_guard<std::mutex> lock(mutex_);
    maxDepth_ = maxDepth;
    
    // Reset tree to just the root
    root_ = std::make_shared<Node3D>(Vec3(0.0f, 0.0f, 0.0f), 0.0f);
    root_->id = nextNodeId_++;
    nodeMap_.clear();
    nodeMap_[root_->id] = root_;
    simulationTime_ = 0.0f;
    
    return true;
}

void BinarySpaceTree::setCurvatureFunction(const CurvatureFunction& func) {
    std::lock_guard<std::mutex> lock(mutex_);
    curvatureFunc_ = func;
    
    // Apply curvature function to all existing nodes
    if (root_) {
        root_->updateCurvature(curvatureFunc_);
    }
}

void BinarySpaceTree::setIntentFunction(const IntentFunction& func) {
    std::lock_guard<std::mutex> lock(mutex_);
    intentFunc_ = func;
    
    // Apply intent function to all existing nodes
    if (root_) {
        std::queue<std::shared_ptr<Node3D>> queue;
        queue.push(root_);
        
        while (!queue.empty()) {
            auto node = queue.front();
            queue.pop();
            
            // Update intent priority
            node->intent_priority = intentFunc_(node->position);
            
            // Add children to queue
            for (const auto& child : node->children) {
                queue.push(child);
            }
        }
    }
}

void BinarySpaceTree::integrateFrame(float dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Skip if energy budget is too low
    if (energyBudget_ < 0.1f) {
        return;
    }
    
    // Scale dt by energy budget for adaptive time stepping
    float effective_dt = dt * std::min(1.0f, energyBudget_);
    
    // Process nodes in order of intent priority if energy is limited
    if (energyBudget_ < 0.9f) {
        std::vector<std::shared_ptr<Node3D>> all_nodes;
        all_nodes.reserve(nodeMap_.size());
        
        for (const auto& pair : nodeMap_) {
            all_nodes.push_back(pair.second);
        }
        
        // Sort nodes by intent priority
        std::sort(all_nodes.begin(), all_nodes.end(), 
            [](const std::shared_ptr<Node3D>& a, const std::shared_ptr<Node3D>& b) {
                return a->intent_priority > b->intent_priority;
            });
        
        // Calculate physics for high priority nodes first
        size_t node_budget = static_cast<size_t>(all_nodes.size() * energyBudget_);
        for (size_t i = 0; i < node_budget && i < all_nodes.size(); ++i) {
            auto& node = all_nodes[i];
            
            // Skip non-dynamic nodes
            if (!node->is_dynamic) continue;
            
            // Apply gravity from all other nodes
            for (size_t j = 0; j < all_nodes.size(); ++j) {
                if (i == j) continue;
                calculateForces_(node, all_nodes[j], effective_dt);
            }
            
            // Apply curvature effects
            float curvature = curvatureFunc_(node->position);
            node->curvature = curvature;
            
            // Apply curvature-based acceleration
            if (curvature != 0.0f) {
                Vec3 curv_accel = node->position.normalized() * (-curvature * 9.8f);
                node->velocity += curv_accel * effective_dt;
            }
            
            // Integrate position
            node->integrate(effective_dt);
        }
    } else {
        // Full energy, process all nodes
        for (const auto& pair : nodeMap_) {
            auto& node = pair.second;
            
            // Skip non-dynamic nodes
            if (!node->is_dynamic) continue;
            
            // Apply gravity from all other nodes
            for (const auto& other_pair : nodeMap_) {
                if (pair.first == other_pair.first) continue;
                calculateForces_(node, other_pair.second, effective_dt);
            }
            
            // Apply curvature effects
            float curvature = curvatureFunc_(node->position);
            node->curvature = curvature;
            
            // Apply curvature-based acceleration
            if (curvature != 0.0f) {
                Vec3 curv_accel = node->position.normalized() * (-curvature * 9.8f);
                node->velocity += curv_accel * effective_dt;
            }
            
            // Integrate position
            node->integrate(effective_dt);
        }
    }
    
    // Update simulation time
    simulationTime_ += effective_dt;
}

void BinarySpaceTree::applyCurvature(const Vec3& observer) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Calculate observer-relative curvature for all nodes
    for (const auto& pair : nodeMap_) {
        auto& node = pair.second;
        
        // Calculate direction to observer
        Vec3 dir = node->position - observer;
        float dist = dir.length();
        
        // Skip if too close to observer
        if (dist < 0.001f) continue;
        
        // Normalize direction
        dir = dir / dist;
        
        // Get base curvature at this position
        float base_curvature = curvatureFunc_(node->position);
        
        // Apply observer-relative curvature (stronger when closer)
        float rel_curvature = base_curvature * (1.0f + 10.0f / (dist * dist + 1.0f));
        
        node->curvature = rel_curvature;
        
        // Non-Euclidean velocity adjustment
        if (node->is_dynamic && rel_curvature != 0.0f) {
            // Calculate geodesic correction to velocity
            // This is a simplified approximation of parallel transport
            float vel_mag = node->velocity.length();
            if (vel_mag > 0.001f) {
                Vec3 vel_dir = node->velocity / vel_mag;
                float alignment = vel_dir.dot(dir);
                
                // Adjust velocity based on curvature
                Vec3 curl = dir.cross(vel_dir);
                float curl_mag = curl.length();
                if (curl_mag > 0.001f) {
                    curl = curl / curl_mag;
                    node->velocity += curl * (rel_curvature * vel_mag * std::abs(alignment) * 0.1f);
                }
            }
        }
    }
}

std::vector<std::shared_ptr<Node3D>> BinarySpaceTree::renderLOD(const Vec3& focus_point, 
                                                               size_t max_nodes) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<Node3D>> result;
    
    // Apply energy budget to max_nodes
    size_t adjusted_max = static_cast<size_t>(max_nodes * energyBudget_);
    
    // Always ensure some minimum nodes
    adjusted_max = std::max(size_t(100), adjusted_max);
    
    size_t remaining_budget = adjusted_max;
    findLODNodes_(root_, focus_point, result, remaining_budget, 0);
    
    return result;
}

uint32_t BinarySpaceTree::addNode(const std::shared_ptr<Node3D>& node) {
    if (!node) return 0;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Assign ID if not already assigned
    if (node->id == 0) {
        node->id = nextNodeId_++;
    }
    
    // Add to map
    nodeMap_[node->id] = node;
    
    // Insert into tree
    insertNode_(root_, node, 0);
    
    return node->id;
}

std::shared_ptr<Node3D> BinarySpaceTree::findNode(uint32_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = nodeMap_.find(id);
    if (it != nodeMap_.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<Node3D> BinarySpaceTree::findNearestNode(const Vec3& position, float max_distance) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::shared_ptr<Node3D> nearest = nullptr;
    float nearest_dist_squared = max_distance * max_distance;
    
    for (const auto& pair : nodeMap_) {
        float dist_squared = Vec3::distanceSquared(position, pair.second->position);
        if (dist_squared < nearest_dist_squared) {
            nearest_dist_squared = dist_squared;
            nearest = pair.second;
        }
    }
    
    return nearest;
}

std::vector<std::shared_ptr<Node3D>> BinarySpaceTree::findNodesInRadius(
    const Vec3& position, float radius) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<Node3D>> result;
    findNodesInRadius_(root_, position, radius, result, 0);
    return result;
}

void BinarySpaceTree::setEnergyBudget(float energy_budget) {
    std::lock_guard<std::mutex> lock(mutex_);
    energyBudget_ = std::max(0.1f, std::min(1.0f, energy_budget));
}

size_t BinarySpaceTree::getNodeCount() const {
    return nodeMap_.size();
}

std::shared_ptr<Node3D> BinarySpaceTree::getRandomNode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If no nodes, return nullptr
    if (nodeMap_.empty()) {
        return nullptr;
    }
    
    // Generate random index
    size_t index = std::rand() % nodeMap_.size();
    
    // Iterate to that index
    auto it = nodeMap_.begin();
    std::advance(it, index);
    
    return it->second;
}

int BinarySpaceTree::getMaxDepth() const {
    return maxDepth_;
}

BinarySpaceTree::IntentFunction BinarySpaceTree::getIntentFunction() const {
    return intentFunc_;
}

float BinarySpaceTree::getRootCurvature() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (root_) {
        return root_->curvature;
    }
    return 0.0f;
}

void BinarySpaceTree::insertNode_(std::shared_ptr<Node3D> parent, 
                                 const std::shared_ptr<Node3D>& node,
                                 int depth) {
    if (!parent || !node || depth > maxDepth_) return;
    
    // Calculate direction from parent to node
    Vec3 dir = node->position - parent->position;
    float dist = dir.length();
    
    // If too close, make child of parent
    if (dist < 0.001f || depth == maxDepth_) {
        parent->addChild(node);
        return;
    }
    
    // Check if we should go deeper
    if (!parent->isLeaf()) {
        // Find best child to descend to
        std::shared_ptr<Node3D> best_child = nullptr;
        float best_score = std::numeric_limits<float>::max();
        
        for (const auto& child : parent->children) {
            Vec3 child_dir = child->position - parent->position;
            float direction_score = 1.0f - dir.normalized().dot(child_dir.normalized());
            float distance_score = Vec3::distance(node->position, child->position);
            float total_score = direction_score + distance_score;
            
            if (total_score < best_score) {
                best_score = total_score;
                best_child = child;
            }
        }
        
        // Only descend if we have a good match
        if (best_child && best_score < 1.0f) {
            insertNode_(best_child, node, depth + 1);
            return;
        }
    }
    
    // Otherwise, add as child of parent
    parent->addChild(node);
}

void BinarySpaceTree::findLODNodes_(std::shared_ptr<Node3D> node,
                                   const Vec3& focus_point,
                                   std::vector<std::shared_ptr<Node3D>>& result,
                                   size_t& remaining_budget,
                                   int depth) {
    if (!node || remaining_budget == 0 || depth > maxDepth_) return;
    
    // Calculate distance to focus point
    float distance = Vec3::distance(node->position, focus_point);
    
    // Higher priority nodes are always included
    bool include_node = (node->intent_priority > 0.8f);
    
    // Include node if close enough or if a leaf
    include_node |= (distance < 10.0f * (maxDepth_ - depth) || node->isLeaf());
    
    // Also include based on level of detail threshold
    float lod_threshold = 5.0f * std::pow(2.0f, depth);
    include_node |= (distance < lod_threshold);
    
    if (include_node && remaining_budget > 0) {
        result.push_back(node);
        remaining_budget--;
    }
    
    // Early exit if no budget left
    if (remaining_budget == 0) return;
    
    // Get children sorted by distance to focus point
    std::vector<std::pair<float, std::shared_ptr<Node3D>>> sorted_children;
    for (const auto& child : node->children) {
        float child_dist = Vec3::distance(child->position, focus_point);
        sorted_children.push_back({child_dist, child});
    }
    
    // Sort by distance (closest first)
    std::sort(sorted_children.begin(), sorted_children.end(),
        [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    
    // Process children
    for (const auto& pair : sorted_children) {
        findLODNodes_(pair.second, focus_point, result, remaining_budget, depth + 1);
        if (remaining_budget == 0) break;
    }
}

void BinarySpaceTree::calculateForces_(std::shared_ptr<Node3D> node1,
                                      std::shared_ptr<Node3D> node2,
                                      float dt) {
    if (!node1 || !node2 || !node1->is_dynamic) return;
    
    // Calculate direction and distance
    Vec3 dir = node2->position - node1->position;
    float dist_squared = dir.lengthSquared();
    
    // Skip if too close or too far
    if (dist_squared < 0.0001f || dist_squared > 1000.0f) return;
    
    float dist = std::sqrt(dist_squared);
    dir = dir / dist;  // Normalize
    
    // Calculate gravitational force (simplified)
    const float G = 6.67430e-2f;  // Scaled gravitational constant
    float force_mag = G * node1->mass * node2->mass / dist_squared;
    
    // Apply scaled force as acceleration
    Vec3 force = dir * force_mag;
    node1->applyForce(force, dt);
    
    // Apply non-Euclidean correction based on curvature
    float avg_curvature = (node1->curvature + node2->curvature) * 0.5f;
    if (std::abs(avg_curvature) > 0.001f) {
        // Calculate geodesic correction
        Vec3 correction = dir.cross(node1->velocity.cross(dir)) * avg_curvature * 0.1f;
        node1->velocity += correction * dt;
    }
}

void BinarySpaceTree::findNodesInRadius_(std::shared_ptr<Node3D> node,
                                        const Vec3& position,
                                        float radius,
                                        std::vector<std::shared_ptr<Node3D>>& result,
                                        int depth) {
    if (!node || depth > maxDepth_) return;
    
    // Check if this node is within radius
    float dist = Vec3::distance(node->position, position);
    if (dist <= radius) {
        result.push_back(node);
    }
    
    // Check if we need to search children
    bool search_children = true;
    
    // Optimization: if distance > radius + max_child_distance, skip children
    if (node->children.size() > 0) {
        float max_child_distance = 0.0f;
        for (const auto& child : node->children) {
            float child_dist = Vec3::distance(child->position, node->position);
            max_child_distance = std::max(max_child_distance, child_dist);
        }
        
        search_children = (dist <= radius + max_child_distance);
    }
    
    // Recursively search children if needed
    if (search_children) {
        for (const auto& child : node->children) {
            findNodesInRadius_(child, position, radius, result, depth + 1);
        }
    }
}

} // namespace core
} // namespace zero_point

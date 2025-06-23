#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <cmath>
#include <array>

namespace physics {
namespace sdk {

/**
 * @brief Tree Binary Node for efficient spatial representation
 * 
 * Implements a tree-based structure optimized for non-Euclidean physics
 * and intent-driven computation allocation.
 */
template <typename T, int Dimensions = 3>
class TreeBinaryNode : public std::enable_shared_from_this<TreeBinaryNode<T, Dimensions>> {
public:
    using VectorT = std::array<T, Dimensions>;
    using NodePtr = std::shared_ptr<TreeBinaryNode>;
    
    // Core node data
    VectorT position;
    T value;
    T intentPriority;
    
    // Tree structure
    NodePtr parent;
    std::vector<NodePtr> children;
    int depth;
    
    // Non-Euclidean data
    T curvature;
    std::vector<std::pair<int, T>> connections;
    
    TreeBinaryNode() : value(0), intentPriority(1.0), depth(0), curvature(0) {
        position.fill(0);
    }
    
    TreeBinaryNode(const VectorT& pos, T val = 0) 
        : position(pos), value(val), intentPriority(1.0), depth(0), curvature(0) {}
    
    /**
     * @brief Add a child node
     * 
     * @param child Child node to add
     */
    void addChild(const NodePtr& child) {
        children.push_back(child);
        child->parent = std::dynamic_pointer_cast<TreeBinaryNode>(this->shared_from_this());
        child->depth = depth + 1;
    }
    
    /**
     * @brief Split this node into children based on octree/quadtree pattern
     * 
     * @param subdivisions Number of subdivisions per dimension
     * @return std::vector<NodePtr> Newly created children
     */
    std::vector<NodePtr> split(int subdivisions = 2) {
        std::vector<NodePtr> newChildren;
        
        // Create 2^Dimensions children (octree in 3D, quadtree in 2D)
        for (int i = 0; i < std::pow(subdivisions, Dimensions); ++i) {
            VectorT childPos = position;
            
            // Calculate child position offset
            for (int d = 0; d < Dimensions; ++d) {
                int bit = (i >> d) & 1;
                T offset = bit ? T(1.0) / std::pow(2, depth + 1) : -T(1.0) / std::pow(2, depth + 1);
                childPos[d] += offset;
            }
            
            // Create child with inherited properties
            auto child = std::make_shared<TreeBinaryNode>(childPos, value);
            child->curvature = this->curvature;
            child->intentPriority = this->intentPriority;
            
            addChild(child);
            newChildren.push_back(child);
        }
        
        return newChildren;
    }
    
    /**
     * @brief Find the node containing the given position
     * 
     * @param pos Position to find
     * @param maxDepth Maximum depth to search
     * @return NodePtr Node containing position, or nullptr if not found
     */
    NodePtr findNode(const VectorT& pos, int maxDepth = -1) {
        if (maxDepth == 0) return this->shared_from_this();
        
        // Check if position is in this node's region
        bool inRegion = true;
        T nodeSize = T(1.0) / std::pow(2, depth);
        
        for (int d = 0; d < Dimensions; ++d) {
            if (std::abs(pos[d] - position[d]) > nodeSize) {
                inRegion = false;
                break;
            }
        }
        
        if (!inRegion) return nullptr;
        
        // Check children recursively
        for (const auto& child : children) {
            NodePtr foundNode = child->findNode(pos, maxDepth > 0 ? maxDepth - 1 : maxDepth);
            if (foundNode) return foundNode;
        }
        
        // If we got here, position is in this node but not in any child
        return this->shared_from_this();
    }
    
    /**
     * @brief Update intent priority based on importance function
     * 
     * @param importanceFunc Function that calculates importance value
     */
    void updateIntent(std::function<T(const VectorT&)> importanceFunc) {
        intentPriority = importanceFunc(position);
        
        // Recursively update children with diminishing effect
        for (auto& child : children) {
            child->intentPriority = 0.8 * intentPriority + 0.2 * importanceFunc(child->position);
            child->updateIntent(importanceFunc);
        }
    }
    
    /**
     * @brief Calculate non-Euclidean distance to another node
     * 
     * @param other Other node
     * @return T Non-Euclidean distance
     */
    T nonEuclideanDistance(const NodePtr& other) const {
        // Start with Euclidean distance
        T euclideanDist = 0;
        for (int d = 0; d < Dimensions; ++d) {
            T diff = position[d] - other->position[d];
            euclideanDist += diff * diff;
        }
        euclideanDist = std::sqrt(euclideanDist);
        
        // Apply curvature correction
        T avgCurvature = (curvature + other->curvature) / 2;
        if (std::abs(avgCurvature) < 1e-6) return euclideanDist;
        
        // Simple model for curved space distance
        if (avgCurvature > 0) {
            // Positive curvature - spherical-like
            return T(2.0) * std::asin(std::min(T(1.0), euclideanDist / (T(2.0) / std::sqrt(avgCurvature))));
        } else {
            // Negative curvature - hyperbolic-like
            avgCurvature = -avgCurvature;
            return T(2.0) * std::asinh(euclideanDist * std::sqrt(avgCurvature) / T(2.0));
        }
    }
};

/**
 * @brief Intent Heap Binary for prioritizing computations
 * 
 * Maintains a priority queue of computation targets based on intent values.
 */
template <typename T, int Dimensions = 3>
class IntentHeapBinary {
public:
    using VectorT = std::array<T, Dimensions>;
    using TreeNode = TreeBinaryNode<T, Dimensions>;
    using TreeNodePtr = std::shared_ptr<TreeNode>;
    
    // Entry in the intent heap
    struct IntentEntry {
        TreeNodePtr node;
        T intentValue;
        int priority;
        
        bool operator<(const IntentEntry& other) const {
            return intentValue < other.intentValue;
        }
    };
    
    IntentHeapBinary() : totalResources(1.0) {}
    
    /**
     * @brief Add a node to the intent heap
     * 
     * @param node Node to add
     * @param intent Intent priority value
     */
    void addNode(const TreeNodePtr& node, T intent = -1) {
        if (intent < 0) intent = node->intentPriority;
        
        IntentEntry entry;
        entry.node = node;
        entry.intentValue = intent;
        entry.priority = entries.size();
        
        entries.push_back(entry);
        std::push_heap(entries.begin(), entries.end());
    }
    
    /**
     * @brief Get the highest intent nodes up to resource limit
     * 
     * @param resourceLimit Total resources to allocate (default: use all)
     * @return std::vector<TreeNodePtr> Nodes to process
     */
    std::vector<TreeNodePtr> getHighPriorityNodes(T resourceLimit = -1) {
        if (resourceLimit < 0) resourceLimit = totalResources;
        
        std::vector<TreeNodePtr> result;
        T usedResources = 0;
        
        // Copy entries to work with
        auto tempEntries = entries;
        std::sort_heap(tempEntries.begin(), tempEntries.end());
        
        // Allocate resources based on intent priority
        for (const auto& entry : tempEntries) {
            if (usedResources >= resourceLimit) break;
            
            result.push_back(entry.node);
            usedResources += entry.node->intentPriority / 100.0; // Simplified resource model
        }
        
        return result;
    }
    
    /**
     * @brief Update intent values from importance function
     * 
     * @param importanceFunc Function that calculates importance value
     */
    void updateIntentValues(std::function<T(const VectorT&)> importanceFunc) {
        for (auto& entry : entries) {
            entry.intentValue = importanceFunc(entry.node->position);
            entry.node->intentPriority = entry.intentValue;
        }
        
        // Rebuild heap
        std::make_heap(entries.begin(), entries.end());
    }
    
    /**
     * @brief Set total available computational resources
     * 
     * @param resources Resource limit (normalized to 1.0)
     */
    void setTotalResources(T resources) {
        totalResources = resources;
    }
    
private:
    std::vector<IntentEntry> entries;
    T totalResources;
};

/**
 * @brief Tree Binary Environment for spatial representation
 * 
 * Complete environment representation using tree binary structure with
 * intent-driven computation allocation.
 */
template <typename T, int Dimensions = 3>
class TreeBinaryEnvironment {
public:
    using VectorT = std::array<T, Dimensions>;
    using TreeNode = TreeBinaryNode<T, Dimensions>;
    using TreeNodePtr = std::shared_ptr<TreeNode>;
    
    TreeBinaryEnvironment() {
        // Initialize root node covering [-1,1] in all dimensions
        VectorT rootPos;
        rootPos.fill(0);
        root = std::make_shared<TreeNode>(rootPos);
    }
    
    /**
     * @brief Initialize environment with specified resolution
     * 
     * @param maxDepth Maximum depth of the tree
     * @param curvatureFunc Function returning curvature at position
     */
    void initialize(int maxDepth, std::function<T(const VectorT&)> curvatureFunc) {
        // Start with root node
        std::vector<TreeNodePtr> currentLevel = {root};
        
        // Build tree level by level
        for (int depth = 0; depth < maxDepth; ++depth) {
            std::vector<TreeNodePtr> nextLevel;
            
            for (const auto& node : currentLevel) {
                // Only subdivide nodes with significant curvature
                T nodeCurv = curvatureFunc(node->position);
                node->curvature = nodeCurv;
                
                if (std::abs(nodeCurv) > subdivisionThreshold || depth < minSubdivisionDepth) {
                    auto children = node->split();
                    nextLevel.insert(nextLevel.end(), children.begin(), children.end());
                    
                    // Update children's curvature
                    for (auto& child : children) {
                        child->curvature = curvatureFunc(child->position);
                    }
                }
            }
            
            if (nextLevel.empty()) break;
            currentLevel = nextLevel;
        }
        
        // Index all nodes for quick lookup
        indexNodes();
    }
    
    /**
     * @brief Get node at specified position
     * 
     * @param position Position to query
     * @return TreeNodePtr Node at position (or nullptr if not found)
     */
    TreeNodePtr getNodeAt(const VectorT& position) {
        return root->findNode(position);
    }
    
    /**
     * @brief Get all nodes with intent above threshold
     * 
     * @param threshold Intent threshold
     * @return std::vector<TreeNodePtr> High-intent nodes
     */
    std::vector<TreeNodePtr> getHighIntentNodes(T threshold) {
        std::vector<TreeNodePtr> result;
        collectHighIntentNodes(root, threshold, result);
        return result;
    }
    
    /**
     * @brief Update intent values across environment
     * 
     * @param intentFunc Function that calculates intent value at position
     */
    void updateIntentValues(std::function<T(const VectorT&)> intentFunc) {
        root->updateIntent(intentFunc);
    }
    
    /**
     * @brief Set subdivision parameters
     * 
     * @param threshold Curvature threshold for subdivision
     * @param minDepth Minimum subdivision depth
     */
    void setSubdivisionParams(T threshold, int minDepth) {
        subdivisionThreshold = threshold;
        minSubdivisionDepth = minDepth;
    }

private:
    TreeNodePtr root;
    std::unordered_map<int, TreeNodePtr> nodeIndex;
    T subdivisionThreshold = 0.1;
    int minSubdivisionDepth = 2;
    
    /**
     * @brief Build index of all nodes for quick lookup
     */
    void indexNodes() {
        nodeIndex.clear();
        indexNodeRecursive(root, 0);
    }
    
    /**
     * @brief Recursive helper for indexing nodes
     * 
     * @param node Current node
     * @param index Current index
     * @return int Next available index
     */
    int indexNodeRecursive(const TreeNodePtr& node, int index) {
        nodeIndex[index] = node;
        int nextIndex = index + 1;
        
        for (const auto& child : node->children) {
            nextIndex = indexNodeRecursive(child, nextIndex);
        }
        
        return nextIndex;
    }
    
    /**
     * @brief Collect nodes with intent above threshold
     * 
     * @param node Current node
     * @param threshold Intent threshold
     * @param result Vector to collect nodes
     */
    void collectHighIntentNodes(const TreeNodePtr& node, T threshold, 
                               std::vector<TreeNodePtr>& result) {
        if (node->intentPriority >= threshold) {
            result.push_back(node);
        }
        
        for (const auto& child : node->children) {
            // Prune branches with low intent
            if (child->intentPriority >= threshold * 0.8) {
                collectHighIntentNodes(child, threshold, result);
            }
        }
    }
};

} // namespace sdk
} // namespace physics

# Binary Space Tree

## Overview

The Binary Space Tree is a spatial partitioning data structure used in the Zero Point Physics Engine for efficient collision detection and spatial queries. This hierarchical structure recursively subdivides space to organize objects based on their spatial location, significantly improving performance for operations that depend on proximity.

## Structure and Implementation

```
┌───────────────────────────┐
│         BSTNode           │
├───────────────────────────┤
│ - AABB bounds             │
│ - std::vector<Node3D*>    │
│   objects                 │
│ - BSTNode* left           │
│ - BSTNode* right          │
│ - uint32_t depth          │
└───────────────┬───────────┘
                │
                ▼
┌───────────────────────────┐
│    BinarySpaceTree        │
├───────────────────────────┤
│ - BSTNode* root           │
│ - uint32_t max_depth      │
│ - uint32_t max_objects    │
│   per_node                │
│ - uint32_t total_nodes    │
│ - std::unordered_map<     │
│   uint32_t, BSTNode*>     │
│   object_map              │
└───────────────────────────┘
```

## Node Implementation

```cpp
struct BSTNode {
    AABB bounds;                         // Spatial bounds of this node
    std::vector<std::shared_ptr<Node3D>> objects;  // Objects in this node
    BSTNode* left;                       // Left child
    BSTNode* right;                      // Right child
    uint32_t depth;                      // Depth in tree
    
    BSTNode(const AABB& b, uint32_t d) : bounds(b), left(nullptr), right(nullptr), depth(d) {}
    
    bool isLeaf() const {
        return left == nullptr && right == nullptr;
    }
    
    void split(uint32_t max_objects, uint32_t max_depth) {
        // Don't split if we've reached max depth or don't have enough objects
        if (depth >= max_depth || objects.size() <= max_objects) {
            return;
        }
        
        // Calculate split axis and position
        Vec3 extents = bounds.getExtents();
        int axis = 0;
        if (extents.y > extents.x) axis = 1;
        if (extents.z > extents[axis]) axis = 2;
        
        float split_pos = bounds.min[axis] + extents[axis] * 0.5f;
        
        // Create child AABBs
        AABB left_bounds = bounds;
        left_bounds.max[axis] = split_pos;
        
        AABB right_bounds = bounds;
        right_bounds.min[axis] = split_pos;
        
        // Create child nodes
        left = new BSTNode(left_bounds, depth + 1);
        right = new BSTNode(right_bounds, depth + 1);
        
        // Distribute objects to children
        for (auto& obj : objects) {
            if (obj->position[axis] <= split_pos) {
                left->objects.push_back(obj);
            } else {
                right->objects.push_back(obj);
            }
        }
        
        // Clear objects from this node as they've been pushed to children
        objects.clear();
        
        // Recursively split children if needed
        left->split(max_objects, max_depth);
        right->split(max_objects, max_depth);
    }
};
```

## Core Operations

### Insertion

```cpp
uint32_t BinarySpaceTree::insertNode(std::shared_ptr<Node3D> node) {
    // Insert node into the tree
    insertNodeRecursive(root, node);
    
    // Add to object map for quick lookups
    object_map[node->id] = node;
    
    // Check if tree needs rebalancing
    if (shouldRebalance()) {
        rebalance();
    }
    
    return node->id;
}

void BinarySpaceTree::insertNodeRecursive(BSTNode* current, std::shared_ptr<Node3D> node) {
    // If node bounds don't fit in current node, return
    if (!current->bounds.contains(node->position)) {
        return;
    }
    
    // If leaf node, add object to this node
    if (current->isLeaf()) {
        current->objects.push_back(node);
        
        // Split if node has too many objects
        if (current->objects.size() > max_objects_per_node && current->depth < max_depth) {
            current->split(max_objects_per_node, max_depth);
        }
    }
    // If not leaf, recursively insert into appropriate child
    else {
        // Choose the child that contains the node position
        if (current->left->bounds.contains(node->position)) {
            insertNodeRecursive(current->left, node);
        } else if (current->right->bounds.contains(node->position)) {
            insertNodeRecursive(current->right, node);
        }
        // If neither child contains it, add to current node
        else {
            current->objects.push_back(node);
        }
    }
}
```

### Querying

```cpp
std::vector<std::shared_ptr<Node3D>> BinarySpaceTree::query(const AABB& area) {
    std::vector<std::shared_ptr<Node3D>> result;
    queryRecursive(root, area, result);
    return result;
}

void BinarySpaceTree::queryRecursive(BSTNode* node, const AABB& area, 
                                    std::vector<std::shared_ptr<Node3D>>& result) {
    // Early exit if query area doesn't intersect this node
    if (!node || !node->bounds.intersects(area)) {
        return;
    }
    
    // Add all objects in this node that intersect query area
    for (auto& obj : node->objects) {
        AABB obj_bounds(obj->position - Vec3(obj->radius), obj->position + Vec3(obj->radius));
        if (obj_bounds.intersects(area)) {
            result.push_back(obj);
        }
    }
    
    // Recursively query children
    if (node->left) queryRecursive(node->left, area, result);
    if (node->right) queryRecursive(node->right, area, result);
}
```

### Removal

```cpp
bool BinarySpaceTree::removeNode(uint32_t node_id) {
    // Check if node exists in object map
    auto it = object_map.find(node_id);
    if (it == object_map.end()) {
        return false;
    }
    
    // Remove node from tree
    bool removed = removeNodeRecursive(root, node_id);
    
    // Remove from object map
    if (removed) {
        object_map.erase(it);
    }
    
    return removed;
}

bool BinarySpaceTree::removeNodeRecursive(BSTNode* node, uint32_t node_id) {
    if (!node) {
        return false;
    }
    
    // Check objects in current node
    for (auto it = node->objects.begin(); it != node->objects.end(); ++it) {
        if ((*it)->id == node_id) {
            node->objects.erase(it);
            return true;
        }
    }
    
    // Recursively check children
    if (node->left && removeNodeRecursive(node->left, node_id)) {
        return true;
    }
    
    if (node->right && removeNodeRecursive(node->right, node_id)) {
        return true;
    }
    
    return false;
}
```

### Finding Potential Collisions

```cpp
std::vector<CollisionPair> BinarySpaceTree::findPotentialCollisions() {
    std::vector<CollisionPair> result;
    findCollisionsRecursive(root, result);
    return result;
}

void BinarySpaceTree::findCollisionsRecursive(BSTNode* node, std::vector<CollisionPair>& result) {
    if (!node) {
        return;
    }
    
    // Check for collisions within this node's objects
    for (size_t i = 0; i < node->objects.size(); ++i) {
        for (size_t j = i + 1; j < node->objects.size(); ++j) {
            // Add potential collision pair
            result.push_back({node->objects[i], node->objects[j]});
        }
    }
    
    // If leaf node, nothing more to do
    if (node->isLeaf()) {
        return;
    }
    
    // Check for collisions in children
    findCollisionsRecursive(node->left, result);
    findCollisionsRecursive(node->right, result);
    
    // Check for collisions between objects in different children
    findCollisionsBetweenNodes(node->left, node->right, result);
}

void BinarySpaceTree::findCollisionsBetweenNodes(BSTNode* node1, BSTNode* node2, 
                                               std::vector<CollisionPair>& result) {
    // Early exit if nodes don't exist or bounds don't overlap
    if (!node1 || !node2 || !node1->bounds.intersects(node2->bounds)) {
        return;
    }
    
    // Check collisions between objects in the two nodes
    for (auto& obj1 : node1->objects) {
        for (auto& obj2 : node2->objects) {
            float dist_sq = (obj1->position - obj2->position).lengthSquared();
            float radius_sum = obj1->radius + obj2->radius;
            
            // Add pair if objects' spheres overlap
            if (dist_sq <= radius_sum * radius_sum) {
                result.push_back({obj1, obj2});
            }
        }
    }
    
    // If both nodes are leaves, we're done
    if (node1->isLeaf() && node2->isLeaf()) {
        return;
    }
    
    // Recursively check combinations of children
    if (!node1->isLeaf() && !node2->isLeaf()) {
        findCollisionsBetweenNodes(node1->left, node2->left, result);
        findCollisionsBetweenNodes(node1->left, node2->right, result);
        findCollisionsBetweenNodes(node1->right, node2->left, result);
        findCollisionsBetweenNodes(node1->right, node2->right, result);
    }
    else if (!node1->isLeaf()) {
        findCollisionsBetweenNodes(node1->left, node2, result);
        findCollisionsBetweenNodes(node1->right, node2, result);
    }
    else {
        findCollisionsBetweenNodes(node1, node2->left, result);
        findCollisionsBetweenNodes(node1, node2->right, result);
    }
}
```

## Tree Rebalancing

```cpp
void BinarySpaceTree::rebalance() {
    // Collect all objects in the tree
    std::vector<std::shared_ptr<Node3D>> all_objects;
    collectAllObjects(root, all_objects);
    
    // Recalculate bounds based on object positions
    AABB new_bounds;
    for (auto& obj : all_objects) {
        new_bounds.expandToInclude(obj->position);
    }
    
    // Add some padding to bounds
    new_bounds.expand(1.0f);
    
    // Delete old tree
    deleteNode(root);
    
    // Create new root with updated bounds
    root = new BSTNode(new_bounds, 0);
    
    // Reinsert all objects
    for (auto& obj : all_objects) {
        insertNodeRecursive(root, obj);
    }
    
    // Update statistics
    updateTreeStatistics();
}

void BinarySpaceTree::collectAllObjects(BSTNode* node, std::vector<std::shared_ptr<Node3D>>& objects) {
    if (!node) {
        return;
    }
    
    // Add objects from current node
    objects.insert(objects.end(), node->objects.begin(), node->objects.end());
    
    // Recursively collect from children
    if (node->left) collectAllObjects(node->left, objects);
    if (node->right) collectAllObjects(node->right, objects);
}
```

## Military-Grade Enhancements

For military applications, the binary space tree incorporates several enhancements:

1. **Adaptive Splitting**: Dynamically adjust split axis and position based on object distribution
2. **Predictive Caching**: Pre-compute likely future spatial configurations
3. **Verified Tree Integrity**: Cryptographically verify tree structure to detect tampering
4. **High-Precision Boundary Calculations**: Use extended precision for critical boundary calculations

## Performance Characteristics

| Operation | Average Case | Worst Case |
|-----------|--------------|------------|
| Insert | O(log n) | O(n) |
| Query | O(log n) | O(n) |
| Remove | O(log n) | O(n) |
| FindCollisions | O(n log n) | O(n²) |
| Rebalance | O(n log n) | O(n log n) |

## Performance Optimization

- **Node Capacity Tuning**: Optimized based on typical object counts and distributions
- **Incremental Rebalancing**: Partial rebalancing of subtrees to avoid full restructuring
- **Bounding Volume Optimization**: Tight-fitting AABBs to minimize overlap
- **Cache-Friendly Memory Layout**: Node storage optimized for CPU cache usage

## Integration with Verification System

The binary space tree structure is verified to ensure spatial coherence:

```cpp
bool BinarySpaceTree::verifyTreeIntegrity() const {
    return verifyNodeIntegrity(root);
}

bool BinarySpaceTree::verifyNodeIntegrity(const BSTNode* node) const {
    if (!node) {
        return true;
    }
    
    // Verify all objects actually fit within this node's bounds
    for (const auto& obj : node->objects) {
        if (!node->bounds.contains(obj->position)) {
            return false;
        }
    }
    
    // If this is a leaf node, we're done with this branch
    if (node->isLeaf()) {
        return true;
    }
    
    // Verify children exist
    if (!node->left || !node->right) {
        return false;
    }
    
    // Verify child bounds are subsets of parent bounds
    if (!node->bounds.contains(node->left->bounds) || 
        !node->bounds.contains(node->right->bounds)) {
        return false;
    }
    
    // Verify child depths
    if (node->left->depth != node->depth + 1 || 
        node->right->depth != node->depth + 1) {
        return false;
    }
    
    // Recursively verify children
    return verifyNodeIntegrity(node->left) && verifyNodeIntegrity(node->right);
}
```

## Usage Example

```cpp
// Create binary space tree with initial bounds
AABB world_bounds(Vec3(-100, -100, -100), Vec3(100, 100, 100));
BinarySpaceTree tree(world_bounds);

// Configure tree parameters
tree.setMaxDepth(10);
tree.setMaxObjectsPerNode(8);

// Add objects to the tree
for (auto& node : physics_nodes) {
    tree.insertNode(node);
}

// Find potential collision pairs
auto collision_pairs = tree.findPotentialCollisions();

// Perform spatial query
AABB query_area(Vec3(5, 5, 5), Vec3(10, 10, 10));
auto objects_in_area = tree.query(query_area);

// Query objects within radius
auto nearby_objects = tree.queryRadius(Vec3(0, 0, 0), 10.0f);
```

## Future Enhancements

1. **GPU-accelerated tree operations** for massive object counts
2. **Dynamic tree depth adjustment** based on scene complexity
3. **Multi-threaded tree updates** for large-scale simulations
4. **Specialized tree variants** for different object distributions

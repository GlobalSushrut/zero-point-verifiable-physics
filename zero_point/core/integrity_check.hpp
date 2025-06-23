#pragma once

#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include "binary_space_tree.hpp"

namespace zero_point {
namespace core {

/**
 * @brief Modern cryptographic hash function for data integrity
 */
class HashFunction {
public:
    using Hash256 = std::array<uint8_t, 32>; // 256-bit hash
    
    /**
     * @brief Initialize hash function
     */
    HashFunction();
    
    /**
     * @brief Update hash with new data
     * @param data Pointer to data
     * @param size Size of data in bytes
     */
    void update(const void* data, size_t size);
    
    /**
     * @brief Reset hash to initial state
     */
    void reset();
    
    /**
     * @brief Complete hash computation
     * @return 256-bit hash value
     */
    Hash256 finalize();
    
    /**
     * @brief Create hash from data in one call
     * @param data Data to hash
     * @param size Size in bytes
     * @return Hash value
     */
    static Hash256 hash(const void* data, size_t size);
    
    /**
     * @brief Convert hash to hexadecimal string
     * @param hash Hash value
     * @return Hex string representation
     */
    static std::string hashToString(const Hash256& hash);
    
private:
    // Internal state (SHA-256 based)
    uint32_t state_[8];
    uint8_t buffer_[64];
    uint64_t bit_count_;
    size_t buffer_index_;
    
    void transform_();
};

/**
 * @brief Military/NASA-grade simulation integrity verification
 * 
 * This class provides cryptographic verification of simulation integrity
 * to detect tampering, corruption, or computational errors.
 */
class IntegrityCheck {
public:
    /**
     * @brief Initialize integrity verification system
     * @param strict_checking Enable strict verification (more performance impact)
     */
    IntegrityCheck(bool strict_checking = false);
    
    /**
     * @brief Toggle strict checking mode
     * @param strict True to enable strict checking
     */
    void setStrictChecking(bool strict);
    
    /**
     * @brief Calculate integrity hash for a node
     * @param node Node to hash
     * @return Hash value
     */
    HashFunction::Hash256 calculateNodeHash(const Node3D& node) const;
    
    /**
     * @brief Calculate integrity hash for entire space tree
     * @param space_tree Space tree to verify
     * @return Hash value
     */
    HashFunction::Hash256 calculateSpaceTreeHash(const BinarySpaceTree& space_tree) const;
    
    /**
     * @brief Verify node integrity against previous hash
     * @param node Node to verify
     * @param previous_hash Previously calculated hash
     * @return True if integrity check passes
     */
    bool verifyNodeIntegrity(const Node3D& node, const HashFunction::Hash256& previous_hash) const;
    
    /**
     * @brief Compute integrity signature for simulation state
     * @param space_tree Space tree containing simulation state
     * @param timestamp Simulation timestamp
     * @return Signature for the state
     */
    std::string computeSimulationSignature(
        const BinarySpaceTree& space_tree, double timestamp) const;
    
    /**
     * @brief Verify data hasn't been corrupted
     * @param data Data buffer
     * @param size Size in bytes
     * @param expected_hash Expected hash value
     * @return True if integrity verification passes
     */
    bool verifyDataIntegrity(
        const void* data, size_t size, const HashFunction::Hash256& expected_hash) const;
    
    /**
     * @brief Generate tamper-evident log entry
     * @param message Log message
     * @param level Log level
     * @return Signed log entry
     */
    std::string generateSecureLogEntry(const std::string& message, const std::string& level) const;
    
private:
    bool strict_checking_;
    
    // Secure random number source for challenge generation
    std::vector<uint8_t> generateSecureRandom_(size_t bytes) const;
    
    // Internal hash combination function
    HashFunction::Hash256 combineHashes_(
        const HashFunction::Hash256& h1, const HashFunction::Hash256& h2) const;
};

} // namespace core
} // namespace zero_point

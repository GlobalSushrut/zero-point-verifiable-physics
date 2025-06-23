#include "integrity_check.hpp"
#include <iomanip>
#include <sstream>
#include <cstring>
#include <random>

namespace zero_point {
namespace core {

// SHA-256 constants
constexpr uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

// SHA-256 helper functions
#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define Ch(x, y, z) (((x) & (y)) ^ (~(x) & (z)))
#define Maj(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define Sigma0(x) (ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define Sigma1(x) (ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define sigma0(x) (ROTR(x, 7) ^ ROTR(x, 18) ^ ((x) >> 3))
#define sigma1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

// HashFunction implementation
HashFunction::HashFunction() {
    reset();
}

void HashFunction::reset() {
    // Initial hash values (first 32 bits of the fractional parts of the square roots of the first 8 primes)
    state_[0] = 0x6a09e667;
    state_[1] = 0xbb67ae85;
    state_[2] = 0x3c6ef372;
    state_[3] = 0xa54ff53a;
    state_[4] = 0x510e527f;
    state_[5] = 0x9b05688c;
    state_[6] = 0x1f83d9ab;
    state_[7] = 0x5be0cd19;
    
    bit_count_ = 0;
    buffer_index_ = 0;
    std::memset(buffer_, 0, sizeof(buffer_));
}

void HashFunction::update(const void* data, size_t size) {
    const uint8_t* ptr = static_cast<const uint8_t*>(data);
    
    // Update bit count
    bit_count_ += size * 8;
    
    // Process data in chunks
    while (size > 0) {
        // Fill buffer
        size_t space = 64 - buffer_index_;
        size_t copy_size = (size < space) ? size : space;
        std::memcpy(buffer_ + buffer_index_, ptr, copy_size);
        
        // Update pointers
        buffer_index_ += copy_size;
        ptr += copy_size;
        size -= copy_size;
        
        // Process block if full
        if (buffer_index_ == 64) {
            transform_();
            buffer_index_ = 0;
        }
    }
}

void HashFunction::transform_() {
    // Prepare message schedule
    uint32_t W[64];
    for (int i = 0; i < 16; i++) {
        W[i] = (static_cast<uint32_t>(buffer_[i * 4]) << 24) |
               (static_cast<uint32_t>(buffer_[i * 4 + 1]) << 16) |
               (static_cast<uint32_t>(buffer_[i * 4 + 2]) << 8) |
                static_cast<uint32_t>(buffer_[i * 4 + 3]);
    }
    
    for (int i = 16; i < 64; i++) {
        W[i] = sigma1(W[i - 2]) + W[i - 7] + sigma0(W[i - 15]) + W[i - 16];
    }
    
    // Initialize working variables
    uint32_t a = state_[0];
    uint32_t b = state_[1];
    uint32_t c = state_[2];
    uint32_t d = state_[3];
    uint32_t e = state_[4];
    uint32_t f = state_[5];
    uint32_t g = state_[6];
    uint32_t h = state_[7];
    
    // Main loop
    for (int i = 0; i < 64; i++) {
        uint32_t T1 = h + Sigma1(e) + Ch(e, f, g) + K[i] + W[i];
        uint32_t T2 = Sigma0(a) + Maj(a, b, c);
        
        h = g;
        g = f;
        f = e;
        e = d + T1;
        d = c;
        c = b;
        b = a;
        a = T1 + T2;
    }
    
    // Update state
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
}

HashFunction::Hash256 HashFunction::finalize() {
    // Add padding
    uint8_t pad[72];
    size_t pad_size = (buffer_index_ < 56) ? (56 - buffer_index_) : (120 - buffer_index_);
    
    // Start with 1 bit followed by zeros
    pad[0] = 0x80;
    std::memset(pad + 1, 0, pad_size - 1);
    
    update(pad, pad_size);
    
    // Add length (big endian)
    pad[0] = static_cast<uint8_t>((bit_count_ >> 56) & 0xFF);
    pad[1] = static_cast<uint8_t>((bit_count_ >> 48) & 0xFF);
    pad[2] = static_cast<uint8_t>((bit_count_ >> 40) & 0xFF);
    pad[3] = static_cast<uint8_t>((bit_count_ >> 32) & 0xFF);
    pad[4] = static_cast<uint8_t>((bit_count_ >> 24) & 0xFF);
    pad[5] = static_cast<uint8_t>((bit_count_ >> 16) & 0xFF);
    pad[6] = static_cast<uint8_t>((bit_count_ >> 8) & 0xFF);
    pad[7] = static_cast<uint8_t>(bit_count_ & 0xFF);
    
    update(pad, 8);
    
    // Generate hash
    Hash256 hash;
    for (int i = 0; i < 8; i++) {
        hash[i * 4] = static_cast<uint8_t>((state_[i] >> 24) & 0xFF);
        hash[i * 4 + 1] = static_cast<uint8_t>((state_[i] >> 16) & 0xFF);
        hash[i * 4 + 2] = static_cast<uint8_t>((state_[i] >> 8) & 0xFF);
        hash[i * 4 + 3] = static_cast<uint8_t>(state_[i] & 0xFF);
    }
    
    return hash;
}

HashFunction::Hash256 HashFunction::hash(const void* data, size_t size) {
    HashFunction hasher;
    hasher.update(data, size);
    return hasher.finalize();
}

std::string HashFunction::hashToString(const Hash256& hash) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    
    for (uint8_t byte : hash) {
        ss << std::setw(2) << static_cast<int>(byte);
    }
    
    return ss.str();
}

// IntegrityCheck implementation
IntegrityCheck::IntegrityCheck(bool strict_checking)
    : strict_checking_(strict_checking) {
}

void IntegrityCheck::setStrictChecking(bool strict) {
    strict_checking_ = strict;
}

HashFunction::Hash256 IntegrityCheck::calculateNodeHash(const Node3D& node) const {
    HashFunction hasher;
    
    // Hash node ID
    hasher.update(&node.id, sizeof(node.id));
    
    // Hash position
    hasher.update(&node.position, sizeof(node.position));
    
    // Hash velocity
    hasher.update(&node.velocity, sizeof(node.velocity));
    
    // Hash other important properties
    hasher.update(&node.mass, sizeof(node.mass));
    hasher.update(&node.is_dynamic, sizeof(node.is_dynamic));
    hasher.update(&node.curvature, sizeof(node.curvature));
    
    // In strict mode, hash all properties
    if (strict_checking_) {
        hasher.update(&node.intent_priority, sizeof(node.intent_priority));
        hasher.update(&node.accumulated_force, sizeof(node.accumulated_force));
        hasher.update(&node.density, sizeof(node.density));
        
        // Note: We don't hash parent pointer to avoid cycles
    }
    
    return hasher.finalize();
}

HashFunction::Hash256 IntegrityCheck::calculateSpaceTreeHash(const BinarySpaceTree& space_tree) const {
    HashFunction hasher;
    
    // Hash total node count
    size_t node_count = space_tree.getNodeCount();
    hasher.update(&node_count, sizeof(node_count));
    
    // In strict mode, hash all individual nodes
    // This is expensive but thorough
    if (strict_checking_ && node_count > 0) {
        // We can't directly iterate all nodes, so use IDs
        for (uint32_t i = 1; i <= node_count; i++) {
            auto node = space_tree.findNode(i);
            if (node) {
                auto node_hash = calculateNodeHash(*node);
                hasher.update(node_hash.data(), node_hash.size());
            }
        }
    } else {
        // In normal mode, just hash some key properties
        float curvature = space_tree.getRootCurvature();
        hasher.update(&curvature, sizeof(curvature));
        
        // Sample a few random nodes for hash
        std::vector<uint32_t> sample_ids = {1, node_count / 2, node_count};
        for (uint32_t id : sample_ids) {
            auto node = space_tree.findNode(id);
            if (node) {
                auto node_hash = calculateNodeHash(*node);
                hasher.update(node_hash.data(), node_hash.size());
            }
        }
    }
    
    return hasher.finalize();
}

bool IntegrityCheck::verifyNodeIntegrity(
    const Node3D& node, 
    const HashFunction::Hash256& previous_hash) const {
    
    auto current_hash = calculateNodeHash(node);
    return current_hash == previous_hash;
}

std::string IntegrityCheck::computeSimulationSignature(
    const BinarySpaceTree& space_tree, double timestamp) const {
    
    HashFunction hasher;
    
    // Hash space tree state
    auto tree_hash = calculateSpaceTreeHash(space_tree);
    hasher.update(tree_hash.data(), tree_hash.size());
    
    // Hash timestamp for temporal integrity
    hasher.update(&timestamp, sizeof(timestamp));
    
    // Add a "salt" based on secure random data
    auto salt = generateSecureRandom_(16);
    hasher.update(salt.data(), salt.size());
    
    // Generate final signature
    auto final_hash = hasher.finalize();
    return HashFunction::hashToString(final_hash);
}

bool IntegrityCheck::verifyDataIntegrity(
    const void* data, 
    size_t size, 
    const HashFunction::Hash256& expected_hash) const {
    
    auto actual_hash = HashFunction::hash(data, size);
    return actual_hash == expected_hash;
}

std::string IntegrityCheck::generateSecureLogEntry(
    const std::string& message, 
    const std::string& level) const {
    
    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
    
    // Format log entry
    std::stringstream entry;
    entry << timestamp << "|" << level << "|" << message;
    
    // Add integrity signature
    HashFunction hasher;
    hasher.update(entry.str().data(), entry.str().size());
    auto signature = hasher.finalize();
    
    // Append signature to log
    entry << "|SIG:" << HashFunction::hashToString(signature);
    
    return entry.str();
}

std::vector<uint8_t> IntegrityCheck::generateSecureRandom_(size_t bytes) const {
    std::vector<uint8_t> random_data(bytes);
    
    // Use a cryptographically secure PRNG
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint16_t> distrib(0, 255);
    
    for (size_t i = 0; i < bytes; i++) {
        random_data[i] = static_cast<uint8_t>(distrib(gen) & 0xFF);
    }
    
    return random_data;
}

HashFunction::Hash256 IntegrityCheck::combineHashes_(
    const HashFunction::Hash256& h1, 
    const HashFunction::Hash256& h2) const {
    
    HashFunction hasher;
    hasher.update(h1.data(), h1.size());
    hasher.update(h2.data(), h2.size());
    return hasher.finalize();
}

} // namespace core
} // namespace zero_point

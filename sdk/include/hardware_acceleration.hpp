#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <unordered_map>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <queue>
#include <condition_variable>
#include <future>

// Include optional SIMD support 
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace physics {
namespace sdk {

/**
 * @brief Hardware acceleration utilities for optimal performance on constrained hardware
 * 
 * Provides optimizations specific to ARM-based systems like Raspberry Pi
 * with support for SIMD, multi-threading, and memory optimization.
 */
template <typename T = float, int Dimensions = 3>
class HardwareAcceleration {
public:
    using VectorT = std::array<T, Dimensions>;
    
    // Memory pool for fixed allocation
    struct MemoryPool {
        std::vector<uint8_t> data;
        size_t capacity;
        size_t used;
        std::mutex mutex;
        
        explicit MemoryPool(size_t initialCapacity = 1024*1024)
            : capacity(initialCapacity)
            , used(0) {
            data.resize(capacity);
        }
        
        void* allocate(size_t size, size_t alignment = alignof(::max_align_t)) {
            std::lock_guard<std::mutex> lock(mutex);
            
            // Align address
            size_t alignmentMask = alignment - 1;
            size_t alignedOffset = (used + alignmentMask) & ~alignmentMask;
            
            // Check if enough space
            if (alignedOffset + size > capacity) {
                return nullptr;  // Out of memory
            }
            
            used = alignedOffset + size;
            return &data[alignedOffset];
        }
        
        void reset() {
            std::lock_guard<std::mutex> lock(mutex);
            used = 0;
        }
    };
    
    // Thread pool for parallel execution
    class ThreadPool {
    public:
        ThreadPool(size_t numThreads = std::thread::hardware_concurrency())
            : stop_(false) {
            
            for (size_t i = 0; i < numThreads; ++i) {
                threads_.emplace_back([this] {
                    while (true) {
                        std::function<void()> task;
                        
                        {
                            std::unique_lock<std::mutex> lock(mutex_);
                            condition_.wait(lock, [this] {
                                return stop_ || !tasks_.empty();
                            });
                            
                            if (stop_ && tasks_.empty()) return;
                            
                            task = std::move(tasks_.front());
                            tasks_.pop();
                        }
                        
                        task();
                    }
                });
            }
        }
        
        ~ThreadPool() {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                stop_ = true;
            }
            
            condition_.notify_all();
            
            for (auto& thread : threads_) {
                thread.join();
            }
        }
        
        template<class F>
        void enqueue(F&& f) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                tasks_.push(std::forward<F>(f));
            }
            condition_.notify_one();
        }
        
        size_t threadCount() const {
            return threads_.size();
        }
        
    private:
        std::vector<std::thread> threads_;
        std::queue<std::function<void()>> tasks_;
        std::mutex mutex_;
        std::condition_variable condition_;
        bool stop_;
    };
    
    HardwareAcceleration() 
        : useSimd_(checkSimdSupport())
        , threadPool_(std::thread::hardware_concurrency())
        , generalPool_(10 * 1024 * 1024)  // 10 MB general pool
        , physicsPool_(20 * 1024 * 1024)  // 20 MB physics pool
        , renderPool_(20 * 1024 * 1024)   // 20 MB render pool
    {}
    
    /**
     * @brief Check if SIMD is supported and enabled
     * 
     * @return bool SIMD support status
     */
    bool hasSimdSupport() const {
        return useSimd_;
    }
    
    /**
     * @brief Get the number of hardware threads available
     * 
     * @return size_t Thread count
     */
    size_t getThreadCount() const {
        return threadPool_.threadCount();
    }
    
    /**
     * @brief Execute a function in parallel across data
     * 
     * @param data Data to process
     * @param func Function to execute on each data item
     */
    template<typename DataType, typename Func>
    void parallelFor(std::vector<DataType>& data, Func func) {
        size_t threadCount = threadPool_.threadCount();
        size_t blockSize = (data.size() + threadCount - 1) / threadCount;
        
        std::atomic<size_t> counter(0);
        std::vector<std::future<void>> futures;
        
        for (size_t t = 0; t < threadCount; ++t) {
            threadPool_.enqueue([&, t] {
                size_t start = t * blockSize;
                size_t end = std::min(start + blockSize, data.size());
                
                for (size_t i = start; i < end; ++i) {
                    func(data[i], i);
                    counter.fetch_add(1, std::memory_order_relaxed);
                }
            });
        }
        
        // Wait until processing is complete
        while (counter.load(std::memory_order_relaxed) < data.size()) {
            std::this_thread::yield();
        }
    }
    
    /**
     * @brief Allocate memory from a specific pool
     * 
     * @param size Size in bytes to allocate
     * @param poolType Pool to allocate from (0=general, 1=physics, 2=render)
     * @return void* Pointer to allocated memory
     */
    void* allocateFromPool(size_t size, int poolType = 0) {
        switch (poolType) {
            case 1: return physicsPool_.allocate(size);
            case 2: return renderPool_.allocate(size);
            default: return generalPool_.allocate(size);
        }
    }
    
    /**
     * @brief Reset a specific memory pool
     * 
     * @param poolType Pool to reset (0=general, 1=physics, 2=render)
     */
    void resetPool(int poolType = 0) {
        switch (poolType) {
            case 1: physicsPool_.reset(); break;
            case 2: renderPool_.reset(); break;
            default: generalPool_.reset(); break;
        }
    }
    
    /**
     * @brief Optimize vector operations with SIMD when available
     * 
     * @param a First vector
     * @param b Second vector
     * @param result Result vector
     * @param operation Operation to perform (0=add, 1=subtract, 2=multiply)
     */
    void vectorOp(const VectorT& a, const VectorT& b, VectorT& result, int operation = 0) {
        if (useSimd_ && Dimensions >= 4) {
            vectorOpSimd(a, b, result, operation);
        } else {
            vectorOpScalar(a, b, result, operation);
        }
    }
    
    /**
     * @brief Calculate dot product with optimizations
     * 
     * @param a First vector
     * @param b Second vector
     * @return T Dot product result
     */
    T dotProduct(const VectorT& a, const VectorT& b) {
        if (useSimd_ && Dimensions >= 4) {
            return dotProductSimd(a, b);
        } else {
            return dotProductScalar(a, b);
        }
    }

private:
    bool useSimd_;
    ThreadPool threadPool_;
    MemoryPool generalPool_;
    MemoryPool physicsPool_;
    MemoryPool renderPool_;
    
    bool checkSimdSupport() {
#ifdef __ARM_NEON
        return true;
#else
        return false;
#endif
    }
    
    void vectorOpScalar(const VectorT& a, const VectorT& b, VectorT& result, int operation) {
        for (int i = 0; i < Dimensions; ++i) {
            switch (operation) {
                case 0: result[i] = a[i] + b[i]; break;
                case 1: result[i] = a[i] - b[i]; break;
                case 2: result[i] = a[i] * b[i]; break;
                default: result[i] = a[i];
            }
        }
    }
    
    T dotProductScalar(const VectorT& a, const VectorT& b) {
        T result = 0;
        for (int i = 0; i < Dimensions; ++i) {
            result += a[i] * b[i];
        }
        return result;
    }
    
#ifdef __ARM_NEON
    void vectorOpSimd(const VectorT& a, const VectorT& b, VectorT& result, int operation) {
        // Example implementation for float32 with NEON
        if constexpr (std::is_same_v<T, float> && Dimensions >= 4) {
            float32x4_t va = vld1q_f32(a.data());
            float32x4_t vb = vld1q_f32(b.data());
            float32x4_t vresult;
            
            switch (operation) {
                case 0: vresult = vaddq_f32(va, vb); break;
                case 1: vresult = vsubq_f32(va, vb); break;
                case 2: vresult = vmulq_f32(va, vb); break;
                default: vresult = va;
            }
            
            vst1q_f32(result.data(), vresult);
            
            // Handle remaining elements scalar
            for (int i = 4; i < Dimensions; ++i) {
                switch (operation) {
                    case 0: result[i] = a[i] + b[i]; break;
                    case 1: result[i] = a[i] - b[i]; break;
                    case 2: result[i] = a[i] * b[i]; break;
                    default: result[i] = a[i];
                }
            }
        } else {
            vectorOpScalar(a, b, result, operation);
        }
    }
    
    T dotProductSimd(const VectorT& a, const VectorT& b) {
        // Example implementation for float32 with NEON
        if constexpr (std::is_same_v<T, float> && Dimensions >= 4) {
            float32x4_t va = vld1q_f32(a.data());
            float32x4_t vb = vld1q_f32(b.data());
            float32x4_t vmul = vmulq_f32(va, vb);
            
            // Horizontal add
            float32x2_t vsum = vadd_f32(vget_low_f32(vmul), vget_high_f32(vmul));
            vsum = vpadd_f32(vsum, vsum);
            T result = vget_lane_f32(vsum, 0);
            
            // Handle remaining elements scalar
            for (int i = 4; i < Dimensions; ++i) {
                result += a[i] * b[i];
            }
            
            return result;
        } else {
            return dotProductScalar(a, b);
        }
    }
#else
    // Fall back to scalar implementations when NEON not available
    void vectorOpSimd(const VectorT& a, const VectorT& b, VectorT& result, int operation) {
        vectorOpScalar(a, b, result, operation);
    }
    
    T dotProductSimd(const VectorT& a, const VectorT& b) {
        return dotProductScalar(a, b);
    }
#endif
};

} // namespace sdk
} // namespace physics

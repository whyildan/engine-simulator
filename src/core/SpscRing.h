#pragma once
#include <atomic>
#include <cstddef>
#include <vector>

namespace es {

// Lock-free single-producer / single-consumer ring buffer of POD items.
template <class T>
class SpscRing {
public:
    explicit SpscRing(size_t capacityPow2 = 16384) : buf_(capacityPow2), mask_(capacityPow2 - 1) {}

    size_t capacity() const { return buf_.size(); }
    size_t size() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
    size_t freeSpace() const { return capacity() - size(); }

    // producer
    size_t push(const T* data, size_t n) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t t = tail_.load(std::memory_order_acquire);
        size_t room = capacity() - (h - t);
        if (n > room) n = room;
        for (size_t i = 0; i < n; ++i) buf_[(h + i) & mask_] = data[i];
        head_.store(h + n, std::memory_order_release);
        return n;
    }
    // consumer
    size_t pop(T* out, size_t n) {
        size_t t = tail_.load(std::memory_order_relaxed);
        size_t h = head_.load(std::memory_order_acquire);
        size_t avail = h - t;
        if (n > avail) n = avail;
        for (size_t i = 0; i < n; ++i) out[i] = buf_[(t + i) & mask_];
        tail_.store(t + n, std::memory_order_release);
        return n;
    }

private:
    std::vector<T> buf_;
    size_t mask_;
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

} // namespace es

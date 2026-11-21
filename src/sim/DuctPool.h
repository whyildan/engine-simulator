#pragma once
// Steps the 1D ducts of a multi-cylinder two-stroke in parallel (T10 follow-up).
//
// Every cylinder of a two-stroke carries two ducts (tuned pipe, intake tract) and their cell updates are ~2/3 of the
// engine's work; they are independent of each other within an engine step (each duct only reads its own boundary
// fluxes, solved before), so they can run on several cores with results identical to the serial order.
//
// The engine steps at 48 kHz, so a step has ~20 us: the hand-off must cost well under a microsecond. The caller
// publishes the task list and works on it itself (it never waits for a worker to wake up); workers spin briefly on
// the generation counter, then yield, then sleep - a sleeping worker simply takes no tasks. The caller waits only
// for tasks a worker has already claimed.
#include "sim/Duct.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <thread>
#include <vector>
#if defined(_MSC_VER)
#include <intrin.h>
#define ES_CPU_PAUSE() _mm_pause()
#elif defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define ES_CPU_PAUSE() _mm_pause()
#else
#define ES_CPU_PAUSE() ((void)0)
#endif

namespace es {

struct DuctTask { Duct* duct; double ambientK; };

class DuctPool {
public:
    explicit DuctPool(int workers) {
        for (int i = 0; i < workers; ++i) threads_.emplace_back([this] { loop(); });
    }
    ~DuctPool() {
        quit_.store(true, std::memory_order_release);
        gen_.fetch_add(1, std::memory_order_release);
        { std::lock_guard<std::mutex> lk(mx_); cv_.notify_all(); }
        for (std::thread& t : threads_) t.join();
    }
    int workers() const { return (int)threads_.size(); }

    // duct->step(dt, ambientK) for every task, in parallel; returns when all are done
    void run(const DuctTask* tasks, int count, double dt) {
        // (the parameters, then done = 0, then next = 0 with release: a worker's claim - an acquire on next - that sees
        //  this round's index also sees its parameters, even if it woke up for an earlier round)
        tasks_ = tasks; count_.store(count, std::memory_order_relaxed); dt_ = dt;
        done_.store(0, std::memory_order_relaxed);
        next_.store(0, std::memory_order_release);
        gen_.fetch_add(1, std::memory_order_acq_rel);   // wake the workers
        if (sleepers_.load(std::memory_order_acquire) > 0) { std::lock_guard<std::mutex> lk(mx_); cv_.notify_all(); }
        work();
        while (done_.load(std::memory_order_acquire) < count) ES_CPU_PAUSE();
    }

private:
    void work() {
        for (;;) {
            const int k = next_.fetch_add(1, std::memory_order_acq_rel);
            if (k >= count_.load(std::memory_order_relaxed)) return;
            tasks_[k].duct->step(dt_, tasks_[k].ambientK);
            done_.fetch_add(1, std::memory_order_acq_rel);
        }
    }
    void loop() {
        unsigned seen = gen_.load(std::memory_order_acquire);
        for (;;) {
            // wait for a new generation: spin ~100 us (the next engine step comes ~20 us later while rendering), then
            // yield a while, then sleep on the condition variable (between audio blocks / paused) - run() wakes the
            // sleepers at once, so a block never starts without them
            int spins = 0;
            unsigned g;
            while ((g = gen_.load(std::memory_order_acquire)) == seen) {
                if (++spins < 3000) ES_CPU_PAUSE();
                else if (spins < 4000) std::this_thread::yield();
                else {
                    std::unique_lock<std::mutex> lk(mx_);
                    sleepers_.fetch_add(1, std::memory_order_acq_rel);
                    cv_.wait_for(lk, std::chrono::milliseconds(20), [&] { return gen_.load(std::memory_order_acquire) != seen; });
                    sleepers_.fetch_sub(1, std::memory_order_acq_rel);
                    spins = 0;
                }
            }
            seen = g;
            if (quit_.load(std::memory_order_acquire)) return;
            work();
        }
    }

    std::vector<std::thread> threads_;
    std::atomic<unsigned> gen_{0};
    std::atomic<int> next_{0}, done_{0};
    std::atomic<bool> quit_{false};
    std::atomic<int> sleepers_{0};
    std::mutex mx_;
    std::condition_variable cv_;
    const DuctTask* tasks_ = nullptr;
    std::atomic<int> count_{0};   // (atomic: a late worker from an earlier round reads it)
    double dt_ = 0;
};

} // namespace es

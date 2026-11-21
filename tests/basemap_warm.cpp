// Fill the base map disk cache for every preset in parallel (one preset per thread at a time). Run it once after
// the base map version changed; every test afterwards just reads the cache.
// usage: basemap_warm [threads]      (BASEMAP_CACHE must point to the cache folder)
#include "ecu/BaseMap.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>
using namespace es;

int main(int argc, char** argv) {
    if (!std::getenv("BASEMAP_CACHE")) { std::printf("set BASEMAP_CACHE to the cache folder first\n"); return 1; }
    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    const int threads = argc > 1 ? std::max(1, std::atoi(argv[1])) : (int)std::max(1u, hw > 2 ? hw - 1 : hw);
    setBaseMapThreads(1);   // (the presets run in parallel here: one thread each)
    std::atomic<int> next{0};
    std::mutex out;
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::thread> pool;
    for (int k = 0; k < threads; ++k)
        pool.emplace_back([&] {
            for (int i = next++; i < presetCount(); i = next++) {
                const auto a = std::chrono::steady_clock::now();
                makeCalibratedPreset(i);   // measures (or loads) and stores the base map
                const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count();
                std::lock_guard<std::mutex> lk(out);
                std::printf("  %2d %-46.46s %6.1f s\n", i, presetName(i), s);
                std::fflush(stdout);
            }
        });
    for (auto& t : pool) t.join();
    std::printf("%d presets with %d threads in %.0f s\n", presetCount(), threads,
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}

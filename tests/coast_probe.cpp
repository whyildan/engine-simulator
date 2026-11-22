// Coast / re-throttle probe: a preset driving in a gear - full throttle, a long lift-off (engine braking), full throttle
// again - through the whole Simulator (audio output, telemetry). Reports NaN / silence / stall.
// usage: coast_probe <preset> [gear] [coastSeconds] [wotLastSeconds]
//        coast_probe <preset> fuzz <seed> [seconds]   - keyboard-like throttle (ramps, setParams every UI frame) at random
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>
using namespace es;

int main(int argc, char** argv) {
    const int id = argc > 1 ? std::atoi(argv[1]) : 30;
    EngineParams p = makeCalibratedPreset(id);
    Simulator sim(p);
    Telemetry t;
    if (argc > 2 && !std::strcmp(argv[2], "fuzz")) {
        // the app: a UI frame every 1/60 s sets the params (the throttle ramping up 2.5/s with the key held, down
        // 4/s when released); audio in 480-frame blocks
        const unsigned seed = argc > 3 ? (unsigned)std::atoi(argv[3]) : 1;
        const double secs = argc > 4 ? std::atof(argv[4]) : 120.0;
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> U(0.0, 1.0);
        std::vector<float> buf(800 * 2);
        p.ignition = 1; p.starter = 1; sim.setParams(p);
        for (int i = 0; i < 60; ++i) sim.render(buf.data(), 800);
        p.starter = 0; p.vehicle = 1; p.gear = 1; sim.setParams(p);
        bool key = false; double keyLeft = 0, gearLeft = 3;
        int silent = 0;
        double peak = 0;
        for (double clock = 0; clock < secs; clock += 1.0 / 60.0) {
            if ((keyLeft -= 1.0 / 60.0) <= 0) { key = !key; keyLeft = key ? 0.2 + 3.0 * U(rng) : 0.3 + 6.0 * U(rng); }
            if ((gearLeft -= 1.0 / 60.0) <= 0) { p.gear = 1 + (int)(U(rng) * 5); gearLeft = 2 + 8 * U(rng); }
            p.throttle = key ? std::min(1.0f, p.throttle + 2.5f / 60) : std::max(0.0f, p.throttle - 4.0f / 60);
            sim.setParams(p);
            sim.render(buf.data(), 800);
            sim.getTelemetry(t);
            double rms = 0; bool nan = false;
            for (float x : buf) { if (!std::isfinite(x)) nan = true; rms += (double)x * x; }
            rms = std::sqrt(rms / buf.size());
            peak = std::max(peak, rms);
            silent = rms < 1e-4 ? silent + 1 : 0;
            const bool bad = nan || !std::isfinite(t.rpm) || !std::isfinite(t.mapKPa) || silent > 30;
            if (bad || std::fmod(clock, 1.0) < 1.0 / 60.0)
                std::printf("%7.2f rpm %6.0f %5.1f km/h g%d thr %.2f map %6.1f rms %.4f run %d%s\n", clock, t.rpm, t.speedKmh, t.gear,
                            p.throttle, t.mapKPa, rms, (int)t.running, nan ? " NaN!" : silent > 30 ? " SILENT" : "");
            if (bad) { std::printf("FAIL seed %u at %.2f\n", seed, clock); return 1; }
        }
        std::printf("done seed %u\n", seed);
        return 0;
    }
    const int gear = argc > 2 ? std::atoi(argv[2]) : 3;
    const double coast = argc > 3 ? std::atof(argv[3]) : 8.0;
    const double wotLast = argc > 4 ? std::atof(argv[4]) : 2.5;   // full throttle in the last gear before the lift
    std::vector<float> buf(480 * 2);
    auto run = [&](double secs, const char* phase) {
        for (double s = 0; s < secs; s += 0.01) {
            sim.render(buf.data(), 480);
            double rms = 0; bool nan = false;
            for (float x : buf) { if (!std::isfinite(x)) nan = true; rms += (double)x * x; }
            rms = std::sqrt(rms / buf.size());
            sim.getTelemetry(t);
            static double next = 0, clock = 0;
            clock += 0.01;
            if (clock >= next || nan) {
                std::printf("%6.2f %-8s rpm %6.0f  %5.1f km/h  gear %d  thr %.2f  map %5.1f  rms %.4f%s  run %d  mis %d\n", clock, phase, t.rpm,
                            t.speedKmh, t.gear, p.throttle, t.mapKPa, rms, nan ? "  NaN!" : "", (int)t.running, t.misfires);
                next = clock + 0.25;
                if (nan) return false;
            }
        }
        return true;
    };
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    run(1.0, "start");
    p.starter = 0; sim.setParams(p);
    run(1.0, "idle");
    p.vehicle = 1; p.gear = 1; sim.setParams(p);
    p.throttle = 1.0f; sim.setParams(p);
    if (!run(2.5, "g1 wot")) return 1;
    for (int g = 2; g <= gear; ++g) { p.gear = g; sim.setParams(p); if (!run(g == gear ? wotLast : 2.5, "wot")) return 1; }
    p.throttle = 0.0f; sim.setParams(p);
    if (!run(coast, "coast")) return 1;
    p.throttle = 1.0f; sim.setParams(p);
    if (!run(4.0, "rethrot")) return 1;
    std::printf("done\n");
    return 0;
}

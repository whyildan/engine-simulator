// Long idle robustness: start each preset like the app does and let it idle for a while; reports the rpm range
// and when (if) it stalled.   usage: idle_hold [seconds] [preset ...]   (no presets: all)
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
using namespace es;

int main(int argc, char** argv) {
    const double secs = argc > 1 ? std::atof(argv[1]) : 20.0;
    std::vector<int> list;
    for (int i = 2; i < argc; ++i) list.push_back(std::atoi(argv[i]));
    if (list.empty()) for (int i = 0; i < presetCount(); ++i) list.push_back(i);
    int stalls = 0;
    for (int idx : list) {
        EngineParams p = makeCalibratedPreset(idx);
        if (const char* g = std::getenv("IDLEGAIN")) {   // diagnostics: scale the idle controller
            const float k = (float)std::atof(g);
            p.cal.idleKp *= k; p.cal.idleKi *= k; p.cal.idleKd *= k;
        }
        if (const char* g = std::getenv("SPARKGAIN")) p.cal.idleSparkGain *= (float)std::atof(g);
        p.ignition = 1; p.starter = 1;
        Simulator sim(p);
        std::vector<float> buf(480 * 2);
        Telemetry t;
        double caught = 0;
        for (int k = 0; k < 600 && caught < 0.15; ++k) {
            sim.render(buf.data(), 480); sim.getTelemetry(t);
            caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
        }
        p.starter = 0; sim.setParams(p);
        double rmin = 1e9, rmax = 0, stalledAt = -1;
        for (double tt = 0; tt < secs; tt += 0.01) {
            sim.render(buf.data(), 480); sim.getTelemetry(t);
            if (tt > 2.0) { rmin = std::min(rmin, (double)t.rpm); rmax = std::max(rmax, (double)t.rpm); }
            if (t.rpm < 250 && stalledAt < 0) { stalledAt = tt; break; }
        }
        if (stalledAt >= 0) ++stalls;
        std::printf("%-46.46s idle %5.0f..%5.0f rpm %s\n", p.name, rmin, rmax,
                    stalledAt >= 0 ? (std::string("  <-- STALLED at ") + std::to_string(stalledAt) + " s").c_str() : "");
    }
    std::printf("%d stalls\n", stalls);
    return stalls ? 1 : 0;
}

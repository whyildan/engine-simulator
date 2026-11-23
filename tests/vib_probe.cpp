// Engine vibration probe: what the mounts pass on (low / high band, m/s^2 RMS) for engines held at idle, mid-range
// and near their limit - to see that a single shakes more than a V12, an I4's secondary grows with speed, etc.
// usage: vib_probe [preset ...]
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
using namespace es;

static void hold(EngineParams p, double rpm, double throttle, double& lo, double& hi) {
    const double fs = 48000.0, dt = 1.0 / fs, w = rpm * 2.0 * kPi / 60.0;
    p.ignition = 1;
    auto e = std::make_unique<Engine>(p, fs);
    e->setCrank(w, 0.0);
    lo = hi = 0;
    int n = 0;
    for (double t = 0; t < 4.0; t += dt) {
        e->setCrank(w, e->theta());
        e->step(dt, throttle, 0.0);
        if (t > 2.5) { lo += e->vib.low(); hi += e->vib.high(); ++n; }
    }
    lo /= n; hi /= n;
}

int main(int argc, char** argv) {
    std::vector<int> ids;
    for (int i = 1; i < argc; ++i) ids.push_back(std::atoi(argv[i]));
    if (ids.empty()) ids = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 24, 25};
    std::printf("%-46s %22s %22s %22s\n", "", "idle (lo / hi)", "mid, half (lo / hi)", "top, full (lo / hi)");
    for (int id : ids) {
        const EngineParams p = makeCalibratedPreset(id);
        const double top = 0.9 * std::min((double)p.revLimit, (double)p.sweepEndRpm), mid = 0.5 * top;
        double a, b, c, d, e, f;
        hold(p, p.idleTargetRpm, 0.0, a, b);
        hold(p, mid, 0.5, c, d);
        hold(p, top, 1.0, e, f);
        std::printf("%-46s %5.0f: %6.2f / %6.2f  %5.0f: %6.2f / %6.2f  %5.0f: %6.2f / %6.2f\n", presetName(id), (double)p.idleTargetRpm, a, b, mid, c,
                    d, top, e, f);
    }
    return 0;
}

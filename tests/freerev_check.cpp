// Diagnostic: torque measured during a full-throttle free rev (inertia-dyno style) should match
// the steady-state dyno curve. Prints torque binned by 500 rpm during the rev.
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>

using namespace es;

int main(int argc, char** argv) {
    int preset = argc > 1 ? std::atoi(argv[1]) : 0;
    EngineParams p = makeCalibratedPreset(preset);
    p.ignition = 1; p.starter = 1;
    Engine e(p, 96000);
    const double dt = 1.0 / 96000;
    for (int k = 0; k < 96000; ++k) e.step(dt, 0, 0);
    p.starter = 0; e.apply(p);
    for (int k = 0; k < 96000 * 2; ++k) e.step(dt, 0, 0);
    double sum[40] = {}; int n[40] = {};
    double t0 = 0;
    for (int k = 0; k < 96000 * 3; ++k) {
        e.step(dt, 1.0, 0);
        if (e.limiterActive()) break;
        int b = (int)(e.rpm() / 500);
        if (b >= 0 && b < 40) { sum[b] += e.netTorque(); n[b]++; }
        t0 = k * dt;
    }
    std::printf("[%s] free rev to the limiter in %.2f s\n", p.name, t0);
    for (int b = 0; b < 40; ++b)
        if (n[b] > 50) std::printf("  %5d rpm  torque %6.1f N m\n", b * 500 + 250, sum[b] / n[b]);
}

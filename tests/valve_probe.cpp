// Valvetrain dynamics of a preset: dynamic vs static float speed, and loft / bounce / severity over rpm.
// usage: valve_probe [preset] [spring scale]
#include "sim/EngineParams.h"
#include "sim/Valvetrain.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace es;

int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 0;
    EngineParams p = makePreset(preset);
    if (argc > 2) { const float k = (float)std::atof(argv[2]); p.comp.springOpen_N *= k; p.comp.springSeat_N *= k; }
    ComponentModel cm; cm.build(p);
    for (int in = 1; in >= 0; --in) {
        const ValvetrainSpec vt = valvetrainSpec(p, in != 0);
        const bool hi = std::getenv("HI") != nullptr;
        const double lift = (hi ? (in ? p.vtecInLift_mm : p.vtecExLift_mm) : (in ? p.intakeLift_mm : p.exhaustLift_mm)) * 1e-3;
        const double dur = hi ? (in ? p.vtecInDuration : p.vtecExDuration) : (in ? p.intakeDuration : p.exhaustDuration);
        std::printf("[%s] %s: mass %.0f g, preload %.0f N, rate %.0f N/mm, train %.1f kN/mm | static float %.0f, dynamic %.0f rpm (limiter %.0f)\n",
                    p.name, in ? "intake" : "exhaust", vt.mass * 1e3, vt.preload, vt.rate * 1e-3, vt.stiffness * 1e-6,
                    in ? cm.floatRpmIn : cm.floatRpmEx, dynamicFloatRpm(vt, lift, dur), p.revLimit);
        for (double rpm = 2000; rpm <= 13000; rpm += 1000) {
            ValveDynamics d;
            d.compute(vt, lift, dur, rpm);
            double mx = 0, at = 0;
            for (double a = -dur / 2 - 10; a < dur / 2 + 60; a += 0.5) if (std::fabs(d.dev(a)) > mx) { mx = std::fabs(d.dev(a)); at = a; }
            if (std::getenv("WHERE")) std::printf("   (max dev at %.1f deg: %+.3f mm)\n", at, d.dev(at) * 1e3);
            std::printf("  %6.0f rpm  max dev %.3f mm  loft %.3f mm  bounce %.3f mm  late %4.0f deg  severity %.2f\n", rpm, mx * 1e3,
                        d.loft * 1e3, d.bounce * 1e3, d.lateClose, d.severity(lift));
        }
    }
    return 0;
}

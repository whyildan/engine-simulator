// Diagnostic: with the engine stopped every sound source must be silent.
// Prints the RMS of each source (Pa @ 1 m) for every preset after 2 s at rest.
#include "sim/Engine.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

using namespace es;

int main() {
    int bad = 0;
    for (int i = 0; i < presetCount(); ++i) {
        EngineParams p = makePreset(i);
        p.ignition = 0; p.starter = 0; p.throttle = 0;
        if (const char* s = std::getenv("STEEP")) p.exhaustSteep = (float)std::atof(s);   // diagnostics
        if (const char* s = std::getenv("RASP")) p.exhaustRasp = (float)std::atof(s);
        Engine e(p, 96000);
        const double dt = 1.0 / 96000;
        double s[4] = {0, 0, 0, 0}; int n = 0;
        for (int k = 0; k < 96000 * 2; ++k) {
            e.step(dt, 0, 0);
            if (k >= 96000) {
                double v[4] = {e.sExhL, e.sExhR, e.sIntake, e.sMech};
                for (int j = 0; j < 4; ++j) s[j] += v[j] * v[j];
                ++n;
            }
        }
        double r[4];
        for (int j = 0; j < 4; ++j) r[j] = std::sqrt(s[j] / n);
        bool loud = r[0] > 1e-4 || r[1] > 1e-4 || r[2] > 1e-4 || r[3] > 1e-4;
        bad += loud;
        std::printf("%-44s exhL %.2e exhR %.2e intake %.2e mech %.2e boost %.4f bar%s\n", p.name, r[0], r[1], r[2], r[3],
                    e.boost.boostBar, loud ? "   <-- NOISE" : "");
    }
    return bad ? 1 : 0;
}

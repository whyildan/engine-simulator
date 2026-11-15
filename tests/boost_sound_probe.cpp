// Diagnostic: level of the forced-induction sound vs the other sources, at idle, under boost,
// and right after lifting off (BOV / surge). Holds the crank speed fixed to isolate the sound.
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

using namespace es;

static void phase(Engine& e, const char* name, double rpm, double pedal, double secs, bool hold) {
    const double dt = 1.0 / 96000;
    double s[5] = {0, 0, 0, 0, 0};
    double hp = 0, hpPrev = 0, hf = 0; // high-frequency content of the boost sound (>~4 kHz)
    int n = 0;
    for (int k = 0; k < (int)(secs * 96000); ++k) {
        if (hold) e.setCrank(rpm * 2 * kPi / 60, e.theta());
        e.step(dt, pedal, 0);
        double b = e.sBoost;
        double v[5] = {e.sExhL, e.sIntake, b, e.sMech, 0};
        hp = 0.75 * (hp + b - hpPrev); hpPrev = b; // crude 1-pole high pass ~4 kHz @ 96 kHz
        hf += hp * hp;
        for (int j = 0; j < 4; ++j) s[j] += v[j] * v[j];
        ++n;
    }
    std::printf("%-22s rpm %5.0f boost %+5.2f bar shaft %6.0f surge %d bov %d | exh %.3f intake %.3f FI %.3f (hf %.3f) mech %.3f Pa\n",
                name, e.rpm(), e.boost.boostBar, e.boost.shaftRpm, e.boost.surging, e.boost.bovOpen,
                std::sqrt(s[0] / n), std::sqrt(s[1] / n), std::sqrt(s[2] / n), std::sqrt(hf / n), std::sqrt(s[3] / n));
}

int main(int argc, char** argv) {
    int preset = argc > 1 ? std::atoi(argv[1]) : 16;
    int bov = argc > 2 ? std::atoi(argv[2]) : -1;
    EngineParams p = makeCalibratedPreset(preset);
    if (bov >= 0) p.bov = bov;
    p.ignition = 1; p.starter = 1;
    Engine e(p, 96000);
    phase(e, "crank", 0, 0, 1.0, false);
    p.starter = 0; e.apply(p);
    phase(e, "idle", 0, 0, 3.0, false);
    phase(e, "idle (measure)", 0, 0, 1.0, false);
    phase(e, "WOT 3500 spooling", 3500, 1, 1.5, true);
    phase(e, "WOT 5000 boost", 5000, 1, 1.5, true);
    phase(e, "lift-off 0-0.3 s", 5000, 0, 0.3, true);
    phase(e, "lift-off 0.3-1 s", 4500, 0, 0.7, true);
    return 0;
}

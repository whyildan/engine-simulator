// Engine vibration (sim/Vibration.h) ranks engines the way their balance says: a V-twin shakes far more than a V12 at
// idle; an inline-4's secondary buzz grows with speed while a straight six stays smooth; the counterweights balance a
// crossplane V8's primary couple; a stopped engine is still.
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Vib { double lo, hi, swing; };   // swing: the block's peak-to-peak motion over the cycle at the mounts (mm)
// held at this speed and throttle; the mean of the last 1.5 s of 4
static Vib hold(int id, double rpm, double throttle, bool ignition = true) {
    EngineParams p = makeCalibratedPreset(id);
    const double fs = 48000.0, dt = 1.0 / fs, w = rpm * 2.0 * kPi / 60.0;
    p.ignition = ignition ? 1 : 0;
    auto e = std::make_unique<Engine>(p, fs);
    e->setCrank(w, 0.0);
    Vib v{0, 0, 0};
    int n = 0;
    for (double t = 0; t < 4.0; t += dt) {
        e->setCrank(w, e->theta());
        e->step(dt, throttle, 0.0);
        if (t > 2.5) { v.lo += e->vib.low(); v.hi += e->vib.high(); ++n; }
    }
    v.lo /= n; v.hi /= n;
    {   // the recorded cycle: lateral / vertical and the roll as motion at the 0.25 m mounts
        const int nb = std::min(EngineVibration::kBins, (int)cycleDeg(p));
        double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9, r0 = 1e9, r1 = -1e9;
        for (int k = 0; k < nb; ++k) {
            x0 = std::min(x0, (double)e->vib.trX[k]); x1 = std::max(x1, (double)e->vib.trX[k]);
            y0 = std::min(y0, (double)e->vib.trY[k]); y1 = std::max(y1, (double)e->vib.trY[k]);
            r0 = std::min(r0, (double)e->vib.trRoll[k]); r1 = std::max(r1, (double)e->vib.trRoll[k]);
        }
        v.swing = 1000.0 * std::max({x1 - x0, y1 - y0, 0.25 * (r1 - r0)});
    }
    std::printf("    %-40s %5.0f rpm thr %.1f: low %.3f  high %.3f m/s^2  block swing %.3f mm\n", presetName(id), rpm, throttle, v.lo, v.hi, v.swing);
    return v;
}

int main() {
    const int I4 = 0, I6 = 2, V8X = 4, V12 = 7, VTWIN = 9;
    const Vib twinIdle = hold(VTWIN, 850, 0), v12Idle = hold(V12, 1000, 0);
    CHECK(twinIdle.lo > 5.0 * v12Idle.lo, "a V-twin shakes far more than a V12 at idle (%.2f vs %.2f)", twinIdle.lo, v12Idle.lo);
    // (the engine view replays this motion: a rubber-mounted big twin visibly rocks at idle, a V12 does not)
    CHECK(twinIdle.swing > 3.0 * v12Idle.swing && twinIdle.swing < 6.0, "the V-twin's block swings more than the V12's, a few mm at most (%.3f vs %.3f mm)",
          twinIdle.swing, v12Idle.swing);
    const Vib i4Mid = hold(I4, 3500, 0.5), i4Top = hold(I4, 7500, 1.0), i6Top = hold(I6, 6300, 1.0);
    CHECK(i4Top.hi > 1.5 * i4Mid.hi, "an I4's secondary buzz grows with speed (%.2f -> %.2f)", i4Mid.hi, i4Top.hi);
    CHECK(i4Top.hi > 5.0 * i6Top.hi, "a straight six stays smooth where an I4 buzzes (%.2f vs %.2f)", i6Top.hi, i4Top.hi);
    const Vib v8Top = hold(V8X, 5500, 1.0);
    CHECK(v8Top.hi < 0.3 * i4Top.hi, "a crossplane V8's primary couple is balanced by its counterweights (%.2f)", v8Top.hi);
    const Vib off = hold(I4, 0.0, 0, false);
    CHECK(off.lo < 1e-3 && off.hi < 1e-3, "a stopped engine is still");
    for (const Vib& v : {twinIdle, v12Idle, i4Mid, i4Top, i6Top, v8Top, off})
        if (!std::isfinite(v.lo) || !std::isfinite(v.hi)) { ++failures; std::printf("  FAIL: not finite\n"); }
    std::printf(failures ? "\n%d FAILED\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}

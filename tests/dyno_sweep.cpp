// WOT dyno sweep of a preset: torque / power / AFR / spark / VTEC / VE over rpm, plus the peaks.
// usage: dyno_sweep [preset]
// tuning overrides (env): LO_DUR LO_LIFT HI_DUR HI_LIFT ICL VTC_SCALE RUNNER RUNNER_DIA PRIM THROTTLE MECH
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;

static void run(Simulator& sim, double s) {
    static std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static bool env(const char* k, float& v) { if (const char* e = std::getenv(k)) { v = (float)std::atof(e); return true; } return false; }

int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 0;
    EngineParams p = makePreset(preset);
    bool changed = false;
    float v;
    if (env("LO_DUR", v)) { p.intakeDuration = v; p.exhaustDuration = v - 4; changed = true; }
    if (env("LO_LIFT", v)) { p.intakeLift_mm = v; p.exhaustLift_mm = v - 0.4f; changed = true; }
    if (env("HI_DUR", v)) { p.vtecInDuration = v; p.vtecExDuration = v - 6; changed = true; }
    if (env("HI_LIFT", v)) { p.vtecInLift_mm = v; p.vtecExLift_mm = v - 0.7f; changed = true; }
    if (env("ICL", v)) { p.intakeCenterline = v; changed = true; }
    if (env("RUNNER", v)) { p.runnerLen_mm = v; changed = true; }
    if (env("RUNNER_DIA", v)) { p.runnerDia_mm = v; changed = true; }
    if (env("PRIM", v)) { for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = v; changed = true; }
    if (env("THROTTLE", v)) { p.throttleBore_mm = v; changed = true; }
    if (env("MECH", v)) { p.mechAdvance = v; changed = true; }
    if (changed) generateCalibration(p);
    if (env("VTC_SCALE", v)) for (auto& row : p.cal.vtcMap.v) for (float& x : row) x *= v;
    Calibration c = p.cal;
    if (!loadCachedBaseMap(p, c)) { measureBaseMap(p, c); saveCachedBaseMap(p, c); }
    p.cal.veMap = c.veMap; p.cal.veTps = c.veTps;
    Simulator sim(p);
    p.ignition = 1; p.starter = 1; sim.setParams(p); run(sim, 1.2);
    p.starter = 0; sim.setParams(p); run(sim, 3.0);
    std::printf("[%s] sweep %.0f..%.0f rpm\n", p.name, p.sweepStartRpm, p.sweepEndRpm);
    std::printf("   rpm  torque Nm   power kW (PS)   AFR    adv   VTEC  VTC   MAP    VE   knock  protection\n");
    double pkT = 0, pkTr = 0, pkP = 0, pkPr = 0;
    const double step = 500;
    for (double r = p.sweepStartRpm; r <= p.sweepEndRpm + 1; r += step) {
        p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)r; p.throttle = 1; sim.setParams(p);
        run(sim, 1.0);
        double tq = 0, afr = 0, adv = 0, vt = 0, vc = 0, map = 0, ve = 0, kn = 0; int n = 0;
        Telemetry last;
        for (int k = 0; k < 10; ++k) {
            run(sim, 0.05); Telemetry t; sim.getTelemetry(t);
            tq += t.torque; afr += t.afr; adv += t.advance; vt += t.vtecEngaged; vc += t.vtcAdvance; map += t.mapKPa; ve += t.ve; kn += t.knock; ++n;
            last = t;
        }
        tq /= n; afr /= n; adv /= n; vt /= n; vc /= n; map /= n; ve /= n; kn /= n;
        const double kw = tq * r * 2 * 3.14159265 / 60.0 / 1000.0;
        if (tq > pkT) { pkT = tq; pkTr = r; }
        if (kw > pkP) { pkP = kw; pkPr = r; }
        std::printf("  %5.0f  %7.1f   %6.1f (%5.1f)  %5.2f  %5.1f  %4.2f  %4.1f  %5.1f  %5.1f  %4.2f ", r, tq, kw, kw * 1.3596, afr, adv, vt, vc, map, ve * 100, kn);
        for (int k = 0; k < kProtCount; ++k) if (last.protState[k]) std::printf(" %s=%d", protInfo(k).key, last.protState[k]);
        std::printf("%s\n", last.limp ? " LIMP" : "");
        // a short cool-down at light load between the points (a real dyno pull does not hold WOT for a minute)
        p.dynoHoldRpm = 2500; p.throttle = 0.1f; sim.setParams(p); run(sim, 1.5);
    }
    std::printf("peak torque %.0f Nm @ %.0f rpm, peak power %.1f kW (%.0f PS) @ %.0f rpm\n", pkT, pkTr, pkP, pkP * 1.3596, pkPr);
    return 0;
}

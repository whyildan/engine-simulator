// Diagnostic: the egt_probe sequence (start, idle, 3000 rpm hold at 30 %, WOT hold at 0.8 x limit) on one preset,
// printing the operating point every 0.25 s.   usage: op_trace [preset]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;

static void run(Simulator& s, double sec) { std::vector<float> b(480 * 2); for (double t = 0; t < sec; t += 0.01) s.render(b.data(), 480); }
static Telemetry tel(Simulator& s) { Telemetry t; s.getTelemetry(t); return t; }
static double T = 0;
static void trace(Simulator& s, double sec, const char* tag, double dt = 0.25) {
    for (double t = 0; t < sec; t += dt) {
        run(s, dt); T += dt;
        Telemetry x = tel(s);
        std::printf("%-5s t %5.2f rpm %5.0f run %d MAP %5.1f AFR %5.2f tgt %5.2f adv %5.1f VE %5.1f air %6.2f g/s fuel %5.2f g/s duty %4.1f "
                    "trim %4.2f cl %d limp %d shut %d cut %.2f thrCap %.2f prot",
                    tag, T, x.rpm, x.running, x.mapKPa, x.afr, x.targetAfr, x.advance, x.ve, x.airFlow * 1e3, x.fuelFlow * 1e3, x.injDuty,
                    x.ecuTrim, x.closedLoop, x.limp, x.shutdown, x.cutFraction, x.tqThrCap);
        for (int k = 0; k < kProtCount; ++k) if (x.protState[k]) std::printf(" %d:%d", k, x.protState[k]);
        const Engine& e = *s.engineForTests();
        std::printf("  learn %d tps %.1f mis %d lam", x.learnSeq, x.snsValue[kSnsTPS], x.misfires);
        for (int c = 0; c < x.cylinders; ++c) std::printf(" %.2f", e.cyl(c).lambda);
        std::printf("\n");
    }
}

int main(int argc, char** argv) {
    const int idx = argc > 1 ? std::atoi(argv[1]) : 11;
    EngineParams p = makeCalibratedPreset(idx);
    std::printf("%s\n", p.name);
    p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    double caught = 0;
    for (int k = 0; k < 600 && caught < 0.15; ++k) {
        run(sim, 0.01); T += 0.01; Telemetry t = tel(sim);
        caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
    }
    p.starter = 0; sim.setParams(p);
    trace(sim, 10.0, "idle");
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 3000; p.throttle = 0.3f; sim.setParams(p);
    trace(sim, 1.0, "3000", 0.05);
    trace(sim, 5.0, "3000");
    p.dynoHoldRpm = p.revLimit * 0.8f; p.throttle = 1.0f; sim.setParams(p);
    trace(sim, 6.0, "WOT");
}

// diesel probe: smoke limiter air estimate against the trapped air (D6), and a low compression ratio (D7)
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace es;

static void run(Simulator& sim, double s) {
    static std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }
static void start(Simulator& sim, EngineParams& p) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (double t = 0; t < 4.0; t += 0.01) { run(sim, 0.01); Telemetry x = tel(sim); if (x.running && x.rpm > 500) break; }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
}

int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 24;
    EngineParams p = makeCalibratedPreset(preset);
    std::printf("[smoke] %s, smoke lambda %.2f\n", p.name, p.cal.smokeLambda);
    for (float frac : {0.3f, 0.45f, 0.6f, 0.75f, 0.85f, 0.93f}) {
        const float rpm = std::round(frac * p.revLimit / 50.0f) * 50.0f;
        EngineParams q = p;
        Simulator sim(q);
        start(sim, q);
        q.dynoMode = DynoMode::Hold; q.dynoHoldRpm = rpm; q.throttle = 1.0f; sim.setParams(q);
        run(sim, 5.0);
        double lam = 0, maf = 0, trap = 0, fuel = 0; int n = 0; int lim = 0;
        for (int k = 0; k < 20; ++k) {
            run(sim, 0.05);
            const Engine& e = *sim.engineForTests();
            const Telemetry t = tel(sim);
            lam += e.cyl(0).lambda; maf += t.snsValue[kSnsMAF]; trap += e.cyl(0).trappedAir; fuel += e.ecu.cmd.dieselFuelMg + e.ecu.cmd.pilotMg;
            lim += e.ecu.smokeLimited; ++n;
        }
        lam /= n; maf /= n; trap /= n; fuel /= n;
        const int cyl = p.cylinders;
        static Telemetry T; sim.getTelemetry(T);
        const double perStroke = maf * 1e-3 / (cyl * rpm / 120.0);   // g/s -> kg per intake stroke
        std::printf("  %5.0f rpm  lambda %.3f  limited %2d/20  fuel %.1f mg  MAF/stroke %.1f mg  trapped air %.1f mg  ratio %.3f  boost %.2f  rpm %.0f Nm %.0f rail %.0f pump %.1f kW emap %.0f vane %.2f egr %.2f run %d\n",
                    rpm, lam, lim, fuel, perStroke * 1e6, trap * 1e6, trap / std::max(1e-12, perStroke), T.boostBar, T.rpm, T.torque, T.railBar, T.pumpKw, T.emapKPa, T.wastegate, T.turbUc * 100.0f + T.turbEff, (int)T.running);
    }
    // D7: low compression ratio - does it run?
    for (float cr : {9.5f, 12.0f, 14.0f, 16.0f}) {
        EngineParams q = p;
        q.compression = cr;
        Simulator sim(q);
        start(sim, q);
        run(sim, 4.0);
        const Engine& e = *sim.engineForTests();
        std::printf("[CR %.1f] rpm %.0f running %d misfires %d  T charge cyl0 %.0f K\n", cr, tel(sim).rpm, (int)tel(sim).running, e.misfires, e.cyl(0).T);
    }
    return 0;
}

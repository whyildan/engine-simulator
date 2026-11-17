// EGT behaviour of a preset: idle EGT, then a free rev to near the redline and a lift-off, traced every 20 ms
// (EGT sensor, gas leaving the cylinders, afterfire heat, exhaust flow, lambda, fuel cut, pops).
// usage: egt_trace [preset] [idle seconds]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;

static void run(Simulator& sim, double s) {
    static std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }

int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 0;
    const double idleS = argc > 2 ? std::atof(argv[2]) : 20.0;
    EngineParams p = makeCalibratedPreset(preset);
    Simulator sim(p);
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 600; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, idleS - 3.0);
    const Engine& e = *sim.engineForTests();
    double egt = 0, cyl = 0, gas = 0, rpm = 0, adv = 0, lam = 0; int n = 0;
    for (int k = 0; k < 60; ++k) {
        run(sim, 0.05); Telemetry t = tel(sim);
        egt += t.egt; cyl += e.egtCyl - 273.15; gas += t.egtGas; rpm += t.rpm; adv += t.advance; lam += t.afr / 14.7; ++n;
    }
    std::printf("[%s] idle after %.0f s: EGT %.0f C (gas %.0f C, leaving the cylinders %.0f C), rpm %.0f, spark %.1f deg, lambda %.2f, coolant %.0f C\n",
                p.name, idleS, egt / n, gas / n, cyl / n, rpm / n, adv / n, lam / n, tel(sim).coolantC);
    // WOT on the dyno at 75 % of the limiter
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 0.75f * p.revLimit; p.throttle = 1; sim.setParams(p);
    run(sim, 3.0);
    egt = cyl = gas = lam = 0; n = 0;
    for (int k = 0; k < 20; ++k) { run(sim, 0.05); Telemetry t = tel(sim); egt += t.egt; cyl += e.egtCyl - 273.15; gas += t.egtGas; lam += t.afr / 14.7; ++n; }
    std::printf("  WOT %.0f rpm: EGT %.0f C (gas %.0f C, leaving the cylinders %.0f C), lambda %.2f, exhaust %.0f g/s\n", p.dynoHoldRpm, egt / n, gas / n, cyl / n, lam / n, e.exhFlowGs());
    {
        double te = 0;
        for (int c = 0; c < e.cylinders(); ++c) te += e.cyl(c).tEvo;
        std::printf("    afterfire %.1f kW at WOT, gas at exhaust valve opening %.0f C\n", e.afterfireKw(), te / e.cylinders() - 273.15);
    }
    p.dynoMode = DynoMode::Off; p.throttle = 0; sim.setParams(p);
    run(sim, 3.0);
    // free rev towards the redline, then lift
    p.throttle = 1; sim.setParams(p);
    const float top = redlineRpm(p) - 200;
    for (int k = 0; k < 400 && tel(sim).rpm < top; ++k) run(sim, 0.01);
    std::printf("  lift at %.0f rpm\n   t      rpm  EGT  gas  cyl  afterfire kW  exh g/s  lambda  cut  pops  adv\n", tel(sim).rpm);
    p.throttle = 0; sim.setParams(p);
    const int pops0 = tel(sim).afterfires;
    for (int k = 0; k < 75; ++k) {
        run(sim, 0.02); Telemetry t = tel(sim);
        if (k % 2 == 0 || k < 20)
            std::printf("  %4.2f  %6.0f  %4.0f %4.0f %4.0f  %10.1f  %7.1f  %6.2f  %3d  %4d  %5.1f\n", 0.02 * (k + 1), t.rpm, t.egt, t.egtGas, e.egtCyl - 273.15,
                        e.afterfireKw(), e.exhFlowGs(), t.afr / 14.7, t.targetAfr <= 0 ? 1 : 0, t.afterfires - pops0, t.advance);
    }
    return 0;
}

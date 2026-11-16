// EGT probe: exhaust gas temperature at a warm idle, at part load and at full load, per preset.
// usage: egt_probe [preset ...]   (no presets: all)
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;

static void run(Simulator& s, double sec) { std::vector<float> b(480 * 2); for (double t = 0; t < sec; t += 0.01) s.render(b.data(), 480); }
static Telemetry tel(Simulator& s) { Telemetry t; s.getTelemetry(t); return t; }
// combustion detail of cylinder 0 over a few cycles: spark-to-50 % burn angle (CA50 relative to firing TDC),
// burn duration, cylinder temperature at exhaust opening and the mass-weighted temperature of the gas leaving
static void detail(Simulator& s, const char* what) {
    std::vector<float> b(48 * 2);
    const Engine& e = *s.engineForTests();
    const double cyc = e.cycle();
    double ca50 = 0, tEvo = 0, dur = 0, start = 0; int n50 = 0, nEvo = 0;
    double xPrev = 0; bool exPrev = e.cyl(0).prevExhaustOpen;
    for (int k = 0; k < 3000; ++k) {
        s.render(b.data(), 48);
        const Cylinder& c = e.cyl(0);
        if (c.burning && xPrev < 0.5 && c.xPrev >= 0.5) {
            double a = c.prevPhi; if (a > cyc / 2) a -= cyc;   // degrees after firing TDC
            double st = c.burnStart; if (st > cyc / 2) st -= cyc;
            ca50 += a; dur += c.burnDur; start += st; ++n50;
        }
        if (c.prevExhaustOpen && !exPrev) { tEvo += c.T; ++nEvo; }
        xPrev = c.burning ? c.xPrev : 0; exPrev = c.prevExhaustOpen;
    }
    std::printf("    %-6s burn start %6.1f  retard %4.1f  CA50 %5.1f deg ATDC  burn %5.1f deg  T at exhaust opening %5.0f C  (egt %5.0f C)\n", what,
                n50 ? start / n50 : 0, e.ecu.cmd.cylRetard[0], n50 ? ca50 / n50 : 0, n50 ? dur / n50 : 0, nEvo ? tEvo / nEvo - 273.15 : 0, e.egtAvg - 273.15);
}

static double avgEgt(Simulator& s, double sec, double& adv, double& afr) {
    double e = 0, a = 0, f = 0; int n = 0;
    for (double t = 0; t < sec; t += 0.05) { run(s, 0.05); Telemetry x = tel(s); e += x.egt; a += x.advance; f += x.afr; ++n; }
    adv = a / n; afr = f / n; return e / n;
}

int main(int argc, char** argv) {
    std::vector<int> list;
    for (int i = 1; i < argc; ++i) list.push_back(std::atoi(argv[i]));
    if (list.empty()) for (int i = 0; i < presetCount(); ++i) list.push_back(i);
    std::printf("%-44s %20s %20s %20s\n", "preset", "idle EGT/adv/AFR", "3000 rpm 30%", "WOT 0.8 x limit");
    for (int idx : list) {
        EngineParams p = makeCalibratedPreset(idx);
        p.ignition = 1; p.starter = 1;
        Simulator sim(p);
        double caught = 0;
        for (int k = 0; k < 600 && caught < 0.15; ++k) {
            run(sim, 0.01); Telemetry t = tel(sim);
            caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
        }
        p.starter = 0; sim.setParams(p);
        run(sim, 8.0);
        double a0, f0, a1, f1, a2, f2;
        const double e0 = avgEgt(sim, 2.0, a0, f0);
        if (std::getenv("DETAIL")) detail(sim, "idle");
        p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 3000; p.throttle = 0.3f; sim.setParams(p); run(sim, 4.0);
        const double e1 = avgEgt(sim, 2.0, a1, f1);
        p.dynoHoldRpm = p.revLimit * 0.8f; p.throttle = 1.0f; sim.setParams(p); run(sim, 4.0);
        const double e2 = avgEgt(sim, 2.0, a2, f2);
        if (std::getenv("DETAIL")) detail(sim, "WOT");
        std::printf("%-44.44s %5.0f C %4.1f %5.2f   %5.0f C %4.1f %5.2f   %5.0f C %4.1f %5.2f\n", p.name, e0, a0, f0, e1, a1, f1, e2, a2, f2);
    }
}

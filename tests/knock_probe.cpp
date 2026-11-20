// Diagnostic: the knock and flame state over load and speed - end-gas temperature and pressure at their peak, the
// knock index and the retard the knock control pulls, the laminar burning velocity, unburned T / p at the spark and
// the burn duration, per operating point.
// usage: knock_probe [preset] [fuel type]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;

static void run(Simulator& s, double sec) { std::vector<float> b(480 * 2); for (double t = 0; t < sec; t += 0.01) s.render(b.data(), 480); }
static Telemetry tel(Simulator& s) { Telemetry t; s.getTelemetry(t); return t; }

int main(int argc, char** argv) {
    const int idx = argc > 1 ? std::atoi(argv[1]) : 0;
    EngineParams p = makeCalibratedPreset(idx);
    if (argc > 2) p.fuelType = std::atoi(argv[2]);
    Simulator sim(p);
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (double t = 0; t < 4.0; t += 0.01) {
        run(sim, 0.01);
        Telemetry x = tel(sim);
        if (x.running && x.rpm > starterReleaseRpm(p, x.snsValue[kSnsECT])) break;
    }
    run(sim, 0.15); p.starter = 0; sim.setParams(p); run(sim, 1.5);
    std::printf("%s\n  rpm  thr    MAP   adv | pIvc  tIvc | endgas T   p bar | index retard\n", presetName(idx));
    const double rpms[] = {2000, 3000, 4500, 6000};
    const double thrs[] = {0.08, 0.15, 0.3, 0.5, 1.0};
    for (double rpm : rpms) {
        if (rpm > p.revLimit - 300) continue;
        for (double thr : thrs) {
            p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = (float)thr; sim.setParams(p);
            run(sim, 4.0);
            double tk = 0, pk = 0, pi = 0, ti = 0, kn = 0, rt = 0; int n = 0;
            double wT = 0, wP = 0, wPi = 0, wTi = 0;   // the worst cycle seen
            double sl = 0, tuS = 0, pS = 0, dur = 0;
            for (int s = 0; s < 20; ++s) {
                run(sim, 0.1);
                const Engine& e = *sim.engineForTests();
                const Telemetry x = tel(sim);
                for (int c = 0; c < e.cylinders(); ++c) {
                    if (e.cyl(c).knockT > wT) { wT = e.cyl(c).knockT; wP = e.cyl(c).knockP; wPi = e.cyl(c).pIvc; wTi = e.cyl(c).tIvc; }
                    tk = std::max(tk, e.cyl(c).knockT); pk += e.cyl(c).knockP; pi += e.cyl(c).pIvc; ti += e.cyl(c).tIvc; ++n;
                    kn = std::max(kn, e.cyl(c).knock);
                    sl += e.cyl(c).slSpark; tuS += e.cyl(c).tuSpark; pS += e.cyl(c).pSpark; dur += e.cyl(c).burnDur;
                }
                for (int c = 0; c < x.cylinders; ++c) rt = std::max(rt, (double)x.knockRetard[c]);
            }
            const Telemetry x = tel(sim);
            std::printf("%5.0f %4.2f %6.1f %5.1f | %4.2f %5.0f | %8.0f %6.1f | %5.2f %5.1f | S_L %5.1f cm/s Tu %4.0f K p %5.1f bar | burn %5.1f deg\n", rpm,
                        thr, x.mapKPa, x.advance, pi / n / 1e5, ti / n, tk, pk / n / 1e5, kn, rt, sl / n, tuS / n, pS / n / 1e5, dur / n);
        }
    }
}

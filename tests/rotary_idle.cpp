// Idle quality probe (rotaries and others): every combustion of a warm idling engine - IMEP, lambda, dilution,
// share of the charge burned, misfire - and the summary a tuner looks at (COV of IMEP, misfire share, rpm swing).
// usage: rotary_idle [preset] [porting -1 = preset's own / 0..3] [seconds]     env VERBOSE: print each combustion
#include "sim/Simulator.h"
#include "sim/EngineParams.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;

int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 12;
    const int porting = argc > 2 ? std::atoi(argv[2]) : -1;
    const double secs = argc > 3 ? std::atof(argv[3]) : 4.0;
    EngineParams p = makeCalibratedPreset(preset);
    if (porting >= 0) applyRotaryPorting(p, porting);
    p.ignition = 1;
    // the real simulator, started like the app does (starter until it runs on its own)
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    Telemetry t;
    p.starter = 1; sim.setParams(p);
    double caught = 0;
    for (int k = 0; k < 600 && caught < 0.15; ++k) {
        sim.render(buf.data(), 480); sim.getTelemetry(t);
        caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
    }
    p.starter = 0; sim.setParams(p);
    const double settle = std::getenv("SETTLE") ? std::atof(std::getenv("SETTLE")) : 4.0;
    for (double tt = 0; tt < settle; tt += 0.01) sim.render(buf.data(), 480);
    const Engine& e = *sim.engineForTests();
    const double dt = 0.01;
    const int n = e.cylinders();
    std::vector<int> seen(n);
    for (int i = 0; i < n; ++i) seen[i] = e.cyl(i).cycles;
    std::vector<double> imep, dils;
    int mis = 0, fired = 0, partial = 0;
    double rmin = 1e9, rmax = 0;
    const bool verbose = std::getenv("VERBOSE") != nullptr;
    for (int k = 0; k < (int)(secs / dt); ++k) {
        sim.render(buf.data(), 480);
        rmin = std::min(rmin, e.rpm()); rmax = std::max(rmax, e.rpm());
        if (std::getenv("RPMTRACE") && k % 25 == 0) {
            Telemetry tt; sim.getTelemetry(tt);
            std::printf("  t %5.2f rpm %5.0f map %5.1f iac %4.0f adv %5.1f afr %5.2f tgt %5.2f mis %d trim %+.1f%%\n", k * dt, tt.rpm, tt.mapKPa, tt.iac, tt.advance,
                        tt.afr, tt.targetAfr, tt.misfires, (tt.ecuTrim - 1) * 100);
        }
        for (int i = 0; i < n; ++i) {
            const Cylinder& c = e.cyl(i);
            if (c.cycles == seen[i]) continue;
            seen[i] = c.cycles;
            imep.push_back(c.imep * 1e-5);
            dils.push_back(c.dilution);
            if (c.misfired) ++mis; else { ++fired; if (c.burnEta < 0.9) ++partial; }
            if (verbose && imep.size() < 120)
                std::printf("  t %.3f ch %d imep %6.2f bar  lam %.2f  dil %.2f  burn %.2f  %s\n", k * dt, i, c.imep * 1e-5, c.lambda,
                            c.dilution, c.burnEta, c.misfired ? "MISFIRE" : "");
        }
    }
    auto mean = [](const std::vector<double>& v) { double s = 0; for (double x : v) s += x; return s / std::max<size_t>(1, v.size()); };
    const double m = mean(imep);
    double sd = 0; for (double x : imep) sd += (x - m) * (x - m); sd = std::sqrt(sd / std::max<size_t>(1, imep.size()));
    std::printf("%s porting %d: rpm %.0f..%.0f  map %.0f kPa  dilution %.2f  misfire %.0f %%  partial %.0f %%  IMEP %.2f bar COV %.0f %%  afterfires %d\n",
                p.name, porting, rmin, rmax, e.intake.map() / 1000, mean(dils), 100.0 * mis / std::max(1, mis + fired),
                100.0 * partial / std::max(1, mis + fired), m, 100.0 * sd / std::max(1e-6, std::fabs(m)), e.afterfires);
}

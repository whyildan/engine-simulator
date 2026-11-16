// How long does the trapped air take to settle at one held operating point? Prints the state that could
// be slow (EGT, pipe gas temperature, IAT, coolant, film, lambda) every 0.25 s.
// usage: ve_settle [preset] [rpm] [throttle] [fromThrottle]
#include "sim/Engine.h"
#include "sim/EngineParams.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
using namespace es;

int main(int argc, char** argv) {
    int preset = argc > 1 ? std::atoi(argv[1]) : 5;
    double rpm = argc > 2 ? std::atof(argv[2]) : 1700, thr = argc > 3 ? std::atof(argv[3]) : 1.0;
    double from = argc > 4 ? std::atof(argv[4]) : 0.6;
    // env CAL: fuel from the measured base map (speed density) instead of the MAF - shows how well the map fits
    EngineParams p = std::getenv("CAL") ? makeCalibratedPreset(preset) : makePreset(preset);
    p.ignition = 1; p.cal.autoTune = 0; p.cal.closedLoop = 0;
    if (!std::getenv("CAL")) p.cal.loadMode = LoadMode::Maf;
    const double fs = 96000, dt = 1 / fs, omega = rpm * 2 * kPi / 60;
    Engine e(p, fs);
    e.ecu.holdIac = true;
    for (int k = 0; k < (int)(3 * fs); ++k) { e.setCrank(omega, e.theta()); e.step(dt, from, 0.0); }
    std::printf("   t    veAtm   lam    egtAvg  iat   coolant  map\n");
    for (int w = 0; w < 32; ++w) {
        double sVe = 0, sLam = 0; int n = 0;
        for (int k = 0; k < (int)(0.25 * fs); ++k) {
            e.setCrank(omega, e.theta()); e.step(dt, thr, 0.0);
            double ve = 0, lam = 0;
            for (int c = 0; c < e.cylinders(); ++c) { ve += e.cyl(c).ve; lam += e.cyl(c).lambda; }
            sVe += ve / e.cylinders(); sLam += lam / e.cylinders(); ++n;
        }
        std::printf("%5.2f  %6.3f  %5.3f  %6.0f  %5.1f  %5.1f  %6.1f\n", (w + 1) * 0.25, sVe / n, sLam / n, e.egtAvg,
                    e.thermal.iatC, e.thermal.coolantC, e.intake.map() / 1000);
    }
}

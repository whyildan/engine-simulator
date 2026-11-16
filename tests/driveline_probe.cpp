// Driveline probe: WOT in a fixed gear (car as the load); compares the instantaneous clutch torque
// with the cycle-averaged value the dyno chart shows, plus knock / misfire / AFR over the window.
// usage: driveline_probe [preset] [gear]      env IMP = imperfection override
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;

static void stats(const std::vector<double>& v, double& mean, double& sd) {
    mean = 0; for (double x : v) mean += x; mean /= v.size();
    sd = 0; for (double x : v) sd += (x - mean) * (x - mean); sd = std::sqrt(sd / v.size());
}

int main(int argc, char** argv) {
    int preset = argc > 1 ? std::atoi(argv[1]) : 0, gear = argc > 2 ? std::atoi(argv[2]) : 3;
    EngineParams p = makeCalibratedPreset(preset);
    if (const char* e = std::getenv("IMP")) p.imperfection = (float)std::atof(e);
    if (std::getenv("NOCL")) { p.cal.autoTune = 0; p.cal.closedLoop = 0; }   // open loop: the map alone
    p.vehicle = 1; p.transMode = TransMode::ManualAutoClutch; p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    std::vector<float> buf(48 * 2);
    auto run = [&](double s) { for (double t = 0; t < s; t += 0.001) sim.render(buf.data(), 48); };
    run(1.5); p.starter = 0; sim.setParams(p); run(1.5);
    for (int g = 1; g <= gear; ++g) { p.gear = g; p.throttle = 0.35f; sim.setParams(p); run(g == 1 ? 3.0 : 1.5); }
    p.throttle = 1; sim.setParams(p); run(1.0);
    if (const char* e = std::getenv("DYNO")) {   // engine alone on the dyno, held at the given rpm
        p.vehicle = 0; p.gear = 0; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)std::atof(e);
        sim.setParams(p); run(3.0);
    }
    Telemetry t;
    sim.getTelemetry(t);
    const int mis0 = t.misfires;
    std::vector<double> T, A, R, K, Adv, Afr;
    for (int i = 0; i < 2000; ++i) {
        run(0.001); sim.getTelemetry(t);
        T.push_back(t.clutchTq); A.push_back(t.torque); R.push_back(t.rpm);
        K.push_back(t.knock); Adv.push_back(t.advance); Afr.push_back(t.afr);
    }
    double m, sd, am, asd, km, ksd, dm, dsd, fm, fsd;
    stats(T, m, sd); stats(A, am, asd); stats(K, km, ksd); stats(Adv, dm, dsd); stats(Afr, fm, fsd);
    std::printf("gear %d  rpm %.0f..%.0f  instantaneous clutch T mean %.1f sd %.1f\n", gear, R.front(), R.back(), m, sd);
    std::printf("cycle-averaged (dyno display): mean %.1f sd %.1f  min %.0f max %.0f\n", am, asd,
                *std::min_element(A.begin(), A.end()), *std::max_element(A.begin(), A.end()));
    {   // spectrum of the rpm wobble (linear trend removed), 1..120 Hz
        const size_t n = R.size();
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (size_t i = 0; i < n; ++i) { sx += i; sy += R[i]; sxx += (double)i * i; sxy += i * R[i]; }
        const double b = (n * sxy - sx * sy) / (n * sxx - sx * sx), a0 = (sy - b * sx) / n;
        std::printf("rpm wobble spectrum:");
        for (double f = 2; f <= 120; f += 2) {
            double re = 0, im = 0;
            for (size_t i = 0; i < n; ++i) { double d = R[i] - (a0 + b * i), ph = 2 * 3.14159265 * f * i * 0.001; re += d * std::cos(ph); im += d * std::sin(ph); }
            double amp = 2 * std::sqrt(re * re + im * im) / n;
            if (amp > 4) std::printf("  %.0fHz:%.0f", f, amp);
        }
        std::printf("   (half order %.1f Hz)\n", R[n / 2] / 120.0);
    }
    std::printf("knock %.2f (sd %.2f)  advance %.1f (sd %.1f, min %.1f)  afr %.2f (sd %.2f)  misfires %d\n", km, ksd, dm, dsd,
                *std::min_element(Adv.begin(), Adv.end()), fm, fsd, t.misfires - mis0);
}

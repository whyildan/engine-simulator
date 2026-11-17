// Idle quality of each preset (warm, on the dyno stand, 12 s): rpm spread, idle valve movement, cycle-to-cycle
// IMEP (CoV) and "thumps" - cycles much stronger than the idle median (the "dung" of a hunting idle).
// usage: idle_probe [preset ...]     (no argument: every preset)
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
    std::vector<int> presets;
    for (int i = 1; i < argc; ++i) presets.push_back(std::atoi(argv[i]));
    if (presets.empty()) for (int i = 0; i < presetCount(); ++i) presets.push_back(i);
    std::printf("%-44s %6s %5s %5s %5s | %5s %5s %6s | %6s %6s %6s\n", "preset", "rpm", "sd", "min", "max", "iac", "max", "jumps",
                "CoV%", "thumps", "misf");
    double sumSd = 0, sumCov = 0; int sumThump = 0, sumJump = 0;
    for (int idx : presets) {
        EngineParams p = makeCalibratedPreset(idx);
        if (std::getenv("IMP")) p.imperfection = (float)std::atof(std::getenv("IMP"));
        Simulator sim(p);
        p.ignition = 1; p.starter = 1; sim.setParams(p);
        for (int k = 0; k < 600; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
        run(sim, 0.15);
        p.starter = 0; sim.setParams(p);
        run(sim, 4.0);
        const int mis0 = tel(sim).misfires;
        std::vector<double> rpm, iac, imep;
        float last[kMaxCyl] = {};
        int jumps = 0;
        double iacPrev = -1;
        for (int k = 0; k < 1200; ++k) {
            run(sim, 0.01);
            Telemetry t = tel(sim);
            rpm.push_back(t.rpm); iac.push_back(t.iac);
            if (std::getenv("TRACE") && k % 10 == 0)
                std::printf("    t%.1f rpm %5.0f iac %5.1f adv %5.1f map %5.1f afr %5.2f mis %d\n", k * 0.01, t.rpm, t.iac, t.advance, t.mapKPa, t.afr, t.misfires);
            // idle valve jumps: more than 8 % opening within 50 ms
            if (k % 5 == 0) { if (iacPrev >= 0 && t.iac - iacPrev > 8.0) ++jumps; iacPrev = t.iac; }
            for (int c = 0; c < std::min(t.cylinders, kMaxCyl); ++c)
                if (t.cylImep[c] != last[c]) { last[c] = t.cylImep[c]; if (k > 0) imep.push_back(t.cylImep[c]); }
        }
        const int mis = tel(sim).misfires - mis0;
        auto mean = [](const std::vector<double>& v) { double s = 0; for (double x : v) s += x; return v.empty() ? 0.0 : s / v.size(); };
        const double rm = mean(rpm);
        double sd = 0; for (double x : rpm) sd += (x - rm) * (x - rm); sd = std::sqrt(sd / std::max<size_t>(1, rpm.size()));
        std::vector<double> sorted = imep; std::sort(sorted.begin(), sorted.end());
        const double med = sorted.empty() ? 0 : sorted[sorted.size() / 2];
        const double im = mean(imep);
        double isd = 0; for (double x : imep) isd += (x - im) * (x - im); isd = std::sqrt(isd / std::max<size_t>(1, imep.size()));
        int thumps = 0; for (double x : imep) if (med > 0.05 && x > 1.6 * med) ++thumps;
        const double cov = im > 1e-3 ? 100.0 * isd / im : 0;
        std::printf("%-44.44s %6.0f %5.0f %5.0f %5.0f | %5.1f %5.1f %6d | %6.1f %6d %6d\n", p.name, rm, sd, *std::min_element(rpm.begin(), rpm.end()),
                    *std::max_element(rpm.begin(), rpm.end()), mean(iac), *std::max_element(iac.begin(), iac.end()), jumps, cov, thumps, mis);
        sumSd += sd; sumCov += cov; sumThump += thumps; sumJump += jumps;
    }
    const int n = (int)presets.size();
    std::printf("mean rpm sd %.0f, mean CoV %.1f %%, thumps %d, idle valve jumps %d\n", sumSd / n, sumCov / n, sumThump, sumJump);
    return 0;
}

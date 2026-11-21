// diesel emissions probe: NOx / soot / FSN over load, with and without EGR / post injection / cooler
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;
static void run(Simulator& sim, double s) { static std::vector<float> b(960); for (double t = 0; t < s; t += 0.01) sim.render(b.data(), 480); }
static void start(Simulator& sim, EngineParams& p) {
    static Telemetry t;
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (double x = 0; x < 4; x += 0.01) { run(sim, 0.01); sim.getTelemetry(t); if (t.running && t.rpm > 600) break; }
    run(sim, 0.2); p.starter = 0; sim.setParams(p);
}
static void point(EngineParams p, float rpm, float pedal, const char* tag) {
    Simulator sim(p);
    start(sim, p);
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = rpm; p.throttle = pedal; sim.setParams(p);
    run(sim, 6.0);
    static Telemetry t;
    double nox = 0, soot = 0, fsn = 0, tq = 0, gs = 0; int n = 0;
    for (int k = 0; k < 8; ++k) { run(sim, 0.5); sim.getTelemetry(t); nox += t.noxPpm; soot += t.sootMgM3; fsn += t.smokeFsn; tq += t.torque; gs += t.noxGs; ++n; }
    nox /= n; soot /= n; fsn /= n; tq /= n; gs /= n;
    const double kw = tq * rpm / 9549.0;
    std::printf("  %-14s %5.0f rpm pedal %3.0f%%  %6.1f Nm  NOx %6.0f ppm %5.2f g/kWh  soot %6.2f mg/m3 FSN %4.2f  EGR %4.1f%% lambda %.2f post %.1f mg egrT %.0f C IAT %.0f\n",
                tag, rpm, pedal * 100, tq, nox, kw > 0.5 ? gs * 3600.0 / kw : 0.0, soot, fsn, t.egrRate * 100, t.lambdaPipe, t.postMg, t.egrGasC, t.snsValue[kSnsIAT]);
}
int main(int argc, char** argv) {
    const int preset = argc > 1 ? std::atoi(argv[1]) : 26;
    EngineParams p = makeCalibratedPreset(preset);
    std::printf("[emissions] %s\n", p.name);
    for (float ped : {0.3f, 0.6f, 1.0f}) point(p, 2000, ped, "base");
    {   EngineParams q = p; q.egrValveDia_mm = 0; point(q, 2000, 0.3f, "no EGR"); }
    {   EngineParams q = p; q.egrCooler = 0; point(q, 2000, 0.3f, "hot EGR"); }
    {   EngineParams q = p; q.cal.postMg = 0; point(q, 2000, 0.6f, "no post"); }
    {   EngineParams q = p; q.cal.postMg = 0; point(q, 2000, 0.3f, "no post"); }
    return 0;
}

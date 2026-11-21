// which sensors report a fault for a preset (idle, warm)
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;
int main(int argc, char** argv) {
    EngineParams p = makeCalibratedPreset(argc > 1 ? std::atoi(argv[1]) : 26);
    Simulator sim(p);
    std::vector<float> buf(960);
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 100; ++k) sim.render(buf.data(), 480);
    p.starter = 0; sim.setParams(p);
    for (int k = 0; k < 300; ++k) sim.render(buf.data(), 480);
    static Telemetry t; sim.getTelemetry(t);
    for (int i = 0; i < kSensorCount; ++i)
        if (t.snsStatus[i] || t.snsFallback[i]) std::printf("%-24s status %d fallback %d truth %.2f value %.2f volts %.2f\n", sensorDef(i).name, t.snsStatus[i], (int)t.snsFallback[i], t.snsTruth[i], t.snsValue[i], t.snsVolts[i]);
    std::printf("done, rpm %.0f\n", t.rpm);
    return 0;
}

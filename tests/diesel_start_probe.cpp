// quick start check for a preset: crank, report rpm over time
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace es;
int main(int argc, char** argv) {
    EngineParams p = makeCalibratedPreset(argc > 1 ? std::atoi(argv[1]) : 29);
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    static Telemetry t;
    for (int k = 0; k < 60; ++k) {
        for (int j = 0; j < 10; ++j) sim.render(buf.data(), 480);
        sim.getTelemetry(t);
        if (k % 5 == 0)
            std::printf("t %.1f rpm %.0f run %d fuel %.1f mg misfires %d idDeg %.1f Nm %.1f smoke %d maf %.2f egt %.0f\n", k * 0.1, t.rpm,
                        (int)t.running, t.dieselFuelMg, t.misfires, t.ignDelayDeg, t.torque, (int)t.smokeLimited, t.snsValue[kSnsMAF], t.egt);
        if (k == 30) { p.starter = 0; sim.setParams(p); }
    }
    return 0;
}

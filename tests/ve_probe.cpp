// Diagnostic: true volumetric efficiency of a preset over rpm x MAP (dyno hold + throttle), compared
// with what the ECU's VE map holds. Also a blip-and-return trace.
// usage: ve_probe <preset> [trace]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace es;

static void run(Simulator& sim, double s) {
    std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}

int main(int argc, char** argv) {
    int idx = argc > 1 ? std::atoi(argv[1]) : 0;
    auto t0 = std::chrono::steady_clock::now();
    EngineParams p = std::getenv("GENERIC") ? makePreset(idx) : makeCalibratedPreset(idx);
    std::printf("base map: %.2f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    if (const char* lm = std::getenv("LOADMODE")) p.cal.loadMode = (LoadMode)std::atoi(lm);
    p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    run(sim, 1.0);
    p.starter = 0; sim.setParams(p); run(sim, std::getenv("IDLE") ? std::atof(std::getenv("IDLE")) : 4.0);
    Telemetry t;
    if (argc > 2 && std::strcmp(argv[2], "blip") == 0) {
        p.throttle = 1; sim.setParams(p);
        for (int k = 0; k < 40; ++k) {
            run(sim, 0.02); sim.getTelemetry(t);
            std::printf("blip t+%.2f rpm %5.0f map %5.1f wb %5.3f target %5.2f veECU %5.1f ae %4.2f duty %.2f lam %.2f %.2f mis %d\n", 0.02 * (k + 1), t.rpm,
                        t.mapKPa, t.snsValue[kSnsWideband], t.targetAfr, t.ecuVe, t.ecuAccel, t.injDuty, t.cylLambda[0], t.cylLambda[1], t.misfires);
        }
        return 0;
    }
    if (argc > 2 && std::strcmp(argv[2], "mapfail") == 0) {
        p.sensors[kSnsMAP].fault = SensorFault::Open; sim.setParams(p);
        for (int k = 0; k < 30; ++k) {
            run(sim, 0.05); sim.getTelemetry(t);
            std::printf("mapfail t+%.2f rpm %5.0f map %5.1f wb %5.3f target %5.2f veECU %5.1f iac %4.1f tps %4.1f lam %.2f %.2f\n", 0.05 * (k + 1), t.rpm,
                        t.mapKPa, t.snsValue[kSnsWideband], t.targetAfr, t.ecuVe, t.iac, t.ecuTps, t.cylLambda[0], t.cylLambda[1]);
        }
        return 0;
    }
    if (argc > 2 && std::strcmp(argv[2], "idle") == 0) {
        for (int k = 0; k < 40; ++k) {
            run(sim, 0.25); sim.getTelemetry(t);
            std::printf("idle t+%.2f rpm %5.0f map %5.1f afr %5.2f wb %5.3f target %5.2f veECU %5.1f tuning %d ect %.0f\n", 0.25 * (k + 1), t.rpm, t.mapKPa,
                        t.afr, t.snsValue[kSnsWideband], t.targetAfr, t.ecuVe, (int)t.tuning, t.snsValue[kSnsECT]);
        }
        return 0;
    }
    if (argc > 2 && std::strcmp(argv[2], "trace") == 0) {
        p.throttle = 1; sim.setParams(p); run(sim, 0.6);
        p.throttle = 0; sim.setParams(p);
        for (int k = 0; k < 200; ++k) {
            run(sim, 0.02); sim.getTelemetry(t);
            std::printf("t+%.2f rpm %5.0f map %5.1f afr %5.2f target %5.2f veECU %5.1f veTrue %5.1f trim %.2f cut %d lamCyl %.2f %.2f\n", 0.02 * (k + 1), t.rpm,
                        t.mapKPa, t.afr, t.targetAfr, t.ecuVe, t.mapKPa > 1 ? t.ve * 101.3 / t.mapKPa * 100 : 0, t.ecuTrim, t.targetAfr <= 0, t.cylLambda[0], t.cylLambda[1]);
        }
        return 0;
    }
    std::printf("%s: true VE %% (cylinder trapped air vs MAP)   [ECU map value]\n", p.name);
    const char* thrs = std::getenv("THRS");
    std::vector<float> tl = {0.08f, 0.15f, 0.25f, 0.4f, 1.0f};
    if (thrs) { tl.clear(); for (const char* q = thrs; *q; ) { tl.push_back((float)std::atof(q)); while (*q && *q != ',') ++q; if (*q) ++q; } }
    for (float thr : tl) {
        for (float rpm : {1500.f, 3000.f, 4500.f, 6000.f}) {
            p.throttle = thr; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = rpm; sim.setParams(p); run(sim, 2.5);
            sim.getTelemetry(t);
            double veTrue = t.ve * 101.3 / std::max(1.0f, t.mapKPa) * 100.0;
            std::printf("  thr %.2f rpm %5.0f map %5.1f  true %5.1f  ECU %5.1f  afr %5.2f\n", thr, t.rpm, t.mapKPa, veTrue,
                        tableLookup(t.calVeMap, t.rpm, t.mapKPa), t.afr);
        }
    }
    return 0;
}

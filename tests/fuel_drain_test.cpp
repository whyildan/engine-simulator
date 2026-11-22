// Running out of fuel: an idling engine whose tank is emptied (Empty tank) stumbles and stops within seconds - fuel
// injection (the pump sucks air, the rail pressure goes), a carburettor (its float bowl runs dry), a diesel (its
// injection pump loses its prime). After refuelling it starts again (the diesel after its lift pump has pushed the
// air out with the ignition on).
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static void test(int id) {
    EngineParams p = makeCalibratedPreset(id);
    std::printf("[%s]\n", presetName(id));
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    Telemetry t;
    bool finite = true;
    auto run = [&](double secs) {
        for (double s = 0; s < secs; s += 0.01) {
            sim.render(buf.data(), 480);
            for (float x : buf) if (!std::isfinite(x)) finite = false;
            sim.getTelemetry(t);
        }
    };
    auto start = [&]() {
        p.ignition = 1; p.starter = 1; sim.setParams(p);
        double caught = 0;
        for (double s = 0; s < 8.0 && caught < 0.15; s += 0.01) {
            run(0.01);
            caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
        }
        p.starter = 0; sim.setParams(p);
        run(3.0);
    };
    start();
    const double idle = t.rpm;
    CHECK(t.running && idle > 400, "idles at %.0f rpm before (tank %.1f L)", idle, t.tankLitres);

    // drained, the engine held at a fast idle (a quarter throttle; a carburettor's float bowl would last a minute or
    // two at idle)
    ++p.drainRequest; p.throttle = 0.25f; sim.setParams(p);
    double stopAt = -1;
    for (double s = 0; s < 40.0 && stopAt < 0; s += 0.1) {
        run(0.1);
        if (!t.running || t.rpm < 100) stopAt = s;
    }
    p.throttle = 0; sim.setParams(p);
    run(3.0);
    std::printf("    tank %.2f L  fuel supply %.2f  rail %.2f bar  stopped after %.1f s\n", t.tankLitres, t.fuelSupply, t.fuelBar, stopAt);
    CHECK(stopAt >= 0 && stopAt < 35.0, "runs out of fuel and stops (%.1f s)", stopAt);
    // (a carburettor stalls lean with some fuel still in its bowl, below the main jet's working level)
    const float dry = p.fuelSystem == FuelSystem::Carburetor ? 0.5f : 0.05f;
    CHECK(t.fuelSupply < dry, "no fuel reaches the engine (supply %.2f)", t.fuelSupply);

    // refuel; the ignition is still on (the lift pump / fuel pump fill the system), then start
    p.refuelType = p.fuelType; p.refuelLitres = 20; ++p.refuelRequest; sim.setParams(p);
    run(6.0);
    start();
    std::printf("    after refuel: tank %.1f L  supply %.2f  %.0f rpm\n", t.tankLitres, t.fuelSupply, t.rpm);
    CHECK(t.running && t.rpm > 0.6 * idle, "starts again after refuelling (%.0f rpm)", t.rpm);
    CHECK(finite, "audio and telemetry stay finite");
}

int main() {
    for (int id : {0, 4, 24}) test(id);
    std::printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures ? 1 : 0;
}

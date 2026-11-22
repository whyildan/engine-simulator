// Cooling: every kind of engine - NA, turbo, twin turbo, supercharged, rotary, diesel, two-stroke - held at full
// throttle near its rated speed with the dyno cell's 50 km/h of cooling air settles below ~106 C coolant and ~128 C oil
// (the radiator and oil cooler are sized for the engine's rated heat); at idle the thermostat keeps it warm. A bigger
// radiator runs cooler; an air-to-oil cooler cools the oil.
// The heat inputs are measured on the engine (3 s at full throttle, held at speed); the thermal model is then run to
// its equilibrium on them (minutes of simulated time in milliseconds).
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// the engine's heat inputs at this speed and throttle (averaged over the last second of 3)
static ThermalInputs measure(EngineParams p, double rpm, double throttle) {
    const double fs = 48000.0, dt = 1.0 / fs, w = rpm * 2.0 * kPi / 60.0;
    p.ignition = 1;
    auto e = std::make_unique<Engine>(p, fs);
    e->setCrank(w, 0.0);
    ThermalInputs acc;
    int n = 0;
    for (double t = 0; t < 3.0; t += dt) {
        e->setCrank(w, e->theta());
        e->step(dt, throttle, 0.0);
        if (t > 2.0) {
            const ThermalInputs& q = e->thermalIn;
            acc.fuelPowerW += q.fuelPowerW; acc.releasedPowerW += q.releasedPowerW; acc.frictionPowerW += q.frictionPowerW;
            acc.turboOilW += q.turboOilW; acc.airflowKgs += q.airflowKgs; ++n;
        }
    }
    acc.fuelPowerW /= n; acc.releasedPowerW /= n; acc.frictionPowerW /= n; acc.turboOilW /= n; acc.airflowKgs /= n;
    acc.rpm = rpm; acc.ambientC = 25; acc.speedMs = 50.0 / 3.6;
    return acc;
}

struct Eq { double coolant, oil; };
static Eq settle(const EngineParams& p, const ThermalInputs& in, double secs = 600) {
    ThermalModel m;
    m.init(p);
    for (double t = 0; t < secs; t += 0.02) m.step(0.02, in, p);
    return {m.coolantC, m.oilC};
}

int main() {
    std::printf("[full load] held at full throttle near rated speed, 50 km/h cooling air, 25 C ambient\n");
    const int ids[] = {0, 2, 4, 7, 16, 17, 19, 20, 33, 34, 35, 18, 12, 24, 26, 28, 25, 30};
    for (int id : ids) {
        const EngineParams p = makeCalibratedPreset(id);
        const double rpm = std::max(1500.0, 0.92 * std::min((double)p.revLimit, (double)p.sweepEndRpm));
        const ThermalInputs in = measure(p, rpm, 1.0);
        const Eq q = settle(p, in);
        std::printf("  %-52s %5.0f rpm  fuel %5.0f kW (burns %5.0f, rated %5.0f)  coolant %5.1f C  oil %5.1f C\n", presetName(id), rpm,
                    in.fuelPowerW * 1e-3, in.releasedPowerW * 1e-3, ThermalModel::ratedFuelPowerW(p) * 1e-3, q.coolant, q.oil);
        CHECK(q.coolant < 106.0 && q.oil < 128.0, "%s: coolant and oil hold", presetName(id));
    }
    {
        std::printf("[idle] warm engines stay warm (the thermostat)\n");
        for (int id : {0, 19, 33, 26}) {
            const EngineParams p = makeCalibratedPreset(id);
            ThermalInputs in = measure(p, p.idleTargetRpm, 0.0);
            in.speedMs = 0;
            const Eq q = settle(p, in);
            std::printf("  %-52s coolant %5.1f C  oil %5.1f C\n", presetName(id), q.coolant, q.oil);
            // (a diesel's idle heat is tiny - a few kW: it cools below its thermostat on a long idle, as real ones do)
            CHECK(q.coolant > (isDiesel(p) ? 75.0 : 84.0), "%s: does not overcool at idle", presetName(id));
        }
    }
    {
        std::printf("[parts] a bigger radiator, an air-to-oil cooler\n");
        EngineParams p = makeCalibratedPreset(19);
        const double rpm = 0.92 * p.sweepEndRpm;
        ThermalInputs in = measure(p, rpm, 1.0);
        in.ambientC = 45;   // (a hot day: past the thermostat's range, where the radiator's size shows)
        const Eq base = settle(p, in);
        EngineParams big = p; big.comp.radiatorSize = 1.5f;
        EngineParams air = p; air.comp.oilCooler = 2;
        EngineParams none = p; none.comp.oilCooler = 0;
        const Eq b = settle(big, in), a = settle(air, in), n0 = settle(none, in);
        std::printf("  drag I6: stock %.1f / %.1f C, radiator 1.5x %.1f / %.1f C, air-to-oil %.1f / %.1f C, no oil cooler %.1f / %.1f C\n",
                    base.coolant, base.oil, b.coolant, b.oil, a.coolant, a.oil, n0.coolant, n0.oil);
        CHECK(b.coolant < base.coolant - 2.0, "a bigger radiator runs cooler");
        CHECK(a.oil < base.oil - 3.0 && n0.oil > base.oil + 3.0, "an air-to-oil cooler cools the oil; without a cooler it runs hotter");
    }
    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

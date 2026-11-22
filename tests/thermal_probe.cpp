// Thermal probe: a preset held on the dyno at full throttle (the cell fan as set), coolant / oil temperatures, power,
// fuel power and the radiator's heat every 10 s.
// usage: thermal_probe <preset> [rpm] [seconds] [vehicleKmh]   (vehicleKmh > 0: the cooling air of that road speed)
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
using namespace es;

int main(int argc, char** argv) {
    const int id = argc > 1 ? std::atoi(argv[1]) : 16;
    const double rpm = argc > 2 ? std::atof(argv[2]) : 5000;
    const double secs = argc > 3 ? std::atof(argv[3]) : 180;
    const double kmh = argc > 4 ? std::atof(argv[4]) : 0;
    EngineParams p = makeCalibratedPreset(id);
    if (kmh > 0) p.coolingFan_kmh = (float)kmh;
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    Telemetry t;
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int i = 0; i < 150; ++i) sim.render(buf.data(), 480);
    p.starter = 0; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = 1.0f; sim.setParams(p);
    std::printf("%s at %.0f rpm, full throttle, cooling air %.0f km/h\n", p.name, rpm, p.coolingFan_kmh);
    for (double s = 0; s <= secs; s += 0.01) {
        sim.render(buf.data(), 480);
        if (std::fmod(s + 1e-6, 10.0) < 0.01) {
            sim.getTelemetry(t);
            const double fuelKw = t.fuelFlow * 1e-3 * t.fuelLhv * 1e3;   // g/s x MJ/kg
            std::printf("  %4.0f s  coolant %5.1f C  oil %5.1f C  fan %d  power %5.0f kW  fuel %5.0f kW  radiator %5.0f kW  boost %.2f\n", s,
                        t.coolantC, t.oilC, (int)t.fan, t.powerKw, fuelKw, t.radiatorKw, t.boostBar);
        }
    }
    return 0;
}

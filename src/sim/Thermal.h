#pragma once
// Engine thermal model (coolant + block, oil, intake air), electrical system (battery / alternator)
// and the fuel supply (pump + pressure regulator + rail). Lumped, deterministic, cheap: stepped at
// the ECU loop rate.
#include "sim/EngineParams.h"

namespace es {

struct ThermalInputs {
    double fuelPowerW = 0;     // chemical power of the fuel injected
    double releasedPowerW = 0; // ... of the part that can burn (a rich mixture's excess fuel leaves unburned: air-limited)
    double turboOilW = 0;      // turbocharger bearings' heat into the oil
    double frictionPowerW = 0; // mechanical friction power
    double airflowKgs = 0;     // air through the engine
    double speedMs = 0;        // vehicle speed (ram air through the radiator)
    double rpm = 0;
    double ambientC = 25;
};

class ThermalModel {
public:
    void init(const EngineParams& p);          // start temperatures (warm or cold start)
    void step(double dt, const ThermalInputs& in, const EngineParams& p);
    double coolantC = 88, oilC = 85, iatC = 25;
    double thermostat = 0;     // opening 0..1
    bool fan = false;
    double radiatorW = 0;      // heat rejected by the radiator
    double oilCoolerW = 0;     // heat rejected by an air-to-oil cooler
    // the fuel power the cooling system is designed for (W): the engine's rated output, estimated from its size,
    // revs and boost (the radiator and oil cooler are sized to it)
    static double ratedFuelPowerW(const EngineParams& p);
    // effects on the physics
    double viscosityFriction() const;           // friction multiplier from oil viscosity (1 = hot oil)
    double coldFactor() const;                  // 1 = stone cold walls, 0 = fully warm (fuel film behaviour)
private:
    double dispL_ = 2;
};

// battery voltage: alternator when running, sags while cranking
class Electrical {
public:
    void step(double dt, double rpm, bool ignition, bool starter);
    double volts = 12.6;
};

// fuel pump + regulator: rail pressure (bar gauge) and how far it sags when the pump can't keep up
class FuelRail {
public:
    // supply: the share of liquid fuel at the pump's inlet (1 = covered pickup, 0 = sucking air: an empty tank)
    void step(double dt, double demandLph, double mapGaugeBar, bool pumpOn, const EngineParams& p, double supply = 1.0);
    double pressureBar = 3.0;  // rail pressure (gauge)
    double setpointBar = 3.0;  // where the regulator wants it
    // pressure across the injector nozzle (rail minus manifold, both gauge)
    double deltaBar(double mapGaugeBar) const { return pressureBar - mapGaugeBar; }
};

} // namespace es

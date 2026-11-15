#include "sim/Thermal.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>

namespace es {

void ThermalModel::init(const EngineParams& p) {
    dispL_ = std::max(0.2, equivalentDisplacementL(p));
    if (p.coldStart) { coolantC = oilC = p.ambientTemp_C; }
    else { coolantC = 88; oilC = 85; }
    iatC = p.ambientTemp_C;
    thermostat = 0; fan = false;
}

double ThermalModel::viscosityFriction() const {
    // cold, thick oil: up to ~80 % more friction at 20 C than at 90 C
    return 1.0 + 0.8 * (1.0 - smoothstep(20.0, 90.0, oilC));
}

double ThermalModel::coldFactor() const { return 1.0 - smoothstep(15.0, 75.0, coolantC); }

void ThermalModel::step(double dt, const ThermalInputs& in, const EngineParams& p) {
    const double d = dispL_;
    // heat capacities (J/K): coolant + the part of the block it soaks; oil + crank / sump
    const double Cc = 4180.0 * (4.0 + 1.5 * d) + 460.0 * (20.0 + 12.0 * d);
    const double Co = 2000.0 * (3.0 + 1.2 * d) + 460.0 * (5.0 + 3.0 * d);
    const double amb = in.ambientC;
    // heat in: ~30 % of the fuel energy goes into the cooling jacket, friction splits oil / coolant
    double qCool = 0.30 * in.fuelPowerW + 0.5 * in.frictionPowerW;
    double qOil = 0.03 * in.fuelPowerW + 0.5 * in.frictionPowerW;
    // thermostat opens 86..96 C; electric fan above 98 C (off below 93)
    thermostat = std::max(0.02, smoothstep(86.0, 96.0, coolantC));
    if (coolantC > 98) fan = true; else if (coolantC < 93) fan = false;
    const double air = std::min(2.0, 0.25 + in.speedMs / 30.0 + (fan ? 0.6 : 0.0));
    const double pump = clampv((double)p.comp.waterPumpCapacity, 0.3, 3.0);
    const double circ = (pump / (pump + 0.3)) * 1.3 * (0.4 + 0.6 * smoothstep(500.0, 3000.0, in.rpm));
    const double uaRad = 700.0 * std::pow(d, 0.7) * air * circ * thermostat;
    radiatorW = uaRad * (coolantC - amb);
    const double uaBlock = 15.0 * std::pow(d, 0.7) * (1.0 + in.speedMs / 20.0);   // block / head to the air
    const double uaOilCool = 500.0 * std::pow(d, 0.7) + 100.0;                      // oil <-> coolant (block, oil cooler)
    const double uaSump = (12.0 + 20.0 * std::min(1.0, in.speedMs / 25.0)) * std::sqrt(d);
    const double xOc = uaOilCool * (oilC - coolantC);
    coolantC += dt * (qCool - radiatorW - uaBlock * (coolantC - amb) + xOc) / Cc;
    oilC += dt * (qOil - xOc - uaSump * (oilC - amb)) / Co;
    // intake air: heat soak from the engine bay at low speed and low airflow
    const double soak = 18.0 * (1.0 - smoothstep(0.0, 25.0, in.speedMs)) * clampv((coolantC - amb) / 70.0, 0.0, 1.5) /
                        (1.0 + in.airflowKgs * 1000.0 / 40.0);
    const double iatTarget = amb + soak;
    iatC += (iatTarget - iatC) * (1.0 - std::exp(-dt / 15.0));
}

void Electrical::step(double dt, double rpm, bool ignition, bool starter) {
    double target;
    if (starter) target = 10.2;                                   // starter motor pulls the battery down
    else if (!ignition) target = 12.6;
    else target = 12.4 + (14.2 - 12.4) * smoothstep(400.0, 900.0, rpm);   // alternator charging
    volts += (target - volts) * (1.0 - std::exp(-dt / (starter ? 0.05 : 0.4)));
}

void FuelRail::step(double dt, double demandLph, double mapGaugeBar, bool pumpOn, const EngineParams& p) {
    const ComponentSet& c = p.comp;
    const double base = std::max(0.5, (double)c.fuelPressure_bar);
    // MAP-referenced regulator keeps the pressure across the injector constant; returnless = fixed
    setpointBar = base + (c.fuelRegulator == FuelRegulator::MapReferenced ? mapGaugeBar : 0.0);
    double target;
    if (!pumpOn) target = pressureBar * std::exp(-dt / 20.0);     // bleeds down slowly with the pump off
    else {
        // pump curve: flow falls linearly to zero at its dead-head pressure (2.2 x base)
        const double pmax = 2.2 * base;
        const double cap = std::max(10.0, (double)c.fuelPumpLph);
        const double flowAtSet = cap * (1.0 - setpointBar / pmax) / (1.0 - base / pmax);
        if (flowAtSet >= demandLph) target = setpointBar;          // regulator bypasses the excess
        else target = std::max(0.0, pmax * (1.0 - demandLph * (1.0 - base / pmax) / cap));
    }
    pressureBar += (target - pressureBar) * (1.0 - std::exp(-dt / 0.08));
}

} // namespace es

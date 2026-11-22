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

// The fuel power at the engine's rated output (W), for sizing its cooling: ~210 kW of fuel per litre at 6500 rpm, in
// proportion to the rev limit (power ~ speed), times the charge density a boosted engine runs (1 + the wastegate's
// boost; a supercharger ~0.7 bar); a diesel runs lean - a little over half a petrol engine's fuel per litre and turn
// (lambda ~1.4 against ~0.85 at full load).
// Chosen, checked against the presets' measured full-load fuel power (K20A 490 kW against 555, I6 3.0 550 / 680,
// I4 turbo 850 / 925, the drag I6 1785 / 2190, 2JZ 880 / 1190 - the cooling has some margin over the real peak)
double ThermalModel::ratedFuelPowerW(const EngineParams& p) {
    const double L = std::max(0.1, equivalentDisplacementL(p));
    double boost = 0.0;
    if (p.induction == Induction::Turbo) boost = std::max(0.3, (double)p.wastegateBar);
    else if (p.induction == Induction::Supercharger) boost = 0.7;
    const double revs = clampv((double)p.revLimit, 2000.0, 15000.0) / 6500.0;
    return 210e3 * L * revs * (1.0 + boost) * (isDiesel(p) ? 0.55 : 1.0);
}

void ThermalModel::step(double dt, const ThermalInputs& in, const EngineParams& p) {
    const double d = dispL_;
    // heat capacities (J/K): coolant + the part of the block it soaks; oil + crank / sump
    const double Cc = 4180.0 * (4.0 + 1.5 * d) + 460.0 * (20.0 + 12.0 * d);
    const double Co = 2000.0 * (3.0 + 1.2 * d) + 460.0 * (5.0 + 3.0 * d);
    const double amb = in.ambientC;
    const double rated = ratedFuelPowerW(p);
    // Heat in. Only the fuel that burns releases heat (a rich mixture's excess leaves in the exhaust: air-limited). Of
    // that, the share into the cooling jacket falls with load - more of the heat leaves with the exhaust (Gharehghani
    // et al., Appl. Therm. Eng. 2013, turbocharged SI engine energy balance; ~20-30 % of the fuel energy to the coolant
    // in the general balances): 30 % at light load to 22 % at full load (chosen). Friction splits oil / coolant; a
    // turbo's bearings heat the oil.
    const double released = in.releasedPowerW > 0 ? in.releasedPowerW : in.fuelPowerW;
    const double share = 0.30 - 0.08 * smoothstep(0.25, 1.0, released / std::max(1.0, rated));
    double qCool = share * released + 0.5 * in.frictionPowerW;
    double qOil = 0.03 * released + 0.5 * in.frictionPowerW + in.turboOilW;
    // thermostat opens 86..96 C; electric fan above 98 C (off below 93)
    thermostat = std::max(0.02, smoothstep(86.0, 96.0, coolantC));
    if (coolantC > 98) fan = true; else if (coolantC < 93) fan = false;
    const double air = std::min(2.0, 0.25 + in.speedMs / 30.0 + (fan ? 0.6 : 0.0));
    const double pump = clampv((double)p.comp.waterPumpCapacity, 0.3, 3.0);
    const double circ = (pump / (pump + 0.3)) * 1.3 * (0.4 + 0.6 * smoothstep(500.0, 3000.0, in.rpm));
    // Radiator: sized, as a car's is, for the heat at rated output - with the fan on and 50 km/h of air (the dyno
    // cell's fan) it holds the coolant at ~100 C there (27 % of the rated fuel power - coolant share plus the friction at
    // the limiter - at
    // 75 K over ambient; air factor 1.31 and circulation 1.0 there - a standard pump at speed). A bigger core scales it
    // (radiatorSize).
    const double uaBase = 0.27 * rated / (75.0 * 1.31 * 1.0);
    const double uaRad = uaBase * clampv((double)p.comp.radiatorSize, 0.5, 2.5) * air * circ * thermostat;
    radiatorW = uaRad * (coolantC - amb);
    const double uaBlock = 15.0 * std::pow(d, 0.7) * (1.0 + in.speedMs / 20.0);   // block / head to the air
    // oil <-> coolant through the block - the oil 10-20 K over the coolant under load (and an oil-to-water cooler: 2.5x);
    // an air-to-oil cooler in front of the
    // radiator rejects to the air - sized for ~5 % of the rated fuel power at oil 115 C, 50 km/h air (chosen)
    const int oc = clampv((int)p.comp.oilCooler, 0, 2);
    const double uaOilCool = (700.0 * std::pow(d, 0.7) + 100.0) * (oc == 1 ? 2.5 : 1.0);
    const double uaSump = (12.0 + 20.0 * std::min(1.0, in.speedMs / 25.0)) * std::sqrt(d);
    const double uaOilAir = oc == 2 ? 0.05 * rated / (90.0 * 0.71) * std::min(2.0, 0.25 + in.speedMs / 30.0) : 0.0;
    oilCoolerW = uaOilAir * (oilC - amb);
    const double xOc = uaOilCool * (oilC - coolantC);
    coolantC += dt * (qCool - radiatorW - uaBlock * (coolantC - amb) + xOc) / Cc;
    oilC += dt * (qOil - xOc - uaSump * (oilC - amb) - oilCoolerW) / Co;
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

void FuelRail::step(double dt, double demandLph, double mapGaugeBar, bool pumpOn, const EngineParams& p, double supply) {
    const ComponentSet& c = p.comp;
    const double base = std::max(0.5, (double)c.fuelPressure_bar);
    // MAP-referenced regulator keeps the pressure across the injector constant; returnless = fixed
    setpointBar = base + (c.fuelRegulator == FuelRegulator::MapReferenced ? mapGaugeBar : 0.0);
    double target, tau = 0.08;
    if (!pumpOn) target = pressureBar * std::exp(-dt / 20.0);     // bleeds down slowly with the pump off
    else if (supply < 0.02) { target = 0.0; tau = 0.4; }          // sucking air: the rail empties through the
                                                                  // injectors / regulator within a second (chosen)
    else {
        // pump curve: flow falls linearly to zero at its dead-head pressure (2.2 x base); a partly uncovered pickup
        // gulps air with the fuel - the delivered flow falls with the liquid share
        const double pmax = 2.2 * base;
        const double cap = std::max(10.0, (double)c.fuelPumpLph) * std::min(1.0, supply);
        const double flowAtSet = cap * (1.0 - setpointBar / pmax) / (1.0 - base / pmax);
        if (flowAtSet >= demandLph) target = setpointBar;          // regulator bypasses the excess
        else target = std::max(0.0, pmax * (1.0 - demandLph * (1.0 - base / pmax) / cap));
    }
    pressureBar += (target - pressureBar) * (1.0 - std::exp(-dt / tau));
}

} // namespace es

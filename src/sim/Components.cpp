#include "sim/Components.h"
#include "sim/EngineParams.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>

namespace es {

const char* pistonTypeNames() { return "Cast aluminium\0Hypereutectic\0Forged\0"; }
const char* rodTypeNames() { return "Powdered metal (stock)\0Forged steel\0Titanium\0Aluminium (drag)\0"; }
const char* crankTypeNames() { return "Cast iron\0Forged steel\0Billet\0"; }
const char* lifterTypeNames() { return "Hydraulic flat tappet (pushrod)\0Hydraulic roller (pushrod)\0Solid bucket (DOHC)\0Finger follower\0"; }
const char* throttleTypeNames() { return "Cable\0Drive-by-wire\0"; }
const char* coilTypeNames() { return "Distributor (one coil)\0Wasted spark\0Coil on plug\0"; }
const char* fuelRegulatorNames() { return "Manifold referenced (return)\0Returnless (fixed)\0"; }

namespace {
// allowed peak inertia load per piston area (MPa) for each material (fatigue limit, with margin)
const double kPistonAllow[] = {3.4, 3.8, 5.2};
const double kRodAllow[] = {3.6, 5.0, 5.6, 4.2};
const double kCrankAllow[] = {5.5, 7.5, 9.0};
// moving mass a lifter adds to each valve (rocker / pushrod reflected), kg; and friction factor
const double kLifterMass[] = {0.095, 0.120, 0.035, 0.028};
const double kLifterFriction[] = {1.6, 0.9, 1.0, 0.75};

// cam profile used by ValveSpec: f(x) = (0.5 (1 + cos(pi x)))^0.75 with x = deg / half-duration;
// largest negative curvature (nose) -> the deceleration the spring must provide
double profileNoseCurvature() {
    static double k = [] {
        double best = 0, hx = 1e-3;
        for (double x = -0.6; x <= 0.6; x += 0.01) {
            auto f = [](double v) { return std::pow(0.5 * (1.0 + std::cos(kPi * v)), 0.75); };
            double d2 = (f(x + hx) - 2 * f(x) + f(x - hx)) / (hx * hx);
            best = std::max(best, -d2);
        }
        return best;
    }();
    return k;
}

double valveMass(double dia_mm) { return 0.045 * std::pow(std::max(10.0, dia_mm) / 33.0, 2.2) + 0.016 + 0.010; }

// valve deceleration demand per (rpm)^2: a = lift * k / h^2 * (6 rpm)^2   [deg/s = 6 rpm]
double accelPerRpm2(double lift_mm, double dur) {
    double h = std::max(30.0, dur * 0.5);
    return lift_mm * 1e-3 * profileNoseCurvature() / (h * h) * 36.0;
}
} // namespace

void ComponentModel::build(const EngineParams& p) {
    const ComponentSet& c = p.comp;
    piston = !isRotary(p);
    pistonArea = kPi * 0.25 * p.bore_mm * p.bore_mm * 1e-6;
    crankRadius = p.stroke_mm * 0.5e-3;
    rodLen = std::max(p.rod_mm * 1e-3, crankRadius * 2.2);
    const double mp = std::max(50.0, (double)c.pistonMass_g) * 1e-3;
    const double mr = std::max(50.0, (double)c.rodMass_g) * 1e-3;
    recipMass = mp + mr / 3.0;
    rotMass = mr * 2.0 / 3.0;
    if (!piston) { extraInertia = 0; safeRpm = 1e9; partSafeRpm[0] = partSafeRpm[1] = partSafeRpm[2] = 1e9; floatRpmIn = floatRpmEx = 1e9; return; }
    // equivalent crank inertia: rotating part fully, reciprocating part on average half
    extraInertia = p.cylinders * (rotMass + 0.5 * recipMass) * crankRadius * crankRadius;
    allowInertiaPa[0] = kPistonAllow[clampv((int)c.pistonType, 0, 2)] * 1e6;
    allowInertiaPa[1] = kRodAllow[clampv((int)c.rodType, 0, 3)] * 1e6;
    allowInertiaPa[2] = kCrankAllow[clampv((int)c.crankType, 0, 2)] * 1e6;
    // load per (rad/s)^2 for each part -> rpm at which it reaches its limit
    const double rl = 1.0 + crankRadius / rodLen;
    const double load[3] = {mp * crankRadius * rl / pistonArea, recipMass * crankRadius * rl / pistonArea,
                            (recipMass * rl + rotMass) * crankRadius / pistonArea};
    safeRpm = 1e9;
    for (int i = 0; i < 3; ++i) {
        double w = std::sqrt(allowInertiaPa[i] / load[i]);
        double rpm = w * 60.0 / (2.0 * kPi);
        partSafeRpm[i] = rpm;
        if (rpm < safeRpm) { safeRpm = rpm; weakest = i; }
    }
    // valve float: spring force at full lift vs mass * cam deceleration
    const int lt = clampv((int)c.lifter, 0, 3);
    valveFrictionK = kLifterFriction[lt];
    moveMassIn = valveMass(p.intakeValveDia_mm) + kLifterMass[lt];
    moveMassEx = valveMass(p.exhaustValveDia_mm) + kLifterMass[lt];
    const double fo = std::max(50.0, (double)c.springOpen_N);
    floatRpmIn = std::sqrt(fo / (moveMassIn * accelPerRpm2(p.intakeLift_mm, p.intakeDuration)));
    floatRpmEx = std::sqrt(fo / (moveMassEx * accelPerRpm2(p.exhaustLift_mm, p.exhaustDuration)));
}

double ComponentModel::stress(double rpm) const {
    if (!piston) return 0;
    return (rpm / safeRpm) * (rpm / safeRpm);
}

double ComponentModel::floatSeverity(double rpm, bool intake) const {
    if (!piston) return 0;
    double f = intake ? floatRpmIn : floatRpmEx;
    return clampv((rpm - f) / (0.15 * f), 0.0, 1.0);
}

double ComponentModel::extraFmep(const EngineParams& p, double rpm, double Sp, double perRev) const {
    const ComponentSet& c = p.comp;
    double fmep = 0;
    if (piston) {
        // piston rings: tension drags on the bore, more with piston speed
        fmep += (clampv((double)c.ringTension, 0.3, 2.0) - 1.0) * 22e3 * (1.0 + Sp / 25.0);
        // valvetrain: proportional to the total spring load, minus what the base model contains
        int nv = p.cylinders * (std::max(1, (int)p.intakeValves) + std::max(1, (int)p.exhaustValves));
        double sum = nv * 0.5 * (c.springSeat_N + c.springOpen_N);
        double dispTotal = perRev * 2.0;
        double vt = 4.7e-3 * sum * valveFrictionK / std::max(1e-5, dispTotal) * (1.0 + 0.3 * std::exp(-rpm / 1500.0));
        // only part of the spring load turns into friction at the cam / follower contact; the base
        // model already contains a standard valvetrain (~15 kPa)
        fmep += 0.5 * (vt - 15e3);
    }
    // pumps: drive torque above a standard pump -> mean effective pressure
    double dispScale = std::pow(std::max(0.2, perRev * 2.0 * 1000.0 / 2.0), 0.7);
    double oil = oilPressure(p, rpm, 90.0);
    EngineParams std1 = p; std1.comp.oilPumpCapacity = 1; std1.comp.oilRelief_bar = 5;
    double oilStd = oilPressure(std1, rpm, 90.0);
    double tOil = 0.45 * dispScale * (clampv((double)c.oilPumpCapacity, 0.3, 3.0) * oil - oilStd);
    double tWater = (clampv((double)c.waterPumpCapacity, 0.3, 3.0) - 1.0) * dispScale * (0.25 + 1.0 * (rpm / 6000.0) * (rpm / 6000.0));
    fmep += (tOil + tWater) * 2.0 * kPi / std::max(1e-5, perRev);
    return fmep;
}

double ComponentModel::coilInterval(const EngineParams& p, double rpm) {
    const double rev = 60.0 / std::max(30.0, rpm);
    switch (p.comp.coilType) {
    case CoilType::Distributor: return (isRotary(p) ? rev : 2.0 * rev) / std::max(1, (int)p.cylinders);
    case CoilType::WastedSpark: return rev;
    case CoilType::CoilOnPlug: default: return isRotary(p) ? rev : 2.0 * rev;
    }
}

// the coil charges for the dwell the ECU commands (never longer than the time between its sparks minus the ~1 ms
// spark); its current rises more slowly at a low battery voltage
double ComponentModel::sparkEnergy(const EngineParams& p, double rpm, double dwellMs, double volts) const {
    const ComponentSet& c = p.comp;
    if (rpm < 30) return c.coilEnergy_mJ;
    const double dwell = std::max(0.0, std::min(dwellMs * 1e-3, coilInterval(p, rpm) - 1.2e-3));
    const double tc = std::max(0.3, (double)c.coilCharge_ms) * 1e-3 * 14.0 / clampv(volts, 6.0, 16.0);
    return c.coilEnergy_mJ * (1.0 - std::exp(-3.0 * dwell / tc));
}

double ComponentModel::sparkNeeded(const EngineParams& p, double pc, double lambda) {
    double gap = clampv((double)p.comp.plugGap_mm, 0.3, 2.0);
    double e = 8.0 * std::pow(std::max(0.5, pc / 10e5), 0.9) * std::pow(gap / 0.8, 1.3);
    if (lambda > 1.0) e *= 1.0 + 1.5 * (std::min(lambda, 1.8) - 1.0);   // lean mixtures are harder to light
    return e;
}

double ComponentModel::oilPressure(const EngineParams& p, double rpm, double oilT) {
    if (rpm < 30) return 0;
    double visc = clampv(std::exp(-0.012 * (oilT - 90.0)), 0.4, 4.0);
    double v = clampv((double)p.comp.oilPumpCapacity, 0.3, 3.0) * 1.7 * std::pow(rpm / 1000.0, 0.9) * visc;
    return std::min(v, std::max(1.0, (double)p.comp.oilRelief_bar));
}

void applyComponentDefaults(EngineParams& p) {
    ComponentSet& c = p.comp;
    const double bore = std::max(30.0, (double)p.bore_mm), rod = std::max(60.0, (double)p.rod_mm);
    c.ringTension = 1.0;
    c.throttleType = p.fuelSystem == FuelSystem::EFI && !p.itb ? ThrottleType::DriveByWire : ThrottleType::Cable;
    c.dbwResponse_ms = 30;   // modern electronic throttle: ~80-100 ms end to end
    c.injDeadTime_ms = p.injector_ccmin > 800 ? 0.9f : 0.7f;
    // fuel supply: 3 bar manifold-referenced (4 bar with boost); pump sized for full power + margin
    c.fuelPressure_bar = isBoosted(p) ? 4.0f : 3.0f;
    c.fuelRegulator = FuelRegulator::MapReferenced;
    {
        double boost = isBoosted(p) ? (p.induction == Induction::Turbo ? p.wastegateBar : 0.9) : 0.0;
        double air = equivalentDisplacementL(p) * 1e-3 * (p.revLimit / 120.0) * 1.2 * 1.05 * (1.0 + boost);   // kg/s
        double lph = air / 11.5 / 0.745 * 3600.0;
        c.fuelPumpLph = (float)std::max(60.0, 1.5 * lph);
    }
    if (p.fuelSystem == FuelSystem::Carburetor) { c.coilType = CoilType::Distributor; c.coilEnergy_mJ = 50; c.coilCharge_ms = 4.5f; }
    else if (p.cylinders <= 2 || isRotary(p)) { c.coilType = CoilType::CoilOnPlug; c.coilEnergy_mJ = 80; c.coilCharge_ms = 3.0f; }
    else { c.coilType = CoilType::CoilOnPlug; c.coilEnergy_mJ = 80; c.coilCharge_ms = 3.0f; }
    // spark plugs: colder for boost and very high revs (tip temperature)
    int heat = 6;
    if (isBoosted(p)) heat += (int)std::ceil(1.5 * (p.induction == Induction::Turbo ? p.wastegateBar : 1.0f));
    if (p.revLimit > 8000) heat += 1;
    c.plugHeatRange = clampv(heat, 5, 9);
    c.plugGap_mm = isBoosted(p) ? 0.65f : 0.8f;
    c.oilPumpCapacity = 1.0f; c.oilRelief_bar = 5.0f; c.waterPumpCapacity = 1.0f;
    c.allowFailure = 0;
    // valvetrain layout
    const int vpc = p.intakeValves + p.exhaustValves;
    // carburetted V6 / V8: classic pushrod engine with flat-tappet hydraulic lifters
    if (p.layout == Layout::V && p.cylinders >= 6 && p.fuelSystem == FuelSystem::Carburetor) c.lifter = LifterType::HydraulicFlat;
    else if (vpc >= 4) c.lifter = LifterType::SolidBucket;
    else if (p.layout == Layout::V && p.cylinders >= 6) c.lifter = LifterType::HydraulicRoller;
    else c.lifter = LifterType::FingerFollower;
    // masses from size, cheapest materials first; upgrade until safe beyond the limiter
    const double target = p.revLimit * 1.12 + 300.0;
    c.pistonType = PistonType::Cast; c.rodType = RodType::PowderedMetal; c.crankType = CrankType::Cast;
    double massScale = 1.0;
    ComponentModel m;
    for (int iter = 0; iter < 14; ++iter) {
        double pf = c.pistonType == PistonType::Forged ? 0.92 : c.pistonType == PistonType::Hypereutectic ? 0.96 : 1.0;
        double rf = c.rodType == RodType::Titanium ? 0.62 : c.rodType == RodType::Aluminium ? 0.8 : 1.0;
        c.pistonMass_g = (float)(400.0 * std::pow(bore / 86.0, 3.0) * pf * massScale);
        c.rodMass_g = (float)(500.0 * std::pow(bore / 86.0, 1.5) * (rod / 143.0) * rf * massScale);
        m.build(p);
        if (!m.piston || m.safeRpm >= target) break;
        if (m.weakest == 0 && c.pistonType != PistonType::Forged) c.pistonType = (PistonType)((int)c.pistonType + 1);
        else if (m.weakest == 1 && c.rodType == RodType::PowderedMetal) c.rodType = RodType::ForgedSteel;
        else if (m.weakest == 1 && c.rodType == RodType::ForgedSteel) c.rodType = RodType::Titanium;
        else if (m.weakest == 2 && c.crankType != CrankType::Billet) c.crankType = (CrankType)((int)c.crankType + 1);
        else massScale *= 0.9;               // already the best material: lighter parts
    }
    // springs: control the valves comfortably past the target rpm
    double need = 0;
    if (!isRotary(p)) {
        const int lt = (int)c.lifter;
        double mi = valveMass(p.intakeValveDia_mm) + kLifterMass[lt], me = valveMass(p.exhaustValveDia_mm) + kLifterMass[lt];
        need = std::max(mi * accelPerRpm2(p.intakeLift_mm, p.intakeDuration), me * accelPerRpm2(p.exhaustLift_mm, p.exhaustDuration)) *
               target * target;
    }
    // margin over the rigid-train estimate: a long pushrod train is elastic and floats earlier (Valvetrain.h)
    static const double kSpringMargin[] = {1.30, 1.25, 1.05, 1.10};
    c.springOpen_N = (float)std::max(250.0, need * kSpringMargin[clampv((int)c.lifter, 0, 3)]);
    c.springSeat_N = c.springOpen_N * 0.45f;
}

} // namespace es

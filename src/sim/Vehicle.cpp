#include "sim/Vehicle.h"

namespace es {

static constexpr double kG = 9.81;
static constexpr double kRhoAir = 1.2;
static constexpr double kClutchK = 9000;     // N m / rad, clutch + shaft torsional stiffness
static constexpr double kClutchC = 6;        // N m s / rad
static constexpr double kInputJ = 0.02;      // gearbox input shaft + clutch disc
static constexpr double kWheelJ = 0.9;       // driven wheels + axles

void Vehicle::reset() {
    gear = 0; v = 0; wheelOmega = 0; inputOmega = 0; engagement = 0; clutchTorque = 0;
    slip = 0; driveForce = 0; distance = 0; clutchLocked = false; twist_ = 0;
    clutchTorqueAvg = 0; avgSum_ = avgPhase_ = avgSeen_ = 0; avgBin_ = 0;
    for (double& b : avgBins_) b = 0;
    shiftTimer_ = 0; sinceShift_ = 10; pendingGear_ = 0; lastRequest_ = -99; autoEng_ = 0;
    runTime = t100 = tQuarter = vQuarter = 0; timing = false;
}

double Vehicle::ratio(const EngineParams& p, int g) const {
    if (g <= 0) return 0;
    g = clampv(g, 1, clampv((int)p.gears, 1, kMaxGears));
    return std::max(0.2f, p.gearRatio[g - 1]) * std::max(0.5f, p.finalDrive);
}

// shift strategy of the automatic: upshift points rise with throttle, kickdown on demand
void Vehicle::autoShift(double rpm, double throttle, const EngineParams& p) {
    const int n = clampv((int)p.gears, 1, kMaxGears);
    if (p.gear <= 0) { pendingGear_ = 0; return; }        // selector in N
    if (gear == 0 && pendingGear_ == 0) { pendingGear_ = 1; return; }
    // only shift with the clutch locked (not while launching / re-engaging) and not too often
    if (shifting || gear == 0 || !clutchLocked || sinceShift_ < 0.9) return;
    double limit = p.revLimit;
    double up = lerp(std::max(2200.0, p.idleTargetRpm * 2.6), limit - 250.0, std::pow(clampv(throttle, 0.0, 1.0), 0.8));
    double down = lerp(std::max(1150.0, p.idleTargetRpm * 1.35), 0.55 * limit, clampv(throttle, 0.0, 1.0));
    if (rpm > up && gear < n) pendingGear_ = gear + 1;
    else if (rpm < down && gear > 1 && sinceShift_ > 1.6) {
        double after = rpm * ratio(p, gear - 1) / ratio(p, gear);
        if (after < limit - 400) pendingGear_ = gear - 1;
    }
}

double Vehicle::step(double dt, double we, const EngineParams& p, double& pedal) {
    if (!sqInit_) { squealBp_.bandpass(1150, 3.0, 1.0 / dt); sqInit_ = true; }
    const int nGears = clampv((int)p.gears, 1, kMaxGears);
    const double rpm = we * 60.0 / (2 * kPi);
    const double r = std::max(0.15, p.tireRadius_mm * 1e-3);
    const double m = std::max(100.0f, p.mass_kg);
    const double throttle = clampv(pedal, 0.0, 1.0);

    // ---------------------------------------------------------------- gear selection
    sinceShift_ += dt;
    if (p.transMode == TransMode::Automatic) {
        // decide on road speed (what the engine would turn with the clutch locked and no wheelspin)
        double rpmRoad = v / std::max(0.15, p.tireRadius_mm * 1e-3) * ratio(p, std::max(gear, 1)) * 60.0 / (2 * kPi);
        autoShift(gear > 0 ? std::min(rpm, rpmRoad + 300.0) : rpm, throttle, p);
    } else if (p.gear != lastRequest_) {
        pendingGear_ = clampv((int)p.gear, 0, nGears);
    }
    lastRequest_ = p.gear;
    const bool autoClutch = p.transMode != TransMode::Manual;
    if (pendingGear_ != gear && !shifting) {
        if (autoClutch) { shifting = true; shiftTimer_ = 0; }
        else { gear = pendingGear_; sinceShift_ = 0; }   // manual: you'd better have the clutch in
    }
    if (shifting) {
        shiftTimer_ += dt;
        if (shiftTimer_ > 0.12 && gear != pendingGear_) { gear = pendingGear_; sinceShift_ = 0; clutchLocked = false; }
        if (shiftTimer_ > 0.2) shifting = false;
    }
    const double g = ratio(p, gear);

    // ---------------------------------------------------------------- clutch engagement
    if (!autoClutch) {
        double pd = clampv((double)p.clutchPedal, 0.0, 1.0);
        engagement = smoothstep(0.15, 0.85, 1.0 - pd);
    } else {
        // automatic clutch: slips to hold the engine near a launch speed, then locks
        double launch = std::max(900.0, p.idleTargetRpm + 350.0) + throttle * 1500.0;
        if (gear == 0 || shifting) autoEng_ = 0;
        else if (clutchLocked && rpm > launch - 200) autoEng_ = std::min(1.0, autoEng_ + dt * 3.0);
        else autoEng_ = clampv(autoEng_ + clampv((rpm - launch) * 0.004, -6.0, 6.0) * dt, 0.0, 1.0);
        // anti-stall: about to stall -> let the clutch go right away
        if (gear > 0 && !shifting && rpm < std::max(500.0, p.idleTargetRpm * 0.75)) autoEng_ = std::max(0.0, autoEng_ - 25.0 * dt);
        // the driver can still override with the pedal
        engagement = std::min(autoEng_, smoothstep(0.15, 0.85, 1.0 - clampv((double)p.clutchPedal, 0.0, 1.0)));
    }
    const double cap = std::max(10.0f, p.clutchTorque) * engagement;

    // ---------------------------------------------------------------- clutch torque (stick / slip)
    inputOmega = gear > 0 ? wheelOmega * g : inputOmega;
    double slipW = we - inputOmega;
    double T;
    if (clutchLocked) {
        twist_ += slipW * dt;
        T = kClutchK * twist_ + kClutchC * slipW;
        if (std::fabs(T) > cap) { clutchLocked = false; twist_ = 0; T = clampv(T, -cap, cap); }
    } else {
        T = cap * std::tanh(slipW / 3.0);
        if (cap > 1.0 && std::fabs(slipW) < 0.8) { clutchLocked = true; twist_ = T / kClutchK; }
    }
    if (cap <= 1.0) { clutchLocked = false; T = 0; }
    clutchTorque = T;
    {   // boxcar over exactly one working cycle (crank angle based, like the engine's own torque average)
        const int nb = isRotary(p) ? 1080 : 720;
        const double dDeg = we * dt * 180.0 / kPi;
        avgPhase_ = std::fmod(avgPhase_ + dDeg, (double)nb);
        const int bin = clampv((int)avgPhase_, 0, nb - 1);
        if (bin != avgBin_) {
            int b = avgBin_ % nb;
            while (b != bin) { b = (b + 1) % nb; avgSum_ -= avgBins_[b]; avgBins_[b] = 0; }
            avgBin_ = bin;
        }
        avgBins_[bin] += T * dDeg; avgSum_ += T * dDeg;
        avgSeen_ = std::min((double)nb, avgSeen_ + dDeg);
        const double tau = 0.05;   // nearly stopped engine: plain smoothing instead
        clutchTorqueAvg = rpm > 200 && avgSeen_ > 1 ? avgSum_ / avgSeen_ : clutchTorqueAvg + (T - clutchTorqueAvg) * std::min(1.0, dt / tau);
    }

    // ---------------------------------------------------------------- driven wheels & tyre
    const double N = m * kG * clampv((double)p.driveFrac, 0.2, 1.0);
    double denom = std::max(std::fabs(v), 2.5);
    slip = (wheelOmega * r - v) / denom;
    // Pacejka-like longitudinal force: peak near 18 % slip, ~75 % of peak left when spinning
    double Fx = clampv((double)p.grip, 0.2, 2.0) * N * std::sin(1.45 * std::atan(10.0 * slip));
    double brakeF = clampv((double)p.brake, 0.0, 1.0) * 1.05 * m * kG;      // ~1 g at full brake
    double brakeDriven = brakeF * clampv((double)p.driveFrac, 0.2, 1.0);
    double brakeOther = brakeF - brakeDriven;
    double eff = T >= 0 ? clampv((double)p.drivelineEff, 0.5, 1.0) : 1.0 / clampv((double)p.drivelineEff, 0.5, 1.0);
    double Tw = gear > 0 ? T * g * eff : 0.0;
    double Jw = kWheelJ + (gear > 0 ? kInputJ * g * g : 0.0);
    double Tb = brakeDriven * r * std::tanh(wheelOmega / 0.5);
    wheelOmega += (Tw - Fx * r - Tb) / Jw * dt;
    if (wheelOmega < 0) wheelOmega = 0;          // no reverse gear
    if (gear == 0) inputOmega += (T - 0.4 * std::tanh(inputOmega)) / kInputJ * dt;
    if (inputOmega < 0) inputOmega = 0;
    driveForce = Fx;

    // ---------------------------------------------------------------- body
    double rolling = p.crr * m * kG * std::tanh(v / 0.3);
    double aero = 0.5 * kRhoAir * std::max(0.1f, p.cdA) * v * v;
    double gradeF = m * kG * std::sin(std::atan(p.grade_pct * 0.01));
    double bOther = brakeOther * std::tanh(v / 0.3);
    v += (Fx - rolling - aero - gradeF - bOther) / m * dt;
    if (v < 0) { v = 0; }
    distance += v * dt;

    // ---------------------------------------------------------------- performance timer
    if (v < 0.2) { timing = true; runTime = 0; }
    else if (timing) {
        runTime += dt;
        if (t100 == 0 && v >= 100.0 / 3.6) t100 = runTime;
        if (distance >= 402.3 && tQuarter == 0) { tQuarter = runTime; vQuarter = v * 3.6; }
        if (runTime > 60) timing = false;
    }
    if (v < 0.2) { distance = 0; if (throttle < 0.05) { t100 = 0; tQuarter = 0; vQuarter = 0; } }

    // ---------------------------------------------------------------- tyre squeal
    double sq = clampv((std::fabs(slip) - 0.12) * 3.0, 0.0, 1.0) * clampv(v / 3.0 + wheelOmega * r / 6.0, 0.0, 1.0);
    sound_ = sq > 0 ? squealBp_.tick(rng_.bipolar()) * 6.0 * sq : squealBp_.tick(0);
    return T;
}

} // namespace es

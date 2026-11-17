#include "sim/Dyno.h"

namespace es {

const char* dynoCorrectionNames() { return "None (observed)\0SAE J1349\0DIN 70020\0ISO 1585 / EEC 80/1269\0SAE J607\0"; }
const char* dynoCorrectionName(DynoCorrection c) {
    static const char* n[] = {"observed", "SAE J1349", "DIN 70020", "ISO 1585", "SAE J607"};
    return n[clampv((int)c, 0, 4)];
}

// correction factors to each standard's reference air (the simulated air is dry, so dry-air pressure = baro)
double dynoCorrectionFactor(DynoCorrection c, double tC, double pKPa) {
    const double T = std::max(200.0, tC + 273.15), p = std::max(10.0, pKPa);
    switch (c) {
    case DynoCorrection::SaeJ1349: return 1.18 * (99.0 / p) * std::sqrt(T / 298.0) - 0.18;   // 25 C, 99 kPa dry
    case DynoCorrection::Din70020: return (101.3 / p) * std::sqrt(T / 293.0);                 // 20 C, 1013 mbar
    case DynoCorrection::Iso1585: return std::pow(99.0 / p, 1.2) * std::pow(T / 298.0, 0.6);  // 25 C, 99 kPa dry
    case DynoCorrection::SaeJ607: return (101.32 / p) * std::sqrt(T / 288.9);                 // 60 F, 29.92 inHg
    default: return 1.0;
    }
}

double Dyno::pi(double dt, double rpm, double J) {
    double err = rpm - target;
    double kp = 3.0 * J + 0.05;   // N m per rpm
    double ki = 6.0 * kp;
    integ_ = clampv(integ_ + ki * err * dt, 0.0, 3000.0);
    return clampv(kp * err + integ_, 0.0, 4000.0);
}

void Dyno::accumulate(const Engine& e) {
    // brake torque = cycle averaged net engine torque
    sumT_ += e.netTorque(); sumAfr_ += e.lambdaAvg * kStoich; sumVe_ += e.veAvg; ++cnt_;
    sumBoost_ += (e.intake.map() - kPatm) * 1e-5;
    sumEgt_ += e.egtProbe() - 273.15;
    sumAdv_ += e.advance();
    sumFuel_ += e.intake.fuelFlow();
    sumIat_ += e.thermal.iatC;
}

DynoPoint Dyno::average(double rpm) const {
    DynoPoint d;
    const double n = std::max(1, cnt_);
    d.rpm = (float)rpm;
    d.torque = (float)(sumT_ / n);
    d.powerKw = (float)(d.torque * d.rpm * 2 * kPi / 60.0 / 1000.0);
    d.afr = (float)(sumAfr_ / n);
    d.ve = (float)(sumVe_ / n);
    d.boostBar = (float)(sumBoost_ / n);
    d.egtC = (float)(sumEgt_ / n);
    d.advance = (float)(sumAdv_ / n);
    d.iatC = (float)(sumIat_ / n);
    d.bsfc = d.powerKw > 0.5f ? (float)(sumFuel_ / n * 3.6e6 / d.powerKw) : 0.0f;
    return d;
}

double Dyno::step(double dt, const EngineParams& p, const Engine& e, double& pedal) {
    const double rpm = e.rpm();
    const double J = p.flywheel;
    auto resetSums = [this] { sumT_ = sumAfr_ = sumVe_ = sumBoost_ = sumEgt_ = sumAdv_ = sumFuel_ = sumIat_ = 0; cnt_ = 0; };

    if (p.sweepRequest != lastRequest_) {
        lastRequest_ = p.sweepRequest;
        if (p.dynoMode == DynoMode::Sweep || p.dynoMode == DynoMode::Step) {
            state = Settling; timer_ = 0; target = p.sweepStartRpm; ++runCounter;
            binRpm_ = p.sweepStartRpm; resetSums(); progress = 0; measuring_ = false;
        }
    }
    const double span = std::max(100.0, (double)(p.sweepEndRpm - p.sweepStartRpm));

    switch (p.dynoMode) {
    case DynoMode::Off:
        state = Idle; integ_ = 0; load = 0;
        return 0;
    case DynoMode::Hold:
        state = Holding; target = p.dynoHoldRpm;
        load = pi(dt, rpm, J);
        return load;
    case DynoMode::Step:
        // steady-state test: hold every step at WOT until rpm and torque have settled, then average
        if (state == Settling || state == Stepping) {
            pedal = 1.0;
            load = pi(dt, rpm, J);
            timer_ += dt;
            if (state == Settling) {
                if (timer_ > 2.0 && std::fabs(rpm - target) < 60) { state = Stepping; timer_ = 0; measuring_ = false; resetSums(); }
                return load;
            }
            if (!measuring_) {
                if (timer_ > 0.8 && std::fabs(rpm - target) < 30) { measuring_ = true; timer_ = 0; resetSums(); }
                return load;
            }
            accumulate(e);
            if (timer_ >= std::max(0.3f, p.stepHold_s)) {
                newPoints.push_back(average(target));
                target += std::max(100.0f, p.stepRpm);
                progress = clampv((target - p.sweepStartRpm) / span, 0.0, 1.0);
                timer_ = 0; measuring_ = false;
                if (target > p.sweepEndRpm + 1 || target > p.revLimit - 50) { state = Done; integ_ = 0; progress = 1; }
            }
            return load;
        }
        load = 0;
        return 0;
    case DynoMode::Sweep:
    default:
        if (state == Settling) {
            pedal = 1.0;
            target = p.sweepStartRpm;
            load = pi(dt, rpm, J);
            timer_ += dt;
            if (timer_ > 2.0 && std::fabs(rpm - target) < 80) { state = Ramping; timer_ = 0; }
            return load;
        }
        if (state == Ramping) {
            pedal = 1.0;
            target += p.sweepRate * dt;
            load = pi(dt, rpm, J);
            progress = clampv((target - p.sweepStartRpm) / span, 0.0, 1.0);
            accumulate(e);
            if (rpm >= binRpm_ + binW_) {
                if (cnt_ > 0 && binRpm_ >= p.sweepStartRpm + binW_)   // skip the first bin (transient)
                    newPoints.push_back(average(binRpm_ + binW_ * 0.5));
                binRpm_ += binW_; resetSums();
            }
            if (target >= p.sweepEndRpm || rpm > p.revLimit - 50) { state = Done; integ_ = 0; }
            return load;
        }
        // idle or done: free rev
        load = 0;
        return 0;
    }
}

} // namespace es

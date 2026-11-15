#include "sim/Dyno.h"

namespace es {

double Dyno::pi(double dt, double rpm, double J) {
    double err = rpm - target;
    double kp = 3.0 * J + 0.05;   // N m per rpm
    double ki = 6.0 * kp;
    integ_ = clampv(integ_ + ki * err * dt, 0.0, 3000.0);
    return clampv(kp * err + integ_, 0.0, 4000.0);
}

double Dyno::step(double dt, const EngineParams& p, const Engine& e, double& pedal) {
    const double rpm = e.rpm();
    const double J = p.flywheel;

    if (p.sweepRequest != lastRequest_) {
        lastRequest_ = p.sweepRequest;
        if (p.dynoMode == DynoMode::Sweep) {
            state = Settling; timer_ = 0; target = p.sweepStartRpm; ++runCounter;
            binRpm_ = p.sweepStartRpm; sumT_ = sumAfr_ = sumVe_ = 0; cnt_ = 0; progress = 0;
        }
    }

    switch (p.dynoMode) {
    case DynoMode::Off:
        state = Idle; integ_ = 0; load = 0;
        return 0;
    case DynoMode::Hold:
        state = Holding; target = p.dynoHoldRpm;
        load = pi(dt, rpm, J);
        return load;
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
            double span = std::max(100.0, (double)(p.sweepEndRpm - p.sweepStartRpm));
            progress = clampv((target - p.sweepStartRpm) / span, 0.0, 1.0);
            // brake torque = cycle averaged net engine torque
            sumT_ += e.netTorque(); sumAfr_ += e.lambdaAvg * kStoich; sumVe_ += e.veAvg; ++cnt_;
            if (rpm >= binRpm_ + binW_) {
                if (cnt_ > 0 && binRpm_ >= p.sweepStartRpm + binW_) { // skip the first bin (transient)
                    DynoPoint d;
                    d.rpm = (float)(binRpm_ + binW_ * 0.5);
                    d.torque = (float)(sumT_ / cnt_);
                    d.powerKw = (float)(d.torque * d.rpm * 2 * kPi / 60.0 / 1000.0);
                    d.afr = (float)(sumAfr_ / cnt_);
                    d.ve = (float)(sumVe_ / cnt_);
                    newPoints.push_back(d);
                }
                binRpm_ += binW_; sumT_ = sumAfr_ = sumVe_ = 0; cnt_ = 0;
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

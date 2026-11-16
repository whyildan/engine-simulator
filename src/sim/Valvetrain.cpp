#include "sim/Valvetrain.h"
#include "sim/EngineParams.h"
#include "core/MathUtil.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
// cam-to-valve stiffness of each lifter type (N/m): long pushrod trains are soft, direct-acting buckets stiff
const double kLifterStiffness[] = {5.0e6, 5.8e6, 1.2e7, 9.0e6};   // ~700-900 Hz pushrod, ~1.5-1.7 kHz overhead cam
// the cam profile of ValveSpec::liftAt (follower lift / max lift), x = deg from the centre / half duration
inline double camF(double x) { return std::fabs(x) >= 1.0 ? 0.0 : std::pow(0.5 * (1.0 + std::cos(kPi * x)), 0.75); }
// The dynamics are driven by a surrogate of that profile: the plain profile has an unbounded acceleration
// where it leaves the base circle (a real cam has a designed ramp there) which would kick the train into
// ringing. The surrogate (1 - x^2)^3 (1 + 1.15 x^2) leaves and lands with zero velocity and acceleration and
// has the same nose curvature (-3.70) - the nose deceleration is what the spring has to provide.
inline double camSmooth(double x) {
    if (std::fabs(x) >= 1.0) return 0.0;
    const double u = 1.0 - x * x;
    return u * u * u * (1.0 + 1.15 * x * x);
}
} // namespace

ValvetrainSpec valvetrainSpec(const EngineParams& p, bool intake) {
    ValvetrainSpec s;
    ComponentModel cm;
    cm.build(p);
    s.mass = std::max(0.01, intake ? cm.moveMassIn : cm.moveMassEx);
    // the spring is specified at the full lift of the biggest cam it has to control
    double liftMax = intake ? p.intakeLift_mm : p.exhaustLift_mm;
    if (p.vtec) liftMax = std::max(liftMax, (double)(intake ? p.vtecInLift_mm : p.vtecExLift_mm));
    liftMax = std::max(1.0, liftMax) * 1e-3;
    s.preload = std::max(50.0, (double)p.comp.springSeat_N);
    s.rate = std::max(0.0, (double)p.comp.springOpen_N - s.preload) / liftMax;
    s.stiffness = kLifterStiffness[clampv((int)p.comp.lifter, 0, 3)];
    s.lash = std::max(0.0, (double)p.valveLash_mm) * 1e-3;
    return s;
}

void ValveDynamics::compute(const ValvetrainSpec& vt, double lift, double dur, double rpm) {
    rpmKey = rpm; liftKey = lift; durKey = dur;
    loft = bounce = lateClose = 0;
    const double h = std::max(30.0, dur * 0.5);
    const double w = std::max(100.0, rpm) * 6.0;                     // crank deg / s
    const double extra = clampv(w * 0.004, 40.0, 200.0);              // after the nominal closing: room for bounces
    const double step = 0.5;
    n_ = std::min(kN, (int)((2.1 * h + extra) / step) + 2);
    d0_ = -1.05 * h; inv_ = 1.0 / step;
    const double m = vt.mass, kc = vt.stiffness, ks = vt.seatStiffness, k = vt.rate, F0 = vt.preload, lash = vt.lash;
    const double cc = 2.0 * 0.08 * std::sqrt(kc * m);                // train damping (zeta ~ 0.08)
    const double cs = 2.0 * 0.25 * std::sqrt(ks * m);                // seat impact: restitution ~ 0.45
    const double dtMax = 2.0 * kPi / std::sqrt(std::max(kc, ks) / m) / 30.0;
    const double cellT = step / w;
    const int sub = std::max(1, (int)std::ceil(cellT / dtMax));
    const double dt = cellT / sub;
    double x = -F0 / ks, v = 0;                                       // resting on the seat
    bool reseated = false;
    double kinClose = 1e9;                                            // where the kinematic lift ends (d > 0)
    // real cams take up the lash on a gentle opening ramp; this profile has none, so the valve is driven by
    // the lift above the lash (no lash impact) - what is left is the inertia of the valve against its spring
    const double lv = std::max(1e-5, lift - lash);
    double c0 = lv * camSmooth(d0_ / h);
    if (keepTrace) { trD.assign(n_, 0); trValve.assign(n_, 0); trCam.assign(n_, 0); }
    for (int i = 0; i < n_; ++i) {
        const double d = d0_ + i * step;
        const double c1 = lv * camSmooth((d + step) / h);
        const double kin = c0;
        const double qs = c0 > F0 / kc ? (c0 - F0 / kc) / (1.0 + k / kc) : 0.0;   // quasi-static lift
        const double valve = std::max(0.0, x);
        dev_[i] = (float)(valve - qs);
        if (keepTrace) { trD[i] = (float)d; trValve[i] = (float)valve; trCam[i] = (float)kin; }
        if (c0 > 0 && x > 0) loft = std::max(loft, x - c0);
        if (d > 0 && kin <= 0 && kinClose > 1e8) kinClose = d;
        if (d > 0 && x <= 0) reseated = true;
        if (reseated) bounce = std::max(bounce, x);
        if (d > kinClose && valve > 5e-6) lateClose = d - kinClose;
        // integrate across the cell; the cam is linear inside it
        const double cdot = (c1 - c0) / step * w;
        for (int s = 0; s < sub; ++s) {
            const double c = c0 + (c1 - c0) * (s + 0.5) / sub;
            const double delta = c - x;
            const double Fc = delta > 0 ? std::max(0.0, kc * delta + cc * (cdot - v)) : 0.0;
            const double Fs = F0 + k * std::max(0.0, x);
            const double Fseat = x < 0 ? std::max(0.0, ks * (-x) - cs * v) : 0.0;
            v += (Fc - Fs + Fseat) / m * dt;
            x += v * dt;
        }
        c0 = c1;
    }
    active_ = true;
}

double ValveDynamics::severity(double lift) const {
    if (!active_ || lift <= 0) return 0;
    const double s = std::max(loft, 1.5 * bounce);
    return clampv((s - 0.02 * lift) / (0.10 * lift), 0.0, 1.0);
}

double dynamicFloatRpm(const ValvetrainSpec& vt, double lift, double dur, double maxRpm) {
    ValveDynamics d;
    for (double rpm = 2500; rpm <= maxRpm; rpm += 100) {
        d.compute(vt, lift, dur, rpm);
        if (d.severity(lift) > 0.02) return rpm;
    }
    return maxRpm;
}

} // namespace es

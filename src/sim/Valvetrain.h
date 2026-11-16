#pragma once
// Valvetrain dynamics (Phase 6): the valve as a mass on its spring, driven by the cam through a stiff but
// elastic train (follower / rocker / pushrod), landing on an elastic seat.
//
// Below the float speed the valve follows the cam (apart from a few microns of flex). Past it the spring
// cannot decelerate the valve over the cam nose: the valve leaves the follower (loft), comes back late, hits
// the closing flank or the seat and bounces. The motion of one event is integrated for the current rpm and
// stored as a deviation table from the kinematic lift, which the cylinder flow then uses.
#include <vector>

namespace es {

struct EngineParams;

struct ValvetrainSpec {
    double mass = 0.08;        // moving mass per valve (valve, retainer, part of the spring, follower) kg
    double preload = 250;      // spring force on the seat (N)
    double rate = 25000;       // spring rate (N/m)
    double stiffness = 6e6;    // cam-to-valve train stiffness (N/m)
    double seatStiffness = 3e7;
    double lash = 0.0002;      // m
};

// the valvetrain of this engine: intake or exhaust valves, base or high (VTEC) cam
ValvetrainSpec valvetrainSpec(const EngineParams& p, bool intake);

class ValveDynamics {
public:
    // motion of one valve event: cam lift (m), duration (deg), at this engine speed
    void compute(const ValvetrainSpec& vt, double lift, double dur, double rpm);
    void clear() { active_ = false; loft = bounce = lateClose = 0; rpmKey = 0; }
    // valve lift minus the kinematic lift (m) at d crank deg from the cam centre
    double dev(double d) const {
        if (!active_) return 0;
        const double x = (d - d0_) * inv_;
        if (x < 0 || x >= n_ - 1) return 0;
        const int i = (int)x;
        const double w = x - i;
        return dev_[i] + w * (dev_[i + 1] - dev_[i]);
    }
    bool active() const { return active_; }
    double severity(double lift) const;   // 0 = follows the cam, 1 = severe float / bounce
    // results of the last compute
    double loft = 0;           // largest gap between the valve and its follower (m)
    double bounce = 0;         // largest lift after the valve first reached its seat (m)
    double lateClose = 0;      // crank degrees the valve seats after the kinematic closing point
    // what the table was computed for
    double rpmKey = 0, liftKey = 0, durKey = 0;
    // full traces of the last compute (for plots): crank deg from the cam centre, valve and kinematic lift (m)
    bool keepTrace = false;
    std::vector<float> trD, trValve, trCam;

private:
    static constexpr int kN = 1400;
    float dev_[kN] = {};
    double d0_ = 0, inv_ = 2;
    int n_ = 0;
    bool active_ = false;
};

// lowest rpm at which this valve visibly floats (loft or bounce), scanning upwards (UI)
double dynamicFloatRpm(const ValvetrainSpec& vt, double lift, double dur, double maxRpm = 16000);

} // namespace es

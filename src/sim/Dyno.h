#pragma once
#include "sim/Engine.h"
#include <vector>

namespace es {

struct DynoPoint { float rpm, torque, powerKw, afr, ve; };

// Eddy-current style absorber: free rev, hold RPM (PI), or a WOT sweep that records curves.
class Dyno {
public:
    enum State { Idle = 0, Holding = 1, Settling = 2, Ramping = 3, Done = 4 };

    // returns absorber torque; may override the pedal (sweep forces WOT)
    double step(double dt, const EngineParams& p, const Engine& e, double& pedal);

    State state = Idle;
    double target = 0;
    double load = 0;
    double progress = 0;      // 0..1 during a sweep
    int runCounter = 0;       // increments when a sweep starts (UI uses it to group points)
    std::vector<DynoPoint> newPoints;

private:
    double pi(double dt, double rpm, double J);
    int lastRequest_ = 0;
    double integ_ = 0, timer_ = 0;
    double binRpm_ = 0, sumT_ = 0, sumAfr_ = 0, sumVe_ = 0; int cnt_ = 0;
    double binW_ = 100;
};

} // namespace es

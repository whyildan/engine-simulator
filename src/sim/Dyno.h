#pragma once
#include "sim/Engine.h"
#include <vector>

namespace es {

struct DynoPoint {
    float rpm, torque, powerKw, afr, ve;
    float boostBar = 0, egtC = 0, advance = 0, bsfc = 0, iatC = 0;   // bsfc: g/kWh
};

// standards that correct measured power to reference air (temperature and dry-air pressure at the inlet)
enum class DynoCorrection : int32_t { None = 0, SaeJ1349 = 1, Din70020 = 2, Iso1585 = 3, SaeJ607 = 4 };
const char* dynoCorrectionNames();   // ImGui combo list
const char* dynoCorrectionName(DynoCorrection c);
double dynoCorrectionFactor(DynoCorrection c, double inletTempC, double baroKPa);

// Eddy-current style absorber: free rev, hold RPM (PI), a WOT sweep that records curves, or a WOT step test
// that holds each rpm until it has settled and averages it (steady-state points, no sweep-rate effects).
class Dyno {
public:
    enum State { Idle = 0, Holding = 1, Settling = 2, Ramping = 3, Done = 4, Stepping = 5 };

    // returns absorber torque; may override the pedal (sweep forces WOT)
    double step(double dt, const EngineParams& p, const Engine& e, double& pedal);

    State state = Idle;
    double target = 0;
    double load = 0;
    double progress = 0;      // 0..1 during a sweep / step test
    int runCounter = 0;       // increments when a sweep starts (UI uses it to group points)
    std::vector<DynoPoint> newPoints;

private:
    double pi(double dt, double rpm, double J);
    void accumulate(const Engine& e);
    DynoPoint average(double rpm) const;
    int lastRequest_ = 0;
    double integ_ = 0, timer_ = 0;
    double binRpm_ = 0, sumT_ = 0, sumAfr_ = 0, sumVe_ = 0; int cnt_ = 0;
    double sumBoost_ = 0, sumEgt_ = 0, sumAdv_ = 0, sumFuel_ = 0, sumIat_ = 0;
    double binW_ = 100;
    bool measuring_ = false;
};

} // namespace es

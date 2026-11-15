#pragma once
// Car (or motorcycle) as the engine's load: friction clutch (stick / slip), manual gearbox with
// final drive, driven wheels with a slip-based tyre model (wheelspin), vehicle mass, rolling
// resistance, aerodynamic drag, road grade and brakes. Optional automatic clutch and an
// automatic shift strategy.
#include "sim/EngineParams.h"
#include "audio/Filters.h"
#include "core/MathUtil.h"

namespace es {

class Vehicle {
public:
    void reset();
    // one simulation step. Returns the clutch torque acting on the crank (positive = load).
    // `pedal` may be modified (automatic throttle blip on downshifts is not used; kept for API).
    double step(double dt, double engineOmega, const EngineParams& p, double& pedal);
    double sound() const { return sound_; }   // tyre squeal (Pa @ 1 m)

    // ---- state / telemetry
    int gear = 0;                 // engaged gear (0 = neutral)
    double v = 0;                 // m/s
    double wheelOmega = 0;        // driven wheels rad/s
    double inputOmega = 0;        // gearbox input shaft rad/s
    double engagement = 0;        // clutch 0..1
    double clutchTorque = 0;
    double slip = 0;              // tyre slip ratio
    double driveForce = 0;
    double distance = 0;
    bool clutchLocked = false;
    bool shifting = false;
    // performance timer
    double runTime = 0, t100 = 0, tQuarter = 0, vQuarter = 0;
    bool timing = false;

private:
    void autoShift(double rpm, double throttle, const EngineParams& p);
    double ratio(const EngineParams& p, int g) const;

    double twist_ = 0;
    double shiftTimer_ = 0, sinceShift_ = 10;
    int pendingGear_ = 0, lastRequest_ = -99;
    double autoEng_ = 0;
    double sound_ = 0;
    Biquad squealBp_;
    bool sqInit_ = false;
    Rng rng_{99};
};

} // namespace es

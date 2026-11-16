#pragma once
// Virtual sensor layer: turns the true engine state into what real sensors would report
// (response lag, noise, ADC resolution, transport delay, injected faults) plus the ECU-side
// plausibility check (out-of-range detection -> fallback value). The ECU only ever sees
// SensorReadings, never the physics.
#include "ecu/SensorDefs.h"
#include "core/MathUtil.h"
#include "sim/EngineParams.h"

namespace es {

// switch inputs and the driven-wheel speed (digital / frequency inputs: read as they are, no lag or noise model)
struct DigitalInputs {
    bool clutch = false;        // clutch switch (pedal more than half down)
    bool transbrake = false;
    bool n2oArm = false;
    bool rollingAls = false;    // rolling anti-lag button
    bool shift = false;         // gear-change signal (gearbox actuator / shift switch)
    double wheelKmh = 0;        // driven wheel speed
    bool crankSync = true;      // trigger decoder: crank angle known (a gap / the cam tooth has passed)
    bool camSync = true;        // cycle phase known (cam tooth seen): sequential injection possible
};

struct SensorTruth {
    double v[kSensorCount] = {};   // physical truth in each sensor's unit (O2 / wideband: lambda)
    // knock per cylinder (port) from the crank-angle window, and its cycle counter (one value per combustion)
    double knockCyl[kMaxCyl] = {};
    int knockSeq[kMaxCyl] = {};
    DigitalInputs in;
};

enum SensorStatus : int { kStatusOk = 0, kStatusLow = 1, kStatusHigh = 2, kStatusNoSignal = 3 };

struct SensorReadings {
    double value[kSensorCount] = {};     // what the ECU uses (fallback substituted when implausible)
    double measured[kSensorCount] = {};  // what the sensor reports (before the plausibility check)
    double volts[kSensorCount] = {};     // signal at the ECU pin (analog sensors)
    int status[kSensorCount] = {};
    bool fallback[kSensorCount] = {};
    double knockCyl[kMaxCyl] = {};       // windowed knock intensity per cylinder (0 when the knock sensor failed)
    int knockSeq[kMaxCyl] = {};
    DigitalInputs in;
    bool ok(int id) const { return !fallback[id]; }
};

class SensorLayer {
public:
    void reset(const EngineParams& p);
    void update(double dt, const SensorTruth& t, const EngineParams& p);
    SensorReadings r;
    SensorTruth truth;                   // last truth (for diagnostics / UI only - never for the ECU)
private:
    double filt_[kSensorCount] = {};
    double stuck_[kSensorCount] = {};
    SensorFault prevFault_[kSensorCount] = {};
    static constexpr int kHist = 512;
    double lamHist_[kHist] = {};         // wideband / O2 transport delay line (ECU-rate samples)
    int histW_ = 0;
    Rng rng_{4242};
    bool init_ = false;
};

} // namespace es

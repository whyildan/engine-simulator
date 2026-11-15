#pragma once
// Forced induction: turbocharger (exhaust turbine + centrifugal compressor on a free shaft) or
// supercharger (crank driven Roots / twin-screw positive displacement blower, or centrifugal).
// The compressor fills the intake "charge" node that feeds the throttle; the turbine sits
// between the exhaust collector node and the downpipe.
#include "sim/EngineParams.h"
#include "sim/Exhaust.h"
#include "sim/Intake.h"
#include "audio/Filters.h"

namespace es {

class ForcedInduction {
public:
    void init(const EngineParams& p, double fs);
    void apply(const EngineParams& p) { p_ = p; }

    // one simulation step; returns the torque the supercharger drive takes from the crank
    double step(double dt, double engineOmega, double egtK, IntakeSystem& in, ExhaustSystem& ex);

    double sound() const { return sound_; }       // Pa @ 1 m (whine, whoosh, flutter)

    // compressor map in normalised form, for the UI
    struct MapPoint { double flow_gs, pr; };
    static double prAtZeroFlow(double n);            // pressure ratio at the surge edge for speed n
    double chokeFlow(double n) const;                // kg/s

    // telemetry
    double shaftRpm = 0;        // turbo / impeller / blower rpm
    double speedFrac = 0;       // n = N / Nmax
    double boostBar = 0;        // charge pressure (gauge)
    double pr = 1;              // compressor pressure ratio
    double compFlow = 0;        // kg/s through the compressor (negative in surge)
    double wastegate = 0;       // 0..1
    double chargeT = 300;       // K, after the intercooler
    double powerKw = 0;         // compressor power (turbo: also the turbine power at balance)
    bool surging = false, bovOpen = false;

private:
    double compressor(double n, double prc, double& eta) const;  // centrifugal map -> kg/s

    EngineParams p_{};
    double fs_ = 96000;
    double ambT_ = 298;
    double omegaT_ = 0;         // turbo shaft rad/s
    double wgPos_ = 0, wgTrim_ = 0, prevBoost_ = 0, boostRate_ = 0;
    double sound_ = 0, phase_ = 0, phase2_ = 0, phase3_ = 0, flutterPh_ = 0;
    // sound shaping
    Biquad whooshBp_, bovHiss_, bovBody_, surgeBp_, surgeLow_, scHiss_;
    OnePole whineLp_, ampSmooth_, wobble_;
    double bovEnv_ = 0, thumpEnv_ = 0, thumpPh_ = 0, surgeEnv_ = 0, surgePitch_ = 0;
    bool prevBov_ = false;
    double prevSurgePh_ = 0;
    Rng rng_{4242};
};

} // namespace es

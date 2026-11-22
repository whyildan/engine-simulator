#pragma once
// Forced induction: turbocharger (exhaust turbine + centrifugal compressor on a free shaft) or
// supercharger (crank driven Roots / twin-screw positive displacement blower, or centrifugal).
// The compressor fills the intake "charge" node that feeds the throttle; the turbine sits
// between the exhaust collector node and the downpipe.
#include "sim/EngineParams.h"
#include "sim/Exhaust.h"
#include "sim/Intake.h"
#include "sim/TurboMap.h"
#include "audio/Filters.h"

namespace es {

class ForcedInduction {
public:
    void init(const EngineParams& p, double fs);
    void apply(const EngineParams& p) { p_ = p; refreshMaps(); }
    // air speed through the intercooler core (vehicle speed or the dyno cell fan)
    void setCoolingAir(double ms) { coolAir_ = ms; }

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
    // ECU boost control: wastegate solenoid duty 0..1, -1 = none (the spring alone holds its pressure)
    void setWgDuty(double d) { wgDuty_ = d; }
    double chargeT = 300;       // K, after the intercooler
    double icCoreT = 300;       // K, intercooler core (heat soak)
    double compEff = 0;         // compressor isentropic efficiency now
    double compFlowCorr = 0;    // kg/s corrected to 298 K / 101.325 kPa (map axis)
    double turbEff = 0;         // turbine efficiency now
    double turbineDropK = 0;    // exhaust temperature drop across turbine + wastegate (mixed, K)
    double turbPowAvg_ = 0, turbFlowAvg_ = 0;
    double turbEr = 1;          // turbine expansion ratio
    double turbUc = 0;          // blade speed ratio U / C
    double turbMfp = 0;         // turbine reduced flow, kg/s sqrt(K) / kPa
    double surgeMargin = 1;     // 1 - pr / surge pr at this speed (< 0: in surge)
    bool mapCompressor() const { return mapC_; }
    bool mapTurbine() const { return mapT_; }
    const CompressorModel& compressorModel() const { return cm_; }
    double powerKw = 0;         // compressor power (turbo: also the turbine power at balance)
    // flow map: compressor outlet temperature (before the intercooler, K) and this step's flows (kg/s, signed)
    double compOutT = 300, ventFlow = 0, turbFlow = 0, wgFlow = 0;
    bool surging = false, bovOpen = false;
    double lineS = 0;           // the speed line's surge (peak) flow now, kg/s
    double bovLift = 0;         // blow-off valve lift 0..1 (its own spring-mass dynamics)
    double dmDuct() const { return dmDuct_; }
    // twin turbos: the second one's speed; sequential: 0 the first alone, 1 the second pre-spooling / its exhaust
    // valve opening, 2 both on the charge
    int units = 1;
    double shaftRpm2 = 0, speedFrac2 = 0;
    int seqStage = 0;

private:
    void refreshMaps();
    CompressorModel cm_;
    TurbineModel tm_;
    bool mapC_ = false, mapT_ = false;
    CompressorMap compSrc_{};
    TurbineMap turbSrc_{};
    int32_t useSrc_ = -1;
    double nMaxC_ = 190000;     // top speed (rpm) of the compressor in use
    double sizeKgs_ = 0.2;      // its flow size (choke flow at the top speed)
    double coolAir_ = 0;
    double wgDuty_ = -1;
    double compressor(double n, double prc, double& eta) const;  // centrifugal map -> kg/s
    // compression system dynamics (Greitzer): each compressor's duct flow state and its current speed line
    static constexpr int kLineN = 24;
    struct Unit {
        double omega = 0;                          // shaft rad/s (a turbo)
        double mDuct = 0, mPrev = 0, dmDuct = 0;   // kg/s through the compressor, its rate (kg/s^2)
        double lineN = -1, lineS = 0.01, linePrS = 1, lineM[kLineN] = {}, linePr[kLineN] = {};
        double rpm = 0, speedFrac = 0, powerC = 0, eta = 0.7;
        double ph1 = 0, phB = 0;                   // shaft order / blade-pass phases
        Biquad tcnBp;
        OnePole amp;
    };
    Unit u_[2];
    int nU_ = 1;                                  // turbochargers (else 1)
    bool seq_ = false;                            // sequential pair
    double ecv_ = 0, ebv_ = 0;                    // sequential: exhaust control / exhaust bypass (pre-spool) valves
    bool icv_ = false;                            // ... intake control valve (the second compressor on the charge)
    double icvPos_ = 0;                           //     its butterfly 0 shut .. 1 open
    double dmDuct_ = 0;
    double tipR_ = 0.025, ductA_ = 1e-3;          // wheel tip radius (m), inducer / duct area (m2)
    void buildLine(Unit& u, double n, double rpmCorr, double corrM);
    double linePr(const Unit& u, double m) const;

    EngineParams p_{};
    double fs_ = 96000;
    double ambT_ = 298;
    double wgPos_ = 0, wgTrim_ = 0, prevBoost_ = 0, boostRate_ = 0;
    double vanePos_ = 1;       // VGT vane opening 0 (closed) .. 1 (open)
    double sound_ = 0, phase_ = 0, phase2_ = 0, phase3_ = 0;
    // sound shaping
    Biquad whooshBp_, bovHiss_, bovBody_, surgeBp_, scHiss_;
    OnePole whineLp_, ampSmooth_, wobble_, flutLp_;
    double bovX_ = 0, bovV_ = 0;                  // blow-off valve lift (0..1) and its rate
    double bovEnv_ = 0, thumpEnv_ = 0, thumpPh_ = 0;
    bool prevBov_ = false;
    Rng rng_{4242};
};

} // namespace es

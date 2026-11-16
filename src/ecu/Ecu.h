#pragma once
// Engine control unit. Runs its control loop at the configured rate on SensorReadings only and
// answers per-cylinder injection events with a pulse width - it never looks at the physics.
// Everything it does is driven by the calibration (EngineParams::cal): VE / lambda / ignition maps,
// enrichment and correction curves, injector data. For carburetted engines it is only the
// ignition module (ignition map + limiter).
#include "ecu/Sensors.h"
#include "ecu/Calibration.h"
#include "sim/EngineParams.h"
#include <vector>

namespace es {

struct ActuatorCommands {
    double iac = 0.35;            // idle air control valve opening 0..1
    double sparkAdvance = 10;     // deg BTDC (before per-cylinder trims)
    bool sparkCut = false;        // rev limiter (ignition cut)
    bool fuelCut = false;         // rev limiter (fuel cut)
    bool antiLag = false;         // retarded spark + cracked throttle
    double throttleCrack = 0;     // minimum throttle opening requested (anti-lag)
    bool syncLost = false;        // no crank signal: no spark, no injection
    bool batchFire = false;       // no cam signal: inject on every revolution, half each time
    bool limiter = false;
    // Phase 3
    double cylRetard[kMaxCyl] = {};  // knock retard per cylinder (port), deg
    double cutFraction = 0;       // soft limiter: share of sparks cut (0..1)
    double throttleTarget = -1;   // drive-by-wire throttle request 0..1 (-1: the pedal drives the throttle directly)
    double wgDuty = -1;           // wastegate solenoid duty 0..1 (-1: no ECU boost control, spring only)
    // Phase 5
    double fuelCutFraction = 0;   // launch limiter in fuel-cut mode: share of injection events dropped
    double n2oDuty = 0;           // nitrous solenoid duty 0..1 (progressive controller)
};

class Ecu {
public:
    void init(const EngineParams& p, double vdCyl, int ports, double revsPerEvent);
    void apply(const EngineParams& p);
    // control loop (idle, spark, limiter, enrichments, closed loop, auto-tune, anti-lag)
    void update(double dt, const SensorReadings& s, bool starter);
    // injection event of a port: pulse width in seconds (0 = none); share < 1 for batch fire
    double injectionPulse(int port, const SensorReadings& s, double share = 1.0);
    // asynchronous tip-in fuel (kg) per port, taken by the engine right after update()
    std::vector<double> asyncFuel;
    // turn a fuel mass into a pulse width with the ECU's injector data (s)
    double pulseFor(double fuelKg, const SensorReadings& s);
    double cylSpark(int port) const { return port >= 0 && port < kCalMaxCyl ? cal_.cylSpark[port] : 0.0; }

    ActuatorCommands cmd;
    static constexpr double kIacNominal = 0.35;
    bool holdIac = false;         // calibration run: idle valve held at its nominal opening
    double veOverride = 0;        // calibration run: VE % used instead of the speed density table (0 = off)
    const Calibration& cal() const { return cal_; }
    int learnSeq = 0;            // bumps whenever auto-tune changed a VE table
    // diagnostics / telemetry
    double targetAfr = 14.7, injDuty = 0, trim = 1.0, warmup = 0, accel = 0, deadComp_ms = 0, flowComp = 1;
    double mapUsed = 101.3;      // kPa used for the fuel load (sensor or alpha-N fallback)
    double veUsed = 0, loadX = 0, loadY = 0, tpsUsed = 0;   // operating point on the maps
    double airPerCyl = 0;        // g per cylinder event the ECU calculated
    bool closedLoop = false, tuning = false;
    // Phase 3 telemetry
    double stft = 0, ltftNow = 0;            // fuel trims %
    int ltftSeq = 0;                         // bumps when the learned LTFT table changed
    double knockRetard[kMaxCyl] = {};        // deg
    int knockEvents = 0;
    double boostTargetBar = 0, boostBar = 0; // gauge
    bool overboostCut = false, boostCut = false;
    double idleTargetNow = 800;
    int protState[kProtCount] = {};          // 0 ok, 1 warning, 2 action active
    double protValue[kProtCount] = {};
    bool limp = false, shutdown = false, throttleLimited = false;
    // Phase 5 telemetry
    int launchState = kLaunchOff;
    double launchLimitRpm = 0;               // rpm the active launch / shift limiter holds (0 = none)
    double tcSlip = 0, tcTarget = 0, tcReduction = 0, tcRetard = 0, tcCut = 0;   // %, %, 0..1, deg, 0..1
    bool n2oLeanLock = false;
    // a latched engine shutdown clears once the engine has stood still; this clears it at once
    void clearProtection();

private:
    double coldFactor(const SensorReadings& s) const;
    void protectionStep(double dt, const SensorReadings& s, double rpm, double tpsPct, bool running);
    void fuelTrimStep(double dt, const SensorReadings& s, double rpm, bool running);
    void knockStep(double dt, const SensorReadings& s, double rpm);
    void boostStep(double dt, const SensorReadings& s, double rpm, double tpsPct);
    void autoTuneStep(double dt, const SensorReadings& s);
    void motorsportStep(double dt, const SensorReadings& s, double rpm, double tpsPct, bool running);
    EngineParams p_{};
    Calibration cal_{}, lastIn_{};
    double vdCyl_ = 5e-4, revsPerEvent_ = 2.0;
    int ports_ = 4;
    std::vector<double> filmEst_, evapAcc_, lastInjT_, lastAir_, lastAfr_;
    double airEstimate(const SensorReadings& s, double rpm, double tpsPct);
    double time_ = 0, tauFilm_ = 0.07;
    // idle control
    double iacI_ = 0.35, dash_ = 0, sinceRelease_ = 10, prevRpm_ = 0, rpmRate_ = 0, afterStart_ = 0;
    bool limiterOn_ = false;
    double alsTimer_ = 10;
    // transients
    double prevTps_ = 0, tpsRate_ = 0, prevMap_ = 101.3, mapRate_ = 0, mapRateSlow_ = 0, aeNow_ = 0, aeTps_ = 0, sinceCut_ = 100;
    bool wasCut_ = false;
    bool dfcoArmed_ = true;      // decel cut allowed (re-armed by opening the throttle after a cut ended)
    double portDry_ = 0;         // 0..1: how dry the port walls are after decel fuel cut (drives the exit enrichment)
    bool fuelling_ = false;
    // Phase 3 state
    double sinceTransient_ = 0, capOn_ = 0;
    bool capLimiter_ = false;
    int knockSeqSeen_[kMaxCyl] = {};
    bool knockSeqInit_ = false;
    double boostI_ = 0, boostRate_ = 0, prevBoost_ = 0, overboostT_ = 0;
    double protTimer_[kProtCount] = {};
    bool protActive_[kProtCount] = {};
    double stoppedFor_ = 0, ltftSeqT_ = 0;
    bool dbwIdentity_ = true;
    // protection actions currently in force
    bool actRetard_ = false, actBoost_ = false, actThrottle_ = false, actFuel_ = false;
    // Phase 5 state: what the launch / shift limiters, traction control and nitrous ask of the spark and fuel
    double msCut_ = 0, msRetard_ = 0, msEnrich_ = 0;
    bool msTimingOn_ = false;
    double msTiming_ = 0;
    bool prevShift_ = false, flatLatched_ = false, rollLatched_ = false;
    double shiftCutT_ = 0, flatRpm_ = 0, rollRpm_ = 0, tcI_ = 0;
    double n2oOnT_ = 0, n2oLeanT_ = 0;
    uint32_t rng_ = 0x9e3779b9u;
    double rand01() { rng_ = rng_ * 1664525u + 1013904223u; return (rng_ >> 8) * (1.0 / 16777216.0); }
    // auto-tune history (what was commanded, matched to the wideband after the transport delay)
    struct Hist { double t, rpm, map, tps, w, lam, trim; bool ok; };
    static constexpr int kHist = 1024;
    Hist hist_[kHist] = {};
    int histW_ = 0;
};

} // namespace es

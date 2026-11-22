#pragma once
#include "sim/Dyno.h"
#include "sim/Engine.h"
#include "sim/Vehicle.h"
#include "audio/Listener.h"
#include "audio/Convolver.h"
#include "core/SpscRing.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace es {

struct Telemetry {
    float rpm = 0, crankDeg = 0, torque = 0, powerKw = 0;
    float mapKPa = 101, afr = 14.7f, targetAfr = 14.7f, ve = 0;
    float airFlow = 0, fuelFlow = 0, injDuty = 0, advance = 0, iac = 0;
    float knock = 0, egt = 0, egtGas = 0, dynoLoad = 0, dynoTarget = 0, dynoProgress = 0;
    // diesel (Package 4)
    float dieselFuelMg = 0, soi = 0, ignDelayDeg = 0; bool smokeLimited = false;
    float pilotMg = 0, pilotSoi = 0, mainDelayDeg = 0;   // pilot injection; the main injection's own delay after it
    float pilot2Mg = 0, postMg = 0, postSoi = 0;            // second pilot, post injection (common rail)
    float railBar = 0, railTargetBar = 0, pumpKw = 0;      // common rail pressure / target, its pump's drive power
    float noxPpm = 0, noxGs = 0, sootMgM3 = 0, smokeFsn = 0;   // diesel emissions (engine-out)
    float egrGasC = 0;                                     // EGR gas into the intake (after the cooler)
    float glowTipC = 0; bool glowOn = false, glowLight = false;
    float tsDelivery = 0, tsTrapping = 0, tsScavenging = 0;   // two-stroke, cylinder 1, last cycle: delivery ratio, TE, SE
    float tsCrankcaseKPa = 0, tsReed = 0;                     // ... crankcase pressure, inlet opening 0..1 (reed lift / port)
    float tsPowerValve = 1, tsOilSmokeGh = 0;
    float exhValve = -1;
    int turboUnits = 1, seqStage = 0;                          // twin turbos; sequential: 0 first alone, 1 pre-spool, 2 both
    float shaftRpm2 = 0, speedFrac2 = 0;                       // the second turbo                                       // valved muffler's flap 0 shut..1 open (-1: no valve)
    float bovLift = 0;                                         // blow-off valve lift 0..1                 // ... power valve 0 lowered..1 lifted, unburned oil out (g/h)
    float tsSquishMs = 0, tsSquishKeMJ = 0, tsBurnDeg = 0;    // ... max squish velocity, squished kinetic energy, burn period
    float egrPos = 0, egrGs = 0, egrRate = 0, vgtOpen = 0;   // EGR valve 0..1, flow g/s, share of the charge; VGT vane opening
    int dynoState = 0, dynoRun = 0;
    int misfires = 0, afterfires = 0;
    bool running = false, limiter = false;
    // hardware components
    float stress = 0, floatIn = 0, floatEx = 0, sparkMargin = 9, sparkAvail = 0, oilBar = 0, plugTempC = 0, fouling = 0, damage = 0;
    int failure = 0;
    // thermal / electrical / fuel supply
    float coolantC = 88, oilC = 85, iatC = 25, battery = 12.6f, fuelBar = 3, thermostat = 0, radiatorKw = 0;
    bool fan = false;
    // ECU
    float ecuTrim = 1, ecuWarmup = 0, ecuDeadMs = 0, ecuFlowComp = 1, ecuMap = 101;
    bool closedLoop = false, syncLost = false, batchFire = false;
    // sensors: truth (diagnostics only), what the ECU reads, pin voltage, status
    float snsTruth[kSensorCount] = {}, snsValue[kSensorCount] = {}, snsVolts[kSensorCount] = {};
    int snsStatus[kSensorCount] = {};
    bool snsFallback[kSensorCount] = {};
    bool paused = false;
    float simLoad = 0, ringFill = 0, peak = 0;
    int cylinders = 0;
    // forced induction
    float boostBar = 0, shaftRpm = 0, speedFrac = 0, pr = 1, compFlow = 0, wastegate = 0, chargeT = 25, fiPowerKw = 0;
    bool surging = false, bovOpen = false, antiLag = false;
    // Phase 6: map operating point, intercooler core, ambient, valvetrain dynamics
    float compEff = 0, compFlowCorr = 0, turbEff = 0, turbEr = 1, turbUc = 0, turbMfp = 0, surgeMargin = 1, icCoreC = 25;
    bool mapComp = false, mapTurb = false;
    float baroKPa = 101.3f, coolAirKmh = 0;
    // Phase 7: trigger / injection / dwell, torque management, gear control
    int injMode = 0;
    bool crankSync = true, camSync = true;
    float dwellMs = 0, syncDeg = 0, tqLoad = 0, tqLimit = 0, tqReduction = 0, tqThrCap = 1, gearLimitRpm = 0;
    float loftIn = 0, loftEx = 0, bounceIn = 0, bounceEx = 0, lateCloseIn = 0, lateCloseEx = 0;   // mm, deg
    // vehicle
    // fuel (Package 1): what burns now (fuel line), what is in the tank, and what the ECU believes
    float lambda = 1, fuelStoich = 14.7f, fuelAlcohol = 0, fuelRon = 95, fuelMon = 85, fuelLhv = 43.5f;
    float tankAlcohol = 0, tankRon = 95, tankLitres = 30, tankCap = 50;
    float fuelSupply = 1;   // 0..1 liquid fuel still reaching the engine (Engine::fuelSupply)
    float ecuFlexPct = 0, ecuStoich = 14.7f, ecuTargetLambda = 1, ecuFlexCold = 0;
    bool ecuFlexLive = false;
    // exhaust sensors (Package 2): gas at the wideband (before its delay), EMAP, drive pressure ratio, EGT after the
    // turbine and per cylinder (thermocouple readings, C), the sensor positions
    float lambdaPipe = 1, emapKPa = 101, driveRatio = 1, egtPostC = 0, egtCylC[kMaxCyl] = {}, turbineDropK = 0;
    bool egtPerCyl = false, widebandPost = false, emapFitted = false;
    // flow map (Package 3): forward / reverse flow per section of the gas path, pressures and temperatures
    FlowState flow;
    bool vehicle = false;
    int gear = 0;
    float speedKmh = 0, clutchEng = 0, clutchTq = 0, wheelSlip = 0, distance = 0, t100 = 0, tQuarter = 0, vQuarter = 0, runTime = 0;
    bool clutchLocked = false, shifting = false;
    // ECU VE table snapshot (rpm x MAP)
    // ECU calibration as the ECU holds it (auto-tune learns into the VE maps) + operating point
    Table2D calVeMap{}, calVeTps{};
    int learnSeq = 0;
    // Phase 3: trims, knock control, boost control, idle / throttle, protection
    Table2D calLtft{};
    int ltftSeq = -1;
    float ecuStft = 0, ecuLtft = 0;
    float knockRetard[kMaxCyl] = {};
    int knockEvents = 0;
    float boostTargetBar = 0, ecuBoostBar = 0, wgDuty = -1;
    bool overboostCut = false, boostCut = false;
    float idleTarget = 0, throttleTarget = -1, cutFraction = 0, wastegatePos = 0;
    int protState[kProtCount] = {};
    float protValue[kProtCount] = {};
    bool limp = false, shutdown = false, throttleLimited = false;
    // Phase 5: launch / shift limiters, traction control, nitrous, transbrake
    int launchState = 0;
    float launchLimitRpm = 0;
    float tcSlip = 0, tcTarget = 0, tcReduction = 0, tcRetard = 0, tcCut = 0, wheelKmh = 0;
    float n2oDuty = 0, n2oFlowGs = 0, n2oBottleKg = 0;
    bool n2oLeanLock = false, transbrake = false;
    // cam control
    float vtecEngaged = 0, vtcAdvance = 0, vtcTarget = 0;
    bool vtecCmd = false;
    int misfireCause[5] = {};
    float ecuVe = 0, ecuLoadX = 0, ecuLoadY = 0, ecuTps = 0, ecuAccel = 0, ecuAirMg = 0;
    bool tuning = false;
    float throttle = 0;
    float cylImep[kMaxCyl] = {}, cylPmax[kMaxCyl] = {}, cylLambda[kMaxCyl] = {};
    static constexpr int kTrace = 1080;
    int traceLen = 720;              // cycle length in degrees (720 piston, 1080 rotary)
    float traceP[kTrace] = {}, traceV[kTrace] = {}, traceExh[kTrace] = {}, traceInt[kTrace] = {};
    static constexpr int kScope = 4096;
    float scope[kScope] = {};
    int scopePos = 0;
};

class Simulator {
public:
    static constexpr int kOutRate = 48000;

    explicit Simulator(const EngineParams& p);
    ~Simulator();

    void setParams(const EngineParams& p);           // any thread (UI)
    void render(float* stereo, int frames);           // offline or sim thread
    void startThread();
    void stopThread();
    size_t pullAudio(float* stereo, size_t frames);   // audio callback
    void getTelemetry(Telemetry& out);
    void takeDynoPoints(std::vector<DynoPoint>& out, int& runId);
    // data logging: sample every log channel at `rateHz` (sim time) while on; the UI collects the samples
    void setLogging(bool on, double rateHz);
    void takeLogSamples(std::vector<double>& times, std::vector<float>& values);   // values: channel-major per sample
    // custom exhaust impulse response (already resampled to kOutRate); used when irChoice = custom
    void setCustomIR(const std::vector<float>& ir);
    // simulation control (any thread)
    void setPaused(bool on) { paused_.store(on); }
    bool paused() const { return paused_.load(); }
    void requestReset() { resetReq_.fetch_add(1); }
    // tests / tools only (same thread as render): the running engine
    const Engine* engineForTests() const { return eng_.get(); }
    void requestClearProtection() { protClearReq_.fetch_add(1); }   // e.g. un-latch an engine shutdown
    int activeIR() const { return irActive_; }

private:
    void syncParams();
    void rebuild(bool crossfade);
    void threadMain();

    std::mutex paramMx_;
    EngineParams pending_;
    std::atomic<uint32_t> version_{1};
    uint32_t seen_ = 0;
    EngineParams cur_;
    double fsSim_ = 96000;
    int os_ = 2;

    std::unique_ptr<Engine> eng_, old_;
    double fade_ = 1, fadeStep_ = 0;
    Dyno dyno_;
    Vehicle vehicle_;
    int prevVehicle_ = 0;
    Listener listener_;
    Biquad dec_[5][2];
    // exhaust impulse-response convolution
    void updateIR();
    Convolver convL_, convR_;
    std::vector<float> dryL_, dryR_;
    int dryPos_ = 0;
    int irActive_ = -1;
    double wet_ = 0;
    std::mutex irMx_;
    std::vector<float> customIR_;
    uint32_t customVer_ = 0, customUsed_ = ~0u;

    SpscRing<float> ring_{1 << 15};
    std::atomic<bool> paused_{false};
    std::atomic<int> resetReq_{0};
    int resetSeen_ = 0;
    std::atomic<int> protClearReq_{0};
    int protClearSeen_ = 0;
    std::thread thread_;
    std::atomic<bool> run_{false};
    float lastL_ = 0, lastR_ = 0;

    std::mutex telMx_;
    Telemetry tel_;
    std::vector<DynoPoint> dynoPts_;
    std::atomic<bool> logOn_{false};
    std::atomic<double> logRate_{50.0};
    double simTime_ = 0, nextLog_ = 0;
    std::vector<double> logTimes_;
    std::vector<float> logVals_;
    float scopeTmp_[Telemetry::kScope] = {};
    int scopeW_ = 0;
    double loadAvg_ = 0;
};

} // namespace es

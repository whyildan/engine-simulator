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
    float knock = 0, egt = 0, dynoLoad = 0, dynoTarget = 0, dynoProgress = 0;
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
    // vehicle
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
    float scopeTmp_[Telemetry::kScope] = {};
    int scopeW_ = 0;
    double loadAvg_ = 0;
};

} // namespace es

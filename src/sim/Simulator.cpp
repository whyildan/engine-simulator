#include "sim/Simulator.h"
#include "sim/DataLog.h"
#include <chrono>
#include "audio/ImpulseResponses.h"

namespace es {

Simulator::Simulator(const EngineParams& p) {
    pending_ = p;
    cur_ = p;
    seen_ = version_.load();
    listener_.init(kOutRate);
    dryL_.assign(Convolver::kBlock, 0.f);
    dryR_.assign(Convolver::kBlock, 0.f);
    listener_.setRoom(p.environment, p.reverbMix);
    rebuild(false);
    updateIR();
}

Simulator::~Simulator() { stopThread(); }

void Simulator::setCustomIR(const std::vector<float>& ir) {
    std::lock_guard<std::mutex> lk(irMx_);
    customIR_ = ir;
    ++customVer_;
}

// choose / rebuild the impulse response when the selection (or what "auto" points at) changes
void Simulator::updateIR() {
    int want = cur_.irChoice;
    if (want == IrAuto) {
        bool bike = cur_.cylinders <= 2 && !isRotary(cur_) && equivalentDisplacementL(cur_) < 2.0;
        if (bike) want = IrMotorcycle;
        else switch (cur_.muffler) {
            case Muffler::None: want = IrRawPipe; break;
            case Muffler::StraightThrough: want = IrRace; break;
            case Muffler::Chambered: want = IrSport; break;
            case Muffler::Turbo: default: want = IrStock; break;
        }
    }
    if (want == IrOff) { irActive_ = IrOff; return; }
    std::vector<float> h;
    if (want == IrCustom) {
        std::lock_guard<std::mutex> lk(irMx_);
        if (customIR_.empty()) { irActive_ = IrOff; return; }
        if (irActive_ == IrCustom && customUsed_ == customVer_) return;
        h = customIR_;
        customUsed_ = customVer_;
    } else {
        if (want == irActive_) return;
        h = makeBuiltinIR(want, kOutRate);
    }
    convL_.setIR(h);
    convR_.setIR(h);
    irActive_ = want;
    wet_ = 0;   // fade the new response in
}

void Simulator::setParams(const EngineParams& p) {
    std::lock_guard<std::mutex> lk(paramMx_);
    pending_ = p;
    version_.fetch_add(1, std::memory_order_release);
}

void Simulator::rebuild(bool crossfade) {
    double omega = eng_ ? eng_->omega() : 0;
    double theta = eng_ ? eng_->theta() : 0;
    int os = clampv(cur_.oversample, 1, 4);
    bool rateChange = os != os_ || !eng_;
    os_ = os;
    fsSim_ = (double)kOutRate * os_;
    if (crossfade && !rateChange && eng_) {
        old_ = std::move(eng_);
        fade_ = 0;
        fadeStep_ = 1.0 / (0.06 * fsSim_);
    } else {
        old_.reset();
        fade_ = 1;
    }
    std::unique_ptr<Engine> fresh = std::make_unique<Engine>(cur_, fsSim_);
    // keep the engine warm across a rebuild (structural change), not across a reset
    if (old_) fresh->copyThermalFrom(*old_);
    else if (eng_) fresh->copyThermalFrom(*eng_);
    eng_ = std::move(fresh);
    eng_->setCrank(omega, theta);
    if (rateChange)
        for (auto& d : dec_) for (Biquad& b : d) { b.lowpass(0.42 * kOutRate, 0.707, fsSim_); b.reset(); }
}

void Simulator::syncParams() {
    uint32_t v = version_.load(std::memory_order_acquire);
    if (v == seen_) return;
    seen_ = v;
    EngineParams np;
    { std::lock_guard<std::mutex> lk(paramMx_); np = pending_; }
    np.cylinders = clampv(np.cylinders, 1, kMaxCyl);
    bool structural = StructuralKey::of(np) != StructuralKey::of(cur_);
    cur_ = np;
    if (structural) rebuild(true);
    else eng_->apply(cur_);
    listener_.set(cur_.listener, cur_.volume);
    listener_.setRoom(cur_.environment, cur_.reverbMix);
    updateIR();
}

void Simulator::render(float* out, int frames) {
    auto t0 = std::chrono::high_resolution_clock::now();
    syncParams();
    // reset: a fresh engine at rest (cold or warm per the start setting), car stopped, dyno idle
    if (resetReq_.load() != resetSeen_) {
        resetSeen_ = resetReq_.load();
        old_.reset();
        eng_.reset();
        rebuild(false);
        vehicle_.reset();
        { int rc = dyno_.runCounter; dyno_ = Dyno(); dyno_.runCounter = rc; }
    }
    if (protClearReq_.load() != protClearSeen_) {
        protClearSeen_ = protClearReq_.load();
        if (eng_) eng_->ecu.clearProtection();
    }
    if (paused_.load()) {
        for (int f = 0; f < frames * 2; ++f) out[f] = 0.f;
        std::lock_guard<std::mutex> lk(telMx_);
        tel_.paused = true;
        return;
    }
    const double dt = 1.0 / fsSim_;
    for (int f = 0; f < frames; ++f) {
        double s[5] = {0, 0, 0, 0, 0};
        for (int k = 0; k < os_; ++k) {
            double pedal = cur_.throttle;
            const bool driving = cur_.vehicle != 0;
            if (driving && !prevVehicle_) vehicle_.reset();
            prevVehicle_ = driving;
            double load = driving ? vehicle_.step(dt, eng_->omega(), cur_, pedal) : dyno_.step(dt, cur_, *eng_, pedal);
            eng_->setVehicleState(driving ? vehicle_.v : 0.0, driving ? vehicle_.gear : 0,
                                  driving ? vehicle_.wheelOmega * std::max(0.15, cur_.tireRadius_mm * 1e-3) : 0.0,
                                  driving && vehicle_.shifting);
            eng_->step(dt, pedal, load, driving);
            double v[5] = {eng_->sExhL, eng_->sExhR, eng_->sIntake, eng_->sMech, eng_->sBoost + (driving ? vehicle_.sound() : 0.0)};
            if (old_) {
                double po = cur_.throttle;
                old_->step(dt, po, load, driving);
                double w = fade_;
                v[0] = v[0] * w + old_->sExhL * (1 - w);
                v[1] = v[1] * w + old_->sExhR * (1 - w);
                v[2] = v[2] * w + old_->sIntake * (1 - w);
                v[3] = v[3] * w + old_->sMech * (1 - w);
                v[4] = v[4] * w + old_->sBoost * (1 - w);
                fade_ += fadeStep_;
                if (fade_ >= 1) { fade_ = 1; old_.reset(); }
            }
            for (int c = 0; c < 5; ++c) {
                double x = v[c];
                if (os_ > 1) x = dec_[c][1].tick(dec_[c][0].tick(x));
                s[c] = x;
            }
        }
        // exhaust impulse response: dry path delayed by the convolver latency (no comb filtering)
        {
            if (cur_.irChoice == IrCustom && customUsed_ != customVer_) updateIR();
            double target = irActive_ > 0 && cur_.soundStyle != 1 ? clampv((double)cur_.irMix, 0.0, 1.0) : 0.0;
            wet_ += (target - wet_) * 0.0008;
            float dl = dryL_[dryPos_], dr = dryR_[dryPos_];
            dryL_[dryPos_] = (float)s[0]; dryR_[dryPos_] = (float)s[1];
            dryPos_ = (dryPos_ + 1) % Convolver::kBlock;
            float wl = irActive_ > 0 ? convL_.process((float)s[0]) : dl;
            float wr = irActive_ > 0 ? convR_.process((float)s[1]) : dr;
            s[0] = dl * (1.0 - wet_) + wl * wet_;
            s[1] = dr * (1.0 - wet_) + wr * wet_;
        }
        float L, R;
        listener_.process(s[0], s[1], s[2], s[3], s[4], L, R);
        out[2 * f] = L;
        out[2 * f + 1] = R;
        scopeTmp_[scopeW_] = 0.5f * (L + R);
        scopeW_ = (scopeW_ + 1) % Telemetry::kScope;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double used = std::chrono::duration<double>(t1 - t0).count();
    double avail = (double)frames / kOutRate;
    loadAvg_ += (used / avail - loadAvg_) * 0.05;

    // ---- telemetry
    std::lock_guard<std::mutex> lk(telMx_);
    const Engine& e = *eng_;
    Telemetry& t = tel_;
    t.rpm = (float)e.rpm();
    t.crankDeg = (float)e.theta();
    t.torque = (float)e.netTorque();
    t.powerKw = (float)(e.netTorque() * e.omega() / 1000.0);
    t.mapKPa = (float)(e.intake.map() / 1000.0);
    t.afr = (float)(e.lambdaAvg * e.fuel().stoich);
    t.targetAfr = (float)e.ecu.targetAfr;
    t.lambda = (float)e.lambdaAvg;
    t.fuelStoich = (float)e.fuel().stoich; t.fuelAlcohol = (float)e.fuel().alcoholPct;
    t.fuelRon = (float)e.fuel().ron; t.fuelMon = (float)e.fuel().mon; t.fuelLhv = (float)(e.fuel().lhv * 1e-6);
    t.tankAlcohol = (float)e.tankFuel().alcoholPct; t.tankRon = (float)e.tankFuel().ron;
    t.tankLitres = (float)e.tankLitres(); t.tankCap = (float)Engine::kTankCapL;
    t.ecuFlexPct = (float)e.ecu.flexPct; t.ecuStoich = (float)e.ecu.stoichEcu; t.ecuTargetLambda = (float)e.ecu.targetLambda;
    t.ecuFlexCold = (float)e.ecu.flexCold; t.ecuFlexLive = e.ecu.flexLive;
    t.lambdaPipe = (float)e.lambdaPipe(); t.emapKPa = (float)e.emapKPa();
    t.driveRatio = (float)(e.sensors.r.value[kSnsEMAP] / std::max(10.0, e.ecu.mapUsed));   // as the ECU sees it
    t.egtPostC = (float)(e.egtPostProbe() - 273.15); t.turbineDropK = (float)e.boost.turbineDropK;
    for (int i = 0; i < kMaxCyl; ++i) t.egtCylC[i] = (float)(e.egtCylProbe(i) - 273.15);
    t.flow = e.flowState();
    t.egtPerCyl = e.params().egtPerCyl != 0; t.emapFitted = e.params().emapSensor != 0;
    t.widebandPost = e.params().widebandPos != 0 && e.exhaust.hasTurbine();
    t.ve = (float)e.veAvg;
    t.airFlow = (float)(e.intake.airFlow() * 1000.0);
    t.fuelFlow = (float)(e.intake.fuelFlow() * 1000.0);
    t.injDuty = (float)e.ecu.injDuty;
    t.advance = (float)e.advance();
    t.iac = (float)(e.ecu.cmd.iac * 100.0);
    t.paused = false;
    t.coolantC = (float)e.thermal.coolantC; t.oilC = (float)e.thermal.oilC; t.iatC = (float)e.thermal.iatC;
    t.battery = (float)e.elec.volts; t.fuelBar = (float)e.rail.pressureBar; t.thermostat = (float)e.thermal.thermostat;
    t.radiatorKw = (float)(e.thermal.radiatorW * 1e-3); t.fan = e.thermal.fan;
    t.ecuTrim = (float)e.ecu.trim; t.ecuWarmup = (float)e.ecu.warmup; t.ecuDeadMs = (float)e.ecu.deadComp_ms;
    t.ecuFlowComp = (float)e.ecu.flowComp; t.ecuMap = (float)e.ecu.mapUsed; t.closedLoop = e.ecu.closedLoop;
    t.syncLost = e.ecu.cmd.syncLost; t.batchFire = e.ecu.cmd.batchFire;
    for (int k = 0; k < kSensorCount; ++k) {
        t.snsTruth[k] = (float)e.sensors.truth.v[k]; t.snsValue[k] = (float)e.sensors.r.value[k];
        t.snsVolts[k] = (float)e.sensors.r.volts[k]; t.snsStatus[k] = e.sensors.r.status[k]; t.snsFallback[k] = e.sensors.r.fallback[k];
    }
    t.knock = (float)e.knockMax;
    t.dieselFuelMg = (float)e.ecu.cmd.dieselFuelMg; t.soi = (float)e.ecu.cmd.soi; t.smokeLimited = e.ecu.smokeLimited;
    t.ignDelayDeg = e.cylinders() > 0 ? (float)e.cyl(0).idDeg : 0.0f;
    t.pilotMg = (float)e.ecu.cmd.pilotMg; t.pilotSoi = (float)e.ecu.cmd.pilotSoi;
    t.mainDelayDeg = e.cylinders() > 0 && e.cyl(0).main2 ? (float)e.cyl(0).idDegMain : 0.0f;
    t.glowTipC = (float)e.glowTipC(); t.glowOn = e.ecu.cmd.glow; t.glowLight = e.ecu.glowLight;
    t.egrPos = (float)e.egrPos(); t.egrGs = (float)(e.egrFlow() * 1000.0);
    t.egrRate = (float)e.egrRate();
    t.vgtOpen = e.params().vgt ? (float)e.boost.wastegate : 1.0f;
    t.stress = (float)e.stress; t.floatIn = (float)e.floatIn; t.floatEx = (float)e.floatEx;
    t.sparkMargin = (float)e.sparkMargin; t.sparkAvail = (float)e.sparkAvail; t.oilBar = (float)e.oilBar;
    t.plugTempC = (float)e.plugTempC; t.fouling = (float)e.fouling; t.damage = (float)e.damage; t.failure = e.failure;
    t.egt = (float)e.egtProbe() - 273.15f;        // what the EGT gauge shows (thermocouple in the header)
    t.egtGas = (float)e.egtAvg - 273.15f;
    t.dynoLoad = (float)dyno_.load;
    t.dynoTarget = (float)dyno_.target;
    t.dynoProgress = (float)dyno_.progress;
    t.dynoState = (int)dyno_.state;
    t.dynoRun = dyno_.runCounter;
    t.misfires = e.misfires;
    t.afterfires = e.afterfires;
    t.running = e.running();
    t.limiter = e.limiterActive();
    t.simLoad = (float)loadAvg_;
    t.ringFill = (float)ring_.size() / 2.0f / kOutRate;
    t.peak = (float)listener_.peak();
    t.cylinders = e.cylinders();
    t.boostBar = (float)e.boost.boostBar; t.shaftRpm = (float)e.boost.shaftRpm; t.speedFrac = (float)e.boost.speedFrac;
    t.pr = (float)e.boost.pr; t.compFlow = (float)(e.boost.compFlow * 1000.0); t.wastegate = (float)e.boost.wastegate;
    t.compEff = (float)e.boost.compEff; t.compFlowCorr = (float)(e.boost.compFlowCorr * 1000.0); t.turbEff = (float)e.boost.turbEff;
    t.turbEr = (float)e.boost.turbEr; t.turbUc = (float)e.boost.turbUc; t.turbMfp = (float)e.boost.turbMfp;
    t.surgeMargin = (float)e.boost.surgeMargin; t.icCoreC = (float)(e.boost.icCoreT - 273.15);
    t.injMode = e.ecu.cmd.injMode; t.crankSync = e.crankSynced(); t.camSync = e.camSynced();
    t.dwellMs = (float)e.ecu.cmd.dwellMs; t.syncDeg = (float)e.lastSyncDeg;
    t.tqLoad = (float)e.ecu.tqLoad; t.tqLimit = (float)e.ecu.tqLimit; t.tqReduction = (float)e.ecu.tqReduction;
    t.tqThrCap = (float)e.ecu.tqThrCap; t.gearLimitRpm = (float)e.ecu.gearLimitRpm;
    t.mapComp = e.boost.mapCompressor(); t.mapTurb = e.boost.mapTurbine();
    t.baroKPa = (float)(ambientPressurePa(e.params()) * 1e-3); t.coolAirKmh = (float)(e.coolingAirMs() * 3.6);
    {
        const double b = e.params().vtec ? e.vtecEngaged() : 0.0;
        auto mix = [b](double lo, double hi) { return (float)((1.0 - b) * lo + b * hi); };
        const ValveDynamics &il = e.valveDyn(true), &ih = e.valveDyn(true, true), &xl = e.valveDyn(false), &xh = e.valveDyn(false, true);
        t.loftIn = mix(il.loft, ih.loft) * 1000.0f; t.loftEx = mix(xl.loft, xh.loft) * 1000.0f;
        t.bounceIn = mix(il.bounce, ih.bounce) * 1000.0f; t.bounceEx = mix(xl.bounce, xh.bounce) * 1000.0f;
        t.lateCloseIn = mix(il.lateClose, ih.lateClose); t.lateCloseEx = mix(xl.lateClose, xh.lateClose);
    }
    t.chargeT = (float)(e.boost.chargeT - 273.15); t.fiPowerKw = (float)e.boost.powerKw;
    if (t.ltftSeq != e.ecu.ltftSeq || t.calLtft.nx == 0) { t.calLtft = e.ecu.cal().ltft; t.ltftSeq = e.ecu.ltftSeq; }
    t.ecuStft = (float)e.ecu.stft; t.ecuLtft = (float)e.ecu.ltftNow;
    for (int i = 0; i < kMaxCyl; ++i) t.knockRetard[i] = (float)e.ecu.cmd.cylRetard[i];
    t.knockEvents = e.ecu.knockEvents;
    t.boostTargetBar = (float)e.ecu.boostTargetBar; t.ecuBoostBar = (float)e.ecu.boostBar; t.wgDuty = (float)e.ecu.cmd.wgDuty;
    t.overboostCut = e.ecu.overboostCut; t.boostCut = e.ecu.boostCut;
    t.idleTarget = (float)e.ecu.idleTargetNow; t.throttleTarget = (float)e.ecu.cmd.throttleTarget; t.cutFraction = (float)e.ecu.cmd.cutFraction;
    t.wastegatePos = (float)e.boost.wastegate;
    for (int i = 0; i < kProtCount; ++i) { t.protState[i] = e.ecu.protState[i]; t.protValue[i] = (float)e.ecu.protValue[i]; }
    t.limp = e.ecu.limp; t.shutdown = e.ecu.shutdown; t.throttleLimited = e.ecu.throttleLimited;
    if (t.learnSeq != e.ecu.learnSeq || t.calVeMap.nx == 0) { t.calVeMap = e.ecu.cal().veMap; t.calVeTps = e.ecu.cal().veTps; t.learnSeq = e.ecu.learnSeq; }
    t.ecuVe = (float)e.ecu.veUsed; t.ecuLoadX = (float)e.ecu.loadX; t.ecuLoadY = (float)e.ecu.loadY; t.ecuTps = (float)e.ecu.tpsUsed;
    t.ecuAccel = (float)e.ecu.accel; t.ecuAirMg = (float)(e.ecu.airPerCyl * 1000.0); t.tuning = e.ecu.tuning;
    t.throttle = (float)e.intake.throttlePosition();
    t.vehicle = cur_.vehicle != 0;
    t.gear = vehicle_.gear; t.speedKmh = (float)(vehicle_.v * 3.6); t.clutchEng = (float)vehicle_.engagement;
    t.clutchTq = (float)vehicle_.clutchTorque; t.wheelSlip = (float)vehicle_.slip; t.distance = (float)vehicle_.distance;
    t.t100 = (float)vehicle_.t100; t.tQuarter = (float)vehicle_.tQuarter; t.vQuarter = (float)vehicle_.vQuarter;
    t.runTime = (float)vehicle_.runTime; t.clutchLocked = vehicle_.clutchLocked; t.shifting = vehicle_.shifting;
    if (t.vehicle) { t.torque = (float)vehicle_.clutchTorqueAvg; t.powerKw = (float)(vehicle_.clutchTorqueAvg * e.omega() / 1000.0); }
    t.surging = e.boost.surging; t.bovOpen = e.boost.bovOpen; t.antiLag = e.antiLagActive;
    t.launchState = e.ecu.launchState; t.launchLimitRpm = (float)e.ecu.launchLimitRpm;
    t.tcSlip = (float)e.ecu.tcSlip; t.tcTarget = (float)e.ecu.tcTarget; t.tcReduction = (float)e.ecu.tcReduction;
    t.tcRetard = (float)e.ecu.tcRetard; t.tcCut = (float)e.ecu.tcCut;
    t.wheelKmh = (float)(t.vehicle ? vehicle_.wheelOmega * std::max(0.15, cur_.tireRadius_mm * 1e-3) * 3.6 : 0.0);
    t.n2oDuty = (float)e.ecu.cmd.n2oDuty; t.n2oFlowGs = (float)(e.n2oFlow * 1000.0); t.n2oBottleKg = (float)e.n2oBottle;
    t.n2oLeanLock = e.ecu.n2oLeanLock; t.transbrake = t.vehicle && vehicle_.transbrake;
    t.vtecEngaged = (float)e.vtecEngaged(); t.vtcAdvance = (float)e.vtcAdvance(); t.vtcTarget = (float)e.ecu.cmd.vtcAdvance;
    t.vtecCmd = e.ecu.cmd.vtec;
    for (int k = 0; k < Engine::kMisCauseCount && k < 5; ++k) t.misfireCause[k] = e.misfireCause[k];
    for (int i = 0; i < e.cylinders(); ++i) {
        t.cylImep[i] = (float)(e.cyl(i).imep * 1e-5);
        t.cylPmax[i] = (float)(e.cyl(i).pmaxCycle * 1e-5);
        t.cylLambda[i] = (float)e.cyl(i).lambda;
    }
    t.traceLen = (int)e.cycle();
    std::memcpy(t.traceP, e.traceP, sizeof(t.traceP));
    std::memcpy(t.traceV, e.traceV, sizeof(t.traceV));
    std::memcpy(t.traceExh, e.traceExh, sizeof(t.traceExh));
    std::memcpy(t.traceInt, e.traceInt, sizeof(t.traceInt));
    std::memcpy(t.scope, scopeTmp_, sizeof(t.scope));
    t.scopePos = scopeW_;
    for (const DynoPoint& d : dyno_.newPoints) dynoPts_.push_back(d);
    dyno_.newPoints.clear();
    // data logging (sim time, so the samples are evenly spaced whatever the UI frame rate)
    simTime_ += (double)frames / kOutRate;
    if (logOn_.load()) {
        if (simTime_ >= nextLog_) {
            const size_t n = (size_t)logChannelCount();
            if (logTimes_.size() < 200000) {          // ~30 min at 100 Hz between UI pulls is plenty
                logTimes_.push_back(simTime_);
                logVals_.resize(logVals_.size() + n);
                sampleChannels(t, logVals_.data() + logVals_.size() - n);
            }
            const double per = 1.0 / std::max(1.0, logRate_.load());
            nextLog_ = std::max(nextLog_ + per, simTime_ - per);
        }
    } else nextLog_ = simTime_;
}

void Simulator::setLogging(bool on, double rateHz) {
    logRate_.store(std::max(1.0, rateHz));
    logOn_.store(on);
}

void Simulator::takeLogSamples(std::vector<double>& times, std::vector<float>& values) {
    std::lock_guard<std::mutex> lk(telMx_);
    times.insert(times.end(), logTimes_.begin(), logTimes_.end());
    values.insert(values.end(), logVals_.begin(), logVals_.end());
    logTimes_.clear(); logVals_.clear();
}

void Simulator::getTelemetry(Telemetry& out) {
    std::lock_guard<std::mutex> lk(telMx_);
    out = tel_;
}

void Simulator::takeDynoPoints(std::vector<DynoPoint>& out, int& runId) {
    std::lock_guard<std::mutex> lk(telMx_);
    out.insert(out.end(), dynoPts_.begin(), dynoPts_.end());
    dynoPts_.clear();
    runId = dyno_.runCounter;
}

void Simulator::threadMain() {
    const int block = 256;
    std::vector<float> buf(block * 2);
    const size_t targetFrames = 3072;
    while (run_.load()) {
        if (ring_.size() / 2 < targetFrames) {
            render(buf.data(), block);
            size_t pushed = 0;
            while (pushed < buf.size() && run_.load()) {
                pushed += ring_.push(buf.data() + pushed, buf.size() - pushed);
                if (pushed < buf.size()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Simulator::startThread() {
    if (run_.load()) return;
    run_ = true;
    thread_ = std::thread([this] { threadMain(); });
}

void Simulator::stopThread() {
    if (!run_.load()) return;
    run_ = false;
    if (thread_.joinable()) thread_.join();
}

size_t Simulator::pullAudio(float* out, size_t frames) {
    size_t got = ring_.pop(out, frames * 2) / 2;
    if (got > 0) { lastL_ = out[2 * got - 2]; lastR_ = out[2 * got - 1]; }
    for (size_t i = got; i < frames; ++i) { // underrun: decay instead of clicking
        lastL_ *= 0.995f; lastR_ *= 0.995f;
        out[2 * i] = lastL_; out[2 * i + 1] = lastR_;
    }
    return got;
}

} // namespace es

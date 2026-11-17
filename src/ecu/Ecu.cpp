#include "ecu/Ecu.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace es {

static constexpr double kFuelDensity = 0.745;   // g/cc
static constexpr double kRgas = 287.0;

void Ecu::init(const EngineParams& p, double vdCyl, int ports, double revsPerEvent) {
    p_ = p;
    cal_ = lastIn_ = p.cal;
    vdCyl_ = vdCyl; ports_ = ports; revsPerEvent_ = revsPerEvent;
    filmEst_.assign(ports, 0.0);
    evapAcc_.assign(ports, 0.0);
    lastInjT_.assign(ports, -1.0); lastAir_.assign(ports, 0.0); lastAfr_.assign(ports, 14.7);
    asyncFuel.assign(ports, 0.0);
    iacI_ = 0.35; driveFf_ = 0; rpmIdle_ = 0; cmd = ActuatorCommands();
}

// take the new calibration - except VE tables the tuner did not touch (they may hold auto-tune results)
void Ecu::apply(const EngineParams& p) {
    p_ = p;
    Table2D keepVe = cal_.veMap, keepTps = cal_.veTps, keepLtft = cal_.ltft;
    bool veSame = std::memcmp(&p.cal.veMap, &lastIn_.veMap, sizeof(Table2D)) == 0;
    bool tpsSame = std::memcmp(&p.cal.veTps, &lastIn_.veTps, sizeof(Table2D)) == 0;
    bool ltftSame = std::memcmp(&p.cal.ltft, &lastIn_.ltft, sizeof(Table2D)) == 0;
    cal_ = p.cal;
    if (veSame) { cal_.veMap = keepVe; cal_.veMap.locked = p.cal.veMap.locked; }
    if (tpsSame) { cal_.veTps = keepTps; cal_.veTps.locked = p.cal.veTps.locked; }
    if (ltftSame) { cal_.ltft = keepLtft; cal_.ltft.locked = p.cal.ltft.locked; }
    lastIn_ = p.cal;
    dbwIdentity_ = true;
    for (int i = 0; i < cal_.dbwMap.n && i < kCurveMax; ++i)
        if (std::fabs(cal_.dbwMap.v[i] - cal_.dbwMap.x[i]) > 0.01f) dbwIdentity_ = false;
}

void Ecu::clearProtection() {
    shutdown = false;
    stoppedFor_ = 0;
    for (int i = 0; i < kProtCount; ++i) { protActive_[i] = false; protTimer_[i] = 0; }
}

// ---- engine protection: every rule watches one sensor-derived value; past its limit for `delay` seconds the
//      rule's action engages, and it clears once the value is back past limit -/+ recover
void Ecu::protectionStep(double dt, const SensorReadings& s, double rpm, double tpsPct, bool running) {
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    const double load = std::max(mapUsed / 101.325, tpsPct * 0.01);
    double v[kProtCount];
    v[kProtOverRev] = rpm;
    v[kProtOverboost] = (mapUsed - baro()) / 100.0;
    v[kProtCoolant] = s.value[kSnsECT];
    v[kProtOilTemp] = s.value[kSnsOilT];
    v[kProtOilPress] = s.value[kSnsOilP];
    v[kProtFuelPress] = s.value[kSnsFuelP] - (mapUsed - baro()) / 100.0;     // pressure across the injectors
    v[kProtLean] = s.value[kSnsWideband];
    double kr = 0;
    for (int i = 0; i < ports_ && i < kMaxCyl; ++i) kr = std::max(kr, knockRetard[i]);
    v[kProtKnock] = kr;
    v[kProtEgt] = s.value[kSnsEGT];
    static const int critical[] = {kSnsTPS, kSnsMAP, kSnsIAT, kSnsECT, kSnsCKP, kSnsCMP, kSnsWideband, kSnsKnock, kSnsOilP, kSnsFuelP};
    int failed = 0;
    for (int id : critical) failed += s.fallback[id] ? 1 : 0;
    v[kProtSensor] = failed;

    bool aRet = false, aBoost = false, aThr = false, aFuel = false, aLimp = false;
    for (int i = 0; i < kProtCount; ++i) {
        const ProtRule& r = cal_.prot[i];
        const ProtInfo& in = protInfo(i);
        protValue[i] = v[i];
        bool applicable = r.enabled != 0;
        // readings that mean nothing in some states
        if (i == kProtOilPress || i == kProtFuelPress) applicable = applicable && running && afterStart_ > 2.0;
        if (i == kProtFuelPress) applicable = applicable && efi;
        if (i == kProtOverboost) applicable = applicable && isBoosted(p_);
        if (i == kProtLean) applicable = applicable && efi && running && fuelling_ && afterStart_ > 2.0 && load > 0.85 &&
                                         !s.fallback[kSnsWideband] && sinceTransient_ > 0.3;
        double limit = r.limit, warn = r.warn;
        if (i == kProtOilPress) { const double k = clampv(rpm / 3000.0, 0.25, 1.0); limit *= k; warn *= k; }   // needed pressure grows with rpm
        auto past = [&](double x, double lim) { return in.high ? x > lim : x < lim; };
        const bool over = applicable && past(v[i], limit);
        const bool warnOver = applicable && past(v[i], warn);
        const bool clear = !applicable || (in.high ? v[i] < limit - r.recover : v[i] > limit + r.recover);
        if (over) {
            protTimer_[i] += dt;
            if (protTimer_[i] >= r.delay) protActive_[i] = true;
        } else {
            protTimer_[i] = std::max(0.0, protTimer_[i] - dt);
            if (protActive_[i] && clear) protActive_[i] = false;
        }
        protState[i] = protActive_[i] ? 2 : warnOver ? 1 : 0;
        if (!protActive_[i]) continue;
        switch (r.action) {
        case kActRetard: aRet = true; break;
        case kActBoostReduce: aBoost = true; break;
        case kActThrottleLimit: aThr = true; break;
        case kActFuelLimit: aFuel = true; break;
        case kActLimp: aLimp = true; break;
        case kActShutdown: shutdown = true; break;   // latched
        default: break;
        }
    }
    actRetard_ = aRet; actBoost_ = aBoost; actThrottle_ = aThr; actFuel_ = aFuel; limp = aLimp;
    // a shutdown holds until the engine has stood still for a moment (the driver has to restart it)
    if (shutdown) {
        stoppedFor_ = rpm < 30.0 ? stoppedFor_ + dt : 0.0;
        if (stoppedFor_ > 2.0) clearProtection();
    }
}

// ---- knock control: the knock window attributes each combustion's knock to its cylinder; a knocking cylinder
//      is retarded a step, every cylinder creeps back to the map at the recovery rate
void Ecu::knockStep(double dt, const SensorReadings& s, double rpm) {
    const bool on = cal_.knockControl && rpm > cal_.knockMinRpm && !s.fallback[kSnsKnock];
    for (int i = 0; i < kMaxCyl; ++i) {
        bool event = false;
        if (s.knockSeq[i] != knockSeqSeen_[i]) {
            if (knockSeqInit_ && on && i < ports_ && s.knockCyl[i] > cal_.knockThreshold) event = true;
            knockSeqSeen_[i] = s.knockSeq[i];
        }
        if (event) { knockRetard[i] = std::min((double)cal_.knockMaxRetard, knockRetard[i] + cal_.knockRetardStep); ++knockEvents; }
        else knockRetard[i] = std::max(0.0, knockRetard[i] - cal_.knockRecover * dt);
        if (!cal_.knockControl || i >= ports_) knockRetard[i] = 0;
        // failed knock sensor: no knock detection -> a fixed safety retard instead
        cmd.cylRetard[i] = knockRetard[i];
        if (cal_.knockControl && s.fallback[kSnsKnock] && i < ports_) cmd.cylRetard[i] = std::max(knockRetard[i], 0.5 * cal_.knockMaxRetard);
    }
    knockSeqInit_ = true;
}

// ---- torque management: the ECU estimates the torque from the cylinder air charge (% of a full naturally
//      aspirated charge) and holds it under a limit per gear. With drive-by-wire a throttle cap integrates towards
//      the limit (clean); what is still above it right now is taken off with retard and then spark cuts.
void Ecu::torqueStep(double dt, const SensorReadings& s, double rpm, double tpsPct, bool dbw, bool running) {
    const double gear = s.fallback[kSnsGear] ? 0.0 : s.value[kSnsGear];
    const double full = 101325.0 * vdCyl_ / (kRgas * 298.0);
    tqLoad = full > 0 ? airEstimate(s, rpm, tpsPct) / full * 100.0 : 0.0;
    tqLimit = 0; tqReduction = 0; tqRetard_ = 0; tqCut_ = 0;
    if (!cal_.torqueMgmt || !running || gear < 0.5 || rpm < 1000.0) {
        tqThrCap = std::min(1.0, tqThrCap + dt * 2.0);
        return;
    }
    tqLimit = std::max(5.0, curveLookup(cal_.tqLimitGear, gear));
    const double over = tqLoad - tqLimit;
    if (dbw) tqThrCap = clampv(tqThrCap - over * std::max(0.0f, cal_.tqThrottleRate) * dt, 0.03, 1.0);
    else tqThrCap = 1.0;
    const double red = tqLoad > 1.0 ? clampv(over / tqLoad, 0.0, 0.95) : 0.0;
    tqReduction = red;
    // retard takes off the first ~15 % (a full retard costs about that much torque), dropped sparks the rest
    // (each dropped spark removes its cylinder's share)
    const double byRetard = std::min(red, 0.15);
    tqRetard_ = byRetard / 0.15 * std::max(0.0f, cal_.tqMaxRetard);
    tqCut_ = std::min(std::max(0.0, (red - byRetard) / (1.0 - byRetard)), clampv((double)cal_.tqMaxCut, 0.0, 1.0));
}

// ---- boost control: target from rpm x throttle (x gear), feed-forward from the wastegate spring + PID on the
//      MAP sensor -> solenoid duty. Overboost and boost cut work as fuel cuts.
void Ecu::boostStep(double dt, const SensorReadings& s, double rpm, double tpsPct) {
    boostBar = (mapUsed - baro()) / 100.0;
    const double rate = (boostBar - prevBoost_) / std::max(1e-4, dt);
    prevBoost_ = boostBar;
    boostRate_ += (rate - boostRate_) * std::min(1.0, dt / 0.03);
    const bool turbo = p_.induction == Induction::Turbo;
    const bool ctl = turbo && cal_.boostControl && !holdIac;
    double target = 0;
    if (ctl) target = std::max(0.0, tableLookup(cal_.boostTarget, rpm, tpsPct)) * curveLookup(cal_.boostGear, s.value[kSnsGear]) * 0.01;
    if (actBoost_ || limp) target = 0;   // back to the wastegate spring - the ECU cannot hold boost lower than that
    boostTargetBar = target;
    if (!ctl) { cmd.wgDuty = -1; boostI_ = 0; }
    else if (target <= 0.02 || s.fallback[kSnsMAP]) { cmd.wgDuty = 0; boostI_ = 0; }
    else {
        const double spring = std::max(0.05, (double)p_.wastegateBar);
        // the solenoid bleeds the actuator signal: it sees boost x (1 - 0.9 duty), so the spring holds spring / (1 - 0.9 duty)
        const double ff = target > spring ? (1.0 - spring / target) / 0.9 : 0.0;
        const double err = target - boostBar;
        if (boostBar > 0.5 * target) boostI_ = clampv(boostI_ + cal_.boostKi * err * dt, -0.4, 0.4);   // no wind-up while spooling
        cmd.wgDuty = clampv(ff + cal_.boostKp * err + boostI_ - cal_.boostKd * boostRate_, 0.0, clampv((double)cal_.wgDutyMax, 0.0, 1.0));
    }
    if (target > 0.02 && boostBar > target + cal_.overboostMargin) overboostT_ += dt; else overboostT_ = 0;
    if (overboostT_ > cal_.overboostTime) overboostCut = true;
    else if (overboostCut && boostBar < target) overboostCut = false;
    if (boostBar > cal_.boostCutBar) boostCut = true;
    else if (boostBar < cal_.boostCutBar - 0.15) boostCut = false;
    if (!isBoosted(p_) || holdIac) overboostCut = boostCut = false;
}

// ---- motorsport (Phase 5): launch control (2-step), the clutch-in limiters while moving (3-step / flat shift),
//      the shift cut, rolling anti-lag, traction control and the nitrous controller. Everything works through
//      the spark (cut share, retard), the fuel (cut share, enrichment) and the nitrous solenoid.
void Ecu::motorsportStep(double dt, const SensorReadings& s, double rpm, double tpsPct, bool running) {
    const DigitalInputs& in = s.in;
    const double vss = s.fallback[kSnsVSS] ? 0.0 : s.value[kSnsVSS];
    const bool standing = vss <= cal_.launchMaxSpeed;
    msCut_ = 0; msRetard_ = 0; msEnrich_ = 0; msTimingOn_ = false;
    cmd.fuelCutFraction = 0;
    launchState = kLaunchOff;
    launchLimitRpm = 0;

    // ---- launch / shift limiters (one at a time, in this order)
    if (in.shift && !prevShift_) shiftCutT_ = cal_.shiftCutTime;
    prevShift_ = in.shift;
    shiftCutT_ = std::max(0.0, shiftCutT_ - dt);
    const bool armedTps = tpsPct >= cal_.launchMinTps;
    // flat shift: the rpm where the clutch went in (at full throttle, moving) is held until it comes out
    if (cal_.clutchMovingMode == 2 && in.clutch && !standing && armedTps && running) {
        if (!flatLatched_) { flatLatched_ = true; flatRpm_ = rpm; }
    } else flatLatched_ = false;
    // rolling anti-lag: the rpm where the button went in is held, with anti-lag timing, to build boost on the move
    if (cal_.rollingAls && in.rollingAls && vss >= cal_.rollingAlsMinSpeed && tpsPct >= cal_.launchMinTps && running && !limp) {
        if (!rollLatched_) { rollLatched_ = true; rollRpm_ = rpm; }
    } else rollLatched_ = false;
    if (cal_.launchControl && (in.clutch || in.transbrake) && standing && armedTps) {
        launchState = kLaunch2Step; launchLimitRpm = cal_.launchRpm;
        msTimingOn_ = true; msTiming_ = cal_.launchTiming; msEnrich_ = cal_.launchEnrich;
    } else if (cal_.clutchMovingMode == 1 && in.clutch && !standing && armedTps) {
        launchState = kLaunch3Step; launchLimitRpm = cal_.threeStepRpm;
        msTimingOn_ = true; msTiming_ = cal_.launchTiming; msEnrich_ = cal_.launchEnrich;
    } else if (flatLatched_) {
        launchState = kLaunchFlatShift; launchLimitRpm = flatRpm_ + cal_.flatShiftMargin;
        msRetard_ = cal_.flatShiftRetard;
    } else if (cal_.shiftCut && shiftCutT_ > 0 && tpsPct >= cal_.shiftCutMinTps) {
        launchState = kLaunchShiftCut;
        msCut_ = 1.0;
    } else if (rollLatched_) {
        launchState = kLaunchRollingAls; launchLimitRpm = rollRpm_;
        msTimingOn_ = true; msTiming_ = cal_.alsTiming; msEnrich_ = cal_.launchEnrich;
    }
    if (launchLimitRpm > 0) {
        // the cut share ramps from 0 at limit - band to 100 % at limit + band: the revs settle on the limit
        const double band = std::max(30.0f, cal_.launchCutBand);
        const double f = clampv((rpm - (launchLimitRpm - band)) / (2.0 * band), 0.0, 1.0);
        if (cal_.launchCutMode == 1 && p_.fuelSystem == FuelSystem::EFI) cmd.fuelCutFraction = f;
        else msCut_ = f;
    }

    // ---- traction control: driven wheel speed against the vehicle speed
    tcSlip = in.wheelKmh > 3.0 ? (in.wheelKmh - vss) / std::max(vss, 10.0) * 100.0 : 0.0;
    tcTarget = curveLookup(cal_.tcSlipTarget, vss);
    const bool tcOn = cal_.tractionControl && running && !s.fallback[kSnsVSS];
    if (tcOn) {
        const double err = tcSlip - tcTarget;
        const double pTerm = cal_.tcKp * err;
        // anti-windup: no further integration while the P term alone already saturates the output, and the
        // integrator lets go quickly once the tyres have hooked up (a wound-up integrator bogs the engine)
        if (!(err > 0 && pTerm >= 1.0)) tcI_ += cal_.tcKi * err * (err < 0 ? 3.0 : 1.0) * dt;
        tcI_ = clampv(tcI_, 0.0, 1.0);
        tcReduction = clampv(pTerm + tcI_, 0.0, 1.0);
    } else { tcI_ = 0; tcReduction = 0; }
    // the first 40 % of the reduction is retard (smooth), the rest cuts sparks (strong; on a turbo engine the
    // retard alone even feeds the turbine)
    tcRetard = std::min(1.0, tcReduction / 0.4) * cal_.tcMaxRetard;
    tcCut = clampv((tcReduction - 0.4) / 0.6, 0.0, 1.0) * clampv((double)cal_.tcMaxCut, 0.0, 1.0);

    // ---- nitrous: armed, full throttle, inside the rpm window, warm, not limiting - then after a delay
    //      the solenoid opens progressively (start duty -> 100 % over the ramp time)
    const bool lean = !s.fallback[kSnsWideband] && s.value[kSnsWideband] > cal_.n2oLeanLambda;
    n2oLeanT_ = (lean && cmd.n2oDuty > 0 && n2oOnT_ > cal_.n2oDelay + 0.3) ? n2oLeanT_ + dt : 0.0;
    if (n2oLeanT_ > 0.2) n2oLeanLock = true;                  // lean while spraying: off until the throttle is lifted
    if (tpsPct < cal_.n2oMinTps) n2oLeanLock = false;
    const bool n2oOk = p_.nitrous && in.n2oArm && running && tpsPct >= cal_.n2oMinTps && rpm >= cal_.n2oMinRpm &&
                       rpm <= cal_.n2oMaxRpm && s.value[kSnsECT] >= cal_.n2oMinEct && vss >= cal_.n2oMinSpeed &&
                       launchState == kLaunchOff && !limiterOn_ && !limp && !shutdown && !n2oLeanLock;
    n2oOnT_ = n2oOk ? n2oOnT_ + dt : 0.0;
    double duty = 0;
    if (n2oOnT_ > cal_.n2oDelay) {
        const double u = cal_.n2oRampTime > 0.01f ? clampv((n2oOnT_ - cal_.n2oDelay) / cal_.n2oRampTime, 0.0, 1.0) : 1.0;
        duty = lerp(clampv(cal_.n2oStartPct * 0.01, 0.0, 1.0), 1.0, u);
    }
    cmd.n2oDuty = duty;
}

// ---- fuel trims: STFT integrates the wideband error in closed loop and decays in open loop; the LTFT table
//      slowly takes over what the STFT keeps correcting in a load cell, so it is there next time (open loop too)
void Ecu::fuelTrimStep(double dt, const SensorReadings& s, double rpm, bool running) {
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    const bool ltftOn = efi && cal_.ltftLearn;
    ltftNow = ltftOn ? tableLookup(cal_.ltft, rpm, mapUsed) : 0.0;
    // with auto-tune on, learning the VE map IS the closed loop (two integrators on one error would fight)
    closedLoop = efi && cal_.closedLoop && !cal_.autoTune && targetAfr > 13.5 && s.value[kSnsECT] > cal_.clMinEct &&
                 afterStart_ > cal_.clDelay && sinceTransient_ > 0.5 * cal_.clDelay && mapUsed < cal_.clMaxMap && running && fuelling_;
    if (closedLoop) {
        if (!s.fallback[kSnsWideband]) stft += cal_.stftGain * (s.value[kSnsWideband] / (targetAfr / 14.7) - 1.0) * 100.0 * dt;
        else if (!s.fallback[kSnsO2]) stft += (s.value[kSnsO2] > 0.45 ? -8.0 : 8.0) * dt;
        else closedLoop = false;
    }
    if (!closedLoop) stft *= std::exp(-dt / 0.5);
    stft = clampv(stft, -(double)cal_.stftLimit, (double)cal_.stftLimit);
    if (closedLoop && ltftOn && !cal_.ltft.locked && sinceTransient_ > cal_.clDelay) {
        const double move = stft * clampv(cal_.ltftRate * dt, 0.0, 1.0);
        Table2D& t = cal_.ltft;
        const CellWeights w = tableWeights(t, rpm, mapUsed);
        const int iy[2] = {w.iy0, w.iy1}, ix[2] = {w.ix0, w.ix1};
        const double wy[2] = {1 - w.wy, w.wy}, wx[2] = {1 - w.wx, w.wx};
        // each corner moves by its weight / sum(weight^2): the interpolated value at this point then moves by `move`
        double sw2 = 0;
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b)
                if (!((a && iy[1] == iy[0]) || (b && ix[1] == ix[0]))) sw2 += wy[a] * wx[b] * wy[a] * wx[b];
        const double before = tableLookup(t, rpm, mapUsed);
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b) {
                if ((a && iy[1] == iy[0]) || (b && ix[1] == ix[0])) continue;
                float& cell = t.v[iy[a]][ix[b]];
                cell = (float)clampv(cell + move * wy[a] * wx[b] / std::max(1e-6, sw2), -(double)cal_.ltftLimit, (double)cal_.ltftLimit);
            }
        stft -= tableLookup(t, rpm, mapUsed) - before;   // hand over exactly what the table took (limits may stop it)
        if (time_ - ltftSeqT_ > 0.25) { ++ltftSeq; ltftSeqT_ = time_; }
    }
    if (cal_.autoTune) stft = 0;
    trim = (1.0 + stft * 0.01) * (1.0 + ltftNow * 0.01);
}

double Ecu::coldFactor(const SensorReadings& s) const { return 1.0 - smoothstep(15.0, 75.0, s.value[kSnsECT]); }

void Ecu::update(double dt, const SensorReadings& s, bool starter) {
    time_ += dt;
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    const bool carb = !efi;
    cmd.syncLost = s.fallback[kSnsCKP];
    // trigger decoder: no crank sync -> no spark / fuel; no cycle phase (cam not synced, no cam sensor or it failed)
    // -> injection twice per cycle (semi-sequential) unless batch fire is what the calibration asks for anyway
    cmd.syncLost = cmd.syncLost || !s.in.crankSync;
    {
        const bool phase = !s.fallback[kSnsCMP] && s.in.camSync;
        const int want = clampv((int)cal_.injMode, 0, kInjModeCount - 1);
        cmd.injMode = want == kInjBatch ? kInjBatch : (phase ? want : kInjSemiSequential);
    }
    cmd.batchFire = cmd.injMode != kInjSequential;
    const double rpm = s.value[kSnsCKP];
    const double pedal = s.value[kSnsAPP] * 0.01;
    const double tpsPct = s.value[kSnsTPS];
    const double cold = coldFactor(s);

    // ---- MAP: sensor, or an alpha-N estimate from the throttle when it is implausible
    if (!s.fallback[kSnsMAP]) mapUsed = s.value[kSnsMAP];
    else mapUsed = baro() * clampv(0.25 + 0.95 * std::pow(tpsPct * 0.01, 0.6), 0.2, 1.0);

    // ---- transients: throttle opening rate (acceleration enrichment), MAP rate (auto-tune steadiness)
    tpsRate_ += ((tpsPct - prevTps_) / std::max(1e-4, dt) - tpsRate_) * (1.0 - std::exp(-dt / 0.03));
    prevTps_ = tpsPct;
    mapRate_ += ((mapUsed - prevMap_) / std::max(1e-4, dt) - mapRate_) * (1.0 - std::exp(-dt / 0.05));
    mapRateSlow_ += ((mapRate_) - mapRateSlow_) * (1.0 - std::exp(-dt / 0.15));   // sustained MAP rise, not idle pulsation
    prevMap_ = mapUsed;
    aeNow_ *= std::exp(-dt / std::max(0.02, (double)cal_.accelDecay));
    aeTps_ *= std::exp(-dt / std::max(0.02, (double)cal_.accelDecay));
    if (tpsRate_ > 0) {
        double ae = curveLookup(cal_.accel, tpsRate_) * 0.01 * (1.0 + (cal_.accelColdMult - 1.0) * cold);
        aeNow_ = std::max(aeNow_, ae);
        aeTps_ = std::max(aeTps_, ae);
    }
    // MAP-rate enrichment: catches air increases the throttle does not show (idle valve opening to catch a
    // falling idle, a dashpot) - the MAP sensor lags the port, so speed density alone runs lean there
    // (the fast rate only while the revs are falling - catching a sagging idle; at a steady idle the manifold
    //  pulsation would make the fast rate enrich all the time, so there only the sustained rate counts)
    const double mapRise = rpmRate_ < -300.0 ? std::max(mapRate_, mapRateSlow_) : mapRateSlow_;
    if (mapRise > 20.0 && s.value[kSnsCKP] > 400.0 && tpsPct < 2.0 && !holdIac)   // (not on the calibration rig)
        aeNow_ = std::max(aeNow_, std::min(0.4, 0.004 * (mapRise - 20.0)) * (1.0 + (cal_.accelColdMult - 1.0) * cold));
    accel = aeNow_;
    sinceCut_ += dt;
    closedFor_ = tpsPct < 1.0 ? closedFor_ + dt : 0.0;
    // port wall dryness: builds up during a fuel cut (dry after ~0.6 s), recovers once fuel flows again;
    // repeated short cuts add up instead of each starting from wet walls
    if (wasCut_) portDry_ = std::min(1.0, portDry_ + dt / 0.6);
    else portDry_ *= std::exp(-dt / std::max(0.02, (double)cal_.dfcoExitDecay));

    // a real transient: the throttle moving, a large MAP swing or real (throttle) acceleration enrichment - not the
    // MAP-rate enrichment with which the ECU itself catches a sagging idle: that is idle control, and resetting the
    // closed loop on every idle dip would keep a hunting idle out of closed loop for good
    sinceTransient_ = (std::fabs(tpsRate_) > 60.0 || std::fabs(mapRate_) > 150.0 || aeTps_ > 0.05) ? 0.0 : sinceTransient_ + dt;
    // idle control works on the speed averaged over about one firing interval: the speed ripple between two firings
    // made the idle spark jump from cycle to cycle (rough idle, "dung") and the valve twitch; real dips are not delayed
    {
        const double fire = 60.0 * revsPerEvent_ / (std::max(rpm, 300.0) * std::max(1, ports_));
        const double tau = clampv(fire, 0.005, 0.08);
        rpmIdle_ += (rpm - rpmIdle_) * (1.0 - std::exp(-dt / tau));
    }
    const double d = dt > 0 ? (rpm - prevRpm_) / dt : 0.0;
    prevRpm_ = rpm;
    rpmRate_ += (d - rpmRate_) * (1.0 - std::exp(-dt / 0.12));
    afterStart_ = rpm < 300 ? 0.0 : afterStart_ + dt;
    const bool running = rpm > 380 && !starter;
    const bool dbw = p_.comp.throttleType == ThrottleType::DriveByWire;

    // ---- protection (decides the actions), then boost control (needs them)
    protectionStep(dt, s, rpm, tpsPct, running);
    boostStep(dt, s, rpm, tpsPct);

    // ---- cam control: VTEC on rpm + load with hysteresis (part-throttle cruising stays on the economical low
    //      cam), VTC intake advance from its map
    if (p_.vtec && !limp) {
        const bool loadOk = mapUsed >= cal_.vtecMinMap && s.value[kSnsECT] >= cal_.vtecMinEct;
        if (!cmd.vtec && loadOk && rpm > cal_.vtecRpm) cmd.vtec = true;
        else if (cmd.vtec && (rpm < cal_.vtecRpm - cal_.vtecHyst || mapUsed < cal_.vtecMinMap - 10.0)) cmd.vtec = false;
    } else cmd.vtec = false;
    cmd.vtcAdvance = p_.vtc ? clampv(tableLookup(cal_.vtcMap, rpm, mapUsed), 0.0, (double)std::max(0.0f, p_.vtcRange)) : 0.0;

    // ---- gear-based control: rev limit and spark trim per gear (gear position sensor)
    const double gear = s.fallback[kSnsGear] ? 0.0 : s.value[kSnsGear];
    const bool gearCtl = cal_.gearControl && gear >= 0.5;
    double revLim = p_.revLimit;
    if (gearCtl) { const double gl = curveLookup(cal_.gearRevLimit, gear); if (gl > 500.0) revLim = std::min(revLim, gl); }
    gearLimitRpm = revLim;

    // ---- rev limiter: hard limit at the rev limit, plus an rpm cap for fuel limitation / limp mode
    //      (and for a throttle limitation the cable throttle cannot do)
    if (rpm > revLim) limiterOn_ = true;
    else if (rpm < revLim - 150) limiterOn_ = false;
    const bool capWanted = actFuel_ || limp || (actThrottle_ && !dbw);
    const double cap = std::max(1500.0f, cal_.limpRpm);
    if (capWanted && rpm > cap) capLimiter_ = true;
    else if (!capWanted || rpm < cap - 150) capLimiter_ = false;
    cmd.limiter = limiterOn_ || capLimiter_;
    const bool limFuel = limiterOn_ && p_.limiter == Limiter::FuelCut && !carb;
    const bool otherCut = capLimiter_ || overboostCut || boostCut;
    cmd.fuelCut = !carb && (limFuel || otherCut || shutdown);
    cmd.sparkCut = (limiterOn_ && !limFuel) || shutdown || (carb && otherCut);
    // soft limiter: retard and a growing share of cut sparks over the last band before the hard limit
    const double band = cal_.softLimitBand;
    const double soft = band > 1.0 ? clampv((rpm - (revLim - band)) / band, 0.0, 1.0) : 0.0;
    cmd.cutFraction = soft * clampv((double)cal_.softLimitCut, 0.0, 1.0);
    // launch / shift limiters, traction control, nitrous
    motorsportStep(dt, s, rpm, tpsPct, running);
    torqueStep(dt, s, rpm, tpsPct, dbw, running);
    cmd.cutFraction = std::max(cmd.cutFraction, std::max(std::max(msCut_, tcCut), tqCut_));

    // ---- spark: ignition map + temperature corrections (per-cylinder trims added by the engine)
    double adv;
    if (rpm < 400) adv = cal_.crankAdvance;
    else adv = tableLookup(cal_.ignition, rpm, mapUsed) + curveLookup(cal_.iatSpark, s.value[kSnsIAT]) +
               curveLookup(cal_.ectSpark, s.value[kSnsECT]);
    const double idleTarget = curveLookup(cal_.idleTarget, s.value[kSnsECT]);   // fast idle when cold
    idleTargetNow = idleTarget;
    if (efi && pedal < 0.02 && rpm > 400 && rpm < idleTarget + 500)      // idle spark control
        adv += clampv((idleTarget - rpmIdle_) * cal_.idleSparkGain, (double)cal_.idleSparkMin, (double)cal_.idleSparkMax);
    adv -= soft * cal_.softLimitRetard;
    if (actRetard_ || limp) adv -= cal_.protRetard;
    if (msTimingOn_ && rpm >= 400) adv = msTiming_;              // launch / 3-step / rolling anti-lag timing
    if (rpm >= 400) adv -= msRetard_ + tcRetard + cal_.n2oRetard * cmd.n2oDuty + tqRetard_;
    if (gearCtl && rpm >= 400) adv += curveLookup(cal_.gearSpark, gear);
    // anti-lag on lift-off: very late spark + a cracked throttle keep the turbine fed
    if (pedal > 0.3) alsTimer_ = 0; else if (pedal < 0.05) alsTimer_ += dt;
    cmd.antiLag = p_.antiLag && p_.induction == Induction::Turbo && p_.ignition && pedal < 0.05 && rpm > cal_.alsMinRpm &&
                  alsTimer_ < cal_.alsMaxTime && !limp;
    cmd.throttleCrack = cmd.antiLag ? clampv(cal_.alsThrottle * 0.01, 0.0, 1.0) : 0.0;
    if (cmd.antiLag) adv = cal_.alsTiming;
    if (launchState == kLaunchRollingAls) cmd.antiLag = true;
    cmd.sparkAdvance = adv;
    // coil dwell from the battery voltage, never more than the allowed share of the time between its sparks
    cmd.dwellMs = std::min(curveLookup(cal_.dwell, s.value[kSnsBattery]),
                           clampv((double)cal_.dwellMaxDuty, 0.05, 0.95) * ComponentModel::coilInterval(p_, std::max(rpm, 100.0)) * 1e3);
    knockStep(dt, s, rpm);

    // ---- drive-by-wire: pedal map and throttle limitation (protection / limp mode)
    throttleLimited = dbw && (actThrottle_ || limp);
    cmd.throttleTarget = -1;
    if (dbw && !holdIac && (!dbwIdentity_ || throttleLimited || s.fallback[kSnsAPP] || tqThrCap < 0.999)) {
        double req = s.fallback[kSnsAPP] ? 0.0 : curveLookup(cal_.dbwMap, s.value[kSnsAPP]) * 0.01;   // failed pedal: idle only
        if (throttleLimited) req = std::min(req, cal_.limpThrottle * 0.01);
        req = std::min(req, tqThrCap);
        cmd.throttleTarget = clampv(req, 0.0, 1.0);
    }

    // ---- X-tau wall-film model runs continuously: the film keeps evaporating during fuel cut
    tauFilm_ = 0.07 * (1.0 + 1.5 * cold);
    for (int i = 0; i < ports_; ++i) {
        double e = filmEst_[i] * (1.0 - std::exp(-dt / tauFilm_));
        filmEst_[i] -= e;
        evapAcc_[i] += e;
    }

    // ---- idle air control (EFI)
    if (efi && (holdIac || s.fallback[kSnsMAP])) {
        // calibration run / MAP failure limp mode: the idle valve sits at its nominal opening so the
        // throttle-angle (alpha-N) map describes the total air path
        cmd.iac = kIacNominal;
    } else if (efi) {
        // drive load: an automatic in gear loads the idle through its converter, against the transbrake like a
        // converter at stall - the ECU opens the idle valve ahead of the dip (neutral / drive switch)
        double ffTarget = 0;
        if (p_.vehicle && s.value[kSnsGear] > 0.5) {
            if (p_.transMode == TransMode::Automatic) ffTarget = 0.05;
            if (s.in.transbrake && p_.transMode != TransMode::Manual) ffTarget = 0.22;
        }
        driveFf_ += (ffTarget - driveFf_) * std::min(1.0, dt / 0.12);
        if (!running) { iacI_ = 0.35; cmd.iac = 0.35; }
        else if (pedal < 0.02) {
            double err = clampv(idleTarget - rpmIdle_, -500.0, 500.0);
            sinceRelease_ += dt;
            if (std::fabs(err) < 400 || sinceRelease_ > 2.5) iacI_ = clampv(iacI_ + cal_.idleKi * err * dt, 0.05, 1.0);
            dash_ *= std::exp(-dt / 0.9);
            double floor = std::max(iacI_ * 0.75, dash_);
            // proportional term stronger when the revs are low (stall side) than when they are high
            cmd.iac = clampv(iacI_ + driveFf_ + cal_.idleKp * (err > 0 ? 1.0 : 0.43) * err - cal_.idleKd * rpmRate_, floor + driveFf_, 1.0);
        } else {
            dash_ = std::max(dash_, std::min(1.0, 0.4 + pedal));
            sinceRelease_ = 0;
            cmd.iac = std::max(iacI_, 0.3);
        }
    }

    // ---- closed-loop fuel trims (short-term on the wideband, long-term learned per load cell)
    fuelTrimStep(dt, s, rpm, running);

    // ---- asynchronous tip-in fuel: a port that was fuelled for a lower load and has not inducted yet
    //      gets the missing fuel at once (otherwise its cylinder fires lean on the first cycle)
    for (int i = 0; i < ports_; ++i) asyncFuel[i] = 0;
    if (efi && cal_.asyncTipIn && fuelling_ && !cmd.fuelCut && rpm > 400 && tpsRate_ > 50.0) {
        const double cyc = 60.0 * revsPerEvent_ / std::max(rpm, 100.0);
        const double airNow = airEstimate(s, rpm, tpsPct);
        for (int i = 0; i < ports_; ++i) {
            if (lastInjT_[i] < 0 || time_ - lastInjT_[i] > 0.7 * cyc) continue;
            if (airNow > lastAir_[i] * 1.1) {
                asyncFuel[i] = (airNow - lastAir_[i]) / std::max(8.0, lastAfr_[i]);
                lastAir_[i] = airNow;
            }
        }
    }

    // ---- auto-tune: remember what was commanded, learn once the gas reaches the wideband
    const bool steady = std::fabs(tpsRate_) < 20.0 && std::fabs(mapRate_) < 40.0 && aeNow_ < 0.01;
    Hist& h = hist_[histW_];
    h.t = time_; h.rpm = rpm; h.map = mapUsed; h.tps = tpsPct; h.lam = targetAfr / 14.7;
    h.w = s.fallback[kSnsMAP] ? 1.0
        : cal_.loadMode == LoadMode::Hybrid ? smoothstep(cal_.hybridTpsLo, cal_.hybridTpsHi, tpsPct)
        : cal_.loadMode == LoadMode::AlphaN ? 1.0 : 0.0;
    h.trim = trim;
    h.ok = efi && fuelling_ && steady && running && afterStart_ > 3.0 && s.value[kSnsECT] > 60.0 && targetAfr > 0 &&
           cal_.loadMode != LoadMode::Maf;
    histW_ = (histW_ + 1) % kHist;
    tuning = false;
    if (efi && cal_.autoTune && !s.fallback[kSnsWideband]) autoTuneStep(dt, s);
}

void Ecu::autoTuneStep(double dt, const SensorReadings& s) {
    // the ECU's own estimate of exhaust transport + sensor response delay
    const double rpm = std::max(300.0, s.value[kSnsCKP]);
    const double delay = clampv(0.015 + 25.0 / rpm, 0.02, 0.12) + 0.05;
    const double want = time_ - delay;
    int k = (histW_ - 1 + kHist) % kHist;
    for (int n = 0; n < kHist && hist_[k].t > want; ++n) k = (k - 1 + kHist) % kHist;
    const Hist& h = hist_[k];
    const Hist& now = hist_[(histW_ - 1 + kHist) % kHist];
    if (!h.ok || !now.ok || h.t <= 0 || h.lam <= 0) return;
    const double ratio = clampv(s.value[kSnsWideband] / h.lam * h.trim, 0.7, 1.4);   // >1: lean -> more VE
    const double g = std::min(1.0, dt / 0.6);
    auto learn = [&](Table2D& t, double x, double y, double weight) {
        if (t.locked || weight <= 0.01) return;
        CellWeights w = tableWeights(t, x, y);
        const int iy[2] = {w.iy0, w.iy1}, ix[2] = {w.ix0, w.ix1};
        const float wy[2] = {1 - w.wy, w.wy}, wx[2] = {1 - w.wx, w.wx};
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b) {
                double cw = wy[a] * wx[b] * weight;
                if (cw < 0.02 || (a && iy[1] == iy[0]) || (b && ix[1] == ix[0])) continue;
                float& v = t.v[iy[a]][ix[b]];
                v = (float)clampv(v + g * cw * (v * ratio - v), 5.0, 160.0);
            }
        tuning = true;
    };
    learn(cal_.veMap, h.rpm, h.map, 1.0 - h.w);
    learn(cal_.veTps, h.rpm, h.tps, h.w);
    if (tuning) ++learnSeq;
}

double Ecu::airEstimate(const SensorReadings& s, double rpm, double tpsPct) {
    const double mapPa = mapUsed * 1000.0;
    const double airT = s.value[kSnsIAT] + 273.15 + 15.0;
    const double airSD = (veOverride > 0 ? veOverride : tableLookup(cal_.veMap, rpm, mapUsed)) * 0.01 * mapPa * vdCyl_ / (kRgas * airT);
    const double airAN = tableLookup(cal_.veTps, rpm, tpsPct) * 0.01 * baro() * 1000.0 * vdCyl_ / (kRgas * airT);
    const LoadMode mode = s.fallback[kSnsMAP] && cal_.loadMode != LoadMode::Maf ? LoadMode::AlphaN : cal_.loadMode;
    switch (mode) {
    case LoadMode::AlphaN: return airAN;
    case LoadMode::Hybrid: { double w = smoothstep(cal_.hybridTpsLo, cal_.hybridTpsHi, tpsPct); return (1 - w) * airSD + w * airAN; }
    case LoadMode::Maf: {
        double events = std::max(1.0, ports_ * std::max(rpm, 100.0) / (60.0 * revsPerEvent_));
        return s.fallback[kSnsMAF] ? airSD : s.value[kSnsMAF] * 1e-3 / events;
    }
    default: return airSD;
    }
}

double Ecu::pulseFor(double fuel, const SensorReadings& s) {
    if (fuel <= 0) return 0;
    const double rated = std::max(10.0, (double)cal_.injFlow) * kFuelDensity * 1e-3 / 60.0;
    const double mapGauge = (mapUsed - baro()) / 100.0;
    double rail = s.fallback[kSnsFuelP] ? p_.comp.fuelPressure_bar + (p_.comp.fuelRegulator == FuelRegulator::MapReferenced ? mapGauge : 0.0)
                                        : s.value[kSnsFuelP];
    double fc = std::sqrt(std::max(0.3, rail - mapGauge) / 3.0);
    return fuel / (rated * fc) + curveLookup(cal_.deadTime, s.value[kSnsBattery]) * 1e-3;
}

double Ecu::injectionPulse(int port, const SensorReadings& s, double share) {
    if (p_.fuelSystem != FuelSystem::EFI || port < 0 || port >= ports_) return 0;
    if (cmd.syncLost) { targetAfr = 0; injDuty = 0; fuelling_ = false; return 0; }
    const double rpm = s.value[kSnsCKP];
    const double tpsPct = s.value[kSnsTPS];
    const double mapPa = mapUsed * 1000.0;
    const double cold = coldFactor(s);
    // deceleration fuel cut-off; resume early when the revs are dropping fast (anticipate ~0.3 s)
    double resume = std::max(1600.0, idleTargetNow + cal_.dfcoResumeAbove) + cal_.dfcoAnticipate * std::max(0.0, -rpmRate_);
    // one cut per deceleration: once fuel is back it stays on until the throttle has been opened again (the resume
    // point moves with the decel rate, which itself drops as soon as the engine fires - otherwise the cut chatters)
    if (tpsPct > 2.0) dfcoArmed_ = true;
    bool dfco = tpsPct < 1.0 && rpm > resume && (wasCut_ || dfcoArmed_) && (wasCut_ || closedFor_ >= cal_.dfcoDelay);
    // launch limiter in fuel-cut mode: a share of the injection events is dropped
    const bool limitDrop = cmd.fuelCutFraction > 0 && rand01() < cmd.fuelCutFraction;
    if (cmd.fuelCut || dfco || limitDrop) {
        if (limitDrop && !cmd.fuelCut && !dfco) { injDuty = 0; return 0; }   // not a decel cut: no port-wall bookkeeping
        targetAfr = 0; injDuty = 0; fuelling_ = false; evapAcc_[port] = 0;   // film evaporated into the cut cycles
        wasCut_ = true;
        return 0;
    }
    if (wasCut_) { wasCut_ = false; sinceCut_ = 0; dfcoArmed_ = false; }
    fuelling_ = true;
    // ---- air per cylinder event from the load model
    const double airT = s.value[kSnsIAT] + 273.15 + 15.0;                // port air a bit warmer than the inlet
    const double airSD = (veOverride > 0 ? veOverride : tableLookup(cal_.veMap, rpm, mapUsed)) * 0.01 * mapPa * vdCyl_ / (kRgas * airT);
    const double airAN = tableLookup(cal_.veTps, rpm, tpsPct) * 0.01 * baro() * 1000.0 * vdCyl_ / (kRgas * airT);
    double air;
    // a failed MAP sensor: limp home on the throttle-angle (alpha-N) map
    const LoadMode mode = s.fallback[kSnsMAP] && cal_.loadMode != LoadMode::Maf ? LoadMode::AlphaN : cal_.loadMode;
    switch (mode) {
    case LoadMode::AlphaN: air = airAN; veUsed = tableLookup(cal_.veTps, rpm, tpsPct); break;
    case LoadMode::Hybrid: {
        double w = smoothstep(cal_.hybridTpsLo, cal_.hybridTpsHi, tpsPct);
        air = (1 - w) * airSD + w * airAN;
        veUsed = (1 - w) * tableLookup(cal_.veMap, rpm, mapUsed) + w * tableLookup(cal_.veTps, rpm, tpsPct);
        break;
    }
    case LoadMode::Maf: {
        double events = std::max(1.0, ports_ * std::max(rpm, 100.0) / (60.0 * revsPerEvent_));
        air = s.fallback[kSnsMAF] ? airSD : s.value[kSnsMAF] * 1e-3 / events;
        veUsed = air * kRgas * airT / std::max(1.0, mapPa * vdCyl_) * 100.0;
        break;
    }
    case LoadMode::SpeedDensity: default: air = airSD; veUsed = veOverride > 0 ? veOverride : tableLookup(cal_.veMap, rpm, mapUsed); break;
    }
    airPerCyl = air * 1000.0;
    lastInjT_[port] = time_; lastAir_[port] = air;
    loadX = rpm; loadY = mapUsed; tpsUsed = tpsPct;
    // ---- target mixture
    const bool cranking = rpm < 400;
    double lamT = tableLookup(cal_.lambda, rpm, mapUsed);
    // power enrichment on throttle angle (wide open before the manifold / boost has caught up)
    lamT = std::min(lamT, lerp(lamT, (double)cal_.powerLambda, smoothstep(cal_.powerTpsLo, std::max(cal_.powerTpsLo + 1.0f, cal_.powerTpsHi), tpsPct)));
    double afr = cranking ? curveLookup(cal_.crankAfr, s.value[kSnsECT]) : lamT * 14.7;
    if (!cranking) afr = std::min(afr, lerp((double)cal_.afterStartAfr, afr, smoothstep(0.0, std::max(0.1, (double)cal_.afterStartTime), afterStart_)));
    targetAfr = afr;
    lastAfr_[port] = afr;
    double fuel = air / std::max(5.0, afr);
    // dry nitrous kit: the ECU adds the fuel for the N2O it believes is flowing (its own calibration value)
    if (p_.nitrous && !p_.n2oWet && cmd.n2oDuty > 0 && rpm > 300) {
        const double eventsPerS = ports_ * rpm / (60.0 * revsPerEvent_);
        fuel += cal_.n2oFlowGs * 1e-3 * cmd.n2oDuty * kN2oAirEquiv / std::max(5.0, afr) / eventsPerS;
    }
    if (launchState != kLaunchOff && launchState != kLaunchShiftCut) fuel *= 1.0 + msEnrich_ * 0.01;
    // ---- enrichments and corrections
    warmup = cranking ? 0.0 : curveLookup(cal_.warmup, s.value[kSnsECT]) * 0.01;   // cranking AFR already covers it
    // the longer the cut, the drier the port walls: full enrichment after ~0.6 s of cut
    const double dfcoExit = cal_.dfcoExitEnrich * 0.01 * (1.0 + (cal_.accelColdMult - 1.0) * cold) * portDry_;
    fuel *= (1.0 + warmup) * (1.0 + (cranking ? 0.0 : aeNow_ + dfcoExit)) * (1.0 + curveLookup(cal_.iatFuel, s.value[kSnsIAT]) * 0.01) *
            (1.0 + cal_.cylFuel[clampv(port, 0, kCalMaxCyl - 1)] * 0.01);
    if (p_.injection == Injection::Port) {
        // X-tau wall-film model (the ECU's own estimate of the fuel film on the port walls)
        const double dep = 0.20 + 0.12 * cold;
        double evap = evapAcc_[port];
        evapAcc_[port] = 0;
        double cmdFuel = std::max(0.0, (fuel - evap) / (1.0 - dep));
        filmEst_[port] += cmdFuel * dep;
        fuel = cmdFuel;
    } else evapAcc_[port] = 0;
    fuel *= trim;   // short-term (closed loop) x long-term trim
    fuel *= share;
    // ---- fuel mass -> pulse width with the ECU's injector data
    const double cyc = rpm > 30 ? 60.0 * revsPerEvent_ / rpm : 1.0;
    const double rated = std::max(10.0, (double)cal_.injFlow) * kFuelDensity * 1e-3 / 60.0;   // kg/s at 3 bar
    const double mapGauge = (mapUsed - baro()) / 100.0;
    double rail = s.fallback[kSnsFuelP] ? p_.comp.fuelPressure_bar + (p_.comp.fuelRegulator == FuelRegulator::MapReferenced ? mapGauge : 0.0)
                                        : s.value[kSnsFuelP];
    double dP = std::max(0.3, rail - mapGauge);
    flowComp = std::sqrt(dP / 3.0);
    deadComp_ms = curveLookup(cal_.deadTime, s.value[kSnsBattery]);
    double pw = fuel / (rated * flowComp) + deadComp_ms * 1e-3;
    if (fuel <= 0) pw = 0;
    injDuty = std::min(1.5, pw / cyc / std::max(1e-3, share));
    return std::min(pw, cyc);
}

} // namespace es

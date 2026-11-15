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
    iacI_ = 0.35; cmd = ActuatorCommands();
}

// take the new calibration - except VE tables the tuner did not touch (they may hold auto-tune results)
void Ecu::apply(const EngineParams& p) {
    p_ = p;
    Table2D keepVe = cal_.veMap, keepTps = cal_.veTps;
    bool veSame = std::memcmp(&p.cal.veMap, &lastIn_.veMap, sizeof(Table2D)) == 0;
    bool tpsSame = std::memcmp(&p.cal.veTps, &lastIn_.veTps, sizeof(Table2D)) == 0;
    cal_ = p.cal;
    if (veSame) { cal_.veMap = keepVe; cal_.veMap.locked = p.cal.veMap.locked; }
    if (tpsSame) { cal_.veTps = keepTps; cal_.veTps.locked = p.cal.veTps.locked; }
    lastIn_ = p.cal;
}

double Ecu::coldFactor(const SensorReadings& s) const { return 1.0 - smoothstep(15.0, 75.0, s.value[kSnsECT]); }

void Ecu::update(double dt, const SensorReadings& s, bool starter) {
    time_ += dt;
    const bool efi = p_.fuelSystem == FuelSystem::EFI;
    const bool carb = !efi;
    cmd.syncLost = s.fallback[kSnsCKP];
    cmd.batchFire = s.fallback[kSnsCMP];
    const double rpm = s.value[kSnsCKP];
    const double pedal = s.value[kSnsAPP] * 0.01;
    const double tpsPct = s.value[kSnsTPS];
    const double cold = coldFactor(s);

    // ---- MAP: sensor, or an alpha-N estimate from the throttle when it is implausible
    if (!s.fallback[kSnsMAP]) mapUsed = s.value[kSnsMAP];
    else mapUsed = 101.3 * clampv(0.25 + 0.95 * std::pow(tpsPct * 0.01, 0.6), 0.2, 1.0);

    // ---- transients: throttle opening rate (acceleration enrichment), MAP rate (auto-tune steadiness)
    tpsRate_ += ((tpsPct - prevTps_) / std::max(1e-4, dt) - tpsRate_) * (1.0 - std::exp(-dt / 0.03));
    prevTps_ = tpsPct;
    mapRate_ += ((mapUsed - prevMap_) / std::max(1e-4, dt) - mapRate_) * (1.0 - std::exp(-dt / 0.05));
    prevMap_ = mapUsed;
    aeNow_ *= std::exp(-dt / std::max(0.02, (double)cal_.accelDecay));
    if (tpsRate_ > 0) {
        double ae = curveLookup(cal_.accel, tpsRate_) * 0.01 * (1.0 + (cal_.accelColdMult - 1.0) * cold);
        aeNow_ = std::max(aeNow_, ae);
    }
    accel = aeNow_;
    sinceCut_ += dt;

    // ---- rev limiter
    if (rpm > p_.revLimit) limiterOn_ = true;
    else if (rpm < p_.revLimit - 150) limiterOn_ = false;
    cmd.limiter = limiterOn_;
    cmd.fuelCut = limiterOn_ && p_.limiter == Limiter::FuelCut && !carb;
    cmd.sparkCut = limiterOn_ && !cmd.fuelCut;

    // ---- spark: ignition map + temperature corrections (per-cylinder trims added by the engine)
    double adv;
    if (rpm < 400) adv = cal_.crankAdvance;
    else adv = tableLookup(cal_.ignition, rpm, mapUsed) + curveLookup(cal_.iatSpark, s.value[kSnsIAT]) +
               curveLookup(cal_.ectSpark, s.value[kSnsECT]);
    const double idleTarget = p_.idleTargetRpm + 300.0 * cold;           // fast idle when cold
    if (efi && pedal < 0.02 && rpm > 400 && rpm < idleTarget + 500)      // idle spark control
        adv += clampv((idleTarget - rpm) * 0.04, -8.0, 10.0);
    if (pedal > 0.3) alsTimer_ = 0; else if (pedal < 0.05) alsTimer_ += dt;
    cmd.antiLag = p_.antiLag && p_.induction == Induction::Turbo && p_.ignition && pedal < 0.05 && rpm > 2500 && alsTimer_ < 2.0;
    cmd.throttleCrack = cmd.antiLag ? 0.10 : 0.0;
    if (cmd.antiLag) adv = -30.0;
    cmd.sparkAdvance = adv;

    // ---- X-tau wall-film model runs continuously: the film keeps evaporating during fuel cut
    tauFilm_ = 0.07 * (1.0 + 1.5 * cold);
    for (int i = 0; i < ports_; ++i) {
        double e = filmEst_[i] * (1.0 - std::exp(-dt / tauFilm_));
        filmEst_[i] -= e;
        evapAcc_[i] += e;
    }

    // ---- idle air control (EFI)
    const double d = dt > 0 ? (rpm - prevRpm_) / dt : 0.0;
    prevRpm_ = rpm;
    rpmRate_ += (d - rpmRate_) * (1.0 - std::exp(-dt / 0.12));
    afterStart_ = rpm < 300 ? 0.0 : afterStart_ + dt;
    const bool running = rpm > 380 && !starter;
    if (efi && (holdIac || s.fallback[kSnsMAP])) {
        // calibration run / MAP failure limp mode: the idle valve sits at its nominal opening so the
        // throttle-angle (alpha-N) map describes the total air path
        cmd.iac = kIacNominal;
    } else if (efi) {
        if (!running) { iacI_ = 0.35; cmd.iac = 0.35; }
        else if (pedal < 0.02) {
            double err = clampv(idleTarget - rpm, -500.0, 500.0);
            sinceRelease_ += dt;
            if (std::fabs(err) < 400 || sinceRelease_ > 2.5) iacI_ = clampv(iacI_ + 0.0002 * err * dt, 0.05, 1.0);
            dash_ *= std::exp(-dt / 0.9);
            double floor = std::max(iacI_ * 0.75, dash_);
            cmd.iac = clampv(iacI_ + (err > 0 ? 0.0007 : 0.0003) * err - 0.00012 * rpmRate_, floor, 1.0);
        } else {
            dash_ = std::max(dash_, std::min(1.0, 0.4 + pedal));
            sinceRelease_ = 0;
            cmd.iac = std::max(iacI_, 0.3);
        }
    }

    // ---- closed-loop fuel trim on the wideband (narrowband switching as backup)
    // with auto-tune on, learning the VE map IS the closed loop (two integrators on one error would fight)
    if (cal_.autoTune) trim += (1.0 - trim) * std::min(1.0, dt / 0.5);
    closedLoop = efi && cal_.closedLoop && !cal_.autoTune && targetAfr > 13.5 && s.value[kSnsECT] > 60.0 && afterStart_ > 3.0 && running && fuelling_;
    if (closedLoop) {
        if (!s.fallback[kSnsWideband]) {
            double lamTarget = targetAfr / 14.7;
            trim = clampv(trim + 0.5 * (s.value[kSnsWideband] / lamTarget - 1.0) * dt, 0.7, 1.3);
        } else if (!s.fallback[kSnsO2]) {
            trim = clampv(trim + (s.value[kSnsO2] > 0.45 ? -0.08 : 0.08) * dt, 0.7, 1.3);
        } else closedLoop = false;
    }

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
    h.trim = (cal_.closedLoop && !cal_.autoTune && targetAfr > 13.5) ? trim : 1.0;
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
    const double airSD = tableLookup(cal_.veMap, rpm, mapUsed) * 0.01 * mapPa * vdCyl_ / (kRgas * airT);
    const double airAN = tableLookup(cal_.veTps, rpm, tpsPct) * 0.01 * 101325.0 * vdCyl_ / (kRgas * airT);
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
    const double mapGauge = (mapUsed - 101.325) / 100.0;
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
    double resume = std::max(1600.0, p_.idleTargetRpm + 1000.0) + 0.3 * std::max(0.0, -rpmRate_);
    bool dfco = tpsPct < 1.0 && rpm > resume;
    if (cmd.fuelCut || dfco) { targetAfr = 0; injDuty = 0; fuelling_ = false; evapAcc_[port] = 0; if (!wasCut_) cutStart_ = time_; wasCut_ = true; return 0; }   // film evaporated into the cut cycles
    if (wasCut_) { wasCut_ = false; sinceCut_ = 0; cutLen_ = time_ - cutStart_; }
    fuelling_ = true;
    // ---- air per cylinder event from the load model
    const double airT = s.value[kSnsIAT] + 273.15 + 15.0;                // port air a bit warmer than the inlet
    const double airSD = tableLookup(cal_.veMap, rpm, mapUsed) * 0.01 * mapPa * vdCyl_ / (kRgas * airT);
    const double airAN = tableLookup(cal_.veTps, rpm, tpsPct) * 0.01 * 101325.0 * vdCyl_ / (kRgas * airT);
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
    case LoadMode::SpeedDensity: default: air = airSD; veUsed = tableLookup(cal_.veMap, rpm, mapUsed); break;
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
    // ---- enrichments and corrections
    warmup = cranking ? 0.0 : curveLookup(cal_.warmup, s.value[kSnsECT]) * 0.01;   // cranking AFR already covers it
    // the longer the cut, the drier the port walls: full enrichment after ~0.6 s of cut
    const double dfcoExit = cal_.dfcoExitEnrich * 0.01 * (1.0 + (cal_.accelColdMult - 1.0) * cold) * smoothstep(0.1, 0.6, cutLen_) * std::exp(-sinceCut_ / std::max(0.02, (double)cal_.dfcoExitDecay));
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
    if (cal_.closedLoop && !cal_.autoTune && afr > 13.5) fuel *= trim;
    fuel *= share;
    // ---- fuel mass -> pulse width with the ECU's injector data
    const double cyc = rpm > 30 ? 60.0 * revsPerEvent_ / rpm : 1.0;
    const double rated = std::max(10.0, (double)cal_.injFlow) * kFuelDensity * 1e-3 / 60.0;   // kg/s at 3 bar
    const double mapGauge = (mapUsed - 101.325) / 100.0;
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

#include "sim/Boost.h"
#include <cstring>

namespace es {

static constexpr double kPRmax = 3.6;          // centrifugal compressor PR at max speed, zero flow
static constexpr double kTurboMaxRpm = 190000; // turbo wheel speed at "100 %"
static constexpr double kImpellerMaxRpm = 60000;
static constexpr double kCpAir = 1005, kCpExh = 1150;

void ForcedInduction::refreshMaps() {
    const bool turbo = p_.induction == Induction::Turbo;
    const bool centrifugal = turbo || p_.scType == Supercharger::Centrifugal;
    const bool use = p_.turbo.useMaps != 0;
    if (use != (useSrc_ == 1) || std::memcmp(&compSrc_, &p_.turbo.comp, sizeof(compSrc_)) != 0 ||
        std::memcmp(&turbSrc_, &p_.turbo.turb, sizeof(turbSrc_)) != 0) {
        compSrc_ = p_.turbo.comp; turbSrc_ = p_.turbo.turb; useSrc_ = use ? 1 : 0;
        if (use) { cm_.build(compSrc_); tm_.build(turbSrc_); }
    }
    mapC_ = use && centrifugal && cm_.valid();
    mapT_ = use && turbo && tm_.valid();
    nMaxC_ = mapC_ ? cm_.maxSpeed() : (turbo ? kTurboMaxRpm : kImpellerMaxRpm);
    sizeKgs_ = mapC_ ? cm_.maxFlow() : std::max(1e-4, p_.compFlow_gs * 1e-3);
}

void ForcedInduction::init(const EngineParams& p, double fs) {
    p_ = p;
    fs_ = fs;
    ambT_ = 273.15 + p.ambientTemp_C;
    chargeT = ambT_;
    icCoreT = ambT_;
    refreshMaps();
    omegaT_ = 0;
    whooshBp_.bandpass(2200, 1.1, fs);
    bovHiss_.bandpass(4800, 0.9, fs);
    bovBody_.bandpass(1300, 1.6, fs);
    surgeBp_.bandpass(950, 2.2, fs);
    surgeLow_.bandpass(320, 1.4, fs);
    scHiss_.bandpass(3500, 1.0, fs);
    whineLp_.lowpass(11000, fs);
    ampSmooth_.lowpass(30, fs);
    wobble_.lowpass(6, fs);
}

double ForcedInduction::prAtZeroFlow(double n) { return 1.0 + (kPRmax - 1.0) * n * n; }

double ForcedInduction::chokeFlow(double n) const {
    return std::max(1e-4, p_.compFlow_gs * 1e-3) * (0.12 + 0.88 * n);
}

// Simplified centrifugal compressor map: each speed line runs from (0 flow, PR0(n)) to
// (choke flow, PR 1). This is only the flow the spinning wheel *pumps*; air that simply leaks
// through the wheel (engine sucking through a stopped turbo) is a separate passive orifice.
double ForcedInduction::compressor(double n, double prc, double& eta) const {
    const double pr0 = prAtZeroFlow(n);
    const double mPump = std::max(1e-4, p_.compFlow_gs * 1e-3) * (0.12 * smoothstep(0.0, 0.12, n) + 0.88 * n); // 0 when stopped
    if (n < 0.02) { eta = 0.4; return 0.0; }
    if (prc <= pr0) {
        double x = clampv((prc - 1.0) / std::max(1e-3, pr0 - 1.0), 0.0, 1.0); // 0 at choke, 1 at surge edge
        double m = mPump * std::sqrt(1.0 - x);
        double f = m / chokeFlow(n);
        eta = clampv(0.76 * (1.0 - 1.6 * (f - 0.62) * (f - 0.62)), 0.42, 0.76);
        return m;
    }
    eta = 0.4;
    if (n < 0.15) return 0.0;               // too slow to stall meaningfully
    return -0.35 * mPump * std::sqrt(std::min(1.0, (prc - pr0) / pr0));
}

double ForcedInduction::step(double dt, double omegaE, double egtK, IntakeSystem& in, ExhaustSystem& ex) {
    GasNode& charge = in.chargeNode();
    const double pIn = kPatm;                    // compressor inlet (air box)
    const double rhoIn = pIn / (kR * ambT_);
    const double engineRpm = omegaE * 60.0 / (2 * kPi);
    const double throttle = in.throttlePosition();
    pr = charge.p() / pIn;
    boostBar = (charge.p() - kPatm) / 1e5;
    double crankTorque = 0;
    double m = 0, eta = 0.7;
    surging = false;
    bool centrifugal = p_.induction == Induction::Turbo || p_.scType == Supercharger::Centrifugal;

    if (centrifugal) {
        double rpm = p_.induction == Induction::Turbo ? omegaT_ * 60.0 / (2 * kPi)
                                                      : engineRpm * std::max(1.0f, p_.scRatio);
        speedFrac = clampv(rpm / nMaxC_, 0.0, 1.3);
        shaftRpm = rpm;
        double chokeNow;
        if (mapC_) {
            // table map: corrected speed and flow (inlet at ambient pressure and temperature)
            const double sq = std::sqrt(kMapTref / ambT_);
            const double corrM = pIn / kMapPref * sq;
            const CompPoint cp = cm_.eval(rpm * sq, pr);
            chokeNow = cp.chokeFlow * corrM;
            eta = cp.eff;
            if (speedFrac < 0.02) { m = 0; eta = 0.4; }
            else if (cp.surge) m = speedFrac < 0.15 ? 0.0 : -0.35 * cp.surgeFlow * corrM * std::sqrt(std::min(1.0, (pr - cp.surgePr) / cp.surgePr));
            else m = cp.flow * corrM;
            surgeMargin = cp.surgePr > 1.0 ? 1.0 - (pr - 1.0) / (cp.surgePr - 1.0) : 1.0;
        } else {
            m = compressor(speedFrac, pr, eta);
            chokeNow = chokeFlow(speedFrac);
            const double pr0 = prAtZeroFlow(speedFrac);
            surgeMargin = pr0 > 1.0 ? 1.0 - (pr - 1.0) / (pr0 - 1.0) : 1.0;
        }
        // surge: the throttle is shut, so the compressor can only push a trickle against high
        // pressure (left of the surge line). The flow then stalls and reverses in bursts.
        bool leftOfSurgeLine = speedFrac > 0.15 && pr > 1.2 && throttle < 0.08 &&
                               std::max(0.0, m) < 0.25 * chokeNow + 1e-4;
        surging = leftOfSurgeLine || (m < 0 && pr > 1.05);
        if (surging) {
            double rate = 7.0 + 26.0 * speedFrac;                 // stall cycles per second
            flutterPh_ += dt * rate;
            if (flutterPh_ >= 1) flutterPh_ -= 1;
            // short reverse-flow burst each cycle, then the wheel re-pressurises
            double back = 0.45 * chokeNow * clampv((pr - 1.0) / 0.8, 0.0, 1.5);
            m = flutterPh_ < 0.28 ? -back : std::max(m, 0.0) * 0.4;
        }
    } else {
        // positive displacement blower: pumps a fixed volume per revolution, minus leakage
        double nsc = engineRpm * std::max(0.1f, p_.scRatio);
        shaftRpm = nsc;
        speedFrac = clampv(nsc / 16000.0, 0.0, 1.3);
        double leak = p_.scType == Supercharger::Roots ? 0.20 : 0.10;
        double mIdeal = rhoIn * std::max(0.05f, p_.scDisplacement_L) * 1e-3 * nsc / 60.0;
        m = mIdeal * (1.0 - leak * std::max(0.0, pr - 1.0)) - 0.002 * std::max(0.0, pr - 1.0);
        eta = p_.scType == Supercharger::Roots ? 0.52 : 0.70;
    }

    // compressor work & outlet temperature
    double tau = std::pow(std::max(pr, 0.5), 0.2857) - 1.0;
    double T2 = ambT_ * (1.0 + std::max(0.0, tau) / std::max(0.3, eta));
    double powerC = std::max(0.0, m) * kCpAir * ambT_ * std::max(0.0, tau) / std::max(0.3, eta);
    // intercooler: effectiveness against its core temperature; the core takes the heat and gives it to the air
    // blown through it (vehicle speed / dyno fan) - parked or on a dyno without a fan it heat-soaks
    compOutT = T2;
    double Tcharge = T2;
    if (p_.intercooler) {
        const double eps = clampv((double)p_.icEfficiency, 0.0, 1.0);
        Tcharge = T2 - eps * (T2 - icCoreT);
        const double size = std::max(0.4, sizeKgs_ / 0.25);
        const double cCore = 5400.0 * size;                                   // ~6 kg of aluminium
        const double ua = (60.0 + 175.0 * std::pow(std::max(0.0, coolAir_), 0.8)) * size;
        const double q = std::max(0.0, m) * kCpAir * (T2 - Tcharge);
        icCoreT += dt * (q - ua * (icCoreT - ambT_)) / cCore;
    } else icCoreT = ambT_;
    compEff = eta;
    compFlowCorr = m / (pIn / kMapPref * std::sqrt(kMapTref / ambT_));

    // passive path through the (stopped or slow) compressor wheel: a plain orifice, smooth in
    // both directions - this is how the engine breathes before the turbo spools
    if (centrifugal) {
        double slow = 1.0 - clampv(speedFrac, 0.0, 1.0);
        double aPass = 1.2 * std::max(0.05, sizeKgs_) / 236.0 * slow * slow;
        double fp = limitedFlow(aPass, pIn, ambT_, 1e3, charge.p(), charge.T, charge.V, dt, 0.4);
        // a spinning wheel blocks reverse flow (that only happens as surge); a stopped one doesn't
        if (fp < 0 && speedFrac > 0.08) fp = 0;
        m += fp;
    } else {
        // positive-displacement blower below atmospheric outlet pressure (engine pulls more than the
        // slow rotors deliver): air also streams through the rotor / case clearances, so the blower is
        // a leaky restriction, not a wall
        double aLeak = (p_.scType == Supercharger::Roots ? 2.5e-3 : 1.2e-3) * std::sqrt(std::max(0.2f, p_.scDisplacement_L));
        double fp = limitedFlow(aLeak, pIn, ambT_, 1e3, charge.p(), charge.T, charge.V, dt, 0.4);
        if (fp > 0) m += fp;
    }
    // never move more than a fraction of the charge mass per step (explicit-step stability)
    m = clampv(m, -0.25 * charge.m / dt, 0.25 * charge.m / dt + 1.0);
    // move mass: air box -> charge node (or back out in surge)
    double q = m * dt;
    if (q > 0) charge.m += q;
    else {
        q = std::min(-q, charge.m * 0.2);
        double ff = charge.fuelFrac(), bf = charge.burnedFrac();
        charge.m -= q; charge.mf -= q * ff; charge.mb -= q * bf;
    }
    compFlow = m;
    // charge air temperature follows the compressor outlet (after the intercooler)
    if (m > 1e-4) chargeT += (Tcharge - chargeT) * std::min(1.0, dt / 0.4);
    else chargeT += (ambT_ - chargeT) * std::min(1.0, dt / 8.0);
    {   // isothermal node: change T while keeping its pressure continuous
        double pNow = charge.p();
        charge.T = chargeT;
        charge.setPressure(pNow);
    }

    // blow-off valve / supercharger bypass: vent the charge when the throttle shuts on boost
    bovOpen = false;
    double vent = 0;
    ventFlow = turbFlow = wgFlow = 0;
    if (p_.induction == Induction::Turbo && p_.bov && throttle < 0.08 && boostBar > 0.05 && charge.p() - in.map() > 0.25e5) bovOpen = true;
    if (p_.induction == Induction::Supercharger && p_.scBypass && throttle < 0.15) bovOpen = true;
    if (bovOpen) {
        double area = p_.induction == Induction::Turbo ? 3.5e-4 : 1.5e-3;
        vent = limitedFlow(area, charge.p(), charge.T, charge.V, kPatm, ambT_, 1e3, dt, 0.4);
        if (vent > 0) {
            double qv = std::min(vent * dt, charge.m * 0.3);
            ventFlow = qv / dt;
            double ff = charge.fuelFrac(), bf = charge.burnedFrac();
            charge.m -= qv; charge.mf -= qv * ff; charge.mb -= qv * bf;
        }
    }

    if (p_.induction == Induction::Turbo && ex.hasTurbine()) {
        // ---- turbine + wastegate between collector and downpipe
        GasNode& tin = ex.turbineIn();
        GasNode& tout = ex.turbineOut();
        double aT = std::max(0.2f, p_.turbineAR) * 1.0e-3 * std::sqrt(std::max(0.3, equivalentDisplacementL(p_) / 2.0));
        // spring + diaphragm wastegate: proportional over ~0.3 bar with a damped actuator, plus a
        // slow integral trim (boost controller) so it settles on the target instead of hunting
        // ECU control: a solenoid bleeds the actuator line, so it only sees boost x (1 - 0.9 duty); the ECU's
        // PID replaces the slow trim of the plain pneumatic controller
        const bool ecuWg = wgDuty_ >= 0.0;
        const double actBar = ecuWg ? boostBar * (1.0 - 0.9 * clampv(wgDuty_, 0.0, 1.0)) : boostBar;
        if (ecuWg) wgTrim_ = 0;
        else wgTrim_ = clampv(wgTrim_ + (boostBar - p_.wastegateBar) * 0.8 * dt, -0.5, 0.8);
        // rate term damps the turbo / wastegate loop (opens early while boost is still rising)
        double rate = (boostBar - prevBoost_) / dt;
        prevBoost_ = boostBar;
        boostRate_ += (rate - boostRate_) * std::min(1.0, dt / 0.03);
        double wgTarget = clampv((actBar - p_.wastegateBar + 0.08) / 0.3 + wgTrim_ + 0.12 * boostRate_ * (ecuWg ? 0.0 : 1.0), 0.0, 1.0);
        wgPos_ += (wgTarget - wgPos_) * std::min(1.0, dt / 0.12);
        wastegate = wgPos_;
        const double erNow = std::max(1.0, tin.p() / std::max(1.0, tout.p()));
        if (mapT_) {
            // table map: the reduced flow at this expansion ratio as an equivalent nozzle area
            const double er = std::max(erNow, 1.02);
            const double mMap = tm_.mfp(er) * tin.p() * 1e-3 / std::sqrt(std::max(300.0, tin.T));
            const double unit = orificeFlow(1.0, tin.p(), tin.T, tin.p() / er);
            if (unit > 1e-12) aT = std::max(1e-6, mMap / unit);
        }
        // a wide-open wastegate bypasses most of the exhaust (real ports pass ~2-3x the turbine's flow area)
        double aWg = wgPos_ * aT * 2.5;
        if (p_.vgt) {
            // variable geometry: no wastegate; the guide vanes set the turbine's flow area. The ECU duty closes them
            // (no duty / no ECU: open). Area over the vane opening u as the ellipse arc of Wahlstrom & Eriksson
            // (LiTH-ISY-R-2882 eq. 53-55): f = c_f2 + c_f1 sqrt(1 - ((u - c_vgt2) / c_vgt1)^2), here centred on the
            // open end (c_vgt1 = c_vgt2 = 1) and running from vgtMinArea (closed) to 1 (open) - their fitted
            // constants belong to their engine. Actuator: first order, 0.15 s (chosen).
            const double u = 1.0 - clampv(wgDuty_ >= 0.0 ? wgDuty_ : 0.0, 0.0, 1.0);
            vanePos_ += (u - vanePos_) * std::min(1.0, dt / 0.15);
            const double amin = clampv((double)p_.vgtMinArea, 0.05, 1.0);
            const double fv = amin + (1.0 - amin) * std::sqrt(std::max(0.0, 1.0 - (vanePos_ - 1.0) * (vanePos_ - 1.0)));
            aT *= fv;
            aWg = 0;
            wgPos_ = 0;
            wastegate = vanePos_;   // (telemetry: the vane opening)
        }
        double flow = limitedFlow(aT + aWg, tin.p(), tin.T, tin.V, tout.p(), tout.T, tout.V, dt, 0.35);
        double qf = flow * dt;
        GasNode& s = qf >= 0 ? tin : tout;
        GasNode& d = qf >= 0 ? tout : tin;
        double qa = std::min(std::fabs(qf), s.m * 0.3);
        double ff = s.fuelFrac(), bf = s.burnedFrac();
        s.m -= qa; s.mf -= qa * ff; s.mb -= qa * bf;
        d.m += qa; d.mf += qa * ff; d.mb += qa * bf;
        {   // (signed: the turbine and the wastegate share the flow by their areas)
            const double q = (qf >= 0 ? qa : -qa) / dt;
            turbFlow = q * aT / (aT + aWg); wgFlow = q - turbFlow;
        }
        double mt = std::max(0.0, flow) * aT / (aT + aWg);
        double prt = std::max(1.0, tin.p() / std::max(1.0, tout.p()));
        double Tin = std::max(600.0, egtK);
        const double dh = kCpExh * Tin * (1.0 - std::pow(prt, -0.248));
        double etaT = 0.68;
        if (mapT_) {
            const double U = omegaT_ * tm_.wheelDia() * 0.5e-3;
            const double Cs = std::sqrt(2.0 * std::max(0.0, dh));
            turbUc = Cs > 1.0 ? U / Cs : 0.0;
            etaT = Cs > 1.0 ? tm_.eff(turbUc) : 0.3;
        } else turbUc = 0;
        turbEff = etaT; turbEr = erNow;
        turbMfp = mt * std::sqrt(std::max(300.0, tin.T)) / std::max(1.0, tin.p() * 1e-3);
        double powerT = mt * dh * etaT;
        // the turbine takes this work out of the gas; the wastegate flow passes without expanding through it
        {   // (averaged power over averaged flow: the pulsating instantaneous ratio would be meaningless)
            const double k = std::min(1.0, dt / 0.08);
            turbPowAvg_ += (powerT - turbPowAvg_) * k;
            turbFlowAvg_ += (std::max(0.0, flow) - turbFlowAvg_) * k;
            turbineDropK = turbPowAvg_ / (std::max(1e-3, turbFlowAvg_) * kCpExh);
        }
        // ---- shaft
        double J = std::max(0.02f, p_.turboInertia) * 1e-4;
        double w = std::max(omegaT_, 400.0);
        double fric = 1.5e-6 * omegaT_ + 2e-4;
        omegaT_ += ((powerT - powerC) / w - fric) / J * dt;
        omegaT_ = clampv(omegaT_, 0.0, nMaxC_ * 1.25 * 2 * kPi / 60.0);
        powerKw = powerC / 1000.0;
    } else if (p_.induction == Induction::Supercharger) {
        double drag = 150.0 + 4e-3 * shaftRpm * shaftRpm * 1e-3;   // bearing / gear losses (W)
        double pw = (bovOpen ? powerC * 0.25 : powerC) + drag;
        powerKw = pw / 1000.0;
        crankTorque = pw / std::max(30.0, omegaE);
        wastegate = 0;
    }

    // ---- sound (Pa @ 1 m). Synthesised from the physical state: shaft speed, flow, pressure
    // ratio, vent flow and surge cycles - so it only appears when the hardware is working.
    double snd = 0;
    const double flowFrac = clampv(std::max(0.0, m) / std::max(1e-4, sizeKgs_), 0.0, 1.2);
    const double prLoad = clampv((pr - 1.0) / 1.2, 0.0, 1.5);
    const double noise = rng_.bipolar();
    if (centrifugal) {
        const bool turbo = p_.induction == Induction::Turbo;
        // spool whistle: pitch follows wheel speed (190k rpm -> ~5 kHz), loudness follows how
        // hard the compressor works (flow x pressure ratio)
        double f1 = shaftRpm / 60.0 * (turbo ? 1.6 : 4.0) * (turbo ? kTurboMaxRpm : kImpellerMaxRpm) / nMaxC_;
        double wob = 1.0 + 0.004 * wobble_.lp(noise * 40.0);
        phase_ += 2 * kPi * std::min(f1 * wob, 12000.0) * dt;
        phase2_ += 2 * kPi * std::min(f1 * 1.503 * wob, 16000.0) * dt;
        phase3_ += 2 * kPi * std::min(f1 * 0.997, 12000.0) * dt;     // slight beating
        double work = speedFrac * speedFrac * (0.15 + 0.85 * std::sqrt(flowFrac)) * (0.35 + 0.65 * prLoad);
        double aw = ampSmooth_.lp((turbo ? 2.4 : 7.0) * work);
        snd += whineLp_.lp(aw * (0.6 * std::sin(phase_) + 0.35 * std::sin(phase3_) + 0.18 * std::sin(phase2_)));
        // aerodynamic whoosh of the air rushing into the inducer
        whooshBp_.bandpass(1400 + 3200 * clampv(speedFrac, 0.0, 1.0), 1.0, fs_);
        snd += whooshBp_.tick(noise) * 2.4 * flowFrac * (0.3 + prLoad) * speedFrac;
    } else {
        // Roots / twin-screw: the classic blower whine is the rotor lobe-pass tone (Roots: 2 rotors x
        // 3 lobes = 6 pulses per turn; screw: 5 lobes, rounder) plus the high timing-gear mesh
        // whine. Both rise with blower speed and get louder (harsher) with boost.
        const bool roots = p_.scType == Supercharger::Roots;
        double lobes = roots ? 6.0 : 10.0;
        double gearTeeth = roots ? 34.0 : 52.0;
        double fl = shaftRpm / 60.0 * lobes, fg = shaftRpm / 60.0 * gearTeeth;
        double wob = 1.0 + 0.003 * wobble_.lp(noise * 40.0);
        phase_ += 2 * kPi * std::min(fl * wob, 9000.0) * dt;
        phase2_ += 2 * kPi * std::min(fg * wob, 14000.0) * dt;
        phase3_ += 2 * kPi * std::min(fg * 1.004, 14000.0) * dt;    // beating against the other gear
        double sp = clampv(shaftRpm / 12000.0, 0.0, 1.5);
        double press = bovOpen ? 0.12 : 0.35 + prLoad;               // bypass open = much quieter
        double a = ampSmooth_.lp(9.0 * sp * sp * press + 0.6 * sp);
        double s1 = std::sin(phase_);
        // lobe pulses: a peaky (pulse-like) waveform rich in harmonics
        double lobe = s1 + 0.55 * std::sin(2 * phase_) + 0.35 * std::sin(3 * phase_) + 0.2 * std::sin(4 * phase_);
        double gear = 0.6 * std::sin(phase2_) + 0.4 * std::sin(phase3_);
        snd += a * ((roots ? 0.55 : 0.35) * lobe + (roots ? 0.6 : 0.8) * gear);
        snd += scHiss_.tick(noise) * 2.0 * sp * prLoad;
    }

    // blow-off valve: valve pops open ("thump"), then the vented charge rushes out ("pssshh")
    if (bovOpen && !prevBov_ && p_.induction == Induction::Turbo && boostBar > 0.2) { thumpEnv_ = 1; thumpPh_ = 0; }
    prevBov_ = bovOpen;
    double ventFlow = std::max(0.0, vent);
    // a supercharger bypass just recirculates quietly; only real boost being dumped is loud
    double target = clampv(ventFlow / 0.06, 0.0, 2.0) * smoothstep(0.04, 0.3, boostBar + (bovEnv_ > 0.05 ? 0.3 : 0.0));
    bovEnv_ += (target - bovEnv_) * std::min(1.0, dt / (target > bovEnv_ ? 0.004 : 0.06));
    if (bovEnv_ > 1e-4) {
        double hiss = bovHiss_.tick(noise), body = bovBody_.tick(noise);
        // brighter at high pressure, duller as it empties
        double bright = clampv(boostBar / 1.0, 0.2, 1.2);
        snd += bovEnv_ * (2.8 * bright * hiss + 1.6 * body);
    } else { bovHiss_.tick(0); bovBody_.tick(0); }
    if (thumpEnv_ > 1e-3) {
        thumpPh_ += 2 * kPi * 70.0 * dt;
        snd += 1.8 * thumpEnv_ * std::sin(thumpPh_);
        thumpEnv_ *= std::exp(-dt / 0.03);
    }

    // compressor surge flutter ("stu-tu-tu"): one sharp chirp per stall cycle, pitch falling
    // as the wheel slows; louder at higher boost
    if (surging && centrifugal) {
        if (flutterPh_ < prevSurgePh_) { surgeEnv_ = 1.0; surgePitch_ = 1.0; }   // new stall cycle
        prevSurgePh_ = flutterPh_;
    }
    if (surgeEnv_ > 1e-3) {
        surgeBp_.bandpass(650 + 900 * clampv(speedFrac, 0.0, 1.0) * surgePitch_, 2.0, fs_);
        double chirp = surgeBp_.tick(noise) * 3.5 + surgeLow_.tick(noise) * 2.0;
        double lvl = clampv(0.35 + std::max(0.0, boostBar), 0.0, 2.0);
        snd += surgeEnv_ * lvl * chirp;
        surgeEnv_ *= std::exp(-dt / 0.014);
        surgePitch_ *= std::exp(-dt / 0.05);
    } else { surgeBp_.tick(0); surgeLow_.tick(0); }

    if (phase_ > 2 * kPi) phase_ = std::fmod(phase_, 2 * kPi);
    if (phase2_ > 2 * kPi) phase2_ = std::fmod(phase2_, 2 * kPi);
    if (phase3_ > 2 * kPi) phase3_ = std::fmod(phase3_, 2 * kPi);
    sound_ = snd;
    return crankTorque;
}

} // namespace es

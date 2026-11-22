#include "sim/Boost.h"
#include <cstring>

namespace es {

static constexpr double kPRmax = 3.6;          // centrifugal compressor PR at max speed, zero flow
static constexpr double kTurboMaxRpm = 190000; // turbo wheel speed at "100 %"
static constexpr double kImpellerMaxRpm = 60000;
static constexpr double kCpAir = 1005, kCpExh = 1150;

// Compression system dynamics (Greitzer, "Surge and rotating stall in axial flow compressors", J. Eng. Power 98, 1976;
// the lumped model Galindo et al. use for small automotive turbo compressors, Exp. Therm. Fluid Sci. 32, 2008): the
// air in the compressor and its duct has inertia, dm/dt = (A_c / L_c) (dp_compressor(m) - dp_plenum), and the charge
// pipes / intercooler are the plenum (the charge node). Left of the peak of a speed line the pressure rise falls with
// falling flow - the operating point there is unstable, and with a big enough plenum (Greitzer's B) the flow swings
// into reverse and back: deep surge, the "flutter" of a turbo whose throttle shuts on boost. Its frequency comes out of
// the plenum volume and the duct, not a set rate.
// The speed line left of its peak (Moore & Greitzer's cubic shape, as a smoothstep): the shut-off pressure rise at zero
// flow is kShut of the peak's (chosen), reverse flow is resisted quadratically, kBack x the peak rise at -peak flow
// (chosen). Duct: L_c 0.5 m (compressor + outlet duct to the charge pipes, chosen), area the inducer's (0.7 x the
// exducer diameter, typical), friction loss kDuctK dynamic heads (chosen).
static constexpr double kShut = 0.80, kBack = 1.0, kDuctL = 0.5, kDuctK = 0.5;
static constexpr double kSurgeFlowShare = 0.25;   // generic map: the surge point at a quarter of the line's pumping flow

// tip radius of the compressor wheel (exducer): a 500 m/s tip speed at the top speed (chosen - the same as the VGT
// turbine's assumption), so the acoustics have the wheel's real tip Mach number
static double tipRadius(double nMaxRpm) { return 500.0 / (std::max(1000.0, nMaxRpm) * 2.0 * kPi / 60.0); }

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
    tipR_ = tipRadius(nMaxC_);
    const double rInd = 0.7 * tipR_;
    ductA_ = kPi * rInd * rInd;
    for (Unit& u : u_) u.lineN = -1;   // (rebuild the speed lines)
    nU_ = turbo && p_.turboCount >= 2 ? 2 : 1;
    seq_ = nU_ == 2 && p_.twinMode == 1;
    units = nU_;
}

// The speed line at this speed as pressure ratio over flow (kg/s at the inlet state): the right branch from the map
// (generic: its closed form) as a table, the peak (surge point) and the left / reverse branch from the shape above.
// Rebuilt when the speed has moved by 0.2 % (a map: tens of evaluations once in a while, not per step).
void ForcedInduction::buildLine(Unit& u, double n, double rpmCorr, double corrM) {
    u.lineN = n;
    if (mapC_) {
        const CompPoint top = cm_.eval(rpmCorr, 99.0);   // (left of everything: the line's surge point)
        u.lineS = top.surgeFlow * corrM; u.linePrS = std::max(1.0, top.surgePr);
        for (int k = 0; k < kLineN; ++k) {
            const double prk = u.linePrS - (u.linePrS - 1.0) * k / (kLineN - 1.0);
            u.lineM[k] = std::max(cm_.eval(rpmCorr, prk).flow * corrM, k ? u.lineM[k - 1] + 1e-6 : u.lineS);
            u.linePr[k] = prk;
        }
    } else {
        const double mP = std::max(1e-4, p_.compFlow_gs * 1e-3) * (0.12 * smoothstep(0.0, 0.12, n) + 0.88 * n);
        const double pr0 = prAtZeroFlow(n);
        u.lineS = kSurgeFlowShare * mP;
        for (int k = 0; k < kLineN; ++k) {
            const double mk = u.lineS + (mP - u.lineS) * k / (kLineN - 1.0);
            u.lineM[k] = mk; u.linePr[k] = 1.0 + (pr0 - 1.0) * (1.0 - (mk / mP) * (mk / mP));
        }
        u.linePrS = u.linePr[0];
    }
}

double ForcedInduction::linePr(const Unit& u, double m) const {
    if (m >= u.lineS) {
        for (int k = 1; k < kLineN; ++k)
            if (m <= u.lineM[k]) {
                const double t = (m - u.lineM[k - 1]) / std::max(1e-9, u.lineM[k] - u.lineM[k - 1]);
                return u.linePr[k - 1] + (u.linePr[k] - u.linePr[k - 1]) * t;
            }
        // past the choke end: the pressure rise collapses
        const double mL = u.lineM[kLineN - 1];
        return u.linePr[kLineN - 1] - 4.0 * (m - mL) / std::max(1e-4, mL);
    }
    const double prShut = 1.0 + kShut * (u.linePrS - 1.0);
    const double x = m / std::max(1e-6, u.lineS);
    if (x >= 0) return prShut + (u.linePrS - prShut) * x * x * (3.0 - 2.0 * x);
    return prShut + kBack * (u.linePrS - 1.0) * x * x;
}

void ForcedInduction::init(const EngineParams& p, double fs) {
    p_ = p;
    fs_ = fs;
    ambT_ = 273.15 + p.ambientTemp_C;
    chargeT = ambT_;
    icCoreT = ambT_;
    refreshMaps();
    for (Unit& u : u_) {
        u = Unit();
        u.tcnBp.bandpass(8000, 6.0, fs);
        u.amp.lowpass(30, fs);
    }
    ecv_ = ebv_ = 0; icv_ = false; icvPos_ = 0; seqStage = 0;
    whooshBp_.bandpass(2500, 0.6, fs);   // (1-4 kHz)
    flutLp_.lowpass(2500, fs);          // (the inlet pulse through the air box)
    bovHiss_.bandpass(4800, 0.9, fs);
    bovBody_.bandpass(1300, 1.6, fs);
    surgeBp_.bandpass(1400, 0.9, fs);   // (stall noise of the reverse flow)
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

    double powerC = 0, T2 = ambT_;
    if (centrifugal) {
        const bool turboNow = p_.induction == Induction::Turbo;
        const int nC = turboNow ? nU_ : 1;
        const double sq = std::sqrt(kMapTref / ambT_), corrM = pIn / kMapPref * sq;
        // Sequential pair (Tashima et al., "Sequential twin turbocharged rotary engine of the latest RX-7", SAE 941030;
        // the 2JZ-GTE's valves: Toyota Supra 1997 repair manual): below the change-over speed the first turbo takes all
        // the exhaust; nearing it an exhaust bypass valve bleeds some to the second turbine to spin it up
        // (pre-spool); at the change-over the exhaust control valve opens to it, and the intake control valve joins
        // its compressor to the charge only once it has caught up - the turbine valve first, the compressor valve
        // after (Galindo, Climent, Guardiola, Domenech, Int. J. Automot. Technol. 10:141, 2009: opening the compressor
        // side early lets the charge blow back through the slow wheel - a torque dip). Change-over under load (pedal
        // past 40 %, back below 30 %), 300 rpm hysteresis, pre-spool from 1200 rpm below it, the bypass valve 0.08 s, the exhaust
        // control valve 0.15 s; the intake valve once the second is at 85 % of the first's speed (chosen)
        if (seq_) {
            const bool load = throttle > (ecv_ > 0.5 ? 0.3 : 0.4);
            const bool on = load && engineRpm > p_.seqRpm - (ecv_ > 0.5 ? 300.0 : 0.0);
            ecv_ += ((on ? 1.0 : 0.0) - ecv_) * std::min(1.0, dt / 0.15);
            // the bypass valve regulates the pre-spool: it holds the second turbo at ~92 % of the first's speed (an
            // unloaded wheel on an open bypass overspeeds, and then dumps its air into the charge all at once when its
            // compressor valve opens - an overshoot the wastegate answers with a dip; chosen)
            const bool pre = load && engineRpm > p_.seqRpm - 1200.0;
            const double ebvT = pre ? clampv((0.92 * u_[0].speedFrac - u_[1].speedFrac) / 0.08, 0.0, 1.0) : 0.0;
            ebv_ += (ebvT - ebv_) * std::min(1.0, dt / 0.08);
            if (!icv_ && ecv_ > 0.6 && u_[1].speedFrac >= 0.85 * u_[0].speedFrac) icv_ = true;
            if (icv_ && ecv_ < 0.5) icv_ = false;
            // its butterfly swings open over ~0.3 s (chosen): the second compressor's air joins the charge gradually
            // (switched at once, the two wheels' air arrived together - a boost spike the wastegate answered with a dip)
            icvPos_ += ((icv_ ? 1.0 : 0.0) - icvPos_) * std::min(1.0, dt / 0.3);
            seqStage = icvPos_ > 0.5 ? 2 : (ebv_ > 0.1 || ecv_ > 0.1) ? 1 : 0;
        } else seqStage = 0;
        double mT2 = 0, mPos = 0;
        dmDuct_ = 0;
        surgeMargin = 1;
        for (int k = 0; k < nC; ++k) {
            Unit& u = u_[k];
            u.rpm = turboNow ? u.omega * 60.0 / (2 * kPi) : engineRpm * std::max(1.0f, p_.scRatio);
            u.speedFrac = clampv(u.rpm / nMaxC_, 0.0, 1.3);
            // a sequential second compressor behind its shut intake control valve blows round its relief path at a
            // small pressure rise (chosen: 5 % of its line's - a relief loop, not a load) - none of it reaches the charge
            // (part open: its outlet between the relief loop and the charge, that share of its air to the charge)
            const double share = seq_ && k == 1 ? icvPos_ : 1.0;
            const bool shut = share < 0.02;
            if (u.lineN < 0 || std::fabs(u.speedFrac - u.lineN) > 0.002) buildLine(u, u.speedFrac, u.rpm * sq, corrM);
            const double dpOut = share * (charge.p() - pIn) + (1.0 - share) * 0.05 * pIn * (u.linePrS - 1.0);
            const double prU = 1.0 + dpOut / pIn;
            double sm = 1.0;
            if (mapC_) {
                // table map: corrected speed and flow (inlet at ambient pressure and temperature) - the efficiency and
                // the surge margin at this pressure ratio
                const CompPoint cp = cm_.eval(u.rpm * sq, prU);
                u.eta = cp.eff;
                sm = cp.surgePr > 1.0 ? 1.0 - (prU - 1.0) / (cp.surgePr - 1.0) : 1.0;
            } else {
                compressor(u.speedFrac, prU, u.eta);
                const double pr0 = prAtZeroFlow(u.speedFrac);
                sm = pr0 > 1.0 ? 1.0 - (prU - 1.0) / (pr0 - 1.0) : 1.0;
            }
            if (!shut) surgeMargin = std::min(surgeMargin, sm);
            // the flow is a state with inertia (Greitzer): accelerated by the line's pressure rise against the outlet
            // pressure, less the duct's friction
            if (u.speedFrac < 0.06) {
                u.mDuct *= std::exp(-dt / 0.01);   // (barely turning: the passive path below carries the flow)
            } else {
                const double dpLine = pIn * (linePr(u, u.mDuct) - 1.0);
                const double dpLoss = kDuctK * u.mDuct * std::fabs(u.mDuct) / (2.0 * rhoIn * ductA_ * ductA_);
                const double accel = ductA_ / kDuctL * (dpLine - dpOut - dpLoss);
                u.mDuct += accel * dt;
                u.mDuct = clampv(u.mDuct, -2.0 * sizeKgs_, 2.0 * sizeKgs_);
            }
            u.dmDuct = (u.mDuct - u.mPrev) / dt;
            u.mPrev = u.mDuct;
            const double tauU = std::pow(std::max(prU, 0.5), 0.2857) - 1.0;
            u.powerC = std::max(0.0, u.mDuct) * kCpAir * ambT_ * std::max(0.0, tauU) / std::max(0.3, u.eta);
            if (!shut) {
                m += share * u.mDuct;
                dmDuct_ += share * u.dmDuct;
                mT2 += share * std::max(0.0, u.mDuct) * ambT_ * (1.0 + std::max(0.0, tauU) / std::max(0.3, u.eta));
                mPos += share * std::max(0.0, u.mDuct);
                powerC += u.powerC;
                // left of the line's peak the operating point is unstable: surge (deep when the flow reverses)
                surging = surging || (u.speedFrac > 0.15 && u.mDuct < u.lineS);
            }
        }
        eta = u_[0].eta;
        shaftRpm = u_[0].rpm; speedFrac = u_[0].speedFrac;
        shaftRpm2 = nC > 1 ? u_[1].rpm : 0.0; speedFrac2 = nC > 1 ? u_[1].speedFrac : 0.0;
        lineS = u_[0].lineS;
        const double tau = std::pow(std::max(pr, 0.5), 0.2857) - 1.0;
        T2 = mPos > 1e-5 ? mT2 / mPos : ambT_ * (1.0 + std::max(0.0, tau) / std::max(0.3, eta));
    } else {
        // positive displacement blower: pumps a fixed volume per revolution, minus leakage
        double nsc = engineRpm * std::max(0.1f, p_.scRatio);
        shaftRpm = nsc;
        speedFrac = clampv(nsc / 16000.0, 0.0, 1.3);
        double leak = p_.scType == Supercharger::Roots ? 0.20 : 0.10;
        double mIdeal = rhoIn * std::max(0.05f, p_.scDisplacement_L) * 1e-3 * nsc / 60.0;
        m = mIdeal * (1.0 - leak * std::max(0.0, pr - 1.0)) - 0.002 * std::max(0.0, pr - 1.0);
        eta = p_.scType == Supercharger::Roots ? 0.52 : 0.70;
        const double tau = std::pow(std::max(pr, 0.5), 0.2857) - 1.0;
        T2 = ambT_ * (1.0 + std::max(0.0, tau) / std::max(0.3, eta));
        powerC = std::max(0.0, m) * kCpAir * ambT_ * std::max(0.0, tau) / std::max(0.3, eta);
    }

    // intercooler: effectiveness against its core temperature; the core takes the heat and gives it to the air
    // blown through it (vehicle speed / dyno fan) - parked or on a dyno without a fan it heat-soaks
    compOutT = T2;
    double Tcharge = T2;
    if (p_.intercooler) {
        const double eps = clampv((double)p_.icEfficiency, 0.0, 1.0);
        Tcharge = T2 - eps * (T2 - icCoreT);
        const double size = std::max(0.4, nU_ * sizeKgs_ / 0.25);
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
        double aPass = 0;
        const int nC = p_.induction == Induction::Turbo ? nU_ : 1;
        for (int k = 0; k < nC; ++k) {
            if (seq_ && k == 1 && icvPos_ < 0.02) continue;
            const double slow = 1.0 - clampv(u_[k].speedFrac, 0.0, 1.0);
            aPass += 1.2 * std::max(0.05, sizeKgs_) / 236.0 * slow * slow;
        }
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
    // turbo blow-off valve: a spring-loaded piston with the charge pressure under it and the manifold pressure on top
    // (its reference line) - it lifts when the throttle shuts on boost. A spring-mass-damper (chosen: ~45 Hz, light
    // damping 0.08, full lift 0.15 bar past the cracking pressure); it bounces off its seat and its stop. Venting drops
    // the charge pressure, so a spring too stiff for the boost leaves it hunting half open - the fluttery valve sound.
    double bovArea = 0;
    if (p_.induction == Induction::Turbo && p_.bov) {
        const double dpBar = (charge.p() - in.map()) / 1e5;
        const double drive = (dpBar - clampv((double)p_.bovSpringBar, 0.05, 1.0)) / 0.15;
        const double w = 2.0 * kPi * 45.0, z = 0.08;
        bovV_ += (w * w * (drive - bovX_) - 2.0 * z * w * bovV_) * dt;
        bovX_ += bovV_ * dt;
        if (bovX_ < 0) { bovX_ = 0; bovV_ = -0.3 * std::min(0.0, bovV_); }
        if (bovX_ > 1) {   // hits its stop: the "thump"
            if (bovV_ > 0) thumpEnv_ = std::max(thumpEnv_, clampv(bovV_ / (0.4 * w), 0.0, 1.5));
            bovX_ = 1; bovV_ = -0.3 * std::max(0.0, bovV_);
        }
        bovLift = bovX_;
        if (bovX_ > 0.01) { bovOpen = true; bovArea = bovX_ * 3.5e-4; }
    } else { bovX_ = bovV_ = 0; bovLift = 0; }
    if (p_.induction == Induction::Supercharger && p_.scBypass && throttle < 0.15) { bovOpen = true; bovArea = 1.5e-3; }
    if (bovOpen) {
        double area = bovArea;
        vent = limitedFlow(area, charge.p(), charge.T, charge.V, kPatm, ambT_, 1e3, dt, 0.4);
        if (vent > 0) {
            double qv = std::min(vent * dt, charge.m * 0.3);
            ventFlow = qv / dt;
            double ff = charge.fuelFrac(), bf = charge.burnedFrac();
            charge.m -= qv; charge.mf -= qv * ff; charge.mb -= qv * bf;
        }
    }

    if (p_.induction == Induction::Turbo && ex.hasTurbine()) {
        // ---- turbine(s) + wastegate(s) between the manifold and the downpipe. Two turbos: each half the area one would
        // have for the same A/R (chosen: two turbos for one engine); a twin-scroll housing takes its two scrolls as two
        // inlets of half the area each
        const double aT1 = std::max(0.2f, p_.turbineAR) * 1.0e-3 * std::sqrt(std::max(0.3, equivalentDisplacementL(p_) / 2.0)) / nU_;
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
        double fv = 1.0;
        if (p_.vgt) {
            // variable geometry: no wastegate; the guide vanes set the turbine's flow area. The ECU duty closes them
            // (no duty / no ECU: open). Area over the vane opening u as the ellipse arc of Wahlstrom & Eriksson
            // (LiTH-ISY-R-2882 eq. 53-55): f = c_f2 + c_f1 sqrt(1 - ((u - c_vgt2) / c_vgt1)^2), here centred on the
            // open end (c_vgt1 = c_vgt2 = 1) and running from vgtMinArea (closed) to 1 (open) - their fitted
            // constants belong to their engine. Actuator: first order, 0.15 s (chosen).
            const double uv = 1.0 - clampv(wgDuty_ >= 0.0 ? wgDuty_ : 0.0, 0.0, 1.0);
            vanePos_ += (uv - vanePos_) * std::min(1.0, dt / 0.15);
            const double amin = clampv((double)p_.vgtMinArea, 0.05, 1.0);
            fv = amin + (1.0 - amin) * std::sqrt(std::max(0.0, 1.0 - (vanePos_ - 1.0) * (vanePos_ - 1.0)));
            wgPos_ = 0;
            wastegate = vanePos_;   // (telemetry: the vane opening)
        }
        GasNode* scream = ex.screamer();
        auto move = [&](GasNode& src, GasNode& dst, double flow) {   // (signed kg/s; returns what moved, signed)
            const double qf = flow * dt;
            GasNode& s = qf >= 0 ? src : dst;
            GasNode& d = qf >= 0 ? dst : src;
            const double qa = std::min(std::fabs(qf), s.m * 0.3);
            const double ff = s.fuelFrac(), bf = s.burnedFrac();
            s.m -= qa; s.mf -= qa * ff; s.mb -= qa * bf;
            d.m += qa; d.mf += qa * ff; d.mb += qa * bf;
            return (qf >= 0 ? qa : -qa) / dt;
        };
        turbFlow = wgFlow = 0;
        double powSum = 0, flowSum = 0;
        for (int k = 0; k < nU_; ++k) {
            Unit& u = u_[k];
            // the paths into this turbine: (inlet node, outlet node, share of its area)
            struct Path { GasNode* in; GasNode* out; double share; };
            Path paths[2];
            int np = 0;
            if (nU_ == 1) {
                const int ni = ex.turbineInCount();
                for (int i = 0; i < ni; ++i) paths[np++] = {&ex.turbineIn(i), &ex.turbineOut(0), 1.0 / ni};
            } else if (!seq_) {
                paths[np++] = {&ex.turbineIn(std::min(k, ex.turbineInCount() - 1)), &ex.turbineOut(std::min(k, ex.turbineOutCount() - 1)), 1.0};
            } else {   // sequential: one manifold; the second turbine behind its exhaust control valve (and the pre-spool
                       // bypass: 25 % of its area, chosen - enough to bring it near the first turbo before it takes the exhaust)
                paths[np++] = {&ex.turbineIn(0), &ex.turbineOut(0), k == 0 ? 1.0 : std::max(ecv_, 0.25 * ebv_)};
            }
            double powerT = 0;
            for (int j = 0; j < np; ++j) {
                GasNode& tin = *paths[j].in;
                GasNode& tout = *paths[j].out;
                double aT = aT1;
                const double erNow = std::max(1.0, tin.p() / std::max(1.0, tout.p()));
                if (mapT_) {
                    // table map: the reduced flow at this expansion ratio as an equivalent nozzle area
                    const double er = std::max(erNow, 1.02);
                    const double mMap = tm_.mfp(er) * tin.p() * 1e-3 / std::sqrt(std::max(300.0, tin.T));
                    const double unit = orificeFlow(1.0, tin.p(), tin.T, tin.p() / er);
                    if (unit > 1e-12) aT = std::max(1e-6, mMap / unit) / nU_;
                }
                aT *= paths[j].share * fv;
                // a wide-open wastegate bypasses most of the exhaust (real ports pass ~2-3x the turbine's flow area);
                // a sequential second turbo has its own only once its exhaust valve is open
                double aWg = p_.vgt ? 0.0 : wgPos_ * aT1 * 2.5 / np * (seq_ && k == 1 ? ecv_ : 1.0);
                double flowT, flowW;
                if (scream) {   // the wastegate to its own pipe: two flows
                    flowT = move(tin, tout, limitedFlow(aT, tin.p(), tin.T, tin.V, tout.p(), tout.T, tout.V, dt, 0.35));
                    flowW = aWg > 0 ? move(tin, *scream, limitedFlow(aWg, tin.p(), tin.T, tin.V, scream->p(), scream->T, scream->V, dt, 0.35)) : 0.0;
                } else {
                    const double q = aT + aWg > 0 ? move(tin, tout, limitedFlow(aT + aWg, tin.p(), tin.T, tin.V, tout.p(), tout.T, tout.V, dt, 0.35)) : 0.0;
                    flowT = aT + aWg > 0 ? q * aT / (aT + aWg) : 0.0;
                    flowW = q - flowT;
                }
                turbFlow += flowT; wgFlow += flowW;
                const double mt = std::max(0.0, flowT);
                const double Tin = std::max(600.0, egtK);
                const double dh = kCpExh * Tin * (1.0 - std::pow(erNow, -0.248));
                double etaT = 0.68;
                if (mapT_) {
                    const double U = u.omega * tm_.wheelDia() * 0.5e-3;
                    const double Cs = std::sqrt(2.0 * std::max(0.0, dh));
                    turbUc = Cs > 1.0 ? U / Cs : 0.0;
                    etaT = Cs > 1.0 ? tm_.eff(turbUc) : 0.3;
                } else if (p_.vgt) {
                    // generic VGT turbine (D5; the wastegated generic turbines keep the constant 0.68 their calibrations
                    // were made with): efficiency over the blade speed ratio, Wahlstrom & Eriksson (LiTH-ISY-R-2882 eqs.
                    // 48-49): eta = eta_max - c_m (BSR - BSR_opt)^2, BSR = R_t w_t / sqrt(2 cp T (1 - PR^(1-1/gamma))).
                    // Peak 0.68, BSR_opt 0.58 (inside the 0.52-0.64 span of their fig. 9), c_m 2.5 (chosen); tip radius
                    // from a 500 m/s tip speed at the top shaft speed (chosen)
                    const double rt = 500.0 / (nMaxC_ * 2.0 * kPi / 60.0);
                    const double Cs = std::sqrt(2.0 * std::max(0.0, dh));
                    turbUc = Cs > 1.0 ? u.omega * rt / Cs : 0.0;
                    etaT = Cs > 1.0 ? clampv(0.68 - 2.5 * (turbUc - 0.58) * (turbUc - 0.58), 0.2, 0.68) : 0.3;
                } else turbUc = 0;
                if (k == 0 && j == 0) {
                    turbEff = etaT; turbEr = erNow;
                    turbMfp = mt * std::sqrt(std::max(300.0, tin.T)) / std::max(1.0, tin.p() * 1e-3);
                }
                powerT += mt * dh * etaT;
                flowSum += std::max(0.0, flowT + flowW);
            }
            powSum += powerT;
            // ---- shaft
            const double J = std::max(0.02f, p_.turboInertia) * 1e-4;
            const double w = std::max(u.omega, 400.0);
            const double fric = 1.5e-6 * u.omega + 2e-4;
            u.omega += ((powerT - u.powerC) / w - fric) / J * dt;
            u.omega = clampv(u.omega, 0.0, nMaxC_ * 1.25 * 2 * kPi / 60.0);
        }
        // the turbines take this work out of the gas; the wastegate flow passes without expanding through them
        {   // (averaged power over averaged flow: the pulsating instantaneous ratio would be meaningless)
            const double k = std::min(1.0, dt / 0.08);
            turbPowAvg_ += (powSum - turbPowAvg_) * k;
            turbFlowAvg_ += (flowSum - turbFlowAvg_) * k;
            turbineDropK = turbPowAvg_ / (std::max(1e-3, turbFlowAvg_) * kCpExh);
        }
        powerKw = powerC / 1000.0;
    } else if (p_.induction == Induction::Supercharger) {
        double drag = 150.0 + 4e-3 * shaftRpm * shaftRpm * 1e-3;   // bearing / gear losses (W)
        double pw = (bovOpen ? powerC * 0.25 : powerC) + drag;
        powerKw = pw / 1000.0;
        crankTorque = pw / std::max(30.0, omegaE);
        wastegate = 0;
    }

    // ---- sound (Pa @ 1 m), from the physical state: shaft speed, flow, pressure ratio, the surge cycles, vent flow
    double snd = 0;
    const double flowFrac = clampv(std::max(0.0, m) / std::max(1e-4, sizeKgs_), 0.0, 1.2);
    const double prLoad = clampv((pr - 1.0) / 1.2, 0.0, 1.5);
    const double noise = rng_.bipolar();
    if (centrifugal) {
        // Centrifugal compressor noise sources (Raitor & Neise, JSV 2008; the review in MDPI Machines 13:948, 2025):
        //  - blade-pass tone at blades x shaft frequency: tens of kHz at full speed (12 x 2.9 kHz) - audible only while
        //    spooling, so it fades out above ~15 kHz
        //  - shaft order 1 (the synchronous tone: residual imbalance - a force growing with speed^2 - and blade-to-blade
        //    differences, SAE 2017-01-1051): the audible "turbo whistle", 1-3 kHz
        //  - buzz-saw: shaft orders together once the inducer tip is near / above sonic (shock waves at the blade tips)
        //  - tip-clearance noise: a narrow band about half the blade-pass frequency
        //  - whoosh: broadband 1-4 kHz near the surge line, on tip-in and on throttle closure (SAE 2009-01-2053,
        //    2005-01-2485, 2020-01-5090)
        //  - surge: each flow reversal pushes a pressure pulse out of the compressor inlet (a monopole, p = (dm/dt) /
        //    4 pi r) with stall noise as the flow churns back through the wheel - the "stu-tu-tu"
        // Levels are chosen (the old whine's scale); what sets pitch and rhythm is the physics.
        const bool turbo = p_.induction == Induction::Turbo;
        const int nC = turbo ? nU_ : 1;
        const double aIn = std::sqrt(1.4 * kR * ambT_);
        const double Z = clampv((double)p_.compBlades, 4.0, 24.0);
        double tone = 0;
        for (int k = 0; k < nC; ++k) {
            Unit& u = u_[k];
            const double fSh = u.rpm / 60.0;
            const bool shut = seq_ && k == 1 && icvPos_ < 0.5;
            const double ff = clampv(std::max(0.0, u.mDuct) / std::max(1e-4, sizeKgs_), 0.0, 1.2);
            const double mRel = 0.7 * tipR_ * u.rpm * 2.0 * kPi / 60.0 / aIn;   // inducer tip Mach (tangential part)
            const double fwd = smoothstep(-0.2, 0.8, u.mDuct / std::max(1e-5, u.lineS));   // tones break up in reverse flow
            const double load = shut ? 0.1 : prLoad;
            const double work = u.speedFrac * u.speedFrac * (0.15 + 0.85 * std::sqrt(ff)) * (0.35 + 0.65 * load);
            const double aw = u.amp.lp((turbo ? 2.4 : 7.0) * work * fwd);
            // (a second wheel: its own slow wander - two turbos never run quite together, they beat)
            const double wob = 1.0 + 0.003 * wobble_.lp(noise * 40.0) * (k ? -1.3 : 1.0);
            u.ph1 = std::fmod(u.ph1 + 2 * kPi * fSh * wob * dt, 2 * kPi);
            tone += (0.55 * aw + 0.25 * u.speedFrac * u.speedFrac * (turbo ? 1.0 : 2.0)) * std::sin(u.ph1);   // shaft order 1
            tone += 0.12 * aw * std::sin(2.0 * u.ph1 + 0.7);                                                // and its 2nd
            const double fBpf = Z * fSh;
            u.phB = std::fmod(u.phB + 2 * kPi * std::min(fBpf, 20000.0) * dt, 2 * kPi);
            tone += 0.45 * aw * (1.0 - smoothstep(13000.0, 17000.0, fBpf)) * std::sin(u.phB);              // blade pass
            const double buzz = smoothstep(0.85, 1.05, mRel);
            if (buzz > 1e-3) {   // shaft orders 2..Z under the Nyquist / 14 kHz, uneven like a real wheel's
                static const double kW[24] = {0.0, 0.0, 0.62, 0.48, 0.71, 0.33, 0.55, 0.29, 0.41, 0.36, 0.22, 0.30, 0.18, 0.26, 0.15,
                                              0.21, 0.12, 0.17, 0.10, 0.14, 0.09, 0.11, 0.08, 0.07};
                double b2 = 0;
                for (int h = 2; h <= (int)Z && h * fSh < 14000.0; ++h) b2 += kW[h] * std::sin(h * u.ph1 + 0.37 * h * h + 1.1 * k);
                tone += 1.2 * aw * buzz * b2;
            }
            const double fTcn = 0.5 * fBpf;
            if (fTcn > 200.0 && fTcn < 16000.0) {
                u.tcnBp.bandpass(fTcn, 6.0, fs_);
                snd += 1.0 * aw * u.tcnBp.tick(noise);
            } else u.tcnBp.tick(0);
        }
        snd += whineLp_.lp(tone);
        // whoosh: near the surge line (margin < ~0.35) with pressure ratio
        const double nearSurge = smoothstep(0.35, -0.05, surgeMargin);
        snd += whooshBp_.tick(noise) * (2.4 * flowFrac * (0.3 + prLoad) * speedFrac + 3.0 * nearSurge * (pr - 1.0) * speedFrac);
        // surge: the inlet pulse and the stall noise of the reverse flow
        const double pulse = flutLp_.lp(dmDuct_ / (4.0 * kPi));
        double rev = 0;
        for (int k = 0; k < nC; ++k) if (!(seq_ && k == 1 && icvPos_ < 0.5)) rev = std::max(rev, clampv(-u_[k].mDuct / std::max(1e-5, u_[k].lineS), 0.0, 1.5));
        const double stall = surgeBp_.tick(noise) * 3.5 * rev * (0.3 + prLoad) * std::max(0.2, speedFrac);
        snd += surging ? pulse + stall : 0.3 * pulse + stall;
    } else {        // Roots / twin-screw: the classic blower whine is the rotor lobe-pass tone (Roots: 2 rotors x
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

    // blow-off valve: the vented charge as a jet ("pssshh"). Jet noise peaks near a Strouhal number of 0.2 on the jet
    // speed and the opening's size (f = 0.2 U / D: a smaller opening or a faster jet hisses higher); the jet speed from
    // the pressure ratio (isentropic, sonic past ~1.9). A recirculating valve sends it back into the compressor inlet,
    // inside the ducting: little of it is heard (x 0.08, chosen). The thump: the valve hitting its stop (above).
    prevBov_ = bovOpen;
    double ventFlow = std::max(0.0, vent);
    const bool recirc = p_.induction == Induction::Turbo && p_.bovType == 1;
    double target = clampv(ventFlow / 0.06, 0.0, 2.0) * smoothstep(0.04, 0.3, boostBar + (bovEnv_ > 0.05 ? 0.3 : 0.0)) *
                    (recirc ? 0.08 : (p_.induction == Induction::Supercharger ? 0.4 : 1.0));
    bovEnv_ += (target - bovEnv_) * std::min(1.0, dt / (target > bovEnv_ ? 0.002 : 0.03));
    if (bovEnv_ > 1e-4) {
        const double prv = std::max(1.0, charge.p() / kPatm);
        const double uj = prv > 1.89 ? std::sqrt(2.0 * 1.4 / 2.4 * kR * chargeT)
                                     : std::sqrt(2.0 * kCpAir * chargeT * std::max(0.0, 1.0 - std::pow(prv, -0.2857)));
        const double dv = std::sqrt(4.0 * std::max(1e-6, bovArea) / kPi);
        bovHiss_.bandpass(clampv(0.2 * uj / dv, 900.0, 9000.0), 0.8, fs_);
        const double hiss = bovHiss_.tick(noise), body = bovBody_.tick(noise);
        const double bright = clampv(uj / 300.0, 0.2, 1.2);
        snd += bovEnv_ * (2.8 * bright * hiss + 1.2 * body);
    } else { bovHiss_.tick(0); bovBody_.tick(0); }
    if (thumpEnv_ > 1e-3) {   // (the valve's thump: a damped knock of the valve body, ~70 Hz)
        thumpPh_ += 2 * kPi * 70.0 * dt;
        snd += 1.8 * thumpEnv_ * std::sin(thumpPh_);
        thumpEnv_ *= std::exp(-dt / 0.03);
    }

    if (phase_ > 2 * kPi) phase_ = std::fmod(phase_, 2 * kPi);
    if (phase2_ > 2 * kPi) phase2_ = std::fmod(phase2_, 2 * kPi);
    if (phase3_ > 2 * kPi) phase3_ = std::fmod(phase3_, 2 * kPi);
    sound_ = snd;
    return crankTorque;
}

} // namespace es

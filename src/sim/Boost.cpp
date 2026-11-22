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
    lineN_ = -1;   // (rebuild the speed line)
}

// The speed line at this speed as pressure ratio over flow (kg/s at the inlet state): the right branch from the map
// (generic: its closed form) as a table, the peak (surge point) and the left / reverse branch from the shape above.
// Rebuilt when the speed has moved by 0.2 % (a map: tens of evaluations once in a while, not per step).
void ForcedInduction::buildLine(double n, double rpmCorr, double corrM) {
    lineN_ = n;
    if (mapC_) {
        const CompPoint top = cm_.eval(rpmCorr, 99.0);   // (left of everything: the line's surge point)
        lineS_ = top.surgeFlow * corrM; linePrS_ = std::max(1.0, top.surgePr);
        for (int k = 0; k < kLineN; ++k) {
            const double prk = linePrS_ - (linePrS_ - 1.0) * k / (kLineN - 1.0);
            lineM_[k] = std::max(cm_.eval(rpmCorr, prk).flow * corrM, k ? lineM_[k - 1] + 1e-6 : lineS_);
            linePr_[k] = prk;
        }
    } else {
        const double mP = std::max(1e-4, p_.compFlow_gs * 1e-3) * (0.12 * smoothstep(0.0, 0.12, n) + 0.88 * n);
        const double pr0 = prAtZeroFlow(n);
        lineS_ = kSurgeFlowShare * mP;
        for (int k = 0; k < kLineN; ++k) {
            const double mk = lineS_ + (mP - lineS_) * k / (kLineN - 1.0);
            lineM_[k] = mk; linePr_[k] = 1.0 + (pr0 - 1.0) * (1.0 - (mk / mP) * (mk / mP));
        }
        linePrS_ = linePr_[0];
    }
}

double ForcedInduction::linePr(double m) const {
    if (m >= lineS_) {
        for (int k = 1; k < kLineN; ++k)
            if (m <= lineM_[k]) {
                const double t = (m - lineM_[k - 1]) / std::max(1e-9, lineM_[k] - lineM_[k - 1]);
                return linePr_[k - 1] + (linePr_[k] - linePr_[k - 1]) * t;
            }
        // past the choke end: the pressure rise collapses
        const double mL = lineM_[kLineN - 1];
        return linePr_[kLineN - 1] - 4.0 * (m - mL) / std::max(1e-4, mL);
    }
    const double prShut = 1.0 + kShut * (linePrS_ - 1.0);
    const double u = m / std::max(1e-6, lineS_);
    if (u >= 0) return prShut + (linePrS_ - prShut) * u * u * (3.0 - 2.0 * u);
    return prShut + kBack * (linePrS_ - 1.0) * u * u;
}

void ForcedInduction::init(const EngineParams& p, double fs) {
    p_ = p;
    fs_ = fs;
    ambT_ = 273.15 + p.ambientTemp_C;
    chargeT = ambT_;
    icCoreT = ambT_;
    refreshMaps();
    omegaT_ = 0;
    whooshBp_.bandpass(2500, 0.6, fs);   // (1-4 kHz)
    tcnBp_.bandpass(8000, 6.0, fs);
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

    if (centrifugal) {
        double rpm = p_.induction == Induction::Turbo ? omegaT_ * 60.0 / (2 * kPi)
                                                      : engineRpm * std::max(1.0f, p_.scRatio);
        speedFrac = clampv(rpm / nMaxC_, 0.0, 1.3);
        shaftRpm = rpm;
        double chokeNow;
        const double sq = std::sqrt(kMapTref / ambT_), corrM = pIn / kMapPref * sq;
        if (mapC_) {
            // table map: corrected speed and flow (inlet at ambient pressure and temperature) - the efficiency and the
            // surge margin at this pressure ratio
            const CompPoint cp = cm_.eval(rpm * sq, pr);
            chokeNow = cp.chokeFlow * corrM;
            eta = cp.eff;
            surgeMargin = cp.surgePr > 1.0 ? 1.0 - (pr - 1.0) / (cp.surgePr - 1.0) : 1.0;
        } else {
            compressor(speedFrac, pr, eta);
            chokeNow = chokeFlow(speedFrac);
            const double pr0 = prAtZeroFlow(speedFrac);
            surgeMargin = pr0 > 1.0 ? 1.0 - (pr - 1.0) / (pr0 - 1.0) : 1.0;
        }
        (void)chokeNow;
        // the flow is a state with inertia (Greitzer): accelerated by the line's pressure rise against the charge
        // pressure, less the duct's friction
        if (lineN_ < 0 || std::fabs(speedFrac - lineN_) > 0.002) buildLine(speedFrac, rpm * sq, corrM);
        if (speedFrac < 0.06) {
            mDuct_ *= std::exp(-dt / 0.01);   // (barely turning: the passive path below carries the flow)
        } else {
            const double dpLine = pIn * (linePr(mDuct_) - 1.0);
            const double dpLoss = kDuctK * mDuct_ * std::fabs(mDuct_) / (2.0 * rhoIn * ductA_ * ductA_);
            const double accel = ductA_ / kDuctL * (dpLine - (charge.p() - pIn) - dpLoss);
            mDuct_ += accel * dt;
            mDuct_ = clampv(mDuct_, -2.0 * sizeKgs_, 2.0 * sizeKgs_);
        }
        dmDuct_ = (mDuct_ - mPrev_) / dt;
        mPrev_ = mDuct_;
        m = mDuct_;
        lineS = lineS_;
        // left of the line's peak the operating point is unstable: surge (deep when the flow reverses)
        surging = speedFrac > 0.15 && mDuct_ < lineS_;
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
        } else if (p_.vgt) {
            // generic VGT turbine (D5; the wastegated generic turbines keep the constant 0.68 their calibrations were made with): efficiency over the blade speed ratio, Wahlstrom & Eriksson (LiTH-ISY-R-2882 eqs.
            // 48-49): eta = eta_max - c_m (BSR - BSR_opt)^2, BSR = R_t w_t / sqrt(2 cp T (1 - PR^(1-1/gamma))) - the
            // blade tip speed over the isentropic expansion speed. (Their c_m also rises with the turbo speed, eq. 50,
            // and their fitted constants belong to their engine; there is no vane-position term - a VGT changes the
            // efficiency only through the expansion ratio its vanes make.) Peak 0.68 (the old constant efficiency),
            // BSR_opt 0.58 (inside the 0.52-0.64 span of their fig. 9), c_m 2.5 (chosen: -0.1 at 0.2 off the optimum);
            // tip radius from a 500 m/s tip speed at the top shaft speed (chosen)
            const double rt = 500.0 / (nMaxC_ * 2.0 * kPi / 60.0);
            const double Cs = std::sqrt(2.0 * std::max(0.0, dh));
            turbUc = Cs > 1.0 ? omegaT_ * rt / Cs : 0.0;
            etaT = Cs > 1.0 ? clampv(0.68 - 2.5 * (turbUc - 0.58) * (turbUc - 0.58), 0.2, 0.68) : 0.3;
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
        const double fSh = shaftRpm / 60.0;
        const double aIn = std::sqrt(1.4 * kR * ambT_);
        const double mRel = 0.7 * tipR_ * shaftRpm * 2.0 * kPi / 60.0 / aIn;   // inducer tip Mach (tangential part)
        const double fwd = smoothstep(-0.2, 0.8, mDuct_ / std::max(1e-5, lineS_));   // tones break up in reverse flow
        const double work = speedFrac * speedFrac * (0.15 + 0.85 * std::sqrt(flowFrac)) * (0.35 + 0.65 * prLoad);
        const double aw = ampSmooth_.lp((turbo ? 2.4 : 7.0) * work * fwd);
        const double Z = clampv((double)p_.compBlades, 4.0, 24.0);
        const double wob = 1.0 + 0.003 * wobble_.lp(noise * 40.0);
        double tone = 0;
        ph1_ = std::fmod(ph1_ + 2 * kPi * fSh * wob * dt, 2 * kPi);
        tone += (0.55 * aw + 0.25 * speedFrac * speedFrac * (turbo ? 1.0 : 2.0)) * std::sin(ph1_);   // shaft order 1
        tone += 0.12 * aw * std::sin(2.0 * ph1_ + 0.7);                                            // and its 2nd
        const double fBpf = Z * fSh;
        phB_ = std::fmod(phB_ + 2 * kPi * std::min(fBpf, 20000.0) * dt, 2 * kPi);
        tone += 0.45 * aw * (1.0 - smoothstep(13000.0, 17000.0, fBpf)) * std::sin(phB_);              // blade pass
        const double buzz = smoothstep(0.85, 1.05, mRel);
        if (buzz > 1e-3) {   // shaft orders 2..Z under the Nyquist / 14 kHz, uneven like a real wheel's
            static const double kW[24] = {0.0, 0.0, 0.62, 0.48, 0.71, 0.33, 0.55, 0.29, 0.41, 0.36, 0.22, 0.30, 0.18, 0.26, 0.15,
                                          0.21, 0.12, 0.17, 0.10, 0.14, 0.09, 0.11, 0.08, 0.07};
            double b2 = 0;
            for (int k = 2; k <= (int)Z && k * fSh < 14000.0; ++k) b2 += kW[k] * std::sin(k * ph1_ + 0.37 * k * k);
            tone += 1.2 * aw * buzz * b2;
        }
        snd += whineLp_.lp(tone);
        const double fTcn = 0.5 * fBpf;
        if (fTcn > 200.0 && fTcn < 16000.0) {
            tcnBp_.bandpass(fTcn, 6.0, fs_);
            snd += 1.0 * aw * tcnBp_.tick(noise);
        } else tcnBp_.tick(0);
        // whoosh: near the surge line (margin < ~0.35) with pressure ratio
        const double nearSurge = smoothstep(0.35, -0.05, surgeMargin);
        snd += whooshBp_.tick(noise) * (2.4 * flowFrac * (0.3 + prLoad) * speedFrac + 3.0 * nearSurge * (pr - 1.0) * speedFrac);
        // surge: the inlet pulse and the stall noise of the reverse flow
        const double pulse = flutLp_.lp(dmDuct_ / (4.0 * kPi));
        const double rev = clampv(-mDuct_ / std::max(1e-5, lineS_), 0.0, 1.5);
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

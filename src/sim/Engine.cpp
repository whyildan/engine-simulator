#include "sim/Engine.h"
#include <cstdlib>
#include <cstdio>
#include <algorithm>

namespace es {

static inline bool crossed(double prev, double cur, double target) {
    // angles in [0,720), motion forward by less than 360 deg per step
    if (cur >= prev) return prev < target && target <= cur;
    return target > prev || target <= cur; // wrapped through 720 -> 0
}

Engine::Engine(const EngineParams& p, double fs) : p_(p), fs_(fs) {
    ambT_ = 273.15 + p.ambientTemp_C;
    geom_.set(p_);
    cycle_ = cycleDeg(p_);
    const int n = clampv(p_.cylinders, 1, geom_.rotary ? 4 : kMaxCyl);
    p_.cylinders = nPorts_ = n;
    computePhases(p_, phase_);
    updateValves();
    comp_.build(p_);
    repairSeen_ = p_.comp.repairCount;
    cyls_.assign(geom_.rotary ? 3 * n : n, Cylinder());
    setupChambers();
    for (Cylinder& c : cyls_) c.init(geom_, wrapPos(theta_ - c.phase, cycle_), ambT_);
    intake.build(p_, fs, geom_.Vd * n);
    exhaust.build(p_, phase_, fs);
    boost.init(p_, fs);
    thermal.init(p_);
    elec.volts = 12.6;
    rail.pressureBar = 0.5;
    sensors.reset(p_);
    ecu.init(p_, geom_.Vd, n, geom_.rotary ? 1.0 : 2.0);
    {
        // mode frequencies scale inversely with size (reference: 86 mm bore)
        const double sc = clampv(0.086 / std::max(0.03, geom_.boreEq), 0.55, 1.8);
        block_.clear();
        //            f (Hz)        Q    gain
        block_.add( 380 * sc,  5.0, 1.00, fs);   // sump / oil pan "boom": body
        block_.add( 640 * sc,  7.0, 0.80, fs);   // block bending
        block_.add(1050 * sc,  9.0, 0.70, fs);
        block_.add(1700 * sc, 11.0, 0.55, fs);   // block / cover panels
        block_.add(2600 * sc, 12.0, 0.40, fs);
        block_.add(3900 * sc, 14.0, 0.25, fs);
        head_.clear();
        head_.add(2100 * sc,  8.0, 0.50, fs);    // valve cover
        head_.add(3300 * sc, 10.0, 0.80, fs);    // valve seat / tappet tick
        head_.add(4800 * sc, 12.0, 0.50, fs);
        head_.add(6800 * sc, 14.0, 0.25, fs);
        // knock: first transverse acoustic mode of the combustion chamber (c ~ 950 m/s)
        double fk = clampv(1.841 * 950.0 / (kPi * std::max(0.03, geom_.boreEq)), 3000.0, 14000.0);
        ping_.bandpass(fk, 40, fs);
    }
    starterBp_.bandpass(900, 2, fs);
}

// chamber phases: piston = one chamber per cylinder; rotary = 3 faces per rotor, each face
// firing 360 shaft degrees after the previous one (one power stroke per rotor per revolution)
void Engine::setupChambers() {
    const int nc = (int)cyls_.size();
    for (int i = 0; i < kMaxCyl; ++i) {
        sprPort_[0][i] = cylSpread(i, nPorts_, 11);
        sprPort_[1][i] = cylSpread(i, nPorts_, 12);
        for (int k = 0; k < 3; ++k) sprCyl_[k][i] = i < nc ? cylSpread(i, nc, 13 + k) : 0.0;
    }
    for (int i = 0; i < (int)cyls_.size(); ++i) {
        if (geom_.rotary) {
            int r = i / 3, face = i % 3;
            portOf_[i] = r;
            cyls_[i].phase = wrapPos(phase_[r] + 360.0 * face, cycle_);
        } else {
            portOf_[i] = i;
            cyls_[i].phase = phase_[i];
        }
    }
}

void Engine::updateValves() {
    // rotary ports: intake around the intake BDC (810), exhaust around the exhaust stroke
    pin_.open = wrapPos(540.0 + p_.rIntakeOpen, 1080.0);
    pin_.close = wrapPos(810.0 + p_.rIntakeClose, 1080.0);
    pin_.area = std::max(0.0, (double)p_.rIntakeArea_cm2) * 1e-4 * 0.72;
    pin_.ramp = 30;
    pex_.open = wrapPos(270.0 - p_.rExhaustOpen, 1080.0);
    pex_.close = wrapPos(540.0 + p_.rExhaustClose, 1080.0);
    pex_.area = std::max(0.0, (double)p_.rExhaustArea_cm2) * 1e-4 * 0.72;
    pex_.ramp = 18;
    vin_.n = std::max(1, (int)p_.intakeValves);
    vin_.dia = p_.intakeValveDia_mm * 1e-3;
    vin_.lift = p_.intakeLift_mm * 1e-3;
    vin_.dur = clampv((double)p_.intakeDuration, 120.0, 400.0);
    vin_.center = 360.0 + p_.intakeCenterline - p_.intakeCamAdvance;
    vin_.lash = p_.valveLash_mm * 1e-3;
    vex_.n = std::max(1, (int)p_.exhaustValves);
    vex_.dia = p_.exhaustValveDia_mm * 1e-3;
    vex_.lift = p_.exhaustLift_mm * 1e-3;
    vex_.dur = clampv((double)p_.exhaustDuration, 120.0, 400.0);
    vex_.center = 360.0 - p_.exhaustCenterline - p_.exhaustCamAdvance;
    vex_.lash = p_.valveLash_mm * 1e-3;
    // VTEC high cam: same valves and centerlines, longer and higher
    vinHi_ = vin_; vexHi_ = vex_;
    vinHi_.lift = p_.vtecInLift_mm * 1e-3; vinHi_.dur = clampv((double)p_.vtecInDuration, 120.0, 400.0);
    vexHi_.lift = p_.vtecExLift_mm * 1e-3; vexHi_.dur = clampv((double)p_.vtecExDuration, 120.0, 400.0);
    vin_.center = vinHi_.center = vin_.center - vtcNow_;
}

// lift with VTEC: once the rockers are locked onto the high cam it sets the lift (it encloses the low profile);
// while the pin slides the two are blended
double Engine::camLift(const ValveSpec& lo, const ValveSpec& hi, double phi, double fl) const {
    double L = valveLift(lo, phi, fl);
    if (vtecBlend_ > 0) { const double H = valveLift(hi, phi, fl); L += vtecBlend_ * (std::max(H, L) - L); }
    return L;
}

void Engine::apply(const EngineParams& p) {
    p_ = p;
    p_.cylinders = nPorts_;
    CylGeom old = geom_;
    geom_.set(p_);
    computePhases(p_, phase_);
    updateValves();
    comp_.build(p_);
    if (p_.comp.repairCount != repairSeen_) { repairSeen_ = p_.comp.repairCount; damage = 0; failure = 0; }
    setupChambers();
    for (Cylinder& c : cyls_) {
        double phi = wrapPos(theta_ - c.phase, cycle_);
        c.prevPhi = phi;
        if (old.Vd != geom_.Vd || old.Vc != geom_.Vc || old.rod != geom_.rod) {
            double dv, wall;
            geom_.chamber(phi, c.V, dv, wall);
            // re-derive pressure for the new volume (isothermal snap) to avoid a p dV spike
            c.p = c.m * kR * c.T / c.V;
        }
    }
    intake.apply(p_);
    exhaust.apply(p_);
    boost.apply(p_);
    ecu.apply(p_);
}

// one ECU loop: electrical / fuel / thermal physics at this rate, sensors, then the controller
void Engine::ecuTick(double h, double pedal, double rpm) {
    const double mapPa = intake.map();
    const double fuelKgs = intake.fuelFlow();
    elec.step(h, rpm, p_.ignition != 0, p_.starter != 0);
    rail.step(h, fuelKgs / 0.745 * 3600.0, (mapPa - kPatm) / 1e5, p_.ignition != 0, p_);
    ThermalInputs ti;
    ti.fuelPowerW = fuelKgs * kLHV;
    ti.frictionPowerW = lastFricW_;
    ti.airflowKgs = std::max(0.0, intake.airFlow());
    ti.speedMs = vehSpeed_;
    ti.rpm = rpm;
    ti.ambientC = p_.ambientTemp_C;
    thermal.step(h, ti, p_);
    intake.setInletTemp(thermal.iatC + 273.15);
    intake.setColdWalls(thermal.coldFactor());
    // the actual injector flow / dead time follow the rail pressure and battery voltage
    const double rated = p_.injector_ccmin * 0.745e-3 / 60.0;
    intake.setSprayRate(rated * std::sqrt(std::max(0.0, rail.deltaBar((mapPa - kPatm) / 1e5)) / 3.0));

    SensorTruth t;
    t.v[kSnsTPS] = intake.throttlePosition() * 100.0;
    t.v[kSnsAPP] = clampv(pedal, 0.0, 1.0) * 100.0;
    t.v[kSnsMAP] = mapPa / 1000.0;
    t.v[kSnsMAF] = std::max(0.0, intake.airFlow()) * 1000.0;
    t.v[kSnsIAT] = isBoosted(p_) ? intake.inlet(0).T - 273.15 : thermal.iatC;
    t.v[kSnsECT] = thermal.coolantC;
    t.v[kSnsO2] = t.v[kSnsWideband] = lamFast_;
    t.v[kSnsCKP] = rpm;
    t.v[kSnsCMP] = 1.0;
    t.v[kSnsKnock] = knockMax;
    for (int i = 0; i < (int)cyls_.size(); ++i) {
        const int port = clampv(portOf_[i], 0, kMaxCyl - 1);
        t.knockCyl[port] = std::max(t.knockCyl[port], cyls_[i].knock);
        t.knockSeq[port] += cyls_[i].cycles;
    }
    t.v[kSnsEGT] = egtAvg - 273.15;
    t.v[kSnsOilP] = oilBar;
    t.v[kSnsOilT] = thermal.oilC;
    t.v[kSnsFuelP] = rail.pressureBar;
    t.v[kSnsVSS] = vehSpeed_ * 3.6;
    t.v[kSnsGear] = vehGear_;
    t.v[kSnsBattery] = elec.volts;
    t.in.clutch = p_.clutchPedal > 0.5f;
    t.in.transbrake = p_.transbrake != 0 && p_.transMode != TransMode::Manual;
    t.in.n2oArm = p_.n2oArm != 0;
    t.in.rollingAls = p_.rollingAlsButton != 0;
    t.in.shift = shifting_;
    t.in.wheelKmh = wheelSpeed_ * 3.6;
    sensors.update(h, t, p_);
    ecu.update(h, sensors.r, p_.starter != 0);
    intake.setIac(ecu.cmd.iac);
    // asynchronous tip-in pulses
    if (p_.fuelSystem == FuelSystem::EFI && p_.ignition && !fuelCutNow_) {
        const double cycT = rpm > 30 ? 60.0 * (geom_.rotary ? 1.0 : 2.0) / rpm : 1.0;
        const double ratedA = p_.injector_ccmin * 0.745e-3 / 60.0;
        const double flowA = ratedA * std::sqrt(std::max(0.0, rail.deltaBar((mapPa - kPatm) / 1e5)) / 3.0);
        const double deadA = std::max(0.0, (double)p_.comp.injDeadTime_ms) * 1e-3 * (1.0 + 0.12 * (14.0 - elec.volts));
        for (int i = 0; i < (int)ecu.asyncFuel.size(); ++i)
            if (ecu.asyncFuel[i] > 0) intake.injectPulse(i, ecu.pulseFor(ecu.asyncFuel[i], sensors.r), cycT, flowA, deadA);
    }
}

// nitrous: the solenoid (ECU duty) lets N2O from the bottle into the ports - modelled as air-equivalent charge
// (oxygen + decomposition heat, kN2oAirEquiv), a wet kit's own fuel, and charge cooling from the liquid flashing
void Engine::nitrousStep(double dt) {
    if (p_.n2oRefill != n2oRefillSeen_) { n2oRefillSeen_ = p_.n2oRefill; n2oBottle = std::max(0.0f, p_.n2oBottleKg); }
    n2oFlow = 0;
    double coolTarget = 0;
    if (p_.nitrous && p_.ignition && ecu.cmd.n2oDuty > 0 && n2oBottle > 0) {
        // the bottle delivers liquid at full pressure until the last ~15 %, then the flow falls off
        const double left = n2oBottle / std::max(0.1, (double)p_.n2oBottleKg);
        n2oFlow = n2oFullFlowKgs(p_) * ecu.cmd.n2oDuty * clampv(left / 0.15, 0.0, 1.0);
        n2oBottle = std::max(0.0, n2oBottle - n2oFlow * dt);
        const double airEq = n2oFlow * kN2oAirEquiv * dt / nPorts_;
        const double fuel = p_.n2oWet ? airEq / 12.5 : 0.0;   // wet kit: jetted a little rich of stoichiometric
        for (int i = 0; i < nPorts_; ++i) intake.addPortCharge(i, airEq, fuel);
        const double air = std::max(1e-4, intake.airFlow());
        coolTarget = 250.0 * n2oFlow / (air + n2oFlow);
    }
    n2oCool_ += (coolTarget - n2oCool_) * std::min(1.0, dt / 0.15);
    if (n2oCool_ > 1e-3 || coolTarget > 0) intake.setPortCooling(n2oCool_);
}

// injection event of a port: the ECU decides the pulse, the injector delivers what physics allows
void Engine::injectPort(int port, double rpm, double share) {
    if (p_.fuelSystem != FuelSystem::EFI) return;
    if (fuelCutNow_ || !p_.ignition) { ecu.targetAfr = 0; return; }
    double pw = ecu.injectionPulse(port, sensors.r, share);
    double cyc = rpm > 30 ? 60.0 * (geom_.rotary ? 1.0 : 2.0) / rpm : 1.0;
    const double rated = p_.injector_ccmin * 0.745e-3 / 60.0;
    double flow = rated * std::sqrt(std::max(0.0, rail.deltaBar((intake.map() - kPatm) / 1e5)) / 3.0);
    double dead = std::max(0.0, (double)p_.comp.injDeadTime_ms) * 1e-3 * (1.0 + 0.12 * (14.0 - elec.volts));
    intake.injectPulse(port, pw, cyc, flow, dead);
}

// worn apex seals let gas blow past into the neighbouring chamber of the same rotor
void Engine::apexLeakage(double dt) {
    double cda = (0.15 + 6.0 * clampv((double)p_.apexLeak, 0.0, 1.0)) * 1e-6 * (geom_.width / 0.08);
    for (int r = 0; r < nPorts_; ++r) {
        for (int k = 0; k < 3; ++k) {
            Cylinder& a = cyls_[r * 3 + k];
            Cylinder& b = cyls_[r * 3 + (k + 1) % 3];
            double f = limitedFlow(cda, a.p, a.T, a.V, b.p, b.T, b.V, dt, 0.2);
            Cylinder& src = f > 0 ? a : b;
            Cylinder& dst = f > 0 ? b : a;
            double q = std::min(std::fabs(f) * dt, src.m * 0.2);
            if (q <= 0) continue;
            double ff = src.mf / src.m, bf = src.mb / src.m, h = q * kCp * src.T;
            src.m -= q; src.mf -= q * ff; src.mb -= q * bf; src.U -= h;
            dst.m += q; dst.mf += q * ff; dst.mb += q * bf; dst.U += h;
            for (Cylinder* cc : {&src, &dst}) {
                cc->T = cc->U / (cc->m * kCv);
                cc->p = cc->m * kR * cc->T / cc->V;
            }
        }
    }
}

// valve lift including valve float: past the float rpm the spring can't keep the follower on the
// cam - the valve lofts, closes late and bounces off its seat
double Engine::valveLift(const ValveSpec& v, double phi, double fl) const {
    double L = v.liftAt(phi);
    if (fl <= 0) return L;
    const double lag = 30.0 * fl;
    L = std::max(L, std::min(v.lift * 1.15, v.liftAt(phi - lag) * (1.0 + 0.15 * fl)));
    double d = wrapSym(phi - (v.center + v.dur * 0.5) - lag, 720.0);
    if (d > 0 && d < 25.0) L += fl * 0.12 * v.lift * std::sin(kPi * d / 25.0);
    return L;
}

double Engine::burnDuration(const Cylinder& c, double rpm, double lambda) {
    double D = 47.0 + 18.0 * clampv(rpm / 7000.0, 0.0, 1.3);
    double dl = lambda - 1.0;
    D *= 1.0 + 2.0 * dl * dl + (lambda > 1.05 ? 1.4 * (lambda - 1.05) : 0.0);
    double resid = c.m > 0 ? c.mb / c.m : 0;
    D *= 1.0 + 2.2 * resid;
    double lowLoad = clampv(1.0 - c.pIvc / kPatm, 0.0, 0.8);
    D *= 1.0 + 0.45 * lowLoad;
    double cov = 0.05 + 0.10 * lowLoad;
    D *= 1.0 + cov * rng_.gauss();
    // rotary: long, thin, moving combustion chamber burns slowly (in shaft degrees)
    if (geom_.rotary) D *= 1.2;
    return clampv(D, 22.0, 220.0);
}

void Engine::stepCylinder(int i, double dt, double phiPrev, double phi, double rpm, bool spark, bool fuelCut) {
    Cylinder& c = cyls_[i];
    const int port = portOf_[i];
    GasNode& ip = intake.port(port);
    GasNode& ep = exhaust.port(port);

    double Vn, dVdth, wallArea;
    geom_.chamber(phi, Vn, dVdth, wallArea);
    const double dV = Vn - c.V;

    double dU = -c.p * dV;
    double dm = 0, dmf = 0, dmb = 0;
    c.work += c.p * dV;

    // ---- valves
    double Ai, Ae;
    if (geom_.rotary) { Ai = pin_.areaAt(phi); Ae = pex_.areaAt(phi); }
    else { Ai = vin_.area(camLift(vin_, vinHi_, phi, floatIn)); Ae = vex_.area(camLift(vex_, vexHi_, phi, floatEx)); }
    // no two ports flow the same (casting, seat, valve-to-guide spread)
    const double imp = effImperfection(p_);
    Ai *= 1.0 + 0.02 * imp * sprPort_[0][port];
    Ae *= 1.0 + 0.02 * imp * sprPort_[1][port];
    bool inOpen = Ai > 0, exOpen = Ae > 0;
    double ffC = c.m > 0 ? c.mf / c.m : 0, bfC = c.m > 0 ? c.mb / c.m : 0;

    if (Ai > 0) {
        double f = limitedFlow(Ai, ip.p(), ip.T, ip.V, c.p, c.T, c.V, dt, 0.35);
        double q = f * dt;
        if (q > 0) {
            q = std::min(q, ip.m * 0.5);
            double ff = ip.fuelFrac(), bf = ip.burnedFrac();
            ip.m -= q; ip.mf -= q * ff; ip.mb -= q * bf;
            dm += q; dmf += q * ff; dmb += q * bf;
            dU += q * kCp * ip.T;
        } else if (q < 0) {
            q = std::min(-q, c.m * 0.3);
            ip.m += q; ip.mf += q * ffC; ip.mb += q * bfC;
            dm -= q; dmf -= q * ffC; dmb -= q * bfC;
            dU -= q * kCp * c.T;
        }
    }
    if (Ae > 0) {
        double f = limitedFlow(Ae, c.p, c.T, c.V, ep.p(), ep.T, ep.V, dt, 0.35);
        double q = f * dt;
        if (q > 0) {
            q = std::min(q, c.m * 0.3);
            ep.m += q; ep.mf += q * ffC; ep.mb += q * bfC;
            dm -= q; dmf -= q * ffC; dmb -= q * bfC;
            dU -= q * kCp * c.T;
            double w = clampv(q / (c.m + 1e-9), 0.0, 1.0);
            c.egt += (c.T - c.egt) * w * 0.5;
            const double airOut = q * std::max(0.0, 1.0 - ffC - bfC);
            if (c.burning) {
                // the flame is still going when the exhaust valve opens (late / slow burn at high rpm): the mixture
                // that leaves keeps burning in the port as a continuous flame - heat for the exhaust, not a pop
                const double b = std::min(q * ffC, airOut / kStoich);
                afHeatJ_ += b * kLHV * 0.9;
                c.afterfireFuel += q * ffC - b;
                c.afterfireAir += airOut - b * kStoich;
            } else {
                c.afterfireFuel += q * ffC;
                c.afterfireAir += airOut;   // fresh air (misfired / lean / cut charge)
            }
        } else if (q < 0) {
            q = std::min(-q, ep.m * 0.5);
            double ff = ep.fuelFrac(), bf = ep.burnedFrac();
            ep.m -= q; ep.mf -= q * ff; ep.mb -= q * bf;
            dm += q; dmf += q * ff; dmb += q * bf;
            dU += q * kCp * ep.T;
        }
    }
    if (ip.mf < 0) ip.mf = 0;
    if (ep.mf < 0) ep.mf = 0;

    // ---- valve closing events (IVC: trapped charge, mechanical tick)
    c.valveSeatEvent = !geom_.rotary && ((c.prevIntakeOpen && !inOpen) || (c.prevExhaustOpen && !exOpen));
    if (c.prevIntakeOpen && !inOpen) {
        double air = std::max(0.0, c.m - c.mf - c.mb);
        c.trappedAir = air;
        c.trappedFuel = c.mf;
        c.lambda = c.mf > 1e-9 ? air / (c.mf * kStoich) : 9.99;
        double rho = kPatm / (kR * ambT_);
        c.ve = air / (rho * geom_.Vd);
        c.pIvc = c.p; c.tIvc = c.T;
    }
    c.prevIntakeOpen = inOpen;
    c.prevExhaustOpen = exOpen;

    // ---- injection event (EFI)
    {
        const double ia = wrapPos(p_.injectionAngle, cycle_);
        // no cam signal: the ECU can't tell the cycle phase -> batch fire on every revolution
        const bool batch = ecu.cmd.batchFire && !geom_.rotary;
        if (crossed(phiPrev, phi, ia)) injectPort(port, rpm, batch ? 0.5 : 1.0);
        if (batch && crossed(phiPrev, phi, wrapPos(ia + 360.0, cycle_))) injectPort(port, rpm, 0.5);
    }

    // ---- spark
    const double sparkAt = wrapPos(-(advance_ + ecu.cylSpark(port) - ecu.cmd.cylRetard[clampv(port, 0, kMaxCyl - 1)]) + 0.5 * imp * sprCyl_[0][i], cycle_);
    if (crossed(phiPrev, phi, sparkAt) && !c.burning) {
        c.knockCycle = 0;
        // soft limiter: the ECU drops a share of the sparks
        if (spark && !(ecu.cmd.cutFraction > 0 && rng_.uniform() < ecu.cmd.cutFraction)) {
            double air = std::max(0.0, c.m - c.mf - c.mb);
            // only the vaporised fuel takes part (cold: part of it is still liquid on the walls)
            const double vapF = 1.0 - 0.25 * thermal.coldFactor();
            double lam = c.mf > 1e-10 ? air / (c.mf * vapF * kStoich) : 99.0;
            double resid = c.m > 0 ? c.mb / c.m : 1;
            // misfire probability per cause; the largest one decides (and is counted for diagnostics)
            double pc[kMisCauseCount] = {};
            // wider plug gap lights leaner mixtures (bigger kernel)
            const double leanLim = 1.35 + 0.15 * clampv((double)p_.comp.plugGap_mm - 0.8, -0.4, 0.6);
            if (lam > leanLim) pc[kMisLean] = clampv((lam - leanLim) / 0.3, 0.0, 1.0);
            // ignition energy: a coil that had too little dwell (or high cylinder pressure / wide gap) can't fire
            if (c.mf > 1e-9) {
                double need = ComponentModel::sparkNeeded(p_, c.p, lam);
                double ratio = sparkAvail / std::max(1e-3, need);
                sparkMargin = std::min(sparkMargin, ratio);
                if (ratio < 1.0) pc[kMisSpark] = clampv((1.0 - ratio) / 0.4, 0.0, 1.0);
            }
            pc[kMisFouled] = clampv(fouling, 0.0, 1.0) * 0.6;   // fouled plug
            if (lam < 0.62) pc[kMisRich] = clampv((0.62 - lam) / 0.15, 0.0, 1.0);
            // dilution: burned gas in the charge (residual, exhaust reverted through a big overlap) - past ~35 % the
            // flame kernel starts to fail. A misfired charge leaves fresh mixture behind, so the next one fires
            // harder: the lumpy idle of big-overlap engines (bridge / peripheral ported rotaries).
            c.dilution = resid;
            if (resid > 0.35) pc[kMisDilution] = clampv((resid - 0.35) / 0.25, 0.0, 1.0);
            // fuel cut: nothing (or only the trace evaporating off the port walls) to light - not a misfire
            // (a charge inducted during the cut - up to one cycle after it ended - counts as cut, too)
            const bool noFuel = c.mf < 1e-9 || (sinceCutS_ < 1.5 * 60.0 * (cycle_ / 360.0) / std::max(rpm, 300.0) && lam > 1.3);
            int cause = 0;
            for (int k = 1; k < kMisCauseCount; ++k) if (pc[k] > pc[cause]) cause = k;
            const double pMis = noFuel ? 1.0 : (cause == kMisRich ? pc[kMisRich] : pc[cause]);
            if (rng_.uniform() < pMis) {
                c.misfired = true;
                if (!noFuel) { ++misfires; ++misfireCause[cause]; }
                c.burnEta = 0;
            } else {
                c.misfired = false;
                const double full = std::min(c.mf * (1.0 - 0.25 * thermal.coldFactor()), air / kStoich);   // cold: poor vaporisation
                c.burnEta = 1.0;
                c.fuelBurn = full;
                c.qTotal = c.fuelBurn * kLHV * 0.92 * (1.0 + 0.015 * imp * sprCyl_[1][i]); // efficiency spread
                c.burnDur = burnDuration(c, rpm, lam) * (1.0 - 0.06 * clampv(((double)p_.comp.plugGap_mm - 0.8) / 0.3, -1.0, 1.5)) *
                            (1.0 + 0.03 * imp * sprCyl_[2][i]) *
                            (1.0 + 0.01 * imp * rng_.gauss());                    // extra cycle-to-cycle variation
                c.burnStart = wrapPos(sparkAt + 0.25 * imp * rng_.gauss(), cycle_);
                c.xPrev = 0;
                c.burning = true;
                fireTimer_ = 0;
            }
        } else c.misfired = true;   // spark cut (limiter / launch / shift / traction control) or ignition off: the charge stays unburned
    }
    if (c.burning) {
        // signed angle since the start of the burn: the cycle-to-cycle jitter can put the start a
        // fraction of a degree after the spark step - that is "not yet", not "720 deg ago"
        double d = wrapSym(phi - c.burnStart, cycle_);
        double x;
        if (d < 0 && d > -10.0) x = 0.0;
        else if (d < 0 || d >= c.burnDur) {   // (d < -10: a stale burn more than half a cycle old - finish it)
            x = 1.0; c.burning = false;
            c.burnEnded = true;
        }
        else { double y = d / c.burnDur; x = 1.0 - std::exp(-5.0 * y * y * y); }
        double dx = std::max(0.0, x - c.xPrev);
        c.xPrev = x;
        dU += c.qTotal * dx;
        double fb = c.fuelBurn * dx;
        dmf -= fb;
        dmb += fb * (1.0 + kStoich);
        if (c.burnEnded) {
            // fuel left over after a rich burn ends up as CO/HC products, not as fuel
            // for the next cycle (no rich run-away through the residual gas)
            c.burnEnded = false;
            // fuel left over after a rich burn ends up as CO/HC products, not as fuel
            // for the next cycle (no rich run-away through the residual gas)
            double left = std::max(0.0, c.mf + dmf);
            dmf -= left * 0.9;
            dmb += left * 0.9;
        }
        if (x < 0.9 && c.pIvc > 0) {
            double tu = c.tIvc * std::pow(std::max(1.0, c.p / c.pIvc), (kGamma - 1.0) / kGamma);
            // end-gas autoignition gets much easier at high absolute pressure (boost)
            tu += 1.3e-5 * std::max(0.0, c.p - 90e5);
            // a too-hot plug tip glows: surface ignition heats the end gas (pre-ignition -> knock)
            tu += 3.0 * std::max(0.0, plugTempC - 850.0);
            tu += 0.6 * (thermal.coolantC - 90.0);             // hot coolant / walls
            c.knockCycle = std::max(c.knockCycle, tu);
        }
    }

    // ---- wall heat transfer (Woschni-like)
    {
        double Sp = 2.0 * geom_.strokeEq * rpm / 60.0;
        double w = 2.28 * Sp + (c.burning ? 3.0 : 0.0) + 0.5;
        double h = 3.26 * std::pow(geom_.boreEq, -0.2) * std::pow(std::max(1.0, c.p / 1000.0), 0.8) *
                   std::pow(std::max(200.0, c.T), -0.55) * std::pow(w, 0.8);
        dU -= h * wallArea * (c.T - (thermal.coolantC + 273.15 + 87.0)) * dt;
    }

    // ---- integrate
    c.U += dU;
    c.m = std::max(1e-8, c.m + dm);
    c.mf = clampv(c.mf + dmf, 0.0, c.m);
    c.mb = clampv(c.mb + dmb, 0.0, c.m - c.mf);
    c.V = Vn;
    c.T = c.U / (c.m * kCv);
    if (c.T < 150 || c.T > 3600) { c.T = clampv(c.T, 150.0, 3600.0); c.U = c.m * kCv * c.T; }
    double pPrev = c.p;
    c.p = c.m * kR * c.T / c.V;
    c.pmax = std::max(c.pmax, c.p);

    // ---- per-cycle bookkeeping at firing TDC
    if (phi < phiPrev) {
        c.imep = c.work / geom_.Vd;
        c.work = 0;
        c.pmaxCycle = c.pmax;
        c.pmax = 0;
        c.knock = clampv((c.knockCycle - 940.0) / 110.0, 0.0, 5.0);
        ++c.cycles;
        c.knockEvent = c.knock > 1.0;
        c.knockCycle = 0;
    }

    // torque on the crank
    torqueInst_ += (c.p - kPatm) * dVdth;
    // structure-borne combustion noise
    blockIn_ += (c.p - pPrev) * 1.0e-6;          // combustion pressure rise shakes the block
    if (c.valveSeatEvent) tickIn_ += 0.6 * (0.3 + rpm / 3000.0);
}

void Engine::step(double dt, double pedal, double loadTorque, bool signedLoad) {
    const double rpm = this->rpm();
    const int n = (int)cyls_.size();

    // ---- ECU loop (sensors -> controller) at its own rate
    {
        const double ecuDt = 1.0 / clampv((double)p_.ecuRateHz, 100.0, 5000.0);
        ecuAcc_ += dt;
        if (ecuAcc_ >= ecuDt) { ecuTick(std::min(ecuAcc_, 0.05), pedal, rpm); ecuAcc_ = 0; }
    }
    double mapPa = intake.map();
    const ActuatorCommands& cmd = ecu.cmd;
    bool fuelCut = cmd.fuelCut || cmd.syncLost;
    fuelCutNow_ = fuelCut;
    sinceCutS_ = (fuelCut || (p_.fuelSystem == FuelSystem::EFI && ecu.targetAfr <= 0)) ? 0.0 : sinceCutS_ + dt;
    bool spark = p_.ignition && !cmd.sparkCut && !cmd.syncLost;
    advance_ = cmd.sparkAdvance;
    antiLagActive = cmd.antiLag;
    if (cmd.throttleTarget >= 0) pedal = cmd.throttleTarget;   // drive-by-wire: the ECU positions the throttle
    pedal = std::max(pedal, cmd.throttleCrack);

    // ---- hardware components: valve float, spark energy, plug temperature, health
    floatIn = comp_.floatSeverity(rpm, true);
    floatEx = comp_.floatSeverity(rpm, false);
    sparkAvail = comp_.sparkEnergy(p_, rpm);
    sparkMargin = std::min(9.0, sparkMargin + dt * 0.5);      // recovers slowly, dips on weak sparks
    updateHealth(dt, rpm, mapPa);
    if (failure) spark = false;

    fireTimer_ += dt;
    running_ = rpm > 300 && fireTimer_ < 0.3;
    boost.setWgDuty(ecu.cmd.wgDuty);
    nitrousStep(dt);
    // ---- cam actuators: the VTEC spool valve locks / unlocks the rockers in ~80 ms, the VTC phaser moves at
    //      ~200 crank deg/s; both need oil pressure (a phaser without it rests at its base position)
    if (p_.vtec || p_.vtc) {
        const double tgt = (p_.vtec && ecu.cmd.vtec && oilBar > 1.0) ? 1.0 : 0.0;
        vtecBlend_ = tgt > vtecBlend_ ? std::min(tgt, vtecBlend_ + dt / 0.08) : std::max(tgt, vtecBlend_ - dt / 0.08);
        const double vt = (p_.vtc && oilBar > 0.5) ? ecu.cmd.vtcAdvance : 0.0;
        vtcNow_ += clampv(vt - vtcNow_, -200.0 * dt, 200.0 * dt);
        vin_.center = vinHi_.center = 360.0 + p_.intakeCenterline - p_.intakeCamAdvance - vtcNow_;
    }
    double scTorque = isBoosted(p_) ? boost.step(dt, omega_, egtAvg, intake, exhaust) : 0.0;
    intake.stepAir(dt, pedal, rpm, p_.ignition && !fuelCut);

    // ---- cylinders
    const double dTheta = omega_ * dt / kDeg;
    const double thetaNew = theta_ + dTheta;
    torqueInst_ = 0;
    for (int i = 0; i < n; ++i) {
        double phiPrev = cyls_[i].prevPhi;
        double phi = wrapPos(thetaNew - cyls_[i].phase, cycle_);
        stepCylinder(i, dt, phiPrev, phi, rpm, spark, fuelCut);
        cyls_[i].prevPhi = phi;
    }
    if (geom_.rotary) apexLeakage(dt);

    // ---- afterfire (unburned fuel + oxygen igniting in a hot primary)
    for (int i = 0; i < n; ++i) {
        Cylinder& c = cyls_[i];
        c.afterfireFuel *= std::exp(-dt / 0.07);
        c.afterfireAir *= std::exp(-dt / 0.07);
        c.egt += (ambT_ + 150 - c.egt) * dt * 0.05; // slow cool down
        // only fuel that meets oxygen can burn: the exhaust of a complete rich burn holds unburned fuel (CO / HC)
        // but no air, so a rich engine does not pop - a misfired, cut or very lean charge brings both
        const double burnable = std::min(c.afterfireFuel, c.afterfireAir / kStoich);
        // a firing cylinder under load fills its port with hot, fast gas: mixture that slips through (scavenging
        // with a big overlap, late burning) burns there as a continuous flame. Pops need a pocket of mixture - an
        // unburned (misfired or cut) charge, or the cooler gas of the overrun.
        if (!c.misfired && c.egt > 900 && burnable > 0) {
            const double b = burnable * std::min(1.0, dt / 0.004);
            c.afterfireFuel -= b; c.afterfireAir = std::max(0.0, c.afterfireAir - b * kStoich);
            afHeatJ_ += b * kLHV * 0.9;
        } else if (burnable > 1.5e-6 && c.egt > 650 && p_.ignition &&
                   // the pocket must be flammable: the trace of fuel the port walls give off during a fuel cut
                   // sits in far too much air to ignite (lean flammability limit ~ lambda 1.7)
                   c.afterfireAir < 1.7 * kStoich * c.afterfireFuel) {
            double rate = 25.0 * (burnable / 1e-5) * smoothstep(650, 950, c.egt);
            if (rng_.uniform() < rate * dt) {
                double burn = burnable * 0.7;
                c.afterfireFuel -= burn;
                c.afterfireAir = std::max(0.0, c.afterfireAir - burn * kStoich);
                afHeatJ_ += burn * kLHV * 0.9;
                GasNode& ep = exhaust.port(portOf_[i]);
                c.popPending += std::min(burn * kLHV * (kGamma - 1.0) / ep.V * 0.03, 0.7e5) * (0.6 + 0.4 * rng_.uniform());
                ++afterfires;
            }
        }
        if (c.popPending > 1.0) {
            // release the pop over ~0.6 ms: a sharp but band-limited bang
            GasNode& ep = exhaust.port(portOf_[i]);
            double dP = c.popPending * std::min(1.0, dt / 0.0006);
            c.popPending -= dP;
            double dmb = dP * ep.V / (kR * ep.T);
            ep.m += dmb; ep.mb += dmb;
            blockIn_ += dP * 2e-6;
        }
    }

    exhaust.step(dt);
    intake.stepWaves(dt);

    // ---- crank dynamics
    // volume swept per shaft revolution: piston = half the displacement, rotary = Vd per rotor
    const double perRev = geom_.rotary ? geom_.Vd * nPorts_ : geom_.Vd * nPorts_ * 0.5;
    const double Sp = 2.0 * geom_.strokeEq * rpm / 60.0;
    const double fmep = geom_.rotary ? (70e3 + 5.0e3 * Sp + 0.15e3 * Sp * Sp)   // no valvetrain / rods
                                     : (88e3 + 6.0e3 * Sp + 0.12e3 * Sp * Sp);
    const double fmepAll = std::max(0.4 * fmep, fmep + comp_.extraFmep(p_, rpm, Sp, perRev)) * thermal.viscosityFriction();
    double tFric = (fmepAll * perRev / (2.0 * kPi) + 4.0) * std::tanh(omega_ / 3.0);
    if (failure) tFric += 600.0 * std::tanh(omega_ / 3.0);     // broken parts: the engine locks up
    lastFricW_ = tFric * omega_;
    double tStarter = 0;
    if (p_.starter) {
        double tmax = 60.0 + 200.0 * geom_.Vd * 1000.0 + 15.0 * nPorts_;
        tStarter = tmax * clampv(1.0 - rpm / 380.0, 0.0, 1.0);
    }
    double tLoad = signedLoad ? loadTorque + scTorque * std::tanh(omega_ / 3.0)
                              : (loadTorque + scTorque) * std::tanh(omega_ / 3.0);
    double J = totalInertia();
    double net = torqueInst_ - tFric;
    omega_ += (net - tLoad + tStarter) / J * dt;
    if (omega_ < 0) omega_ = 0;
    theta_ = wrapPos(thetaNew, cycle_);

    double tau = clampv(240.0 / std::max(rpm, 100.0), 0.04, 1.0);
    // cycle-average torque: integral of torque over exactly the last working cycle, kept in
    // one-degree bins (a boxcar over 720 / 1080 deg - only half a cycle of delay, no smearing)
    {
        const int nb = (int)cycle_;
        int bin = clampv((int)theta_, 0, nb - 1);
        if (bin != tqBin_) {
            // entering new bin(s): clear everything we skipped over
            int b = tqBin_;
            while (b != bin) { b = (b + 1) % nb; tqSum_ -= tqBins_[b]; tqBins_[b] = 0; }
            tqBin_ = bin;
        }
        double w = net * dTheta;
        tqBins_[bin] += w; tqSum_ += w;
        double cov = std::min((double)nb, cycleDegSeen_ += dTheta);
        double boxcar = cov > 1 ? tqSum_ / cov : net;
        // fall back to the old smoothing while stopped / cranking very slowly
        torqueAvg_ = rpm > 200 ? boxcar : torqueAvg_ + (net - torqueAvg_) * (dt / tau);
    }

    // ---- telemetry traces for cylinder 0
    {
        const Cylinder& c = cyls_[0];
        int idx = clampv((int)c.prevPhi, 0, (int)cycle_ - 1);
        traceP[idx] = (float)(c.p * 1e-5);
        traceV[idx] = (float)(c.V * 1e6);
        traceExh[idx] = (float)(exhaust.portPressure(0) * 1e-5);
        traceInt[idx] = (float)(intake.port(0).p() * 1e-5);
    }
    double lam = 0, ve = 0, kn = 0, egt = 0;
    for (const Cylinder& c : cyls_) { lam += std::min(c.lambda, 3.0); ve += c.ve; kn = std::max(kn, c.knock); egt += c.egt; }
    double a = dt / 0.4;
    lambdaAvg += (lam / n - lambdaAvg) * a;
    lamFast_ += (lam / n - lamFast_) * std::min(1.0, dt / 0.02);
    veAvg += (ve / n - veAvg) * a;
    // gas leaving the cylinders -> exhaust port and manifold: heat goes into the coolant-cooled port walls before
    // the gas reaches the pipes / the EGT sensor. Pipe-flow heat transfer (Dittus-Boelter, a laminar floor, x1.8
    // for the pulsating flow) over port + manifold run of ~4 port diameters: little loss at full flow, a lot at
    // idle, where the slow gas spends long in the port - that is why a real idle EGT is only some 300-550 C.
    {
        exhFlowAvg_ += (std::max(0.0, intake.airFlow() + intake.fuelFlow()) - exhFlowAvg_) * std::min(1.0, dt / 0.2);
        const double D = clampv((double)p_.primaryDia_mm, 20.0, 90.0) * 1e-3;
        const double mPort = std::max(1e-5, exhFlowAvg_ / std::max(1, nPorts_));
        const double Re = 4.0 * mPort / (kPi * D * 4.0e-5);
        const double Nu = 1.8 * std::max(8.0, 0.023 * std::pow(Re, 0.8) * 0.9);
        const double hA = Nu * 0.06 / D * kPi * D * (4.0 * D);
        const double ntu = hA / (mPort * 1150.0);
        const double Tw = thermal.coolantC + 273.15 + 120.0;
        const double Tcyl = egt / n;
        egtCyl = Tcyl;
        double Tport = Tcyl > Tw ? Tw + (Tcyl - Tw) * std::exp(-ntu) : Tcyl;
        // afterfire: fuel burning in the manifold (overrun pops, a spark-cut 2-step / limiter) heats the exhaust
        // gas downstream of the port - this is what spools a turbo on the 2-step
        afPower_ += (afHeatJ_ / std::max(1e-6, dt) - afPower_) * std::min(1.0, dt / 0.15);
        afHeatJ_ = 0;
        Tport += std::min(900.0, afPower_ / (std::max(2e-3, exhFlowAvg_) * 1150.0));
        egtAvg += (Tport - egtAvg) * a;
    }
    exhaust.setGasTemp(egtAvg);   // pipe sound speed follows the gas temperature
    knockMax = std::max(kn, knockMax - dt * 0.5);

    // ---- sound
    exhaust.radiated(sExhL, sExhR);
    sIntake = intake.radiated();
    sBoost = isBoosted(p_) ? boost.sound() : 0.0;
    for (const Cylinder& c : cyls_) if (c.knockEvent && c.prevPhi < 40 && c.prevPhi > 5) pingIn_ += 0.05 * c.knock;
    for (Cylinder& c : cyls_) if (c.prevPhi > 40) c.knockEvent = false;
    double mech = head_.tick(tickIn_ * 0.0012) + block_.tick(blockIn_ * 0.08);
    mech += ping_.tick(pingIn_) * 2.0;
    tickIn_ = 0; pingIn_ = 0; blockIn_ = 0;
    if (p_.starter) {
        // ring gear mesh whine + motor brush noise
        starterPhase_ += 2.0 * kPi * (rpm / 60.0 * 120.0 + 180.0) * dt;
        if (starterPhase_ > 2 * kPi) starterPhase_ -= 2 * kPi;
        mech += 0.08 * std::sin(starterPhase_) + starterBp_.tick(rng_.bipolar() * 0.25);
    }
    sMech = mech;
}

// stress, plug temperature / fouling, oil pressure and (optionally) mechanical damage
void Engine::updateHealth(double dt, double rpm, double mapPa) {
    stress = comp_.stress(rpm);
    oilBar = ComponentModel::oilPressure(p_, rpm, thermal.oilC);
    // plug tip temperature: rises with load and speed, colder heat range runs cooler
    const double load = clampv(mapPa / kPatm, 0.0, 3.0);
    const double rpmF = clampv(rpm / 6000.0, 0.0, 1.6);
    const int heat = clampv((int)p_.comp.plugHeatRange, 4, 10);
    double target = 450.0 + 330.0 * std::sqrt(load) * std::pow(rpmF, 0.7) + 55.0 * (6 - heat);
    if (rpm < 100) target = ambT_ - 273.15 + 20.0;
    plugTempC += (target - plugTempC) * (1.0 - std::exp(-dt / 3.0));
    // too cold + rich: carbon fouling; hot enough: it burns off
    if (plugTempC < 450.0 && lambdaAvg < 0.95 && rpm > 300) fouling += dt * 0.02 * (450.0 - plugTempC) / 50.0;
    else if (plugTempC > 500.0) fouling -= dt * 0.05;
    fouling = clampv(fouling, 0.0, 1.0);
    if (!p_.comp.allowFailure || failure) return;
    double d = 0;
    int cause = 0;
    if (stress > 1.0) { d += 2.0 * (stress - 1.0); cause = 1 + comp_.weakest; }
    double fl = std::max(floatIn, floatEx);
    if (fl > 0.5) { d += 0.5 * (fl - 0.5); if (!cause) cause = 4; }
    if (knockMax > 1.0) {
        static const double kKnockDmg[] = {0.05, 0.10, 0.015};   // cast, hypereutectic (brittle), forged
        d += (knockMax - 1.0) * kKnockDmg[clampv((int)p_.comp.pistonType, 0, 2)];
        if (!cause) cause = 5;
    }
    damage = std::min(1.0, damage + d * dt);
    if (damage >= 1.0) failure = cause ? cause : 1;
}

} // namespace es

#include "sim/Engine.h"
#include "sim/Flame.h"
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
    diesel_ = isDiesel(p_);
    ambT_ = 273.15 + p.ambientTemp_C;
    baroPa_ = ambientPressurePa(p_);
    kPatm = baroPa_;
    geom_.set(p_);
    cycle_ = cycleDeg(p_);
    const int n = clampv(p_.cylinders, 1, geom_.rotary ? 4 : kMaxCyl);
    p_.cylinders = nPorts_ = n;
    computePhases(p_, phase_);
    updateValves();
    comp_.build(p_);
    vtIn_ = valvetrainSpec(p_, true); vtEx_ = valvetrainSpec(p_, false);
    repairSeen_ = p_.comp.repairCount;
    cyls_.assign(geom_.rotary ? 3 * n : n, Cylinder());
    setupChambers();
    {
        const thermo::Burn st = thermo::burn(fuel_.molC, fuel_.molH, fuel_.molO, fuel_.stoich);
        for (Cylinder& c : cyls_) c.init(geom_, wrapPos(theta_ - c.phase, cycle_), ambT_, st.products);
    }
    intake.build(p_, fs, geom_.Vd * n);
    fuelSetup(true);
    exhaust.build(p_, phase_, fs);
    boost.init(p_, fs);
    thermal.init(p_);
    // EGT probe and header wall: at ambient on a cold start, still warm on a warm one
    egtWall_ = egtProbe_ = ambT_ + (p_.coldStart ? 0.0 : 250.0);
    egtPost_ = egtPostWall_ = egtProbe_;
    for (int i = 0; i < kMaxCyl; ++i) egtCylProbe_[i] = egtCylWall_[i] = egtProbe_;
    emapKPa_ = baroPa_ * 1e-3;
    elec.volts = 12.6;
    rail.pressureBar = 0.5;
    sensors.reset(p_);
    ecu.init(p_, geom_.Vd, n, geom_.rotary ? 1.0 : 2.0);
    {
        // mode frequencies scale inversely with size (reference: 86 mm bore)
        const double sc = clampv(0.086 / std::max(0.03, geom_.boreEq), 0.55, 1.8);
        cnStruct_.init(fs);   // diesel combustion noise: structure attenuation (CombustionNoise.h)
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
double Engine::camLift(const ValveSpec& lo, const ValveSpec& hi, double phi, bool intake) const {
    double L = valveLift(lo, phi, intake ? vdIn_ : vdEx_);
    if (vtecBlend_ > 0) {
        // locked onto the high cam: the high rocker drives the valve (its dynamics); the low lobe only matters
        // where its profile is above the high one (kinematically)
        const double H = std::max(valveLift(hi, phi, intake ? vdInHi_ : vdExHi_), lo.liftAt(phi));
        L += vtecBlend_ * (H - L);
    }
    return L;
}

// trigger decoder: the crank wheel's teeth give the speed (one reading per tooth); the angle is only known once
// a missing-tooth gap has passed (a few teeth after it), the cycle phase once the cam tooth has passed after that.
// The gaps sit 90 deg before TDC of cylinder 1 (+ every 360 / gaps), the cam tooth at 600 deg of the 720 deg cycle.
void Engine::triggerStep(double dTheta, double dt) {
    const TriggerInfo& tr = triggerInfo(p_.cal.triggerPattern);
    const double spacing = 360.0 / std::max(1, tr.teeth);
    const double rpmNow = omega_ * 60.0 / (2.0 * kPi);
    // speed from the tooth period
    toothAcc_ += std::fabs(dTheta);
    toothT_ += dt;
    if (toothAcc_ >= spacing) { ckpRpm_ = toothAcc_ / std::max(1e-6, toothT_) / 6.0; toothAcc_ = 0; toothT_ = 0; }
    else if (toothT_ > 0.25) ckpRpm_ = std::min(ckpRpm_, toothAcc_ / toothT_ / 6.0);   // stopping: no more teeth
    if (rpmNow < 15.0) {   // stopped: the decoder starts over
        syncArmed_ = crankSync_ = camSync_ = false;
        return;
    }
    if (!syncArmed_) {
        syncArmed_ = true; syncTravel_ = 0;
        const double confirm = 2.0 * spacing;                // a couple of teeth to recognise the gap
        constexpr double kCamTooth = 600.0;
        const double toCam = wrapPos(kCamTooth - theta_, 720.0);
        if (tr.camRef) syncNeedCrank_ = toCam + confirm;
        else syncNeedCrank_ = wrapPos(270.0 - theta_, 360.0 / std::max(1, tr.gaps)) + confirm;
        syncNeedCam_ = toCam + confirm;
        while (syncNeedCam_ < syncNeedCrank_) syncNeedCam_ += 720.0;
        if (tr.camRef) syncNeedCam_ = syncNeedCrank_;
    }
    syncTravel_ += std::fabs(dTheta);
    if (!crankSync_ && syncTravel_ >= syncNeedCrank_) { crankSync_ = true; lastSyncDeg = syncTravel_; }
    // the cycle phase: from the cam tooth (a rotor's eccentric shaft carries the whole cycle per wheel turn)
    camSync_ = crankSync_ && (geom_.rotary || (p_.cal.camSensor && syncTravel_ >= syncNeedCam_));
}

// valvetrain dynamics: the valve motion of each cam profile at the current speed, recomputed when the speed
// (or the profile) has moved on. Below ~1500 rpm every valvetrain follows its cam.
void Engine::updateValveDynamics(double rpm) {
    if (geom_.rotary || rpm < 1500.0) {
        vdIn_.clear(); vdEx_.clear(); vdInHi_.clear(); vdExHi_.clear();
        floatIn = floatEx = 0;
        return;
    }
    auto upd = [rpm](ValveDynamics& d, const ValvetrainSpec& s, const ValveSpec& v) {
        if (!d.active() || std::fabs(rpm - d.rpmKey) > 0.015 * rpm || d.liftKey != v.lift || d.durKey != v.dur)
            d.compute(s, v.lift, v.dur, rpm);
    };
    upd(vdIn_, vtIn_, vin_);
    upd(vdEx_, vtEx_, vex_);
    if (p_.vtec) { upd(vdInHi_, vtIn_, vinHi_); upd(vdExHi_, vtEx_, vexHi_); }
    else { vdInHi_.clear(); vdExHi_.clear(); }
    const double b = p_.vtec ? vtecBlend_ : 0.0;
    floatIn = (1.0 - b) * vdIn_.severity(vin_.lift) + b * vdInHi_.severity(vinHi_.lift);
    floatEx = (1.0 - b) * vdEx_.severity(vex_.lift) + b * vdExHi_.severity(vexHi_.lift);
}

void Engine::apply(const EngineParams& p) {
    p_ = p;
    p_.cylinders = nPorts_;
    diesel_ = isDiesel(p_);
    CylGeom old = geom_;
    geom_.set(p_);
    computePhases(p_, phase_);
    updateValves();
    comp_.build(p_);
    vtIn_ = valvetrainSpec(p_, true); vtEx_ = valvetrainSpec(p_, false);
    vdIn_.clear(); vdEx_.clear(); vdInHi_.clear(); vdExHi_.clear();
    baroPa_ = ambientPressurePa(p_);
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
    fuelSetup(false);
}

// the tank: filled with the chosen fuel on a new engine; choosing another fuel drains and refills it; a refuel
// request adds litres of a fuel to what is in it (mixing by volume)
void Engine::fuelSetup(bool fresh) {
    const int type = clampv((int)p_.fuelType, 0, kFuelTypeCount - 1);
    if (fresh) {
        tank_ = line_ = fuelTypeBlend(type);
        tankL_ = 30.0;
        fuelTypeSeen_ = type; refuelSeen_ = p_.refuelRequest;
    } else {
        if (type != fuelTypeSeen_) { tank_ = fuelTypeBlend(type); fuelTypeSeen_ = type; }
        if (p_.refuelRequest != refuelSeen_) {
            refuelSeen_ = p_.refuelRequest;
            const double add = clampv((double)p_.refuelLitres, 0.0, kTankCapL - tankL_);
            tank_ = mixBlends(tank_, tankL_, fuelTypeBlend(p_.refuelType), add);
            tankL_ += add;
        }
    }
    tankProps_ = fuelProps(tank_);
    fuel_ = fuelProps(line_);
    intake.setFuel(fuel_);
}

// fuel line + rail (~1 L): the pump circulates it through the return line, so a new fuel in the tank reaches the
// injectors over about half a minute; the engine draws from the tank
void Engine::fuelStep(double dt, double fuelKgs) {
    const double usedL = std::max(0.0, fuelKgs) / std::max(0.5, fuel_.density) * dt;
    tankL_ = std::max(0.5, tankL_ - usedL);
    const double lineL = 1.0;
    const double flowL = (p_.ignition ? 120.0 / 3600.0 * dt : 0.0) + usedL;   // circulation (pump on) + consumption
    if (flowL > 0) {
        line_ = mixBlends(line_, std::max(0.0, lineL - flowL), tank_, std::min(flowL, lineL));
        fuel_ = fuelProps(line_);
        intake.setFuel(fuel_);
    }
}

// one ECU loop: electrical / fuel / thermal physics at this rate, sensors, then the controller
void Engine::ecuTick(double h, double pedal, double rpm) {
    const double mapPa = intake.map();
    const double fuelKgs = intake.fuelFlow();
    elec.step(h, rpm, p_.ignition != 0, p_.starter != 0);
    rail.step(h, fuelKgs / fuel_.density * 3600.0, (mapPa - kPatm) / 1e5, p_.ignition != 0, p_);
    fuelStep(h, fuelKgs);
    ThermalInputs ti;
    ti.fuelPowerW = fuelKgs * fuel_.lhv;
    ti.frictionPowerW = lastFricW_;
    ti.airflowKgs = std::max(0.0, intake.airFlow());
    ti.speedMs = coolingAirMs();
    ti.rpm = rpm;
    ti.ambientC = p_.ambientTemp_C;
    thermal.step(h, ti, p_);
    boost.setCoolingAir(coolingAirMs());
    updateValveDynamics(rpm);
    intake.setInletTemp(thermal.iatC + 273.15);
    intake.setColdWalls(thermal.coldFactor());
    // the actual injector flow / dead time follow the rail pressure and battery voltage
    // an injector flows about the same volume of any of these fuels (the mass follows the density)
    const double rated = p_.injector_ccmin * fuel_.density * 1e-3 / 60.0;
    intake.setSprayRate(rated * std::sqrt(std::max(0.0, rail.deltaBar((mapPa - kPatm) / 1e5)) / 3.0));

    SensorTruth t;
    t.v[kSnsTPS] = intake.throttlePosition() * 100.0;
    t.v[kSnsAPP] = clampv(pedal, 0.0, 1.0) * 100.0;
    t.v[kSnsMAP] = mapPa / 1000.0;
    t.v[kSnsMAF] = std::max(0.0, intake.airFlow()) * 1000.0;
    t.v[kSnsIAT] = isBoosted(p_) ? intake.inlet(0).T - 273.15 : thermal.iatC;
    t.v[kSnsECT] = thermal.coolantC;
    // the lambda sensors read the gas in the pipe (collector, or downpipe after a turbo): misfires, fresh air
    // blown through on the overlap and rich-burn products all show up as they would on a real sensor
    t.v[kSnsO2] = t.v[kSnsWideband] = lamPipe_;
    {
        const double base = 0.015 + 25.0 / std::max(300.0, rpm);
        t.lamDelay = (p_.widebandPos && exhaust.hasTurbine()) ? 1.5 * base + 0.02 : base;
    }
    // EMAP: the exhaust manifold before the turbine; without a turbo the mean of the exhaust ports
    if (exhaust.hasTurbine()) emapKPa_ = exhaust.turbineIn().p() * 1e-3;
    else {
        double s = 0;
        for (int i = 0; i < exhaust.portCount(); ++i) s += exhaust.portPressure(i);
        emapKPa_ = s / std::max(1, exhaust.portCount()) * 1e-3;
    }
    t.v[kSnsEMAP] = emapKPa_;
    t.v[kSnsEGT2] = egtPost_ - 273.15;
    for (int i = 0; i < nPorts_ && i < kMaxCyl; ++i) t.egtCyl[i] = egtCylProbe_[i] - 273.15;
    t.v[kSnsCKP] = ckpRpm_;
    t.in.crankSync = crankSync_;
    t.in.camSync = camSync_;
    t.v[kSnsCMP] = 1.0;
    t.v[kSnsKnock] = knockMax;
    for (int i = 0; i < (int)cyls_.size(); ++i) {
        const int port = clampv(portOf_[i], 0, kMaxCyl - 1);
        t.knockCyl[port] = std::max(t.knockCyl[port], cyls_[i].knock);
        t.knockSeq[port] += cyls_[i].cycles;
    }
    t.v[kSnsEGT] = egtProbe_ - 273.15;
    t.v[kSnsOilP] = oilBar;
    t.v[kSnsOilT] = thermal.oilC;
    t.v[kSnsFuelP] = rail.pressureBar;
    t.v[kSnsVSS] = vehSpeed_ * 3.6;
    t.v[kSnsGear] = vehGear_;
    t.v[kSnsBattery] = elec.volts;
    t.v[kSnsFlex] = fuel_.alcoholPct;
    t.in.clutch = p_.clutchPedal > 0.5f;
    t.in.transbrake = p_.transbrake != 0 && p_.transMode != TransMode::Manual;
    t.in.n2oArm = p_.n2oArm != 0;
    t.in.rollingAls = p_.rollingAlsButton != 0;
    t.in.shift = shifting_;
    t.in.misfireCount = misfires;
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
    const double rated = p_.injector_ccmin * fuel_.density * 1e-3 / 60.0;
    double flow = rated * std::sqrt(std::max(0.0, rail.deltaBar((intake.map() - kPatm) / 1e5)) / 3.0);
    double dead = std::max(0.0, (double)p_.comp.injDeadTime_ms) * 1e-3 * (1.0 + 0.12 * (14.0 - elec.volts));
    intake.injectPulse(port, pw, cyc, flow, dead);
}

// diesel (Package 4): the ECU's fuel quantity is injected straight into the cylinder from its start of injection at
// the injector's rate; the injected fuel lights after the ignition delay (Hardenberg-Hase, integrated over the
// changing charge state - Fuel.h); what was injected during the delay burns premixed, the rest in the mixing-
// controlled diffusion and tail phases (Watson-type shapes, Fuel.h), never faster than it is injected. A charge
// that has not lit by exhaust opening leaves unburned (a misfire: white smoke).
void Engine::dieselStep(Cylinder& c, double dt, double phiPrev, double phi, double rpm, bool exOpen, bool fuelCut,
                        double& dm, double& dmf, double& dmb, double& dmbNew, double& dU) {
    // start of injection
    if (crossed(phiPrev, phi, wrapPos(-ecu.cmd.soi, cycle_))) {
        const double q = (p_.ignition && !fuelCut && !failure && !ecu.cmd.syncLost) ? std::max(0.0, ecu.cmd.dieselFuelMg) * 1e-6 : 0.0;
        c.injLeft = q; c.injDone = 0;
        c.injRate = std::max(1.0, (double)p_.dieselInjRate) * 1e-3;   // mg/ms -> kg/s
        c.idI = 0; c.idOpen = q > 0; c.dBurning = false; c.dBurned = 0; c.soiPhi = phi; c.ringKicked = false; c.dpStep = 0;
        c.dPre = c.dDiff = c.dTail = 0;
    }
    if (c.injLeft > 0) {
        const double q = std::min(c.injLeft, c.injRate * dt);
        c.injLeft -= q; c.injDone += q;
        dm += q; dmf += q;
        // the spray evaporates in the hot air: its latent heat comes out of the charge (rail fuel taken at 330 K)
        dU += q * (thermo::enthalpy(0.0, c.yb, 330.0) - fuel_.hvap);
    }
    // ignition delay: integral of dt / tau over the charge state since the start of injection
    if (c.idOpen && c.injDone > 0) {
        if (exOpen) {   // never lit: the fuel leaves unburned
            c.idOpen = false; c.misfired = true; c.burnEta = 0;
            ++misfires; ++misfireCause[kMisLean];
        } else {
            const double Sp = 2.0 * geom_.strokeEq * std::max(rpm, 30.0) / 60.0;
            const double tauDeg = dieselDelayDeg(Sp, c.T, c.p * 1e-5, fuel_.cetane);
            // (cold: the charge alone lights only above the cold limit, otherwise a hot glow plug has to - Fuel.h)
            if (c.T >= kDieselColdIgnK || glowC_ >= kGlowIgnC) c.idI += dt * 6.0 * std::max(rpm, 30.0) / tauDeg;
            if (c.idI >= 1.0) {
                c.idOpen = false; c.dBurning = true; c.socPhi = phi;
                c.idDeg = wrapPos(phi - c.soiPhi, cycle_);
                const double total = c.injDone + c.injLeft;
                const double air = std::max(0.0, c.m + dm - (c.mf + dmf) - (c.mb + dmb));
                c.dPhi = std::min(0.85, total * fuel_.stoich / std::max(1e-12, air));
                c.dPre = c.injDone;                     // injected during the delay: premixed
                c.dTail = c.injLeft * 0.6 * c.dPhi * c.dPhi;
                c.dDiff = c.injLeft - c.dTail;
                const thermo::Burn bp = thermo::burn(fuel_.molC, fuel_.molH, fuel_.molO, air / std::max(1e-12, total));
                c.airPerFuel = bp.airUsed;
                c.yNew = bp.products;
                c.qTotal = std::max(0.0, fuel_.lhv - bp.energyLeft) * 0.97;   // J per kg of fuel burned (as the SI burn)
                c.lambda = air / std::max(1e-12, total * fuel_.stoich);
                c.fired = true; c.misfired = false; c.burnEta = 1.0;
                fireTimer_ = 0;
            }
        }
    }
    if (c.dBurning) {
        const double d = wrapPos(phi - c.socPhi, cycle_);
        const double t = d / 125.0;
        const double cd3 = 0.055 / (1.0 + 0.5 * c.dPhi), ct3 = 3.7 * cd3 / (1.0 + 1.12 * c.dPhi);
        double target = c.dPre * wiebeWave(0.75, 2.0, t) + c.dDiff * wiebeWave(cd3, 1.75, t) + c.dTail * wiebeWave(ct3, 2.5, t);
        target = std::min(target, c.injDone);                       // nothing burns before it is injected
        double fb = std::min(std::max(0.0, target - c.dBurned), std::max(0.0, c.mf + dmf));
        c.dBurned += fb;
        dU += fb * c.qTotal;
        dmf -= fb;
        dmb += fb * (1.0 + c.airPerFuel);
        dmbNew += fb * (1.0 + c.airPerFuel);
        if (exOpen || d > 170.0) c.dBurning = false;   // what is left goes out with the exhaust
    }
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
            double ff = src.mf / src.m, bf = src.mb / src.m, h = q * thermo::enthalpy(bf, src.yb, src.T);
            src.m -= q; src.mf -= q * ff; src.mb -= q * bf; src.U -= h;
            dst.m += q; dst.mf += q * ff; dst.mb += q * bf; dst.U += h;
            for (Cylinder* cc : {&src, &dst}) {
                cc->solveT();
                cc->p = cc->m * kR * cc->T / cc->V;
            }
        }
    }
}

// valve lift including the valvetrain dynamics: past the float rpm the spring can't keep the follower on the
// cam - the valve lofts, closes late and bounces off its seat (see Valvetrain.h)
double Engine::valveLift(const ValveSpec& v, double phi, const ValveDynamics& dyn) const {
    const double L = v.liftAt(phi);
    if (!dyn.active()) return L;
    return std::max(0.0, L + dyn.dev(wrapSym(phi - v.center, 720.0)));
}

double Engine::burnDuration(Cylinder& c, double rpm, double lambda) {
    // base: the turbulence (u' grows with the piston speed) - burn angle rises slowly with rpm (x 1.12: the base was
    // set together with the old lambda / residual factors, which came to 1.12 at the full-load reference state)
    double D = 1.12 * (47.0 + 18.0 * clampv(rpm / 7000.0, 0.0, 1.3));
    // the mixture: laminar burning velocity at the spark (unburned gas, cylinder pressure, burned-gas dilution) sets
    // the kernel and the turbulent flame speed (Flame.h) - lean, rich, diluted (residual at low load), hot, cold,
    // high pressure and the fuel all act through it
    const double resid = c.m > 0 ? c.mb / c.m : 0;
    c.slSpark = laminarFlameSpeed(fuel_, 1.0 / std::max(0.2, lambda), c.tEnd, c.p, resid);
    c.tuSpark = c.tEnd; c.pSpark = c.p;
    D *= flameDurationFactor(c.slSpark, c.tEnd, c.p);
    // cycle-to-cycle variation: larger in a diluted, slow-burning charge
    const double cov = 0.05 + 0.10 * clampv(1.0 - c.pIvc / kPatm, 0.0, 0.8);
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
    double dm = 0, dmf = 0, dmb = 0, dmbNew = 0;   // dmbNew: products made by the burn this step
    c.work += c.p * dV;

    // ---- valves
    double Ai, Ae;
    if (geom_.rotary) { Ai = pin_.areaAt(phi); Ae = pex_.areaAt(phi); }
    else { Ai = vin_.area(camLift(vin_, vinHi_, phi, true)); Ae = vex_.area(camLift(vex_, vexHi_, phi, false)); }
    // no two ports flow the same (casting, seat, valve-to-guide spread)
    const double imp = effImperfection(p_);
    Ai *= 1.0 + 0.02 * imp * sprPort_[0][port];
    Ae *= 1.0 + 0.02 * imp * sprPort_[1][port];
    bool inOpen = Ai > 0, exOpen = Ae > 0;
    if (exOpen && !c.prevExhaustOpen) { c.tEvo = c.T; c.fired = false; }
    double ffC = c.m > 0 ? c.mf / c.m : 0, bfC = c.m > 0 ? c.mb / c.m : 0;

    if (Ai > 0) {
        double f = limitedFlow(Ai, ip.p(), ip.T, ip.V, c.p, c.T, c.V, dt, 0.35);
        double q = f * dt;
        if (q > 0) {
            q = std::min(q, ip.m * 0.5);
            flInF_ += q;
            double ff = ip.fuelFrac(), bf = ip.burnedFrac();
            ip.m -= q; ip.mf -= q * ff; ip.mb -= q * bf;
            dm += q; dmf += q * ff; dmb += q * bf;
            dU += q * thermo::enthalpy(bf, c.yb, ip.T);
            // charge cooling: the fuel evaporating on the way in takes part of its latent heat from the air (the
            // rest from the port walls / valve) - small for gasoline (~9 K), large for alcohols (E85 ~25 K, methanol
            // ~60 K): a denser, cooler charge that resists knock. Liquid that stays on cold walls takes nothing.
            dU -= kChargeCoolShare * fuel_.hvap * q * ff * (1.0 - fuel_.coldVap * thermal.coldFactor());
        } else if (q < 0) {
            q = std::min(-q, c.m * 0.3);
            flInB_ += q;
            ip.m += q; ip.mf += q * ffC; ip.mb += q * bfC;
            dm -= q; dmf -= q * ffC; dmb -= q * bfC;
            dU -= q * thermo::enthalpy(bfC, c.yb, c.T);
        }
    }
    if (Ae > 0) {
        double f = limitedFlow(Ae, c.p, c.T, c.V, ep.p(), ep.T, ep.V, dt, 0.35);
        double q = f * dt;
        if (q > 0) {
            q = std::min(q, c.m * 0.3);
            ep.m += q; ep.mf += q * ffC; ep.mb += q * bfC;
            dm -= q; dmf -= q * ffC; dmb -= q * bfC;
            exhOutKg_ += q;
            flExF_ += q;
            dU -= q * thermo::enthalpy(bfC, c.yb, c.T);
            // mass-weighted over the whole exhaust event (weighting by what is left in the cylinder made the last,
            // cool overlap gas count most: a full-load EGT came out hundreds of K too low)
            c.exhH += q * c.T; c.exhM += q;
            const double airOut = q * std::max(0.0, 1.0 - ffC - bfC);
            // what the wideband will see of this gas: oxygen (free air, and the air bound in the combustion products)
            // against fuel (the fuel bound in the products, and unburned fuel - which diffuses through the sensor's
            // barrier more slowly than oxygen, so a misfire reads lean). The products carry the mixture they came
            // from: a rich charge leaves CO / H2 that still lack their oxygen (lean: the excess air is free air)
            {
                const double lp = clampv(c.lambda, 0.4, 1.0), st = fuel_.stoich;
                const double mbOut = q * bfC;
                wbAirKg_ += airOut + mbOut * lp * st / (1.0 + lp * st);
                wbFuelKg_ += kHcSensed * q * ffC + mbOut / (1.0 + lp * st);
            }
            if (c.burning) {
                // the flame is still going when the exhaust valve opens (late / slow burn at high rpm): the mixture
                // that leaves keeps burning in the port as a continuous flame - heat for the exhaust, not a pop
                const double b = std::min(q * ffC, airOut / fuel_.stoich);
                afHeatJ_ += b * fuel_.lhv * 0.9;
                c.afterfireFuel += q * ffC - b;
                c.afterfireAir += airOut - b * fuel_.stoich;
            } else {
                c.afterfireFuel += q * ffC;
                c.afterfireAir += airOut;   // fresh air (misfired / lean / cut charge)
            }
        } else if (q < 0) {
            q = std::min(-q, ep.m * 0.5);
            double ff = ep.fuelFrac(), bf = ep.burnedFrac();
            ep.m -= q; ep.mf -= q * ff; ep.mb -= q * bf;
            dm += q; dmf += q * ff; dmb += q * bf;
            exhOutKg_ -= q;   // (net flow out of the cylinders)
            flExB_ += q;
            dU += q * thermo::enthalpy(bf, c.yb, ep.T);
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
        if (!diesel_) c.lambda = c.mf > 1e-9 ? air / (c.mf * fuel_.stoich) : 9.99;   // (diesel: set at the start of combustion)
        double rho = kPatm / (kR * ambT_);
        c.ve = air / (rho * geom_.Vd);
        c.pIvc = c.p; c.tIvc = c.T; c.vIvc = c.V;
        c.lwI = 0; c.lwOpen = true; c.tEnd = c.T; c.knockCycle = 0; c.knockPCycle = 0;
    }
    c.prevIntakeOpen = inOpen;
    if (!exOpen && c.prevExhaustOpen && c.exhM > 1e-9) {   // exhaust valve closed: this cycle's exhaust temperature
        c.egt += (c.exhH / c.exhM - c.egt) * 0.6;
        c.exhH = c.exhM = 0;
    }
    c.prevExhaustOpen = exOpen;

    // ---- diesel: direct injection, ignition delay, premixed / diffusion / tail burn (Package 4; Fuel.h)
    if (diesel_) dieselStep(c, dt, phiPrev, phi, rpm, exOpen, fuelCut, dm, dmf, dmb, dmbNew, dU);

    // ---- injection event (EFI)
    if (!diesel_) {
        const double ia = wrapPos(p_.injectionAngle, cycle_);
        const int mode = geom_.rotary ? kInjSequential : ecu.cmd.injMode;
        if (mode == kInjSequential) {
            if (crossed(phiPrev, phi, ia)) injectPort(port, rpm, 1.0);
        } else {
            // semi-sequential: each port at its own angle, twice per cycle (half each - the ECU doesn't know which
            // revolution is the intake one); batch: every injector at once, at an angle of cylinder 1's cycle
            const double sh = mode == kInjBatch ? c.phase - cyls_[0].phase : 0.0;
            const double a = wrapPos(phiPrev + sh, cycle_), b = wrapPos(phi + sh, cycle_);
            if (crossed(a, b, ia)) injectPort(port, rpm, 0.5);
            if (crossed(a, b, wrapPos(ia + 360.0, cycle_))) injectPort(port, rpm, 0.5);
        }
    }

    // ---- spark
    const double sparkAt = wrapPos(-(advance_ + ecu.cylSpark(port) - ecu.cmd.cylRetard[clampv(port, 0, kMaxCyl - 1)]) + 0.5 * imp * sprCyl_[0][i], cycle_);
    if (!diesel_ && crossed(phiPrev, phi, sparkAt) && !c.burning) {
        // soft limiter: the ECU drops a share of the sparks
        if (spark && !(ecu.cmd.cutFraction > 0 && rng_.uniform() < ecu.cmd.cutFraction)) {
            double air = std::max(0.0, c.m - c.mf - c.mb);
            // only the vaporised fuel takes part (cold: part of it is still liquid on the walls)
            // (alcohols: single boiling point and a large heat of vaporisation - far more stays liquid when cold)
            const double vapF = 1.0 - fuel_.coldVap * thermal.coldFactor();
            double lam = c.mf > 1e-10 ? air / (c.mf * vapF * fuel_.stoich) : 99.0;
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
            // dilution: burned gas in the charge (residual, exhaust reverted through a big overlap). Past the dilution
            // limit the flame kernel fails. This is a threshold with a small cycle-to-cycle scatter (mixing at the
            // plug), not a coin toss: near the limit the cycles couple through the residual they leave - a misfire
            // leaves fresh mixture (the next charge is less diluted and fires hard), a strong burn leaves hot
            // products (the next one misfires). That deterministic feedback with a little noise (Daw, Finney et al.,
            // ORNL, SAE 962086; the same mechanism as a two-stroke's "four-stroking") gives the rhythmic lope of
            // big-overlap engines - the bridge / peripheral port rotary "brap" - instead of a random sputter. (Limit
            // 0.475 = the middle of the earlier 0.35-0.60 probability ramp, so the average dilution tolerance is
            // unchanged.) Tried and left out (ECU_PLAN, "Brap tahap 2"): the paper's continuous combustion efficiency
            // (partial burns) broke up the long misfire runs whose rpm swings make the slow brap, and popping the
            // misfire pocket on the next firing - both sounded worse than this.
            c.dilution = resid;
            if (resid + 0.03 * rng_.gauss() > 0.475) pc[kMisDilution] = 1.0;
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
                // the vaporised fuel burns (cold: part of it is still liquid on the walls); a rich charge uses all its
                // oxygen and leaves CO and H2, whose chemical energy is not released (Thermo.h)
                const double full = c.mf * vapF;
                const thermo::Burn bp = thermo::burn(fuel_.molC, fuel_.molH, fuel_.molO, full > 1e-12 ? air / full : 99.0);
                c.burnEta = 1.0;
                c.fuelBurn = full;
                c.airPerFuel = bp.airUsed;
                c.yNew = bp.products;
                // combustion efficiency 0.97: unburned hydrocarbons (crevices, quench layers) - Heywood gives 95-98 %
                // for lambda >= 1; the rich-mixture losses (CO / H2) are counted separately above
                c.qTotal = c.fuelBurn * std::max(0.0, fuel_.lhv - bp.energyLeft) * 0.97 *
                           (1.0 + 0.015 * imp * sprCyl_[1][i]); // efficiency spread
                c.burnDur = burnDuration(c, rpm, lam) * (1.0 - 0.06 * clampv(((double)p_.comp.plugGap_mm - 0.8) / 0.3, -1.0, 1.5)) *
                            (1.0 + 0.03 * imp * sprCyl_[2][i]) *
                            (1.0 + 0.01 * imp * rng_.gauss());                    // extra cycle-to-cycle variation
                c.burnStart = wrapPos(sparkAt + 0.25 * imp * rng_.gauss(), cycle_);
                c.xPrev = 0;
                c.burning = true;
                c.fired = true;
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
        dmb += fb * (1.0 + c.airPerFuel);
        dmbNew += fb * (1.0 + c.airPerFuel);
        if (c.burnEnded) {
            // fuel left over after a rich burn ends up as CO/HC products, not as fuel
            // for the next cycle (no rich run-away through the residual gas)
            c.burnEnded = false;
            // fuel left over after a rich burn ends up as CO/HC products, not as fuel
            // for the next cycle (no rich run-away through the residual gas)
            double left = std::max(0.0, c.mf + dmf);
            dmf -= left * 0.9;
            dmb += left * 0.9;
            dmbNew += left * 0.9;
        }
        if (x >= 0.9) closeKnockWindow(c);   // the end gas is consumed
    }

    // ---- end-gas autoignition (Livengood-Wu, Douaud-Eyzat delay): the unburned charge compressed isentropically
    //      from intake closing (variable cp), heated by a glowing plug tip and hot coolant / walls
    if (c.lwOpen && c.pIvc > 0 && !diesel_) {
        c.tEnd = thermo::isentropicAir(c.tIvc, std::max(1.0, c.p / c.pIvc), c.tEnd);
        double tu = c.tEnd;
        tu += 3.0 * std::max(0.0, plugTempC - 850.0);    // surface ignition from a too-hot plug (pre-ignition)
        tu += 0.6 * (thermal.coolantC - 90.0);           // hot coolant / walls
        if (tu > 650.0) {
            const double on = fuel_.octaneIndex(knockOnK(c.pIvc / 101325.0));
            c.lwI += dt / knockDelayS(on, c.p, tu);
        }
        if (tu > c.knockCycle) { c.knockCycle = tu; c.knockPCycle = c.p; }
        if (exOpen) closeKnockWindow(c);   // no burn this cycle (misfire / cut): closes at exhaust opening
    }

    // ---- wall heat transfer: Woschni (SAE 670931), h = 3.26 B^-0.2 p^0.8 T^-0.55 w^0.8 (B m, p kPa, T K, w m/s),
    //      w = C1 Sp + C2 (Vd Tr) / (pr Vr) (p - pm): C1 = 6.18 while a valve is open (gas exchange), 2.28 with the
    //      valves shut; C2 = 3.24e-3 m/(s K) from combustion to exhaust opening (0 during compression). pm = motored
    //      pressure from the state at intake closing (polytropic n = 1.32). (+0.5 m/s: an engine at rest still
    //      exchanges some heat.)
    {
        const double Sp = 2.0 * geom_.strokeEq * rpm / 60.0;
        double w = (inOpen || exOpen ? 6.18 : 2.28) * Sp + 0.5;
        if (c.fired && !inOpen && !exOpen && c.vIvc > 0 && c.pIvc > 0) {
            const double pm = c.pIvc * std::pow(c.vIvc / c.V, 1.32);
            w += 3.24e-3 * geom_.Vd * c.tIvc / (c.pIvc * c.vIvc) * std::max(0.0, c.p - pm);
        }
        double h = 3.26 * std::pow(geom_.boreEq, -0.2) * std::pow(std::max(1.0, c.p / 1000.0), 0.8) *
                   std::pow(std::max(200.0, c.T), -0.55) * std::pow(w, 0.8);
        dU -= h * wallArea * (c.T - (thermal.coolantC + 273.15 + 87.0)) * dt;
    }

    // ---- integrate
    // burned-gas composition: the new products mix with the burned gas already there (residual / reverted exhaust)
    if (dmbNew > 0) {
        const double w = dmbNew / std::max(1e-12, c.mb + dmbNew);
        for (int k = 0; k < kSpCount; ++k) c.yb.y[k] += (c.yNew.y[k] - c.yb.y[k]) * w;
    }
    c.U += dU;
    c.m = std::max(1e-8, c.m + dm);
    c.mf = clampv(c.mf + dmf, 0.0, c.m);
    c.mb = clampv(c.mb + dmb, 0.0, c.m - c.mf);
    c.V = Vn;
    c.solveT();
    if (c.T <= thermo::kTmin + 1e-6 || c.T >= thermo::kTmax - 1e-6) c.setT(c.T);   // (out of the table range)
    double pPrev = c.p;
    c.p = c.m * kR * c.T / c.V;
    c.pmax = std::max(c.pmax, c.p);
    {
        const double dTh = wrapPos(phi - phiPrev, cycle_);
        const double dp = dTh > 1e-6 && dTh < 10.0 ? (c.p - pPrev) / dTh : 0.0;
        c.dpMax = std::max(c.dpMax, dp);
        // diesel: the sudden premixed burn excites the gas in the chamber at its acoustic modes (ringing): peak-to-peak
        // amplitude beta (dP/dt)max at the first tangential mode (CombustionNoise.h). Only for the sound - the 0-D
        // thermodynamics stay as they are.
        // (the peak is taken as the highest rate since the start of combustion, once the rate has fallen to half of it
        //  - step-to-step jitter right at ignition must not count as the peak)
        if (diesel_ && c.dBurning && !c.ringKicked) {
            c.dpStep = std::max(c.dpStep, dp);
            if (c.dpStep > 0 && dp < 0.5 * c.dpStep) {
                c.ringA = 0.5 * kRingBeta * c.dpStep * 6.0 * std::max(rpm, 60.0);   // half the peak-to-peak, Pa
                c.ringKickA = c.ringA;
                c.ringPh = 0;
                c.ringKicked = true;
            }
        }
    }
    if (c.ringA > 1.0) {
        c.ringPh += 2.0 * kPi * ringFrequency(geom_.boreEq, c.T, kR) * dt;
        c.ringA *= std::exp(-dt / kRingDecay);
    } else c.ringA = 0;

    // ---- per-cycle bookkeeping (at the cycle's wrap: 0 deg = firing TDC). The peaks are taken over the half cycle
    //      either side of the wrap (combustion straddles firing TDC): read at bottom dead centre of the power stroke
    if (phi < phiPrev) {
        c.imep = c.work / geom_.Vd;
        c.work = 0;
    }
    if (crossed(phiPrev, phi, 0.5 * cycle_)) {
        c.pmaxCycle = c.pmax; c.pmax = 0;
        c.dpMaxCycle = c.dpMax; c.dpMax = 0;
    }

    // torque on the crank
    torqueInst_ += (c.p - kPatm) * dVdth;
    // structure-borne combustion noise
    blockIn_ += (c.p - pPrev) * 1.0e-6;          // combustion pressure rise shakes the block
    if (c.valveSeatEvent) tickIn_ += 0.6 * (0.3 + rpm / 3000.0);
}

void Engine::step(double dt, double pedal, double loadTorque, bool signedLoad) {
    kPatm = baroPa_;   // ambient pressure of this engine (per thread)
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
    sparkAvail = comp_.sparkEnergy(p_, rpm, ecu.cmd.dwellMs, elec.volts);
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
    // glow plug tip (Bosch glow plug brochure: metal 850 C in under 5 s, max 1150 C; ceramic 1000 C in under 2 s, max
    // 1300 C): first order towards its maximum while powered (time constants from those heat-up figures from 20 C),
    // back towards the coolant temperature when off
    if (diesel_ && p_.glowPlugs) {
        const bool ceramic = p_.glowPlugs == 2;
        const double tMax = ceramic ? 1300.0 : 1150.0, tau = ceramic ? 1.38 : 3.77;
        const double tgt = (ecu.cmd.glow && elec.volts > 8.0) ? tMax : thermal.coolantC;
        glowC_ += (tgt - glowC_) * (1.0 - std::exp(-dt / tau));
    } else glowC_ = thermal.coolantC;
    double scTorque = isBoosted(p_) ? boost.step(dt, omega_, egtAvg, intake, exhaust) : 0.0;
    // ---- exhaust gas recirculation (Package 4b): a valve from the exhaust manifold (before the turbine) into the
    //      plenum. Compressible orifice flow, one way only - Wahlstrom & Eriksson (LiTH-ISY-R-2882 eq. 31-34) clip
    //      the pressure ratio at 1, so no reverse flow; the gas carries its burned share (it dilutes the charge and
    //      takes the place of fresh air the MAF sensor no longer sees). Uncooled: it arrives at the exhaust
    //      temperature and warms the plenum (mass-weighted). Valve area = its open share of the bore; actuator 0.1 s.
    double egrNow = 0;
    if (p_.egrValveDia_mm > 0 && intake.inletCount() > 0) {
        egrPos_ += (ecu.cmd.egr - egrPos_) * std::min(1.0, dt / 0.1);
        GasNode& src = exhaust.hasTurbine() ? exhaust.turbineIn() : exhaust.port(0);
        GasNode& dst = intake.inletNode(0);
        const double d = p_.egrValveDia_mm * 1e-3;
        const double a = clampv(egrPos_, 0.0, 1.0) * 0.25 * kPi * d * d;
        double f = a > 0 ? limitedFlow(a, src.p(), src.T, src.V, dst.p(), dst.T, dst.V, dt, 0.35) : 0.0;
        if (f > 0) {
            const double q = std::min(f * dt, src.m * 0.3);
            const double ff = src.fuelFrac(), bf = src.burnedFrac();
            src.m -= q; src.mf -= q * ff; src.mb -= q * bf;
            dst.m += q; dst.mf += q * ff; dst.mb += q * bf;
            egrNow = q / dt;
        }
    } else egrPos_ = 0;
    egrFlow_ += (egrNow - egrFlow_) * std::min(1.0, dt / 0.1);   // (averaged over the exhaust pulses)
    // uncooled: the plenum takes the flow-weighted mix of the fresh charge and the gas at the exhaust temperature
    intake.setEgr(egrFlow_ / std::max(1e-6, egrFlow_ + std::max(0.0, intake.airFlow())), std::max(400.0, egtAvg));
    intake.stepAir(dt, diesel_ ? 1.0 : pedal, rpm, p_.ignition && !fuelCut);   // a diesel has no throttle

    // ---- cylinders
    const double dTheta = omega_ * dt / kDeg;
    triggerStep(dTheta, dt);
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
        const double burnable = std::min(c.afterfireFuel, c.afterfireAir / fuel_.stoich);
        // a firing cylinder under load fills its port with hot, fast gas: mixture that slips through (scavenging
        // with a big overlap, late burning) burns there as a continuous flame. Pops need a pocket of mixture - an
        // unburned (misfired or cut) charge, or the cooler gas of the overrun.
        // (the hot blowdown gas lights it - the overlap flow itself is cool fresh mixture, so the average temperature
        //  of everything leaving the cylinder would wrongly keep a scavenging engine's slip from ever burning)
        if (!c.misfired && std::max(c.egt, c.tEvo) > 900 && burnable > 0) {
            const double b = burnable * std::min(1.0, dt / 0.004);
            c.afterfireFuel -= b; c.afterfireAir = std::max(0.0, c.afterfireAir - b * fuel_.stoich);
            afHeatJ_ += b * fuel_.lhv * 0.9;
        } else if (burnable > 1.5e-6 && c.egt > 650 && p_.ignition &&
                   // the pocket must be flammable: the trace of fuel the port walls give off during a fuel cut
                   // sits in far too much air to ignite (lean flammability limit ~ lambda 1.7)
                   c.afterfireAir < 1.7 * fuel_.stoich * c.afterfireFuel) {
            double rate = 25.0 * (burnable / 1e-5) * smoothstep(650, 950, c.egt);
            if (rng_.uniform() < rate * dt) {
                double burn = burnable * 0.7;
                c.afterfireFuel -= burn;
                c.afterfireAir = std::max(0.0, c.afterfireAir - burn * fuel_.stoich);
                afHeatJ_ += burn * fuel_.lhv * 0.9;
                GasNode& ep = exhaust.port(portOf_[i]);
                c.popPending += std::min(burn * fuel_.lhv * (kGamma - 1.0) / ep.V * 0.03, 0.7e5) * (0.6 + 0.4 * rng_.uniform());
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
    const double fmepAll = std::max(0.4 * fmep, fmep + comp_.extraFmep(p_, rpm, Sp, perRev)) * thermal.viscosityFriction() *
                           frictionScale(p_);
    double tFric = (fmepAll * perRev / (2.0 * kPi) + 4.0) * std::tanh(omega_ / 3.0);
    if (failure) tFric += 600.0 * std::tanh(omega_ / 3.0);     // broken parts: the engine locks up
    lastFricW_ = tFric * omega_;
    double tStarter = 0;
    if (p_.starter) {
        double tmax = 60.0 + 200.0 * geom_.Vd * 1000.0 + 15.0 * nPorts_;
        // the starter is sized for the compression it has to push through (peak compression pressure ~ CR^1.3):
        // a diesel's is far stronger than a petrol engine's of the same size
        tmax *= std::max(1.0, std::pow(std::max(1.0, (double)p_.compression) / 10.0, 1.3));
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
        // the gas actually leaving the cylinders (the throttle flow stops at once on a lift-off, the exhaust flow
        // does not), averaged like the afterfire heat below so that the two always describe the same gas
        exhFlowAvg_ += (exhOutKg_ / std::max(1e-6, dt) - exhFlowAvg_) * std::min(1.0, dt / 0.15);
        exhFlowAvg_ = std::max(0.0, exhFlowAvg_);
        exhOutKg_ = 0;
        const double D = clampv((double)p_.primaryDia_mm, 20.0, 90.0) * 1e-3;
        const double mPort = std::max(1e-5, exhFlowAvg_ / std::max(1, nPorts_));
        const double Re = 4.0 * mPort / (kPi * D * 4.0e-5);
        const double Nu = 1.8 * std::max(14.0, 0.023 * std::pow(Re, 0.8) * 0.9);
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
        // (a burning pocket cannot heat the gas beyond a stoichiometric flame of it: a few hundred K at most)
        Tport += std::min(700.0, afPower_ / (std::max(1e-3, exhFlowAvg_) * 1150.0));
        egtAvg += (Tport - egtAvg) * a;

        // EGT sensor: a sheathed thermocouple in the header. Its tip takes heat from the gas by convection (strong in
        // the fast gas at full load, weak in the slow gas at idle / on a lift-off) and loses it by radiation to the
        // cooler pipe wall, which itself warms up slowly. So at idle it reads well below the gas, follows the gas
        // closely at full load, and a short hot burst at low flow barely moves it.
        const double area = kPi * 0.25 * D * D;
        const double G = mPort / area;                                // kg / m^2 s through a primary
        const double hOut = 20.0 + 1.2 * std::min(40.0, vehSpeed_);   // pipe outside: radiation + air
        const double Ta = ambT_;
        auto probe = [&](double Tgas, double Gp, double& Tp, double& Tw, double h) {
            const double hProbe = 800.0 * std::sqrt(std::max(0.02, Gp) / 32.0);
            const double hGasWall = 120.0 * std::pow(std::max(0.02, Gp) / 32.0, 0.8);
            Tw += h * (hGasWall * (Tgas - Tw) - hOut * (Tw - Ta)) / 4200.0;
            const double hRad = 4.0 * 0.8 * 5.67e-8 * std::pow(0.5 * (Tp + Tw), 3.0);
            Tp += h * (hProbe * (Tgas - Tp) - hRad * (Tp - Tw)) / 560.0;
        };
        probe(egtAvg, G, egtProbe_, egtWall_, dt);
        // the other thermocouples move slowly: updated every millisecond
        egtSub_ += dt;
        if (egtSub_ >= 1e-3) {
            const double h = egtSub_;
            egtSub_ = 0;
            // per-cylinder probes in the primaries: each sees its own cylinder's gas after the port
            if (p_.egtPerCyl) {
                double sum[kMaxCyl] = {}; int cnt[kMaxCyl] = {};
                for (int i = 0; i < (int)cyls_.size(); ++i) { const int pt = clampv(portOf_[i], 0, kMaxCyl - 1); sum[pt] += cyls_[i].egt; ++cnt[pt]; }
                const double afRise = std::min(700.0, afPower_ / (std::max(1e-3, exhFlowAvg_) * 1150.0));
                for (int pt = 0; pt < nPorts_ && pt < kMaxCyl; ++pt) {
                    const double Tc = cnt[pt] ? sum[pt] / cnt[pt] : Tcyl;
                    const double Tg = (Tc > Tw ? Tw + (Tc - Tw) * std::exp(-ntu) : Tc) + afRise;
                    probe(Tg, G, egtCylProbe_[pt], egtCylWall_[pt], h);
                }
            }
            // after the turbine: the gas has given up the turbine's work (the wastegate flow mixes back in)
            if (exhaust.hasTurbine()) {
                const double Dd = clampv((double)p_.pipeDia_mm, 30.0, 120.0) * 1e-3;
                const double Gd = exhFlowAvg_ / (kPi * 0.25 * Dd * Dd);
                // and loses heat to the turbine housing / downpipe on the way (a lot when it flows slowly)
                const double mt = std::max(1e-5, exhFlowAvg_);
                const double Ret = 4.0 * mt / (kPi * Dd * 4.0e-5);
                const double Nut = 1.8 * std::max(14.0, 0.023 * std::pow(Ret, 0.8) * 0.9);
                const double ntuT = Nut * 0.06 * kPi * (8.0 * Dd) / (mt * 1150.0);
                const double Tt = std::max(Ta, egtAvg - boost.turbineDropK);
                const double Tpost = egtPostWall_ + (Tt - egtPostWall_) * std::exp(-ntuT);
                probe(std::max(Ta, Tpost), Gd, egtPost_, egtPostWall_, h);
            } else { egtPost_ = egtProbe_; egtPostWall_ = egtWall_; }
        }
    }
    // ---- the gas at the wideband: what left the cylinders, mixed in the collector (or the turbine housing and
    //      downpipe after a turbo) - its residence time is the mixing volume over the volume flow
    {
        const bool post = p_.widebandPos && exhaust.hasTurbine();
        const double disp = geom_.Vd * nPorts_ * (geom_.rotary ? 2.0 : 1.0);
        const double V = post ? 1.0e-3 + 0.6 * disp : 0.3e-3 + 0.25 * disp;
        const double rho = kPatm / (kR * std::max(500.0, egtAvg));
        const double tau = clampv(V * rho / std::max(1e-4, exhFlowAvg_), 0.003, 0.3);
        const double k = std::min(1.0, dt / tau);
        wbA_ += (wbAirKg_ / std::max(1e-9, dt) - wbA_) * k;
        wbF_ += (wbFuelKg_ / std::max(1e-9, dt) - wbF_) * k;
        wbAirKg_ = wbFuelKg_ = 0;
        lamPipe_ = wbF_ > 1e-9 ? clampv(wbA_ / (wbF_ * fuel_.stoich), 0.3, 9.99) : (wbA_ > 1e-9 ? 9.99 : lamPipe_);
    }
    exhaust.setGasTemp(egtAvg);   // pipe sound speed follows the gas temperature
    flowStep(dt);
    knockMax = std::max(kn, knockMax - dt * 0.5);

    // ---- sound
    exhaust.radiated(sExhL, sExhR);
    sIntake = intake.radiated();
    sBoost = isBoosted(p_) ? boost.sound() : 0.0;
    for (const Cylinder& c : cyls_) if (c.knockEvent && c.prevPhi < 40 && c.prevPhi > 5) pingIn_ += 0.05 * c.knock;
    for (Cylinder& c : cyls_) if (c.prevPhi > 40) c.knockEvent = false;
    double mech = head_.tick(tickIn_ * 0.0012);
    if (diesel_) {
        // combustion noise: all cylinder pressures (and their ringing) through the structure attenuation -> Pa
        double ps = 0;
        for (const Cylinder& c : cyls_) ps += c.p - kPatm + c.ringA * std::sin(c.ringPh);
        combNoise_ = cnStruct_.tick(ps);
        mech += combNoise_;
    } else mech += block_.tick(blockIn_ * 0.08);
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

// the end gas is gone (90 % burned, or the exhaust opened without a burn): this cycle's knock index
void Engine::closeKnockWindow(Cylinder& c) {
    if (!c.lwOpen) return;
    c.lwOpen = false;
    c.knock = knockIndexFromIntegral(c.lwI);
    c.knockEvent = c.knock > 1.0;
    c.knockT = c.knockCycle; c.knockP = c.knockPCycle;
    ++c.cycles;   // the ECU's knock window reads one value per cycle
}

// flow map (Package 3): what moves through each section of the gas path and the state of its volumes, averaged for
// display - forward and reverse flow kept apart so that a flow pulsing back and forth shows both directions
void Engine::flowStep(double dt) {
    const double kF = std::min(1.0, dt / 0.15), kP = std::min(1.0, dt / 0.1);
    FlowState& f = flow_;
    auto both = [&](int seg, double signedKgs) {
        f.fwd[seg] += (std::max(0.0, signedKgs) - f.fwd[seg]) * kF;
        f.back[seg] += (std::max(0.0, -signedKgs) - f.back[seg]) * kF;
    };
    auto split = [&](int seg, double fwdKgs, double backKgs) {
        f.fwd[seg] += (fwdKgs - f.fwd[seg]) * kF;
        f.back[seg] += (backKgs - f.back[seg]) * kF;
    };
    const double inv = 1.0 / std::max(1e-9, dt);
    double thrF, thrB;
    intake.throttleFlows(thrF, thrB);
    const bool boosted = isBoosted(p_), turbo = boosted && exhaust.hasTurbine();
    if (boosted) both(kFlowInlet, boost.compFlow); else split(kFlowInlet, thrF, thrB);
    split(kFlowBov, boosted ? boost.ventFlow : 0.0, 0.0);
    split(kFlowThrottle, thrF, thrB);
    split(kFlowIntakeValve, flInF_ * inv, flInB_ * inv);
    split(kFlowExhaustValve, flExF_ * inv, flExB_ * inv);
    both(kFlowTurbine, turbo ? boost.turbFlow : 0.0);
    both(kFlowWastegate, turbo ? boost.wgFlow : 0.0);
    // the tailpipe carries the net flow (the pulses are smoothed out by the pipes and silencers on the way)
    flTailNet_ += ((turbo ? boost.turbFlow + boost.wgFlow : (flExF_ - flExB_) * inv) - flTailNet_) * kF;
    f.fwd[kFlowTail] = std::max(0.0, flTailNet_); f.back[kFlowTail] = std::max(0.0, -flTailNet_);
    flInF_ = flInB_ = flExF_ = flExB_ = 0;

    auto ema = [kP](double& v, double x) { v += (x - v) * kP; };
    const double pa = kPatm * 1e-3;
    f.pAmb = pa; f.tAmb = ambT_;
    ema(f.pCharge, boosted ? intake.chargeNode().p() * 1e-3 : pa);
    ema(f.tCompOut, boosted ? boost.compOutT : ambT_);
    ema(f.tCharge, boosted ? boost.chargeT : ambT_);
    ema(f.pPlenum, intake.map() * 1e-3);
    ema(f.tPlenum, intake.plenumTemp());
    double pi = 0, ti = 0, pe = 0;
    for (int i = 0; i < nPorts_; ++i) { pi += intake.port(i).p(); ti += intake.port(i).T; pe += exhaust.port(i).p(); }
    const double np = std::max(1, nPorts_);
    ema(f.pPortIn, pi / np * 1e-3);
    ema(f.tPortIn, ti / np);
    ema(f.pPortEx, pe / np * 1e-3);
    ema(f.tCyl, egtCyl);
    ema(f.tCollector, egtAvg);
    ema(f.pTurbIn, turbo ? exhaust.turbineIn().p() * 1e-3 : pe / np * 1e-3);
    ema(f.pTurbOut, turbo ? exhaust.turbineOut().p() * 1e-3 : pa);
    ema(f.tPostTurb, std::max(ambT_, egtAvg - (turbo ? boost.turbineDropK : 0.0)));
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
    // (carbon builds up over minutes of rich running on a too-cold plug, not seconds)
    if (plugTempC < 450.0 && lambdaAvg < 0.95 && rpm > 300) fouling += dt * 0.0035 * (450.0 - plugTempC) / 50.0;
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

#include "sim/Intake.h"
#include <algorithm>

namespace es {

static double circ(double d_mm) { double d = d_mm * 1e-3; return kPi * d * d * 0.25; }

void IntakeSystem::build(const EngineParams& p, double fs, double displacement) {
    p_ = p;
    fs_ = fs;
    const int n = p.cylinders;
    vdCyl_ = displacement / n;
    revsPerEvent_ = revsPerFiring(p);
    ambT_ = 273.15 + p.ambientTemp_C;

    boosted_ = isBoosted(p);
    charge_ = GasNode();
    charge_.V = std::max(0.3, (double)p.chargeVolume_L) * 1e-3;
    charge_.T = ambT_;
    charge_.setPressure(kPatm);

    const int nIn = p.itb ? n : 1;
    inlets_.assign(nIn, GasNode());
    for (GasNode& g : inlets_) {
        g.V = p.itb ? 0.12e-3 : std::max(0.2, (double)p.plenum_L) * 1e-3;
        g.T = ambT_;
        plenumT0_ = g.T;
        g.setPressure(kPatm);
    }
    ports_.assign(n, GasNode());
    for (GasNode& g : ports_) {
        // (two-stroke: the reed block in front of the crankcase - about 15 % of the cylinder's displacement, chosen)
        g.V = isTwoStroke(p) ? std::max(10e-6, 0.15 * vdCyl_)
            : isRotary(p) ? 30e-6 * std::max(0.5, p.rIntakeArea_cm2 / 4.0)
                          : 45e-6 * std::max(0.5, p.intakeValves * p.intakeValveDia_mm * p.intakeValveDia_mm / (2.0 * 34.0 * 34.0));
        g.T = ambT_ + 15;
        g.setPressure(kPatm);
    }
    inletFilm_.assign(nIn, 0); portFilm_.assign(n, 0);
    backM_.assign(nIn, 0); backMf_.assign(nIn, 0); backMb_.assign(nIn, 0);
    inletFlow_.assign(nIn, 0); pending_.assign(n, 0);
    inletT_ = ambT_;
    injSpread_.resize(n);
    for (int i = 0; i < n; ++i) injSpread_[i] = cylSpread(i, n, 16);

    net = WaveNetwork();
    net.init(fs, std::sqrt(1.4 * kR * ambT_), kPatm / (kR * ambT_), kPatm);
    // two-stroke (T1): the tract from the reed block / inlet port out to the carburettor bell and the air box as 1D
    // unsteady gas dynamics - the crankcase draws on it with suction waves of a third of an atmosphere and more, and
    // the ramming it sends back fills the crankcase past what a quasi-steady (or small-signal) runner gives (Blair
    // models the intake ducting this way, sec. 5.2.3). Length / bore: the runner's (Intake tab); ~20 mm cells.
    ducts_.clear();
    if (isTwoStroke(p)) {
        const double len = std::max(0.06, p.runnerLen_mm * 1e-3), a = circ(p.runnerDia_mm);
        const int nc = std::max(4, (int)std::lround(len / 0.04));
        std::vector<double> xs(nc + 1), as(nc + 1, a);
        for (int k = 0; k <= nc; ++k) xs[k] = len * k / nc;
        ducts_.resize(n);
        for (Duct& d : ducts_) {
            d.build(xs, as, ambT_, kPatm, std::max(1, (int)std::lround(fs / 48000.0)));
            d.setFresh();
        }
    }
    for (int i = 0; i < n && ducts_.empty(); ++i) {
        int id = net.addPipe(circ(p.runnerDia_mm), p.runnerLen_mm * 1e-3, 10000, 0.04);
        GasNode& in = inlets_[p.itb ? i : 0];
        net.addNodeEnd(PipeEnd{id, 0}, &ports_[i], &in);
        net.addNodeEnd(PipeEnd{id, 1}, &in, &ports_[i]);
    }
    net.reset();

    throttlePos_ = prevThrottle_ = 0;
    iac_ = 0.35;
    pumpFuel_ = 0;
    filtA_ = onePoleCoef(6000, fs);
}

void IntakeSystem::applyGeometry() {
    boosted_ = isBoosted(p_);
    {
        double pr = charge_.p();
        charge_.V = std::max(0.3, (double)p_.chargeVolume_L) * 1e-3;
        charge_.setPressure(pr);
    }
    for (Pipe& pp : net.pipes) {
        pp.area = circ(p_.runnerDia_mm);
        pp.length = std::max(0.04, p_.runnerLen_mm * 1e-3);
    }
    if (!p_.itb && !inlets_.empty()) {
        // plenum volume change: keep pressure
        GasNode& g = inlets_[0];
        double pr = g.p(), ff = g.fuelFrac();
        g.V = std::max(0.2, (double)p_.plenum_L) * 1e-3;
        g.setPressure(pr); g.mf = g.m * ff;
    }
}

void IntakeSystem::addFuel(GasNode& n, double& film, double kg, double filmFrac) {
    film += kg * filmFrac;
    n.mf += kg * (1 - filmFrac);
    n.m += kg * (1 - filmFrac);
}

double IntakeSystem::map() const {
    double s = 0;
    for (const GasNode& g : inlets_) s += g.p();
    return s / inlets_.size();
}

void IntakeSystem::stepAir(double dt, double pedal, double rpm, bool fuelEnabled) {
    const int nIn = (int)inlets_.size();
    const int n = (int)ports_.size();
    // throttle actuator / cable lag
    // cable: the butterfly follows the pedal almost at once; drive-by-wire: motor time constant
    const double thrTau = p_.comp.throttleType == ThrottleType::DriveByWire ? std::max(5.0, (double)p_.comp.dbwResponse_ms) * 1e-3 : 0.025;
    throttlePos_ += (clampv(pedal, 0.0, 1.0) - throttlePos_) * (1.0 - std::exp(-dt / thrTau));
    double alpha = 0.5 * kPi * std::pow(throttlePos_, 0.85);
    double plateFrac = 1.0 - std::cos(alpha);

    const bool carb = p_.fuelSystem == FuelSystem::Carburetor;
    int barrels = carb ? std::max(1, (int)p_.carbBarrels) : 1;
    double aBore = circ(p_.throttleBore_mm);
    double aTotal = (p_.itb ? n : barrels) * aBore;
    const double dispL = vdCyl_ * n * 1000.0 * (2.0 / revsPerEvent_);  // 4-stroke equivalent
    // idle air: carb = fixed idle speed screw, EFI = idle air control valve (scaled to engine size)
    // carb: idle speed screw, opened further by the fast-idle cam while the automatic choke is on
    double aIdle = carb ? p_.idleScrew * 2.6e-6 * dispL * (1.0 + 1.2 * cold_) : iac_ * 2.0e-5 * dispL + 0.0025 * aTotal;
    double aThr = 0.82 * (plateFrac * aTotal + aIdle + 1e-7);
    double aFilter = 0.014 * (1.08 - clampv((double)p_.airFilter, 0.0, 1.0)) * std::max(1.0, aTotal / 3e-3);
    // with boost the air filter sits in front of the compressor, not the throttle
    double cda = boosted_ ? aThr : 1.0 / std::sqrt(1.0 / (aThr * aThr) + 1.0 / (aFilter * aFilter));
    cda /= nIn;

    // accelerator pump (carb)
    if (carb) {
        double dthr = throttlePos_ - prevThrottle_;
        if (dthr > 0) pumpFuel_ += p_.accelPump * dthr * 0.15e-3 * (vdCyl_ * n * 1000.0);
    }
    prevThrottle_ = throttlePos_;

    double totalAir = 0, totalFuel = 0;
    for (int k = 0; k < nIn; ++k) {
        GasNode& g = inlets_[k];
        double flow;
        double fresh = 0;   // kg/s of fresh air coming in (not the standoff puff drawn back) - what a carb meters
        // Gas the intake pushes back out through the throttle (reversion at low rpm and full throttle carries the
        // fuel just injected) is not lost: with boost it goes back into the charge pipe, without it stays as a
        // fuel-laden puff in front of the throttle (air box / velocity stack "standoff") that the next intake
        // pulse draws in first. Dropping it lost most of the fuel at a tip-in from idle on big-overlap engines -
        // the charges went far too lean and the engine bogged down and stalled.
        if (boosted_) {
            // throttle fed by the pressurised charge pipe; the plenum takes its temperature
            flow = limitedFlow(cda, charge_.p(), charge_.T, charge_.V, g.p(), g.T, g.V, dt, 0.4);
            if (flow >= 0) {
                const double q = std::min(flow * dt, charge_.m * 0.3);
                const double ff = charge_.fuelFrac(), bf = charge_.burnedFrac();
                charge_.m -= q; charge_.mf -= q * ff; charge_.mb -= q * bf;
                g.m += q; g.mf += q * ff; g.mb += q * bf;
                flow = q / dt;
                fresh = flow;
            } else {
                const double dm = std::min(-flow * dt, g.m * 0.5);
                const double ff = g.fuelFrac(), bf = g.burnedFrac();
                g.m -= dm; g.mf -= dm * ff; g.mb -= dm * bf;
                charge_.m += dm; charge_.mf += dm * ff; charge_.mb += dm * bf;
            }
            // the plenum takes the charge temperature at constant pressure (its composition stays the same) - with
            // EGR the flow-weighted mix of the charge and the recirculated gas
            const double pNow = g.p(), m0 = g.m;
            const double tIn = (1.0 - egrShare_) * charge_.T + egrShare_ * egrT_;
            g.T += (tIn - g.T) * std::min(1.0, dt / 0.3);
            g.setPressure(pNow);
            if (m0 > 1e-12) { const double s = g.m / m0; g.mf *= s; g.mb *= s; }
        } else {
            if (egrShare_ > 1e-4 || std::fabs(g.T - plenumT0_) > 0.5) {
                // EGR into a naturally aspirated plenum: the flow-weighted mix of the fresh air and the gas
                const double pNow = g.p(), m0 = g.m;
                const double tIn = (1.0 - egrShare_) * plenumT0_ + egrShare_ * egrT_;
                g.T += (tIn - g.T) * std::min(1.0, dt / 0.3);
                g.setPressure(pNow);
                if (m0 > 1e-12) { const double s = g.m / m0; g.mf *= s; g.mb *= s; }
            }
            flow = limitedFlow(cda, kPatm, inletT_, 1e3, g.p(), g.T, g.V, dt, 0.4);
            if (flow >= 0) {
                const double q = flow * dt;
                // the standoff puff comes back in first
                const double t = std::min(q, backM_[k]);
                if (t > 0) {
                    const double ff = backMf_[k] / backM_[k], bf = backMb_[k] / backM_[k];
                    g.mf += t * ff; g.mb += t * bf;
                    backM_[k] -= t; backMf_[k] -= t * ff; backMb_[k] -= t * bf;
                }
                g.m += q;
                fresh = (q - std::max(0.0, t)) / dt;
            } else {
                const double dm = std::min(-flow * dt, g.m * 0.5);
                const double ff = g.fuelFrac(), bf = g.burnedFrac();
                g.m -= dm; g.mf -= dm * ff; g.mb -= dm * bf;
                // an air box keeps the puff; open velocity stacks lose much of it to the surroundings
                const double keep = clampv(0.4 + 0.6 * (double)p_.airFilter, 0.4, 1.0);
                backM_[k] += dm * keep; backMf_[k] += dm * ff * keep; backMb_[k] += dm * bf * keep;
                // what does not fit in front of the throttle (about the inlet's own volume of air) drifts away
                const double cap = 1.2 * std::max(0.3e-3, g.V);
                if (backM_[k] > cap) { const double s = cap / backM_[k]; backM_[k] = cap; backMf_[k] *= s; backMb_[k] *= s; }
            }
        }
        inletFlow_[k] = flow;
        totalAir += flow;

        if (carb) {
            double ma = std::max(flow, 0.0);   // the whole flow through the venturi makes the signal ...
            int bpi = p_.itb ? 1 : barrels;
            double av = bpi * circ(p_.venturi_mm) * 0.9;
            double rho = kPatm / (kR * inletT_);
            double dpv = (ma / av) * (ma / av) / (2 * rho);
            double s = smoothstep(60, 500, dpv);                    // main circuit signal
            double idleF = 1.0 - smoothstep(0.02, 0.14, throttlePos_); // idle / off-idle circuit
            double afrMain = 12.8 / std::max(0.3, (double)p_.mainJet);
            double afrIdle = 13.0 / std::max(0.3, (double)p_.idleMixture);
            double fr = s / afrMain + (1 - s) * (idleF / afrIdle + (1 - idleF) * 0.82 / afrMain);
            // the jets are sized for gasoline: they pass about the same volume of another fuel (mass ~ sqrt of the
            // density through an orifice) - on E85 or methanol the same jets run very lean
            fr *= std::sqrt(fuel_.density / 0.745);
            fr *= 1.0 + 0.9 * std::max((double)p_.choke, 0.4 * cold_);   // manual choke or the automatic (bimetal) choke
            fr *= 1.0 + 0.5 * (1.0 - smoothstep(250, 600, rpm)); // cranking / just-started richness
            double mf = std::max(0.0, fresh) * fr;   // ... but the standoff puff already carries its fuel
            if (pumpFuel_ > 0) {
                double shot = std::min(pumpFuel_, pumpFuel_ * dt / 0.15 + 1e-9) / nIn;
                pumpFuel_ -= shot * nIn;
                mf += shot / dt;
            }
            mf *= carbSupply_;
            addFuel(g, inletFilm_[k], mf * dt, 0.22 + 0.12 * cold_);
            totalFuel += mf * dt;
        }
        // film evaporation
        double ev = inletFilm_[k] * dt / ((carb ? 0.22 : 0.12) * filmSlow());
        inletFilm_[k] -= ev; g.mf += ev; g.m += ev;
    }

    // EFI spray
    if (!carb) {
        const double rate = sprayRate_;   // actual injector flow (rail pressure)
        for (int i = 0; i < n; ++i) {
            if (pending_[i] <= 0) continue;
            double q = std::min(pending_[i], rate * dt);
            pending_[i] -= q;
            if (!fuelEnabled) continue;
            if (p_.injection == Injection::Port)
                addFuel(ports_[i], portFilm_[i], q, 0.20 + 0.12 * cold_);
            else {
                int k = p_.itb ? i : 0;
                addFuel(inlets_[k], inletFilm_[k], q, 0.25 + 0.12 * cold_);
            }
            totalFuel += q;
        }
        for (int i = 0; i < n; ++i) {
            double ev = portFilm_[i] * dt / (0.07 * filmSlow());
            portFilm_[i] -= ev; ports_[i].mf += ev; ports_[i].m += ev;
        }
    }

    // diesel (D4): the intake ports take the plenum's temperature (+15 K from the warm head, as they start), so the charge
    // the cylinders draw follows the boost / intercooler / EGR mix. (The port nodes are isothermal and otherwise stay at
    // ambient + 15 K: the plenum's temperature never reached the cylinders - an EGR cooler, or hot EGR, changed nothing.
    // Petrol engines keep that behaviour for now: their base maps and calibrations were measured with it.)
    if (isDiesel(p_)) {
        for (int i = 0; i < n; ++i) {
            GasNode& g = ports_[i];
            const GasNode& src = inlets_[p_.itb ? i : 0];
            const double pNow = g.p(), m0 = g.m;
            g.T += (src.T + 15.0 - g.T) * std::min(1.0, dt / 0.05);
            g.setPressure(pNow);
            if (m0 > 1e-12) { const double sc = g.m / m0; g.mf *= sc; g.mb *= sc; }
        }
    }

    airFlow_ = totalAir;
    fuelFlow_ = totalFuel / dt;
    fuelFlowAvg_ += (fuelFlow_ - fuelFlowAvg_) * (1.0 - std::exp(-dt / 0.3));

    // induction noise: monopole from fluctuating inlet mass flow, filtered by the air box
    double d = (totalAir - prevFlow_) / dt;
    prevFlow_ = totalAir;
    double fc = 300 + 7000 * (1.0 - 0.85 * clampv((double)p_.airFilter, 0.0, 1.0));
    double a = onePoleCoef(fc, fs_);
    radLp_ += a * (d - radLp_);
    radiated_ = radLp_ / (4.0 * kPi) * (boosted_ ? 0.3 : 1.0);  // boosted: throttle is inside the pipework
}

// two-stroke intake ducts: the far end opens into the plenum / air box (its pressure: p = p0 + 2 in with no
// impedance), the mass that crosses moves between the duct and the plenum node; fuel an injector put into the port
// node goes into the duct's first cell (the reed end); the port node mirrors that cell (traces, sensors)
void IntakeSystem::stepDucts(double dt) {
    stepDuctsPre(dt);
    for (Duct& d : ducts_) d.step(dt, ambT_);
    stepDuctsPost();
}

// (pre: the outlets and the mass they move - the outflow is fixed by the outlet solve, before the duct steps; post:
//  the port nodes mirror the first cells. The steps between can run in parallel - Engine / DuctPool)
void IntakeSystem::stepDuctsPre(double dt) {
    for (size_t i = 0; i < ducts_.size(); ++i) {
        Duct& d = ducts_[i];
        GasNode& pl = inlets_[p_.itb ? i : 0];
        GasNode& g = ports_[i];
        if (g.mf > 1e-15) { d.addFuel(g.mf); g.mf = 0; }
        d.setOutlet(0.5 * (pl.p() - kPatm), 0.0, 1.0, pl.T, pl.fuelFrac(), pl.burnedFrac());
        d.solveOutlet(dt);
        const double q = d.outflow() * dt;   // + into the plenum
        if (q > 0) { pl.m += q; pl.mf += q * d.outFuelFrac(); pl.mb += q * d.outBurnedFrac(); }
        else if (q < 0) {
            const double r = std::min(-q, pl.m * 0.5), ff = pl.fuelFrac(), bf = pl.burnedFrac();
            pl.m -= r; pl.mf -= r * ff; pl.mb -= r * bf;
        }
    }
}

void IntakeSystem::stepDuctsPost() {
    for (size_t i = 0; i < ducts_.size(); ++i) {
        const Duct& d = ducts_[i];
        GasNode& g = ports_[i];
        g.setPressure(d.cellP(0)); g.mf = 0; g.mb = g.m * d.cellBurned(0);
    }
}

// physical injector: the needle opens after its dead time, then sprays at the flow the rail
// pressure across the nozzle allows; injector-to-injector manufacturing spread on top
void IntakeSystem::injectPulse(int cyl, double pw, double cycleTime, double flowKgs, double deadS) {
    if (p_.fuelSystem != FuelSystem::EFI || cyl < 0 || cyl >= (int)pending_.size()) return;
    double open = std::max(0.0, std::min(pw, cycleTime) - deadS);
    double fuel = flowKgs * open * (1.0 + 0.015 * effImperfection(p_) * injSpread_[cyl]);
    pending_[cyl] += fuel;
    // an injector cannot spray more than one cycle's worth: the excess never comes out
    double maxPerCycle = flowKgs * std::max(0.0, cycleTime - deadS);
    if (pending_[cyl] > maxPerCycle) pending_[cyl] = maxPerCycle;
}

} // namespace es

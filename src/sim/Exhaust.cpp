#include "sim/Exhaust.h"
#include "sim/Cylinder.h"
#include <cmath>
#include <algorithm>

namespace es {

static double circ(double d_mm) { double d = d_mm * 1e-3; return kPi * d * d * 0.25; }

double ExhaustSystem::areaFor(const Pipe& pp) const {
    if (tsPipe_ && pp.role == Primary) return kPi * 0.25 * tp_.dia[0] * tp_.dia[0];   // (two-stroke tuned pipe: L1, d1)
    switch ((Role)pp.role) {
    case Primary: return circ(p_.primaryDia_mm);
    case Secondary: return circ(p_.primaryDia_mm * 1.25);
    case Collector: case Log: return circ(p_.collectorDia_mm);
    default: return circ(p_.pipeDia_mm) * pp.areaMul;
    }
}

double ExhaustSystem::lengthFor(const Pipe& pp) const {
    if (tsPipe_ && pp.role == Primary) return std::max(0.02, tp_.len[0]);
    switch ((Role)pp.role) {
    case Primary: return std::max(0.05, p_.primaryLen_mm[pp.roleIdx] * 1e-3 *
                                  (1.0 + 0.02 * effImperfection(p_) * cylSpread(pp.roleIdx, (int)ports_.size(), 17)));
    case Secondary: return std::max(0.05, p_.secondaryLen_mm * 1e-3);
    case Collector: return std::max(0.05, p_.collectorLen_mm * 1e-3);
    case Mid: return std::max(0.05, p_.midpipeLen_mm * 1e-3 * pp.lenMul);
    case Tail: return std::max(0.05, p_.tailpipeLen_mm * 1e-3 * pp.lenMul);
    default: return pp.lenMul; // fixed length in metres
    }
}

int ExhaustSystem::newPipe(Role role, int idx, double areaMul, double lenParam, double fc, double loss) {
    int id = net.addPipe(1e-3, 0.5, fc, loss);
    Pipe& pp = net.pipes[id];
    pp.role = role; pp.roleIdx = idx; pp.areaMul = areaMul; pp.lenMul = lenParam;
    pp.area = areaFor(pp); pp.length = lengthFor(pp);
    // gas cools on its way out (fraction of the valve-exit temperature rise above ambient)
    switch (role) {
    case Primary: pp.zone = 1.0; break;
    case Secondary: case Log: pp.zone = 0.95; break;
    case Collector: pp.zone = 0.85; break;
    case Mid: case Balance: pp.zone = 0.7; break;
    case Tail: pp.zone = 0.5; break;
    default: pp.zone = fixedZone_; break;
    }
    return id;
}

PipeEnd ExhaustSystem::chain(PipeEnd from, Role role, int idx, double areaMul, double lenParam, double fc, double loss) {
    int id = newPipe(role, idx, areaMul, lenParam, fc, loss);
    net.connect({from, PipeEnd{id, 0}});
    return PipeEnd{id, 1};
}

// Expansion chamber. With extended tubes, the inlet pipe reaches L/2 into the chamber and the outlet starts L/4 before its
// end; the annular spaces around them are closed quarter-wave side branches that fill the chamber's transmission-loss
// troughs at kL = n pi (the classic reactive-muffler design: Munjal, Acoustics of Ducts and Mufflers).
PipeEnd ExhaustSystem::chamber(PipeEnd from, double ar, double L, double fc, double loss, bool ext) {
    if (!ext) return chain(from, Fixed, 0, ar, L, fc, loss);
    const double li = 0.5 * L, lo = 0.25 * L;
    const int tin = newPipe(Fixed, 0, 1.0, li, 9000, 0.03);          // inlet tube inside the chamber
    net.connect({from, PipeEnd{tin, 0}});
    const int annIn = newPipe(Fixed, 0, ar - 1.0, li, fc, loss);       // annulus around it, closed at the wall
    net.addClosedEnd(PipeEnd{annIn, 0});
    const int mid = newPipe(Fixed, 0, ar, L - li - lo, fc, loss);
    net.connect({PipeEnd{tin, 1}, PipeEnd{annIn, 1}, PipeEnd{mid, 0}});
    const int annOut = newPipe(Fixed, 0, ar - 1.0, lo, fc, loss);
    net.addClosedEnd(PipeEnd{annOut, 1});
    const int tout = newPipe(Fixed, 0, 1.0, lo, 9000, 0.03);         // outlet tube inside the chamber
    net.connect({PipeEnd{mid, 1}, PipeEnd{annOut, 0}, PipeEnd{tout, 0}});
    return PipeEnd{tout, 1};
}

double ExhaustSystem::helmholtzHz() const {
    if (p_.resonatorHz > 1.0f) return p_.resonatorHz;
    // auto: the firing frequency at 2000 rpm (cylinders / 2 per revolution; rotors and two-strokes fire once per turn)
    const double perRev = p_.cylinders / revsPerFiring(p_);
    return std::max(20.0, perRev * 2000.0 / 60.0);
}

// Two-stroke expansion chamber (tuned pipe), Blair's empirical design (Design and Simulation of Two-Stroke Engines,
// SAE 1996, sec. 6.2.5): the blowdown pulse leaves the port when it opens; the diffuser sends back a suction wave that
// helps the scavenging, the rear cone a pressure wave that pushes the fresh charge that has run into the pipe back
// into the cylinder before the port closes ("plugging").
//  - tuned length, port to tail-pipe entry, from the reflection period = the exhaust period (eq. 6.2.4):
//    LT = 83.3 a0 theta_ep / rpm (mm), a0 the sound speed at the pipe's mid-section temperature - 600 C for a Grand
//    Prix engine, 500 C for an enduro engine (in between: interpolated, chosen)
//  - lengths (eq. 6.2.5 / 6.2.6): L1 0.1, L2 0.275, L3 0.183, L4 0.092, L5 0.11, L6 0.24 LT, L7 = L6
//  - diameters: d0 the exhaust port's full area; d1 = k1 d0, k1 1.125 (enduro) .. 1.05 (road racing) (6.2.7);
//    d4 = d5 = k2 d0, k2 2.125 .. 3.25 (6.2.8); tail pipe d7 = 0.7 / 0.65 / 0.6 d0 for enduro / motocross / road
//    racing (p. 441); d2, d3 on a horn from d1 to d4 over L2..L4 with the horn coefficient kh 1.5 (Blair's program
//    default; range 1.25-2): d = d1 exp((x / (L2 + L3 + L4))^kh ln(d4 / d1)) - read from the scanned text and
//    checked against the book's worked example (125 cc, 11,750 rpm: d2 56, d3 89 mm printed; this gives 56.4 / 88.7).
//    Between enduro (class 0) and road racing (1) the constants are interpolated (chosen); motocross is 0.5.
TunedPipeGeom tunedPipeGeometry(const EngineParams& p) {
    TunedPipeGeom g;
    CylGeom cg; cg.set(p);
    const double cls = clampv((double)p.tsPipeClass, 0.0, 1.0);
    const double area0 = std::max(1e-5, cg.wEx * (cg.stroke - cg.sEx));   // the exhaust port fully open (rectangle)
    g.d0 = std::sqrt(4.0 * area0 / kPi);
    const double thetaEp = 360.0 - 2.0 * clampv((double)p.tsExhaustOpen, 45.0, 150.0);
    const double rpm = clampv((double)p.tsPipeRpm, 2000.0, 20000.0);
    g.texcC = 500.0 + 100.0 * cls;
    const double a0 = std::sqrt(kGamma * kR * (g.texcC + 273.15));
    g.lt = a0 * thetaEp / (12.0 * rpm);
    const double frac[7] = {0.1, 0.275, 0.183, 0.092, 0.11, 0.24, 0.24};
    for (int k = 0; k < 7; ++k) g.len[k] = frac[k] * g.lt;
    const double k1 = 1.125 - 0.075 * cls, k2 = 2.125 + 1.125 * cls, kt = 0.7 - 0.1 * cls, kh = 1.5;
    const double d1 = k1 * g.d0, d4 = k2 * g.d0;
    const double lDiff = g.len[1] + g.len[2] + g.len[3];
    auto horn = [&](double x) { return d1 * std::exp(std::pow(x / lDiff, kh) * std::log(d4 / d1)); };
    g.dia[0] = d1;
    g.dia[1] = horn(g.len[1]);
    g.dia[2] = horn(g.len[1] + g.len[2]);
    g.dia[3] = d4;
    g.dia[4] = d4;
    g.dia[5] = kt * g.d0;
    g.dia[6] = kt * g.d0;
    return g;
}

// the pipe after its first section (the primary): diffuser cones, belly, rear cone, tail pipe - each cone as short
// parallel steps
PipeEnd ExhaustSystem::expansionChamber(PipeEnd from) {
    const TunedPipeGeom& g = tp_;
    const double S = circ(p_.pipeDia_mm);
    int steps[7], total = 0;
    for (int k = 1; k < 7; ++k) { steps[k] = (k == 4 || k == 6) ? 1 : std::max(2, (int)std::lround(g.len[k] / 0.04)); total += steps[k]; }
    // (each pipe low-passes a wave once per traversal: many short sections at the usual 9 kHz filtered the pulses
    //  the pipe works with many times over - the sections get a cutoff that adds up to about one pipe's)
    const double fc = 9000.0 * std::sqrt((double)total);
    double x = g.len[0];                       // distance from the port, for the gas temperature zone
    PipeEnd e = from;
    double dPrev = g.dia[0];
    for (int k = 1; k < 7; ++k) {
        const int n = steps[k];
        for (int j = 0; j < n; ++j) {
            const double f = (j + 0.5) / n;
            const double d = dPrev + (g.dia[k] - dPrev) * f;   // cone: diameter linear along it
            const double l = g.len[k] / n;
            fixedZone_ = 0.98 - 0.2 * clampv((x + 0.5 * l) / std::max(0.1, g.lt * 1.24), 0.0, 1.0);
            e = chain(e, Fixed, 0, circ(d * 1e3) / S, l, fc, 0.03);
            x += l;
        }
        dPrev = g.dia[k];
    }
    return e;
}

void ExhaustSystem::finishPath(PipeEnd from, float pan, double midFrac) {
    PipeEnd e = from;
    if (midFrac > 0) e = chain(e, Mid, 0, 1.0, midFrac, 9000, 0.03);
    fixedZone_ = 0.75;
    if (p_.cat) {
        e = chain(e, Fixed, 0, 3.2, 0.14, 2600, 0.9);     // catalyst brick: wide, lossy
        e = chain(e, Fixed, 0, 1.0, 0.30, 9000, 0.03);
    }
    fixedZone_ = 0.62;
    if (p_.resonator == 1) {
        e = chain(e, Fixed, 0, 3.5, 0.35, 3500, 0.35);
        e = chain(e, Fixed, 0, 1.0, 0.35, 9000, 0.03);
    } else if (p_.resonator == 2) {
        // Helmholtz side branch on the mid pipe: a neck (half the pipe area, 0.25 m) into a closed volume, sized for
        // the tuning frequency - f = 1 / 2pi sqrt(A K / (V l)), K the volume's stiffness. (The waveguide neck carries
        // the inertia itself; no end corrections are added, so the volume follows from the neck as built. The gas
        // volumes of the model are isothermal - p = m R T / V at a set T - so K = R T rather than the adiabatic
        // gamma R T: the volume is sized for the stiffness it really has, and the resonance lands on the tuning.)
        // It cancels the pipe's wave around that frequency: the cruise "drone".
        const double S = circ(p_.pipeDia_mm), A = 0.5 * S, l = 0.25;
        const double T = 300.0 + (gasT_ > 0 ? gasT_ : kGasT) * 0.62 - 300.0 * 0.62;   // (the zone of this section)
        const double w = 2.0 * kPi * helmholtzHz();
        GasNode v; v.V = std::max(2e-4, kR * T * A / (w * w * l)); v.T = T; v.setPressure(kPatm); v.mb = v.m;
        helm_.push_back(v);
        const int seg = newPipe(Fixed, 0, 1.0, 0.10, 9000, 0.03);
        const int neck = newPipe(Fixed, 0, 0.5, l, 9000, 0.03);
        net.connect({e, PipeEnd{seg, 0}, PipeEnd{neck, 0}});
        net.addNodeEnd(PipeEnd{neck, 1}, &helm_.back());
        e = PipeEnd{seg, 1};
    }
    fixedZone_ = 0.55;
    switch (p_.muffler) {
    case Muffler::StraightThrough:  // perforated core in packing: absorbs highs
        e = chain(e, Fixed, 0, 1.0, 0.45, 1700, 0.45);
        break;
    case Muffler::Chambered:        // reactive chambers (plane-wave expansion chambers, area ratio 6.5): resonant tone
        e = chamber(e, 6.5, 0.12, 6000, 0.12, p_.mufflerExtTubes != 0);
        e = chain(e, Fixed, 0, 0.8, 0.10, 9000, 0.05);
        e = chamber(e, 6.5, 0.18, 6000, 0.12, p_.mufflerExtTubes != 0);
        e = chain(e, Fixed, 0, 0.8, 0.12, 9000, 0.05);
        e = chamber(e, 6.5, 0.10, 6000, 0.12, p_.mufflerExtTubes != 0);
        break;
    case Muffler::Turbo:            // chambers + absorptive tubes: quiet, mellow
        e = chain(e, Fixed, 0, 5.0, 0.16, 1800, 0.6);
        e = chain(e, Fixed, 0, 0.6, 0.30, 1400, 0.7);
        e = chain(e, Fixed, 0, 5.0, 0.20, 1800, 0.6);
        break;
    case Muffler::None: default: break;
    }
    e = chain(e, Tail, 0, 1.0, 1.0, 10000, 0.03);
    net.addOpenEnd(e, pan);
}

void ExhaustSystem::build(const EngineParams& p, const float phase[kMaxCyl], double fs) {
    p_ = p;
    const int n = p.cylinders;
    tsPipe_ = isTwoStroke(p) && p.tsPipe;
    if (tsPipe_) tp_ = tunedPipeGeometry(p);
    net = WaveNetwork();
    const double c = std::sqrt(kGamma * kR * kGasT);
    net.init(fs, c, kPatm / (kR * kGasT), kPatm);

    burned_.V = 1; burned_.T = kGasT; burned_.m = 1; burned_.mb = 1; burned_.mf = 0;
    helm_.clear(); helm_.reserve(4);
    ports_.assign(n, GasNode());
    for (int i = 0; i < n; ++i) {
        GasNode& g = ports_[i];
        // (two-stroke: only the small volume right behind the port window - the duct through the cylinder is part of
        //  the wave path, counted in the header length: a lumped duct volume soaked up the blowdown pulse the tuned
        //  pipe works with. 15 % of the displacement, chosen: smaller starves the pipe - the waveguide draws at most 30 % of a node per step)
        g.V = isTwoStroke(p) ? std::max(4e-6, 0.15 * 3.14159265 * 0.25 * p.bore_mm * p.bore_mm * p.stroke_mm * 1e-9)
            : isRotary(p) ? 40e-6 * std::max(0.5, p.rExhaustArea_cm2 / 4.0)
                          : 70e-6 * std::max(0.5, p.exhaustValves * p.exhaustValveDia_mm * p.exhaustValveDia_mm / (2.0 * 30.0 * 30.0));
        g.T = kGasT;
        g.setPressure(kPatm);
        g.mb = g.m; // full of exhaust
    }

    // two-stroke with expansion chambers: every cylinder its own tuned pipe - 1D gas dynamics from the port to the end
    // of the tail pipe (Duct) - then its silencer (acoustic, fed by the duct's outflow)
    ducts_.clear();
    if (tsPipe_) {
        direct_ = true;
        outlets = n; groups = n;
        ducts_.resize(n);
        portPipe_.assign(n, 0);
        const TunedPipeGeom& g = tp_;
        // faces every ~40 mm along the profile (MC-limited slopes: the Blair 125 check gives charging efficiency 0.65
        // against 0.67 at 20-30 mm - 3 % for about a third less work, so a twin stays inside real time): L1 parallel, the three diffuser cones, the belly, the rear cone, the tail
        // pipe (diameters linear along each cone)
        double total = 0; for (double l : g.len) total += l;
        const int nc = std::max(12, (int)std::lround(total / 0.04));
        const int stepsPer = std::max(1, (int)std::lround(fs / 48000.0));   // (the duct at ~48 kHz)
        std::vector<double> xs(nc + 1), as(nc + 1);
        auto diaAt = [&](double x) {
            double x0 = 0, d0 = g.dia[0];
            for (int k = 0; k < 7; ++k) {
                const double x1 = x0 + g.len[k], dEnd = g.dia[k];
                if (x <= x1 || k == 6) return k == 0 ? g.dia[0] : d0 + (dEnd - d0) * clampv((x - x0) / std::max(1e-9, g.len[k]), 0.0, 1.0);
                x0 = x1; d0 = dEnd;
            }
            return g.dia[6];
        };
        for (int k = 0; k <= nc; ++k) { xs[k] = total * k / nc; const double d = diaAt(xs[k]); as[k] = kPi * 0.25 * d * d; }
        for (int i = 0; i < n; ++i) {
            ducts_[i].build(xs, as, kGasT, kPatm, stepsPer);
            // a short acoustic stub of the tail pipe's size: its first end takes the duct's outflow
            const double S = circ(p_.pipeDia_mm);
            fixedZone_ = 0.75;
            const int stub = newPipe(Fixed, 0, as[nc] / S, 0.03, 10000, 0.03);
            portPipe_[i] = stub;
            net.addFlowEnd(PipeEnd{stub, 0});   // (flow end i = cylinder i's duct)
            finishPath(PipeEnd{stub, 1}, n > 1 ? (float)(-0.7 + 1.4 * i / (n - 1)) : 0.0f, 0);
            GasNode& gp = ports_[i];
            gp.V = std::max(4e-6, as[0] * 0.03);   // (nominal: scales the afterfire pops)
        }
        hasTurbine_ = false;
        if (gasT_ <= 0) gasT_ = kGasT;
        net.setGasTemp(gasT_, 300.0);
        net.reset(1.0);
        net.setTone(p);
        net.setNonlinear(p.exhaustSteep);
        return;
    }

    // primaries
    std::vector<int> prim(n);
    direct_ = isTwoStroke(p);
    portPipe_.assign(n, 0);
    for (int i = 0; i < n; ++i) {
        prim[i] = newPipe(Primary, i, 1, 0, 10000, 0.03);
        portPipe_[i] = prim[i];
        if (direct_) net.addFlowEnd(PipeEnd{prim[i], 0});   // (flow end i = port i)
        else net.addNodeEnd(PipeEnd{prim[i], 0}, &ports_[i], &burned_);
    }

    // group cylinders by bank
    std::vector<std::vector<int>> grp(2);
    for (int i = 0; i < n; ++i) grp[(p.layout == Layout::Inline) ? 0 : clampv(p.bank[i], 0, 1)].push_back(i);
    if (grp[1].empty()) grp.pop_back();
    else if (grp[0].empty()) { grp[0] = grp[1]; grp.pop_back(); }
    groups = (int)grp.size();

    std::vector<PipeEnd> groupOut;
    for (auto& g : grp) {
        std::vector<PipeEnd> feeds;
        if (p.header == HeaderType::Log && g.size() > 1) {
            // log manifold: closed stub, then segments; each primary tees in
            double spacing = std::max(0.06, p.bore_mm * 1.2e-3);
            int stub = newPipe(Log, 0, 1, 0.04, 8000, 0.05);
            net.addClosedEnd(PipeEnd{stub, 0});
            PipeEnd prev{stub, 1};
            for (size_t k = 0; k < g.size(); ++k) {
                int seg = newPipe(Log, 0, 1, k + 1 < g.size() ? spacing : 0.05, 8000, 0.05);
                net.connect({prev, PipeEnd{prim[g[k]], 1}, PipeEnd{seg, 0}});
                prev = PipeEnd{seg, 1};
            }
            feeds.push_back(prev);
        } else if (p.header == HeaderType::TriY && g.size() >= 3) {
            // pair cylinders ~360 deg apart in firing (4-2-1 / tri-Y)
            std::vector<int> s = g;
            std::sort(s.begin(), s.end(), [&](int a, int b) { return phase[a] < phase[b]; });
            size_t half = s.size() / 2;
            for (size_t k = 0; k < half; ++k) {
                int sec = newPipe(Secondary, 0, 1, 0, 9000, 0.03);
                net.connect({PipeEnd{prim[s[k]], 1}, PipeEnd{prim[s[k + half]], 1}, PipeEnd{sec, 0}});
                feeds.push_back(PipeEnd{sec, 1});
            }
            if (s.size() % 2) feeds.push_back(PipeEnd{prim[s.back()], 1});
        } else {
            for (int c2 : g) feeds.push_back(PipeEnd{prim[c2], 1});
        }
        int col = newPipe(Collector, 0, 1, 0, 9000, 0.03);
        std::vector<PipeEnd> j = feeds;
        j.push_back(PipeEnd{col, 0});
        net.connect(j);
        groupOut.push_back(PipeEnd{col, 1});
    }

    hasTurbine_ = p.induction == Induction::Turbo;
    if (hasTurbine_) {
        // all collectors feed one turbine; after it a single downpipe -> cat / muffler / tail
        double disp = std::max(0.3, equivalentDisplacementL(p));
        tin_ = GasNode(); tin_.V = 0.35e-3 * disp; tin_.T = kGasT; tin_.setPressure(kPatm); tin_.mb = tin_.m;
        tout_ = GasNode(); tout_.V = 0.25e-3 * disp; tout_.T = kGasT; tout_.setPressure(kPatm); tout_.mb = tout_.m;
        for (const PipeEnd& g : groupOut) net.addNodeEnd(g, &tin_);
        fixedZone_ = 0.7;    // expansion through the turbine cools the gas
        int dp = newPipe(Fixed, 0, 1.15, 0.55, 9000, 0.03);
        net.addNodeEnd(PipeEnd{dp, 0}, &tout_);
        outlets = 1;
        finishPath(PipeEnd{dp, 1}, 0.f, 1.0);
    } else if (groupOut.size() == 1) {
        outlets = 1;
        finishPath(groupOut[0], 0.f, 1.0);
    } else {
        switch (p.crossover) {
        case Crossover::TrueDual:
            outlets = 2;
            finishPath(groupOut[0], -0.7f, 1.0);
            finishPath(groupOut[1], 0.7f, 1.0);
            break;
        case Crossover::HPipe: {
            outlets = 2;
            PipeEnd a = chain(groupOut[0], Mid, 0, 1, 0.35);
            PipeEnd b = chain(groupOut[1], Mid, 0, 1, 0.35);
            int bal = newPipe(Balance, 0, 1.0, 0.30, 9000, 0.03);
            int a2 = newPipe(Mid, 0, 1, 0.65, 9000, 0.03);
            int b2 = newPipe(Mid, 0, 1, 0.65, 9000, 0.03);
            net.connect({a, PipeEnd{bal, 0}, PipeEnd{a2, 0}});
            net.connect({b, PipeEnd{bal, 1}, PipeEnd{b2, 0}});
            finishPath(PipeEnd{a2, 1}, -0.7f, 0);
            finishPath(PipeEnd{b2, 1}, 0.7f, 0);
            break;
        }
        case Crossover::XPipe: {
            outlets = 2;
            PipeEnd a = chain(groupOut[0], Mid, 0, 1, 0.35);
            PipeEnd b = chain(groupOut[1], Mid, 0, 1, 0.35);
            int a2 = newPipe(Mid, 0, 1, 0.65, 9000, 0.03);
            int b2 = newPipe(Mid, 0, 1, 0.65, 9000, 0.03);
            net.connect({a, b, PipeEnd{a2, 0}, PipeEnd{b2, 0}});
            finishPath(PipeEnd{a2, 1}, -0.7f, 0);
            finishPath(PipeEnd{b2, 1}, 0.7f, 0);
            break;
        }
        case Crossover::MergeY: default: {
            outlets = 1;
            int m = newPipe(Mid, 0, 1, 1.0, 9000, 0.03);
            net.connect({groupOut[0], groupOut[1], PipeEnd{m, 0}});
            finishPath(PipeEnd{m, 1}, 0.f, 0);
            break;
        }
        }
    }
    if (gasT_ <= 0) gasT_ = kGasT;
    net.setGasTemp(gasT_, 300.0);
    net.reset(1.0);
    net.setTone(p);
    net.setNonlinear(p.exhaustSteep);
}

void ExhaustSystem::setGasTemp(double egtK) {
    double t = clampv(egtK, 450.0, 1400.0);
    if (std::fabs(t - gasT_) < 2.0) return;   // pipes only re-derive their gas state on real changes
    gasT_ = t;
    net.setGasTemp(gasT_, 300.0);
}

ExhaustSystem::PortWave ExhaustSystem::portWave(int i) const {
    const Pipe& pp = net.pipes[portPipe_[i]];
    PortWave w;
    w.in = pp.in0; w.Z = pp.Z; w.rho = pp.rho; w.T = pp.T;
    w.ff = pp.cm > 1e-12 ? pp.cmf / pp.cm : 0.0; w.bf = pp.cm > 1e-12 ? pp.cmb / pp.cm : 1.0;
    return w;
}

void ExhaustSystem::step(double dt) {
    // tuned pipes: gas dynamics up to the silencer, which gets their outflow as its source (the waveguide's waves
    // arriving there are already in: arrive() came first)
    for (size_t i = 0; i < ducts_.size(); ++i) {
        Duct& d = ducts_[i];
        const Pipe& sp = net.pipes[portPipe_[i]];
        d.setOutlet(sp.in0, sp.Z, sp.rho, sp.T, sp.cm > 1e-12 ? sp.cmf / sp.cm : 0.0, sp.cm > 1e-12 ? sp.cmb / sp.cm : 1.0);
        d.solveOutlet(dt);
        d.step(dt, ambK_);
        setPortFlow((int)i, d.outflow(), d.outFuelFrac(), d.outBurnedFrac());
    }
    net.step(dt);
    if (!ducts_.empty())   // the port "volume" shows the gas in the duct's first cell
        for (size_t i = 0; i < ports_.size(); ++i) {
            GasNode& g = ports_[i];
            const Duct& d = ducts_[i];
            g.setPressure(d.cellP(0));
            g.mf = g.m * d.cellFuel(0); g.mb = g.m * d.cellBurned(0);
        }
    else if (direct_)   // the port "volume" shows the pressure and gas at the pipe's end (sensors, traces, pops' scale)
        for (size_t i = 0; i < ports_.size(); ++i) {
            const Pipe& pp = net.pipes[portPipe_[i]];
            GasNode& g = ports_[i];
            g.setPressure(std::max(0.05 * kPatm, kPatm + pp.in0 + pp.out0));
            const double ff = pp.cm > 1e-12 ? pp.cmf / pp.cm : 0.0, bf = pp.cm > 1e-12 ? pp.cmb / pp.cm : 1.0;
            g.mf = g.m * ff; g.mb = g.m * bf;
        }
    // unburned HC in hot exhaust oxidises / is carried away: don't let it accumulate
    double k = 1.0 - std::exp(-dt / 0.04);
    for (Pipe& pp : net.pipes) { double d = pp.cmf * k; pp.cmf -= d; pp.cmb += d; }
    for (GasNode& g : ports_) { double d = g.mf * k; g.mf -= d; g.mb += d; }
    if (hasTurbine_) for (GasNode* g : {&tin_, &tout_}) { double d = g->mf * k; g->mf -= d; g->mb += d; }
}

void ExhaustSystem::apply(const EngineParams& p) {
    p_ = p;
    net.setTone(p);
    net.setNonlinear(p.exhaustSteep);
    for (Pipe& pp : net.pipes) {
        pp.area = areaFor(pp);
        pp.length = lengthFor(pp);
    }
}

void ExhaustSystem::radiated(double& left, double& right) const {
    left = right = 0;
    for (const OpenEnd& o : net.openEnds) {
        double l = 0.5 * (1.0 - o.pan), r = 0.5 * (1.0 + o.pan);
        left += o.sound * l * 2.0;
        right += o.sound * r * 2.0;
    }
}

} // namespace es

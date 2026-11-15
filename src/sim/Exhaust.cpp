#include "sim/Exhaust.h"
#include <algorithm>

namespace es {

static double circ(double d_mm) { double d = d_mm * 1e-3; return kPi * d * d * 0.25; }

double ExhaustSystem::areaFor(const Pipe& pp) const {
    switch ((Role)pp.role) {
    case Primary: return circ(p_.primaryDia_mm);
    case Secondary: return circ(p_.primaryDia_mm * 1.25);
    case Collector: case Log: return circ(p_.collectorDia_mm);
    default: return circ(p_.pipeDia_mm) * pp.areaMul;
    }
}

double ExhaustSystem::lengthFor(const Pipe& pp) const {
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

void ExhaustSystem::finishPath(PipeEnd from, float pan, double midFrac) {
    PipeEnd e = from;
    if (midFrac > 0) e = chain(e, Mid, 0, 1.0, midFrac, 9000, 0.03);
    fixedZone_ = 0.75;
    if (p_.cat) {
        e = chain(e, Fixed, 0, 3.2, 0.14, 2600, 0.9);     // catalyst brick: wide, lossy
        e = chain(e, Fixed, 0, 1.0, 0.30, 9000, 0.03);
    }
    fixedZone_ = 0.62;
    if (p_.resonator) {
        e = chain(e, Fixed, 0, 3.5, 0.35, 3500, 0.35);
        e = chain(e, Fixed, 0, 1.0, 0.35, 9000, 0.03);
    }
    fixedZone_ = 0.55;
    switch (p_.muffler) {
    case Muffler::StraightThrough:  // perforated core in packing: absorbs highs
        e = chain(e, Fixed, 0, 1.0, 0.45, 1700, 0.45);
        break;
    case Muffler::Chambered:        // reactive chambers: resonant, "honky" tone
        e = chain(e, Fixed, 0, 6.5, 0.12, 6000, 0.12);
        e = chain(e, Fixed, 0, 0.8, 0.10, 9000, 0.05);
        e = chain(e, Fixed, 0, 6.5, 0.18, 6000, 0.12);
        e = chain(e, Fixed, 0, 0.8, 0.12, 9000, 0.05);
        e = chain(e, Fixed, 0, 6.5, 0.10, 6000, 0.12);
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
    net = WaveNetwork();
    const double c = std::sqrt(kGamma * kR * kGasT);
    net.init(fs, c, kPatm / (kR * kGasT), kPatm);

    burned_.V = 1; burned_.T = kGasT; burned_.m = 1; burned_.mb = 1; burned_.mf = 0;
    ports_.assign(n, GasNode());
    for (int i = 0; i < n; ++i) {
        GasNode& g = ports_[i];
        g.V = isRotary(p) ? 40e-6 * std::max(0.5, p.rExhaustArea_cm2 / 4.0)
                          : 70e-6 * std::max(0.5, p.exhaustValves * p.exhaustValveDia_mm * p.exhaustValveDia_mm / (2.0 * 30.0 * 30.0));
        g.T = kGasT;
        g.setPressure(kPatm);
        g.mb = g.m; // full of exhaust
    }

    // primaries
    std::vector<int> prim(n);
    for (int i = 0; i < n; ++i) {
        prim[i] = newPipe(Primary, i, 1, 0, 10000, 0.03);
        net.addNodeEnd(PipeEnd{prim[i], 0}, &ports_[i], &burned_);
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

void ExhaustSystem::step(double dt) {
    net.step(dt);
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

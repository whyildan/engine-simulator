#include "sim/Waveguide.h"

namespace es {

static constexpr int kBufLen = 16384; // power of two

void WaveNetwork::init(double fs, double c, double rho0, double p0) {
    fs_ = fs; c_ = c; rho0_ = rho0; p0_ = p0;
    T0_ = p0 / (rho0 * kR);
    hotT_ = coldT_ = T0_;
    mask_ = kBufLen - 1;
    glide_ = 1.0 - std::exp(-1.0 / (0.08 * fs)); // ~80 ms glide on length changes
    dcA_ = onePoleCoef(4.0, fs);
    radA_ = onePoleCoef(9000.0, fs);
    uDcA_ = onePoleCoef(18.0, fs);     // separates the mean exhaust flow from the pulses
    bodyA_ = onePoleCoef(260.0, fs);   // "body" band of the gas pulses
    vA_ = onePoleCoef(150.0, fs);      // envelope of the exit velocity for the jet noise (breathes, no buzz)
    stA_ = onePoleCoef(1500.0, fs);    // steepening follows the smoothed wave, not every sample
}

int WaveNetwork::addPipe(double area, double length, double absorbFc, double lossPerM) {
    Pipe p;
    p.area = area; p.length = length; p.absorbFc = absorbFc; p.lossPerM = lossPerM;
    pipes.push_back(p);
    return (int)pipes.size() - 1;
}

void WaveNetwork::connect(std::initializer_list<PipeEnd> ends) {
    Junction j; j.ends.assign(ends.begin(), ends.end());
    junctions.push_back(j);
}
void WaveNetwork::connect(const std::vector<PipeEnd>& ends) {
    Junction j; j.ends = ends;
    junctions.push_back(j);
}

int WaveNetwork::addNodeEnd(PipeEnd e, GasNode* node, const GasNode* src) {
    NodeEnd n; n.e = e; n.node = node; n.inflowSource = src;
    nodeEnds.push_back(n);
    return (int)nodeEnds.size() - 1;
}

int WaveNetwork::addOpenEnd(PipeEnd e, float pan) {
    OpenEnd o; o.e = e; o.pan = pan;
    o.jetBp.bandpass(2600, 0.6, fs_);
    o.jetHi.lowpass(6000, 0.7, fs_);
    o.sm1.lowpass(smoothHz_, 0.5412, fs_); o.sm2.lowpass(smoothHz_, 1.3066, fs_);
    o.air1.lowpass(2000, 0.5412, fs_); o.air2.lowpass(2000, 1.3066, fs_);
    o.rng = Rng(1234 + openEnds.size() * 77);
    openEnds.push_back(o);
    return (int)openEnds.size() - 1;
}

void WaveNetwork::addClosedEnd(PipeEnd e) { closedEnds.push_back(e); }

void WaveNetwork::setTone(const EngineParams& p) {
    rasp_ = p.exhaustRasp; punch_ = p.exhaustPunch;
    style_ = p.soundStyle;
    pmix_ = clampv((double)p.pressureMix, 0.0, 1.0);
    double hz = clampv((double)p.smoothHz, 300.0, 0.45 * fs_);
    if (hz != smoothHz_ || openEnds.empty() || openEnds[0].sm1.b0 == 1.0) {
        smoothHz_ = hz;
        for (OpenEnd& o : openEnds) { o.sm1.lowpass(hz, 0.5412, fs_); o.sm2.lowpass(hz, 1.3066, fs_); }
    }
    setNonlinear(style_ == 1 ? 0.0 : (double)p.exhaustSteep);
}

void WaveNetwork::updateGas(Pipe& p) const {
    double T = p.zone < 0 ? T0_ : coldT_ + (hotT_ - coldT_) * p.zone;
    if (std::fabs(T - p.T) < 0.5 && p.T > 0) return;
    p.T = T;
    if (p.zone < 0) { p.c = c_; p.rho = rho0_; }
    else { p.c = std::sqrt(kGamma * kR * T); p.rho = p0_ / (kR * T); }
}

void WaveNetwork::setGasTemp(double hotK, double coldK) {
    hotT_ = hotK; coldT_ = coldK;
    for (Pipe& p : pipes) updateGas(p);
}

void WaveNetwork::reset(double burnedFrac) {
    for (Pipe& p : pipes) {
        p.T = 0;
        updateGas(p);
        p.cm = p.rho * p.area * p.length;
        p.cmb = p.cm * burnedFrac;
        p.cmf = 0;
        p.fwd.assign(kBufLen, 0.f);
        p.bwd.assign(kBufLen, 0.f);
        p.delay = clampv(p.length / p.c * fs_, 2.0, (double)kBufLen - 4);
        p.lpF = p.lpB = p.dcF = p.dcB = p.in0 = p.in1 = p.out0 = p.out1 = 0;
        p.stF = p.stB = p.offF = p.offB = 0;
    }
    w_ = 0;
}

static inline double readFrac(const std::vector<float>& b, int w, double d, int mask) {
    double pos = (double)w - d;
    double fl = std::floor(pos);
    int i0 = (int)fl;
    double fr = pos - fl;
    float a = b[i0 & mask], c = b[(i0 + 1) & mask];
    return a + (c - a) * fr;
}

void WaveNetwork::step(double dt) {
    const double maxDelay = (double)kBufLen - 4;
    // 1) arriving waves (with loss filters)
    for (Pipe& p : pipes) {
        double target = clampv(p.length / p.c * fs_, 2.0, maxDelay);
        p.delay += (target - p.delay) * glide_;
        p.Z = p.rho * p.c / p.area;
        p.Y = 1.0 / p.Z;
        if (p.cFc != p.absorbFc || p.cLen != p.length) {
            p.cFc = p.absorbFc; p.cLen = p.length;
            p.cA = onePoleCoef(p.absorbFc, fs_);
            p.cG = std::exp(-p.lossPerM * p.length);
        }
        const double a = p.cA, g = p.cG;
        double xF = readFrac(p.fwd, w_, p.delay, mask_);
        double xB = readFrac(p.bwd, w_, p.delay, mask_);
        if (nl_ > 0 && p.steep > 0) {
            // finite amplitude: a crest arrives early, a trough late (transit time ~ 1 - k p), so pulses
            // steepen. The shift follows the smoothed wave and may only change slowly, so the read
            // point always moves forward smoothly (no folding / sample skipping = no buzz).
            const double k = nl_ * p.steep;
            p.stF += stA_ * (xF - p.stF);
            p.stB += stA_ * (xB - p.stB);
            double tF = p.delay * (clampv(1.0 - k * p.stF, 0.75, 1.2) - 1.0);
            double tB = p.delay * (clampv(1.0 - k * p.stB, 0.75, 1.2) - 1.0);
            p.offF += clampv(tF - p.offF, -0.3, 0.3);
            p.offB += clampv(tB - p.offB, -0.3, 0.3);
            xF = readFrac(p.fwd, w_, std::max(2.0, p.delay + p.offF), mask_);
            xB = readFrac(p.bwd, w_, std::max(2.0, p.delay + p.offB), mask_);
        }
        p.lpF += a * (xF - p.lpF);
        p.lpB += a * (xB - p.lpB);
        p.dcF += dcA_ * (p.lpF - p.dcF);
        p.dcB += dcA_ * (p.lpB - p.dcB);
        p.in1 = p.dcF + (p.lpF - p.dcF) * g;
        p.in0 = p.dcB + (p.lpB - p.dcB) * g;
    }
    auto inOf = [&](const PipeEnd& e) -> double { const Pipe& p = pipes[e.pipe]; return e.end == 0 ? p.in0 : p.in1; };
    auto setOut = [&](const PipeEnd& e, double v) { Pipe& p = pipes[e.pipe]; if (e.end == 0) p.out0 = v; else p.out1 = v; };

    // 2) junction scattering (admittance A / (rho c): hot and cold pipes differ)
    for (const Junction& j : junctions) {
        double sy = 0, syp = 0;
        for (const PipeEnd& e : j.ends) { double y = pipes[e.pipe].Y; sy += y; syp += y * inOf(e); }
        double pj = 2.0 * syp / sy;
        for (const PipeEnd& e : j.ends) setOut(e, pj - inOf(e));
    }

    // 3) lumped nodes: pressure at pipe end equals node pressure
    for (NodeEnd& n : nodeEnds) {
        double in = inOf(n.e);
        Pipe& pp = pipes[n.e.pipe];
        const double Z = pp.Z, rho = pp.rho;
        // node pressure at the middle of this step's exchange with the pipe (trapezoidal: neither adds nor removes
        // energy). Using the pressure from before the exchange lags the node's compliance by a sample, which acts
        // as a negative resistance: with the engine at rest (no flow losses to hide it) the runner / port
        // resonance rang up by itself - a whine at 0 rpm
        const double k = kR * n.node->T / n.node->V * rho * dt / Z;
        double pn = (n.node->p() - p0_ + k * in) / (1.0 + 0.5 * k);
        double out = pn - in;
        double u = (out - in) / Z;              // volume velocity into pipe
        // never draw more than a fraction of the node's mass in one step; keep the
        // emitted wave consistent with the flow actually delivered (mass conservation)
        double maxU = 0.3 * n.node->m / (rho * dt);
        if (u > maxU) { u = maxU; out = in + Z * u; }
        setOut(n.e, out);
        double mdot = rho * u;
        n.massFlowIntoPipe = mdot;
        GasNode& g = *n.node;
        double dm = mdot * dt;
        if (dm > 0) {            // node -> pipe
            double ff = g.fuelFrac(), bf = g.burnedFrac();
            g.m -= dm; g.mf -= dm * ff; g.mb -= dm * bf;
            pp.cm += dm; pp.cmf += dm * ff; pp.cmb += dm * bf;
        } else if (dm < 0) {     // pipe -> node, carries the pipe's content
            double q = -dm;
            double ff = pp.cm > 1e-12 ? pp.cmf / pp.cm : 0, bf = pp.cm > 1e-12 ? pp.cmb / pp.cm : 0;
            g.m += q; g.mf += q * ff; g.mb += q * bf;
            pp.cm -= q; pp.cmf -= q * ff; pp.cmb -= q * bf;
        }
        if (g.mf < 0) g.mf = 0;
        if (g.mb < 0) g.mb = 0;
    }
    // keep pipe content at its nominal mass (net flow leaves through the far end / junction)
    for (Pipe& p : pipes) {
        double nom = p.rho * p.area * p.length;
        if (p.cm > 1e-12) { double s = nom / p.cm; p.cmf = std::max(0.0, p.cmf * s); p.cmb = std::max(0.0, p.cmb * s); }
        else { p.cmf = 0; }
        p.cm = nom;
    }

    // 4) open ends: frequency dependent negative reflection, radiation
    for (OpenEnd& o : openEnds) {
        double in = inOf(o.e);
        {   // radiation reflection (Waveguide.h): recomputed when the pipe radius or its gas sound speed moves
            const Pipe& pp = pipes[o.e.pipe];
            const double tau = std::sqrt(pp.area / kPi) / std::max(100.0, pp.c);
            if (std::fabs(tau - o.rTau) > 0.002 * tau) {
                o.rTau = tau;
                constexpr double n1 = 0.167, d1 = 1.393, d2 = 0.457;
                const double x = 2.0 * fs_ * tau;                       // bilinear: s = 2 fs (1 - z^-1) / (1 + z^-1)
                const double a0 = 1.0 + d1 * x + d2 * x * x;
                o.rb0 = -(1.0 + n1 * x) / a0; o.rb1 = -2.0 / a0; o.rb2 = -(1.0 - n1 * x) / a0;
                o.ra1 = (2.0 - 2.0 * d2 * x * x) / a0; o.ra2 = (1.0 - d1 * x + d2 * x * x) / a0;
            }
        }
        const double refl = o.rb0 * in + o.rb1 * o.rx1 + o.rb2 * o.rx2 - o.ra1 * o.ry1 - o.ra2 * o.ry2;
        o.rx2 = o.rx1; o.rx1 = in; o.ry2 = o.ry1; o.ry1 = refl;
        // (x 0.985: losses of the mean exhaust flow leaving the pipe - vortex shedding - that the no-flow theory leaves out)
        double out = 0.985 * refl;
        setOut(o.e, out);
        o.u = (in - out) / pipes[o.e.pipe].Z;
        // monopole far field at 1 m, radiating into ambient air
        double r = 1.2 / (4.0 * kPi) * (o.u - o.prevU) * fs_;
        o.radiated += radA_ * (r - o.radiated);   // tame the very top end
        o.prevU = o.u;

        // body: the pressure pulses themselves (low band), like a microphone close to the
        // tailpipe hears them - gives the thump / punch a pure far-field derivative lacks
        o.uDc += uDcA_ * (o.u - o.uDc);
        o.bodyLp += bodyA_ * ((o.u - o.uDc) - o.bodyLp);
        double body = punch_ * 1.2 / (4.0 * kPi) * (2.0 * kPi * 110.0) * o.bodyLp;

        // turbulent jet noise of the gas leaving the pipe: broadband, its strength follows the
        // exit velocity (~v^2), so it pulses with every exhaust pulse (rasp, crackle, "breath")
        double v = std::max(0.0, o.u) / pipes[o.e.pipe].area;
        o.vLp += vA_ * (v - o.vLp);
        double n = o.rng.bipolar();
        // jet noise peaks near Strouhal 0.3 (f = 0.3 v / D): low "breath" at idle, sharper at speed
        if ((++o.tick & 63) == 0) {
            double D = 2.0 * std::sqrt(pipes[o.e.pipe].area / kPi);
            double fc = clampv(0.3 * o.vLp / D, 250.0, 3800.0);
            o.jetBp.bandpass(fc, 0.55, fs_);
            o.jetHi.lowpass(std::min(7500.0, fc * 3.0), 0.7, fs_);
        }
        double jet = rasp_ * 2.4e-4 * o.vLp * o.vLp * o.jetHi.tick(o.jetBp.tick(n));
        if (style_ == 1) {                     // raw: the original pure monopole
            o.sound = o.radiated;
        } else {
            // smooth (engine-sim style): mostly the pulse pressure itself (flow pulse scaled so it equals
            // the radiated level at ~300 Hz), lightly modulated by low-passed air noise, everything
            // band-limited; the impulse response afterwards supplies the timbre
            double airN = o.air2.tick(o.air1.tick(o.rng.bipolar()));
            double ac = o.u - o.uDc;
            o.presHp += uDcA_ * (ac - o.presHp);           // second DC stage: no sub-audio drift / rumble at rest
            double pres = 1.2 / (4.0 * kPi) * (2.0 * kPi * 300.0) * (ac - o.presHp) * (1.0 + 0.5 * rasp_ * airN);
            double s = (1.0 - pmix_) * o.radiated + pmix_ * pres + body + jet;
            o.sound = o.sm2.tick(o.sm1.tick(s));
        }
    }
    for (const PipeEnd& e : closedEnds) setOut(e, inOf(e));

    // 5) write outgoing waves
    for (Pipe& p : pipes) {
        p.fwd[w_ & mask_] = (float)p.out0;
        p.bwd[w_ & mask_] = (float)p.out1;
    }
    ++w_;
}

} // namespace es

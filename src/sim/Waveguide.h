#pragma once
// 1D acoustic digital waveguide network: pipes (bidirectional fractional delay lines with
// frequency dependent losses), N-way scattering junctions, node (lumped volume) boundaries,
// open (radiating) ends and closed ends. Pressures are relative to the reference p0.
#include "sim/Gas.h"
#include "audio/Filters.h"
#include "sim/EngineParams.h"
#include <vector>

namespace es {

struct PipeEnd { int pipe = -1; int end = 0; };

struct Pipe {
    // configuration (may change live)
    double area = 1e-3;          // m^2
    double length = 0.5;         // m (target)
    double absorbFc = 9000;      // Hz, loss lowpass cutoff per traversal
    double lossPerM = 0.03;      // amplitude loss per metre (wall friction)
    // role metadata used by builders to re-apply parameters live
    int role = 0, roleIdx = 0;
    double areaMul = 1, lenMul = 1;
    // temperature zone: < 0 = network temperature; 0..1 = blend between the cold and hot gas
    // temperature (1 = right behind the valve, ~0.5 at the tailpipe)
    double zone = -1;
    double steep = 1;            // finite-amplitude steepening weight (0 = linear pipe)
    double stF = 0, stB = 0, offF = 0, offB = 0;   // steepening: smoothed wave value, current read offset

    // runtime
    std::vector<float> fwd, bwd; // fwd: written at end0 -> arrives at end1
    double delay = 10;           // samples (current, glides toward target)
    double lpF = 0, lpB = 0;
    double dcF = 0, dcB = 0;     // slow means: losses act only on the AC part (mass conservation)
    double in0 = 0, in1 = 0;     // waves arriving at end0 / end1
    double out0 = 0, out1 = 0;   // waves leaving end0 / end1
    double cFc = -1, cLen = -1, cA = 0.5, cG = 1; // cached filter coefs
    // gas content of the pipe (species transport between its two ends)
    double cm = 0, cmf = 0, cmb = 0;
    // gas state of this pipe (from its temperature)
    double T = 0, c = 343, rho = 1.2, Z = 1, Y = 1;
};

struct Junction { std::vector<PipeEnd> ends; };

struct NodeEnd {
    PipeEnd e;
    GasNode* node = nullptr;
    const GasNode* inflowSource = nullptr; // composition of gas flowing into the node from the pipe
    double massFlowIntoPipe = 0;           // kg/s (last step)
};

// a pipe end fed by a mass flow that its owner solves each step (a two-stroke's exhaust port: the cylinder's flow
// through the port window against the wave arriving at the pipe - see Engine::stepCylinder). Set mdot after arrive()
// and before step(); positive into the pipe, carrying the fuel / burned fractions ff / bf.
struct FlowEnd {
    PipeEnd e;
    double mdot = 0, ff = 0, bf = 1;
    double extraKg = 0;    // burned gas added to the flow once (an afterfire pop)
};

struct OpenEnd {
    PipeEnd e;
    double prevU = 0;
    double radiated = 0;   // far-field pressure at 1 m (Pa)
    double u = 0;          // outgoing volume velocity (m^3/s)
    float pan = 0;         // -1 left .. +1 right
    // tone extras
    double sound = 0;      // radiated + body + jet noise (Pa @ 1 m) - what the listener gets
    // reflection at the open (unflanged) pipe end: Silva, Guillemain, Kergomard, Mallaroni & Norris, "Approximation
    // formulae for the acoustic radiation impedance of a cylindrical pipe" (arXiv 0811.3625), causal rational model
    // (eq. 16, table 1): R = -(1 + n1 tau s) / (1 + d1 tau s + d2 tau^2 s^2), tau = a / c, n1 0.167, d1 1.393,
    // d2 0.457 - Levine & Schwinger's |R| = 1 - (ka)^2 / 2 and end correction 0.6133 a at low frequency, the
    // radiation loss rising with ka (a wide tailpipe radiates more of the highs). Bilinear transform per end.
    double rb0 = -1, rb1 = 0, rb2 = 0, ra1 = 0, ra2 = 0, rx1 = 0, rx2 = 0, ry1 = 0, ry2 = 0, rTau = -1;
    double uDc = 0, bodyLp = 0, vLp = 0, presHp = 0;
    Biquad jetBp, jetHi;
    Biquad sm1, sm2, air1, air2;   // smooth style: output low-pass, air-noise low-pass (4th-order Butterworth)
    unsigned tick = 0;
    Rng rng{7};
};

class WaveNetwork {
public:
    void init(double fs, double c, double rho0, double p0);
    int addPipe(double area, double length, double absorbFc = 9000, double lossPerM = 0.03);
    void connect(std::initializer_list<PipeEnd> ends);
    void connect(const std::vector<PipeEnd>& ends);
    int addNodeEnd(PipeEnd e, GasNode* node, const GasNode* inflowSource = nullptr);
    int addOpenEnd(PipeEnd e, float pan);
    void addClosedEnd(PipeEnd e);
    int addFlowEnd(PipeEnd e);
    void reset(double burnedFrac = 0); // allocate buffers, snap delays, fill content

    void step(double dt);  // one sample
    // the first part of step() on its own: the waves arriving at the pipe ends this sample (for FlowEnd owners, who
    // need them to solve their flow); step() then skips it
    void arrive();
    double arriving(PipeEnd e) const { const Pipe& p = pipes[e.pipe]; return e.end == 0 ? p.in0 : p.in1; }

    // gas temperature: pipes with a zone get T = cold + (hot - cold) * zone (sound speed and
    // impedance follow, so resonances move with load and warm-up)
    void setGasTemp(double hotK, double coldK);
    // finite-amplitude steepening: high-pressure parts of a wave travel faster (0 = linear)
    void setNonlinear(double k) { nl_ = k * (kGamma + 1.0) / (2.0 * kGamma * p0_); }

    double impedance(int pipe) const { return pipes[pipe].Z; }
    double pressureAt(PipeEnd e) const {
        const Pipe& p = pipes[e.pipe];
        return e.end == 0 ? p.in0 + p.out0 : p.in1 + p.out1;
    }

    std::vector<Pipe> pipes;
    std::vector<Junction> junctions;
    std::vector<NodeEnd> nodeEnds;
    std::vector<OpenEnd> openEnds;
    std::vector<PipeEnd> closedEnds;
    std::vector<FlowEnd> flowEnds;

    // tone controls (set by the exhaust builder)
    void setTone(const EngineParams& p);
    double fs() const { return fs_; }
    double c() const { return c_; }
    double rho0() const { return rho0_; }
    double p0() const { return p0_; }

private:
    void updateGas(Pipe& p) const;
    template <class InOf, class SetOut> void scatter(double dt, InOf inOf, SetOut setOut);
    double fs_ = 96000, c_ = 343, rho0_ = 1.2, p0_ = kPatm, T0_ = 293;
    double hotT_ = 0, coldT_ = 0, nl_ = 0;
    double glide_ = 0.0004;
    double dcA_ = 0.0001, radA_ = 0.5;
    double rasp_ = 1.0, punch_ = 0.4, uDcA_ = 0.001, bodyA_ = 0.02, vA_ = 0.05, stA_ = 0.1;
    int style_ = 0;
    double pmix_ = 0.85, smoothHz_ = 1900;
    int mask_ = 8191;
    int w_ = 0;
    bool arrived_ = false;
};

} // namespace es

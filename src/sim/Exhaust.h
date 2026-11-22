#pragma once
#include "sim/EngineParams.h"
#include "sim/Waveguide.h"
#include "sim/Duct.h"
#include <vector>

namespace es {

// Two-stroke expansion chamber dimensions from Blair's empirical design (Design and Simulation of Two-Stroke Engines,
// SAE 1996, sec. 6.2.5, Fig. 5.7 nomenclature): segment 0 the first pipe from the port (L1, d1), 1-3 the diffuser
// cones (L2-L4, ending at d2, d3, d4), 4 the belly (L5, d4), 5 the rear cone (L6, ending at d7), 6 the tail pipe
// (L7, d7). Metres. d0: the diameter of the exhaust port's full area; lt: tuned length, port to tail-pipe entry.
struct TunedPipeGeom { double d0 = 0, lt = 0, texcC = 0, len[7] = {}, dia[7] = {}; };
TunedPipeGeom tunedPipeGeometry(const EngineParams& p);

// Builds the exhaust waveguide network (headers, collectors, crossover, cat, resonator,
// muffler, tailpipe) from the parameters and owns the exhaust port nodes.
class ExhaustSystem {
public:
    static constexpr double kGasT = 820.0;   // K, mean exhaust gas temperature in the pipes

    void build(const EngineParams& p, const float phase[kMaxCyl], double fs);
    void apply(const EngineParams& p);       // live length / diameter changes
    void step(double dt);
    // ... split around the tuned pipes' own steps, so they can run in parallel (DuctPool): stepPre, every duct's
    // step(dt, ambientK()), stepPost - the same as step()
    void stepPre(double dt);
    void stepPost(double dt);
    double ambientK() const { return ambK_; }
    int ductCount() const { return (int)ducts_.size(); }
    // exhaust gas temperature at the valves (K): sets the temperature (sound speed) of every pipe
    void setGasTemp(double egtK);
    // valved muffler: the flap in its bypass moves toward `target` (0 shut .. 1 open) with the actuator's lag
    void stepValve(double target, double dt);
    double valveOpen() const { return valveOpen_; }
    bool hasValve() const { return !valvePipes_.empty(); }

    GasNode& port(int i) { return ports_[i]; }
    // two-stroke: the port window feeds its pipe directly (no lumped port volume - it soaked up and smoothed the
    // blowdown pulse the tuned pipe works with). Each step: arrive(), then the cylinder solves its port flow against
    // the wave arriving at the pipe (portWave) and sets it (setPortFlow), then step(). port(i) mirrors the pipe end.
    struct PortWave { double in, Z, rho, T, ff, bf; };
    bool directPorts() const { return direct_; }
    void arrive() { net.arrive(); }
    PortWave portWave(int i) const;
    void setPortFlow(int i, double mdot, double ff, double bf) { FlowEnd& f = net.flowEnds[i]; f.mdot = mdot; f.ff = ff; f.bf = bf; }
    void addPortMass(int i, double kg) {
        if (ductPorts()) ducts_[i].inject(kg, ports_[i].T); else net.flowEnds[i].extraKg += kg;
    }
    // two-stroke with a tuned pipe: the pipe from the port to the end of the tail pipe is 1D gas dynamics (Duct.h);
    // the silencer after it stays acoustic. The cylinder solves its port against duct(i) instead of portWave.
    bool ductPorts() const { return !ducts_.empty(); }
    // area of the pipe the port opens into (two-stroke: the discharge coefficient depends on port / pipe area)
    double portPipeArea(int i) const { return ductPorts() ? ducts_[i].portArea() : net.pipes[portPipe_[i]].area; }
    void setAmbient(double k) { ambK_ = k; }
    Duct& duct(int i) { return ducts_[i]; }
    const Duct& duct(int i) const { return ducts_[i]; }
    bool hasTurbine() const { return hasTurbine_; }
    GasNode& turbineIn() { return tin_[0]; }     // exhaust manifold / turbine housing (the first)
    GasNode& turbineOut() { return tout_[0]; }   // turbine exit / downpipe start (the first)
    // two of them: a twin-scroll manifold (two inlets, one turbine), parallel twins (one inlet and outlet each); a
    // sequential pair shares one inlet and one outlet
    int turbineInCount() const { return nTin_; }
    int turbineOutCount() const { return nTout_; }
    GasNode& turbineIn(int i) { return tin_[i]; }
    GasNode& turbineOut(int i) { return tout_[i]; }
    bool twinScroll() const { return twinScroll_; }
    // a screamer pipe: the wastegate's own pipe to the air (nullptr: the wastegate goes into the downpipe)
    GasNode* screamer() { return screamer_ ? &scream_ : nullptr; }
    int portCount() const { return (int)ports_.size(); }
    double portPressure(int i) const { return ports_[i].p(); }

    // sound: sum of radiated pressure (Pa @1m) split by pan
    void radiated(double& left, double& right) const;

    WaveNetwork net;
    // schematic info for the UI
    int groups = 1;
    int outlets = 1;

private:
    enum Role { Primary, Secondary, Collector, Log, Mid, Balance, Tail, Fixed, Valve, ValveInv };   // (ValveInv: shut as the flap opens)
    PipeEnd chain(PipeEnd from, Role role, int idx, double areaMul, double lenParam,
                  double fc = 9000, double loss = 0.03);
    int newPipe(Role role, int idx, double areaMul, double lenParam, double fc, double loss);
    void finishPath(PipeEnd from, float pan, double midFrac);
    PipeEnd muffle(PipeEnd e, double am);   // the muffler (am: its pipe's area / the system pipe's)
    std::vector<int> valvePipes_;           // valved muffler: the bypass pipes behind the flap
    double valveOpen_ = 0, valveApplied_ = -1;
    double areaFor(const Pipe& pp) const;
    double lengthFor(const Pipe& pp) const;

    std::vector<GasNode> ports_;
    std::vector<GasNode> helm_;   // Helmholtz resonator volumes (one per exhaust path; reserved - pointers stay valid)
    // expansion chamber of length L (m) and area ratio ar; extended inlet / outlet tubes (L/2, L/4) when asked
    PipeEnd chamber(PipeEnd from, double ar, double L, double fc, double loss, bool extTubes);
    PipeEnd expansionChamber(PipeEnd from);   // two-stroke tuned pipe after its first pipe (Package 5)
    TunedPipeGeom tp_;                        // (two-stroke with a tuned pipe: its dimensions, set in build)
    bool tsPipe_ = false;
    bool direct_ = false;                     // two-stroke: ports are flow ends (index = port)
    std::vector<int> portPipe_;
    std::vector<Duct> ducts_;                 // two-stroke tuned pipes (one per cylinder; flow end i feeds its silencer)
    double ambK_ = 293.0;
    double helmholtzHz() const;
    GasNode burned_;   // composition of gas flowing back from the pipes (pure products)
    GasNode tin_[2], tout_[2], scream_;
    int nTin_ = 1, nTout_ = 1;
    bool twinScroll_ = false, screamer_ = false;
    bool hasTurbine_ = false;
    double fixedZone_ = 0.6;   // temperature zone for the next fixed (cat / muffler) pieces
    double gasT_ = 0;
    EngineParams p_{};
};

} // namespace es

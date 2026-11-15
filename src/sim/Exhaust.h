#pragma once
#include "sim/EngineParams.h"
#include "sim/Waveguide.h"
#include <vector>

namespace es {

// Builds the exhaust waveguide network (headers, collectors, crossover, cat, resonator,
// muffler, tailpipe) from the parameters and owns the exhaust port nodes.
class ExhaustSystem {
public:
    static constexpr double kGasT = 820.0;   // K, mean exhaust gas temperature in the pipes

    void build(const EngineParams& p, const float phase[kMaxCyl], double fs);
    void apply(const EngineParams& p);       // live length / diameter changes
    void step(double dt);
    // exhaust gas temperature at the valves (K): sets the temperature (sound speed) of every pipe
    void setGasTemp(double egtK);

    GasNode& port(int i) { return ports_[i]; }
    bool hasTurbine() const { return hasTurbine_; }
    GasNode& turbineIn() { return tin_; }     // exhaust manifold / turbine housing
    GasNode& turbineOut() { return tout_; }   // turbine exit / downpipe start
    int portCount() const { return (int)ports_.size(); }
    double portPressure(int i) const { return ports_[i].p(); }

    // sound: sum of radiated pressure (Pa @1m) split by pan
    void radiated(double& left, double& right) const;

    WaveNetwork net;
    // schematic info for the UI
    int groups = 1;
    int outlets = 1;

private:
    enum Role { Primary, Secondary, Collector, Log, Mid, Balance, Tail, Fixed };
    PipeEnd chain(PipeEnd from, Role role, int idx, double areaMul, double lenParam,
                  double fc = 9000, double loss = 0.03);
    int newPipe(Role role, int idx, double areaMul, double lenParam, double fc, double loss);
    void finishPath(PipeEnd from, float pan, double midFrac);
    double areaFor(const Pipe& pp) const;
    double lengthFor(const Pipe& pp) const;

    std::vector<GasNode> ports_;
    GasNode burned_;   // composition of gas flowing back from the pipes (pure products)
    GasNode tin_, tout_;
    bool hasTurbine_ = false;
    double fixedZone_ = 0.6;   // temperature zone for the next fixed (cat / muffler) pieces
    double gasT_ = 0;
    EngineParams p_{};
};

} // namespace es

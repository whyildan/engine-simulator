#pragma once
#include "sim/EngineParams.h"
#include "sim/Waveguide.h"
#include <vector>

namespace es {

// Air path (filter -> throttle(s) -> plenum / ITB stacks -> runners -> ports) and the
// fuel system (EFI or carburetor) that meters fuel into it.
class IntakeSystem {
public:
    void build(const EngineParams& p, double fs, double displacement_m3);
    void apply(const EngineParams& p) { p_ = p; applyGeometry(); }

    // per sim step, before cylinders: throttle airflow, carb fuel, film evaporation
    void stepAir(double dt, double pedal, double rpm, bool fuelEnabled);
    void stepWaves(double dt) { net.step(dt); }

    // EFI actuators (commanded by the ECU)
    void setIac(double v) { iac_ = clampv(v, 0.0, 1.0); }
    // one injector pulse: pw (s) including the dead time, delivered at the actual flow (kg/s)
    void injectPulse(int cyl, double pw, double cycleTime, double flowKgs, double deadS);
    void setSprayRate(double kgs) { sprayRate_ = kgs; }
    // thermal state from the engine: inlet air temperature (K) and how cold the port walls are (0..1)
    void setInletTemp(double K) { inletT_ = K; }
    void setColdWalls(double cold) { cold_ = clampv(cold, 0.0, 1.0); }

    GasNode& port(int i) { return ports_[i]; }
    GasNode& chargeNode() { return charge_; }        // compressor outlet / throttle inlet (boost)
    double throttlePosition() const { return throttlePos_; }
    int inletCount() const { return (int)inlets_.size(); }
    const GasNode& inlet(int i) const { return inlets_[i]; }

    // telemetry
    double map() const;                  // Pa, mean inlet pressure
    double airFlow() const { return airFlow_; }   // kg/s through throttle(s)
    double fuelFlow() const { return fuelFlowAvg_; }
    double iacPct() const { return iac_ * 100.0; }
    double radiated() const { return radiated_; } // induction noise (Pa @1m)

    WaveNetwork net;

private:
    void applyGeometry();
    void addFuel(GasNode& n, double& film, double kg, double filmFrac);

    EngineParams p_{};
    double fs_ = 96000;
    double vdCyl_ = 5e-4;
    double revsPerEvent_ = 2.0;   // shaft revs between intake events of one port
    double ambT_ = 298;
    std::vector<GasNode> inlets_;    // plenum (1) or ITB stacks (n)
    GasNode charge_;                 // boost: charge piping + intercooler volume
    bool boosted_ = false;
    std::vector<GasNode> ports_;
    std::vector<double> inletFilm_, portFilm_;
    std::vector<double> inletFlow_;  // kg/s per inlet (last step)
    std::vector<double> pending_;    // EFI fuel waiting to be sprayed per cylinder (kg)
    std::vector<double> injSpread_;
    double throttlePos_ = 0, prevThrottle_ = 0;
    double airFlow_ = 0, fuelFlow_ = 0, fuelFlowAvg_ = 0;
    double pumpFuel_ = 0;
    double iac_ = 0.35;
    double sprayRate_ = 1e-3;          // kg/s an open injector delivers right now
    double inletT_ = 298, cold_ = 0;
    double radiated_ = 0, prevFlow_ = 0, radLp_ = 0;
    double filtA_ = 0.5;
};

} // namespace es

#pragma once
#include "sim/Fuel.h"
#include "sim/EngineParams.h"
#include "sim/Waveguide.h"
#include "sim/Duct.h"
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
    void stepWaves(double dt) { net.step(dt); stepDucts(dt); }
    // ... split around the two-stroke intake ducts' own steps (parallel: DuctPool): pre, step(dt, ambientK()), post
    void stepWavesPre(double dt) { net.step(dt); stepDuctsPre(dt); }
    void stepWavesPost() { stepDuctsPost(); }
    double ambientK() const { return ambT_; }

    // EFI actuators (commanded by the ECU)
    void setIac(double v) { iac_ = clampv(v, 0.0, 1.0); }
    // one injector pulse: pw (s) including the dead time, delivered at the actual flow (kg/s)
    void injectPulse(int cyl, double pw, double cycleTime, double flowKgs, double deadS);
    void setSprayRate(double kgs) { sprayRate_ = kgs; }
    // carburettor: the share of its normal fuel the jets still get (a float bowl running dry - Engine::fuelStep)
    void setCarbSupply(double f) { carbSupply_ = clampv(f, 0.0, 1.0); }
    // thermal state from the engine: inlet air temperature (K) and how cold the port walls are (0..1)
    void setInletTemp(double K) { inletT_ = K; }
    void setColdWalls(double cold) { cold_ = clampv(cold, 0.0, 1.0); }
    // the fuel in the line: a carburettor meters it by volume through its jets, and alcohols evaporate off cold
    // walls far more slowly
    void setFuel(const FuelProps& f) { fuel_ = f; }

    GasNode& port(int i) { return ports_[i]; }
    // two-stroke: the intake tract from the reed / inlet port to the air box is 1D gas dynamics (Duct.h), one per
    // cylinder - its port end is the crankcase's inlet (Engine::crankcaseStep), its far end the plenum / air box
    bool ductPorts() const { return !ducts_.empty(); }
    Duct& duct(int i) { return ducts_[i]; }
    // nitrous: air-equivalent charge and fuel fed straight into a port, and the charge cooling it causes
    void addPortCharge(int i, double airKg, double fuelKg) { GasNode& g = ports_[i]; g.m += airKg + fuelKg; g.mf += fuelKg; }
    void setPortCooling(double dK) { for (GasNode& g : ports_) g.T = ambT_ + 15.0 - clampv(dK, 0.0, 150.0); }
    GasNode& chargeNode() { return charge_; }        // compressor outlet / throttle inlet (boost)
    double throttlePosition() const { return throttlePos_; }
    int inletCount() const { return (int)inlets_.size(); }
    const GasNode& inlet(int i) const { return inlets_[i]; }
    GasNode& inletNode(int i) { return inlets_[i]; }   // (EGR enters the plenum)
    // EGR: share of the gas entering the plenum that is recirculated exhaust, and its temperature (K)
    void setEgr(double share, double tK) { egrShare_ = std::max(0.0, std::min(1.0, share)); egrT_ = tK; }

    // telemetry
    double map() const;                  // Pa, mean inlet pressure
    double airFlow() const { return airFlow_; }   // kg/s through throttle(s)
    // the throttle flow of the last step split into forward (into the plenum) and reverse (pushed back out), kg/s
    void throttleFlows(double& fwd, double& back) const {
        fwd = back = 0;
        for (double f : inletFlow_) { if (f > 0) fwd += f; else back -= f; }
    }
    double plenumTemp() const { double s = 0; for (const GasNode& g : inlets_) s += g.T; return inlets_.empty() ? inletT_ : s / (double)inlets_.size(); }
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
    double egrShare_ = 0, egrT_ = 600, plenumT0_ = 300;   // EGR mix into the plenum; NA plenum temperature without it
    bool boosted_ = false;
    std::vector<GasNode> ports_;
    std::vector<Duct> ducts_;
    void stepDucts(double dt);
    void stepDuctsPre(double dt);
    void stepDuctsPost();
    std::vector<double> inletFilm_, portFilm_;
    std::vector<double> backM_, backMf_, backMb_;   // standoff: gas pushed back out in front of each throttle
    std::vector<double> inletFlow_;  // kg/s per inlet (last step)
    std::vector<double> pending_;    // EFI fuel waiting to be sprayed per cylinder (kg)
    std::vector<double> injSpread_;
    double throttlePos_ = 0, prevThrottle_ = 0;
    double airFlow_ = 0, fuelFlow_ = 0, fuelFlowAvg_ = 0;
    double pumpFuel_ = 0;
    double iac_ = 0.35;
    double sprayRate_ = 1e-3;          // kg/s an open injector delivers right now
    double carbSupply_ = 1.0;
    double inletT_ = 298, cold_ = 0;
    FuelProps fuel_;
    // wall film evaporation time: longer when cold, and much longer for alcohols when cold
    double filmSlow() const { return 1.0 + 1.5 * cold_ * (fuel_.coldVap / 0.25); }
    double radiated_ = 0, prevFlow_ = 0, radLp_ = 0;
    double filtA_ = 0.5;
};

} // namespace es

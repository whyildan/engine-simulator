#pragma once
// Quasi one-dimensional unsteady gas dynamics in a duct of varying area (a two-stroke's expansion chamber).
//
// Why: the tuned pipe works with pressure waves of the order of the ambient pressure itself - Blair (Design and
// Simulation of Two-Stroke Engines, SAE 1996, Fig. 5.34) shows a 125 cc racing engine's blowdown pulse near 2 atm, a
// suction wave pulling the cylinder to 0.7 atm at BDC and a plugging pulse lifting it from 1 to over 2 atm before the
// port closes. At that size the gas moves at half the speed of sound behind the pulse and its momentum pulls the
// vacuum; small-signal acoustics (the waveguide the other exhausts use) gave a third of the suction and plugging.
// Blair's own simulation is 1D unsteady gas dynamics; this is the same physics with a standard modern scheme.
//
// Equations: the Euler equations for flow in a duct of area A(x), conservative form per cell (mass, momentum, energy
// x A), the wall pressure on the changing area as the momentum source, plus two passive scalars (fuel vapour, burned
// gas) carried with the flow - so the fresh charge that escapes into the pipe is still there for the plugging pulse
// to push back. Ideal gas, gamma and R as in Gas.h.
// Scheme: finite volumes, MUSCL-Hancock (second order: MC-limited slopes, a half-step predictor) with the HLLC
// approximate Riemann solver (Toro, Riemann Solvers and Numerical Methods for Fluid Dynamics, 3rd ed., ch. 10 and
// 14); ~40 mm cells, stepped at ~48 kHz, sub-steps keep the Courant number below 0.8.
// Walls: friction tau = Cf rho u|u| / 2 and heat by the Reynolds analogy h = Cf rho |u| cp / (2 Pr), Cf from Blasius
// (Blair's method, secs. 2.3.1 / 2.4);
// the wall temperature follows the gas slowly (Twall = ambient + 0.55 (mean gas - ambient), chosen so a racing pipe
// near 650 C sits at Blair's assumed exhaust duct wall temperature, 350 C).
// Ends: the port is an orifice (effective area CdA) between the cylinder and the first cell; the duct side obeys the
// characteristic that arrives from the pipe (u - 2a/(gamma-1) constant, isentropic) - the classic "partially open
// end". The far end hands over to the acoustic waveguide (silencer, tail, open end) through the outgoing
// characteristic and the waveguide's pressure relation p = p0 + 2 in + Z u.
#include "sim/Gas.h"
#include <vector>

namespace es {

class Duct {
public:
    // x: positions of the cell faces from the port (m), area: face areas (m2) - same size, >= 3
    // stepsPer: the duct steps once every so many step() calls (the engine runs at 96-192 kHz; the duct at ~48 kHz)
    void build(const std::vector<double>& x, const std::vector<double>& area, double T0, double p0, int stepsPer = 1);

    // ---- port end (cylinder side), once per engine step, before step():
    // solve the port flow for the cylinder's pressure / temperature and the port's effective area (0 = shut).
    // Returns kg/s out of the cylinder (negative: back flow into it).
    double solvePort(double cdA, double pCyl, double tCyl);
    // the gas that actually crosses (after the cylinder's own limits): mass flow, its fuel / burned fractions and
    // stagnation temperature (out of the cylinder: the cylinder's outflow; back flow: the duct's, see backGas)
    void setPortGas(double mdot, double ff, double bf, double t0);
    // back flow: what comes from the first cell (fractions, stagnation temperature)
    void backGas(double& ff, double& bf, double& t0) const;

    // ---- far end: the waveguide's arriving wave (Pa, relative), its impedance Z (Pa s / m3) and gas density, gas
    // temperature (for back flow); after step(), outflow() is the mass flow handed to the waveguide (kg/s) with its
    // composition
    void setOutlet(double in, double Z, double rhoW, double tW, double ffW, double bfW) { inW_ = in; zW_ = Z; rhoW_ = rhoW; tW_ = tW; ffW_ = ffW; bfW_ = bfW; }
    double outflow() const { return mOut_; }
    double outFuelFrac() const { return ffOut_; }
    double outBurnedFrac() const { return bfOut_; }

    // every call: the outlet against the waveguide's wave (sets outflow()), then step() - which advances the duct when
    // its own step is due
    void solveOutlet(double dt);
    void step(double dt, double ambientK);

    int cells() const { return (int)m_.size(); }
    double cellP(int i) const;
    double cellT(int i) const;
    double cellU(int i) const { return m_[i] > 1e-15 ? mom_[i] / m_[i] : 0.0; }
    double cellX(int i) const { return 0.5 * (x_[i] + x_[i + 1]); }
    double cellBurned(int i) const { return m_[i] > 1e-15 ? mb_[i] / m_[i] : 1.0; }
    double cellFuel(int i) const { return m_[i] > 1e-15 ? mf_[i] / m_[i] : 0.0; }
    double length() const { return x_.back(); }
    double portArea() const { return af_[0]; }
    double wallT() const { return tWall_; }
    // an afterfire pop: burned gas of kg at T into the first cell
    void inject(double kg, double T) { m_[0] += kg; mb_[0] += kg; e_[0] += kg * kCv * T; }
    // fuel vapour into the first cell (an injector at the port end of an intake duct), at the cell's temperature
    void addFuel(double kg) { const double T = cellT(0); m_[0] += kg; mf_[0] += kg; e_[0] += kg * kCv * T; }
    // an intake duct: start full of fresh air (build() fills a pipe with burned gas)
    void setFresh() { for (size_t i = 0; i < mb_.size(); ++i) { mb_[i] = 0; mf_[i] = 0; } }

private:
    struct Prim { double r, u, p, yf, yb; };
    Prim prim(int i) const;
    static void hllc(const Prim& L, const Prim& R, double& fm, double& fmu, double& fe, double& fyf, double& fyb);

    std::vector<double> x_, af_, vol_, dx_;     // faces, face areas, cell volumes / lengths
    std::vector<double> invVol_, wall_, dlnA_, dA_, dia_, cf_;
    int cfAge_ = 0;                              // duct steps since the friction factors were refreshed   // per cell, fixed: 1 / volume, wall area, area change (ln, abs)
    std::vector<double> m_, mom_, e_, mf_, mb_; // per cell: kg, kg m/s, J, fuel kg, burned kg
    std::vector<Prim> wl_, wr_, W_, D_;         // scratch: face states, cell states and slopes
    std::vector<double> fm_, fmu_, fe_, ff_, fb_;   // face fluxes (per area)
    // port boundary (face 0)
    double pfM_ = 0, pfMu_ = 0, pfE_ = 0, pfF_ = 0, pfB_ = 0;   // flux into the duct (absolute, per s)
    double portP_ = 0, portU_ = 0, portT0_ = 300, portRho_ = 1;
    // outlet (face N)
    double inW_ = 0, zW_ = 1, rhoW_ = 1, tW_ = 600, ffW_ = 0, bfW_ = 1;
    double mOut_ = 0, ffOut_ = 0, bfOut_ = 1;
    double tWall_ = 600, tMean_ = 600, p0_ = 101325;
    double smax_ = 1000, dxMin_ = 0.02;          // last step's highest wave speed, smallest cell
    double ubPort_ = 0, ubOut_ = 0;              // last boundary velocities: the solves start next to them
    int div_ = 1, cnt_ = 0;                      // steps once every div_ calls
    double accT_ = 0, accM_ = 0, accMu_ = 0, accE_ = 0, accF_ = 0, accB_ = 0;   // port flux summed over those calls
    double oAccM_ = 0, oAccMu_ = 0, oAccE_ = 0, oAccF_ = 0, oAccB_ = 0;        // ... and the outlet flux
};

} // namespace es

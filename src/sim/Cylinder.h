#pragma once
#include "sim/EngineParams.h"
#include "sim/Gas.h"

namespace es {

struct CylGeom {
    bool rotary = false;
    double bore, stroke, rod, cr;
    double r, A, Vd, Vc;            // Vd / Vc per chamber
    double R = 0.105, ecc = 0.015, width = 0.08;   // Wankel geometry
    double strokeEq = 0.08, boreEq = 0.08;         // for friction / heat transfer correlations
    void set(const EngineParams& p) {
        rotary = isRotary(p);
        cr = std::max(4.0, (double)p.compression);
        if (rotary) {
            R = std::max(40.0, (double)p.rotorRadius_mm) * 1e-3;
            ecc = clampv((double)p.eccentricity_mm * 1e-3, 0.004, R * 0.3);
            width = std::max(20.0, (double)p.rotorWidth_mm) * 1e-3;
            Vd = 3.0 * std::sqrt(3.0) * R * ecc * width;   // swept volume per face
            Vc = Vd / (cr - 1.0);
            strokeEq = 3.0 * ecc;
            boreEq = 2.0 * std::sqrt(R * width / kPi);
            // keep the piston fields sane for code that still reads them
            bore = boreEq; stroke = strokeEq; rod = stroke * 2; r = stroke * 0.5; A = Vd / stroke;
            return;
        }
        bore = p.bore_mm * 1e-3; stroke = p.stroke_mm * 1e-3;
        rod = std::max((double)p.rod_mm * 1e-3, stroke * 0.55 + 1e-3);
        r = stroke * 0.5; A = kPi * bore * bore * 0.25; Vd = A * stroke; Vc = Vd / (cr - 1.0);
        strokeEq = stroke; boreEq = bore;
    }
    // piston distance from TDC (m) and its derivative per radian
    void kin(double thetaRad, double& s, double& dsdth) const {
        double sn = std::sin(thetaRad), cs = std::cos(thetaRad);
        double q = std::sqrt(std::max(1e-9, rod * rod - r * r * sn * sn));
        s = r * (1 - cs) + rod - q;
        dsdth = r * sn * (1.0 + r * cs / q);
    }
    // chamber volume, dV/dtheta (per radian of crank / eccentric shaft) and wall area at a
    // cycle angle phi (deg, 0 = firing TDC). Rotary: the volume oscillates twice per 1080 deg.
    void chamber(double phiDeg, double& V, double& dVdth, double& wall) const {
        if (rotary) {
            double x = phiDeg * kDeg * (2.0 / 3.0);
            V = Vc + Vd * 0.5 * (1.0 - std::cos(x));
            dVdth = Vd / 3.0 * std::sin(x);
            // long thin combustion chamber: large surface for its volume
            wall = 3.46 * R * width + 1.2 * R * R * (0.3 + V / (Vc + Vd));
            return;
        }
        double s, ds;
        kin(wrapPos(phiDeg, 360.0) * kDeg, s, ds);
        V = Vc + A * s;
        dVdth = A * ds;
        wall = 2.2 * A + kPi * bore * (s + Vc / A);
    }
};

// Wankel housing port: opens / closes as the rotor flank sweeps over it (shaft degrees)
struct PortSpec {
    double open = 0, close = 0, area = 0, ramp = 25;
    double areaAt(double phi) const {
        double len = wrapPos(close - open, 1080.0);
        double d = wrapPos(phi - open, 1080.0);
        if (d >= len || area <= 0) return 0;
        double x = std::min(d, len - d) / ramp;
        return area * smoothstep(0.0, 1.0, x);
    }
};

struct ValveSpec {
    int n = 2;
    double dia = 0.03;     // m
    double lift = 0.01;    // m
    double dur = 250;      // deg
    double center = 360;   // deg of max lift in the 0..720 cycle (0 = firing TDC)
    double lash = 0.0002;  // m
    double liftAt(double phi) const {
        double d = wrapSym(phi - center, 720.0);
        double h = dur * 0.5;
        if (std::fabs(d) >= h) return 0;
        double x = d / h;
        double f = std::pow(0.5 * (1.0 + std::cos(kPi * x)), 0.75);
        return std::max(0.0, lift * f - lash);
    }
    double area(double L) const {
        if (L <= 0) return 0;
        double curtain = kPi * dia * L;
        double throat = 0.82 * kPi * dia * dia * 0.25;
        return n * 0.66 * std::min(curtain, throat);
    }
};

struct Cylinder {
    // composition / thermo state
    double m = 0, mf = 0, mb = 0, U = 0;
    double p = kPatm, T = 300, V = 1e-4;
    double phase = 0;          // firing phase (deg)
    double prevPhi = 0;
    bool prevIntakeOpen = false, prevExhaustOpen = false;

    // combustion
    bool burning = false, burnEnded = false;
    double burnStart = 0, burnDur = 50, qTotal = 0, fuelBurn = 0, xPrev = 0;
    double pIvc = kPatm, tIvc = 300;

    // per cycle results
    double pmax = 0, pmaxCycle = 0;
    double work = 0, imep = 0;            // J, Pa
    double trappedAir = 0, trappedFuel = 0, lambda = 1, ve = 0;
    double knock = 0, knockCycle = 0;
    int cycles = 0;               // completed working cycles (the knock window counts per cycle)
    double egt = 800;
    double tEvo = 1000;                   // K: burned gas temperature when the exhaust valve opened (blowdown)
    double exhH = 0, exhM = 0;            // mass x temperature, mass that left during this exhaust event
    double afterfireFuel = 0;             // unburned fuel sitting in the primary
    double afterfireAir = 0;              // ... and the fresh air (oxygen) next to it
    double popPending = 0;                // afterfire pressure still to be released (Pa)
    bool misfired = false;
    double dilution = 0;                  // burned gas share of the charge at the spark (residual + reverted exhaust)
    double burnEta = 1;                   // 1 fired, 0 misfired (diagnostics)
    bool valveSeatEvent = false;          // set on the step a valve closes (mechanical click)
    bool knockEvent = false;

    void init(const CylGeom& g, double phi, double Tamb);
};

} // namespace es

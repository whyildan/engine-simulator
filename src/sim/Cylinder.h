#pragma once
#include "sim/EngineParams.h"
#include "sim/Gas.h"
#include "sim/Thermo.h"

namespace es {

struct CylGeom {
    bool rotary = false;
    bool twoStroke = false;
    double sEx = 0, sTr = 0, wEx = 0, wTr = 0;     // two-stroke ports: top edges (m below TDC of the crown), widths (m)
    double sExLow = 0;                             // ... the exhaust's timing edge with the power valve fully lowered
    double sIn = 0, wIn = 0;                       // ... piston-ported inlet: piston position it opens at, port width
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
        twoStroke = isTwoStroke(p);
        if (twoStroke) {
            // a port's top edge sits where the crown is at its opening angle; it is fully open at BDC
            double s, ds;
            const double eo = clampv((double)p.tsExhaustOpen, 45.0, 150.0);
            const double to = clampv((double)p.tsTransferOpen, eo + 2.0, 170.0);
            kin(eo * kDeg, s, ds); sEx = s;
            kin(to * kDeg, s, ds); sTr = s;
            wEx = clampv((double)p.tsExhaustWidth, 0.1, 1.5) * bore;
            wTr = clampv((double)p.tsTransferWidth, 0.1, 2.5) * bore;
            kin(clampv((double)p.tsPvLowOpen, eo, std::max(eo, to - 2.0)) * kDeg, s, ds); sExLow = p.tsPowerValve ? s : sEx;
            // piston port: the skirt's lower edge uncovers it while the piston is near TDC - open from tsInletOpen BTDC
            // to as far ATDC, fully open at TDC; its height is the piston travel from there to TDC, width from its area
            kin(clampv((double)p.tsInletOpen, 20.0, 170.0) * kDeg, s, ds); sIn = s;
            wIn = clampv((double)p.tsInletArea_cm2, 1.0, 80.0) * 1e-4 / std::max(1e-4, sIn);
        }
    }
    // two-stroke: geometric area of a port the crown has uncovered at a crank angle (deg, 0 = TDC)
    double portArea(double phiDeg, double sTop, double w) const {
        double s, ds;
        kin(wrapPos(phiDeg, 360.0) * kDeg, s, ds);
        return w * clampv(s - sTop, 0.0, stroke - sTop);
    }
    // piston-ported inlet: geometric area at a crank angle (open while the piston is within sIn of TDC)
    double inletPortArea(double phiDeg) const {
        double s, ds;
        kin(wrapPos(phiDeg, 360.0) * kDeg, s, ds);
        return wIn * clampv(sIn - s, 0.0, sIn);
    }
    // exhaust port with a power valve at pv (0 = fully lowered, 1 = lifted clear): the valve's edge stands in for the
    // port's top edge; it does not seal perfectly - a narrow leakage path of kPvLeak x the width stays (Blair: "can
    // never seal the port to the extent that a piston can"; share chosen)
    static constexpr double kPvLeak = 0.05;
    double exhaustArea(double phiDeg, double pv) const {
        double s, ds;
        kin(wrapPos(phiDeg, 360.0) * kDeg, s, ds);
        const double top = sExLow + clampv(pv, 0.0, 1.0) * (sEx - sExLow);
        double a = wEx * clampv(s - top, 0.0, stroke - top);
        if (top > sEx) a += kPvLeak * wEx * clampv(std::min(s, top) - sEx, 0.0, top - sEx);
        return a;
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
    double m = 0, mf = 0, mb = 0, U = 0;   // U: sensible internal energy (J), temperature-dependent properties (Thermo.h)
    GasComp yb;                // composition of the burned gas (mb); the rest of the charge is air + fuel vapour
    GasComp yNew;              // composition of the products the current burn makes
    double airPerFuel = 14.7;  // kg air that ends up in the products per kg fuel burned (current burn)
    double p = kPatm, T = 300, V = 1e-4;
    double phase = 0;          // firing phase (deg)
    double prevPhi = 0;
    bool prevIntakeOpen = false, prevExhaustOpen = false;

    // combustion
    bool burning = false, burnEnded = false;
    double burnStart = 0, burnDur = 50, qTotal = 0, fuelBurn = 0, xPrev = 0;
    double pIvc = kPatm, tIvc = 300, vIvc = 5e-4;   // state at intake valve closing (Woschni reference)
    bool fired = false;        // from the spark of a burning charge until the exhaust valve opens (Woschni C2 term)

    // per cycle results
    double pmax = 0, pmaxCycle = 0;
    double dpMax = 0, dpMaxCycle = 0;     // peak pressure rise rate this / last cycle, Pa per crank degree
    double work = 0, imep = 0;            // J, Pa
    double trappedAir = 0, trappedFuel = 0, lambda = 1, ve = 0;
    double knock = 0, knockCycle = 0;
    double slSpark = 0, tuSpark = 300, pSpark = 1e5;   // laminar burning velocity (cm/s), unburned T, p at the last spark
    double lwI = 0;            // Livengood-Wu integral of the current end gas
    bool lwOpen = false;       // end gas alive: from intake closing to 90 % burned (or the exhaust opening)
    double tEnd = 300;         // end-gas temperature (isentropic from intake closing)
    double knockP = 0, knockPCycle = 0, knockT = 0;   // end-gas pressure (Pa) / temperature (K) at the peak (diagnostics)
    int cycles = 0;               // completed working cycles (the knock window counts per cycle)
    double egt = 800;
    double tEvo = 1000;                   // K: burned gas temperature when the exhaust valve opened (blowdown)
    double exhH = 0, exhM = 0;            // mass x temperature, mass that left during this exhaust event
    double afterfireFuel = 0;             // unburned fuel sitting in the primary
    double afterfireAir = 0;              // ... and the fresh air (oxygen) next to it
    double popPending = 0;                // afterfire pressure still to be released (Pa)
    // diesel (Package 4 + D1 / D2): up to four injections a cycle - pilots, main, post - each with its own start and
    // quantity. Pulses that start before any of the cycle's fuel has lit share one ignition delay (a group); a pulse that
    // starts into a burning chamber gets a group of its own with a shorter delay (Fuel.h kPilotMainDelayDiv). Each lit
    // group burns premixed (what was injected during its delay), then diffusion + tail (the rest) on its own clock.
    struct DieselPulse {
        double soi = 0;            // deg BTDC (negative: after TDC), latched at the cycle's first start
        double q = 0, done = 0;    // kg planned / injected
        double t = 0, dur = 0;     // s since its start; common rail: the injector's energizing time
        bool started = false, main = false;
        int group = -1;
    };
    struct DieselGroup {
        double idI = 0;            // ignition delay integral (dt / tau) since the group's first injection
        double startPhi = 0, litPhi = 0;
        double planned = 0, done = 0, pre = 0, diff = 0, tail = 0;   // kg
        bool lit = false, open = true, afterBurn = false, hasMain = false;
    };
    static constexpr int kMaxPulses = 4;
    DieselPulse dPulse[kMaxPulses]; int nPulse = 0;
    DieselGroup dGroup[kMaxPulses]; int nGroup = 0;
    double injDone = 0, injPlanned = 0;             // kg injected / planned this cycle
    bool injStarted = false;                        // this cycle's injection has begun (cleared at exhaust opening)
    double dPhiNow = 0.5;                           // overall equivalence ratio of this cycle's fuel (delay correlation)
    bool dBurning = false;                          // diesel burn running
    double socPhi = 0, soiPhi = 0;                  // deg: first start of combustion / of injection
    double dBurned = 0, dPhi = 0.5;                 // kg burned, overall phi (burn shapes)
    double idDegMain = 0; bool mainOwnDelay = false;   // the main injection's own delay (after a burning pilot)
    // diesel emissions this cycle (D3): NO formed (mol), soot (kg), the burned zone's temperature, the EGR / residual
    // gas per kg of air at intake closing
    bool emisOpen = false;
    double noMol = 0, soot = 0, tZone = 0, dilIvc = 0;
    double idDeg = 0;                     // last ignition delay, deg CA (diagnostics)
    double sparkDelay = 0;                // two-stroke: deg from the spark to the start of heat release (Blair)
    double ringA = 0, ringPh = 0;         // combustion chamber pressure ringing: amplitude Pa, phase rad
    double ringKickA = 0;                 // amplitude the last burn excited (diagnostics), Pa
    double dpStep = 0;                    // highest pressure rise rate since the start of combustion, Pa / deg
    double ringKickPeak = 0;              // pressure rise rate (Pa / deg) that last excited the ringing this cycle
    // two-stroke (Package 5): scavenging of this cycle (the gas that leaves: sim/Scavenge.h)
    double mdotTr = 0;                              // two-stroke: mass flow in the transfer passages (kg/s, + into the cylinder)
    double scDelivered = 0, scFreshEvo = 0;         // kg this cycle: fresh air delivered through the transfers; fresh air
                                                    // in the cylinder when the exhaust opened
    double dr = 0, te = 0, se = 0, ce = 0;          // last cycle: delivery ratio, trapping, scavenging and charging efficiency
    bool misfired = false;
    double dilution = 0;                  // burned gas share of the charge at the spark (residual + reverted exhaust)
    double burnEta = 1;                   // 1 fired, 0 misfired (diagnostics)
    bool valveSeatEvent = false;          // set on the step a valve closes (mechanical click)
    bool knockEvent = false;

    void init(const CylGeom& g, double phi, double Tamb, const GasComp& products);
    void setT(double t) { T = t; U = thermo::energy(m - mb, mb, yb, T); }   // after a direct change of T
    void solveT() { T = thermo::temperature(m - mb, mb, yb, U, T); }
};

} // namespace es

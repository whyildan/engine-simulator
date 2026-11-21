#pragma once
// Two-stroke squish band (T5): squish velocity from the chamber geometry, and the burn period it gives.
//
// Squish velocity - Blair, Design and Simulation of Two-Stroke Engines (SAE 1996), sec. 4.5.1, eqs. 4.5.1-4.5.8, central
// chamber (Fig. 4.13a): from trapping (exhaust closing) to TDC the trapped charge is spread over the squish band and the
// bowl in proportion to their volumes; as the piston rises the band shrinks faster than the bowl, and the mass that
// must move into the bowl in a step crosses the "curtain" round the bowl's edge:
//   dm_sq = m_t (Vs1 / Vc1 - Vs2 / Vc2),  c_sq = dm_sq / (rho_2 A_sq dt),  A_sq = pi d_b (x_s + (x1 + x2) / 2),
// rho_2 = m_t / Vc2 - so c_sq is pure geometry and proportional to engine speed. Squished kinetic energy (eq. 4.5.12)
// with the trapped charge at 1 atm / 20 C as Blair assumes. Check (Blair Fig. 4.14: 70 x 70 x 125 mm, 5000 rpm, exhaust
// opens 104 ATDC, trapped CR 7, squish area ratio 0.5, clearance 1.2 mm): 20.9 m/s at 10 deg BTDC, 6.94 mJ.
//
// Burn period - Blair eq. 4.3.1: the flame front moves at the laminar velocity plus the turbulence velocity, and the
// squish velocity stands in for the turbulence (sec. 4.5.3, Kee's measured flame speeds: a central chamber 24.5 m/s with
// c_sq 11.6, a QUB deflector chamber 50 m/s with c_sq 38 - both leave ~12.5 m/s, "the laminar velocity", in that
// QUB 400 engine at 3000 rpm). Here that remainder is taken to scale with the mean piston speed (1.84 x 7.0 m/s in
// Kee's test - chosen: it keeps the burn period independent of speed, as Blair finds it "independent of speed and
// load"), and the burn period is the time to cover kFlameBores x bore: Kee's LS400 test (85 mm bore, 3000 rpm, full
// throttle, 24.5 m/s) burned in b = 60 deg (Blair sec. 4.3.6) -> 0.96 bores. One engine's data: both constants are
// calibrations.
#include <algorithm>
#include <cmath>

namespace es {

struct SquishResult { double vMax = 0, degBtdc = 0, keJ = 0; };

// bore / stroke / rod in m, eoAtdc: the exhaust opening (it closes as far before TDC), crTrapped: trapped compression
// ratio, csq: squish area ratio, xs: squish clearance (m)
inline SquishResult squishVelocity(double bore, double stroke, double rod, double eoAtdc, double crTrapped, double csq,
                                   double xs, double rpm) {
    SquishResult r;
    const double pi = 3.14159265358979, deg = pi / 180.0;
    const double ap = pi * 0.25 * bore * bore, as = csq * ap, ab = ap - as;
    if (csq <= 0.0 || ab <= 0.0 || rpm <= 0.0) return r;
    const double db = std::sqrt(4.0 * ab / pi), rc = 0.5 * stroke;
    auto pos = [&](double thDeg) {   // piston distance from TDC
        const double t = thDeg * deg, sn = std::sin(t);
        return rc * (1.0 - std::cos(t)) + rod - std::sqrt(rod * rod - rc * rc * sn * sn);
    };
    const double xt = pos(eoAtdc);                          // trapped stroke: from the exhaust closing
    const double vc = ap * xt / std::max(1.01, crTrapped - 1.0);
    const double vdome = vc - ap * xs;                      // the bowl above the squish plane
    if (vdome <= 0.0) return r;
    const double mt = 101325.0 / (287.0 * 293.0) * (vc + ap * xt);
    const double dth = 0.25, dt = dth / (360.0 * rpm / 60.0);
    for (double th = eoAtdc; th > dth * 0.5; th -= dth) {   // crank deg before TDC, compression
        const double x1 = pos(th), x2 = pos(th - dth);
        const double vs1 = as * (xs + x1), vs2 = as * (xs + x2);
        const double vc1 = vs1 + ab * (xs + x1) + vdome, vc2 = vs2 + ab * (xs + x2) + vdome;
        const double dm = mt * (vs1 / vc1 - vs2 / vc2);
        const double asq = pi * db * (xs + 0.5 * (x1 + x2));
        const double c = dm / (mt / vc2 * asq * dt);
        r.keJ += 0.5 * dm * c * c;
        if (c > r.vMax) { r.vMax = c; r.degBtdc = th - 0.5 * dth; }
    }
    return r;
}

constexpr double kFlameLamPerUp = 1.84;   // "laminar" remainder of the flame speed per m/s of mean piston speed
constexpr double kFlameBores = 0.96;      // flame travel over the burn period, in bores

// Vibe burn period (deg) from the squish band; speed cancels out (both velocities scale with it)
inline double squishBurnDeg(double bore, double stroke, double rod, double eoAtdc, double crTrapped, double csq, double xs) {
    const double rpm = 6000.0;
    const double up = 2.0 * stroke * rpm / 60.0;
    const double cfl = kFlameLamPerUp * up + squishVelocity(bore, stroke, rod, eoAtdc, crTrapped, csq, xs, rpm).vMax;
    return 6.0 * rpm * kFlameBores * bore / cfl;
}

} // namespace es

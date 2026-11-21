#pragma once
// Two-stroke scavenging: what leaves through the exhaust port while the cylinder is being scavenged.
//
// Blair, Design and Simulation of Two-Stroke Engines (SAE 1996), sec. 3.3:
//  - measured on the QUB single-cycle gas scavenging rig (isothermal, isobaric, by volume), each cylinder's
//    scavenging efficiency against scavenge ratio is fitted by ln(1 - SEv) = K0 + K1 SRv + K2 SRv^2 (eq. 3.3.3) -
//    Fig. 3.16 lists K0..K2 for ten real cylinders. (The Benson-Brandham displacement / mixing / short-circuit model
//    cannot follow them: their log slopes lie between -1 and -2, the model allows only 0..-1 - Fig. 3.14.)
//  - the purity of the gas entering the exhaust port is the slope of that curve (eq. 3.3.11 / 3.3.12):
//    Pe = 1 - dSEv/dSRv = 1 + (K1 + 2 K2 SRv) exp(K0 + K1 SRv + K2 SRv^2)
//  - in a firing engine the fresh air and the burned gas differ in temperature: Tex = Tta + Ctemp Tcy (3.3.13),
//    Ctemp = 0.65 Kse Kcp (3.3.14) with Blair's fitted Kse(SE trapped last cycle) and Kcp(mean piston speed), and
//    SE Tta + (1 - SE) Tex = Tcy (3.3.15, equal specific heats here). Cylinder SE by mass -> by volume (3.3.16):
//    SEv = SE Tta / Tcy; invert the curve for SRv; the exit purity (by volume = by mass here: one gas constant) and the
//    exit temperature Te = Pe Tta + (1 - Pe) Tex (3.3.18).
// Read from a scanned copy (the K table and equations checked against each other: SEv(0) = 1 - exp(K0) ~ 0).
#include "core/MathUtil.h"
#include <cmath>

namespace es {

struct ScavCylinder { const char* name; const char* type; double k0, k1, k2; };
inline constexpr ScavCylinder kScavCylinders[10] = {
    {"UNIFLOW", "ported uniflow", 3.3488e-2, -1.3651, -0.21735},
    {"LOOPSAW", "2-port loop (chainsaw)", 2.6355e-2, -1.2916, -0.12919},
    {"SCRE", "3-port loop", 1.6182e-2, -1.1682, -0.28773},
    {"CROSS", "4-port + deflector", 1.2480e-2, -1.2289, -0.090576},
    {"QUBCR", "QUB cross", 2.0505e-2, -1.3377, -0.16627},
    {"GPBDEF", "GPB cross", 8.0384e-2, -1.7861, 0.29291},
    {"YAM1", "5-port loop", 1.8621e-2, -0.91737, -0.46621},
    {"YAM6", "5-port loop", 3.1655e-2, -1.0587, -0.16814},
    {"YAM12", "5-port loop (poor)", -1.4568e-2, -0.84285, -0.28438},
    {"YAM14", "5-port loop (good)", 2.9204e-2, -1.0508, -0.34226},
};

// scavenging efficiency (by volume) at a scavenge ratio (by volume)
inline double scavSE(const ScavCylinder& c, double sr) {
    return clampv(1.0 - std::exp(c.k0 + c.k1 * sr + c.k2 * sr * sr), 0.0, 1.0);
}

// purity (fresh share) and temperature of the gas entering the exhaust port. se: the cylinder's fresh share by mass
// now; tcy: its mean temperature; seTrapped: the scavenging efficiency trapped last cycle; cp: mean piston speed m/s
inline void scavExit(const ScavCylinder& c, double se, double tcy, double seTrapped, double cp, double& purity, double& tExit) {
    se = clampv(se, 0.0, 1.0);
    const double s = clampv(seTrapped, 0.0, 1.0);
    const double kse = clampv(-16.12 + 100.15 * s - 229.72 * s * s + 229.56 * s * s * s - 82.898 * s * s * s * s, 0.0, 1.0);
    const double kcp = clampv(-0.34564 + 0.17968 * cp - 8.0818e-3 * cp * cp + 1.2161e-4 * cp * cp * cp, 0.0, 1.0);
    const double ct = 0.65 * kse * kcp;
    const double tta = tcy * (1.0 - (1.0 - se) * ct), tex = tta + ct * tcy;
    const double sev = clampv(se * tta / std::max(1.0, tcy), 0.0, 0.999999);
    // invert ln(1 - SEv) = K0 + K1 SR + K2 SR^2 for the smallest SR >= 0 (SE rises with SR)
    const double y = std::log(1.0 - sev) - c.k0;
    double sr;
    if (std::fabs(c.k2) < 1e-9) sr = y / c.k1;
    else {
        const double d = c.k1 * c.k1 + 4.0 * c.k2 * y;
        if (d < 0) sr = -c.k1 / (2.0 * c.k2);   // (beyond the fitted curve's turning point: hold it there)
        else {
            const double r1 = (-c.k1 - std::sqrt(d)) / (2.0 * c.k2), r2 = (-c.k1 + std::sqrt(d)) / (2.0 * c.k2);
            sr = (r1 >= 0 && r2 >= 0) ? std::min(r1, r2) : std::max(r1, r2);
        }
    }
    sr = std::max(0.0, sr);
    purity = clampv(1.0 + (c.k1 + 2.0 * c.k2 * sr) * std::exp(c.k0 + c.k1 * sr + c.k2 * sr * sr), 0.0, 1.0);
    tExit = purity * tta + (1.0 - purity) * tex;
}

} // namespace es

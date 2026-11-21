#pragma once
// Laminar burning velocity and the burn duration it sets (Group A4, "hybrid" combustion model).
//
// Laminar burning velocity: Metghalchi & Keck, Combustion and Flame 48:191-210 (1982), eq. (24) / Table 3A:
//   S_u = S_u0 (Tu / 298 K)^alpha (p / 1 atm)^beta (1 - 2.1 f)
//   S_u0 = B_M + B_2 (phi - phi_M)^2,  alpha = 2.18 - 0.8 (phi - 1),  beta = -0.16 + 0.22 (phi - 1)
//   indolene (RMFD-303, the gasoline reference): phi_M 1.13, B_M 27.58 cm/s, B_2 -78.34 cm/s
//   f = mass fraction of diluent (burned gas: residual / reverted exhaust). Measured for phi 0.8-1.5, p 0.4-50 atm,
//   Tu 298-700 K, f 0-0.2. The inputs are held inside that range: the correlation is not extrapolated. What lies
//   beyond it - the lean limit, heavy dilution - is the misfire model's job (Engine: lean / dilution misfire).
// Fuel effect: the ratio of the laminar burning velocities measured in one study (Gulder 1982, 19th Symposium on
// Combustion: maxima near phi 1.075 of 0.455 m/s isooctane, 0.47 ethanol, 0.50 methanol) - FuelProps::slRatio.
// (Metghalchi & Keck's own methanol data are 1.34x their indolene; that study and modern data disagree, so one
// consistent source is used for all three fuels.)
//
// Burn duration (the Wiebe shape is kept), in two parts:
//  - the flame kernel (the 0-10 % part of the Wiebe burn, 28 % of its duration) grows as a laminar flame: ~ 1 / S_L;
//  - the developed turbulent flame moves at u_t = C u' Da^1/4 (Zimont; LUSIE entrainment model, Int. J. Engine
//    Research 2018, eq. 4), Da = (L / u') / (delta_L / S_L) with the flame thickness delta_L = alpha / S_L, so for a
//    given turbulence u_t ~ S_L^1/2 alpha^-1/4. Thermal diffusivity of the unburned gas alpha ~ T^1.75 / p
//    (k ~ T^0.75, rho ~ p / T): a low-pressure charge has a thick flame that burns slowly.
//   duration = base(rpm) x [0.28 r + 0.72 sqrt(r) (alpha / alpha_ref)^1/4],  r = S_L,ref / S_L
// base(rpm) carries the turbulence (u' grows with the piston speed) - the empirical rpm scaling the model had. The
// reference state is the K20A at 4500 rpm full load at the spark (752 K, 18.3 bar, phi 1.15), where the
// duration equals the previous model's. Light load comes out ~1.45-1.6x longer than full load, as engines show.
#include "sim/Fuel.h"
#include <algorithm>
#include <cmath>

namespace es {

inline double laminarFlameSpeed(const FuelProps& f, double phi, double TuK, double pPa, double dilution) {
    phi = std::clamp(phi, 0.8, 1.5);
    const double su0 = 27.58 - 78.34 * (phi - 1.13) * (phi - 1.13);   // cm/s
    const double a = 2.18 - 0.8 * (phi - 1.0), b = -0.16 + 0.22 * (phi - 1.0);
    const double tu = std::clamp(TuK, 298.0, 700.0), p = std::clamp(pPa / 101325.0, 0.4, 50.0);
    const double dil = 1.0 - 2.1 * std::clamp(dilution, 0.0, 0.2);
    return f.slRatio * su0 * std::pow(tu / 298.0, a) * std::pow(p, b) * dil;   // cm/s
}

// reference state (see above; S_L with the inputs held in range: 117.8 x (700 / 752)^2.06 = 101.7 cm/s)
constexpr double kFlameRefCmS = 101.7, kFlameRefK = 752.0, kFlameRefPa = 18.3e5;

// the two parts of the burn, relative to the reference state: the kernel (laminar, ~ 1 / S_L) and the developed
// turbulent flame (~ S_L^-1/2 alpha^1/4)
inline double flameKernelFactor(double sl) { return kFlameRefCmS / std::max(1.0, sl); }
inline double flameTurbulentFactor(double sl, double TuK, double pPa) {
    const double alphaRatio = std::pow(std::max(250.0, TuK) / kFlameRefK, 1.75) * kFlameRefPa / std::max(1e4, pPa);
    return std::sqrt(flameKernelFactor(sl)) * std::pow(alphaRatio, 0.25);
}

// burn duration multiplier for the laminar burning velocity S_L (cm/s) of unburned gas at TuK, pPa
inline double flameDurationFactor(double sl, double TuK, double pPa) {
    return 0.28 * flameKernelFactor(sl) + 0.72 * flameTurbulentFactor(sl, TuK, pPa);
}

} // namespace es

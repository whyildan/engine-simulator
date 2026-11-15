#pragma once
// Thermodynamic helpers: ideal gas + compressible orifice flow.
#include "core/MathUtil.h"

namespace es {

constexpr double kR = 287.0;          // J/(kg K)
constexpr double kGamma = 1.35;       // average for air / combustion products
constexpr double kCv = kR / (kGamma - 1.0);
constexpr double kCp = kCv + kR;
constexpr double kPatm = 101325.0;
constexpr double kStoich = 14.7;      // gasoline AFR
constexpr double kLHV = 44.0e6;       // J/kg gasoline

// Gas "bin" that a flow carries: total mass with a fuel fraction and burned fraction
struct GasNode {
    double V = 1e-4;       // m^3
    double T = 300;        // K (isothermal node)
    double m = 0;          // kg total
    double mf = 0;         // unburned fuel vapour mass
    double mb = 0;         // burned products mass
    double p() const { return m * kR * T / V; }
    void setPressure(double p) { m = p * V / (kR * T); }
    double fuelFrac() const { return m > 1e-12 ? mf / m : 0; }
    double burnedFrac() const { return m > 1e-12 ? mb / m : 0; }
};

// Mass flow (kg/s, >= 0) through an orifice of effective area CdA from upstream (p0,T0) to downstream p1.
inline double orificeFlow(double CdA, double pUp, double TUp, double pDn) {
    if (CdA <= 0 || pUp <= pDn) return 0;
    constexpr double g = kGamma;
    const double pr = pDn / pUp;
    static const double prCrit = std::pow(2.0 / (g + 1.0), g / (g - 1.0));
    static const double psiChoked = std::sqrt(g) * std::pow(2.0 / (g + 1.0), (g + 1.0) / (2.0 * (g - 1.0)));
    double psi;
    if (pr <= prCrit) {
        psi = psiChoked;
    } else if (pr < 0.995) {
        psi = std::sqrt(2.0 * g / (g - 1.0) * (std::pow(pr, 2.0 / g) - std::pow(pr, (g + 1.0) / g)));
    } else {
        // linearise near equal pressure to avoid the infinite slope at pr -> 1
        const double pr0 = 0.995;
        const double psi0 = std::sqrt(2.0 * g / (g - 1.0) * (std::pow(pr0, 2.0 / g) - std::pow(pr0, (g + 1.0) / g)));
        psi = psi0 * (1.0 - pr) / (1.0 - pr0);
    }
    return CdA * pUp / std::sqrt(kR * TUp) * psi;
}

// Signed flow a->b with stability limiter: never transfers more than `frac` of the mass
// needed to equalise the two volumes in one step.
inline double limitedFlow(double CdA, double pa, double Ta, double Va, double pb, double Tb, double Vb,
                          double dt, double frac = 0.4) {
    double mdot = pa >= pb ? orificeFlow(CdA, pa, Ta, pb) : -orificeFlow(CdA, pb, Tb, pa);
    // equalisation mass (approximate, isothermal)
    double k = kR * Ta / Va + kR * Tb / Vb;
    double meq = (pa - pb) / k;
    double lim = std::fabs(meq) * frac / dt;
    if (std::fabs(mdot) > lim) mdot = mdot > 0 ? lim : -lim;
    return mdot;
}

} // namespace es

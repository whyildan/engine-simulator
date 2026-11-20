#include "sim/Thermo.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace es::thermo {

namespace {

constexpr double kRu = 8.314462618;   // J/mol K
// NASA 7-coefficient polynomials, GRI-Mech 3.0 (Cantera data/gri30.yaml): [low T range, high T range], Tmid 1000 K
struct Nasa { double lo[7], hi[7], M; };   // M in kg/mol
const Nasa kNasa[kSpCount] = {
    // N2 (300 - 1000 - 5000 K)
    {{3.298677, 1.4082404e-03, -3.963222e-06, 5.641515e-09, -2.444854e-12, -1020.8999, 3.950372},
     {2.92664, 1.4879768e-03, -5.68476e-07, 1.0097038e-10, -6.753351e-15, -922.7977, 5.980528}, 28.014e-3},
    // O2 (200 - 1000 - 3500 K)
    {{3.78245636, -2.99673416e-03, 9.84730201e-06, -9.68129509e-09, 3.24372837e-12, -1063.94356, 3.65767573},
     {3.28253784, 1.48308754e-03, -7.57966669e-07, 2.09470555e-10, -2.16717794e-14, -1088.45772, 5.45323129}, 31.998e-3},
    // CO2
    {{2.35677352, 8.98459677e-03, -7.12356269e-06, 2.45919022e-09, -1.43699548e-13, -4.83719697e+04, 9.90105222},
     {3.85746029, 4.41437026e-03, -2.21481404e-06, 5.23490188e-10, -4.72084164e-14, -4.8759166e+04, 2.27163806}, 44.009e-3},
    // H2O
    {{4.19864056, -2.0364341e-03, 6.52040211e-06, -5.48797062e-09, 1.77197817e-12, -3.02937267e+04, -0.849032208},
     {3.03399249, 2.17691804e-03, -1.64072518e-07, -9.7041987e-11, 1.68200992e-14, -3.00042971e+04, 4.9667701}, 18.015e-3},
    // CO
    {{3.57953347, -6.1035368e-04, 1.01681433e-06, 9.07005884e-10, -9.04424499e-13, -1.4344086e+04, 3.50840928},
     {2.71518561, 2.06252743e-03, -9.98825771e-07, 2.30053008e-10, -2.03647716e-14, -1.41518724e+04, 7.81868772}, 28.010e-3},
    // H2
    {{2.34433112, 7.98052075e-03, -1.9478151e-05, 2.01572094e-08, -7.37611761e-12, -917.935173, 0.683010238},
     {3.3372792, -4.94024731e-05, 4.99456778e-07, -1.79566394e-10, 2.00255376e-14, -950.158922, -3.20502331}, 2.016e-3},
};
const double* coef(int k, double T) { return T < 1000.0 ? kNasa[k].lo : kNasa[k].hi; }
double cpMol(int k, double T) { const double* a = coef(k, T); return kRu * (a[0] + T * (a[1] + T * (a[2] + T * (a[3] + T * a[4])))); }
double sMol(int k, double T) {   // standard entropy J/mol K
    const double* a = coef(k, T);
    return kRu * (a[0] * std::log(T) + T * (a[1] + T * (a[2] / 2 + T * (a[3] / 3 + T * a[4] / 4))) + a[6]);
}
double hMol(int k, double T) {   // total (formation + sensible) J/mol
    const double* a = coef(k, T);
    return kRu * T * (a[0] + T * (a[1] / 2 + T * (a[2] / 3 + T * (a[3] / 4 + T * a[4] / 5))) + a[5] / T);
}

constexpr double kStep = 2.0;
constexpr int kN = (int)((kTmax - kTmin) / kStep) + 1;

struct Tables {
    // per species, per kg: sensible u and h, cv
    std::vector<double> u[kSpCount], h[kSpCount], cv[kSpCount];
    std::vector<double> uA, hA, cvA, sA;   // air (sA: standard entropy per kg)
    double rAir = 287;
    GasComp air;
    double lhvCO = 0, lhvH2 = 0;
    Tables() {
        air.y[kSpN2] = 0.79 * kNasa[kSpN2].M / (0.79 * kNasa[kSpN2].M + 0.21 * kNasa[kSpO2].M);
        air.y[kSpO2] = 1.0 - air.y[kSpN2];
        for (int k = 0; k < kSpCount; ++k) {
            u[k].resize(kN); h[k].resize(kN); cv[k].resize(kN);
            const double M = kNasa[k].M, R = kRu / M, h298 = hMol(k, 298.15) / M;
            for (int i = 0; i < kN; ++i) {
                const double T = kTmin + i * kStep;
                h[k][i] = hMol(k, T) / M - h298;
                u[k][i] = h[k][i] - R * T;
                cv[k][i] = cpMol(k, T) / M - R;
            }
        }
        uA.resize(kN); hA.resize(kN); cvA.resize(kN); sA.resize(kN);
        for (int i = 0; i < kN; ++i) {
            uA[i] = hA[i] = cvA[i] = sA[i] = 0;
            const double T = kTmin + i * kStep;
            for (int k = 0; k < kSpCount; ++k) {
                uA[i] += air.y[k] * u[k][i]; hA[i] += air.y[k] * h[k][i]; cvA[i] += air.y[k] * cv[k][i];
                sA[i] += air.y[k] * sMol(k, T) / kNasa[k].M;   // (mixing entropy is constant for a fixed composition)
            }
        }
        rAir = 0;
        for (int k = 0; k < kSpCount; ++k) rAir += air.y[k] * kRu / kNasa[k].M;
        lhvCO = (hMol(kSpCO, 298.15) + 0.5 * hMol(kSpO2, 298.15) - hMol(kSpCO2, 298.15)) / kNasa[kSpCO].M;
        lhvH2 = (hMol(kSpH2, 298.15) + 0.5 * hMol(kSpO2, 298.15) - hMol(kSpH2O, 298.15)) / kNasa[kSpH2].M;
    }
};
const Tables& tab() { static const Tables t; return t; }

inline void locate(double T, int& i, double& w) {
    const double x = (std::clamp(T, kTmin, kTmax) - kTmin) / kStep;
    i = std::min(kN - 2, (int)x);
    w = x - i;
}
inline double mixAt(const std::vector<double>* s, const GasComp& c, int i) {
    double v = 0;
    for (int k = 0; k < kSpCount; ++k) v += c.y[k] * s[k][i];
    return v;
}
inline double interp(const std::vector<double>* s, const GasComp& c, double T) {
    int i; double w; locate(T, i, w);
    return mixAt(s, c, i) * (1 - w) + mixAt(s, c, i + 1) * w;
}

} // namespace

const GasComp& air() { return tab().air; }
double u(const GasComp& c, double T) { return interp(tab().u, c, T); }
double h(const GasComp& c, double T) { return interp(tab().h, c, T); }
double cv(const GasComp& c, double T) { return interp(tab().cv, c, T); }
double gasR(const GasComp& c) {
    double r = 0;
    for (int k = 0; k < kSpCount; ++k) r += c.y[k] / kNasa[k].M;
    return kRu * r;
}
double lhvCO() { return tab().lhvCO; }
double lhvH2() { return tab().lhvH2; }

double energy(double ma, double mb, const GasComp& pb, double T) {
    const Tables& t = tab();
    int i; double w; locate(T, i, w);
    const double e0 = ma * t.uA[i] + mb * mixAt(t.u, pb, i), e1 = ma * t.uA[i + 1] + mb * mixAt(t.u, pb, i + 1);
    return e0 + (e1 - e0) * w;
}

double enthalpy(double xb, const GasComp& pb, double T) {
    const Tables& t = tab();
    int i; double w; locate(T, i, w);
    xb = std::clamp(xb, 0.0, 1.0);
    const double e0 = (1 - xb) * t.hA[i] + xb * mixAt(t.h, pb, i), e1 = (1 - xb) * t.hA[i + 1] + xb * mixAt(t.h, pb, i + 1);
    return e0 + (e1 - e0) * w;
}

double temperature(double ma, double mb, const GasComp& pb, double U, double T0) {
    const Tables& t = tab();
    auto E = [&](int i) { return ma * t.uA[i] + mb * mixAt(t.u, pb, i); };
    int i; double w; locate(T0, i, w);
    // u is monotonic in T: walk from the last temperature to the bracketing interval (a step or two per call)
    double e0 = E(i), e1 = E(i + 1);
    while (U < e0 && i > 0) { --i; e1 = e0; e0 = E(i); }
    while (U > e1 && i < kN - 2) { ++i; e0 = e1; e1 = E(i + 1); }
    const double f = e1 > e0 ? (U - e0) / (e1 - e0) : 0.0;
    return std::clamp(kTmin + (i + std::clamp(f, 0.0, 1.0)) * kStep, kTmin, kTmax);
}

double isentropicAir(double T0, double pr, double Tguess) {
    const Tables& t = tab();
    int i0; double w0; locate(T0, i0, w0);
    const double s = t.sA[i0] + (t.sA[i0 + 1] - t.sA[i0]) * w0 + t.rAir * std::log(std::max(1e-6, pr));
    int i; double w; locate(Tguess, i, w);
    double s0 = t.sA[i], s1 = t.sA[i + 1];
    while (s < s0 && i > 0) { --i; s1 = s0; s0 = t.sA[i]; }
    while (s > s1 && i < kN - 2) { ++i; s0 = s1; s1 = t.sA[i + 1]; }
    const double f = s1 > s0 ? (s - s0) / (s1 - s0) : 0.0;
    return std::clamp(kTmin + (i + std::clamp(f, 0.0, 1.0)) * kStep, kTmin, kTmax);
}

Burn burn(double C, double H, double O, double airPerFuel) {
    Burn b;
    const double Mair = 0.79 * kNasa[kSpN2].M + 0.21 * kNasa[kSpO2].M;   // kg/mol
    const double o2Need = C + H / 4.0 - O / 2.0;                         // mol O2 per kg fuel for complete burning
    const double o2Air = std::max(0.0, airPerFuel) / Mair * 0.21;       // mol O2 the air brings
    double nCO2, nCO, nH2O, nH2, o2Used;
    if (o2Air >= o2Need) {           // lean / stoichiometric: complete; the excess air stays air
        nCO2 = C; nCO = 0; nH2O = H / 2.0; nH2 = 0; o2Used = o2Need;
    } else {                         // rich: all the oxygen is used; CO / H2 from the water-gas shift equilibrium
        constexpr double K = 3.5;
        const double Ob = 2.0 * o2Air + O;   // oxygen atoms available
        // a = CO2: (K - 1) a^2 + [K (H/2 - Ob + C) + Ob] a - C (Ob - C) = 0
        const double A = K - 1.0, B = K * (H / 2.0 - Ob + C) + Ob, Cq = -C * std::max(0.0, Ob - C);
        double a = A > 1e-12 ? (-B + std::sqrt(std::max(0.0, B * B - 4 * A * Cq))) / (2 * A) : -Cq / B;
        a = std::clamp(a, 0.0, C);
        nCO2 = a; nCO = C - a;
        nH2O = std::clamp(Ob - a - C, 0.0, H / 2.0);
        nH2 = H / 2.0 - nH2O;
        o2Used = o2Air;
    }
    const double nN2 = o2Used * 0.79 / 0.21;
    const double m[kSpCount] = {nN2 * kNasa[kSpN2].M, 0.0, nCO2 * kNasa[kSpCO2].M, nH2O * kNasa[kSpH2O].M, nCO * kNasa[kSpCO].M,
                                nH2 * kNasa[kSpH2].M};
    double sum = 0;
    for (double x : m) sum += x;
    for (int k = 0; k < kSpCount; ++k) b.products.y[k] = sum > 0 ? m[k] / sum : 0.0;
    b.airUsed = o2Used / 0.21 * Mair;
    b.energyLeft = m[kSpCO] * tab().lhvCO + m[kSpH2] * tab().lhvH2;
    return b;
}

} // namespace es::thermo

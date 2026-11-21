#include "sim/Duct.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
constexpr double g = kGamma;
constexpr double gm1 = kGamma - 1.0;
constexpr double kCf = 0.005;     // wall friction coefficient (Fanning), chosen
constexpr double kHMin = 10.0;    // W/m2K: heat transfer floor of still gas, chosen
constexpr double kPMin = 2000.0;  // Pa: positivity floor

inline double minmod(double a, double b) { return a * b <= 0 ? 0.0 : (std::fabs(a) < std::fabs(b) ? a : b); }

// Illinois (regula falsi that does not stall) on f over [a, b], f(a) and f(b) of opposite sign
template <class F>
double illinois(F f, double a, double b, double fa, double fb, double tol) {
    int side = 0;
    for (int it = 0; it < 60 && std::fabs(b - a) > tol; ++it) {
        const double x = (a * fb - b * fa) / (fb - fa);
        const double fx = f(x);
        if ((fx > 0) == (fb > 0)) { b = x; fb = fx; if (side == -1) fa *= 0.5; side = -1; }
        else { a = x; fa = fx; if (side == 1) fb *= 0.5; side = 1; }
    }
    return (a * fb - b * fa) / (fb - fa);
}

// root of f (decreasing or increasing - sign change only) on [lo, hi], starting from a small bracket around the last
// root and widening it - the boundary states move little from one step to the next, so most solves take a few
// evaluations instead of narrowing down the whole range
template <class F>
double warmRoot(F f, double lo, double hi, double last, double span, double tol) {
    double a = clampv(last - span, lo, hi), b = clampv(last + span, lo, hi);
    double fa = f(a), fb = f(b);
    for (int k = 0; k < 8 && (fa > 0) == (fb > 0); ++k) {
        if (a <= lo && b >= hi) break;
        span *= 4.0;
        if (std::fabs(fa) < std::fabs(fb) || b >= hi) { a = clampv(last - span, lo, hi); fa = f(a); }
        else { b = clampv(last + span, lo, hi); fb = f(b); }
    }
    if ((fa > 0) == (fb > 0)) return std::fabs(fa) < std::fabs(fb) ? a : b;   // (no sign change: the nearer end)
    return illinois(f, a, b, fa, fb, tol);
}
} // namespace

void Duct::build(const std::vector<double>& x, const std::vector<double>& area, double T0, double p0) {
    x_ = x; af_ = area; p0_ = p0;
    const int n = (int)x_.size() - 1;
    vol_.assign(n, 0); dx_.assign(n, 0); invVol_.assign(n, 0); wall_.assign(n, 0); dlnA_.assign(n, 0); dA_.assign(n, 0);
    m_.assign(n, 0); mom_.assign(n, 0); e_.assign(n, 0); mf_.assign(n, 0); mb_.assign(n, 0);
    dxMin_ = 1e9; smax_ = 1000.0;
    for (int i = 0; i < n; ++i) {
        dx_[i] = x_[i + 1] - x_[i];
        dxMin_ = std::min(dxMin_, dx_[i]);
        // (frustum volume of a cone between the faces)
        const double r0 = std::sqrt(af_[i] / kPi), r1 = std::sqrt(af_[i + 1] / kPi);
        vol_[i] = kPi * dx_[i] / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
        invVol_[i] = 1.0 / vol_[i];
        wall_[i] = 2.0 * std::sqrt(kPi * 0.5 * (af_[i] + af_[i + 1])) * dx_[i];
        dA_[i] = af_[i + 1] - af_[i];
        dlnA_[i] = 2.0 * dA_[i] / (af_[i + 1] + af_[i]);
        m_[i] = p0 / (kR * T0) * vol_[i];
        e_[i] = p0 / gm1 * vol_[i];
        mb_[i] = m_[i];
    }
    wl_.assign(n + 1, Prim{}); wr_.assign(n + 1, Prim{}); W_.assign(n, Prim{}); D_.assign(n, Prim{});
    fm_.assign(n + 1, 0); fmu_.assign(n + 1, 0); fe_.assign(n + 1, 0); ff_.assign(n + 1, 0); fb_.assign(n + 1, 0);
    tWall_ = tMean_ = T0;
    pfM_ = pfE_ = pfF_ = pfB_ = 0; pfMu_ = p0 * af_[0];
    portP_ = p0; portU_ = 0; portT0_ = T0; portRho_ = p0 / (kR * T0);
    mOut_ = 0; ubPort_ = ubOut_ = 0;
}

Duct::Prim Duct::prim(int i) const {
    Prim w;
    const double im = 1.0 / std::max(1e-15, m_[i]);
    w.r = std::max(1e-6, m_[i] * invVol_[i]);
    w.u = mom_[i] * im;
    w.p = std::max(kPMin, gm1 * (e_[i] * invVol_[i] - 0.5 * w.r * w.u * w.u));
    w.yf = clampv(mf_[i] * im, 0.0, 1.0);
    w.yb = clampv(mb_[i] * im, 0.0, 1.0 - w.yf);
    return w;
}

double Duct::cellP(int i) const { return prim(i).p; }
double Duct::cellT(int i) const { const Prim w = prim(i); return w.p / (kR * w.r); }

// HLLC flux per unit area (Toro ch. 10): mass, momentum, energy and the two scalars
void Duct::hllc(const Prim& L, const Prim& R, double& fm, double& fmu, double& fe, double& fyf, double& fyb) {
    const double aL = std::sqrt(g * L.p / L.r), aR = std::sqrt(g * R.p / R.r);
    const double EL = L.p / gm1 + 0.5 * L.r * L.u * L.u, ER = R.p / gm1 + 0.5 * R.r * R.u * R.u;
    const double SL = std::min(L.u - aL, R.u - aR), SR = std::max(L.u + aL, R.u + aR);
    const double den = L.r * (SL - L.u) - R.r * (SR - R.u);
    const double Ss = std::fabs(den) > 1e-12 ? (R.p - L.p + L.r * L.u * (SL - L.u) - R.r * R.u * (SR - R.u)) / den : 0.5 * (L.u + R.u);
    auto flux = [](const Prim& w, double E, double& a, double& b, double& c, double& d, double& e) {
        const double mu = w.r * w.u;
        a = mu; b = mu * w.u + w.p; c = w.u * (E + w.p); d = mu * w.yf; e = mu * w.yb;
    };
    if (SL >= 0) { flux(L, EL, fm, fmu, fe, fyf, fyb); return; }
    if (SR <= 0) { flux(R, ER, fm, fmu, fe, fyf, fyb); return; }
    const bool left = Ss >= 0;
    const Prim& W = left ? L : R;
    const double S = left ? SL : SR, E = left ? EL : ER;
    double a, b, c, d, e;
    flux(W, E, a, b, c, d, e);
    const double k = W.r * (S - W.u) / (S - Ss);
    const double us0 = k, us1 = k * Ss, us2 = k * (E / W.r + (Ss - W.u) * (Ss + W.p / (W.r * (S - W.u))));
    const double U0 = W.r, U1 = W.r * W.u, U2 = E;
    fm = a + S * (us0 - U0);
    fmu = b + S * (us1 - U1);
    fe = c + S * (us2 - U2);
    fyf = fm * W.yf; fyb = fm * W.yb;   // (scalars ride on the mass flux, upwind of the contact)
}

double Duct::solvePort(double cdA, double pCyl, double tCyl) {
    const Prim w = prim(0);
    const double a1 = std::sqrt(g * w.p / w.r), A0 = af_[0];
    // duct side: the characteristic arriving from the pipe, u - 2a/(g-1) = const (isentropic from the first cell)
    auto pChar = [&](double ub) {
        const double ab = std::max(0.02 * a1, a1 + 0.5 * gm1 * (ub - w.u));
        return w.p * std::pow(ab / a1, 2.0 * g / gm1);
    };
    const double p00 = pChar(0.0);
    double ub = 0.0;
    if (cdA <= 0) {   // shut: a closed end
        portP_ = p00; portU_ = 0; portRho_ = w.r * std::pow(std::max(0.02, p00 / w.p), 1.0 / g);
        portT0_ = w.p / (kR * w.r);
        pfM_ = pfE_ = pfF_ = pfB_ = 0; pfMu_ = p00 * A0;
        return 0.0;
    }
    double mdot = 0.0;
    if (pCyl > p00) {
        // out of the cylinder: orifice flow from the cylinder's state; the gas keeps its stagnation enthalpy
        const double uTop = std::min(std::sqrt(2.0 * kCp * tCyl) * 0.98,
                                     w.u + 2.0 * a1 / gm1 * (std::pow(pCyl / w.p, gm1 / (2.0 * g)) - 1.0));
        auto f = [&](double u) {
            const double pb = pChar(u), tb = std::max(50.0, tCyl - u * u / (2.0 * kCp));
            return pb / (kR * tb) * u * A0 - orificeFlow(cdA, pCyl, tCyl, pb);
        };
        ub = uTop > 0 ? warmRoot(f, 0.0, uTop, clampv(ubPort_, 0.0, uTop), 0.03 * a1 + 1.0, 1e-3) : 0.0;
        const double pb = pChar(ub), tb = std::max(50.0, tCyl - ub * ub / (2.0 * kCp));
        portRho_ = pb / (kR * tb);
        mdot = portRho_ * ub * A0;
        portP_ = pb; portU_ = ub; portT0_ = tCyl;
    } else if (pCyl < p00) {
        // back into the cylinder: the duct's gas (its entropy) through the orifice
        const double t1 = w.p / (kR * w.r);
        const double abMin = a1 * std::pow(std::max(0.02, pCyl / w.p), gm1 / (2.0 * g));
        const double uLow = w.u + 2.0 / gm1 * (abMin - a1);
        auto state = [&](double u, double& pb, double& rb, double& tb) {
            const double ab = std::max(0.02 * a1, a1 + 0.5 * gm1 * (u - w.u));
            pb = w.p * std::pow(ab / a1, 2.0 * g / gm1); tb = t1 * (ab / a1) * (ab / a1); rb = pb / (kR * tb);
        };
        auto f = [&](double u) {
            double pb, rb, tb; state(u, pb, rb, tb);
            return -rb * u * A0 - orificeFlow(cdA, pb, tb, pCyl);
        };
        const double lo = std::min(0.0, uLow);
        ub = lo < 0 ? warmRoot(f, lo, 0.0, clampv(ubPort_, lo, 0.0), 0.03 * a1 + 1.0, 1e-3) : 0.0;
        double pb, rb, tb; state(ub, pb, rb, tb);
        portRho_ = rb; mdot = rb * ub * A0;
        portP_ = pb; portU_ = ub; portT0_ = tb + ub * ub / (2.0 * kCp);
    } else {
        portP_ = p00; portU_ = 0; portT0_ = tCyl; portRho_ = w.r;
    }
    ubPort_ = ub;
    // provisional flux (setPortGas fixes the gas that actually crosses)
    pfM_ = mdot; pfMu_ = mdot * portU_ + portP_ * A0; pfE_ = mdot * kCp * portT0_;
    pfF_ = mdot > 0 ? 0.0 : mdot * w.yf; pfB_ = mdot > 0 ? mdot : mdot * w.yb;
    return mdot;
}

void Duct::setPortGas(double mdot, double ff, double bf, double t0) {
    pfM_ = mdot;
    pfMu_ = mdot * portU_ + portP_ * af_[0];
    pfE_ = mdot * kCp * t0;
    pfF_ = mdot * ff; pfB_ = mdot * bf;
}

void Duct::backGas(double& ff, double& bf, double& t0) const {
    const Prim w = prim(0);
    ff = w.yf; bf = w.yb; t0 = portT0_;
}

void Duct::step(double dt, double ambientK) {
    const int n = (int)m_.size();
    // sub-steps: Courant number <= 0.8 (wave speeds of the last step: they change little in one)
    const int sub = std::max(1, (int)std::ceil(1.15 * smax_ * dt / (0.8 * dxMin_)));
    const double h = dt / sub;
    double mOutSum = 0, fOutSum = 0, bOutSum = 0;
    std::vector<Prim>& W = W_;
    std::vector<Prim>& D = D_;
    double smax = 1.0, tm = 0;
    for (int s = 0; s < sub; ++s) {
        for (int i = 0; i < n; ++i) W[i] = prim(i);
        if (s == 0)
            for (int i = 0; i < n; ++i) {
                smax = std::max(smax, std::fabs(W[i].u) + std::sqrt(g * W[i].p / W[i].r));
                tm += W[i].p / W[i].r * vol_[i];   // (mass x temperature x R)
            }
        // limited slopes (primitive variables), none in the end cells
        for (int i = 0; i < n; ++i) {
            if (i == 0 || i == n - 1) { D[i] = Prim{0, 0, 0, 0, 0}; continue; }
            D[i].r = minmod(W[i].r - W[i - 1].r, W[i + 1].r - W[i].r);
            D[i].u = minmod(W[i].u - W[i - 1].u, W[i + 1].u - W[i].u);
            D[i].p = minmod(W[i].p - W[i - 1].p, W[i + 1].p - W[i].p);
            D[i].yf = minmod(W[i].yf - W[i - 1].yf, W[i + 1].yf - W[i].yf);
            D[i].yb = minmod(W[i].yb - W[i - 1].yb, W[i + 1].yb - W[i].yb);
        }
        // Hancock half step (primitive form, with the area change: d(rho)/dt gets -rho u dlnA/dx, dp/dt -g p u dlnA/dx)
        for (int i = 0; i < n; ++i) {
            const Prim& w = W[i]; const Prim& d = D[i];
            const double k = 0.5 * h / dx_[i];
            const double dlnA = dlnA_[i];
            Prim c;
            c.r = w.r - k * (w.u * d.r + w.r * d.u + w.r * w.u * dlnA);
            c.u = w.u - k * (w.u * d.u + d.p / w.r);
            c.p = w.p - k * (g * w.p * d.u + w.u * d.p + g * w.p * w.u * dlnA);
            c.yf = w.yf - k * w.u * d.yf;
            c.yb = w.yb - k * w.u * d.yb;
            Prim L{c.r - 0.5 * d.r, c.u - 0.5 * d.u, c.p - 0.5 * d.p, c.yf - 0.5 * d.yf, c.yb - 0.5 * d.yb};
            Prim R{c.r + 0.5 * d.r, c.u + 0.5 * d.u, c.p + 0.5 * d.p, c.yf + 0.5 * d.yf, c.yb + 0.5 * d.yb};
            if (L.r <= 0 || R.r <= 0 || L.p <= kPMin || R.p <= kPMin) { L = w; R = w; }   // (positivity: first order)
            wr_[i] = L;        // the state on the right of face i
            wl_[i + 1] = R;    // ... on the left of face i + 1
        }
        for (int f = 1; f < n; ++f) hllc(wl_[f], wr_[f], fm_[f], fmu_[f], fe_[f], ff_[f], fb_[f]);

        // far end: the outgoing characteristic u + 2a/(g-1) meets the waveguide's p = p0 + 2 in + Z u
        double oM, oMu, oE, oF, oB;
        {
            const Prim w = W[n - 1];
            const double aN = std::sqrt(g * w.p / w.r), A = af_[n];
            auto st = [&](double u, double& pb, double& rb, double& tb) {
                const double ab = std::max(0.02 * aN, aN - 0.5 * gm1 * (u - w.u));
                pb = w.p * std::pow(ab / aN, 2.0 * g / gm1);
                if (u >= 0) { tb = w.p / (kR * w.r) * (ab / aN) * (ab / aN); rb = pb / (kR * tb); }
                else { tb = tW_; rb = pb / (kR * tW_); }
            };
            auto f = [&](double u) {
                double pb, rb, tb; st(u, pb, rb, tb);
                return pb - (p0_ + 2.0 * inW_ + zW_ * rb * u * A / rhoW_);
            };
            const double lo = -1.5 * aN, hi = w.u + 2.0 * aN / gm1 * 0.95;
            const double ub = warmRoot(f, lo, hi, clampv(ubOut_, lo, hi), 0.03 * aN + 1.0, 1e-3);
            ubOut_ = ub;
            double pb, rb, tb; st(ub, pb, rb, tb);
            const double md = rb * ub * A;
            oM = md; oMu = md * ub + pb * A; oE = md * (kCp * tb + 0.5 * ub * ub);
            oF = md * (ub >= 0 ? w.yf : ffW_); oB = md * (ub >= 0 ? w.yb : bfW_);
        }
        mOutSum += oM; fOutSum += oF; bOutSum += oB;

        // update: fluxes through the faces, wall pressure on the changing area, wall friction and heat
        for (int i = 0; i < n; ++i) {
            const double inM = i == 0 ? pfM_ : fm_[i] * af_[i], inMu = i == 0 ? pfMu_ : fmu_[i] * af_[i];
            const double inE = i == 0 ? pfE_ : fe_[i] * af_[i], inF = i == 0 ? pfF_ : ff_[i] * af_[i];
            const double inB = i == 0 ? pfB_ : fb_[i] * af_[i];
            const bool last = i == n - 1;
            const double outM = last ? oM : fm_[i + 1] * af_[i + 1], outMu = last ? oMu : fmu_[i + 1] * af_[i + 1];
            const double outE = last ? oE : fe_[i + 1] * af_[i + 1], outF = last ? oF : ff_[i + 1] * af_[i + 1];
            const double outB = last ? oB : fb_[i + 1] * af_[i + 1];
            const Prim& w = W[i];
            const double perim = wall_[i];   // wall area
            const double tau = 0.5 * kCf * w.r * w.u * std::fabs(w.u);
            const double T = w.p / (kR * w.r);
            const double hc = std::max(kHMin, 0.5 * kCf * w.r * std::fabs(w.u) * kCp);
            m_[i] += h * (inM - outM);
            mom_[i] += h * (inMu - outMu + w.p * dA_[i] - tau * perim);
            e_[i] += h * (inE - outE + hc * perim * (tWall_ - T));
            mf_[i] += h * (inF - outF);
            mb_[i] += h * (inB - outB);
            m_[i] = std::max(1e-9 * vol_[i], m_[i]);
            mf_[i] = clampv(mf_[i], 0.0, m_[i]); mb_[i] = clampv(mb_[i], 0.0, m_[i] - mf_[i]);
            const double ek = 0.5 * mom_[i] * mom_[i] / m_[i];
            if (e_[i] - ek < kPMin / gm1 * vol_[i]) e_[i] = ek + kPMin / gm1 * vol_[i];
        }
    }
    mOut_ = mOutSum / sub;
    ffOut_ = mOutSum != 0 ? clampv(fOutSum / mOutSum, 0.0, 1.0) : 0.0;
    bfOut_ = mOutSum != 0 ? clampv(bOutSum / mOutSum, 0.0, 1.0) : 1.0;
    smax_ = smax;
    // wall temperature: follows the mean gas temperature slowly
    double mm = 0;
    for (int i = 0; i < n; ++i) mm += m_[i];
    tMean_ += (tm / (kR * std::max(1e-12, mm)) - tMean_) * std::min(1.0, dt / 20.0);
    tWall_ = ambientK + 0.55 * (tMean_ - ambientK);
}

} // namespace es

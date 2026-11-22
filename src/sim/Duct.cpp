#include "sim/Duct.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
constexpr double g = kGamma;
constexpr double gm1 = kGamma - 1.0;
constexpr double kHMin = 10.0;    // W/m2K: heat transfer floor of still gas, chosen
constexpr double kPr = 0.7;       // Prandtl number of air / exhaust gas
// wall friction and heat as Blair computes them (Design and Simulation of Two-Stroke Engines, secs. 2.3.1 and 2.4):
// Fanning friction factor from Blasius, Cf = 0.0791 Re^-0.25 (turbulent, Re >= 4000; 0.01 below - eq. 2.3.13), and
// the Reynolds analogy Nu = Cf Re / 2 (eq. 2.4.1), i.e. h = Cf rho |u| cp / (2 Pr) with Re = rho |u| d / mu (mu of air,
// Sutherland). (Before: Cf 0.005 fixed and h = Cf rho |u| cp / 2 - chosen; the tuned pipe's gas came out 80-160 K
// hotter than Blair's measured mid-pipe temperatures, Fig. 5.30.)
inline double blasiusCf(double rho, double u, double d, double T) {
    const double mu = 1.458e-6 * T * std::sqrt(T) / (T + 110.4);
    const double re = rho * std::fabs(u) * d / mu;
    return re < 4000.0 ? 0.01 : 0.0791 / std::sqrt(std::sqrt(re));
}
constexpr double kPMin = 2000.0;  // Pa: positivity floor

// monotonised central (MC) slope limiter (van Leer 1977): sharper than minmod - on the Blair 125 check it gives with
// 30 mm cells what minmod needs 20 mm for (charging efficiency 0.59 vs 0.60 converged)
inline double limitedSlope(double a, double b) {
    if (a * b <= 0) return 0.0;
    const double c = 0.5 * (a + b), m = std::min(std::fabs(c), 2.0 * std::min(std::fabs(a), std::fabs(b)));
    return c > 0 ? m : -m;
}

// orifice flow as Gas.h orificeFlow, with one pow instead of two: pr^(2/g) - pr^((g+1)/g) = x^2 - pr x, x = pr^(1/g)
// (the port solves call it ~a dozen times a step per duct; same values to rounding)
inline double ductOrifice(double cdA, double pUp, double tUp, double pDn) {
    if (cdA <= 0 || pUp <= pDn) return 0;
    static const double prCrit = std::pow(2.0 / (g + 1.0), g / gm1);
    static const double psiChoked = std::sqrt(g) * std::pow(2.0 / (g + 1.0), (g + 1.0) / (2.0 * gm1));
    static const double psi0 = [] { const double x = std::pow(0.995, 1.0 / g); return std::sqrt(2.0 * g / gm1 * (x * x - 0.995 * x)); }();
    const double pr = pDn / pUp;
    double psi;
    if (pr <= prCrit) psi = psiChoked;
    else if (pr < 0.995) { const double x = std::pow(pr, 1.0 / g); psi = std::sqrt(2.0 * g / gm1 * (x * x - pr * x)); }
    else psi = psi0 * (1.0 - pr) / 0.005;
    return cdA * pUp / std::sqrt(kR * tUp) * psi;
}

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

void Duct::build(const std::vector<double>& x, const std::vector<double>& area, double T0, double p0, int stepsPer) {
    x_ = x; af_ = area; p0_ = p0;
    div_ = std::max(1, stepsPer); cnt_ = 0; accT_ = accM_ = accMu_ = accE_ = accF_ = accB_ = 0;
    const int n = (int)x_.size() - 1;
    vol_.assign(n, 0); dx_.assign(n, 0); dia_.assign(n, 0); cf_.assign(n, 0.005); cfAge_ = 0; invVol_.assign(n, 0); wall_.assign(n, 0); dlnA_.assign(n, 0); dA_.assign(n, 0);
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
        dia_[i] = 2.0 * std::sqrt(0.5 * (af_[i] + af_[i + 1]) / kPi);
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
    const double rsu = W.r * (S - W.u), k = rsu / (S - Ss);
    const double us0 = k, us1 = k * Ss, us2 = k * E / W.r + (Ss - W.u) * (k * Ss + W.p * k / rsu);
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
            return pb / (kR * tb) * u * A0 - ductOrifice(cdA, pCyl, tCyl, pb);
        };
        ub = uTop > 0 ? warmRoot(f, 0.0, uTop, clampv(ubPort_, 0.0, uTop), 0.03 * a1 + 1.0, 0.05) : 0.0;
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
            return -rb * u * A0 - ductOrifice(cdA, pb, tb, pCyl);
        };
        const double lo = std::min(0.0, uLow);
        ub = lo < 0 ? warmRoot(f, lo, 0.0, clampv(ubPort_, lo, 0.0), 0.03 * a1 + 1.0, 0.05) : 0.0;
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

// Far end, every call (not only when the duct steps): the outgoing characteristic u + 2a/(g-1) of the last cell meets
// the waveguide's p = p0 + 2 in + Z u. (Solved only when the duct stepped, the flow handed to the waveguide was held
// for two of its steps - a delay in the loop that pumped energy in: at rest the pipe rang by itself at ~500 Hz.)
void Duct::solveOutlet(double dtCaller) {
    const int n = (int)m_.size();
    const Prim w = prim(n - 1);
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
    const double ub = warmRoot(f, lo, hi, clampv(ubOut_, lo, hi), 0.03 * aN + 1.0, 0.05);
    ubOut_ = ub;
    double pb, rb, tb; st(ub, pb, rb, tb);
    const double md = rb * ub * A;
    const double ff = ub >= 0 ? w.yf : ffW_, bf = ub >= 0 ? w.yb : bfW_;
    mOut_ = md; ffOut_ = clampv(ff, 0.0, 1.0); bfOut_ = clampv(bf, 0.0, 1.0);
    oAccM_ += md * dtCaller; oAccMu_ += (md * ub + pb * A) * dtCaller; oAccE_ += md * (kCp * tb + 0.5 * ub * ub) * dtCaller;
    oAccF_ += md * ff * dtCaller; oAccB_ += md * bf * dtCaller;
}

void Duct::step(double dtCaller, double ambientK) {
    // the duct steps once every div_ calls (~48 kHz): the port and outlet flows of the calls in between are averaged
    // (mass, momentum and energy conserved). A fixed rhythm: letting it wait as long as the Courant limit allowed
    // (up to 4 calls with the slow waves at rest) lagged the boundaries enough for the pipe to ring by itself at rest
    accT_ += dtCaller; accM_ += pfM_ * dtCaller; accMu_ += pfMu_ * dtCaller; accE_ += pfE_ * dtCaller;
    accF_ += pfF_ * dtCaller; accB_ += pfB_ * dtCaller;
    if (++cnt_ < div_) return;
    const double dt = accT_;
    pfM_ = accM_ / dt; pfMu_ = accMu_ / dt; pfE_ = accE_ / dt; pfF_ = accF_ / dt; pfB_ = accB_ / dt;
    cnt_ = 0; accT_ = accM_ = accMu_ = accE_ = accF_ = accB_ = 0;
    const int n = (int)m_.size();
    // sub-steps: Courant number <= 0.8 (wave speeds of the last step: they change little in one)
    const int sub = std::max(1, (int)std::ceil(1.15 * smax_ * dt / (0.8 * dxMin_)));
    const double h = dt / sub;
    std::vector<Prim>& W = W_;
    std::vector<Prim>& D = D_;
    double smax = 1.0, tm = 0, mm = 0;
    for (int s = 0; s < sub; ++s) {
        for (int i = 0; i < n; ++i) W[i] = prim(i);
        if (s == 0)
            for (int i = 0; i < n; ++i) {
                const double pr = W[i].p / W[i].r;
                smax = std::max(smax, std::fabs(W[i].u) + std::sqrt(g * pr));
                const double mi = W[i].r * vol_[i];
                tm += pr * mi; mm += mi;   // (mass x temperature x R, and the mass: a mass-weighted mean)
            }
        // limited slopes (primitive variables, none in the end cells) and the Hancock half step (primitive form, with
        // the area change: d(rho)/dt gets -rho u dlnA/dx, dp/dt -g p u dlnA/dx), in one pass
        for (int i = 0; i < n; ++i) {
            const Prim& w = W[i];
            Prim& d = D[i];
            if (i == 0 || i == n - 1) d = Prim{0, 0, 0, 0, 0};
            else {
                const Prim& wm = W[i - 1]; const Prim& wp = W[i + 1];
                d.r = limitedSlope(w.r - wm.r, wp.r - w.r);
                d.u = limitedSlope(w.u - wm.u, wp.u - w.u);
                d.p = limitedSlope(w.p - wm.p, wp.p - w.p);
                d.yf = limitedSlope(w.yf - wm.yf, wp.yf - w.yf);
                d.yb = limitedSlope(w.yb - wm.yb, wp.yb - w.yb);
            }
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

        // far end: the outlet fluxes solved on every call (solveOutlet), averaged over this step
        const double oM = oAccM_ / dt, oMu = oAccMu_ / dt, oE = oAccE_ / dt, oF = oAccF_ / dt, oB = oAccB_ / dt;

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
            const double T = w.p / (kR * w.r);
            // (Cf moves with Re^-0.25 only: refreshed every 16 duct steps, ~3 kHz - it cost ~a quarter of a cell update)
            if (cfAge_ == 0 && s == 0) cf_[i] = blasiusCf(w.r, w.u, dia_[i], T);
            const double cf = cf_[i];
            const double tau = 0.5 * cf * w.r * w.u * std::fabs(w.u);
            const double hc = std::max(kHMin, 0.5 * cf * w.r * std::fabs(w.u) * kCp / kPr);
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
    oAccM_ = oAccMu_ = oAccE_ = oAccF_ = oAccB_ = 0;
    if (++cfAge_ >= 16) cfAge_ = 0;
    smax_ = smax;
    // wall temperature: follows the mass-weighted mean gas temperature slowly. (It was sum(R T V) / sum(m) - T / rho,
    // not T: in hot exhaust (rho ~0.35) the "wall" ran hotter than the gas, heated it, the gas thinned and the wall
    // got hotter still - after a minute or so of light load the tail of a tuned pipe ran away to 1e4 K and NaN.)
    tMean_ += (tm / (kR * std::max(1e-12, mm)) - tMean_) * std::min(1.0, dt / 20.0);
    tWall_ = ambientK + 0.55 * (tMean_ - ambientK);
    // safety net: a duct state gone non-finite (it would poison the waveguide and every audio filter after it, for
    // good) starts again at rest - ambient pressure, the wall temperature
    double chk = tMean_;
    for (int i = 0; i < n; ++i) chk += e_[i] + mom_[i];
    if (!std::isfinite(chk)) {
        if (!std::isfinite(tMean_)) tMean_ = ambientK;
        tWall_ = ambientK + 0.55 * (tMean_ - ambientK);
        for (int i = 0; i < n; ++i) {
            m_[i] = p0_ / (kR * tWall_) * vol_[i]; mom_[i] = 0; e_[i] = p0_ / gm1 * vol_[i];
            mf_[i] = 0; mb_[i] = m_[i];
        }
        smax_ = 1000.0; ubPort_ = ubOut_ = 0;
    }
}

} // namespace es

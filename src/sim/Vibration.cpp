#include "sim/Vibration.h"
#include "sim/Components.h"
#include <algorithm>
#include <cmath>

namespace es {

namespace {
constexpr double kRefMassKg = 400.0;   // sprung mass the mounts shake (chosen: the front of a car)
constexpr double kSplitHz = 35.0;      // between the two rumble motors (chosen)
constexpr double kEnvS = 0.025;        // RMS envelope (chosen: lets a single's ~8-10 Hz idle thump through)
constexpr double kRollLever = 0.25;    // m, crank axis to the mounts (chosen)
}

void EngineVibration::setup(const EngineParams& p, const ComponentModel& comp) {
    piston_ = !isRotary(p);
    n_ = clampv((int)p.cylinders, 1, kMaxCyl);
    cycle_ = cycleDeg(p);
    mRec_ = comp.recipMass;
    r_ = comp.crankRadius;
    lam_ = comp.crankRadius / std::max(1e-3, comp.rodLen);
    area_ = comp.pistonArea;
    // engine mass (chosen: ~40 kg + 45 kg per litre, a diesel's block and crank 30 % heavier) and its roll inertia
    // (radius of gyration ~0.18 m - chosen)
    const double L = std::max(0.05, ratedDisplacementL(p));
    mass_ = (40.0 + 45.0 * L) * (isDiesel(p) ? 1.3 : 1.0);
    rollI_ = mass_ * 0.18 * 0.18;
    // cylinder axes and positions along the crank, as the engine view lays them out (EngineModel.cpp): banks side
    // by side, a V's / boxer's second bank half a bore further back
    const double B = std::max(30.0, (double)p.bore_mm) * 1e-3;
    const double pitch = 1.32 * B, bankOff = p.layout == Layout::Inline ? 0.0 : 0.46 * B;
    int col[2] = {0, 0}, idx[kMaxCyl] = {};
    for (int i = 0; i < n_; ++i) { const int b = p.layout == Layout::Inline ? 0 : clampv((int)p.bank[i], 0, 1); idx[i] = col[b]++; }
    const int cols = std::max(1, std::max(col[0], col[1]));
    const double z0 = -(cols - 1) * pitch * 0.5;
    for (int i = 0; i < n_; ++i) {
        const int b = p.layout == Layout::Inline ? 0 : clampv((int)p.bank[i], 0, 1);
        double ax = 0;
        if (p.layout == Layout::V) ax = (b == 0 ? -0.5 : 0.5) * p.bankAngle * kPi / 180.0;
        if (p.layout == Layout::Boxer) ax = (b == 0 ? -0.5 : 0.5) * kPi;
        ux_[i] = std::sin(ax); uy_[i] = std::cos(ax);
        z_[i] = z0 + idx[i] * pitch + (b == 0 ? -0.5 : 0.5) * bankOff;
    }
    halfLen_ = 0.5 * cols * pitch + 0.1;
    cwReady_ = false;   // (the counterweight sums need the phases: on the next step)
    bin_ = -1;
    // mount modes (chosen, inside the 6-15 Hz of typical rubber mounts): lateral, vertical, roll, pitch, yaw
    const double hz[5] = {9.0, 10.0, 12.0, 11.0, 10.0};
    for (int k = 0; k < 5; ++k) { m_[k].wn = 2.0 * kPi * hz[k]; m_[k].z = 0.15; }
}

// one mount direction, per unit of inertia: q'' = in - wn^2 q - 2 z wn q'; what it passes on is wn^2 q + 2 z wn q'
double EngineVibration::transmit(Mode& m, double in, double dt) {
    m.v += (in - m.wn * m.wn * m.x - 2.0 * m.z * m.wn * m.v) * dt;
    m.x += m.v * dt;
    return m.wn * m.wn * m.x + 2.0 * m.z * m.wn * m.v;
}

void EngineVibration::step(double dt, double theta, double omega, const double* phases, int n, double gasTorque) {
    double fx = 0, fy = 0, mPitch = 0, mYaw = 0, tIn = 0;
    if (piston_) {
        const double w2 = omega * omega;
        n = std::min(n, n_);
        if (!cwReady_) {
            // primary sums: cos a u = (u e^{ja} + u e^{-ja}) / 2, a = theta - phase: the coefficients of e^{+j theta}
            // (S+ = sum m u e^{-j phase}) and of e^{-j theta} (S- = sum m u e^{+j phase}), for the force and the couple
            // (z u). The counterweights cancel the one turning with the crank: the coherent, larger one (taken as the
            // one leaving the smaller residual - an inline engine's two are equal)
            double s[2][4] = {};
            for (int i = 0; i < n; ++i) {
                const double ph = phases[i] * kPi / 180.0;
                for (int d = 0; d < 2; ++d) {
                    const double c = std::cos(ph), sn = (d == 0 ? -1.0 : 1.0) * std::sin(ph);   // e^{-+j phase}
                    const double rx = ux_[i] * c - uy_[i] * sn, ry = ux_[i] * sn + uy_[i] * c;
                    s[d][0] += mRec_ * rx; s[d][1] += mRec_ * ry;
                    s[d][2] += mRec_ * z_[i] * rx; s[d][3] += mRec_ * z_[i] * ry;
                }
            }
            auto mag = [&](int d) { return s[d][0] * s[d][0] + s[d][1] * s[d][1] + (s[d][2] * s[d][2] + s[d][3] * s[d][3]) / (halfLen_ * halfLen_); };
            const int d = mag(0) >= mag(1) ? 0 : 1;
            cwFx_ = s[d][0]; cwFy_ = s[d][1]; cwKx_ = s[d][2]; cwKy_ = s[d][3]; cwSign_ = d == 0 ? 1.0 : -1.0;
            cwReady_ = true;
        }
        {   // what the counterweights put back: - (1/2) r w^2 e^{s j theta} (F, K)
            const double t = cwSign_ * theta * kPi / 180.0, c = std::cos(t), sn = std::sin(t), k = 0.5 * r_ * w2;
            fx -= k * (cwFx_ * c - cwFy_ * sn); fy -= k * (cwFx_ * sn + cwFy_ * c);
            mYaw -= k * (cwKx_ * c - cwKy_ * sn); mPitch -= k * (cwKx_ * sn + cwKy_ * c);
        }
        for (int i = 0; i < n; ++i) {
            double a = std::fmod(theta - phases[i], 360.0);
            if (a < 0) a += 360.0;
            a *= kPi / 180.0;
            const double ca = std::cos(a), sa = std::sin(a);
            const double xdd = r_ * w2 * (ca + lam_ * (2.0 * ca * ca - 1.0));     // piston acceleration (towards the crank +)
            const double dxda = r_ * (sa + lam_ * sa * ca);                        // dx / da  (sin a + lambda/2 sin 2a)
            const double f = mRec_ * xdd;                                          // on the block, along its axis outwards
            fx += f * ux_[i]; fy += f * uy_[i];
            mPitch += f * uy_[i] * z_[i]; mYaw += f * ux_[i] * z_[i];
            tIn -= mRec_ * xdd * dxda;
        }
    }
    // the roll: the crank torque without its mean (the block takes it back)
    const double tq = gasTorque + tIn;
    tqMean_ += (tq - tqMean_) * std::min(1.0, dt / 0.5);
    const double roll = -(tq - tqMean_);
    const double pitchI = mass_ * (halfLen_ * halfLen_ / 3.0 + 0.03);
    // transmitted force at the mounts (N) of each direction
    const double ft[5] = {
        mass_ * transmit(m_[0], fx / mass_, dt),
        mass_ * transmit(m_[1], fy / mass_, dt),
        rollI_ * transmit(m_[2], roll / rollI_, dt) / kRollLever,
        pitchI * transmit(m_[3], mPitch / pitchI, dt) / halfLen_,
        pitchI * transmit(m_[4], mYaw / pitchI, dt) / halfLen_,
    };
    const double aSplit = 1.0 - std::exp(-2.0 * kPi * kSplitHz * dt);
    const double aEnv = std::min(1.0, dt / kEnvS), aMean = std::min(1.0, dt / 0.3);
    double lo = 0, hi = 0;
    for (int k = 0; k < 5; ++k) {
        Mode& m = m_[k];
        const double f = std::isfinite(ft[k]) ? ft[k] / kRefMassKg : 0.0;
        m.lp += (f - m.lp) * aSplit;
        m.hp += (m.lp - m.hp) * aMean;            // (the slow mean: not felt as vibration)
        const double l = m.lp - m.hp, h = f - m.lp;
        m.envLo += (l * l - m.envLo) * aEnv;
        m.envHi += (h * h - m.envHi) * aEnv;
        if (!std::isfinite(m.envLo) || !std::isfinite(m.envHi) || !std::isfinite(m.x) || !std::isfinite(m.v)) m = Mode{m.wn, m.z};
        lo += m.envLo; hi += m.envHi;
    }
    low_ = std::sqrt(lo); high_ = std::sqrt(hi);
    // the displacements into the cycle trace (every degree passed since the last step)
    {
        const int nb = std::min(kBins, (int)cycle_);
        double th = std::fmod(theta, cycle_);
        if (th < 0) th += cycle_;
        const int b = std::min(nb - 1, (int)th);
        const float x = (float)m_[0].x, y = (float)m_[1].x, rl = (float)m_[2].x;
        // (still in the same degree, or the first step: just this bin; else every bin from the last one up to here -
        //  a jump of more than half a cycle is a reset or a wrap the other way: this bin only)
        const int ahead = bin_ < 0 || bin_ >= nb ? 0 : (b - bin_ + nb) % nb;
        if (ahead == 0 || ahead > nb / 2) { trX[b] = x; trY[b] = y; trRoll[b] = rl; }
        else for (int s = 1, k = bin_; s <= ahead; ++s) { k = (k + 1) % nb; trX[k] = x; trY[k] = y; trRoll[k] = rl; }
        bin_ = b;
    }
}

} // namespace es

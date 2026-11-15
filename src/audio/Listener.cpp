#include "audio/Listener.h"

namespace es {

void Listener::init(double fs) {
    fs_ = fs;
    tailInLp_.lowpass(1600, 0.7, fs);
    bayExLpL_.lowpass(1400, 0.7, fs);
    bayExLpR_.lowpass(1400, 0.7, fs);
    // cabin: body panels and glass act as a 4th-order Butterworth low-pass (~800 Hz), the closed
    // cabin volume booms at its first modes, and the firewall / floor pass the low mids
    cabLp1_.lowpass(800, 0.5412, fs);
    cabLp2_.lowpass(800, 1.3066, fs);
    cabShelf_.lowshelf(120, 3.0, fs);
    cabBoom_.peaking(85, 3.2, 9.0, fs);          // first longitudinal cabin mode
    cabBoom2_.peaking(85 * 2.07, 4.0, 4.0, fs);   // next cabin mode
    cabMid_.peaking(260, 1.4, 3.0, fs);
    dcL_.highpass(18, 0.7, fs);
    dcR_.highpass(18, 0.7, fs);
    tailMechLp_.lowpass(2500, fs);
    tailBoostLp_.lowpass(7000, fs);
    cabBoostLp_.lowpass(4500, fs);
    erL_.assign(4096, 0.f);
    erR_.assign(4096, 0.f);
    erW_ = 0;
}

void Listener::setRoom(Environment env, float mix) {
    if (env == room_ && mix == lastMix_ && !fdnBuf_[0].empty()) return;
    lastMix_ = mix;
    if (env == room_ && !fdnBuf_[0].empty()) {   // only the mix changed: no buffer reset (no click)
        wetTarget_ = clampv((double)mix, 0.0, 1.0) * (env == Environment::OpenRoad ? 0.35 : 1.0);
        return;
    }
    // delay lengths (ms, mutually prime-ish) and reverb time per space
    double len[kFdn], rt60 = 0.3, damp = 6000;
    double base;
    switch (env) {
    case Environment::Garage:   base = 9.0;  rt60 = 0.9;  damp = 5000; break;  // hard concrete box
    case Environment::DynoCell: base = 6.0;  rt60 = 0.55; damp = 4000; break;  // treated, smaller
    case Environment::Tunnel:   base = 31.0; rt60 = 2.6;  damp = 3500; break;  // long, slapping
    case Environment::OpenRoad: default: base = 7.0; rt60 = 0.18; damp = 3000; break; // ground + far walls
    }
    const double ratios[kFdn] = {1.0, 1.37, 1.79, 2.31};
    for (int i = 0; i < kFdn; ++i) {
        len[i] = base * ratios[i];
        fdnLen_[i] = std::max(8, (int)(len[i] * 1e-3 * fs_));
        if ((int)fdnBuf_[i].size() < fdnLen_[i] + 1) fdnBuf_[i].assign(fdnLen_[i] + 1, 0.f);
        fdnPos_[i] = 0;
    }
    // per-pass gain for the mean loop length to reach -60 dB after rt60
    double meanLen = base * 1.62e-3;
    fdnGain_ = std::pow(10.0, -3.0 * meanLen / rt60);
    fdnDampA_ = onePoleCoef(damp, fs_);
    room_ = env;
    wetTarget_ = clampv((double)mix, 0.0, 1.0) * (env == Environment::OpenRoad ? 0.35 : 1.0);
    wetHp_.highpass(90, 0.7, fs_);
}

void Listener::process(double exL, double exR, double in, double mech, double boost, float& outL, float& outR) {
    // smooth crossfade between positions
    double tgt[3] = {0, 0, 0};
    tgt[(int)pos_] = 1;
    double a = 1.0 / (0.15 * fs_);
    for (int i = 0; i < 3; ++i) w_[i] += (tgt[i] - w_[i]) * a * 3.0;

    double L = 0, R = 0;
    // 1) behind the car, ~1 m from the tailpipe
    if (w_[0] > 1e-4) {
        double i2 = tailInLp_.tick(in) * 0.12;
        double m2 = tailMechLp_.lp(mech) * 0.05;
        double b2 = tailBoostLp_.lp(boost) * 0.45;      // turbo whistle / BOV heard from behind
        L += w_[0] * (exL + i2 + m2 + b2);
        R += w_[0] * (exR + i2 + m2 + b2);
    } else { tailInLp_.tick(in); tailMechLp_.lp(mech); tailBoostLp_.lp(boost); }
    // 2) engine bay, hood open: induction + mechanical dominate, exhaust distant and muffled
    {
        double eL = bayExLpL_.tick(exL) * 0.22, eR = bayExLpR_.tick(exR) * 0.22;
        if (w_[1] > 1e-4) {
            double c = in * 0.9 + mech * 0.7 + boost * 1.0;
            L += w_[1] * (eL + c * 1.05);
            R += w_[1] * (eR + c * 0.95);
        }
    }
    // 3) cabin: everything through the body panels -> lowpassed, boomy
    {
        double mono = (exL + exR) * 0.5 * 0.45 + in * 0.18 + mech * 0.12;
        double y = cabLp1_.tick(mono);
        y = cabLp2_.tick(y);
        y = cabShelf_.tick(y);
        y = cabBoom_.tick(y);
        y = cabBoom2_.tick(y);
        y = cabMid_.tick(y);
        double cb = cabBoostLp_.lp(boost) * 0.35;       // the whistle carries into the cabin
        if (w_[2] > 1e-4) { L += w_[2] * (y + cb); R += w_[2] * (y + cb); }
    }

    // early reflections (ground + nearby walls)
    const int mask = 4095;
    erL_[erW_ & mask] = (float)L;
    erR_[erW_ & mask] = (float)R;
    auto tap = [&](std::vector<float>& b, double ms) { int d = (int)(ms * 1e-3 * fs_); return (double)b[(erW_ - d) & mask]; };
    double rl = -0.30 * tap(erL_, 4.1) + 0.16 * tap(erR_, 13.0) + 0.09 * tap(erL_, 23.0) + 0.05 * tap(erR_, 37.0);
    double rr = -0.30 * tap(erR_, 5.3) + 0.15 * tap(erL_, 15.0) + 0.08 * tap(erR_, 27.0) + 0.05 * tap(erL_, 41.0);
    ++erW_;
    L += rl; R += rr;

    // late reverb of the space (4-line feedback delay network, Householder mixing)
    if (!fdnBuf_[0].empty()) {
        wet_ += (wetTarget_ - wet_) * 0.0005;
        double o[kFdn];
        for (int i = 0; i < kFdn; ++i) {
            int rp = fdnPos_[i];
            double v = fdnBuf_[i][rp];
            fdnDamp_[i] += fdnDampA_ * (v - fdnDamp_[i]);   // air / wall absorption of highs
            o[i] = fdnDamp_[i];
        }
        double sum = (o[0] + o[1] + o[2] + o[3]) * 0.5;
        double in = wetHp_.tick(0.5 * (L + R)) * 0.35;
        for (int i = 0; i < kFdn; ++i) {
            double fb = (o[i] - sum) * fdnGain_;           // Householder: energy preserving mix
            fdnBuf_[i][fdnPos_[i]] = (float)(in + fb);
            fdnPos_[i] = (fdnPos_[i] + 1) % fdnLen_[i];
        }
        L += wet_ * (o[0] + 0.6 * o[2]);
        R += wet_ * (o[1] + 0.6 * o[3]);
    }

    L = dcL_.tick(L);
    R = dcR_.tick(R);

    // physical scale: 1.0 full scale ~= 28 Pa (~123 dB SPL)
    gainSmooth_ += (volume_ - gainSmooth_) * 0.001;
    double g = gainSmooth_ / 28.0;
    L *= g; R *= g;
    // gentle 2:1 compressor so idle is audible without the WOT clipping
    double e2 = 0.5 * (L * L + R * R);
    env_ += (e2 - env_) * (1.0 / (0.25 * fs_));
    double env = std::sqrt(std::max(env_, 1e-10));
    double cg = clampv(std::pow(env / 0.12, -0.6), 0.35, 4.0);
    comp_ += (cg - comp_) * (1.0 / (0.05 * fs_));
    L *= comp_; R *= comp_;
    peak_ = std::max(std::fabs(L), std::max(std::fabs(R), peak_ * 0.9995));
    outL = (float)std::tanh(L);
    outR = (float)std::tanh(R);
}

} // namespace es

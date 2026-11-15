#pragma once
#include "core/MathUtil.h"

namespace es {

// RBJ biquad
// Bank of two-pole resonators (structure-borne modes of an engine block / head).
// Each mode: y = 2 r cos(w) y1 - r^2 y2 + g sin(w) x, r = exp(-1/(tau fs)), tau = Q / (pi f).
struct ModalBank {
    static constexpr int kMax = 10;
    int n = 0;
    double a1[kMax] = {}, a2[kMax] = {}, g[kMax] = {}, y1[kMax] = {}, y2[kMax] = {};
    void clear() { n = 0; }
    void add(double f, double q, double gain, double fs) {
        if (n >= kMax || f <= 0 || f > 0.45 * fs) return;
        double w = 2 * kPi * f / fs, r = std::exp(-kPi * f / (q * fs));
        a1[n] = 2 * r * std::cos(w); a2[n] = -r * r; g[n] = gain * std::sin(w);
        y1[n] = y2[n] = 0;
        ++n;
    }
    double tick(double x) {
        double s = 0;
        for (int i = 0; i < n; ++i) {
            double y = a1[i] * y1[i] + a2[i] * y2[i] + g[i] * x;
            y2[i] = y1[i]; y1[i] = y;
            s += y;
        }
        return s;
    }
};

struct Biquad {
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    double z1 = 0, z2 = 0;
    void reset() { z1 = z2 = 0; }
    double tick(double x) {
        double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void set(double B0, double B1, double B2, double A0, double A1, double A2) {
        b0 = B0 / A0; b1 = B1 / A0; b2 = B2 / A0; a1 = A1 / A0; a2 = A2 / A0;
    }
    void lowpass(double fc, double q, double fs) {
        double w = 2 * kPi * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        set((1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void highpass(double fc, double q, double fs) {
        double w = 2 * kPi * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        set((1 + c) / 2, -(1 + c), (1 + c) / 2, 1 + al, -2 * c, 1 - al);
    }
    void bandpass(double fc, double q, double fs) { // 0 dB peak
        double w = 2 * kPi * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        set(al, 0, -al, 1 + al, -2 * c, 1 - al);
    }
    void peaking(double fc, double q, double gainDb, double fs) {
        double A = std::pow(10.0, gainDb / 40.0);
        double w = 2 * kPi * fc / fs, c = std::cos(w), al = std::sin(w) / (2 * q);
        set(1 + al * A, -2 * c, 1 - al * A, 1 + al / A, -2 * c, 1 - al / A);
    }
    void lowshelf(double fc, double gainDb, double fs) {
        double A = std::pow(10.0, gainDb / 40.0);
        double w = 2 * kPi * fc / fs, c = std::cos(w), s = std::sin(w);
        double al = s / 2 * std::sqrt(2.0);
        double sa = 2 * std::sqrt(A) * al;
        set(A * ((A + 1) - (A - 1) * c + sa), 2 * A * ((A - 1) - (A + 1) * c), A * ((A + 1) - (A - 1) * c - sa),
            (A + 1) + (A - 1) * c + sa, -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - sa);
    }
};

struct OnePole {
    double a = 1, y = 0;
    void lowpass(double fc, double fs) { a = onePoleCoef(fc, fs); }
    double lp(double x) { y += a * (x - y); return y; }
    double hp(double x) { y += a * (x - y); return x - y; }
};

} // namespace es

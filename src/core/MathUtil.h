#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace es {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

template <class T> inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
template <class T> inline T lerp(T a, T b, T t) { return a + (b - a) * t; }
inline double smoothstep(double e0, double e1, double x) {
    double t = clampv((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
// wrap angle into [0, period)
inline double wrapPos(double a, double period) {
    a = std::fmod(a, period);
    return a < 0 ? a + period : a;
}
// wrap angle into [-period/2, period/2)
inline double wrapSym(double a, double period) {
    a = wrapPos(a + period * 0.5, period);
    return a - period * 0.5;
}

// small fast PRNG (xorshift) - deterministic, lock free, per-instance
struct Rng {
    uint64_t s = 0x9E3779B97F4A7C15ull;
    explicit Rng(uint64_t seed = 1) : s(seed * 0x9E3779B97F4A7C15ull + 0x632BE59BD9B4E019ull) {}
    uint64_t next() {
        s ^= s << 13; s ^= s >> 7; s ^= s << 17;
        return s;
    }
    double uniform() { return (next() >> 11) * (1.0 / 9007199254740992.0); }   // [0,1)
    double bipolar() { return uniform() * 2.0 - 1.0; }                           // [-1,1)
    double gauss() { // approx normal(0,1)
        return (uniform() + uniform() + uniform() + uniform() - 2.0) * 1.7320508;
    }
};

// deterministic ~N(0,1) value for (a, b): fixed per-cylinder "manufacturing" spread
inline double hashGauss(uint32_t a, uint32_t b) {
    uint64_t x = (uint64_t)a * 0x9E3779B97F4A7C15ull ^ ((uint64_t)b + 0x632BE59BD9B4E019ull) * 0xC2B2AE3D27D4EB4Full;
    double s = 0;
    for (int i = 0; i < 4; ++i) {
        x += 0x9E3779B97F4A7C15ull;
        uint64_t z = x;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        s += (z >> 11) * (1.0 / 9007199254740992.0);
    }
    return (s - 2.0) * 1.7320508;
}

// cylinder-to-cylinder spread: hashGauss with the engine's mean removed, so it only makes the
// cylinders differ from each other (a single cylinder gets none) and never shifts the overall tune
inline double cylSpread(int idx, int count, uint32_t kind) {
    if (count <= 1) return 0.0;
    double mean = 0;
    for (int j = 0; j < count; ++j) mean += hashGauss((uint32_t)j, kind);
    return hashGauss((uint32_t)idx, kind) - mean / count;
}

// one-pole lowpass coefficient for cutoff fc at sample rate fs
inline double onePoleCoef(double fc, double fs) { return 1.0 - std::exp(-2.0 * kPi * fc / fs); }

} // namespace es

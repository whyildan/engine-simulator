// Diagnostic: an engine standing still at any crank angle must be silent.
// For every preset (or one: stop_probe <preset>) the crank is parked at 0..690 deg, the engine stepped 1.5 s with
// the ignition on (key on, engine stalled) and the RMS and the dominant frequency of each sound source printed.
// Also: start, idle, key off, wait for it to stop, then measure.
#include "sim/Engine.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;

static const char* kSrc[5] = {"exhL", "exhR", "intake", "mech", "boost"};

struct Meas { double rms[5]; double freq[5]; };

static double domFreq(const std::vector<float>& x, double fs) {
    // coarse DFT peak 50 Hz .. 12 kHz on the last 4096 samples
    const int N = 4096; if ((int)x.size() < N) return 0;
    const float* s = x.data() + x.size() - N;
    double best = 0, bf = 0;
    for (double f = 50; f < 12000; f *= 1.01) {
        double re = 0, im = 0, w = 2 * 3.14159265358979 * f / fs;
        for (int k = 0; k < N; ++k) { re += s[k] * std::cos(w * k); im += s[k] * std::sin(w * k); }
        const double m = re * re + im * im;
        if (m > best) { best = m; bf = f; }
    }
    return bf;
}

static Meas measure(Engine& e, double fs, double secs) {
    std::vector<float> buf[5];
    double ss[5] = {};
    const int n = (int)(fs * secs), from = n / 3;
    for (int k = 0; k < n; ++k) {
        e.step(1.0 / fs, 0, 0);
        if (k >= from) {
            const double v[5] = {e.sExhL, e.sExhR, e.sIntake, e.sMech, e.sBoost};
            for (int j = 0; j < 5; ++j) { ss[j] += v[j] * v[j]; buf[j].push_back((float)v[j]); }
        }
    }
    Meas m;
    for (int j = 0; j < 5; ++j) {
        m.rms[j] = std::sqrt(ss[j] / (n - from));
        m.freq[j] = m.rms[j] > 1e-4 ? domFreq(buf[j], fs) : 0;
    }
    return m;
}

static void print(const char* tag, const Meas& m, double rpm) {
    bool loud = false;
    for (int j = 0; j < 5; ++j) loud = loud || m.rms[j] > 1e-4;
    std::printf("  %-10s rpm %5.0f", tag, rpm);
    for (int j = 0; j < 5; ++j)
        if (m.rms[j] > 1e-4) std::printf("  %s %.1e @%.0f Hz", kSrc[j], m.rms[j], m.freq[j]);
    std::printf("%s\n", loud ? "   <-- NOISE" : "");
}

int main(int argc, char** argv) {
    const double fs = 96000;
    const int only = argc > 1 ? std::atoi(argv[1]) : -1;
    int bad = 0;
    if (argc > 2) {   // stop_probe <preset> <angle>: intake / exhaust RMS every 0.25 s over 8 s
        EngineParams p = makePreset(only);
        p.ignition = 0; p.starter = 0; p.throttle = 0;
        if (const char* v = std::getenv("RUNNER")) p.runnerLen_mm = (float)std::atof(v);
        if (const char* v = std::getenv("PLENUM")) p.plenum_L = (float)std::atof(v);
        if (const char* v = std::getenv("FILTER")) p.airFilter = (float)std::atof(v);
        std::printf("runner %.0f mm dia %.0f, plenum %.1f L, filter %.2f\n", p.runnerLen_mm, p.runnerDia_mm, p.plenum_L, p.airFilter);
        Engine e(p, fs);
        e.setCrank(0, std::atof(argv[2]));
        const int nb = std::getenv("NB") ? std::atoi(std::getenv("NB")) : 32;
        std::vector<float> rec;
        for (int b = 0; b < nb; ++b) {
            double si = 0, se = 0; const int n = (int)(fs / 4);
            for (int k = 0; k < n; ++k) { e.step(1 / fs, 0, 0); si += e.sIntake * e.sIntake; se += e.sExhL * e.sExhL; rec.push_back((float)e.sIntake); }
            std::printf("t %5.2f  intake %.2e  exh %.2e  rpm %.2f theta %.2f\n", (b + 1) * 0.25, std::sqrt(si / n), std::sqrt(se / n), e.rpm(),
                        e.theta());
        }
        std::printf("intake dominant %.0f Hz\n", domFreq(rec, fs));
        return 0;
    }
    for (int i = 0; i < presetCount(); ++i) {
        if (only >= 0 && i != only) continue;
        std::printf("%s\n", makePreset(i).name);
        for (int a = 0; a < 720; a += 30) {
            EngineParams p = makePreset(i);
            p.ignition = std::getenv("IGN") ? std::atoi(std::getenv("IGN")) : 1; p.starter = 0; p.throttle = 0;
            Engine e(p, fs);
            e.setCrank(0, a);
            Meas m = measure(e, fs, 1.5);
            bool loud = false;
            for (int j = 0; j < 5; ++j) loud = loud || m.rms[j] > 1e-4;
            if (loud) { char t[16]; std::snprintf(t, sizeof(t), "park %d", a); print(t, m, e.rpm()); ++bad; }
        }
        {
            EngineParams p = makePreset(i);
            p.ignition = 1; p.starter = 1;
            Engine e(p, fs);
            for (int k = 0; k < (int)fs; ++k) e.step(1 / fs, 0, 0);
            p.starter = 0; e.apply(p);
            for (int k = 0; k < (int)(fs * 2); ++k) e.step(1 / fs, 0, 0);
            p.ignition = 0; e.apply(p);
            for (int k = 0; k < (int)(fs * 4); ++k) e.step(1 / fs, 0, 0);
            Meas m = measure(e, fs, 1.5);
            char t[24]; std::snprintf(t, sizeof(t), "key off %.0f", std::fmod(e.theta(), 720.0));
            print(t, m, e.rpm());
            for (int j = 0; j < 5; ++j) if (m.rms[j] > 1e-4) { ++bad; break; }
        }
    }
    std::printf(bad ? "\n%d noisy case(s)\n" : "\nall silent\n", bad);
    return bad ? 1 : 0;
}

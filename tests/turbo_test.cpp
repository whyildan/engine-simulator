// Turbo compression system and its sound: Greitzer surge on a lift-off without a blow-off valve (flow reversal, its
// rhythm from the charge volume), the blow-off valve's dynamics (vent / recirculating), and the compressor's tones
// (the whistle at the shaft order, the flutter loud enough to hear).
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Run {
    double wotShaft = 0, wotBoost = 0, cyclesHz = 0, minFlow = 0, maxLift = 0;
    double rmsWot = 0, rmsLift = 0;
    std::vector<float> wot;   // boost sound at full throttle (48 kHz)
    bool wotSurge = false;
};

// the engine held at 4500 rpm: 4 s full throttle, then the throttle shut for 1.5 s
static Run run(EngineParams p) {
    const double fs = 96000.0, dt = 1.0 / fs, w = 4500.0 * 2.0 * kPi / 60.0;
    p.ignition = 1;
    Engine e(p, fs);
    e.setCrank(w, 0.0);
    Run r;
    int k = 0, rev = 0; bool inRev = false;
    double sw = 0, sl = 0; int nw = 0, nl = 0;
    for (double t = 0; t < 5.5; t += dt) {
        const bool lift = t >= 4.0;
        e.setCrank(w, e.theta());
        e.step(dt, lift ? 0.0 : 1.0, 0.0);
        const double s = e.boost.sound();
        if (!lift && t > 3.0) {
            if (++k % 2 == 0) r.wot.push_back((float)s);
            sw += s * s; ++nw;
            r.wotShaft += e.boost.shaftRpm; r.wotBoost += e.boost.boostBar;
            r.wotSurge = r.wotSurge || e.boost.surging;
        }
        if (lift && t < 5.0) {
            sl += s * s; ++nl;
            r.minFlow = std::min(r.minFlow, e.boost.compFlow);
            r.maxLift = std::max(r.maxLift, e.boost.bovLift);
            const bool rv = inRev ? e.boost.compFlow < 0.2 * e.boost.lineS : e.boost.compFlow < 0.0;
            if (rv && !inRev) ++rev;
            inRev = rv;
        }
    }
    r.wotShaft /= nw; r.wotBoost /= nw;
    r.cyclesHz = rev / 1.0;
    r.rmsWot = 20 * std::log10(std::sqrt(sw / nw) + 1e-12);
    r.rmsLift = 20 * std::log10(std::sqrt(sl / nl) + 1e-12);
    return r;
}

// frequency of the strongest spectral line between lo and hi (Hz) - a plain DFT scan
static double peakHz(const std::vector<float>& x, double fs, double lo, double hi) {
    const int n = std::min<int>((int)x.size(), 16384);
    double best = 0, bf = 0;
    for (double f = lo; f <= hi; f += 2.0) {
        std::complex<double> acc = 0;
        for (int i = 0; i < n; ++i) acc += (double)x[i] * std::polar(1.0, -2.0 * kPi * f * i / fs) * (0.5 - 0.5 * std::cos(2.0 * kPi * i / n));
        if (std::abs(acc) > best) { best = std::abs(acc); bf = f; }
    }
    return bf;
}

int main() {
    const EngineParams base = makeCalibratedPreset(16);   // I4 2.0 turbo

    std::printf("[surge] lift-off without a blow-off valve (Greitzer compression system)\n");
    EngineParams p = base; p.bov = 0;
    const Run r = run(p);
    std::printf("  WOT: shaft %.0f rpm, boost %.2f bar; lift: %.0f reversal cycles in 1 s, flow down to %.3f kg/s\n", r.wotShaft, r.wotBoost,
                r.cyclesHz, r.minFlow);
    CHECK(!r.wotSurge, "steady at full throttle: no surge");
    CHECK(r.minFlow < -0.005, "the flow reverses through the wheel (deep surge)");
    CHECK(r.cyclesHz >= 8 && r.cyclesHz <= 45, "a flutter rhythm of %.0f Hz (literature: deep surge 3-10 Hz on rigs with big plenums, faster with smaller ones)",
          r.cyclesHz);
    CHECK(r.rmsLift > r.rmsWot - 8.0, "the flutter is audible next to the full-throttle whistle (%.1f vs %.1f dB)", r.rmsLift, r.rmsWot);

    std::printf("[surge] a smaller charge volume (pipes / intercooler) flutters faster\n");
    EngineParams q = p; q.chargeVolume_L = 1.2f;
    const Run rs = run(q);
    std::printf("  3.0 L: %.0f Hz   1.2 L: %.0f Hz\n", r.cyclesHz, rs.cyclesHz);
    CHECK(rs.cyclesHz > r.cyclesHz, "shorter surge period with a smaller plenum");

    std::printf("[blow-off valve] vent to atmosphere / recirculating\n");
    EngineParams v = base; v.bov = 1; v.bovType = 0;
    const Run rv = run(v);
    EngineParams c = base; c.bov = 1; c.bovType = 1;
    const Run rc = run(c);
    std::printf("  vent: lift %.2f, flow down to %.3f kg/s, lift-off %.1f dB | recirculating: %.1f dB\n", rv.maxLift, rv.minFlow, rv.rmsLift, rc.rmsLift);
    CHECK(rv.maxLift > 0.9 && rv.minFlow > -0.002, "the valve opens fully and the compressor does not surge");
    CHECK(rc.rmsLift < rv.rmsLift - 3.0, "a recirculating valve is quieter than one venting to the air");
    CHECK(std::fabs(rv.wotBoost - r.wotBoost) < 0.02, "the valve stays shut at full throttle (same boost)");

    std::printf("[whistle] the audible turbo whistle is the shaft order\n");
    const double f1 = r.wotShaft / 60.0, pk = peakHz(r.wot, 48000.0, 0.5 * f1, 1.5 * f1);
    std::printf("  shaft %.0f rpm = %.0f Hz; strongest line between %.0f and %.0f Hz: %.0f Hz\n", r.wotShaft, f1, 0.5 * f1, 1.5 * f1, pk);
    CHECK(std::fabs(pk / f1 - 1.0) < 0.03, "whistle at the shaft frequency");

    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

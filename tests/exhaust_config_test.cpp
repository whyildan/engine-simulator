// Exhaust configurations: open stacks, the glasspack, the reverse-flow box, the valved (active) muffler, a high-flow
// cat and the split (dual / quad) exit. Every one builds, runs finite and keeps the engine pulling; the loudness order
// follows what the parts do (open stacks and a glasspack louder than a chambered box, the reverse-flow OEM box quieter;
// the valved muffler louder with its flap open than shut, and the flap follows its mode); a split exit has twice the
// tips.
#include "sim/Simulator.h"
#include "sim/Engine.h"
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Res { double rmsDb = -200, hfDb = -200, torque = 0, valve = -1; bool finite = true; };

// RBJ biquad high-pass (Q 0.707): what a muffler does shows above a few hundred hertz - below, the firing pulses pass
// any reactive box nearly whole
struct HighPass {
    double b0, b1, b2, a1, a2, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    HighPass(double fc, double fs) {
        const double w = 2.0 * 3.14159265358979 * fc / fs, c = std::cos(w), al = std::sin(w) / (2.0 * 0.7071), a0 = 1.0 + al;
        b0 = (1.0 + c) * 0.5 / a0; b1 = -(1.0 + c) / a0; b2 = b0; a1 = -2.0 * c / a0; a2 = (1.0 - al) / a0;
    }
    double tick(double x) { const double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2; x2 = x1; x1 = x; y2 = y1; y1 = y; return y; }
};

// held at 3500 rpm on the dyno at 70 % throttle; the output's RMS over the last second (dB re 1)
static Res measure(EngineParams p) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 3500; p.irMix = 0;   // (the dry exhaust: no impulse response)
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    Telemetry t;
    Res r;
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int i = 0; i < 100; ++i) sim.render(buf.data(), 480);
    p.starter = 0; p.throttle = 0.7f; sim.setParams(p);
    double e = 0, eh = 0, tq = 0; long n = 0; int m = 0;
    HighPass hp(500.0, Simulator::kOutRate);
    for (int i = 0; i < 300; ++i) {
        sim.render(buf.data(), 480);
        sim.getTelemetry(t);
        for (float x : buf) if (!std::isfinite(x)) r.finite = false;
        for (size_t j = 0; j < buf.size(); j += 2) {
            const double mono = 0.5 * ((double)buf[j] + buf[j + 1]), h = hp.tick(mono);
            if (i >= 200) { e += mono * mono; eh += h * h; ++n; }
        }
        if (i >= 200) { tq += t.torque; ++m; }
    }
    r.rmsDb = 10.0 * std::log10(std::max(1e-20, e / std::max(1L, n)));
    r.hfDb = 10.0 * std::log10(std::max(1e-20, eh / std::max(1L, n)));
    r.torque = tq / std::max(1, m);
    r.valve = t.exhValve;
    return r;
}

int main() {
    const EngineParams base = makePreset(0);   // the K20A-style I4: tri-Y header, cat, resonator, straight-through can
    auto with = [&](auto&& edit) { EngineParams p = base; edit(p); return p; };
    std::printf("[configurations] I4 at 3500 rpm, 70 %% throttle: output level (dB re full scale), above 500 Hz, torque\n");
    struct Row { const char* name; EngineParams p; Res r; };
    std::vector<Row> rows = {
        {"chambered", with([](EngineParams& p) { p.muffler = Muffler::Chambered; }), {}},
        {"glasspack", with([](EngineParams& p) { p.muffler = Muffler::Glasspack; }), {}},
        {"reverse-flow", with([](EngineParams& p) { p.muffler = Muffler::ReverseFlow; }), {}},
        {"valved, open", with([](EngineParams& p) { p.muffler = Muffler::Valved; p.exhaustValveMode = 1; }), {}},
        {"valved, shut", with([](EngineParams& p) { p.muffler = Muffler::Valved; p.exhaustValveMode = 2; }), {}},
        {"valved, auto (opens > 3000)", with([](EngineParams& p) { p.muffler = Muffler::Valved; p.exhaustValveMode = 0; p.exhaustValveRpm = 3000; }), {}},
        {"open stacks", with([](EngineParams& p) { p.header = HeaderType::OpenStacks; }), {}},
        {"high-flow cat", with([](EngineParams& p) { p.muffler = Muffler::Chambered; p.cat = 2; }), {}},
        {"chambered, split", with([](EngineParams& p) { p.muffler = Muffler::Chambered; p.exhaustSplit = 1; }), {}},
    };
    for (Row& w : rows) {
        w.r = measure(w.p);
        std::printf("  %-28s %6.1f dB  above 500 Hz %6.1f dB  %5.0f Nm\n", w.name, w.r.rmsDb, w.r.hfDb, w.r.torque);
        if (w.r.valve >= 0) std::printf("  %-28s flap %.2f\n", "", w.r.valve);
    }
    auto R = [&](int i) { return rows[i].r; };
    bool finite = true; double tqMin = 1e9;
    for (const Row& w : rows) { finite &= w.r.finite; tqMin = std::min(tqMin, w.r.torque); }
    CHECK(finite, "every configuration runs finite");
    CHECK(tqMin > 0.6 * R(0).torque, "and keeps the engine pulling (least torque %.0f vs %.0f Nm)", tqMin, R(0).torque);
    CHECK(R(1).hfDb > R(0).hfDb + 1.0, "above 500 Hz: a glasspack is louder than a chambered box (%+.1f dB)", R(1).hfDb - R(0).hfDb);
    CHECK(R(2).hfDb < R(0).hfDb - 1.0, "the reverse-flow OEM box is quieter than a chambered one (%+.1f dB)", R(2).hfDb - R(0).hfDb);
    CHECK(R(3).hfDb > R(4).hfDb + 2.0, "the valved muffler is louder with its flap open (%+.1f dB)", R(3).hfDb - R(4).hfDb);
    CHECK(R(3).valve > 0.95 && R(4).valve < 0.05 && R(5).valve > 0.95, "the flap: open / shut as set, auto open above its speed");
    CHECK(R(6).rmsDb > R(0).rmsDb + 3.0, "open stacks are the loudest of these (%+.1f dB over the chambered box)", R(6).rmsDb - R(0).rmsDb);
    CHECK(R(7).hfDb > R(0).hfDb, "a high-flow cat lets more through than the ceramic one (%+.1f dB)", R(7).hfDb - R(0).hfDb);

    {
        std::printf("[outlets] tips per layout\n");
        auto a = std::make_unique<Engine>(with([](EngineParams& p) { p.exhaustSplit = 1; }), 96000.0);   // (heap: an Engine is large)
        EngineParams v8 = makePreset(4); v8.crossover = Crossover::TrueDual;
        auto b = std::make_unique<Engine>(v8, 96000.0);
        v8.exhaustSplit = 1;
        auto c = std::make_unique<Engine>(v8, 96000.0);
        auto d = std::make_unique<Engine>(with([](EngineParams& p) { p.header = HeaderType::OpenStacks; }), 96000.0);
        std::printf("  I4 split %d, V8 true dual %d, + split %d, I4 open stacks %d\n", a->exhaust.outlets, b->exhaust.outlets, c->exhaust.outlets, d->exhaust.outlets);
        CHECK(a->exhaust.outlets == 2 && b->exhaust.outlets == 2 && c->exhaust.outlets == 4 && d->exhaust.outlets == 4,
              "split: one pipe two tips, a true dual four; open stacks one per cylinder");
        CHECK((int)a->exhaust.net.openEnds.size() == a->exhaust.outlets && (int)c->exhaust.net.openEnds.size() == 4, "each tip an open end of the network");
    }
    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

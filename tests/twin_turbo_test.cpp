// Twin turbos and turbo manifolds: parallel twins (an inline's front / rear halves, a V's banks), a sequential pair
// (Mazda 13B-REW / Toyota 2JZ-GTE style), the twin-scroll manifold and a screamer pipe - on the dyno.
#include "sim/Simulator.h"
#include "sim/Engine.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <memory>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Trace { std::vector<float> rpm, boost, n1, n2; std::vector<int> stage; bool finite = true; };

// started, then held on the dyno: `segs` of (hold rpm, throttle, seconds); telemetry every 10 ms
static Trace run(EngineParams p, const std::vector<std::array<double, 3>>& segs) {
    // started off the dyno (cranked until it runs on its own, as START ENGINE does), then held
    p.dynoMode = DynoMode::Off;
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    Telemetry t;
    Trace tr;
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (double x = 0, caught = 0; x < 8.0 && caught < 0.15; x += 0.01) {
        sim.render(buf.data(), 480);
        sim.getTelemetry(t);
        caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
    }
    p.starter = 0; sim.setParams(p);
    for (int i = 0; i < 100; ++i) sim.render(buf.data(), 480);
    p.dynoMode = DynoMode::Hold;
    for (const auto& s : segs) {
        p.dynoHoldRpm = (float)s[0]; p.throttle = (float)s[1]; sim.setParams(p);
        for (double x = 0; x < s[2]; x += 0.01) {
            sim.render(buf.data(), 480);
            sim.getTelemetry(t);
            for (float v : buf) if (!std::isfinite(v)) tr.finite = false;
            tr.rpm.push_back(t.rpm); tr.boost.push_back(t.boostBar); tr.n1.push_back(t.shaftRpm); tr.n2.push_back(t.shaftRpm2);
            tr.stage.push_back(t.seqStage);
        }
    }
    return tr;
}
static double timeTo(const Trace& t, size_t from, double level) {
    for (size_t i = from; i < t.boost.size(); ++i) if (t.boost[i] >= level) return (i - from) * 0.01;
    return 99.0;
}
static double meanOf(const std::vector<float>& v, size_t a, size_t b) { double s = 0; for (size_t i = a; i < b; ++i) s += v[i]; return s / std::max<size_t>(1, b - a); }

int main() {
    // ---- parallel twins against one turbo of the same total size (I6 big single turbo preset)
    {
        std::printf("[parallel] I6 3.0 at 3500 rpm: tip-in from a light throttle to full\n");
        EngineParams one = makeCalibratedPreset(19);
        EngineParams two = one; applyTurboCount(two, 2); two.twinMode = 0;
        const std::vector<std::array<double, 3>> seg = {{3500, 0.15, 2.0}, {3500, 1.0, 6.0}};
        const Trace a = run(one, seg), b = run(two, seg);
        const size_t t0 = 200;
        const double full = meanOf(a.boost, a.boost.size() - 100, a.boost.size());
        const double ta = timeTo(a, t0, 0.5 * full), tb = timeTo(b, t0, 0.5 * full);
        const double fb = meanOf(b.boost, b.boost.size() - 100, b.boost.size());
        std::printf("  single: %.2f bar after 6 s, half of it in %.2f s | twins: %.2f bar, half in %.2f s; twin speeds %.0f / %.0f rpm\n",
                    full, ta, fb, tb, b.n1.back(), b.n2.back());
        CHECK(a.finite && b.finite, "both run finite");
        CHECK(tb < ta, "the two smaller turbos spool quicker than one big one");
        CHECK(std::fabs(b.n1.back() - b.n2.back()) < 0.1 * b.n1.back(), "the twins, front and rear, run at nearly the same speed");
    }
    // ---- a V8, one turbo per bank
    {
        std::printf("[per bank] V8 4.5 flat-plane with a turbo per bank, true dual\n");
        EngineParams v = makePreset(5);
        applyInductionDefaults(v, Induction::Turbo); applyTurboCount(v, 2); v.twinMode = 0; v.crossover = Crossover::TrueDual;
        generateCalibration(v);
        measureBaseMap(v, v.cal);   // (the app measures it when the engine changes)
        auto e = std::make_unique<Engine>(v, 96000.0);
        std::printf("  turbine inlets %d, outlets %d, exhaust tips %d\n", e->exhaust.turbineInCount(), e->exhaust.turbineOutCount(), e->exhaust.outlets);
        CHECK(e->exhaust.turbineInCount() == 2 && e->exhaust.turbineOutCount() == 2 && e->exhaust.outlets == 2, "each bank into its own turbo and its own downpipe");
        const Trace t = run(v, {{4500, 1.0, 5.0}});
        std::printf("  4500 rpm full throttle: %.2f bar, turbos %.0f / %.0f rpm\n", t.boost.back(), t.n1.back(), t.n2.back());
        CHECK(t.finite && t.boost.back() > 0.3 && t.n2.back() > 0.5 * t.n1.back(), "both turbos make boost");
    }
    // ---- sequential: the rotary twin turbo preset (13B-REW-style)
    {
        std::printf("[sequential] 2-rotor 1.3 turbo as a sequential pair, change-over at 4500 rpm\n");
        EngineParams s = makeCalibratedPreset(18);
        applyTurboCount(s, 2); applyTwinMode(s, 1); s.seqRpm = 4500;
        const Trace lo = run(s, {{3000, 1.0, 4.0}});
        const Trace hi = run(s, {{5500, 1.0, 4.0}});
        std::printf("  3000 rpm: stage %d, turbos %.0f / %.0f rpm, %.2f bar | 5500 rpm: stage %d, %.0f / %.0f rpm, %.2f bar\n", lo.stage.back(),
                    lo.n1.back(), lo.n2.back(), lo.boost.back(), hi.stage.back(), hi.n1.back(), hi.n2.back(), hi.boost.back());
        CHECK(lo.stage.back() == 0 && lo.n2.back() < 0.35 * lo.n1.back(), "down low the first turbo works alone");
        CHECK(hi.stage.back() == 2 && hi.n2.back() > 0.7 * hi.n1.back(), "above the change-over both are on the charge");
        // full boost on the first turbo at 3800 rpm, then a slow climb through the change-over at full throttle: the
        // second joins without the boost falling away (the compressor valve waits until it has caught up)
        std::vector<std::array<double, 3>> ramp = {{3800, 1.0, 3.0}};
        for (int r = 3900; r <= 5400; r += 100) ramp.push_back({(double)r, 1.0, 0.30});
        const Trace sw = run(s, ramp);
        int iPre = -1, iX = -1, iOn = -1;
        for (size_t i = 0; i < sw.stage.size(); ++i) {
            if (iPre < 0 && sw.stage[i] >= 1) iPre = (int)i;
            if (iX < 0 && i > 300 && sw.rpm[i] >= s.seqRpm) iX = (int)i;
            if (iOn < 0 && sw.stage[i] == 2) iOn = (int)i;
        }
        if (std::getenv("TWIN_TRACE"))
            for (int i = std::max(0, iX - 60); i < std::min((int)sw.boost.size(), iOn + 120); i += 5)
                std::printf("    t %5.2f rpm %5.0f stage %d boost %5.2f n1 %6.0f n2 %6.0f\n", i * 0.01, sw.rpm[i], sw.stage[i], sw.boost[i], sw.n1[i], sw.n2[i]);
        const double before = iX > 30 ? meanOf(sw.boost, iX - 30, iX) : 0;
        double dip = 1e9;
        for (int i = std::max(0, iX); i < std::min((int)sw.boost.size(), iOn + 60); ++i) dip = std::min(dip, (double)sw.boost[i]);
        std::printf("  climbing: pre-spool from %.0f rpm, both on at %.0f rpm; boost %.2f bar before the change-over, lowest %.2f through it, %.2f at the end\n",
                    iPre >= 0 ? sw.rpm[iPre] : 0.0, iOn >= 0 ? sw.rpm[iOn] : 0.0, before, dip, sw.boost.back());
        CHECK(sw.finite && iPre >= 0 && iOn > iPre, "pre-spool first, then the second turbo comes on");
        CHECK(before > 0.5 && dip > 0.75 * before, "the boost does not fall away at the change-over (the compressor valve waits for the second turbo)");
    }
    // ---- twin-scroll against a log manifold (I4 turbo), low speed
    {
        std::printf("[twin-scroll] I4 2.0 turbo at 1300 rpm full throttle, the wastegate shut: log manifold vs twin-scroll\n");
        EngineParams a = makeCalibratedPreset(16);
        a.header = HeaderType::Log; a.wastegateBar = 2.5f;   // (the wastegate shut: what the turbine itself does)
        EngineParams b = a; b.header = HeaderType::TwinScroll;
        auto e = std::make_unique<Engine>(b, 96000.0);
        CHECK(e->exhaust.twinScroll() && e->exhaust.turbineInCount() == 2, "twin-scroll: two turbine inlets (1+4, 2+3)");
        const Trace ta = run(a, {{1300, 1.0, 5.0}}), tb = run(b, {{1300, 1.0, 5.0}});
        const double ba = meanOf(ta.boost, ta.boost.size() - 100, ta.boost.size()), bb = meanOf(tb.boost, tb.boost.size() - 100, tb.boost.size());
        const double na = meanOf(ta.n1, ta.n1.size() - 100, ta.n1.size()), nb = meanOf(tb.n1, tb.n1.size() - 100, tb.n1.size());
        std::printf("  log: %.2f bar, turbo %.0f rpm | twin-scroll: %.2f bar, turbo %.0f rpm\n", ba, na, bb, nb);
        CHECK(tb.finite && nb > 1.2 * na, "the divided manifold keeps the pulses' energy: the turbo spins faster down low");
    }
    // ---- the twin turbo presets
    {
        std::printf("[presets] 13B-REW, 2JZ-GTE (sequential), RB26DETT, VG30DETT (parallel): full throttle on the dyno\n");
        const int ids[4] = {18, 33, 34, 35};
        for (int id : ids) {
            const EngineParams q = makeCalibratedPreset(id);
            const bool seq = q.twinMode == 1;
            const double loRpm = seq ? std::min(3000.0, q.seqRpm - 1500.0) : 3000.0;   // (below the pre-spool)
            const Trace lo = run(q, {{loRpm, 1.0, 4.0}}), hi = run(q, {{5500, 1.0, 4.0}});
            std::printf("  %-52s low: %.2f bar stage %d %3.0f/%3.0f krpm | 5500: %.2f bar stage %d %3.0f/%3.0f krpm, engine %.0f rpm\n", presetName(id),
                        lo.boost.back(), lo.stage.back(), lo.n1.back() / 1000, lo.n2.back() / 1000, hi.boost.back(), hi.stage.back(), hi.n1.back() / 1000,
                        hi.n2.back() / 1000, hi.rpm.back());
            CHECK(q.turboCount == 2 && lo.finite && hi.finite, "%s: two turbos, runs finite", presetName(id));
            CHECK(std::fabs(hi.rpm.back() - 5500) < 300 && hi.boost.back() > 0.5 && hi.n2.back() > 0.6 * hi.n1.back(),
                  "  pulls at 5500 rpm with boost (%.2f bar) from both turbos", hi.boost.back());
            if (seq) CHECK(lo.stage.back() == 0 && hi.stage.back() == 2, "  sequential: the first alone below the change-over (%.0f rpm), both at 5500", loRpm);
        }
    }
    // ---- screamer pipe
    {
        std::printf("[screamer] the wastegate to its own pipe\n");
        EngineParams a = makeCalibratedPreset(16);
        a.wgScreamer = 1;
        auto e = std::make_unique<Engine>(a, 96000.0);
        CHECK(e->exhaust.screamer() != nullptr && e->exhaust.outlets == 2, "a second open end: the screamer pipe");
        const Trace t = run(a, {{5500, 1.0, 4.0}});
        CHECK(t.finite && t.boost.back() > 0.3, "runs and holds boost with the wastegate venting to the air (%.2f bar)", t.boost.back());
    }
    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

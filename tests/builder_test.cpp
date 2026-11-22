// New engine from scratch (EngineRecipe -> buildEngine): every kind of engine the builder makes has the size asked
// for, starts, idles near its target, revs out to its limiter with the throttle open and keeps running after a lift.
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

struct Run { double idleRpm = 0, idleMin = 1e9, peakRpm = 0, afterRpm = 0; bool finite = true; };

static bool gVerbose = false;
static Run drive(EngineParams p) {
    Simulator sim(p);
    std::vector<float> buf(480 * 2);
    Telemetry t;
    Run r;
    auto run = [&](double secs, auto&& each) {
        for (double s = 0; s < secs; s += 0.01) {
            sim.render(buf.data(), 480);
            for (float x : buf) if (!std::isfinite(x)) r.finite = false;
            sim.getTelemetry(t);
            if (!std::isfinite(t.rpm)) r.finite = false;
            static int tick = 0;
            if (gVerbose && ++tick % 25 == 0)
                std::printf("    rpm %6.0f map %5.1f thr %.2f iac %5.1f afr %5.2f adv %5.1f run %d mis %d starter %d\n", t.rpm, t.mapKPa, p.throttle, t.iac, t.afr,
                            t.advance, (int)t.running, t.misfires, (int)p.starter);
            each(s);
        }
    };
    // START ENGINE as the app does it: crank until it runs on its own above the release speed for 0.15 s (8 s at most)
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    double caught = 0;
    for (double s = 0; s < 8.0 && caught < 0.15; s += 0.01) {
        run(0.01, [](double) {});
        caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
    }
    p.starter = 0; sim.setParams(p);
    int n = 0;
    run(4.0, [&](double s) { if (s > 2.0) { r.idleRpm += t.rpm; ++n; r.idleMin = std::min(r.idleMin, (double)t.rpm); } });
    r.idleRpm /= std::max(1, n);
    p.throttle = 1.0f; sim.setParams(p);
    run(2.5, [&](double) { r.peakRpm = std::max(r.peakRpm, (double)t.rpm); });
    p.throttle = 0.0f; sim.setParams(p);
    run(3.0, [&](double) { r.afterRpm = t.rpm; });
    return r;
}

int main(int argc, char** argv) {
    if (argc > 2 && !std::strcmp(argv[1], "preset")) {   // the same run on a preset (for comparison)
        const EngineParams p = makeCalibratedPreset(std::atoi(argv[2]));
        const Run r = drive(p);
        std::printf("'%s': idle %.0f (min %.0f, target %.0f), peak %.0f (limiter %.0f), after %.0f, finite %d\n", p.name, r.idleRpm, r.idleMin,
                    p.idleTargetRpm, r.peakRpm, p.revLimit, r.afterRpm, (int)r.finite);
        return 0;
    }
    struct Case { const char* what; EngineRecipe r; };
    std::vector<Case> cases;
    auto add = [&](const char* what, BuildKind k, auto&& edit) { EngineRecipe r = defaultRecipe(k); edit(r); cases.push_back({what, r}); };
    add("petrol I4 2.0 (defaults)", BuildKind::Petrol, [](EngineRecipe&) {});
    add("petrol V8 5.7 pushrod carb", BuildKind::Petrol, [](EngineRecipe& r) {
        r.layout = Layout::V; r.cylinders = 8; r.displacement_L = 5.7f; r.boreStroke = 1.15f; r.valves = 2; r.cam = CamLayout::OHV; r.fuel = 2; });
    add("petrol I6 3.0 turbo sport", BuildKind::Petrol, [](EngineRecipe& r) { r.cylinders = 6; r.displacement_L = 3.0f; r.aspiration = 1; r.character = 1; });
    add("petrol flat-4 2.0 ITB race", BuildKind::Petrol, [](EngineRecipe& r) {
        r.layout = Layout::Boxer; r.cylinders = 4; r.displacement_L = 2.0f; r.boreStroke = 1.2f; r.fuel = 1; r.character = 2; });
    add("petrol single 0.6 carb air-cooled", BuildKind::Petrol, [](EngineRecipe& r) {
        r.cylinders = 1; r.displacement_L = 0.6f; r.boreStroke = 1.1f; r.valves = 2; r.cam = CamLayout::SOHC; r.fuel = 2; r.cooling = Cooling::Air; });
    add("petrol V6 3.5 centrifugal", BuildKind::Petrol, [](EngineRecipe& r) { r.layout = Layout::V; r.cylinders = 6; r.displacement_L = 3.5f; r.aspiration = 4; });
    add("petrol V12 6.0 sport", BuildKind::Petrol, [](EngineRecipe& r) { r.layout = Layout::V; r.cylinders = 12; r.displacement_L = 6.0f; r.character = 1; });
    add("diesel I4 2.0 turbo (defaults)", BuildKind::Diesel, [](EngineRecipe&) {});
    add("diesel I6 6.0 turbo", BuildKind::Diesel, [](EngineRecipe& r) { r.cylinders = 6; r.displacement_L = 6.0f; });
    add("diesel single 0.5 NA", BuildKind::Diesel, [](EngineRecipe& r) { r.cylinders = 1; r.displacement_L = 0.5f; r.aspiration = 0; r.valves = 2; });
    add("two-stroke 125 race", BuildKind::TwoStroke, [](EngineRecipe& r) { r.displacement_L = 0.125f; r.boreStroke = 1.0f; r.character = 2; });
    add("two-stroke 3-cyl 750 street", BuildKind::TwoStroke, [](EngineRecipe& r) { r.cylinders = 3; r.displacement_L = 0.75f; r.character = 0; });
    add("two-stroke 300 EFI single", BuildKind::TwoStroke, [](EngineRecipe& r) { r.displacement_L = 0.3f; r.boreStroke = 1.0f; r.fuel = 1; });
    add("rotary 2-rotor 1.3 (defaults)", BuildKind::Rotary, [](EngineRecipe&) {});
    add("rotary 2-rotor 1.3 bridge port turbo", BuildKind::Rotary, [](EngineRecipe& r) { r.character = 2; r.aspiration = 1; });
    add("rotary 3-rotor 2.0 street port turbo", BuildKind::Rotary, [](EngineRecipe& r) { r.cylinders = 3; r.displacement_L = 2.0f; r.character = 1; r.aspiration = 1; });

    // combinations not run reliably yet are swapped for the nearest that is
    {
        std::printf("[limits]\n");
        EngineRecipe a = defaultRecipe(BuildKind::Rotary); a.cylinders = 3; a.character = 2;
        EngineRecipe b = defaultRecipe(BuildKind::TwoStroke); b.cylinders = 3; b.fuel = 1;
        recipeLimit(a); recipeLimit(b);
        CHECK(a.character == 1 && b.fuel == 0, "a 3-rotor bridge port becomes a street port, an injected triple two-stroke carburetted");
    }

    // builder_test case <i> [key=value ...]: one case, with a recipe / parameter change (for tuning the builder)
    int only = -1;
    if (argc > 2 && !std::strcmp(argv[1], "case")) only = std::atoi(argv[2]);
    for (size_t ci = 0; ci < cases.size(); ++ci) {
        Case c = cases[ci];
        if (only >= 0 && (int)ci != only) continue;
        std::vector<std::pair<std::string, float>> tweak;
        for (int a = 3; only >= 0 && a < argc; ++a) {
            const char* eq = std::strchr(argv[a], '=');
            if (eq) tweak.push_back({std::string(argv[a], eq - argv[a]), (float)std::atof(eq + 1)});
        }
        for (auto& t : tweak) {
            if (t.first == "fuel") c.r.fuel = (int)t.second;
            if (t.first == "cyl") c.r.cylinders = (int)t.second;
            if (t.first == "disp") c.r.displacement_L = t.second;
            if (t.first == "chr") c.r.character = (int)t.second;
            if (t.first == "asp") c.r.aspiration = (int)t.second;
        }
        std::printf("[%s]\n", c.what);
        EngineParams p = buildEngine(c.r);
        for (auto& t : tweak) {
            if (t.first == "screw") p.idleScrew = t.second;
            if (t.first == "idle") p.idleTargetRpm = t.second;
            if (t.first == "pump") p.accelPump = t.second;
            if (t.first == "cr") p.compression = t.second;
            if (t.first == "plenum") p.plenum_L = t.second;
            if (t.first == "charge") p.chargeVolume_L = t.second;
            if (t.first == "gain") { p.cal.idleKp *= t.second; p.cal.idleKi *= t.second; p.cal.idleKd *= t.second; }
            if (t.first == "sgain") p.cal.idleSparkGain *= t.second;
            if (t.first == "itb") p.itb = (int)t.second;
            if (t.first == "verbose") gVerbose = t.second != 0;
            if (t.first == "crank") { for (int i = 0; i < p.cal.crankAfr.n; ++i) p.cal.crankAfr.v[i] *= t.second; p.cal.afterStartAfr *= t.second; }
        }
        if (baseMapApplies(p)) measureBaseMap(p, p.cal);   // (the app measures it when the engine is created)
        const double disp = ratedDisplacementL(p);
        std::printf("  '%s': %.3f L, %s%.1f x %.1f mm, CR %.1f, limiter %.0f rpm, idle target %.0f\n", p.name, disp,
                    isRotary(p) ? "rotor R x e x B " : "", isRotary(p) ? p.rotorRadius_mm : p.bore_mm, isRotary(p) ? p.eccentricity_mm : p.stroke_mm,
                    p.compression, p.revLimit, p.idleTargetRpm);
        CHECK(std::fabs(disp / c.r.displacement_L - 1.0) < 0.03, "built to the displacement asked for (%.3f vs %.3f L)", disp, c.r.displacement_L);
        CHECK(p.cylinders == c.r.cylinders, "with the cylinders asked for");
        const Run r = drive(p);
        std::printf("  idle %.0f rpm (min %.0f), free rev peak %.0f, after the lift %.0f\n", r.idleRpm, r.idleMin, r.peakRpm, r.afterRpm);
        CHECK(r.finite, "finite throughout");
        // (a two-stroke idles four-stroking - lumpy: the KTM TPI preset dips to 0.4 x its target the same way; a bridge
        //  or peripheral port rotary idles with its brap - the bridge-port preset dips to 0.5 x)
        const bool lumpy = isTwoStroke(p) || (isRotary(p) && c.r.character >= 2);
        const double dip = lumpy ? 0.3 : 0.4;
        CHECK(r.idleMin > dip * p.idleTargetRpm && r.idleRpm > 0.6 * p.idleTargetRpm && r.idleRpm < 1.6 * p.idleTargetRpm,
              "idles near its target (a carburettor sets its own idle with the screw)");
        CHECK(r.peakRpm > 0.8 * p.revLimit, "revs out toward its limiter with the throttle open");
        CHECK(r.afterRpm > 0.5 * p.idleTargetRpm, "still running after the lift");
    }
    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

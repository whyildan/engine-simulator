// Phase 3: closed-loop fuel trims (STFT / LTFT), knock control, soft limiter, idle target, drive-by-wire map,
// ECU boost control with overboost / boost cut, engine protection (limp mode, shutdown).
// usage: control_test
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static void run(Simulator& sim, double s) {
    std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }

static void startIdle(Simulator& sim, EngineParams& p, double idleSeconds) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 400; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, idleSeconds);
}

// average of a telemetry value over a window
template <class F> static double avg(Simulator& sim, double sec, F f) {
    double s = 0; int n = 0;
    for (double t = 0; t < sec; t += 0.05) { run(sim, 0.05); s += f(tel(sim)); ++n; }
    return s / std::max(1, n);
}

static void hold(Simulator& sim, EngineParams& p, float rpm, float thr) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = rpm; p.throttle = thr; sim.setParams(p);
}

int main() {
    // ---------------- closed loop: an injector the ECU believes 15 % too big runs lean; STFT corrects it,
    //                  the LTFT table learns it and keeps it in open loop
    {
        std::printf("[fuel trims] ECU injector data 15 %% too large\n");
        EngineParams p = makeCalibratedPreset(0);
        p.cal.autoTune = 0; p.cal.closedLoop = 1; p.cal.ltftLearn = 1; p.cal.ltftRate = 0.5f;
        p.cal.injFlow *= 1.15f;
        Simulator sim(p);
        startIdle(sim, p, 3.0);
        hold(sim, p, 2500, 0.12f);
        run(sim, 25.0);
        Telemetry t = tel(sim);
        const double lam = avg(sim, 2.0, [](const Telemetry& x) { return x.afr / x.targetAfr; });
        std::printf("  closed loop: STFT %+.1f %% LTFT %+.1f %%  AFR/target %.3f  map %.0f kPa\n", t.ecuStft, t.ecuLtft, lam, t.mapKPa);
        CHECK(t.closedLoop, "closed loop active at warm part load");
        CHECK(t.ecuLtft > 8.0, "the long-term trim learned the error (+%.1f %%)", t.ecuLtft);
        CHECK(std::fabs(lam - 1.0) < 0.04, "mixture back on target");
        p.cal.closedLoop = 0; sim.setParams(p);   // the learned table stays in the ECU (VE / LTFT only change when edited)
        run(sim, 3.0);
        const double lamOpen = avg(sim, 2.0, [](const Telemetry& x) { return x.afr / x.targetAfr; });
        t = tel(sim);
        std::printf("  open loop with the learned LTFT: STFT %+.1f %% LTFT %+.1f %%  AFR/target %.3f\n", t.ecuStft, t.ecuLtft, lamOpen);
        CHECK(std::fabs(lamOpen - 1.0) < 0.06, "open loop still on target thanks to the LTFT");
    }
    // ---------------- knock control: 12 degrees too much advance at full load
    {
        std::printf("[knock control] ignition map +12 deg, 3000 rpm WOT\n");
        auto runKnock = [](bool control, double& retard, double& knock, double& torque) {
            EngineParams p = makeCalibratedPreset(0);
            for (auto& row : p.cal.ignition.v) for (float& v : row) v += 12;
            p.cal.knockControl = control;
            Simulator sim(p);
            startIdle(sim, p, 1.5);
            hold(sim, p, 3000, 1.0f);
            run(sim, 4.0);
            // 6 s: the knock control steps its retard per knock event and recovers between them - a short window
            // right after the hold catches it still converging and depends on the random burn sequence
            knock = avg(sim, 6.0, [](const Telemetry& x) { return x.knock; });
            Telemetry t = tel(sim);
            retard = 0; for (int i = 0; i < p.cylinders; ++i) retard = std::max(retard, (double)t.knockRetard[i]);
            torque = t.torque;
        };
        double r0, k0, tq0, r1, k1, tq1;
        runKnock(false, r0, k0, tq0);
        runKnock(true, r1, k1, tq1);
        std::printf("  without control: knock %.2f retard %.1f | with control: knock %.2f retard %.1f deg\n", k0, r0, k1, r1);
        CHECK(k0 > 1.0, "the over-advanced engine knocks");
        CHECK(r1 > 2.0 && k1 < k0 * 0.8, "knock control retards the knocking cylinders and the knock drops");
    }
    // ---------------- soft limiter: free rev at WOT
    {
        std::printf("[soft limiter] free rev WOT\n");
        EngineParams p = makeCalibratedPreset(0);
        p.cal.softLimitBand = 300; p.cal.softLimitCut = 0.4f; p.cal.softLimitRetard = 10;
        Simulator sim(p);
        startIdle(sim, p, 1.0);
        p.throttle = 1; sim.setParams(p);
        run(sim, 3.0);
        double cut = 0, rmax = 0;
        for (int k = 0; k < 40; ++k) { run(sim, 0.05); Telemetry t = tel(sim); cut = std::max(cut, (double)t.cutFraction); rmax = std::max(rmax, (double)t.rpm); }
        std::printf("  max rpm %.0f (limit %.0f), soft cut up to %.0f %%\n", rmax, p.revLimit, cut * 100);
        CHECK(cut > 0.05, "the soft limiter cuts sparks before the hard limit");
        CHECK(rmax < p.revLimit + 250, "revs held near the limit");
    }
    // ---------------- idle target curve and drive-by-wire map
    {
        std::printf("[idle / DBW] idle target +350 rpm, half-throttle pedal map\n");
        EngineParams p = makeCalibratedPreset(0);
        for (int i = 0; i < p.cal.idleTarget.n; ++i) p.cal.idleTarget.v[i] += 350;
        for (int i = 0; i < p.cal.dbwMap.n; ++i) p.cal.dbwMap.v[i] = p.cal.dbwMap.x[i] * 0.5f;
        Simulator sim(p);
        startIdle(sim, p, 6.0);
        const double idle = avg(sim, 2.0, [](const Telemetry& x) { return x.rpm; });
        Telemetry t = tel(sim);
        std::printf("  idle %.0f rpm (target %.0f)\n", idle, t.idleTarget);
        CHECK(std::fabs(idle - t.idleTarget) < 120, "idle follows the target curve");
        p.throttle = 1; sim.setParams(p);
        run(sim, 1.0);
        t = tel(sim);
        std::printf("  full pedal -> throttle %.0f %%\n", t.throttle * 100);
        CHECK(p.comp.throttleType != ThrottleType::DriveByWire || std::fabs(t.throttle - 0.5f) < 0.06f, "drive-by-wire map halves the throttle");
    }
    // ---------------- boost control: target 0.4 bar above the spring
    {
        std::printf("[boost control] I4 turbo, target spring + 0.4 bar at 5000 rpm WOT\n");
        auto runBoost = [](bool ctl, float addBar, float cutBar, double& boost, bool& cut) {
            EngineParams p = makeCalibratedPreset(16);
            p.cal.boostControl = ctl;
            for (int r = 0; r < p.cal.boostTarget.ny; ++r)
                for (int k = 0; k < p.cal.boostTarget.nx; ++k)
                    p.cal.boostTarget.v[r][k] = p.cal.boostTarget.y[r] >= 40 ? p.wastegateBar + addBar : 0.0f;
            if (cutBar > 0) p.cal.boostCutBar = cutBar;
            Simulator sim(p);
            startIdle(sim, p, 1.0);
            hold(sim, p, 5000, 1.0f);
            run(sim, 5.0);
            cut = false;
            double s = 0; int n = 0;
            for (int k = 0; k < 40; ++k) { run(sim, 0.05); Telemetry t = tel(sim); s += t.mapKPa / 101.325 - 1.0; ++n; cut = cut || t.boostCut || t.overboostCut; }
            boost = s / n;
        };
        EngineParams p = makePreset(16);
        double b0, b1, b2; bool c0, c1, c2;
        runBoost(false, 0.4f, 0, b0, c0);
        runBoost(true, 0.4f, 0, b1, c1);
        std::printf("  spring only %.2f bar | ECU control %.2f bar (target %.2f)\n", b0, b1, p.wastegateBar + 0.4f);
        CHECK(std::fabs(b0 - p.wastegateBar) < 0.15, "without control the spring sets the boost");
        CHECK(std::fabs(b1 - (p.wastegateBar + 0.4)) < 0.12, "ECU control holds the target above the spring");
        runBoost(true, 0.4f, p.wastegateBar + 0.2f, b2, c2);
        std::printf("  boost cut at %.2f bar: average %.2f bar, cut seen %d\n", p.wastegateBar + 0.2f, b2, (int)c2);
        CHECK(c2 && b2 < b1 - 0.05, "boost cut engages and pulls the boost down");
    }
    // ---------------- protection: coolant limit below the running temperature -> limp mode
    {
        std::printf("[protection] coolant limit 70 C -> limp mode\n");
        EngineParams p = makeCalibratedPreset(0);
        p.cal.prot[kProtCoolant] = {1, 65, 70, 0.5f, 5, kActLimp};
        p.cal.limpRpm = 3000;
        Simulator sim(p);
        startIdle(sim, p, 2.0);
        p.throttle = 1; sim.setParams(p);
        run(sim, 3.0);
        double rmax = 0;
        for (int k = 0; k < 20; ++k) { run(sim, 0.05); rmax = std::max(rmax, (double)tel(sim).rpm); }
        Telemetry t = tel(sim);
        std::printf("  limp %d, state %d, max rpm %.0f, throttle %.0f %%\n", (int)t.limp, t.protState[kProtCoolant], rmax, t.throttle * 100);
        CHECK(t.limp && t.protState[kProtCoolant] == 2, "limp mode active");
        CHECK(rmax < 3300, "rpm capped in limp mode");
        CHECK(p.comp.throttleType != ThrottleType::DriveByWire || t.throttle < 0.4f, "throttle limited (drive-by-wire)");
        p.cal.prot[kProtCoolant].limit = 130; p.cal.prot[kProtCoolant].warn = 120; sim.setParams(p);
        run(sim, 1.0);
        CHECK(!tel(sim).limp, "limp clears when the condition is gone");
    }
    // ---------------- protection: oil pressure rule with shutdown
    {
        std::printf("[protection] oil pressure limit 9 bar -> shutdown\n");
        EngineParams p = makeCalibratedPreset(0);
        p.cal.prot[kProtOilPress] = {1, 10, 9, 0.3f, 0.3f, kActShutdown};
        Simulator sim(p);
        startIdle(sim, p, 4.0);
        Telemetry t = tel(sim);
        std::printf("  shutdown %d, rpm %.0f\n", (int)t.shutdown, t.rpm);
        CHECK(t.rpm < 100, "the engine was shut down");
        run(sim, 3.0);
        CHECK(!tel(sim).shutdown, "the shutdown clears once the engine has stopped");
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

// Phase 7: trigger wheels and sync, injection modes, coil dwell, torque management, gear-based control,
// friction multiplier, redline, calibration file round trip of the new settings.
// usage: phase7_test
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include "ecu/CalibrationIO.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static void run(Simulator& sim, double s) {
    static std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }
template <class F> static double avg(Simulator& sim, double sec, F f) {
    double s = 0; int n = 0;
    for (double t = 0; t < sec; t += 0.05) { run(sim, 0.05); s += f(tel(sim)); ++n; }
    return s / std::max(1, n);
}
// crank until the engine runs on its own; returns the time to the first running state (s), -1 = no start
static double start(Simulator& sim, EngineParams& p, double idle = 1.5) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    double t = 0, first = -1;
    for (; t < 4.0; t += 0.01) {
        run(sim, 0.01);
        Telemetry x = tel(sim);
        if (first < 0 && x.running) first = t;
        if (x.running && x.rpm > starterReleaseRpm(p, x.snsValue[kSnsECT])) break;
    }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, idle);
    return first;
}
static double wot(Simulator& sim, EngineParams& p, double rpm, double settle, Telemetry* last = nullptr) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = 1; sim.setParams(p);
    run(sim, settle);
    const double tq = avg(sim, 1.0, [](const Telemetry& t) { return t.torque; });
    if (last) *last = tel(sim);
    return tq;
}
static void flat(Curve1D& c, float v) { for (int i = 0; i < c.n; ++i) c.v[i] = v; }

int main() {
    // ---------------- trigger wheels
    {
        std::printf("[trigger] K20A: crank degrees to sync and time to start per trigger wheel\n");
        double deg[kTrigCount] = {};
        for (int k = 0; k < kTrigCount; ++k) {
            EngineParams p = makeCalibratedPreset(0);
            p.cal.triggerPattern = k;
            Simulator sim(p);
            const double t0 = start(sim, p);
            Telemetry t = tel(sim);
            deg[k] = t.syncDeg;
            std::printf("  %-40s sync after %4.0f deg, running after %.2f s, idle %4.0f rpm, injection %d\n", triggerInfo(k).name, t.syncDeg, t0,
                        t.rpm, t.injMode);
            CHECK(t0 > 0 && t0 < 2.0 && t.rpm > 600 && t.crankSync && t.camSync && t.injMode == kInjSequential,
                  "%s: starts and runs sequential", triggerInfo(k).name);
        }
        CHECK(deg[kTrig36_2_2_2] < deg[kTrig36_1] && deg[kTrig36_1] < deg[kTrig4Cam], "36-2-2-2 syncs first (%.0f deg), the cam-only wheel last (%.0f deg)",
              deg[kTrig36_2_2_2], deg[kTrig4Cam]);
        EngineParams p = makeCalibratedPreset(0);
        p.cal.camSensor = 0;
        Simulator sim(p);
        start(sim, p, 3.0);
        Telemetry t = tel(sim);
        CHECK(!t.camSync && t.injMode == kInjSemiSequential && t.rpm > 600, "no cam sensor: semi-sequential, idles at %.0f rpm", t.rpm);
    }

    // ---------------- injection modes
    {
        std::printf("[injection] sequential / semi-sequential / batch: idle and WOT 4000 rpm\n");
        double tq[3] = {};
        for (int m = 0; m < 3; ++m) {
            EngineParams p = makeCalibratedPreset(0);
            p.cal.injMode = m;
            Simulator sim(p);
            start(sim, p, 3.0);
            const double idle = avg(sim, 1.0, [](const Telemetry& x) { return x.rpm; });
            const double idleAfr = avg(sim, 1.0, [](const Telemetry& x) { return x.afr; });
            Telemetry w;
            tq[m] = wot(sim, p, 4000, 2.0, &w);
            static const char* nm[] = {"sequential", "semi-sequential", "batch"};
            std::printf("  %-16s idle %4.0f rpm AFR %.2f | WOT %.1f Nm AFR %.2f (target %.2f)\n", nm[m], idle, idleAfr, tq[m], w.afr, w.targetAfr);
            CHECK(w.injMode == m && idle > 600 && std::fabs(w.afr - w.targetAfr) < 0.8, "%s runs on its target", nm[m]);
        }
        CHECK(std::fabs(tq[1] - tq[0]) < 0.06 * tq[0] && std::fabs(tq[2] - tq[0]) < 0.06 * tq[0], "WOT torque within 6 %% across the modes");
    }

    // ---------------- dwell
    {
        std::printf("[dwell] I4 turbo at 4500 rpm WOT: dwell table vs a too-short dwell\n");
        int mis[2] = {};
        double tq[2] = {};
        for (int k = 0; k < 2; ++k) {
            EngineParams p = makeCalibratedPreset(16);
            if (k) flat(p.cal.dwell, 0.25f);
            Simulator sim(p);
            start(sim, p);
            Telemetry t0 = tel(sim);
            Telemetry t;
            tq[k] = wot(sim, p, 4500, 3.0, &t);
            mis[k] = t.misfireCause[1] - t0.misfireCause[1];
            std::printf("  dwell %.1f ms: spark %.0f mJ, %.1f Nm, spark misfires %d\n", t.dwellMs, t.sparkAvail, tq[k], mis[k]);
        }
        CHECK(mis[0] == 0 && mis[1] > 10 && tq[1] < 0.9 * tq[0], "short dwell starves the spark under boost (%d misfires)", mis[1]);
    }

    // ---------------- torque management
    {
        std::printf("[torque] K20A in first gear at full throttle, torque limit 60 %% in first\n");
        for (int cable = 0; cable < 2; ++cable) {
            double clutch[2] = {}, red = 0, load[2] = {}, cap = 1;
            for (int on = 0; on < 2; ++on) {
                EngineParams p = makeCalibratedPreset(0);
                p.vehicle = 1; p.transMode = TransMode::ManualAutoClutch;
                if (cable) p.comp.throttleType = ThrottleType::Cable;
                p.cal.torqueMgmt = on; flat(p.cal.tqLimitGear, 150); p.cal.tqLimitGear.v[0] = 60;
                Simulator sim(p);
                start(sim, p);
                p.gear = 1; p.throttle = 0.3f; sim.setParams(p);
                for (int k = 0; k < 300; ++k) { run(sim, 0.02); Telemetry x = tel(sim); if (x.clutchLocked && x.rpm > 2500) break; }
                p.throttle = 1; sim.setParams(p);
                run(sim, 0.4);
                // engine torque, not clutch torque: right after the clutch locks in first gear the driveline shuffles
                // and the clutch torque swings 20-180 Nm from sample to sample (the average depended on the random
                // burn sequence); the engine torque averages cleanly. 1.5 s: the cable throttle's share of random
                // spark cuts needs a few hundred sparks to average out (0.6 s spread 72-98 Nm over seeds)
                double sum = 0; int n = 0;
                for (int k = 0; k < 75; ++k) { run(sim, 0.02); sum += tel(sim).torque; ++n; }
                clutch[on] = sum / n;
                Telemetry t = tel(sim);
                load[on] = t.tqLoad;
                if (on) { red = t.tqReduction; cap = t.tqThrCap; }
            }
            std::printf("  %s: engine torque %.0f -> %.0f Nm, ECU load %.0f -> %.0f %% (limit 60), reduction %.2f, throttle cap %.2f\n",
                        cable ? "cable throttle" : "drive-by-wire", clutch[0], clutch[1], load[0], load[1], red, cap);
            CHECK(clutch[1] < 0.8 * clutch[0], "%s: the limit takes torque off in first gear", cable ? "cable" : "DBW");
            if (!cable) CHECK(cap < 0.9 && load[1] < 75, "DBW: the throttle cap holds the load near the limit (%.0f %%)", load[1]);
            else CHECK(red > 0.2, "cable: retard / cut take it off (reduction %.2f)", red);
        }
    }

    // ---------------- gear-based control
    {
        std::printf("[gear control] K20A: 5000 rpm limit and -6 deg in first gear\n");
        EngineParams p = makeCalibratedPreset(0);
        p.vehicle = 1; p.transMode = TransMode::ManualAutoClutch;
        p.cal.gearControl = 1; p.cal.gearRevLimit.v[0] = 5000; p.cal.gearSpark.v[0] = -6;
        Simulator sim(p);
        start(sim, p);
        p.gear = 1; p.throttle = 0.25f; sim.setParams(p);
        for (int k = 0; k < 300; ++k) { run(sim, 0.02); Telemetry x = tel(sim); if (x.clutchLocked && x.rpm > 2500) break; }
        const double advOn = avg(sim, 0.5, [](const Telemetry& x) { return x.advance; });
        p.cal.gearControl = 0; sim.setParams(p);
        const double advOff = avg(sim, 0.5, [](const Telemetry& x) { return x.advance; });
        p.cal.gearControl = 1; p.throttle = 1; sim.setParams(p);
        double rmax = 0;
        for (int k = 0; k < 150; ++k) { run(sim, 0.02); rmax = std::max(rmax, (double)tel(sim).rpm); }
        Telemetry t = tel(sim);
        std::printf("  advance %.1f (gear trim) vs %.1f, max rpm %.0f, limit in effect %.0f\n", advOn, advOff, rmax, t.gearLimitRpm);
        CHECK(advOn < advOff - 4, "spark trim by gear");
        CHECK(rmax < 5300 && t.gearLimitRpm == 5000, "rev limit by gear holds first gear under 5300 rpm");
    }

    // ---------------- friction, redline
    {
        std::printf("[engine] friction multiplier and redline\n");
        double tq[2];
        for (int k = 0; k < 2; ++k) {
            EngineParams p = makeCalibratedPreset(0);
            p.frictionMult = k ? 1.5f : 1.0f;
            Simulator sim(p);
            start(sim, p);
            tq[k] = wot(sim, p, 6000, 2.0);
        }
        CHECK(tq[1] < tq[0] - 5, "friction x1.5: %.1f -> %.1f Nm at 6000 rpm", tq[0], tq[1]);
        EngineParams p = makePreset(0);
        const float r0 = redlineRpm(p);
        p.redline = 8000;
        CHECK(r0 == p.revLimit - 300 && redlineRpm(p) == 8000, "redline: default %.0f (limiter - 300), set 8000", r0);
    }

    // ---------------- calibration file
    {
        std::printf("[calibration] Phase 7 settings survive a JSON round trip\n");
        EngineParams p = makePreset(0);
        Calibration& c = p.cal;
        c.triggerPattern = kTrig60_2; c.camSensor = 0; c.injMode = kInjBatch; c.dwellMaxDuty = 0.55f;
        c.torqueMgmt = 1; c.tqMaxRetard = 9; c.gearControl = 1;
        c.dwell.v[2] = 4.25f; c.tqLimitGear.v[1] = 77; c.gearRevLimit.v[0] = 6100; c.gearSpark.v[2] = -2.5f;
        CalMeta m; m.name = "p7";
        Calibration back = makePreset(0).cal;
        CalMeta mb; std::string err;
        const bool ok = calibrationFromJson(calibrationToJson(c, m), back, mb, err);
        CHECK(ok && back.triggerPattern == kTrig60_2 && back.camSensor == 0 && back.injMode == kInjBatch && back.dwellMaxDuty == 0.55f &&
              back.torqueMgmt == 1 && back.tqMaxRetard == 9 && back.gearControl == 1 && back.dwell.v[2] == 4.25f &&
              back.tqLimitGear.v[1] == 77 && back.gearRevLimit.v[0] == 6100 && back.gearSpark.v[2] == -2.5f,
              "trigger, injection, dwell, torque and gear settings read back");
    }

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}

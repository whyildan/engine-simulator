// Phase 5: launch control (2-step), flat shift, shift cut, traction control, transbrake, rolling anti-lag, nitrous.
// usage: motorsport_test
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
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

static void startIdle(Simulator& sim, EngineParams& p, double idleSeconds) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 400; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, idleSeconds);
}
template <class F> static double avg(Simulator& sim, double sec, F f) {
    double s = 0; int n = 0;
    for (double t = 0; t < sec; t += 0.05) { run(sim, 0.05); s += f(tel(sim)); ++n; }
    return s / std::max(1, n);
}

// car moving in first gear at full throttle (manual: clutch let out like a driver), until `rpm`
static void pullInFirst(Simulator& sim, EngineParams& p, double rpm) {
    if (p.transMode == TransMode::Manual) { p.clutchPedal = 1; sim.setParams(p); run(sim, 0.3); }
    p.gear = 1; sim.setParams(p); run(sim, 0.3);
    p.throttle = 1;
    for (double tt = 0; tt < 10; tt += 0.02) {
        if (p.transMode == TransMode::Manual) p.clutchPedal = (float)clampv(1.0 - tt / 0.8, 0.0, 1.0);
        sim.setParams(p); run(sim, 0.02);
        if (tt > 1.0 && tel(sim).rpm > rpm) break;
    }
}

int main() {
    // ---------------- 2-step: standing, clutch in, full throttle -> the revs sit on the launch rpm
    {
        std::printf("[2-step] I4 2.0, clutch in, WOT, launch at 4000 rpm (spark cut)\n");
        EngineParams p = makeCalibratedPreset(0);
        p.cal.launchControl = 1; p.cal.launchRpm = 4000; p.cal.launchCutMode = 0;
        Simulator sim(p);
        startIdle(sim, p, 1.5);
        const int pops0 = tel(sim).afterfires;
        p.clutchPedal = 1; p.throttle = 1; sim.setParams(p);
        run(sim, 2.0);
        double rmax = 0;
        const double r = avg(sim, 1.5, [&](const Telemetry& x) { rmax = std::max(rmax, (double)x.rpm); return x.rpm; });
        Telemetry t = tel(sim);
        std::printf("  rpm %.0f (max %.0f), state %s, cut %.0f %%, afterfire pops %d\n", r, rmax, launchStateName(t.launchState), t.cutFraction * 100,
                    t.afterfires - pops0);
        CHECK(t.launchState == kLaunch2Step, "2-step active");
        CHECK(std::fabs(r - 4000) < 250 && rmax < 4500, "revs held at the launch rpm");
        CHECK(t.afterfires > pops0, "the spark cut sends unburned mixture into the exhaust (pops)");
        p.clutchPedal = 0; sim.setParams(p); run(sim, 1.0);
        t = tel(sim);
        CHECK(t.launchState == kLaunchOff && t.rpm > 4600, "clutch out (no load): the limiter lets go");
    }
    // ---------------- 2-step on a turbo: retarded launch timing puts heat into the turbine and spins it up
    {
        std::printf("[2-step turbo] I6 3.0 big single turbo, launch 3500 rpm, launch timing -15 vs +15 deg\n");
        struct R { double boost, shaft, egt, rpm; };
        auto at = [](float timing) {
            EngineParams p = makeCalibratedPreset(19);
            p.cal.launchControl = 1; p.cal.launchRpm = 3500; p.cal.launchTiming = timing;
            Simulator sim(p);
            startIdle(sim, p, 1.5);
            p.clutchPedal = 1; p.throttle = 1; sim.setParams(p);
            run(sim, 2.5);
            R r;
            r.rpm = avg(sim, 1.0, [](const Telemetry& x) { return x.rpm; });
            r.boost = avg(sim, 1.0, [](const Telemetry& x) { return x.boostBar; });
            r.shaft = avg(sim, 1.0, [](const Telemetry& x) { return x.shaftRpm; });
            r.egt = avg(sim, 1.0, [](const Telemetry& x) { return x.snsValue[kSnsEGT]; });
            return r;
        };
        const R a = at(-15), b = at(15);
        std::printf("  retarded: %.0f rpm, boost %.2f bar, turbo %.0f rpm, EGT %.0f C | advanced: %.0f rpm, boost %.2f bar, turbo %.0f rpm, EGT %.0f C\n",
                    a.rpm, a.boost, a.shaft, a.egt, b.rpm, b.boost, b.shaft, b.egt);
        CHECK(a.shaft > b.shaft * 1.1 && a.egt > b.egt + 50, "retarded launch timing heats the exhaust and spins the turbo up");
    }
    // ---------------- flat shift: clutch in at full throttle while moving -> revs held, not flaring to the limiter
    {
        std::printf("[flat shift] I4 2.0 manual, clutch in at 5000 rpm in first, throttle kept open\n");
        auto flare = [](int mode, double& atClutch) {
            EngineParams p = makeCalibratedPreset(0);
            p.vehicle = 1; p.transMode = TransMode::Manual;
            p.cal.clutchMovingMode = mode; p.cal.flatShiftMargin = 150;
            Simulator sim(p);
            startIdle(sim, p, 1.5);
            pullInFirst(sim, p, 5000);
            atClutch = tel(sim).rpm;
            p.clutchPedal = 1; sim.setParams(p);
            double rmax = 0;
            for (int k = 0; k < 20; ++k) { run(sim, 0.02); rmax = std::max(rmax, (double)tel(sim).rpm); }
            return rmax;
        };
        double c0, c1;
        const double off = flare(0, c0), on = flare(2, c1);
        std::printf("  clutch in at %.0f -> max %.0f rpm without, %.0f -> max %.0f rpm with flat shift\n", c0, off, c1, on);
        CHECK(on < c1 + 150 + 300, "flat shift holds the revs near the clutch-in rpm");
        CHECK(off > on + 300, "without it the engine flares");
    }
    // ---------------- shift cut: auto-clutch upshift at full throttle
    {
        std::printf("[shift cut] I4 2.0 auto-clutch, upshift at 5500 rpm at full throttle\n");
        auto flare = [](int cut) {
            EngineParams p = makeCalibratedPreset(0);
            p.vehicle = 1; p.transMode = TransMode::ManualAutoClutch;
            p.cal.shiftCut = cut; p.cal.shiftCutTime = 0.15f;
            Simulator sim(p);
            startIdle(sim, p, 1.5);
            pullInFirst(sim, p, 5500);
            const double at = tel(sim).rpm;
            p.gear = 2; sim.setParams(p);
            double rmax = 0; int st = 0;
            for (int k = 0; k < 15; ++k) { run(sim, 0.02); Telemetry t = tel(sim); rmax = std::max(rmax, (double)t.rpm); st = std::max(st, t.launchState); }
            std::printf("  shift cut %s: shift at %.0f, max %.0f rpm (state %s)\n", cut ? "on " : "off", at, rmax, launchStateName(st));
            return rmax - at;
        };
        const double off = flare(0), on = flare(1);
        CHECK(on < off - 150, "the shift cut stops the flare during the gear change (%.0f vs %.0f rpm)", on, off);
    }
    // ---------------- traction control: a powerful car on a low-grip surface
    {
        std::printf("[traction control] I4 2.0 turbo, grip 0.6, rolling in first at 15 km/h, then full throttle\n");
        auto launch = [](int tc, double& speed) {
            EngineParams p = makeCalibratedPreset(16);
            p.vehicle = 1; p.transMode = TransMode::ManualAutoClutch; p.grip = 0.6f;
            p.cal.tractionControl = tc;
            Simulator sim(p);
            startIdle(sim, p, 1.5);
            // rolling in first gear with the clutch locked, then full throttle
            p.gear = 1; p.throttle = 0.3f; sim.setParams(p);
            for (int k = 0; k < 400; ++k) { run(sim, 0.02); Telemetry x = tel(sim); if (x.speedKmh > 15 && x.clutchLocked) break; }
            p.throttle = 1; sim.setParams(p);
            if (std::getenv("MSDBG"))
                for (int k = 0; k < 30; ++k) {
                    run(sim, 0.1); Telemetry x = tel(sim);
                    std::printf("    tc%d t %.1f  %5.0f rpm  %5.1f km/h wheel %5.1f  slip %4.0f %% (ecu %4.0f %% tgt %3.0f)  red %.2f ret %4.1f cut %.2f  boost %.2f clutch %.2f\n",
                                tc, 0.1 * (k + 1), x.rpm, x.speedKmh, x.wheelKmh, x.wheelSlip * 100, x.tcSlip, x.tcTarget, x.tcReduction, x.tcRetard, x.tcCut, x.boostBar, x.clutchEng);
                }
            const double slip = avg(sim, 2.0, [](const Telemetry& x) { return x.wheelSlip; });
            speed = tel(sim).speedKmh;
            return slip;
        };
        double v0, v1;
        const double s0 = launch(0, v0), s1 = launch(1, v1);
        std::printf("  mean slip %.0f %% -> %.0f %% with TC, speed after 2 s %.0f -> %.0f km/h\n", s0 * 100, s1 * 100, v0, v1);
        CHECK(s0 > 0.4, "without TC the tyres spin");
        CHECK(s1 < s0 * 0.6, "traction control holds the slip down");
        CHECK(v1 > v0 * 0.95, "and the car is not slower for it");
    }
    // ---------------- transbrake: automatic, the car is held while the engine sits on the 2-step
    {
        std::printf("[transbrake] I4 2.0 turbo automatic, transbrake + 2-step at 4000 rpm\n");
        EngineParams p = makeCalibratedPreset(16);
        p.vehicle = 1; p.transMode = TransMode::Automatic;
        p.cal.launchControl = 1; p.cal.launchRpm = 4000;
        Simulator sim(p);
        startIdle(sim, p, 1.5);
        p.gear = 1; p.transbrake = 1; sim.setParams(p); run(sim, 0.3);
        p.throttle = 1; sim.setParams(p); run(sim, 2.5);
        Telemetry t = tel(sim);
        std::printf("  held: %.1f km/h, %.0f rpm, state %s, boost %.2f bar\n", t.speedKmh, t.rpm, launchStateName(t.launchState), t.boostBar);
        CHECK(t.speedKmh < 1.0 && t.transbrake, "the transbrake holds the car");
        CHECK(t.launchState == kLaunch2Step && t.rpm < 4300 && t.rpm > 1800, "the engine pulls against the transbrake at or below the launch limit");
        CHECK(t.boostBar > 0.5, "loaded against the transbrake, the turbo builds boost before the launch");
        p.transbrake = 0; sim.setParams(p); run(sim, 2.0);
        t = tel(sim);
        std::printf("  released: %.0f km/h after 2 s\n", t.speedKmh);
        CHECK(t.speedKmh > 25, "releasing the button launches the car");
    }
    // ---------------- rolling anti-lag: moving, button held -> revs held, boost builds
    {
        std::printf("[rolling anti-lag] I4 2.0 turbo, rolling in first gear, button + full throttle\n");
        EngineParams p = makeCalibratedPreset(16);
        p.vehicle = 1; p.transMode = TransMode::ManualAutoClutch;
        Simulator sim(p);
        startIdle(sim, p, 1.5);
        p.gear = 1; p.throttle = 0.3f; sim.setParams(p);
        for (int k = 0; k < 400; ++k) { run(sim, 0.02); Telemetry x = tel(sim); if (x.speedKmh > 20 && x.clutchLocked) break; }
        p.throttle = 0.1f; sim.setParams(p); run(sim, 0.5);
        Telemetry t0 = tel(sim);
        p.rollingAlsButton = 1; p.throttle = 1; sim.setParams(p);
        run(sim, 2.0);
        Telemetry t = tel(sim);
        std::printf("  %.0f km/h: rpm %.0f -> %.0f, boost %.2f bar, state %s\n", t0.speedKmh, t0.rpm, t.rpm, t.boostBar, launchStateName(t.launchState));
        CHECK(t.launchState == kLaunchRollingAls, "rolling anti-lag active");
        CHECK(t.rpm < t0.rpm + 350, "revs held where the button went in");
        CHECK(t.boostBar > 0.2, "boost builds while rolling");
    }
    // ---------------- nitrous on the dyno: 5000 rpm hold, full throttle
    {
        std::printf("[nitrous] I6 3.0, dyno hold 5000 rpm WOT\n");
        EngineParams p = makeCalibratedPreset(2);
        p.nitrous = 1; p.n2oWet = 1; p.n2oShotHp = 100; p.cal.n2oFlowGs = (float)(n2oFullFlowKgs(p) * 1000.0);
        p.cal.n2oRetard = 4;
        Simulator sim(p);
        startIdle(sim, p, 1.5);
        p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 5000; p.throttle = 1; sim.setParams(p);
        run(sim, 3.0);
        const double tq0 = avg(sim, 1.0, [](const Telemetry& x) { return x.torque; });
        const double bottle0 = tel(sim).n2oBottleKg;
        p.n2oArm = 1; sim.setParams(p);
        run(sim, 2.0);
        const double tq1 = avg(sim, 1.0, [](const Telemetry& x) { return x.torque; });
        const double afr1 = avg(sim, 0.5, [](const Telemetry& x) { return x.afr; });
        Telemetry t = tel(sim);
        const double hp = (tq1 - tq0) * 5000 * 2 * kPi / 60.0 / 745.7;
        std::printf("  wet 100 hp shot: %.0f -> %.0f Nm (+%.0f hp), AFR %.1f, duty %.0f %%, %.0f g/s, bottle %.2f -> %.2f kg\n",
                    tq0, tq1, hp, afr1, t.n2oDuty * 100, t.n2oFlowGs, bottle0, t.n2oBottleKg);
        CHECK(hp > 60 && hp < 150, "the shot adds about its rated power (+%.0f hp)", hp);
        CHECK(afr1 > 10.5 && afr1 < 13.8, "wet kit mixture sane (AFR %.1f)", afr1);
        CHECK(t.n2oBottleKg < bottle0 - 0.05, "the bottle empties");
        // dry kit: the ECU adds the fuel
        p.n2oArm = 0; p.n2oWet = 0; sim.setParams(p); run(sim, 1.5);
        p.n2oArm = 1; sim.setParams(p); run(sim, 2.0);
        const double afrDry = avg(sim, 0.5, [](const Telemetry& x) { return x.afr; });
        std::printf("  dry kit: AFR %.1f (target %.1f)\n", afrDry, tel(sim).targetAfr);
        CHECK(afrDry > 10.5 && afrDry < 13.8, "dry kit: the ECU fuels the nitrous");
        // dry kit with an ECU that adds no fuel: lean -> the controller shuts the nitrous off
        p.n2oArm = 0; p.cal.n2oFlowGs = 0; sim.setParams(p); run(sim, 1.5);
        p.n2oArm = 1; sim.setParams(p); run(sim, 2.0);
        t = tel(sim);
        std::printf("  dry kit, ECU not fuelling it: lean lock %d, duty %.0f %%\n", (int)t.n2oLeanLock, t.n2oDuty * 100);
        CHECK(t.n2oLeanLock && t.n2oDuty == 0, "lean while spraying: nitrous shut off");
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

// Headless driving tests: automatic launch + full-throttle run, manual run with clutch work,
// and a clutch dump in first gear at idle (should stall a manual car).
// usage: drive_test [preset] [mode: 0 manual, 1 manual+auto clutch, 2 automatic] [verbose]
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace es;

static std::vector<float> buf(480 * 2);
static void run(Simulator& s, double sec) { for (double t = 0; t < sec; t += 0.01) s.render(buf.data(), 480); }

static int failures = 0;

static void accelRun(int preset, TransMode mode, bool verbose) {
    EngineParams p = makeCalibratedPreset(preset);
    p.vehicle = 1; p.transMode = mode; p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    Telemetry t;
    run(sim, 1.5); p.starter = 0; sim.setParams(p); run(sim, 2.0);
    sim.getTelemetry(t);
    std::printf("[%s | %s] idle %.0f rpm\n", p.name, mode == TransMode::Automatic ? "automatic" : mode == TransMode::Manual ? "manual" : "manual+auto clutch", t.rpm);
    const int n = p.gears;
    // select first / drive
    if (mode == TransMode::Manual) { p.clutchPedal = 1; sim.setParams(p); run(sim, 0.3); }
    p.gear = 1; sim.setParams(p); run(sim, 0.3);
    p.throttle = 1.0f;
    double t0 = 0;
    bool stalled = false;
    double lastShift = -10;
    for (double tt = 0; tt < 40; tt += 0.05) {
        if (mode == TransMode::Manual) {
            // simple driver: let the clutch out over 0.8 s from the launch, shift at the limiter
            sim.getTelemetry(t);
            if (tt < 0.9) p.clutchPedal = (float)clampv(1.0 - tt / 0.8, 0.0, 1.0);
            else if (t.rpm > p.revLimit - 250 && p.gear < n && p.clutchPedal == 0) { p.clutchPedal = 1; p.throttle = 0; sim.setParams(p); run(sim, 0.08); p.gear++; sim.setParams(p); run(sim, 0.08); p.clutchPedal = 0.3f; sim.setParams(p); run(sim, 0.08); p.clutchPedal = 0; p.throttle = 1; }
        } else if (mode == TransMode::ManualAutoClutch) {
            sim.getTelemetry(t);
            if (t.rpm > p.revLimit - 250 && p.gear < n && !t.shifting && tt - lastShift > 0.6) { p.gear++; lastShift = tt; }
        }
        sim.setParams(p);
        run(sim, 0.05);
        t0 += 0.05;
        sim.getTelemetry(t);
        if (verbose && ((int)(tt * 20) % 10 == 0))
            std::printf("  t %5.1f  %6.1f km/h  gear %d  %5.0f rpm  clutch %.2f %s  slip %+.2f  boost %+.2f  T %.0f Nm\n", tt, t.speedKmh, t.gear,
                        t.rpm, t.clutchEng, t.clutchLocked ? "L" : "s", t.wheelSlip, t.boostBar, t.clutchTq);
        if (t.rpm < 200) { stalled = true; break; }
        if (t.tQuarter > 0 && t.speedKmh > 0) break;
    }
    sim.getTelemetry(t);
    std::printf("  0-100 km/h %.2f s   1/4 mile %.2f s @ %.0f km/h   final gear %d   %s\n", t.t100, t.tQuarter, t.vQuarter, t.gear,
                stalled ? "STALLED" : "");
    if (stalled || t.t100 <= 0) { ++failures; std::printf("  FAIL\n"); }
}

static void clutchDump(int preset) {
    EngineParams p = makeCalibratedPreset(preset);
    p.vehicle = 1; p.transMode = TransMode::Manual; p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    Telemetry t;
    run(sim, 1.5); p.starter = 0; sim.setParams(p); run(sim, 2.0);
    p.clutchPedal = 1; sim.setParams(p); run(sim, 0.3);
    p.gear = 1; sim.setParams(p); run(sim, 0.3);
    p.clutchPedal = 0; sim.setParams(p); run(sim, 2.0);      // dump the clutch with no throttle
    sim.getTelemetry(t);
    std::printf("[clutch dump at idle, %s] rpm %.0f speed %.1f km/h -> %s\n", p.name, t.rpm, t.speedKmh, t.rpm < 200 ? "stalled (expected)" : "survived");
    // slow release instead: should creep without stalling
    EngineParams q = makeCalibratedPreset(preset);
    q.vehicle = 1; q.transMode = TransMode::Manual; q.ignition = 1; q.starter = 1;
    Simulator s2(q);
    run(s2, 1.5); q.starter = 0; s2.setParams(q); run(s2, 2.0);
    q.clutchPedal = 1; s2.setParams(q); run(s2, 0.3); q.gear = 1; s2.setParams(q); run(s2, 0.3);
    for (double tt = 0; tt < 3; tt += 0.05) { q.clutchPedal = (float)clampv(1.0 - tt / 2.5, 0.0, 1.0); q.throttle = 0.12f; s2.setParams(q); run(s2, 0.05); }
    s2.getTelemetry(t);
    std::printf("[slow clutch release + 12%% gas] rpm %.0f speed %.1f km/h -> %s\n", t.rpm, t.speedKmh, t.rpm > 400 ? "moving" : "STALLED");
    if (t.rpm < 400) ++failures;
}

int main(int argc, char** argv) {
    int preset = argc > 1 ? std::atoi(argv[1]) : 0;
    int mode = argc > 2 ? std::atoi(argv[2]) : -1;
    bool verbose = argc > 3;
    if (mode >= 0) accelRun(preset, (TransMode)mode, verbose);
    else {
        accelRun(preset, TransMode::Automatic, verbose);
        accelRun(preset, TransMode::ManualAutoClutch, verbose);
        accelRun(preset, TransMode::Manual, verbose);
        clutchDump(preset);
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

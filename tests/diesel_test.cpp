// Package 4: diesel. The Hardenberg-Hase ignition delay against a hand calculation, then the turbo diesel preset:
// it starts and idles on its governor with an open intake (no throttle), the load follows the pedal (fuel quantity),
// the smoke limiter holds the mixture at its lambda where the air runs short, a lower cetane fuel lights later,
// and turning the key off stops it (no injection).
// usage: diesel_test
#include "sim/Simulator.h"
#include "sim/Fuel.h"
#include "sim/CombustionNoise.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
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
static double start(Simulator& sim, EngineParams& p, double crankS = 4.0) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    double first = -1;
    for (double t = 0; t < crankS; t += 0.01) {
        run(sim, 0.01);
        Telemetry x = tel(sim);
        if (x.running && x.rpm > starterReleaseRpm(p, x.snsValue[kSnsECT])) { first = t; break; }
    }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    return first;
}
static void hold(Simulator& sim, EngineParams& p, float rpm, float pedal) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = rpm; p.throttle = pedal; sim.setParams(p);
}
constexpr int kDieselPreset = 24;

int main() {
    // ---------------- the correlation itself
    {
        std::printf("[ignition delay] Hardenberg-Hase\n");
        // Sp 10 m/s, 900 K, 60 bar, CN 50: E_A = 618840 / 75 = 8251.2 J/mol;
        // x = 8251.2 (1 / (8.314 * 900) - 1 / 17190) (21.2 / 47.6)^0.63 = 0.3741; tau = 2.56 e^x = 3.72 deg
        const double t = dieselDelayDeg(10.0, 900.0, 60.0, 50.0);
        std::printf("  tau(10 m/s, 900 K, 60 bar, CN 50) = %.3f deg CA\n", t);
        CHECK(std::fabs(t - 3.72) < 0.02, "matches the hand calculation (3.72 deg)");
        CHECK(dieselDelayDeg(10, 800, 60, 50) > t && dieselDelayDeg(10, 900, 40, 50) > t && dieselDelayDeg(10, 900, 60, 40) > t,
              "longer when colder, at lower pressure and with a lower cetane number");
        CHECK(dieselDelayDeg(10, 900, 12.0, 50) > 1e8, "no ignition below 12.4 bar (outside the correlation)");
    }

    // ---------------- start, idle, no throttle
    EngineParams p = makeCalibratedPreset(kDieselPreset);
    std::printf("[idle] %s\n", p.name);
    {
        Simulator sim(p);
        const double ts = start(sim, p);
        run(sim, 4.0);
        const double rpm = avg(sim, 2.0, [](const Telemetry& x) { return x.rpm; });
        const double map = avg(sim, 1.0, [](const Telemetry& x) { return x.mapKPa; });
        const Engine& e = *sim.engineForTests();
        std::printf("  start %.2f s, idle %.0f rpm (target %.0f), MAP %.0f kPa, %.1f mg/stroke, misfires %d\n", ts, rpm, p.idleTargetRpm,
                    map, e.ecu.cmd.dieselFuelMg, e.misfires);
        CHECK(ts >= 0 && ts < 2.0, "starts warm");
        CHECK(std::fabs(rpm - p.idleTargetRpm) < 100.0, "the governor holds the idle (%.0f rpm)", rpm);
        CHECK(map > 95.0, "no throttle: the manifold stays near ambient at idle (%.0f kPa)", map);
        CHECK(e.misfires == 0, "no misfires");
        // key off: no injection, it stops
        p.ignition = 0; sim.setParams(p);
        run(sim, 3.0);
        CHECK(tel(sim).rpm < 100.0, "key off: no injection, the engine stops (%.0f rpm)", tel(sim).rpm);
    }

    // ---------------- load = fuel quantity; smoke limiter
    {
        std::printf("[load] dyno hold 1500 rpm: half and full pedal\n");
        EngineParams q = p;   // (its own copy: the dyno hold must not carry over into the next sections)
        Simulator sim(q);
        start(sim, q);
        hold(sim, q, 1500, 0.5f);
        run(sim, 4.0);
        const double tqHalf = avg(sim, 1.0, [](const Telemetry& x) { return x.torque; });
        const double qHalf = sim.engineForTests()->ecu.cmd.dieselFuelMg;
        hold(sim, q, 1500, 1.0f);
        run(sim, 4.0);
        const double tqFull = avg(sim, 1.0, [](const Telemetry& x) { return x.torque; });
        const Engine& e = *sim.engineForTests();
        const double lam = e.cyl(0).lambda;
        std::printf("  half pedal %.1f mg %.0f Nm | full pedal %.1f mg %.0f Nm, lambda %.2f, smoke limited %d\n", qHalf, tqHalf,
                    e.ecu.cmd.dieselFuelMg, tqFull, lam, (int)e.ecu.smokeLimited);
        CHECK(tqFull > 1.3 * tqHalf && tqHalf > 20.0, "torque follows the fuel quantity");
        CHECK(lam > p.cal.smokeLambda - 0.08, "the smoke limiter holds lambda >= %.2f (%.2f)", p.cal.smokeLambda, lam);
    }

    // ---------------- cetane: a lower cetane number lights later
    {
        // at idle: the cetane number matters most in a cooler charge (Hardenberg-Hase's cetane term shrinks as the
        // charge gets hotter - at full load the difference is only a few per cent)
        std::printf("[cetane] ignition delay at idle: CN 51 against CN 40\n");
        double delay[2] = {};
        for (int k = 0; k < 2; ++k) {
            EngineParams q = p;
            if (k) q.fuelType = kFuelDieselCn40;
            Simulator sim(q);
            start(sim, q);
            run(sim, 4.0);
            delay[k] = avg(sim, 2.0, [&](const Telemetry&) { return sim.engineForTests()->cyl(0).idDeg; });
        }
        std::printf("  delay CN 51 %.2f deg, CN 40 %.2f deg\n", delay[0], delay[1]);
        CHECK(delay[1] > delay[0] * 1.05, "lower cetane: longer ignition delay");
    }

    // ---------------- combustion noise (4c): structure attenuation filter, chamber resonance, ringing
    {
        std::printf("[combustion noise] structure attenuation against the AVL curve (OSTI 1123537 Fig. 2)\n");
        const double fs = 96000;
        const double pts[][2] = {{500, -103}, {1000, -94}, {2000, -90}, {3000, -91}, {5000, -99}, {7000, -111}, {10000, -132}};
        double worst = 0;
        for (const auto& pt : pts) {
            StructureAttenuation st; st.init(fs);
            double in2 = 0, out2 = 0;
            for (int n = 0; n < 96000; ++n) {
                const double x = 1e5 * std::sin(2.0 * 3.14159265358979 * pt[0] * n / fs);
                const double y = st.tick(x);
                if (n > 48000) { in2 += x * x; out2 += y * y; }
            }
            const double db = 10.0 * std::log10(out2 / in2);
            std::printf("  %5.0f Hz: %7.1f dB (AVL %4.0f)\n", pt[0], db, pt[1]);
            worst = std::max(worst, std::fabs(db - pt[1]));
        }
        CHECK(worst < 3.0, "within 3 dB of the curve at every point (worst %.1f dB)", worst);
        // US 10443535 worked example: 1.28 x 287 x 2000 K -> c ~857 m/s, B 0.106 m -> ~4.7 kHz (here gamma 1.3)
        const double f = ringFrequency(0.106, 2000.0);
        std::printf("  first tangential mode, B 106 mm, 2000 K: %.0f Hz (source example ~4.7 kHz with gamma 1.28)\n", f);
        CHECK(std::fabs(f / 4700.0 - 1.0) < 0.03, "chamber resonance formula matches the worked example");

        std::printf("[ringing] idle: amplitude against Eng's beta (dP/dt)max\n");
        Simulator sim(p);
        start(sim, p);
        run(sim, 3.0);
        const Engine& e = *sim.engineForTests();
        double rmax = 0, dpm = 0, rpm = 0; int n = 0;
        for (int k = 0; k < 150; ++k) {
            run(sim, 0.01);
            rmax = std::max(rmax, e.cyl(0).ringKickA); dpm = std::max(dpm, e.cyl(0).dpMaxCycle); rpm += tel(sim).rpm; ++n;
        }
        rpm /= n;
        const double expect = 0.5 * kRingBeta * dpm * 6.0 * rpm;
        std::printf("  peak rise rate %.1f bar/deg at %.0f rpm -> expected amplitude %.2f bar, measured %.2f bar\n", dpm / 1e5, rpm, expect / 1e5, rmax / 1e5);
        CHECK(rmax > 0.6 * expect && rmax < 1.4 * expect, "the premixed burn rings the chamber at the amplitude Eng's relation gives");
    }

    // ---------------- glow plugs / cold start (4c)
    {
        std::printf("[cold start] key on, wait for the glow lamp, crank (cold engine)\n");
        auto coldStart = [&](float amb, int glow, double* lampS) {
            EngineParams q = p;
            q.ambientTemp_C = amb; q.coldStart = 1; q.glowPlugs = glow;
            Simulator sim(q);
            q.ignition = 1; sim.setParams(q);
            double wait = 0;
            for (int k = 0; k < 3000; ++k) { run(sim, 0.01); if (!tel(sim).glowLight && k > 5) break; wait += 0.01; }
            if (lampS) *lampS = wait;
            return start(sim, q, 6.0);
        };
        double lamp5 = 0;
        const double warmNoGlow = coldStart(10, 0, nullptr);
        const double coldNoGlow = coldStart(-15, 0, nullptr);
        const double coldGlow = coldStart(-15, 1, nullptr);
        coldStart(-5, 1, &lamp5);
        std::printf("  +10 C no glow plugs: %.2f s | -15 C no glow plugs: %.2f s | -15 C metal glow plugs: %.2f s | pre-heat at -5 C %.1f s\n",
                    warmNoGlow, coldNoGlow, coldGlow, lamp5);
        CHECK(warmNoGlow >= 0, "+10 C: starts without glow plugs");
        CHECK(coldNoGlow < 0, "-15 C: does not start without glow plugs (Bosch: DI needs them below about -5 to -10 C)");
        CHECK(coldGlow >= 0 && coldGlow < 2.0, "-15 C: starts with pre-heated glow plugs");
        CHECK(std::fabs(lamp5 - 5.0) < 0.5, "pre-heat time at -5 C ~5 s (BorgWarner / BERU)");
    }

    // ---------------- variable geometry turbine (4b)
    {
        std::printf("[VGT] full pedal: boost against the target; vanes that cannot close (fixed big turbine)\n");
        auto boostAt = [&](float rpm, float minArea, double* target) {
            EngineParams q = p;
            q.vgtMinArea = minArea;
            Simulator sim(q);
            start(sim, q);
            hold(sim, q, rpm, 1.0f);
            run(sim, 5.0);
            if (target) *target = avg(sim, 1.0, [](const Telemetry& x) { return x.boostTargetBar; });
            return avg(sim, 1.0, [](const Telemetry& x) { return x.boostBar; });
        };
        double t1500 = 0, t3500 = 0;
        const double b1500 = boostAt(1500, p.vgtMinArea, &t1500);
        const double bOpen = boostAt(1500, 1.0f, nullptr);
        const double b3500 = boostAt(3500, p.vgtMinArea, &t3500);
        std::printf("  1500 rpm: %.2f bar (target %.2f), vanes stuck open %.2f bar | 3500 rpm: %.2f bar (target %.2f)\n", b1500, t1500, bOpen, b3500, t3500);
        CHECK(std::fabs(b1500 - t1500) < 0.1, "1500 rpm: the vanes close and hold the target");
        CHECK(bOpen < 0.5 * b1500, "the same turbine with its vanes open makes far less boost low down");
        CHECK(std::fabs(b3500 - t3500) < 0.15, "3500 rpm: the vanes open and hold the target (no overboost)");
    }

    // ---------------- exhaust gas recirculation (4b)
    {
        std::printf("[EGR] 1500 rpm, 30 %% pedal: valve against none; full pedal 3000 rpm\n");
        struct R { double rate, maf, lam, iat; };
        auto egrAt = [&](float rpm, float pedal, float dia) {
            EngineParams q = p;
            q.egrValveDia_mm = dia;
            Simulator sim(q);
            start(sim, q);
            hold(sim, q, rpm, pedal);
            run(sim, 6.0);
            R r;
            r.rate = avg(sim, 1.0, [](const Telemetry& x) { return x.egrRate; });
            r.maf = avg(sim, 1.0, [](const Telemetry& x) { return x.airFlow; });
            r.lam = sim.engineForTests()->cyl(0).lambda;
            r.iat = avg(sim, 1.0, [](const Telemetry& x) { return x.snsValue[kSnsIAT]; });
            return r;
        };
        const R on = egrAt(1500, 0.3f, p.egrValveDia_mm), off = egrAt(1500, 0.3f, 0), full = egrAt(3000, 1.0f, p.egrValveDia_mm);
        std::printf("  with EGR: %.0f %% of the charge, fresh air %.1f g/s, lambda %.2f, intake %.0f C | without: fresh air %.1f g/s, lambda %.2f, intake %.0f C\n",
                    on.rate * 100, on.maf, on.lam, on.iat, off.maf, off.lam, off.iat);
        std::printf("  full pedal 3000 rpm: %.1f %% EGR\n", full.rate * 100);
        CHECK(on.rate > 0.1, "part load: the valve recirculates a real share of the charge (%.0f %%)", on.rate * 100);
        CHECK(on.maf < 0.9 * off.maf && on.lam < off.lam, "the recirculated gas takes the place of fresh air (less air, lower lambda)");
        CHECK(on.iat > off.iat + 10.0, "uncooled EGR warms the intake");
        CHECK(full.rate < 0.02, "full load: the valve stays shut");
    }

    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

// Package 4: diesel. The Hardenberg-Hase ignition delay against a hand calculation, then the turbo diesel preset:
// it starts and idles on its governor with an open intake (no throttle), the load follows the pedal (fuel quantity),
// the smoke limiter holds the mixture at its lambda where the air runs short, a lower cetane fuel lights later,
// and turning the key off stops it (no injection).
// usage: diesel_test
#include "sim/Simulator.h"
#include "sim/Fuel.h"
#include "sim/CombustionNoise.h"
#include "sim/Injector.h"
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
        // the engine's delay: Assanis et al. 2003 - phi 0.5, 60 bar, 900 K, CN 51: 2.4 x 0.5^-0.2 x 60^-1.02 x e^(2100/900)
        // = 2.4 x 1.1487 x 0.015365 x 10.312 = 0.4368 ms; 700 K: x e^(2100/700 - 2100/900) = x 1.948
        const double a = dieselDelayMs(0.5, 60.0, 900.0, 51.0), a7 = dieselDelayMs(0.5, 60.0, 700.0, 51.0);
        std::printf("  Assanis tau(phi 0.5, 60 bar, 900 K) = %.4f ms; 700 K / 900 K = %.3f\n", a, a7 / a);
        CHECK(std::fabs(a - 0.4368) < 0.002, "Assanis correlation matches the hand calculation (0.4368 ms)");
        CHECK(std::fabs(a7 / a - 1.948) < 0.01, "its temperature sensitivity (x1.95 from 900 to 700 K; Hardenberg-Hase gives ~x1.36)");
        CHECK(dieselDelayMs(0.5, 60, 900, 40) > a * 1.05, "a lower cetane number lengthens it (Hardenberg-Hase cetane factor)");
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

    // ---------------- pilot injection: less combustion noise (Busch et al., Sandia: ~6 dB at 1500 rpm, IMEPg 9 bar)
    {
        std::printf("[pilot] combustion noise with / without the pilot (structure-attenuated level)\n");
        auto noiseAt = [&](float rpm, float pedal, bool pilot) {
            EngineParams q = p;
            if (!pilot) q.cal.pilotMg = 0;
            Simulator sim(q);
            start(sim, q);
            if (rpm > 0) hold(sim, q, rpm, pedal);
            run(sim, 5.0);
            Engine& e = *const_cast<Engine*>(sim.engineForTests());
            e.combNoiseRms();   // (reset)
            run(sim, 3.0);
            return 20.0 * std::log10(std::max(1e-9, e.combNoiseRms()) / 20e-6);
        };
        const double on15 = noiseAt(1500, 0.5f, true), off15 = noiseAt(1500, 0.5f, false);
        const double onI = noiseAt(0, 0, true), offI = noiseAt(0, 0, false);
        std::printf("  1500 rpm 50 %% pedal (~9 bar IMEP): %.1f dB with the pilot, %.1f dB without (%+.1f dB)\n", on15, off15, on15 - off15);
        std::printf("  idle: %.1f dB with the pilot, %.1f dB without (%+.1f dB)\n", onI, offI, onI - offI);
        CHECK(off15 - on15 > 4.0 && off15 - on15 < 9.0, "1500 rpm mid load: the pilot cuts the noise by about Sandia's 6 dB");
        CHECK(offI - onI > 3.0, "idle: the pilot (longer dwell) quietens the clatter");
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

    // ---------------- D1-D11 (the open items of 2026-11-21)
    auto measure = [&](EngineParams q, float rpm, float pedal, double settle, auto f) {
        Simulator sim(q);
        start(sim, q);
        hold(sim, q, rpm, pedal);
        run(sim, settle);
        return f(sim);
    };
    // D6: the smoke limiter lands on its lambda (the air per stroke from the MAF integrated over each crank segment)
    {
        std::printf("[D6 smoke limiter] full pedal, lambda against %.2f\n", p.cal.smokeLambda);
        for (float rpm : {1000.0f, 1500.0f, 3000.0f}) {
            const double lam = measure(p, rpm, 1.0f, 5.0, [&](Simulator& sim) {
                double l = 0; for (int k = 0; k < 10; ++k) { run(sim, 0.05); l += sim.engineForTests()->cyl(0).lambda; } return l / 10; });
            std::printf("  %.0f rpm: lambda %.3f\n", rpm, lam);
            CHECK(std::fabs(lam - p.cal.smokeLambda) < 0.04, "%.0f rpm: within 0.04 of the smoke lambda (%.3f)", rpm, lam);
        }
    }
    // D7: a diesel needs its compression - too low a ratio does not light (the charge stays under the cold limit)
    {
        std::printf("[D7 compression] warm start at CR 12 and CR 16\n");
        auto runs = [&](float cr) { EngineParams q = p; q.compression = cr; Simulator sim(q); start(sim, q); run(sim, 3.0); return tel(sim).rpm; };
        const double lo = runs(12.0f), hi = runs(16.0f);
        std::printf("  CR 12: %.0f rpm, CR 16: %.0f rpm\n", lo, hi);
        CHECK(lo < 100.0, "CR 12 does not run");
        CHECK(hi > 600.0, "CR 16 runs");
    }
    // D2: common rail injector maths and the EA189-style preset
    EngineParams cr = makeCalibratedPreset(26);
    {
        std::printf("[D2 common rail] %s\n", cr.name);
        const double f1 = nozzleFlow(cr, 837.0, 400e5, 60e5), f2 = nozzleFlow(cr, 837.0, 1660e5, 60e5);
        std::printf("  full-lift flow %.1f mg/ms at 400 bar, %.1f at 1660 bar (ratio %.3f, sqrt of the pressure drops %.3f)\n", f1 * 1e3, f2 * 1e3, f2 / f1, 2.0);
        CHECK(std::fabs(f2 / f1 - std::sqrt(1600.0 / 340.0)) < 1e-6, "orifice flow goes with the square root of the pressure drop");
        bool inv = true;
        for (double q : {0.5e-6, 2e-6, 40e-6}) {
            const double dur = energizingFor(q, f2);
            double got = 0; for (double t = 0; t < 0.01; t += 1e-7) got += f2 * needleOpen(t, dur) * 1e-7;
            if (std::fabs(got / q - 1.0) > 0.01) inv = false;
            std::printf("  %.1f mg -> %.3f ms energized -> %.2f mg delivered\n", q * 1e6, dur * 1e3, got * 1e6);
        }
        CHECK(inv, "the energizing time delivers the asked quantity, ballistic pulses included (1 %%)");
        struct RailR { double rail, target, pump, tq; };
        const RailR r = measure(cr, 3500, 1.0f, 5.0, [&](Simulator& sim) {
            RailR x{}; for (int k = 0; k < 10; ++k) { run(sim, 0.05); Telemetry t = tel(sim); x.rail += t.railBar; x.target += t.railTargetBar; x.pump += t.pumpKw; x.tq += t.torque; }
            x.rail /= 10; x.target /= 10; x.pump /= 10; x.tq /= 10; return x; });
        std::printf("  3500 rpm full pedal: rail %.0f bar (target %.0f), pump %.1f kW, %.0f Nm\n", r.rail, r.target, r.pump, r.tq);
        CHECK(std::fabs(r.rail - r.target) < 0.1 * r.target, "the rail holds its target");
        CHECK(r.pump > 1.0 && r.pump < 5.0, "high-pressure pump: a few kW at full load (up to ~5 kW for such pumps)");
    }
    // D1: pilots, main and post - separate ignitions, all the fuel burns; the post's soot effect is reported
    {
        std::printf("[D1 multi-pulse] 2000 rpm, 60 %% pedal\n");
        struct M { int groups; double inj, burned, pilot2, post, fsn; };
        auto mp = [&](EngineParams q) { return measure(q, 2000, 0.6f, 5.0, [&](Simulator& sim) {
            M m{}; double f = 0;
            for (int k = 0; k < 8; ++k) { run(sim, 0.5); f += tel(sim).smokeFsn; }
            const Engine& e = *sim.engineForTests();
            m.groups = e.cyl(0).nGroup; m.inj = e.cyl(0).injDone * 1e6; m.burned = e.cyl(0).dBurned * 1e6;
            m.pilot2 = e.ecu.cmd.pilot2Mg; m.post = e.ecu.cmd.postMg; m.fsn = f / 8; return m; }); };
        const M a = mp(cr);
        EngineParams np = cr; np.cal.postMg = 0;
        const M b = mp(np);
        std::printf("  pilot2 %.1f mg, post %.1f mg: %d ignition groups, injected %.1f mg, burned %.1f mg | FSN %.2f with the post, %.2f without\n",
                    a.pilot2, a.post, a.groups, a.inj, a.burned, a.fsn, b.fsn);
        CHECK(a.pilot2 > 0 && a.post > 0, "the ECU schedules a second pilot and a post injection");
        CHECK(a.groups >= 2, "the post injection lights on its own (a group of its own)");
        CHECK(a.burned > 0.97 * a.inj, "all the injected fuel burns (post included)");
    }
    // D3 + D4: NOx and soot; EGR and its cooler
    {
        std::printf("[D3/D4 emissions] %s, 2000 rpm\n", cr.name);
        struct E { double nox, fsn, iat; };
        auto em = [&](EngineParams q, float pedal) { return measure(q, 2000, pedal, 6.0, [&](Simulator& sim) {
            E e{}; for (int k = 0; k < 8; ++k) { run(sim, 0.5); Telemetry t = tel(sim); e.nox += t.noxPpm; e.fsn += t.smokeFsn; e.iat += t.snsValue[kSnsIAT]; }
            e.nox /= 8; e.fsn /= 8; e.iat /= 8; return e; }); };
        EngineParams noEgr = cr; noEgr.egrValveDia_mm = 0;
        EngineParams hot = cr; hot.egrCooler = 0;
        const E base = em(cr, 0.3f), off = em(noEgr, 0.3f), hotE = em(hot, 0.3f), mid = em(cr, 0.6f), full = em(cr, 1.0f);
        std::printf("  30 %%: NOx %.0f ppm (cooled EGR), %.0f (hot EGR), %.0f (no EGR); intake %.0f / %.0f / %.0f C\n", base.nox, hotE.nox, off.nox, base.iat, hotE.iat, off.iat);
        std::printf("  FSN 30 %% %.2f (no EGR %.2f), 60 %% %.2f, full %.2f\n", base.fsn, off.fsn, mid.fsn, full.fsn);
        CHECK(std::fabs(sootMgM3FromFsn(1.0) - 4.95 / 0.405 * std::exp(0.38)) < 1e-9 && std::fabs(fsnFromSootMgM3(sootMgM3FromFsn(2.3)) - 2.3) < 1e-3,
              "FSN <-> soot concentration (AVL relation) and its inverse");
        CHECK(off.nox > 300 && off.nox < 4000, "NOx without EGR in the range of a DI diesel (%.0f ppm)", off.nox);
        CHECK(base.nox < 0.5 * off.nox, "EGR cuts NOx");
        CHECK(hotE.iat > base.iat + 20 && hotE.nox > base.nox, "the cooler: a cooler intake and less NOx than hot EGR");
        CHECK(base.fsn > off.fsn, "EGR raises soot (the NOx / soot trade-off)");
        CHECK(full.fsn > mid.fsn && full.fsn > 1.0, "soot rises towards the smoke limit at full load");
    }
    // D5: VGT turbine efficiency over the blade speed ratio
    {
        std::printf("[D5 VGT efficiency] preset 24, full pedal\n");
        for (float rpm : {1500.0f, 3500.0f}) {
            Telemetry t = measure(p, rpm, 1.0f, 5.0, [&](Simulator& sim) { run(sim, 0.2); return tel(sim); });
            const double want = std::clamp(0.68 - 2.5 * (t.turbUc - 0.58) * (t.turbUc - 0.58), 0.2, 0.68);
            std::printf("  %.0f rpm: BSR %.2f, efficiency %.3f\n", rpm, t.turbUc, t.turbEff);
            CHECK(std::fabs(t.turbEff - want) < 1e-3 && t.turbUc > 0.2 && t.turbUc < 1.2, "%.0f rpm: efficiency on the BSR parabola", rpm);
        }
    }
    // D8: a hot glow plug shortens the cold engine's ignition delay (post-heat on against off)
    {
        std::printf("[D8 post-heat] -5 C start, ignition delay over the first seconds\n");
        CHECK(glowPlugDelayS(0.3, 30.0, 1400.0, 51.0) >= 1.5e-3 && glowPlugDelayS(0.3, 30.0, 1100.0, 51.0) >= glowPlugDelayS(0.3, 30.0, 1400.0, 51.0),
              "glow plug path: never under 1.5 ms, no faster above 1200 K");
        auto cold = [&](bool post) {
            EngineParams q = p; q.ambientTemp_C = -5; q.coldStart = 1; q.cal.afterglowTime = post ? 180.0f : 0.0f;
            Simulator sim(q);
            q.ignition = 1; sim.setParams(q); run(sim, 6.0);
            start(sim, q, 6.0);
            double id = 0; for (int k = 0; k < 20; ++k) { run(sim, 0.1); id += tel(sim).ignDelayDeg; }
            return id / 20;
        };
        const double on = cold(true), off = cold(false);
        std::printf("  delay with post-heat %.2f deg, without %.2f deg\n", on, off);
        CHECK(on < off, "post-heat shortens the delay of the cold engine");
    }
    // D11: the new presets start, idle and make torque near their ratings
    {
        std::printf("[D11 presets]\n");
        struct P { int idx; double ratedNm; float rpm; };
        for (P x : {P{26, 320, 2250}, P{27, 597, 1750}, P{28, 1234, 2550}, P{29, 21.6f, 2800}}) {
            EngineParams q = makeCalibratedPreset(x.idx);
            Simulator sim(q);
            const double ts = start(sim, q);
            run(sim, 4.0);
            const double idle = tel(sim).rpm;
            const double tq = measure(q, x.rpm, 1.0f, 5.0, [&](Simulator& s2) { double t = 0; for (int k = 0; k < 10; ++k) { run(s2, 0.05); t += tel(s2).torque; } return t / 10; });
            std::printf("  %-48s start %.2f s, idle %.0f (target %.0f), full pedal %.0f rpm %.0f Nm (rated peak %.0f)\n", q.name, ts, idle, q.idleTargetRpm, x.rpm, tq, x.ratedNm);
            CHECK(ts >= 0 && std::fabs(idle - q.idleTargetRpm) < 150, "%s starts and idles", q.name);
            CHECK(std::fabs(tq / x.ratedNm - 1.0) < 0.2, "%s: torque within 20 %% of its rated peak", q.name);
        }
    }

    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

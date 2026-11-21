// Two-stroke probe (T1-T13 work): Blair's 125 cc racing engine on a speed-held rig with Blair's empirical pipe or the
// measured pipe "A2", the power valve, the inlet types. Prints delivery / trapping / charging and bmep against
// Blair's measured values (Design and Simulation of Two-Stroke Engines, Figs. 5.28 / 5.29).
// usage: ts_probe [mode]   mode 0 = empirical vs A2 sweep, 1 = power valve at 8500, 2 = inlet types (preset 25)
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
using namespace es;

static double gFs = 96000.0;
struct Rig { double tq, dr, te, ce, bmep, imep, lam, pmax, burn; };
static Rig rig(EngineParams p, double rpm, double thr, double settle = 1.5, double meas = 0.8) {
    const double fs = gFs, dt = 1.0 / fs;
    p.ignition = 1;
    Engine e(p, fs);
    const double w = rpm * 2.0 * kPi / 60.0;
    e.setCrank(w, 0.0);
    Rig r{};
    double tq = 0, dr = 0, te = 0, ce = 0;
    int n = 0, cyc = 0;
    double lastPhi = 0;
    for (double t = 0; t < settle + meas; t += dt) {
        e.setCrank(w, e.theta());
        e.step(dt, thr, 0.0);
        if (t < settle) continue;
        tq += e.netTorque(); ++n;
        const Cylinder& c = e.cyl(0);
        if (c.prevPhi < lastPhi) { dr += c.dr; te += c.te; ce += c.ce; r.imep += c.imep; r.lam += c.lambda; r.pmax += c.pmaxCycle; r.burn += c.burnDur; ++cyc; }
        lastPhi = c.prevPhi;
    }
    r.tq = tq / std::max(1, n);
    r.imep /= std::max(1, cyc) * 1e5; r.lam /= std::max(1, cyc); r.pmax /= std::max(1, cyc) * 1e5; r.burn /= std::max(1, cyc);
    r.dr = dr / std::max(1, cyc); r.te = te / std::max(1, cyc); r.ce = ce / std::max(1, cyc);
    CylGeom g; g.set(p);
    r.bmep = 2.0 * kPi * r.tq / (g.Vd * p.cylinders) * 1e-5;   // two-stroke: one cycle per revolution
    return r;
}

static EngineParams blair125(EngineParams b, bool a2) {
    b.bore_mm = 56; b.stroke_mm = 50.6f; b.rod_mm = 110;
    b.tsExhaustOpen = 81; b.tsTransferOpen = 115; b.tsExhaustWidth = 42.0f / 56.0f; b.tsTransferWidth = 80.4f / 56.0f;
    b.tsCrankcaseCR = 1.35f; b.tsReedArea_cm2 = 15; b.tsScavenge = 9; b.tsBurnDeg = 41;
    b.tsPipe = 1; b.tsPipeRpm = 11750; b.tsPipeClass = 1.0f; b.tsPipeCustom = a2 ? 1 : 0;
    const float L[7] = {83, 189, 209, 65, 78, 205, 250}, D[7] = {37.5f, 48.5f, 100, 116, 116, 21, 21};
    for (int j = 0; j < 7; ++j) { b.tsPipeLen_mm[j] = L[j]; b.tsPipeDia_mm[j] = D[j]; }
    b.baseAdvance = 20; b.mechAdvance = 0; b.revLimit = 14000; b.venturi_mm = 38; b.throttleBore_mm = 38;
    // power valve: fully lowered closes at 95 deg BTDC, fully lifted by 9500 rpm (Blair sec. 5.5.2)
    b.tsPowerValve = 1; b.tsPvLowOpen = 95; b.tsPvRpmLow = 8800; b.tsPvRpmHigh = 9500;
    CylGeom g; g.set(b);
    const double A = kPi * 0.25 * g.bore * g.bore, vc = A * g.sEx / 8.0;   // trapped compression ratio 9
    b.compression = (float)((A * g.stroke + vc) / vc);
    generateCalibration(b);
    return b;
}

int main(int argc, char** argv) {
    const int mode = argc > 1 ? std::atoi(argv[1]) : 0;
    if (mode == 4 && argc > 2) gFs = std::atof(argv[2]);
    const EngineParams p25 = makePreset(25);
    if (mode == 0) {
        struct M { double rpm, dr, te, ce, bmep; } meas[] = {{8780, 1.26, 0.50, 0.63, 8.1}, {9600, 1.20, 0.54, 0.66, 8.6},
            {10400, 1.13, 0.59, 0.68, 9.5}, {11200, 1.15, 0.66, 0.76, 10.2}, {11750, 1.15, 0.69, 0.79, 10.4}, {12300, 1.10, 0.72, 0.80, 9.9}};
        for (int a2 = 0; a2 < 2; ++a2) {
            const EngineParams b = blair125(p25, a2 != 0);
            std::printf("%s pipe\n", a2 ? "A2 (measured)" : "empirical");
            for (const M& m : meas) {
                const Rig r = rig(b, m.rpm, 1.0);
                std::printf("  %5.0f rpm  DR %.2f (%.2f)  TE %.2f (%.2f)  CE %.2f (%.2f)  bmep %.1f (%.1f) bar\n", m.rpm, r.dr, m.dr,
                            r.te, m.te, r.ce, m.ce, r.bmep, m.bmep);
            }
        }
    } else if (mode == 1) {
        // Blair Fig. 5.31: 8500 rpm, exhaust opening swept by the valve - optimum ~100 deg ATDC, ~8 bar
        EngineParams b = blair125(p25, true);
        b.tsPvRpmLow = b.tsPvRpmHigh = 20000;   // held fully lowered
        for (float eo : {81.0f, 85.0f, 90.0f, 95.0f, 100.0f, 105.0f, 110.0f}) {
            b.tsPvLowOpen = eo;
            const Rig r = rig(b, 8500, 1.0);
            std::printf("  valve edge opens %3.0f ATDC: bmep %.1f bar  DR %.2f TE %.2f CE %.2f\n", eo, r.bmep, r.dr, r.te, r.ce);
        }
    } else if (mode == 2) {
        for (int k = 0; k < 3; ++k) {
            EngineParams q = p25; q.tsInlet = k;
            if (k == 1) { q.tsInletOpen = 100; q.tsInletArea_cm2 = 14; }
            if (k == 2) { q.tsInletOpen = 140; q.tsInletClose = 80; q.tsInletArea_cm2 = 14; }
            std::printf("inlet %s\n", k == 0 ? "reed" : k == 1 ? "piston port" : "disc valve");
            for (double rpm : {4000.0, 6000.0, 8000.0, 9500.0}) {
                const Rig r = rig(q, rpm, 1.0);
                std::printf("  %5.0f rpm  torque %.1f Nm  DR %.2f TE %.2f CE %.2f\n", rpm, r.tq, r.dr, r.te, r.ce);
            }
        }
    }
    if (mode == 10) {   // the duct pool must not change the result: serial and parallel engines side by side, bit for bit
        EngineParams q = makeCalibratedPreset(31); q.ignition = 1; q.starter = 1;
        Engine a(q, 48000.0), b(q, 48000.0);
        b.setParallel(2);
        long diff = 0, n = 0;
        for (double t = 0; t < 3.0; t += 1.0 / 48000.0) {
            const double thr = t < 1.0 ? 0.0 : t < 2.0 ? 1.0 : 0.0;
            if (t > 0.6) { q.starter = 0; a.apply(q); b.apply(q); }
            a.step(1.0 / 48000.0, thr, 0.0); b.step(1.0 / 48000.0, thr, 0.0);
            ++n;
            if (a.netTorque() != b.netTorque() || a.rpm() != b.rpm() || a.sExhL != b.sExhL) ++diff;
        }
        std::printf("serial vs parallel: %ld of %ld steps differ (rpm %.0f / %.0f)\n", diff, n, a.rpm(), b.rpm());
    }
    if (mode == 9) {   // load over throttle blips: full throttle 1.5 s, off 1.5 s, three times - per 0.25 s window
        const int id = argc > 2 ? std::atoi(argv[2]) : 31;
        const int ncyl = argc > 3 ? std::atoi(argv[3]) : 0;
        EngineParams q = makeCalibratedPreset(id);
        if (ncyl > 0) { q.cylinders = ncyl; applyLayoutDefaults(q); }
        Simulator sim(q);
        static float buf[480 * 2];
        q.ignition = 1; q.starter = 1; sim.setParams(q);
        for (int k = 0; k < 80; ++k) sim.render(buf, 480);
        q.starter = 0; sim.setParams(q);
        for (int k = 0; k < 150; ++k) sim.render(buf, 480);
        std::printf("%s, %d cyl\n", q.name, q.cylinders);
        double worst = 0;
        for (int blip = 0; blip < 6; ++blip) {
            q.throttle = blip % 2 == 0 ? 1.0f : 0.0f; sim.setParams(q);
            for (int w = 0; w < 6; ++w) {
                const auto t0 = std::chrono::steady_clock::now();
                for (int k = 0; k < 25; ++k) sim.render(buf, 480);
                const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                Telemetry t; sim.getTelemetry(t);
                worst = std::max(worst, s / 0.25);
                std::printf("  thr %.0f  t+%.2f s: %4.0f %% load  %5.0f rpm\n", q.throttle, w * 0.25, s / 0.25 * 100.0, t.rpm);
            }
        }
        std::printf("worst window %.0f %%\n", worst * 100.0);
    }
    if (mode == 8) {   // new presets: full-throttle torque on the rig (power valve working), against the makers' figures
        const int ids[] = {25, 30, 31, 32};
        for (int id : ids) {
            const auto t0 = std::chrono::steady_clock::now();
            const EngineParams q = makeCalibratedPreset(id);
            const double cal = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("%s (calibration %.1f s)\n ", q.name, cal);
            double best = 0, bestRpm = 0, pk = 0, pkRpm = 0;
            for (double rpm = 3000; rpm <= q.revLimit - 300; rpm += 1000) {
                const Rig r = rig(q, rpm, 1.0, 1.2, 0.6);
                const double kw = r.tq * rpm * 2.0 * kPi / 60.0 * 1e-3;
                if (r.tq > best) { best = r.tq; bestRpm = rpm; }
                if (kw > pk) { pk = kw; pkRpm = rpm; }
                std::printf(" %.0f:%.1f", rpm, r.tq);
            }
            std::printf("\n  peak torque %.1f Nm @ %.0f, power %.1f hp @ %.0f\n", best, bestRpm, pk / 0.7457, pkRpm);
        }
    }
    if (mode == 7) {   // T10: real-time load - wall clock per simulated second (render with sound), 1 / 2 / 3 cylinders
        for (int nc = 1; nc <= 3; ++nc) {
            EngineParams q = makeCalibratedPreset(25); q.cylinders = nc; applyLayoutDefaults(q); if (argc > 2) q.oversample = std::atoi(argv[2]);
            Simulator sim(q);
            static float buf[480 * 2];
            q.ignition = 1; q.starter = 1; sim.setParams(q);
            for (int k = 0; k < 100; ++k) sim.render(buf, 480);
            q.starter = 0; q.throttle = 0.5f; sim.setParams(q);
            for (int k = 0; k < 100; ++k) sim.render(buf, 480);
            const auto t0 = std::chrono::steady_clock::now();
            const int blocks = 300;   // 3 s at 48 kHz
            for (int k = 0; k < blocks; ++k) sim.render(buf, 480);
            const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            Telemetry t; sim.getTelemetry(t);
            std::printf("  %d cyl: %.0f %% of real time (%.0f rpm)\n", nc, s / (blocks * 0.01) * 100.0, t.rpm);
        }
    }
    if (mode == 6) {   // T3: mid-pipe mean gas temperature vs Blair Fig. 5.30, and the bmep peak of the A2 pipe
        EngineParams b = blair125(p25, true); b.plenum_L = 3.0f; b.runnerLen_mm = 200;
        const double meas[] = {390, 435, 470, 492, 517, 550};
        const double rpms[] = {8780, 9600, 10400, 11200, 11750, 12300};
        for (int k = 0; k < 6; ++k) {
            const double fs = 96000.0, dt = 1.0 / fs, w = rpms[k] * 2.0 * kPi / 60.0;
            EngineParams q = b; q.ignition = 1;
            Engine e(q, fs);
            e.setCrank(w, 0.0);
            double tq = 0, tm = 0, twall = 0; int n = 0;
            for (double t = 0; t < 2.3; t += dt) {
                e.setCrank(w, e.theta());
                e.step(dt, 1.0, 0.0);
                if (t < 1.5) continue;
                Duct& d = e.exhaust.duct(0);
                int mid = 0; for (int i = 0; i < d.cells(); ++i) if (d.cellX(i) < 0.5 * d.length()) mid = i;
                tm += d.cellT(mid); twall += d.wallT(); tq += e.netTorque(); ++n;
            }
            CylGeom g; g.set(b);
            std::printf("  %5.0f rpm  mid-pipe %.0f C (measured %.0f)  wall %.0f C  bmep %.1f\n", rpms[k], tm / n - 273.15, meas[k],
                        twall / n - 273.15, 2.0 * kPi * tq / n / g.Vd * 1e-5);
        }
        for (double rpm : {12800.0, 13300.0, 13800.0}) { const Rig r = rig(b, rpm, 1.0); std::printf("  %5.0f rpm bmep %.1f\n", rpm, r.bmep); }
    }
    if (mode == 5) {   // preset 25 intake geometry (torque, Nm; CR250R: 51 Nm @ 8000 Honda, YZ250 41.5 Nm MXA dyno)
        const float pl[] = {0.2f, 0.2f, 1.0f, 1.0f, 3.0f}, rl[] = {120, 200, 200, 250, 200};
        for (int k = 0; k < 5; ++k) {
            EngineParams q = p25; q.plenum_L = pl[k]; q.runnerLen_mm = rl[k];
            std::printf("plenum %.1f L runner %3.0f:", pl[k], rl[k]);
            for (double rpm : {3000.0, 5000.0, 6500.0, 7500.0, 8500.0, 9200.0}) {
                const Rig r = rig(q, rpm, 1.0);
                std::printf("  %.0f: %.1f Nm DR %.2f |", rpm, r.tq, r.dr);
            }
            std::printf("\n");
        }
    }
    if (mode == 4) {   // intake geometry for the Blair rig: open bell (big air box), tract length
        for (float L : {150.0f, 200.0f, 250.0f, 300.0f, 350.0f}) {
            EngineParams b = blair125(p25, true); b.plenum_L = 3.0f; b.runnerLen_mm = L;
            std::printf("tract %3.0f mm:", L);
            for (double rpm : {8780.0, 9600.0, 10400.0, 11200.0, 11750.0, 12300.0}) {
                const Rig r = rig(b, rpm, 1.0);
                std::printf("  %.0f: DR %.2f CE %.2f bmep %.1f |", rpm, r.dr, r.ce, r.bmep);
            }
            std::printf("\n");
        }
    }
    if (mode == 3) {   // diagnostics, A2 pipe at 11750: conversion and intake sensitivity
        const EngineParams b = blair125(p25, true);
        auto show = [&](const char* what, const EngineParams& q) {
            const Rig r = rig(q, 11750, 1.0);
            std::printf("  %-28s DR %.2f CE %.2f  imep %.1f bmep %.1f  lambda %.2f  pmax %.0f bar  burn %.0f\n", what, r.dr, r.ce, r.imep,
                        r.bmep, r.lam, r.pmax, r.burn);
        };
        show("baseline", b);
        { EngineParams q = b; q.tsReedArea_cm2 = 30; show("reed area x2", q); }
        { EngineParams q = b; q.runnerLen_mm = 40; show("runner 40 mm", q); }
        { EngineParams q = b; q.runnerLen_mm = 250; show("runner 250 mm", q); }
        { EngineParams q = b; q.plenum_L = 3.0f; show("airbox 3 L", q); }
        { EngineParams q = b; q.tsTransferLen = 2.5f; show("transfer len 2.5", q); }
        { EngineParams q = b; q.tsCrankcaseCR = 1.45f; show("crankcase 1.45", q); }
    }
    return 0;
}

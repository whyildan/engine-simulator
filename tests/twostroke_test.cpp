// Package 5: two-stroke engine - piston-controlled ports, crankcase pumping through a reed valve, scavenging of
// measured cylinders, Blair's 125 cc racing engine as a check, ignition delay / burn, the expansion chamber, start /
// idle and light-load four-stroking.
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include "sim/Scavenge.h"
#include "sim/Squish.h"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace es;

static int failures = 0;
#define CHECK(c, ...) do { if (c) { std::printf("  ok:   "); std::printf(__VA_ARGS__); std::printf("\n"); } \
                           else { ++failures; std::printf("  FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// the engine held at a fixed speed (a motoring rig with the ignition on): averages over `meas` seconds
struct Rig { double tq, dr, te, se, ce, ccMin, ccMax, reedMax, reedAtPeak, imep, misShare, dilShare, lam; };
static Rig rig(EngineParams p, double rpm, double thr, double settle = 2.0, double meas = 1.0) {
    const double fs = 96000.0, dt = 1.0 / fs;
    p.ignition = 1;
    Engine e(p, fs);
    const double w = rpm * 2.0 * kPi / 60.0;
    e.setCrank(w, 0.0);
    Rig r{};
    r.ccMin = 1e9;
    double tq = 0, dr = 0, te = 0, se = 0, ce = 0, imep = 0;
    int n = 0, cyc = 0, mis0 = 0, dil0 = 0;
    double lastPhi = 0;
    for (double t = 0; t < settle + meas; t += dt) {
        e.setCrank(w, e.theta());
        e.step(dt, thr, 0.0);
        if (t < settle) { mis0 = (int)e.misfires; dil0 = e.misfireCause[Engine::kMisDilution]; continue; }
        tq += e.netTorque(); ++n;
        const double pc = e.crankcaseP(0), pp = e.intake.port(0).p();
        (void)pp;
        r.ccMin = std::min(r.ccMin, pc);
        if (pc > r.ccMax) { r.ccMax = pc; r.reedAtPeak = e.reedOpen(0); }
        r.reedMax = std::max(r.reedMax, e.reedOpen(0));
        const Cylinder& c = e.cyl(0);
        if (c.prevPhi < lastPhi) { dr += c.dr; te += c.te; se += c.se; ce += c.ce; imep += c.imep; r.lam += c.lambda; ++cyc; }
        lastPhi = c.prevPhi;
    }
    r.tq = tq / std::max(1, n); r.lam /= std::max(1, cyc);
    r.dr = dr / std::max(1, cyc); r.te = te / std::max(1, cyc); r.se = se / std::max(1, cyc); r.ce = ce / std::max(1, cyc); r.imep = imep / std::max(1, cyc) * 1e-5;
    r.misShare = ((int)e.misfires - mis0) / std::max(1.0, (double)cyc);
    r.dilShare = (e.misfireCause[Engine::kMisDilution] - dil0) / std::max(1.0, (double)((int)e.misfires - mis0));
    return r;
}

// Blair's 125 cc racing engine (Design and Simulation of Two-Stroke Engines, sec. 5.5.2, Cartwright's engine with
// pipe "A2" - its measured lengths / diameters): 56 x 50.6 mm, rod 110, trapped CR 9, crankcase 1.35, exhaust /
// transfers open 81 / 115 deg ATDC, transfers 80.4 mm wide, exhaust ~42 mm, 38 mm slide carburettor, ignition 20 deg
// BTDC, burn 41 deg; the cylinder is taken as YAM14 (Blair used it too). Its exhaust control valve closes the port at
// 95 deg BTDC when lowered and is fully lifted by 9500 rpm. The intake tract (Blair gives no dimensions): an open
// bell - a big air box - and 200 mm from it to the reed (chosen).
static EngineParams blair125(EngineParams b) {
    b.bore_mm = 56; b.stroke_mm = 50.6f; b.rod_mm = 110;
    b.tsExhaustOpen = 81; b.tsTransferOpen = 115; b.tsExhaustWidth = 42.0f / 56.0f; b.tsTransferWidth = 80.4f / 56.0f;
    b.tsCrankcaseCR = 1.35f; b.tsReedArea_cm2 = 15; b.tsScavenge = 9; b.tsBurnDeg = 41;
    b.tsPipe = 1; b.tsPipeRpm = 11750; b.tsPipeClass = 1.0f; b.tsPipeCustom = 1;
    {
        const float L[7] = {83, 189, 209, 65, 78, 205, 250}, D[7] = {37.5f, 48.5f, 100, 116, 116, 21, 21};
        for (int j = 0; j < 7; ++j) { b.tsPipeLen_mm[j] = L[j]; b.tsPipeDia_mm[j] = D[j]; }
    }
    b.tsPowerValve = 1; b.tsPvLowOpen = 95; b.tsPvRpmLow = 8800; b.tsPvRpmHigh = 9500;
    b.plenum_L = 3.0f; b.runnerLen_mm = 200; b.runnerDia_mm = 38;
    b.baseAdvance = 20; b.mechAdvance = 0; b.revLimit = 14000; b.venturi_mm = 38; b.throttleBore_mm = 38;
    CylGeom g; g.set(b);
    const double A = kPi * 0.25 * g.bore * g.bore, vc = A * g.sEx / 8.0;   // trapped compression ratio 9
    b.compression = (float)((A * g.stroke + vc) / vc);
    generateCalibration(b);
    return b;
}

static void run(Simulator& sim, double s) { static std::vector<float> buf(480 * 2); for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480); }
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }

int main() {
    const EngineParams p = makePreset(25);
    std::printf("preset: %s, %.0f cc, exhaust opens %.0f ATDC, transfers %.0f ATDC, crankcase %.2f:1\n", p.name,
                ratedDisplacementL(p) * 1000.0, p.tsExhaustOpen, p.tsTransferOpen, p.tsCrankcaseCR);

    // ---------------- ports: uncovered by the crown, symmetric about BDC, transfers inside the exhaust event
    {
        std::printf("[ports] geometry\n");
        CylGeom g; g.set(p);
        const double eo = p.tsExhaustOpen, to = p.tsTransferOpen;
        const double aBefore = g.portArea(eo - 1.0, g.sEx, g.wEx), aAfter = g.portArea(eo + 5.0, g.sEx, g.wEx);
        const double aBdc = g.portArea(180.0, g.sEx, g.wEx), aSym = g.portArea(360.0 - eo - 5.0, g.sEx, g.wEx);
        const double tBefore = g.portArea(to - 1.0, g.sTr, g.wTr), tBdc = g.portArea(180.0, g.sTr, g.wTr);
        std::printf("  exhaust: %.2f cm2 just before opening, %.2f cm2 5 deg after, %.2f cm2 at BDC (= width x port height %.2f cm2)\n",
                    aBefore * 1e4, aAfter * 1e4, aBdc * 1e4, g.wEx * (g.stroke - g.sEx) * 1e4);
        CHECK(aBefore == 0.0 && aAfter > 0.0, "the exhaust port opens at its angle");
        CHECK(std::fabs(aBdc - g.wEx * (g.stroke - g.sEx)) < 1e-9, "fully open at BDC");
        CHECK(std::fabs(aSym - aAfter) < 1e-9, "closes as far before TDC as it opened after (symmetric timing)");
        CHECK(tBefore == 0.0 && tBdc > 0.0 && g.sTr > g.sEx, "the transfer ports open later, below the exhaust port's top edge");
    }

    // ---------------- crankcase pumping and the reed valve
    {
        std::printf("[crankcase] full throttle 5500 rpm, 125 cc version (the reed data's engine: Collins 1994)\n");
        EngineParams q = p; q.bore_mm = 54; q.stroke_mm = 54.5f; q.rod_mm = 110; q.tsReedArea_cm2 = 12;
        const Rig r = rig(q, 5500, 1.0);
        std::printf("  crankcase %.0f..%.0f kPa, reed lift max %.0f %% of its 10 mm stop, delivery ratio %.2f\n", r.ccMin * 1e-3,
                    r.ccMax * 1e-3, r.reedMax * 100, r.dr);
        CHECK(r.ccMin < kPatm - 5e3 && r.ccMax > kPatm + 30e3, "the piston's underside pumps the crankcase below and well above ambient");
        CHECK(r.reedAtPeak < 0.02, "the petals are back on their seat when the crankcase pressure peaks (one way: no flow back out)");
        CHECK(r.reedMax > 0.55 && r.reedMax < 0.9, "peak petal lift near Collins' 6.3-8.2 mm (%.1f mm)", r.reedMax * 10.0);
    }

    // ---------------- scavenging: Blair's measured cylinders (QUB single-cycle rig, Scavenge.h)
    {
        std::printf("[scavenging] exit purity from the measured SE-SR curves; 5500 rpm full throttle, good vs poor cylinder\n");
        const ScavCylinder& y14 = kScavCylinders[9];
        double pu0, pu1, t0, t1;
        scavExit(y14, 0.05, 900.0, 0.9, 15.0, pu0, t0);   // burned gas still filling the cylinder
        scavExit(y14, 0.90, 500.0, 0.9, 15.0, pu1, t1);   // mostly fresh charge
        std::printf("  YAM14: SE at SR 1 = %.2f; exit purity %.2f (cylinder 5 %% fresh) -> %.2f (90 %% fresh); exit T %.0f K / %.0f K\n",
                    scavSE(y14, 1.0), pu0, pu1, t0, t1);
        CHECK(pu0 < 0.05 && pu1 > 0.5, "burned gas leaves first, the fresh charge once the cylinder is mostly fresh");
        const Rig good = rig(p, 5500, 1.0);
        EngineParams q = p; q.tsScavenge = 8;   // YAM12, Blair's example of a poor loop cylinder
        const Rig poor = rig(q, 5500, 1.0);
        std::printf("  YAM14: DR %.2f TE %.2f SE %.2f | YAM12: DR %.2f TE %.2f SE %.2f\n", good.dr, good.te, good.se, poor.dr, poor.te, poor.se);
        CHECK(good.se > 0.75 && good.se < 0.99, "full-load scavenging efficiency of a good loop cylinder (%.2f)", good.se);
        // (with the tuned pipe plugging the port both reach SE ~0.95 at this delivery: the poor one loses more of its charge)
        CHECK(good.te > poor.te && good.ce > poor.ce, "the poor cylinder traps less and charges less");
    }

    // ---------------- validation: Blair's 125 cc racing engine (sec. 5.5.2), measured scavenging (Fig. 5.29)
    {
        std::printf("[Blair 125] reed-valve racing engine with its tuned pipe, full throttle on the rig\n");
        // measured (Figs. 5.28 / 5.29): DR, TE, CE, bmep. (8780 rpm is left out: the valve's position there is not known -
        // Blair leaves it out of his own comparison too)
        const EngineParams b = blair125(p);
        struct M { double rpm, dr, te, ce, bmep; } meas[4] = {{9600, 1.20, 0.54, 0.66, 8.6}, {11200, 1.15, 0.66, 0.76, 10.2},
                                                              {11750, 1.15, 0.69, 0.79, 10.4}, {12300, 1.10, 0.72, 0.80, 9.9}};
        double eDr = 0, eTe = 0, eCe = 0, eBm = 0;
        Rig r[4];
        for (int k = 0; k < 4; ++k) {
            r[k] = rig(b, meas[k].rpm, 1.0, 1.5, 0.8);
            const double bm = 2.0 * kPi * r[k].tq / (kPi * 0.25 * 0.056 * 0.056 * 0.0506) * 1e-5;
            std::printf("  %5.0f rpm: DR %.2f (measured %.2f)  TE %.2f (%.2f)  CE %.2f (%.2f)  bmep %.1f (%.1f) bar\n", meas[k].rpm,
                        r[k].dr, meas[k].dr, r[k].te, meas[k].te, r[k].ce, meas[k].ce, bm, meas[k].bmep);
            eDr = std::max(eDr, std::fabs(r[k].dr - meas[k].dr)); eTe = std::max(eTe, std::fabs(r[k].te - meas[k].te));
            eCe = std::max(eCe, std::fabs(r[k].ce - meas[k].ce)); eBm = std::max(eBm, std::fabs(bm / meas[k].bmep - 1.0));
        }
        CHECK(eDr < 0.1, "delivery ratio within 0.1 of the measured (1D intake tract + transfer passages): worst %.2f", eDr);
        CHECK(eTe < 0.08, "trapping efficiency within 0.08: worst %.2f", eTe);
        CHECK(eCe < 0.08, "charging efficiency within 0.08: worst %.2f", eCe);
        CHECK(eBm < 0.2, "bmep within 20 %% of the dyno: worst %.0f %%", eBm * 100.0);
        CHECK(r[2].ce > r[0].ce + 0.05, "the tuned pipe raises charging toward its speed");
    }

    // ---------------- combustion: Blair's ignition delay and Vibe burn (sec. 4.3.6)
    {
        std::printf("[burn] ignition delay and burn period\n");
        auto burnRig = [](EngineParams p, double rpm, double thr, double& delay, double& b50, double& burn) {
            const double fs = 96000.0, dt = 1.0 / fs;
            p.ignition = 1;
            Engine e(p, fs);
            const double w = rpm * 2.0 * kPi / 60.0;
            e.setCrank(w, 0.0);
            double sd = 0, s50 = 0, sb = 0; int n = 0; double last = 0;
            for (double t = 0; t < 3.0; t += dt) {
                e.setCrank(w, e.theta()); e.step(dt, thr, 0.0);
                const Cylinder& c = e.cyl(0);
                if (t > 1.5 && c.prevPhi < last && !c.misfired) {
                    // 50 % burned: x = 1 - exp(-a y^(m+1)) = 0.5 at y = (ln 2 / a)^(1 / (m+1)), a 5.25, m + 1 = 2.25
                    double a50 = c.burnStart + std::pow(std::log(2.0) / 5.25, 1.0 / 2.25) * c.burnDur;
                    if (a50 > 180.0) a50 -= 360.0;
                    sd += c.sparkDelay; s50 += a50; sb += c.burnDur; ++n;
                }
                last = c.prevPhi;
            }
            delay = sd / std::max(1, n); b50 = s50 / std::max(1, n); burn = sb / std::max(1, n);
        };
        EngineParams b = blair125(p);
        double d, a50, burn;
        burnRig(b, 10350, 1.0, d, a50, burn);
        std::printf("  Blair 125, 10,350 rpm, spark 20 deg BTDC: delay %.1f deg (measured 12), burn %.0f deg (41), 50 %% burned %.1f deg ATDC (9)\n",
                    d, burn, a50);
        CHECK(std::fabs(d - 12.0) < 1.5, "ignition delay of the measured GP engine");
        CHECK(a50 > 5.0 && a50 < 11.0, "50 %% burned 5-10 deg after TDC, where Blair finds it in optimised engines");
        // LS400-sized research engine at 3000 rpm, full and light throttle: Reid's delay 13 -> 23 deg as the charge gets
        // diluted, the burn period about the same (49 / 43 / 45)
        EngineParams r = p;
        r.bore_mm = 85; r.stroke_mm = 70; r.rod_mm = 140; r.tsPipe = 0; r.primaryLen_mm[0] = 600; r.primaryDia_mm = 40;
        r.tsBurnDeg = 49; r.baseAdvance = 22; r.mechAdvance = 0;
        CylGeom g; g.set(r);
        const double A = kPi * 0.25 * g.bore * g.bore, vc = A * g.sEx / 5.62;   // trapped compression ratio 6.6 (LS400)
        r.compression = (float)((A * g.stroke + vc) / vc);
        generateCalibration(r);
        double dFull, dLight, x0, x1, bFull, bLight;
        burnRig(r, 3000, 1.0, dFull, x0, bFull);
        burnRig(r, 3000, 0.1, dLight, x1, bLight);
        std::printf("  400 cc, 3000 rpm: delay %.1f deg full throttle, %.1f deg light (Reid 13 / 23); burn %.0f / %.0f deg (49 / 45)\n",
                    dFull, dLight, bFull, bLight);
        CHECK(dFull > 8.0 && dFull < 15.0 && dLight > dFull + 6.0, "the delay grows with the diluted light-load charge, as measured");
    }

    // ---------------- the expansion chamber
    {
        std::printf("[expansion chamber] full-throttle torque with the tuned pipe / a plain header pipe\n");
        EngineParams plain = p; plain.tsPipe = 0; plain.primaryLen_mm[0] = 900;
        double best = 0, bestRpm = 0, gain = 0;
        for (double rpm : {6000.0, 7000.0, 8000.0}) {
            const Rig a = rig(p, rpm, 1.0), b = rig(plain, rpm, 1.0);
            std::printf("  %5.0f rpm: tuned pipe %5.1f Nm (TE %.2f)  plain pipe %5.1f Nm (TE %.2f)\n", rpm, a.tq, a.te, b.tq, b.te);
            gain += a.tq - b.tq;
            if (a.tq > best) { best = a.tq; bestRpm = rpm; }
        }
        CHECK(gain > 0, "around its tuned speed the expansion chamber makes more torque than a plain pipe");
        // the speed of the torque peak follows the pipe's tuning
        double peak[2] = {}, peakRpm[2] = {};
        const float tune[2] = {5500.f, 8000.f};
        for (int k = 0; k < 2; ++k) {
            EngineParams q = p; q.tsPipeRpm = tune[k];
            for (double rpm = 5000; rpm <= 9000; rpm += 500) {
                const Rig a = rig(q, rpm, 1.0, 1.5, 0.7);
                if (a.tq > peak[k]) { peak[k] = a.tq; peakRpm[k] = rpm; }
            }
            std::printf("  pipe tuned for %.0f rpm: torque peak %.1f Nm at %.0f rpm\n", tune[k], peak[k], peakRpm[k]);
        }
        CHECK(peakRpm[0] < peakRpm[1], "a pipe tuned lower (longer) moves the torque peak down");
    }

    // ---------------- power valve (T6): Blair's 125 at 8500 rpm, the valve's timing swept (Fig. 5.31: best ~100 ATDC)
    {
        std::printf("[power valve] Blair 125 at 8500 rpm, valve held lowered at each timing\n");
        EngineParams b = blair125(p);
        b.tsPvRpmLow = b.tsPvRpmHigh = 20000;   // held fully lowered
        double best = 0, bestEo = 0, raised = 0;
        for (float eo : {81.0f, 90.0f, 95.0f, 100.0f, 105.0f, 110.0f}) {
            b.tsPvLowOpen = eo;
            const Rig r = rig(b, 8500, 1.0, 1.5, 0.6);
            std::printf("  opens %3.0f ATDC: %.1f Nm  TE %.2f\n", eo, r.tq, r.te);
            if (eo == 81.0f) raised = r.tq;
            if (r.tq > best) { best = r.tq; bestEo = eo; }
        }
        CHECK(bestEo >= 95.0f && bestEo <= 105.0f, "best timing at 8500 rpm near Blair's 100 deg ATDC (%.0f)", bestEo);
        CHECK(best > 1.2 * raised, "lowering the valve gains torque below the pipe's speed (+%.0f %%)", (best / raised - 1.0) * 100.0);
        Engine e(blair125(p), 96000.0);
        CHECK(e.powerValve() < 0.01, "the servo holds it lowered at rest");
    }

    // ---------------- inlet: piston port and disc valve (T8)
    {
        std::printf("[inlet] preset 25 with a piston port / a disc valve, full throttle\n");
        for (int k = 1; k <= 2; ++k) {
            EngineParams q = p; q.tsInlet = k; q.tsInletArea_cm2 = 14; q.ignition = 1;
            if (k == 1) q.tsInletOpen = 100; else { q.tsInletOpen = 140; q.tsInletClose = 80; }
            Engine e(q, 96000.0);
            const double w = 7000.0 * 2.0 * kPi / 60.0;
            e.setCrank(w, 0.0);
            double tq = 0, back = 0, open = 0; int n = 0;
            for (double t = 0; t < 2.0; t += 1.0 / 96000.0) {
                e.setCrank(w, e.theta()); e.step(1.0 / 96000.0, 1.0, 0.0);
                if (t < 1.2) continue;
                tq += e.netTorque(); ++n;
                if (e.intake.duct(0).cellU(0) > 5.0 && e.inletOpen(0) > 0.05) back += 1.0;   // gas leaving the crankcase
                open += e.inletOpen(0) > 0 ? 1.0 : 0.0;
            }
            std::printf("  %s: %.1f Nm at 7000 rpm, port open %.0f %% of the time, flowing back out %.0f %%\n",
                        k == 1 ? "piston port" : "disc valve", tq / n, open / n * 100.0, back / n * 100.0);
            CHECK(tq / n > 15.0, "%s engine runs and pulls", k == 1 ? "piston-port" : "disc-valve");
            if (k == 1) CHECK(std::fabs(open / n - 200.0 / 360.0) < 0.02, "piston port open 200 deg of 360 (100 BTDC to 100 ATDC)");
            if (k == 1) CHECK(back > 0, "the crankcase spits charge back into the intake through the open port");
            if (k == 2) CHECK(std::fabs(open / n - 220.0 / 360.0) < 0.02, "disc valve open 220 deg (140 BTDC to 80 ATDC)");
        }
    }

    // ---------------- squish (T5): Blair's Prog.4.1 example
    {
        std::printf("[squish] Blair Fig. 4.14: 70 x 70 x 125, 5000 rpm, EO 104, trapped CR 7, Csq 0.5, 1.2 mm\n");
        const SquishResult q = squishVelocity(0.070, 0.070, 0.125, 104.0, 7.0, 0.5, 0.0012, 5000.0);
        std::printf("  max %.1f m/s at %.1f deg BTDC, kinetic energy %.2f mJ (Blair 20.9 / 10 / 6.94)\n", q.vMax, q.degBtdc, q.keJ * 1e3);
        CHECK(std::fabs(q.vMax - 20.9) < 0.6 && std::fabs(q.degBtdc - 10.0) < 1.5 && std::fabs(q.keJ * 1e3 - 6.94) < 0.3,
              "squish velocity, its angle and the squished energy as Blair's program");
        const double bTight = squishBurnDeg(0.056, 0.0506, 0.110, 81.0, 9.0, 0.6, 0.0008), bOpen = squishBurnDeg(0.056, 0.0506, 0.110, 81.0, 9.0, 0.3, 0.002);
        std::printf("  burn period from squish, 125 GP: tight band %.0f deg, open band %.0f deg (measured 41 with its own band)\n", bTight, bOpen);
        CHECK(bTight < bOpen, "a tighter, larger squish band burns faster");
    }

    // ---------------- premix oil (T7)
    {
        std::printf("[oil] the same carburettor on 20:1 and 50:1 premix, 7000 rpm full throttle\n");
        double lam[2], smoke[2];
        const float ratio[2] = {20.0f, 50.0f};
        for (int k = 0; k < 2; ++k) {
            EngineParams q = p; q.tsOilRatio = ratio[k];
            const Rig r = rig(q, 7000, 1.0, 1.2, 0.6);
            Engine e(q, 96000.0);
            lam[k] = r.lam; smoke[k] = premixOilShare(ratio[k], 0.745) * (1.0 - kOilBurned);
            std::printf("  %2.0f:1  trapped lambda %.3f, oil %.1f %% of the liquid by mass\n", ratio[k], r.lam, premixOilShare(ratio[k], 0.745) * 100.0);
        }
        CHECK(lam[0] > lam[1] + 0.01, "more oil in the fuel runs leaner (it displaces gasoline in the jets)");
        CHECK(smoke[0] > 2.0 * smoke[1], "and puts out more unburned oil");
    }

    // ---------------- tuned pipe gas temperature (T3): Blair 125 mid-pipe, measured (Fig. 5.30)
    {
        std::printf("[pipe gas] Blair 125 at 11,750 rpm: mean gas temperature at the pipe's mid-section\n");
        const double fs = 96000.0, dt = 1.0 / fs, w = 11750.0 * 2.0 * kPi / 60.0;
        EngineParams q = blair125(p); q.ignition = 1;
        Engine e(q, fs);
        e.setCrank(w, 0.0);
        double tm = 0; int n = 0;
        for (double t = 0; t < 2.3; t += dt) {
            e.setCrank(w, e.theta()); e.step(dt, 1.0, 0.0);
            if (t < 1.5) continue;
            Duct& d = e.exhaust.duct(0);
            int mid = 0; for (int i = 0; i < d.cells(); ++i) if (d.cellX(i) < 0.5 * d.length()) mid = i;
            tm += d.cellT(mid); ++n;
        }
        std::printf("  %.0f C (measured 517, Blair's own simulation ~540)\n", tm / n - 273.15);
        CHECK(std::fabs(tm / n - 273.15 - 517.0) < 100.0, "within 100 K of the thermocouple");
    }

    // ---------------- light load: four-stroking (residual feedback misfires - SAE 830091), not at full load
    {
        std::printf("[four-stroking] 3000 rpm: a crack of throttle (4.5 %%) vs full throttle\n");
        const Rig light = rig(p, 3000, 0.045, 2.0, 2.0), full = rig(p, 3000, 1.0, 2.0, 2.0);
        std::printf("  4.5 %% throttle: DR %.2f misfires %.0f %% of cycles (%.0f %% of them on residual) | full throttle: DR %.2f SE %.2f misfires %.0f %%\n",
                    light.dr, light.misShare * 100, light.dilShare * 100, full.dr, full.se, full.misShare * 100);
        CHECK(light.misShare > 0.2 && light.misShare < 0.8 && light.dilShare > 0.8,
              "light load: misfires on the residual and fires again on the fresh charge it leaves (four-stroking)");
        CHECK(full.misShare < 0.05, "full load: fires every cycle");
    }

    // ---------------- start and idle
    {
        std::printf("[start] warm kick start, idle\n");
        EngineParams q = makeCalibratedPreset(25);
        Simulator sim(q);
        q.ignition = 1; q.starter = 1; sim.setParams(q);
        double t0 = 0;
        for (; t0 < 4.0; t0 += 0.01) { run(sim, 0.01); Telemetry x = tel(sim); if (x.running && x.rpm > starterReleaseRpm(q, x.snsValue[kSnsECT])) break; }
        run(sim, 0.15); q.starter = 0; sim.setParams(q);
        run(sim, 4.0);
        double rpm = 0; int n = 0;
        for (int i = 0; i < 100; ++i) { run(sim, 0.02); rpm += tel(sim).rpm; ++n; }
        rpm /= n;
        std::printf("  started after %.2f s, idles at %.0f rpm\n", t0, rpm);
        CHECK(t0 < 3.0, "a warm engine starts within 3 s");
        CHECK(rpm > 1000 && rpm < 2500, "it idles on its own");
    }

    // ---------------- the multi-cylinder / EFI presets (T11, T13): start, idle, full-throttle torque
    {
        std::printf("[presets] start, idle and torque against the makers' / dyno figures\n");
        struct T { int id; double rpm, nm; const char* src; } tg[3] = {{30, 8000, 40.2, "RD350F2 spec"}, {31, 7000, 56.0, "H1 Moto-Data"},
                                                                       {32, 7500, 43.7, "300 SX rear-wheel dyno"}};
        for (const T& t : tg) {
            EngineParams q = makeCalibratedPreset(t.id);
            Simulator sim(q);
            q.ignition = 1; q.starter = 1; sim.setParams(q);
            double t0 = 0;
            for (; t0 < 4.0; t0 += 0.01) { run(sim, 0.01); Telemetry x = tel(sim); if (x.running && x.rpm > starterReleaseRpm(q, x.snsValue[kSnsECT])) break; }
            run(sim, 0.15); q.starter = 0; sim.setParams(q);
            run(sim, 4.0);
            double rpm = 0;
            for (int i = 0; i < 50; ++i) { run(sim, 0.02); rpm += tel(sim).rpm / 50.0; }
            const Rig r = rig(q, t.rpm, 1.0, 1.2, 0.6);
            std::printf("  %s: started in %.2f s, idles at %.0f rpm, %.1f Nm at %.0f rpm (%s %.1f)\n", q.name, t0, rpm, r.tq, t.rpm, t.src, t.nm);
            CHECK(t0 < 3.0 && rpm > 900 && rpm < 2600, "%s starts and idles", q.name);
            CHECK(std::fabs(r.tq / t.nm - 1.0) < 0.3, "%s torque within 30 %% of its figure", q.name);
        }
    }

    // ---------------- parallel ducts (DuctPool): the multi-cylinder engine steps its pipes on worker threads - the result
    //                  must be the serial one, bit for bit
    {
        std::printf("[parallel] triple, serial vs ducts on 2 worker threads, 3 s of start / blip / overrun\n");
        EngineParams q = makeCalibratedPreset(31); q.ignition = 1; q.starter = 1;
        Engine a(q, 48000.0), b(q, 48000.0);
        b.setParallel(2);
        long diff = 0, n = 0;
        for (double t = 0; t < 3.0; t += 1.0 / 48000.0) {
            const double thr = t < 1.0 ? 0.0 : t < 2.0 ? 1.0 : 0.0;
            if (t > 0.6 && q.starter) { q.starter = 0; a.apply(q); b.apply(q); }
            a.step(1.0 / 48000.0, thr, 0.0); b.step(1.0 / 48000.0, thr, 0.0);
            ++n;
            if (a.netTorque() != b.netTorque() || a.rpm() != b.rpm() || a.sExhL != b.sExhL) ++diff;
        }
        std::printf("  %ld of %ld steps differ, %.0f rpm at the end\n", diff, n, a.rpm());
        CHECK(diff == 0 && a.rpm() > 1000, "identical to the serial engine");
    }

    // ---------------- at rest: no sound (the tuned pipe's gas dynamics must not ring by themselves)
    {
        std::printf("[rest] output at 0 rpm: never started, and after the ignition is switched off\n");
        auto rms = [](Simulator& sim, double secs) {
            std::vector<float> buf(480 * 2);
            double s = 0; long n = 0;
            for (double t = 0; t < secs; t += 0.01) {
                sim.render(buf.data(), 480);
                if (t > secs - 0.5) for (int i = 0; i < 480; ++i) { s += (double)buf[2 * i] * buf[2 * i]; ++n; }
            }
            return std::sqrt(s / std::max(1L, n));
        };
        EngineParams q = makeCalibratedPreset(25);
        Simulator cold(q);
        const double r0 = rms(cold, 2.0);
        Simulator sim(q);
        q.ignition = 1; q.starter = 1; sim.setParams(q);
        run(sim, 1.5); q.starter = 0; sim.setParams(q); run(sim, 2.0);
        q.ignition = 0; sim.setParams(q);
        const double r1 = rms(sim, 6.0);
        std::printf("  never started: rms %.5f | 6 s after switching off: rms %.5f\n", r0, r1);
        CHECK(r0 < 1e-3 && r1 < 1e-3, "silent at rest");
    }

    std::printf(failures ? "%d FAILED\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

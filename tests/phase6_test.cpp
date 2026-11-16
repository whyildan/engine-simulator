// Phase 6: compressor / turbine maps, valvetrain dynamics, ambient pressure (altitude), intercooler heat soak,
// dyno step test and correction factors.
// usage: phase6_test
#include "sim/Simulator.h"
#include "sim/TurboMap.h"
#include "sim/Valvetrain.h"
#include "ecu/BaseMap.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
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
static void start(Simulator& sim, EngineParams& p) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 400; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, 1.5);
}
// WOT torque held at rpm on the dyno
static double wotTorque(Simulator& sim, EngineParams& p, double rpm, double settle = 2.0, Telemetry* last = nullptr) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = 1; sim.setParams(p);
    run(sim, settle);
    float fl = 0;   // float severity swings cycle to cycle once the valves float: keep the worst seen
    const double tq = avg(sim, 1.0, [&fl](const Telemetry& t) { fl = std::max(fl, std::max(t.floatIn, t.floatEx)); return t.torque; });
    if (last) { *last = tel(sim); last->floatIn = fl; }
    return tq;
}

int main() {
    const std::string tmp = std::getenv("TEMP") ? std::getenv("TEMP") : ".";

    // ---------------- compressor map tables
    {
        std::printf("[maps] table evaluation, generic equivalence, CSV round trip\n");
        CompressorModel g;
        CHECK(g.build(genericCompressorMap(220, 190000)), "generic map builds");
        // the analytic model at n = 0.7, PR 2.0: flow = mch sqrt(1 - x)
        const double n = 0.7, pr0 = 1.0 + 2.6 * n * n, mch = 0.22 * (0.12 + 0.88 * n);
        const double ref = mch * std::sqrt(1.0 - (2.0 - 1.0) / (pr0 - 1.0));
        const CompPoint c = g.eval(n * 190000, 2.0);
        CHECK(std::fabs(c.flow - ref) < 0.04 * ref, "generic table flow %.4f kg/s vs analytic %.4f", c.flow, ref);
        CHECK(g.eval(n * 190000, pr0 + 0.3).surge, "past the top of the speed line -> surge");
        CHECK(g.eval(n * 190000, 1.0).choke, "pressure ratio 1 -> choke");
        for (int i = 0; i < builtinCompressorCount(); ++i) {
            CompressorModel m;
            const bool ok = m.build(builtinCompressor(i));
            CHECK(ok && m.peakEff() > 0.74 && m.peakEff() < 0.82, "built-in '%s': %.0fk rpm, %.0f g/s, peak eff %.2f", builtinCompressorName(i),
                  m.maxSpeed() / 1000, m.maxFlow() * 1000, m.peakEff());
        }
        // flow rises with speed at a fixed pressure ratio, falls with pressure ratio at a fixed speed
        CompressorModel m; m.build(builtinCompressor(1));
        const double f1 = m.eval(120000, 1.8).flow, f2 = m.eval(140000, 1.8).flow, f3 = m.eval(140000, 2.2).flow;
        CHECK(f2 > f1 && f3 < f2, "flow 120k %.3f < 140k %.3f, and lower at PR 2.2 (%.3f)", f1, f2, f3);
        // CSV round trip
        const std::string path = tmp + "\\es_comp_map.csv";
        CHECK(saveCompressorCsv(path, builtinCompressor(1), "Medium 52 mm"), "compressor CSV saved");
        CompressorMap back; std::string name, err;
        const bool ld = loadCompressorCsv(path, back, name, err);
        CHECK(ld, "compressor CSV loaded (%s)", ld ? name.c_str() : err.c_str());
        CompressorModel mb; mb.build(back);
        const CompPoint a = m.eval(150000, 2.4), b = mb.eval(150000, 2.4);
        CHECK(std::fabs(a.flow - b.flow) < 1e-3 && std::fabs(a.eff - b.eff) < 1e-3, "same operating point after the round trip (%.4f / %.4f)", a.flow, b.flow);
        const std::string tp = tmp + "\\es_turb_map.csv";
        CHECK(saveTurbineCsv(tp, builtinTurbine(1), "Medium turbine"), "turbine CSV saved");
        TurbineMap tb; std::string tn;
        const bool tl = loadTurbineCsv(tp, tb, tn, err);
        TurbineModel t0, t1; t0.build(builtinTurbine(1)); t1.build(tb);
        CHECK(tl && std::fabs(t0.mfp(2.0) - t1.mfp(2.0)) < 1e-5 && std::fabs(t0.eff(0.6) - t1.eff(0.6)) < 1e-3 && t1.wheelDia() == t0.wheelDia(),
              "turbine CSV round trip: mfp(2.0) %.5f, eff(0.6) %.3f, wheel %.0f mm", t1.mfp(2.0), t1.eff(0.6), t1.wheelDia());
        CompressorMap bad; std::string bn;
        const std::string bp = tmp + "\\es_bad_map.csv";
        { FILE* f = std::fopen(bp.c_str(), "w"); std::fprintf(f, "speed,flow,pr,eff\n100000,0.1,1.5,0.7\n"); std::fclose(f); }
        CHECK(!loadCompressorCsv(bp, bad, bn, err), "a single speed line is rejected (%s)", err.c_str());
    }

    // ---------------- valvetrain dynamics
    {
        std::printf("[valvetrain] spring-mass valve: follows the cam, then lofts and bounces\n");
        EngineParams p = makePreset(0);
        const ValvetrainSpec vt = valvetrainSpec(p, true);
        const double lift = p.intakeLift_mm * 1e-3, dur = p.intakeDuration;
        ValveDynamics d;
        d.compute(vt, lift, dur, 4000);
        double mx = 0;
        for (double a = -dur / 2; a < dur / 2 + 40; a += 1) mx = std::max(mx, std::fabs(d.dev(a)));
        CHECK(mx < 0.01 * lift && d.severity(lift) == 0, "4000 rpm: valve follows the cam (largest deviation %.3f mm)", mx * 1e3);
        const double fr = dynamicFloatRpm(vt, lift, dur);
        ComponentModel cm; cm.build(p);
        CHECK(fr > p.revLimit && std::fabs(fr - cm.floatRpmIn) < 0.25 * cm.floatRpmIn,
              "stock springs float at %.0f rpm (static estimate %.0f, limiter %.0f)", fr, cm.floatRpmIn, p.revLimit);
        d.compute(vt, lift, dur, fr * 1.15);
        CHECK(d.loft > 0.05e-3 && d.severity(lift) > 0.2, "15%% past it: loft %.2f mm, bounce %.2f mm, late %.0f deg, severity %.2f",
              d.loft * 1e3, d.bounce * 1e3, d.lateClose, d.severity(lift));
        // weak springs: float well below the limiter
        EngineParams w = p; w.comp.springOpen_N *= 0.55f; w.comp.springSeat_N *= 0.55f;
        const double fw = dynamicFloatRpm(valvetrainSpec(w, true), lift, dur);
        CHECK(fw < 0.85 * fr, "springs at 55%% float at %.0f rpm", fw);
        // a soft pushrod train floats earlier than a stiff bucket with the same springs and mass
        ValvetrainSpec soft = vt; soft.stiffness = 2.2e6;
        const double fs = dynamicFloatRpm(soft, lift, dur);
        CHECK(fs <= fr, "soft train (2.2 kN/mm) floats at %.0f rpm vs %.0f", fs, fr);
    }
    {
        std::printf("[valvetrain] in the engine: weak springs lose power at high rpm\n");
        EngineParams p = makeCalibratedPreset(0);
        Simulator s0(p); EngineParams p0 = p; start(s0, p0);
        Telemetry t0;
        const double tq0 = wotTorque(s0, p0, 7800, 2.0, &t0);
        EngineParams w = makeCalibratedPreset(0);
        w.comp.springOpen_N *= 0.5f; w.comp.springSeat_N *= 0.5f;   // the high cam floats from ~7400 rpm
        Simulator s1(w); start(s1, w);
        Telemetry t1;
        const double tq1 = wotTorque(s1, w, 7800, 2.0, &t1);
        std::printf("  stock: %.1f Nm, float %.2f, loft %.2f mm | weak: %.1f Nm, float %.2f, loft %.2f mm, bounce %.2f mm, late %.0f deg\n",
                    tq0, t0.floatIn, t0.loftIn, tq1, t1.floatIn, t1.loftIn, t1.bounceIn, t1.lateCloseIn);
        CHECK(t0.floatIn < 0.05 && t1.floatIn > 0.3, "float severity stock %.2f, weak springs %.2f", t0.floatIn, t1.floatIn);
        CHECK(tq1 < 0.93 * tq0, "torque at 7800 rpm: %.1f -> %.1f Nm", tq0, tq1);
    }

    // ---------------- altitude
    {
        std::printf("[altitude] 2000 m (%.1f kPa): NA loses power, the turbo makes it back\n", baroAtAltitude(2000));
        EngineParams a = makeCalibratedPreset(0);
        Simulator sa(a); start(sa, a);
        const double na0 = wotTorque(sa, a, 5000);
        EngineParams b = makeCalibratedPreset(0);
        b.baro_kPa = (float)baroAtAltitude(2000);
        Calibration c = b.cal;
        if (!loadCachedBaseMap(b, c)) { measureBaseMap(b, c); saveCachedBaseMap(b, c); }
        b.cal.veMap = c.veMap; b.cal.veTps = c.veTps;
        Simulator sb(b); start(sb, b);
        Telemetry tb;
        const double na1 = wotTorque(sb, b, 5000, 2.0, &tb);
        const double ratio = na1 / na0;
        CHECK(ratio > 0.70 && ratio < 0.86, "NA WOT 5000 rpm: %.1f -> %.1f Nm (x%.2f, air density x%.2f), AFR %.1f", na0, na1, ratio,
              baroAtAltitude(2000) / 101.325, tb.afr);
        CHECK(std::fabs(tb.mapKPa - tb.baroKPa) < 8, "MAP at WOT %.1f kPa near the baro %.1f", tb.mapKPa, tb.baroKPa);

        EngineParams t = makeCalibratedPreset(16);
        Simulator st(t); start(st, t);
        Telemetry tt0;
        const double tq0 = wotTorque(st, t, 4500, 4.0, &tt0);
        EngineParams u = makeCalibratedPreset(16);
        u.baro_kPa = (float)baroAtAltitude(2000);
        Calibration cu = u.cal;
        if (!loadCachedBaseMap(u, cu)) { measureBaseMap(u, cu); saveCachedBaseMap(u, cu); }
        u.cal.veMap = cu.veMap; u.cal.veTps = cu.veTps;
        Simulator su(u); start(su, u);
        Telemetry tt1;
        const double tq1 = wotTorque(su, u, 4500, 4.0, &tt1);
        CHECK(tq1 / tq0 > ratio + 0.05, "turbo WOT 4500 rpm: %.1f -> %.1f Nm (x%.2f), turbo %.0fk -> %.0fk rpm, boost %.2f -> %.2f bar",
              tq0, tq1, tq1 / tq0, tt0.shaftRpm / 1000, tt1.shaftRpm / 1000, tt0.boostBar, tt1.boostBar);
    }

    // ---------------- turbo maps in the engine
    {
        std::printf("[turbo maps] I4 turbo on built-in compressor / turbine maps\n");
        EngineParams p = makeCalibratedPreset(16);
        Simulator s0(p); start(s0, p);
        Telemetry g;
        const double tqG = wotTorque(s0, p, 5000, 4.0, &g);
        EngineParams m = makeCalibratedPreset(16);
        m.turbo.useMaps = 1; m.turbo.compChoice = 1; m.turbo.turbChoice = 1;
        m.turbo.comp = builtinCompressor(1); m.turbo.turb = builtinTurbine(1);
        Calibration c = m.cal;
        if (!loadCachedBaseMap(m, c)) { measureBaseMap(m, c); saveCachedBaseMap(m, c); }
        m.cal.veMap = c.veMap; m.cal.veTps = c.veTps;
        Simulator s1(m); start(s1, m);
        Telemetry t;
        const double tqM = wotTorque(s1, m, 5000, 4.0, &t);
        std::printf("  generic: %.1f Nm, boost %.2f, %.0fk rpm | map: %.1f Nm, boost %.2f bar, %.0fk rpm, comp eff %.2f, turb eff %.2f, U/C %.2f, ER %.2f\n",
                    tqG, g.boostBar, g.shaftRpm / 1000, tqM, t.boostBar, t.shaftRpm / 1000, t.compEff, t.turbEff, t.turbUc, t.turbEr);
        CHECK(t.mapComp && t.mapTurb, "both maps in use");
        CHECK(t.boostBar > 0.6 * m.wastegateBar, "boost builds on the maps: %.2f bar (wastegate %.2f)", t.boostBar, m.wastegateBar);
        CHECK(t.compEff > 0.55 && t.compEff < 0.80 && t.turbEff > 0.4 && t.turbEff < 0.80, "efficiencies from the maps: compressor %.2f, turbine %.2f", t.compEff, t.turbEff);
        CHECK(tqM > 0.7 * tqG && tqM < 1.3 * tqG, "torque in the same range as the generic model (%.0f vs %.0f Nm)", tqM, tqG);
    }

    // ---------------- intercooler heat soak
    {
        std::printf("[intercooler] WOT on the dyno with and without the cell fan\n");
        double charge[2], core[2];
        for (int k = 0; k < 2; ++k) {
            EngineParams p = makeCalibratedPreset(16);
            p.coolingFan_kmh = k == 0 ? 60.0f : 0.0f;
            Simulator s(p); start(s, p);
            wotTorque(s, p, 4500, 12.0);
            Telemetry t = tel(s);
            charge[k] = t.chargeT; core[k] = t.icCoreC;
            std::printf("  fan %2.0f km/h: charge %.1f C, core %.1f C, boost %.2f bar\n", p.coolingFan_kmh, t.chargeT, t.icCoreC, t.boostBar);
        }
        CHECK(core[1] > core[0] + 8 && charge[1] > charge[0] + 4, "without the fan the core soaks: core %.0f vs %.0f C, charge %.0f vs %.0f C",
              core[1], core[0], charge[1], charge[0]);
    }

    // ---------------- dyno: step test and correction
    {
        std::printf("[dyno] step test and correction factors\n");
        CHECK(std::fabs(dynoCorrectionFactor(DynoCorrection::SaeJ1349, 25, 99) - 1.0) < 1e-3 &&
              std::fabs(dynoCorrectionFactor(DynoCorrection::Din70020, 20, 101.3) - 1.0) < 1e-3 &&
              std::fabs(dynoCorrectionFactor(DynoCorrection::Iso1585, 25, 99) - 1.0) < 1e-3, "factors are 1 at their reference air");
        const double sea = dynoCorrectionFactor(DynoCorrection::SaeJ1349, 25, 101.325), hot = dynoCorrectionFactor(DynoCorrection::SaeJ1349, 35, 99),
                     high = dynoCorrectionFactor(DynoCorrection::SaeJ1349, 25, 80);
        CHECK(sea < 1.0 && hot > 1.0 && high > 1.2, "SAE J1349: 25 C at sea level x%.3f, 35 C x%.3f, 25 C at 80 kPa x%.3f", sea, hot, high);
        EngineParams p = makeCalibratedPreset(0);
        Simulator sim(p); start(sim, p);
        p.dynoMode = DynoMode::Step; p.sweepStartRpm = 3000; p.sweepEndRpm = 6000; p.stepRpm = 1000; p.stepHold_s = 1.0f;
        p.sweepRequest++; sim.setParams(p);
        for (int k = 0; k < 400 && tel(sim).dynoState != 4; ++k) run(sim, 0.1);
        std::vector<DynoPoint> pts; int runId = 0;
        sim.takeDynoPoints(pts, runId);
        CHECK(tel(sim).dynoState == 4 && pts.size() == 4, "step test finished with %zu points", pts.size());
        for (const DynoPoint& d : pts)
            std::printf("    %5.0f rpm  %6.1f Nm  %5.1f kW  AFR %.1f  adv %.1f  EGT %.0f C  BSFC %.0f g/kWh\n", d.rpm, d.torque, d.powerKw, d.afr, d.advance, d.egtC, d.bsfc);
        bool sane = !pts.empty();
        for (const DynoPoint& d : pts) sane = sane && d.torque > 100 && d.bsfc > 200 && d.bsfc < 450 && d.egtC > 500;
        CHECK(sane, "every point has torque, BSFC 200..450 g/kWh and EGT");
    }

    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}

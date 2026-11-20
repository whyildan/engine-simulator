// Package 1: fuels and flex fuel. Fuel properties and blending, knock by octane, ethanol / methanol with a flex
// fuel sensor (the ECU adapts) and without one (it does not), refuelling a blend, cold starts, a carburettor on
// E85, charge cooling.
// usage: fuel_test
#include "sim/Simulator.h"
#include "sim/Fuel.h"
#include "ecu/BaseMap.h"
#include "ecu/CalibrationIO.h"
#include "ecu/EngineFile.h"
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
// crank (at most `crankS` seconds) until it runs on its own; returns the time to start, -1 = no start
static double start(Simulator& sim, EngineParams& p, double crankS = 4.0, double idle = 1.5) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    double t = 0, first = -1;
    for (; t < crankS; t += 0.01) {
        run(sim, 0.01);
        Telemetry x = tel(sim);
        if (x.running && x.rpm > starterReleaseRpm(p, x.snsValue[kSnsECT])) { first = t; break; }
    }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, idle);
    return tel(sim).running ? first : -1;
}
struct Wot { double tq, lam, knock, retard, ve, duty; };
static Wot wot(Simulator& sim, EngineParams& p, double rpm, double settle) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = 1; sim.setParams(p);
    run(sim, settle);
    Wot w{};
    w.tq = avg(sim, 1.5, [](const Telemetry& t) { return t.torque; });
    Telemetry x = tel(sim);
    w.lam = x.lambda; w.knock = x.knock; w.ve = x.ve * 100.0; w.duty = x.injDuty * 100.0;
    for (int i = 0; i < x.cylinders; ++i) w.retard = std::max(w.retard, (double)x.knockRetard[i]);
    return w;
}
// a preset filled with another fuel (the calibration stays the one made for premium unless changed here)
struct Opt { bool flex = true, learn = true; double injScale = 1.0; float alcStoich = 0; };
static void print(const char* tag, const Wot& w) {
    std::printf("    %-32s torque %6.1f Nm  lambda %.3f  knock %.2f  retard %4.1f  VE %5.1f  inj duty %3.0f %%\n", tag, w.tq, w.lam, w.knock,
                w.retard, w.ve, w.duty);
}
static Wot runFuel(int preset, int fuel, double rpm, const char* tag, Opt o = Opt()) {
    EngineParams p = makeCalibratedPreset(preset);
    p.fuelType = fuel; p.flexSensor = o.flex ? 1 : 0;
    if (!o.learn) { p.cal.autoTune = 0; p.cal.closedLoop = 0; }
    p.injector_ccmin *= (float)o.injScale; p.cal.injFlow *= (float)o.injScale;   // bigger injectors, ECU told so
    if (o.alcStoich > 0) p.cal.stoichAlcohol = o.alcStoich;
    Simulator sim(p);
    start(sim, p);
    Wot w = wot(sim, p, rpm, 5.0);
    print(tag, w);
    return w;
}

int main() {
    {
        std::printf("[properties] pure fuels and blends\n");
        auto P = [](int t) { return fuelProps(fuelTypeBlend(t)); };
        const FuelProps e0 = P(kFuelPremium95), e10 = P(kFuelE10), e85 = P(kFuelE85), e100 = P(kFuelE100), m100 = P(kFuelM100);
        CHECK(std::fabs(e0.stoich - 14.7) < 0.01 && std::fabs(e100.stoich - 9.0) < 0.01 && std::fabs(m100.stoich - 6.45) < 0.01,
              "stoich AFR: gasoline %.2f, ethanol %.2f, methanol %.2f", e0.stoich, e100.stoich, m100.stoich);
        CHECK(e10.stoich > 13.9 && e10.stoich < 14.3 && e85.stoich > 9.6 && e85.stoich < 10.0,
              "blends by mass: E10 %.2f (~14.1), E85 %.2f (~9.8)", e10.stoich, e85.stoich);
        CHECK(e10.ron - P(kFuelPremium92).ron > 2.5 && e10.ron - P(kFuelPremium92).ron < 4.0 && e85.ron > 105 && e85.ron < 109.5,
              "octane: E10 +%.1f RON on its base, E85 RON %.1f", e10.ron - P(kFuelPremium92).ron, e85.ron);
        CHECK(e85.lhv < 0.72 * e0.lhv && e85.hvap > 2 * e0.hvap && m100.hvap > e100.hvap,
              "E85 heating value %.1f MJ/kg, heat of vaporisation %.0f kJ/kg (gasoline %.0f)", e85.lhv * 1e-6, e85.hvap * 1e-3, e0.hvap * 1e-3);
        // energy per kg of air at stoichiometric: the alcohols give slightly more
        CHECK(m100.lhv / m100.stoich > e0.lhv / e0.stoich && e100.lhv / e100.stoich > e0.lhv / e0.stoich,
              "energy per kg air: gasoline %.2f, ethanol %.2f, methanol %.2f MJ", e0.lhv / e0.stoich * 1e-6, e100.lhv / e100.stoich * 1e-6,
              m100.lhv / m100.stoich * 1e-6);
        const FuelBlend mix = mixBlends(fuelTypeBlend(kFuelPremium95), 30, fuelTypeBlend(kFuelE85), 20);
        CHECK(std::fabs(fuelProps(mix).alcoholPct - 34.0) < 0.5, "30 L premium + 20 L E85 = E%.1f", fuelProps(mix).alcoholPct);
    }
    {
        std::printf("[knock] K20A (CR 11.5) at full load on each gasoline\n");
        for (double rpm : {3000.0, 5500.0}) {
            char t[64];
            std::snprintf(t, sizeof(t), "regular RON 90, %.0f rpm", rpm);  Wot r = runFuel(0, kFuelRegular90, rpm, t);
            std::snprintf(t, sizeof(t), "premium RON 95, %.0f rpm", rpm);  Wot p = runFuel(0, kFuelPremium95, rpm, t);
            std::snprintf(t, sizeof(t), "race RON 104, %.0f rpm", rpm);    Wot h = runFuel(0, kFuelRace, rpm, t);
            // (below the knock limit the index shows the margin; at the limit the knock control holds it there by retard)
            CHECK(r.knock > p.knock + 0.2 || r.retard > p.retard + 1.0, "%.0f rpm: regular knocks more (index %.2f vs %.2f, retard %.1f vs %.1f deg)",
                  rpm, r.knock, p.knock, r.retard, p.retard);
            CHECK(h.retard <= p.retard + 0.01 && h.knock < p.knock, "%.0f rpm: race fuel has more margin (index %.2f, retard %.1f)", rpm, h.knock,
                  h.retard);
        }
    }
    {
        std::printf("[flex fuel] premium calibration, filled with alcohol\n");
        Opt still; still.learn = false;                // no auto-tune / closed loop: what the fuelling alone does
        Wot base = runFuel(0, kFuelPremium95, 5500, "premium", still);
        Wot e85 = runFuel(0, kFuelE85, 5500, "E85 + flex sensor", still);
        Opt noSns = still; noSns.flex = false;
        Wot e85n = runFuel(0, kFuelE85, 5500, "E85, no sensor", noSns);
        CHECK(std::fabs(e85.lam - base.lam) < 0.04, "with the sensor the ECU holds the target lambda (%.3f vs %.3f on premium)", e85.lam, base.lam);
        CHECK(e85.tq > base.tq * 1.02, "E85 makes more torque (%.0f vs %.0f Nm): charge cooling, knock margin", e85.tq, base.tq);
        CHECK(e85n.lam > 1.25, "without the sensor the ECU fuels for gasoline: lean (lambda %.2f)", e85n.lam);
        Opt learn = noSns; learn.learn = true;
        Wot e85l = runFuel(0, kFuelE85, 5500, "E85, no sensor, auto-tune", learn);
        CHECK(e85l.lam < e85n.lam - 0.1, "a learning ECU pulls part of it back (lambda %.2f)", e85l.lam);
        Opt big = still; big.injScale = 2.3;
        Wot m = runFuel(0, kFuelM100, 5500, "methanol, sensor at stoich 9.0", big);
        CHECK(m.lam > 1.2, "methanol on a flex setup for ethanol (stoich 9.0) runs lean (lambda %.2f)", m.lam);
        big.alcStoich = 6.45f;
        Wot m2 = runFuel(0, kFuelM100, 5500, "methanol, alcohol stoich 6.45", big);
        CHECK(std::fabs(m2.lam - base.lam) < 0.05 && m2.tq > base.tq * 1.03, "methanol with the alcohol stoich at 6.45: lambda %.3f, torque %.0f Nm",
              m2.lam, m2.tq);
        CHECK(m2.ve > base.ve + 1.0, "methanol's charge cooling fills the cylinders better (VE %.1f vs %.1f %%)", m2.ve, base.ve);
        Wot ms = runFuel(0, kFuelM100, 5500, "methanol, stock injectors", Opt{true, false, 1.0, 6.45f});
        CHECK(ms.duty > 95 && ms.lam > m2.lam + 0.05, "stock injectors cannot flow methanol (duty %.0f %%, lambda %.2f)", ms.duty, ms.lam);
    }
    {
        std::printf("[refuel] 30 L premium, then 20 L E85 while idling\n");
        EngineParams p = makeCalibratedPreset(0);
        Simulator sim(p);
        start(sim, p);
        p.refuelType = kFuelE85; p.refuelLitres = 20; ++p.refuelRequest; sim.setParams(p);
        run(sim, 0.5);
        const Telemetry a = tel(sim);
        run(sim, 5.0);
        const Telemetry b = tel(sim);
        run(sim, 120.0);
        const Telemetry c = tel(sim);
        std::printf("    tank E%.1f (%.1f L)   sensor after 0.5 s %.1f %%, 5 s %.1f %%, 125 s %.1f %%   ECU stoich %.2f   lambda %.3f\n",
                    c.tankAlcohol, c.tankLitres, a.snsValue[kSnsFlex], b.snsValue[kSnsFlex], c.snsValue[kSnsFlex], c.ecuStoich, c.lambda);
        CHECK(std::fabs(c.tankAlcohol - 34) < 1 && c.tankLitres > 49, "tank now E%.0f, %.1f L", c.tankAlcohol, c.tankLitres);
        CHECK(a.snsValue[kSnsFlex] < 3 && b.snsValue[kSnsFlex] < 15 && c.snsValue[kSnsFlex] > 30,
              "the blend reaches the sensor gradually (%.1f -> %.1f -> %.1f %%)", a.snsValue[kSnsFlex], b.snsValue[kSnsFlex], c.snsValue[kSnsFlex]);
        CHECK(c.running && std::fabs(c.lambda - c.ecuTargetLambda) < 0.08, "and the engine keeps running on target (lambda %.3f / %.3f)",
              c.lambda, c.ecuTargetLambda);
        // a failed sensor: the ECU falls back to its calibration (premium) - lean on E34
        p.sensors[kSnsFlex].fault = SensorFault::Open; sim.setParams(p);
        run(sim, 6.0);
        const Telemetry d = tel(sim);
        CHECK(!d.ecuFlexLive && d.ecuStoich > 14.5, "sensor open circuit: the ECU uses its fallback (stoich %.2f)", d.ecuStoich);
    }
    {
        std::printf("[cold start] -10 C, K20A\n");
        auto cold = [](int fuel, bool flexCold, const char* tag) {
            EngineParams p = makeCalibratedPreset(0);
            p.fuelType = fuel; p.coldStart = 1; p.ambientTemp_C = -10;
            if (!flexCold) for (int i = 0; i < p.cal.flexCold.n; ++i) p.cal.flexCold.v[i] = 0;
            Simulator sim(p);
            const double t = start(sim, p, 6.0, 3.0);
            std::printf("    %-34s start %s %.2f s, idle %4.0f rpm\n", tag, t >= 0 ? "after" : "FAILED", t, tel(sim).rpm);
            return t;
        };
        const double g = cold(kFuelPremium95, true, "premium");
        const double e = cold(kFuelE85, true, "E85, flex cold enrichment");
        const double e0 = cold(kFuelE85, false, "E85, no cold enrichment");
        const double n = cold(kFuelE100, false, "E100, no cold enrichment");
        CHECK(g >= 0 && e >= 0, "premium and E85 (with cold enrichment) start (%.2f / %.2f s)", g, e);
        CHECK(e0 < 0 && n < 0, "E85 / E100 without the flex cold enrichment do not start at -10 C (%.2f / %.2f)", e0, n);
    }
    {
        std::printf("[carburettor] V8 carb on E85: the gasoline jets run lean\n");
        EngineParams p = makeCalibratedPreset(4);
        Simulator sim(p);
        start(sim, p);
        Wot g = wot(sim, p, 3500, 4.0);
        EngineParams q = makeCalibratedPreset(4);
        q.fuelType = kFuelE85;
        Simulator sim2(q);
        start(sim2, q);
        Wot e = wot(sim2, q, 3500, 4.0);
        std::printf("    gasoline lambda %.3f torque %.0f Nm   E85 lambda %.3f torque %.0f Nm\n", g.lam, g.tq, e.lam, e.tq);
        CHECK(e.lam > g.lam * 1.25, "E85 through gasoline jets: lambda %.2f vs %.2f", e.lam, g.lam);
    }
    {
        std::printf("[files] the fuel and the flex settings are saved\n");
        EngineParams p = makePreset(0);
        p.fuelType = kFuelE85; p.flexSensor = 0; p.cal.flexFallbackPct = 85; p.cal.stoichAlcohol = 8.8f; p.cal.flexSpark.v[6] = 5.5f;
        EngineParams q = makePreset(3); EngineMeta m; std::string err;
        engineFromJson(engineToJson(p, m), q, m, err);
        CHECK(q.fuelType == kFuelE85 && q.flexSensor == 0 && q.cal.flexFallbackPct == 85 && q.cal.stoichAlcohol == 8.8f && q.cal.flexSpark.v[6] == 5.5f,
              "engine file: fuel %d, sensor %d, fallback %.0f %%, alcohol stoich %.2f", q.fuelType, q.flexSensor, q.cal.flexFallbackPct, q.cal.stoichAlcohol);
        auto msgs = validateCalibration(p.cal, p);
        bool warned = false;
        for (auto& v : msgs) warned = warned || v.text.find("stoich") != std::string::npos;
        EngineParams r = makePreset(0); r.fuelType = kFuelE85; r.flexSensor = 0;   // calibration still for premium
        auto msgs2 = validateCalibration(r.cal, r);
        bool warned2 = false;
        for (auto& v : msgs2) warned2 = warned2 || v.text.find("stoich") != std::string::npos;
        CHECK(!warned && warned2, "validation: matching fuel settings pass, a premium calibration on E85 without a sensor is flagged");
    }
    std::printf(failures ? "\n%d check(s) FAILED\n" : "\nall checks passed\n", failures);
    return failures ? 1 : 0;
}

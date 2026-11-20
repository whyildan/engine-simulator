// Package 3: flow map and engine health. The averaged gas-path flows must add up (throttle = intake valves, exhaust
// valves = intake + fuel, compressor = throttle, turbine + wastegate = exhaust), reverse flows must show where the
// physics pushes gas back (reversion, standoff, surge), and the health levels must follow their thresholds.
// usage: flow_test
#include "sim/Simulator.h"
#include "sim/Health.h"
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
static void start(Simulator& sim, EngineParams& p) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (double t = 0; t < 4.0; t += 0.01) {
        run(sim, 0.01);
        Telemetry x = tel(sim);
        if (x.running && x.rpm > starterReleaseRpm(p, x.snsValue[kSnsECT])) break;
    }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, 1.5);
}
static void hold(Simulator& sim, EngineParams& p, double rpm, double thr, double settle) {
    p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm; p.throttle = (float)thr; sim.setParams(p);
    run(sim, settle);
}
// flow state averaged over a while (the state itself is a ~0.15 s average)
static FlowState flowAvg(Simulator& sim, double sec) {
    FlowState a{};
    int n = 0;
    for (double t = 0; t < sec; t += 0.05) {
        run(sim, 0.05);
        const FlowState f = tel(sim).flow;
        for (int k = 0; k < kFlowSegCount; ++k) { a.fwd[k] += f.fwd[k]; a.back[k] += f.back[k]; }
        a.pCharge += f.pCharge; a.pPlenum += f.pPlenum; a.pTurbIn += f.pTurbIn; a.pTurbOut += f.pTurbOut; a.pPortEx += f.pPortEx;
        a.tCompOut += f.tCompOut; a.tCharge += f.tCharge; a.tCollector += f.tCollector; a.tPostTurb += f.tPostTurb;
        a.pAmb = f.pAmb; a.tAmb = f.tAmb;
        ++n;
    }
    const double s = 1.0 / std::max(1, n);
    for (int k = 0; k < kFlowSegCount; ++k) { a.fwd[k] *= s; a.back[k] *= s; }
    a.pCharge *= s; a.pPlenum *= s; a.pTurbIn *= s; a.pTurbOut *= s; a.pPortEx *= s;
    a.tCompOut *= s; a.tCharge *= s; a.tCollector *= s; a.tPostTurb *= s;
    return a;
}
static double net(const FlowState& f, int seg) { return f.fwd[seg] - f.back[seg]; }
static double rel(double a, double b) { return std::fabs(a - b) / std::max(1e-6, std::fabs(b)); }
static void printFlow(const char* tag, const FlowState& f) {
    static const char* nm[kFlowSegCount] = {"inlet", "bov", "throttle", "in.valve", "ex.valve", "turbine", "wastegate", "tail"};
    std::printf("    %s:", tag);
    for (int k = 0; k < kFlowSegCount; ++k) if (f.fwd[k] + f.back[k] > 1e-6) std::printf("  %s %.1f/-%.1f", nm[k], f.fwd[k] * 1e3, f.back[k] * 1e3);
    std::printf("  g/s\n");
}

int main() {
    {
        std::printf("[mass balance] K20A part load and full load\n");
        EngineParams p = makeCalibratedPreset(0);
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 3000, 0.10, 4.0);
        FlowState f = flowAvg(sim, 2.0);
        printFlow("3000 rpm 10 %", f);
        const Telemetry t = tel(sim);
        // (port injection: the fuel joins the air in the ports, so the intake valves pass air + fuel)
        const double fuel = t.fuelFlow * 1e-3;
        CHECK(rel(net(f, kFlowIntakeValve), net(f, kFlowThrottle) + fuel) < 0.03, "air through the throttle + fuel = into the cylinders (%.2f + %.2f / %.2f g/s)",
              net(f, kFlowThrottle) * 1e3, fuel * 1e3, net(f, kFlowIntakeValve) * 1e3);
        CHECK(rel(net(f, kFlowExhaustValve), net(f, kFlowIntakeValve)) < 0.03, "out of the cylinders = in (%.2f / %.2f g/s)",
              net(f, kFlowExhaustValve) * 1e3, net(f, kFlowIntakeValve) * 1e3);
        CHECK(rel(net(f, kFlowTail), net(f, kFlowExhaustValve)) < 0.05, "tailpipe carries the exhaust flow (%.2f g/s)", net(f, kFlowTail) * 1e3);
        CHECK(f.pPlenum < f.pAmb - 20, "part throttle: plenum vacuum (%.0f kPa)", f.pPlenum);
        CHECK(f.back[kFlowThrottle] < 0.02 * f.fwd[kFlowThrottle], "part throttle: no standoff (%.2f g/s back)", f.back[kFlowThrottle] * 1e3);
        CHECK(f.tCollector > 600 && f.tCollector < 1300, "collector gas %.0f C", f.tCollector - 273.15);
        const HealthReport h = evaluateHealth(p, t);
        for (int k = 0; k < kHealthItemCount; ++k)
            if (h.item[k].level > kHealthOk) std::printf("    %s: %s\n      %s\n", healthItemName(k), h.item[k].value, h.item[k].detail);
        if (h.part[kPartHead].level > kHealthOk) std::printf("    head: %s\n", h.part[kPartHead].detail);
        CHECK(h.item[kHealthCel].level == kHealthOk && h.item[kHealthKnock].level == kHealthOk && h.part[kPartHead].level == kHealthOk &&
              h.item[kHealthRods].level == kHealthOk && h.item[kHealthSensors].level == kHealthOk &&
              h.item[kHealthValves].level == kHealthOk && h.item[kHealthRepair].level == kHealthOk,
              "healthy engine at cruise: check engine / knock / head / rods / sensors / valves / repair green");
        CHECK(h.part[kPartCompressor].level == kHealthNone && h.part[kPartTurbine].level == kHealthNone, "NA: no compressor or turbine status");
        hold(sim, p, 5500, 1.0, 4.0);
        f = flowAvg(sim, 2.0);
        printFlow("5500 rpm WOT ", f);
        const double fuelW = tel(sim).fuelFlow * 1e-3;
        CHECK(rel(net(f, kFlowIntakeValve), net(f, kFlowThrottle) + fuelW) < 0.03, "WOT: throttle + fuel = intake valves (%.1f + %.1f / %.1f g/s)",
              net(f, kFlowThrottle) * 1e3, fuelW * 1e3, net(f, kFlowIntakeValve) * 1e3);
        CHECK(f.pPlenum > f.pAmb - 12, "WOT: plenum near ambient (%.0f kPa)", f.pPlenum);
    }
    {
        std::printf("[reverse flow] ITB race engine (big overlap) at low rpm, full throttle\n");
        EngineParams p = makeCalibratedPreset(11);
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 2500, 1.0, 4.0);
        FlowState f = flowAvg(sim, 2.0);
        printFlow("2500 rpm WOT ", f);
        CHECK(f.back[kFlowIntakeValve] > 0.05 * f.fwd[kFlowIntakeValve], "reversion: gas pushed back into the ports (%.1f of %.1f g/s)",
              f.back[kFlowIntakeValve] * 1e3, f.fwd[kFlowIntakeValve] * 1e3);
        CHECK(f.back[kFlowThrottle] > 0.01 * f.fwd[kFlowThrottle], "standoff: gas pushed back out through the stacks (%.2f g/s)",
              f.back[kFlowThrottle] * 1e3);
        CHECK(f.back[kFlowExhaustValve] > 0, "overlap: exhaust drawn back into the cylinders (%.2f g/s)", f.back[kFlowExhaustValve] * 1e3);
        const double fuelI = tel(sim).fuelFlow * 1e-3;
        CHECK(rel(net(f, kFlowIntakeValve), net(f, kFlowThrottle) + fuelI) < 0.05, "net flows still balance (%.1f + %.1f fuel / %.1f g/s)",
              net(f, kFlowThrottle) * 1e3, fuelI * 1e3, net(f, kFlowIntakeValve) * 1e3);
    }
    {
        std::printf("[turbo] I4 turbo on boost, then the throttle snaps shut\n");
        EngineParams p = makeCalibratedPreset(16);
        Simulator sim(p);
        start(sim, p);
        hold(sim, p, 4500, 1.0, 6.0);
        FlowState f = flowAvg(sim, 2.0);
        Telemetry t = tel(sim);
        printFlow("4500 rpm WOT ", f);
        std::printf("    boost %.2f bar  charge %.0f kPa  compressor out %.0f C  after IC %.0f C  EMAP %.0f / downpipe %.0f kPa  "
                    "collector %.0f C  after turbine %.0f C  IC core %.0f C\n",
                    t.boostBar, f.pCharge, f.tCompOut - 273.15, f.tCharge - 273.15, f.pTurbIn, f.pTurbOut, f.tCollector - 273.15,
                    f.tPostTurb - 273.15, t.icCoreC);
        CHECK(t.boostBar > 0.4f, "on boost (%.2f bar)", t.boostBar);
        CHECK(rel(net(f, kFlowInlet), net(f, kFlowThrottle)) < 0.08, "compressor = throttle (%.1f / %.1f g/s)", net(f, kFlowInlet) * 1e3,
              net(f, kFlowThrottle) * 1e3);
        // (the exhaust pipes are an acoustic waveguide: their mean mass flux follows each pipe's own gas density, so it
        //  is not carried over exactly between pipes at different temperatures - the turbine sees ~10-15 % more)
        CHECK(rel(net(f, kFlowTurbine) + net(f, kFlowWastegate), net(f, kFlowExhaustValve)) < 0.18, "turbine + wastegate ~ exhaust (%.1f + %.1f / %.1f g/s)",
              net(f, kFlowTurbine) * 1e3, net(f, kFlowWastegate) * 1e3, net(f, kFlowExhaustValve) * 1e3);
        CHECK(f.tCompOut > f.tCharge + 10 && f.tCharge > f.tAmb, "compressor heats the air, the intercooler takes part of it back");
        CHECK(f.pCharge > f.pPlenum - 2 && f.pCharge > f.pAmb + 30, "charge pipe pressurised (%.0f kPa)", f.pCharge);
        CHECK(f.pTurbIn > f.pTurbOut + 20, "pressure drop across the turbine (%.0f -> %.0f kPa)", f.pTurbIn, f.pTurbOut);
        CHECK(f.tPostTurb < f.tCollector - 20, "the turbine takes heat out of the gas (%.0f -> %.0f C)", f.tCollector - 273.15, f.tPostTurb - 273.15);
        const HealthReport h = evaluateHealth(p, t);
        std::printf("    health: compressor %d (%s)  turbine %d (%s)  intercooler %d (%s)\n", h.part[kPartCompressor].level,
                    h.part[kPartCompressor].value, h.part[kPartTurbine].level, h.part[kPartTurbine].value, h.part[kPartIntercooler].level,
                    h.part[kPartIntercooler].value);
        CHECK(h.part[kPartCompressor].level == kHealthOk && h.part[kPartIntercooler].level == kHealthOk, "steady boost: compressor and intercooler green");
        // lift off: with the blow-off valve the charge vents; without it the compressor surges (flow backwards)
        const bool hadBov = p.bov != 0;
        p.dynoMode = DynoMode::Off; p.throttle = 0; sim.setParams(p);
        double ventMax = 0;
        for (int i = 0; i < 20; ++i) { run(sim, 0.02); ventMax = std::max(ventMax, tel(sim).flow.fwd[kFlowBov]); }
        if (hadBov) CHECK(ventMax > 1e-3, "lift-off: the blow-off valve vents the charge (%.1f g/s)", ventMax * 1e3);
        p.bov = 0;
        hold(sim, p, 4500, 1.0, 4.0);
        p.dynoMode = DynoMode::Off; p.throttle = 0; sim.setParams(p);
        double backMax = 0; int surgeLevel = 0;
        for (int i = 0; i < 25; ++i) {
            run(sim, 0.02);
            const Telemetry x = tel(sim);
            backMax = std::max(backMax, x.flow.back[kFlowInlet]);
            surgeLevel = std::max(surgeLevel, evaluateHealth(p, x).part[kPartCompressor].level);
        }
        CHECK(backMax > 2e-3, "no blow-off valve: surge pushes air back out of the compressor (%.1f g/s)", backMax * 1e3);
        CHECK(surgeLevel == kHealthLimit, "surge shows the compressor red");
    }
    {
        std::printf("[health levels] thresholds from the protection rules and component limits\n");
        const EngineParams p = makeCalibratedPreset(0);
        Telemetry t{};
        t.running = true; t.rpm = 3000; t.sparkMargin = 9; t.plugTempC = 600; t.egt = 700; t.tqThrCap = 1;
        HealthReport h = evaluateHealth(p, t);
        CHECK(h.part[kPartHead].level == kHealthOk && h.part[kPartPlugs].level == kHealthOk && h.part[kPartHeader].level == kHealthOk &&
              h.part[kPartThrottle].level == kHealthOk && h.item[kHealthCel].level == kHealthOk, "baseline green");
        {
            Telemetry x = t; x.knockRetard[2] = 0.95f * p.cal.knockMaxRetard;
            const HealthReport k = evaluateHealth(p, x);
            CHECK(k.part[kPartHead].level == kHealthLimit && k.cyl[2] == kHealthLimit && k.cyl[0] == kHealthOk && k.item[kHealthKnock].level == kHealthLimit,
                  "knock control out of retard: head, cylinder 3 and the knock tile red (%s)", k.item[kHealthKnock].value);
            x.knockRetard[2] = 0.5f * p.cal.knockMaxRetard;
            CHECK(evaluateHealth(p, x).cyl[2] == kHealthWarn, "half the authority: yellow");
        }
        {
            Telemetry x = t; x.plugTempC = 900;
            CHECK(evaluateHealth(p, x).part[kPartPlugs].level == kHealthWarn, "plug tip 900 C: pre-ignition warning");
            x.plugTempC = 600; x.fouling = 0.5f;
            CHECK(evaluateHealth(p, x).item[kHealthPlugs].level == kHealthLimit, "fouled plugs: red");
            x.fouling = 0; x.sparkMargin = 0.9f;
            CHECK(evaluateHealth(p, x).part[kPartPlugs].level == kHealthLimit, "too little spark energy: red");
        }
        {
            Telemetry x = t; x.egt = p.cal.prot[kProtEgt].warn + 5;
            CHECK(evaluateHealth(p, x).part[kPartHeader].level == kHealthWarn, "EGT past the rule's warning: header yellow");
            x.egt = p.cal.prot[kProtEgt].limit + 5;
            CHECK(evaluateHealth(p, x).part[kPartHeader].level == kHealthLimit, "EGT past the rule's limit: header red");
        }
        {
            EngineParams q = p; q.sensors[kSnsMAP].fault = SensorFault::Stuck;
            CHECK(evaluateHealth(q, t).item[kHealthSensors].level == kHealthWarn, "stuck sensor (not detected): sensors yellow");
            Telemetry x = t; x.snsFallback[kSnsIAT] = true;
            const HealthReport k = evaluateHealth(p, x);
            CHECK(k.item[kHealthSensors].level == kHealthLimit && k.item[kHealthCel].level == kHealthWarn, "detected fault: sensors red, check engine on");
            x = t; x.protState[kProtCoolant] = 2;
            CHECK(evaluateHealth(p, x).item[kHealthCel].level == kHealthLimit, "protection action: check engine red");
        }
        {
            Telemetry x = t; x.throttleLimited = true;
            CHECK(evaluateHealth(p, x).part[kPartThrottle].level == kHealthLimit, "throttle limited by protection: red");
            x = t; x.tqThrCap = 0.6f;
            CHECK(evaluateHealth(p, x).part[kPartThrottle].level == kHealthWarn, "torque management cap: yellow");
        }
        {
            ComponentModel cm; cm.build(p);
            Telemetry x = t; x.rpm = (float)(0.95 * cm.partSafeRpm[1]);
            HealthReport k = evaluateHealth(p, x);
            CHECK(k.item[kHealthRods].level >= kHealthWarn, "rods near their safe speed (%.0f of %.0f rpm): %s", x.rpm, cm.partSafeRpm[1], k.item[kHealthRods].value);
            x.rpm = (float)(1.02 * cm.safeRpm);
            k = evaluateHealth(p, x);
            const int weakest = cm.weakest == 0 ? kHealthPistons : cm.weakest == 1 ? kHealthRods : kHealthCrank;
            CHECK(k.item[weakest].level == kHealthLimit, "past the weakest part's limit: %s red", healthItemName(weakest));
            x = t; x.failure = 2;
            k = evaluateHealth(p, x);
            CHECK(k.item[kHealthRods].level == kHealthLimit && k.item[kHealthRepair].level == kHealthLimit, "rod failure: rods and repair red");
            x = t; x.floatIn = 0.3f;
            CHECK(evaluateHealth(p, x).item[kHealthValves].level == kHealthWarn, "valve float 30 %%: yellow");
        }
        {
            EngineParams q = makeCalibratedPreset(16);
            Telemetry x = t; x.flow.tAmb = 298; x.icCoreC = 25 + 45; x.speedFrac = 0.5f; x.surgeMargin = 1; x.boostBar = 1.0f;
            HealthReport k = evaluateHealth(q, x);
            CHECK(k.part[kPartIntercooler].level == kHealthLimit, "intercooler core 45 K above ambient: heat soak red");
            x.icCoreC = 30; x.speedFrac = 0.97f;
            k = evaluateHealth(q, x);
            CHECK(k.part[kPartIntercooler].level == kHealthOk && k.part[kPartCompressor].level == kHealthWarn, "turbo at 97 %% of top speed: yellow");
            x.speedFrac = 0.5f; x.flow.pPlenum = 201.3; x.flow.pTurbIn = 2.4 * 201.3;
            CHECK(evaluateHealth(q, x).part[kPartTurbine].level >= kHealthWarn, "drive pressure past the rule's warning: turbine yellow");
        }
        {
            const EngineParams r = makeCalibratedPreset(12);
            const HealthReport k = evaluateHealth(r, t);
            CHECK(k.item[kHealthValves].level == kHealthNone && k.item[kHealthRods].level == kHealthNone, "rotary: no valves, no rods");
        }
    }
    std::printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

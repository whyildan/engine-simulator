// Phase 1c: virtual sensors, ECU fallbacks and closed loop, thermal model, battery, fuel supply.
// usage: ecu_test
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include "ecu/Sensors.h"
#include <cmath>
#include <cstdio>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static void run(Simulator& sim, double s) {
    std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}
static Telemetry tel(Simulator& sim) { Telemetry t; sim.getTelemetry(t); return t; }

// start and idle; returns the simulator
static void startIdle(Simulator& sim, EngineParams& p, double idleSeconds) {
    p.ignition = 1; p.starter = 1; sim.setParams(p);
    for (int k = 0; k < 400; ++k) { run(sim, 0.01); Telemetry t = tel(sim); if (t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT])) break; }
    run(sim, 0.15);
    p.starter = 0; sim.setParams(p);
    run(sim, idleSeconds);
}

int main() {
    // ---------------- sensor layer on its own
    {
        std::printf("[sensors] lag, ADC, faults\n");
        EngineParams p = makeCalibratedPreset(0);
        SensorLayer s; s.reset(p);
        SensorTruth t;
        for (int i = 0; i < kSensorCount; ++i) t.v[i] = sensorDef(i).lo + 0.3 * (sensorDef(i).hi - sensorDef(i).lo);
        t.v[kSnsECT] = 20; t.v[kSnsWideband] = t.v[kSnsO2] = 1.0;
        const double dt = 1e-3;
        s.update(dt, t, p);
        t.v[kSnsECT] = 90;
        for (int k = 0; k < 2000; ++k) s.update(dt, t, p);   // 2 s = one time constant of the ECT sensor
        std::printf("  ECT step 20 -> 90 C, after 2 s the sensor reads %.1f C\n", s.r.value[kSnsECT]);
        CHECK(std::fabs(s.r.value[kSnsECT] - (20 + 70 * 0.632)) < 3.0, "first-order lag of the coolant sensor");
        double res = (sensorDef(kSnsMAP).hi - sensorDef(kSnsMAP).lo) / 4.0 * 5.0 / 1023.0;
        CHECK(std::fabs(s.r.value[kSnsMAP] - t.v[kSnsMAP]) < 3 * res + 4 * sensorDef(kSnsMAP).noise, "MAP within ADC resolution + noise (%.2f kPa steps)", res);
        p.sensors[kSnsMAP].fault = SensorFault::Open; s.update(dt, t, p);
        CHECK(s.r.status[kSnsMAP] == kStatusLow && s.r.fallback[kSnsMAP], "open MAP wire -> out of range low -> fallback");
        p.sensors[kSnsMAP].fault = SensorFault::Short; s.update(dt, t, p);
        CHECK(s.r.status[kSnsMAP] == kStatusHigh && s.r.fallback[kSnsMAP], "MAP shorted to 5 V -> out of range high -> fallback");
        p.sensors[kSnsMAP].fault = SensorFault::None;
        p.sensors[kSnsIAT].fault = SensorFault::Stuck; s.update(dt, t, p);
        double stuck = s.r.value[kSnsIAT];
        t.v[kSnsIAT] += 40; for (int k = 0; k < 5000; ++k) s.update(dt, t, p);
        CHECK(std::fabs(s.r.value[kSnsIAT] - stuck) < 1e-9 && !s.r.fallback[kSnsIAT], "stuck sensor holds its value and is not detected");
        p.sensors[kSnsCKP].fault = SensorFault::Open; s.update(dt, t, p);
        CHECK(s.r.status[kSnsCKP] == kStatusNoSignal, "broken crank sensor wire -> no signal");
    }
    // ---------------- ECU fallbacks on a running engine
    {
        std::printf("[ECU fallbacks] I4 idling\n");
        EngineParams p = makeCalibratedPreset(0);
        Simulator sim(p); startIdle(sim, p, 3.0);
        Telemetry a = tel(sim);
        p.sensors[kSnsMAP].fault = SensorFault::Open; sim.setParams(p); run(sim, 4.0);
        Telemetry b = tel(sim);
        std::printf("  MAP sensor open: ECU uses %.1f kPa (alpha-N estimate, real %.1f), rpm %.0f -> %.0f\n", b.ecuMap, b.mapKPa, a.rpm, b.rpm);
        CHECK(b.snsFallback[kSnsMAP] && b.rpm > 450, "MAP failure: limp on the throttle-position estimate, engine keeps running");
        p.sensors[kSnsMAP].fault = SensorFault::None;
        p.sensors[kSnsCKP].fault = SensorFault::Open; sim.setParams(p); run(sim, 1.5);
        Telemetry c = tel(sim);
        std::printf("  crank sensor open: rpm %.0f, sync lost %d\n", c.rpm, (int)c.syncLost);
        CHECK(c.syncLost && c.rpm < 200, "no crank signal -> no spark / fuel -> engine stops");
    }
    // ---------------- closed loop corrects a wrong sensor: battery sensor stuck at the key-off voltage
    {
        std::printf("[closed loop] battery sensor stuck at key-off voltage -> dead time over-compensated -> rich\n");
        for (int tune = 0; tune < 2; ++tune) {
            EngineParams p = makeCalibratedPreset(0);
            p.sensors[kSnsBattery].fault = SensorFault::Stuck;
            p.cal.autoTune = tune; p.cal.closedLoop = 1;
            Simulator sim(p); startIdle(sim, p, tune ? 20.0 : 12.0);   // auto-tune learns a few % per second at a hunting idle
            int clN = 0, n = 0;   // share of the time in closed loop over the last 2 s (a single sample can hit a transient)
            for (; n < 40; ++n) { run(sim, 0.05); clN += tel(sim).closedLoop ? 1 : 0; }
            Telemetry t = tel(sim);
            std::printf("  closed loop %.0f %% of the time\n", 100.0 * clN / n);
            t.closedLoop = clN > n / 2;
            std::printf("  %s: battery real %.2f V, ECU sees %.2f V, trim %.3f, wideband lambda %.3f (target %.3f)\n",
                        tune ? "auto-tune" : "closed-loop trim", t.battery, t.snsValue[kSnsBattery], t.ecuTrim, t.snsValue[kSnsWideband], t.targetAfr / 14.7f);
            if (!tune) CHECK(t.closedLoop && t.ecuTrim < 0.985, "short-term trim pulls the extra fuel out");
            else CHECK(std::fabs(t.snsValue[kSnsWideband] / (t.targetAfr / 14.7f) - 1.0f) < 0.04f, "auto-tune learns it into the VE map (lambda on target)");
            if (tune) CHECK(t.battery > 13.8, "alternator charges while running (%.2f V)", t.battery);
        }
    }
    // ---------------- thermal model
    {
        std::printf("[thermal] cold start at 20 C: idle 60 s, then 3500 rpm / 45 %% throttle on the dyno for 300 s\n");
        EngineParams p = makeCalibratedPreset(0);
        p.coldStart = 1; p.ambientTemp_C = 20;
        Simulator sim(p); startIdle(sim, p, 60.0);
        Telemetry a = tel(sim);
        std::printf("  after 60 s idle: coolant %.1f C, oil %.1f C, idle %.0f rpm (fast idle), warm-up enrichment %.0f %%\n", a.coolantC, a.oilC, a.rpm, a.ecuWarmup * 100);
        CHECK(a.coolantC > 23 && a.rpm > p.idleTargetRpm + 50 && a.ecuWarmup > 0.1, "cold engine warms, idles fast and runs warm-up enrichment");
        p.throttle = 0.45f; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 3500; sim.setParams(p);
        double maxC = 0;
        for (int k = 0; k < 30; ++k) { run(sim, 10.0); maxC = std::max(maxC, (double)tel(sim).coolantC); }
        Telemetry b = tel(sim);
        std::printf("  after 300 s load: coolant %.1f C (max %.1f), oil %.1f C, thermostat %.0f %%, fan %d, radiator %.1f kW\n",
                    b.coolantC, maxC, b.oilC, b.thermostat * 100, (int)b.fan, b.radiatorKw);
        CHECK(b.coolantC > 82 && maxC < 106, "thermostat regulates the coolant (82..106 C)");
        CHECK(b.ecuWarmup < 0.02, "warm-up enrichment ends once warm");
    }
    // ---------------- fuel supply: an undersized pump can't hold pressure at full power
    {
        std::printf("[fuel pump] I4 at 6000 rpm WOT, stock vs tiny pump\n");
        for (int tiny = 0; tiny < 2; ++tiny) {
            EngineParams p = makeCalibratedPreset(0);
            if (tiny) p.comp.fuelPumpLph = 22;
            // this checks the physics: no protection may step in (low fuel pressure / lean would limit the throttle)
            p.cal.prot[kProtFuelPress].enabled = 0; p.cal.prot[kProtLean].enabled = 0;
            Simulator sim(p); startIdle(sim, p, 1.0);
            p.throttle = 1; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = 6000; sim.setParams(p); run(sim, 3.0);
            Telemetry t = tel(sim);
            std::printf("  pump %.0f L/h: rail %.2f bar, AFR %.2f (target %.2f)\n", p.comp.fuelPumpLph, t.fuelBar, t.afr, t.targetAfr);
            static float afrStock = 0;
            if (!tiny) { afrStock = t.afr; CHECK(t.fuelBar > 2.8, "stock pump holds the pressure"); }
            else CHECK(t.fuelBar < 2.0 && t.afr > afrStock + 1.0, "tiny pump: pressure sags, the engine runs lean");
        }
    }
    // ---------------- ECU loop rate
    {
        std::printf("[ECU rate] idle stays stable at 100 Hz and 5 kHz loop rates\n");
        for (float hz : {100.0f, 5000.0f}) {
            EngineParams p = makeCalibratedPreset(0); p.ecuRateHz = hz;
            Simulator sim(p); startIdle(sim, p, 4.0);
            Telemetry t = tel(sim);
            std::printf("  %5.0f Hz: idle %.0f rpm\n", hz, t.rpm);
            CHECK(t.rpm > 600 && t.rpm < 1200, "idle with the ECU at %.0f Hz", hz);
        }
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}

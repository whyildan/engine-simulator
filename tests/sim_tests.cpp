// Headless sanity checks for the physics. Prints a WOT torque table per preset and checks
// idle stability. Exit code != 0 on failure.
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <cmath>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static void runSeconds(Simulator& sim, double s) {
    std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}

// motored closed cylinder: valves closed (no lift), no fuel -> pressure must return to start after a cycle
static void testMotoredCycle() {
    std::printf("[motored closed cylinder]\n");
    EngineParams p = makePreset(0);
    p.intakeLift_mm = 0; p.exhaustLift_mm = 0; p.valveLash_mm = 0.5f;
    Engine e(p, 96000);
    const double w = 2 * kPi * 1200 / 60.0;
    const double dt = 1.0 / 96000;
    const int stepsPerCycle = (int)std::lround(720.0 * kDeg / (w * dt));
    auto run = [&](int n) { for (int i = 0; i < n; ++i) { e.setCrank(w, e.theta()); e.step(dt, 0, 0); } };
    run(stepsPerCycle);
    double p0 = e.cyl(0).p, m0 = e.cyl(0).m, th0 = e.theta();
    run(stepsPerCycle);
    double p1 = e.cyl(0).p, m1 = e.cyl(0).m;
    std::printf("  theta %.1f -> %.1f  p0=%.4f bar p1=%.4f bar  mass drift %.3g%%\n", th0, e.theta(), p0 / 1e5, p1 / 1e5, (m1 - m0) / m0 * 100);
    CHECK(std::fabs(m1 - m0) / m0 < 1e-6, "mass not conserved in closed cylinder");
    // walls (450 K) heat the cold charge, so only check it stays bounded
    CHECK(p1 < p0 * 1.4 && p1 > p0 * 0.7, "closed cycle pressure drift too large");
}

static void wotTable(int preset, double expectMinNm, double expectMaxNm) {
    EngineParams p = makeCalibratedPreset(preset);
    std::printf("[%s]  WOT dyno hold\n", p.name);
    p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    runSeconds(sim, 3.0);
    p.starter = 0;
    sim.setParams(p);
    runSeconds(sim, 1.5);
    Telemetry t;
    sim.getTelemetry(t);
    std::printf("  idle: rpm %.0f map %.1f kPa afr %.2f\n", t.rpm, t.mapKPa, t.afr);
    CHECK(t.rpm > 450 && t.rpm < 1600, "idle rpm %.0f out of range", t.rpm);
    double peak = 0;
    double rmax = std::min(7000.0, p.revLimit - 300.0);
    for (double rpm = 1500; rpm <= rmax; rpm += 1000) {
        p.throttle = 1; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = (float)rpm;
        sim.setParams(p);
        runSeconds(sim, 1.5);
        sim.getTelemetry(t);
        std::printf("  %5.0f rpm  torque %6.1f Nm  power %6.1f kW (%5.1f hp)  map %5.1f afr %5.2f ve %.2f adv %.0f pmax %.0f bar egt %.0f C load %.2f | boost %.2f shaft %.0f wg %.2f knock %.2f chg %.0fC duty %.2f\n",
                    t.rpm, t.torque, t.powerKw, t.powerKw * 1.341f, t.mapKPa, t.afr, t.ve, t.advance, t.cylPmax[0], t.egt, t.simLoad,
                    t.boostBar, t.shaftRpm, t.wastegate, t.knock, t.chargeT, t.injDuty);
        peak = std::max(peak, (double)t.torque);
    }
    CHECK(peak > expectMinNm && peak < expectMaxNm, "peak torque %.0f Nm outside %.0f..%.0f", peak, expectMinNm, expectMaxNm);
}

static void debugRun(int preset, float thr, double rpmHold) {
    EngineParams p = makeCalibratedPreset(preset);
    p.ignition = 1; p.starter = 1; p.throttle = thr;
    if (std::getenv("DUR")) { p.intakeDuration = p.exhaustDuration = (float)std::atof(std::getenv("DUR")); }
    if (std::getenv("FUEL0")) { p.injector_ccmin = 0.01f; }
    Engine e(p, 96000);
    double dt = 1.0 / 96000;
    for (int i = 0; i < 96000 * 5; ++i) {
        double load = 0;
        double st = std::getenv("START") ? std::atof(std::getenv("START")) : 1.0;
        if (i == (int)(96000 * st)) { p.starter = 0; e.apply(p); }
        if (rpmHold > 0) e.setCrank(rpmHold * 2 * kPi / 60, e.theta());
        double tip = std::getenv("TIP") ? std::atof(std::getenv("TIP")) : 0;
        double rel = std::getenv("REL") ? std::atof(std::getenv("REL")) : 1e9;
        double pedalNow = ((tip > 0 && i * dt < tip) || i * dt > rel) ? 0.0 : thr;
        e.step(dt, pedalNow, load);
        if (i % (std::getenv("FINE") ? 960 : 9600) == 0) {
            const Cylinder& c = e.cyl(0);
            std::printf("t=%.2f rpm %.0f tq %.1f map %.1f | air %.3g fuel %.3g lam %.2f resid %.2f ve %.2f | pmax %.1f imep %.2f | port m %.3g mf %.3g p %.3f | exh p %.3f | mis %d air %.2f g/s iac %.1f\n",
                i * dt, e.rpm(), e.netTorque(), e.intake.map() / 1000, c.trappedAir, c.trappedFuel, c.lambda,
                c.m > 0 ? c.mb / c.m : 0, c.ve, c.pmaxCycle / 1e5, c.imep / 1e5,
                e.intake.port(0).m, e.intake.port(0).mf, e.intake.port(0).p() / 1e5, e.exhaust.portPressure(0) / 1e5, e.misfires, e.intake.airFlow()*1000, e.intake.iacPct());
        }
    }
}

static void massAudit() {
    EngineParams p = makePreset(0);
    p.injector_ccmin = 0.01f; p.ignition = 0;
    Engine e(p, 96000);
    double dt = 1.0 / 96000;
    double w = 1500 * 2 * kPi / 60;
    for (int i = 0; i < 96000; ++i) { e.setCrank(w, e.theta()); e.step(dt, 0, 0); }
    double sPortEnd = 0, sPlenEnd = 0, sThr = 0;
    double plen0 = e.intake.inlet(0).m;
    for (int i = 0; i < 9600; ++i) {
        e.setCrank(w, e.theta()); e.step(dt, 0, 0);
        auto& ne = e.intake.net.nodeEnds;
        for (size_t k = 0; k < ne.size(); ++k) (k % 2 == 0 ? sPortEnd : sPlenEnd) += ne[k].massFlowIntoPipe * dt;
        sThr += e.intake.airFlow() * dt;
    }
    std::printf("mass audit 0.1s: throttle in %.3g  plenum->pipes %.3g  ports->pipes %.3g  plenum dm %.3g\n",
                sThr, sPlenEnd, sPortEnd, e.intake.inlet(0).m - plen0);
}

// Random live parameter changes (as a user dragging sliders) - checks for NaN / blow ups.
static void fuzz(int seconds) {
    Rng rng(777);
    std::vector<float> buf(480 * 2);
    EngineParams p = makePreset(0);
    p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    Telemetry t;
    int bad = 0, changes = 0;
    auto U = [&](float lo, float hi) { return (float)(lo + (hi - lo) * rng.uniform()); };
    for (int step = 0; step < seconds * 4; ++step) {
        int what = (int)(rng.uniform() * 18);
        switch (what) {
        case 17: {   // ECU rate, cold start, fuel supply, sensor faults / noise / lag
            p.ecuRateHz = U(100, 5000); p.coldStart = rng.uniform() < 0.3; p.ambientTemp_C = U(-25, 45);
            p.comp.fuelPressure_bar = U(1.5f, 6); p.comp.fuelRegulator = (FuelRegulator)(int)(rng.uniform() * 2); p.comp.fuelPumpLph = U(20, 400);
            int s = (int)(rng.uniform() * kSensorCount);
            p.sensors[s].fault = (SensorFault)(int)(rng.uniform() * 5);
            p.sensors[s].noiseScale = U(0, 10); p.sensors[s].lag_ms = U(0, 5000);
            break;
        }
        case 16: {   // hardware components, including absurd ones and failures
            ComponentSet& k = p.comp;
            k.pistonType = (PistonType)(int)(rng.uniform() * 3); k.rodType = (RodType)(int)(rng.uniform() * 4);
            k.crankType = (CrankType)(int)(rng.uniform() * 3); k.lifter = (LifterType)(int)(rng.uniform() * 4);
            k.pistonMass_g = U(150, 1200); k.rodMass_g = U(200, 1400); k.ringTension = U(0.5f, 1.8f);
            k.springOpen_N = U(150, 2000); k.springSeat_N = U(80, 1000);
            k.throttleType = (ThrottleType)(int)(rng.uniform() * 2); k.dbwResponse_ms = U(5, 200);
            k.injDeadTime_ms = U(0.2f, 2.5f); k.coilType = (CoilType)(int)(rng.uniform() * 3);
            k.coilEnergy_mJ = U(10, 150); k.coilCharge_ms = U(1, 8); k.plugHeatRange = 4 + (int)(rng.uniform() * 7);
            k.plugGap_mm = U(0.4f, 1.6f); k.oilPumpCapacity = U(0.4f, 2.0f); k.oilRelief_bar = U(2, 9);
            k.waterPumpCapacity = U(0.4f, 2.0f); k.allowFailure = rng.uniform() < 0.5;
            if (rng.uniform() < 0.3) k.repairCount++;
            break;
        }
        case 0: { int keep = p.ignition; p = makePreset((int)(rng.uniform() * presetCount())); p.ignition = keep; break; }
        case 1: p.engineType = rng.uniform() < 0.4 ? EngineType::Rotary : EngineType::Piston;
                p.cylinders = 1 + (int)(rng.uniform() * 16); p.layout = (Layout)(int)(rng.uniform() * 3); applyLayoutDefaults(p);
                if (isRotary(p)) { p.rotorRadius_mm = U(70, 140); p.eccentricity_mm = U(10, 22); p.rotorWidth_mm = U(50, 110);
                                   applyRotaryPorting(p, (int)(rng.uniform() * 4)); p.apexLeak = U(0, 1); }
                break;
        case 2: p.bore_mm = U(40, 140); p.stroke_mm = U(40, 140); p.rod_mm = std::max(p.stroke_mm * 1.2f, U(60, 260)); p.compression = U(6, 16); break;
        case 3: p.intakeDuration = U(180, 340); p.exhaustDuration = U(180, 340); p.intakeCamAdvance = U(-30, 40); p.intakeLift_mm = U(4, 18); break;
        case 4: p.baseAdvance = U(-10, 40); p.mechAdvance = U(0, 35); p.revLimit = U(3000, 12000); p.limiter = (Limiter)(int)(rng.uniform() * 2); break;
        case 5: p.fuelSystem = (FuelSystem)(int)(rng.uniform() * 2); p.itb = rng.uniform() < 0.3; p.plenum_L = U(0.3f, 12); break;
        case 15: p.vehicle = rng.uniform() < 0.6; p.transMode = (TransMode)(int)(rng.uniform() * 3);
                 p.gear = (int)(rng.uniform() * (p.gears + 1)); p.clutchPedal = rng.uniform() < 0.5 ? 0 : U(0, 1);
                 p.brake = rng.uniform() < 0.7 ? 0 : U(0, 1); p.grip = U(0.3f, 1.8f); p.mass_kg = U(150, 3000);
                 p.gears = 1 + (int)(rng.uniform() * 8); p.finalDrive = U(1.5f, 7); p.clutchTorque = U(80, 2500); p.grade_pct = U(-15, 25); break;
        case 14: applyInductionDefaults(p, (Induction)(int)(rng.uniform() * 3), (Supercharger)(int)(rng.uniform() * 3));
                 p.wastegateBar = U(0.2f, 2.5f); p.turbineAR = U(0.3f, 1.4f); p.bov = rng.uniform() < 0.5; p.antiLag = rng.uniform() < 0.3;
                 p.scRatio = p.scType == Supercharger::Centrifugal ? U(5, 14) : U(0.6f, 2.5f); p.intercooler = rng.uniform() < 0.7; break;
        case 6: p.runnerLen_mm = U(60, 800); p.runnerDia_mm = U(20, 70); p.throttleBore_mm = U(20, 110); break;
        case 7: p.header = (HeaderType)(int)(rng.uniform() * 3); p.crossover = (Crossover)(int)(rng.uniform() * 4); p.muffler = (Muffler)(int)(rng.uniform() * 4); p.cat = rng.uniform() < 0.5; p.resonator = rng.uniform() < 0.5; break;
        case 8: for (int i = 0; i < kMaxCyl; ++i) p.primaryLen_mm[i] = U(80, 1500); p.primaryDia_mm = U(25, 60); p.pipeDia_mm = U(30, 110); p.tailpipeLen_mm = U(50, 2500); break;
        case 9: p.throttle = rng.uniform() < 0.4 ? 0 : U(0, 1); break;
        case 10: p.dynoMode = (DynoMode)(int)(rng.uniform() * 3); p.dynoHoldRpm = U(800, 7000); p.sweepRequest++; break;
        case 11: p.oversample = 1 + (int)(rng.uniform() * 3); p.listener = (ListenerPos)(int)(rng.uniform() * 3);
                 p.exhaustSteep = U(0, 2.5f); p.exhaustRasp = U(0, 3); p.exhaustPunch = U(0, 1.5f);
                 p.soundStyle = rng.uniform() < 0.3; p.pressureMix = U(0, 1); p.smoothHz = U(300, 20000); break;
        case 12: p.evenFire = rng.uniform() < 0.5; for (int i = 0; i < kMaxCyl; ++i) p.phase[i] = U(0, 720); break;
        case 13: p.starter = 1; p.ignition = 1; break;
        }
        if (what != 13) p.starter = 0;
        ++changes;
        sim.setParams(p);
        for (int k = 0; k < 25; ++k) {
            sim.render(buf.data(), 480);
            for (float s : buf) if (!std::isfinite(s)) { ++bad; break; }
        }
        sim.getTelemetry(t);
        if (!std::isfinite(t.rpm) || !std::isfinite(t.mapKPa) || t.rpm > 30000 || t.mapKPa > 1000) {
            std::printf("  step %d: rpm %.0f map %.1f (cyl %d layout %d fuel %d itb %d)\n", step, t.rpm, t.mapKPa,
                        p.cylinders, (int)p.layout, (int)p.fuelSystem, p.itb);
            ++bad;
        }
    }
    std::printf("fuzz: %d parameter changes, %d bad states\n", changes, bad);
    if (bad) ++failures;
}

static void survey() {
    std::vector<float> buf(480 * 2);
    Telemetry t;
    for (int i = 0; i < presetCount(); ++i) {
        EngineParams p = std::getenv("GENERIC") ? makePreset(i) : makeCalibratedPreset(i);
        if (std::getenv("IMP")) p.imperfection = (float)std::atof(std::getenv("IMP"));
        // diagnostics: part overrides
        if (std::getenv("CABLE")) p.comp.throttleType = ThrottleType::Cable;
        if (std::getenv("OS")) p.oversample = std::atoi(std::getenv("OS"));
        if (std::getenv("COLD")) { p.coldStart = 1; p.ambientTemp_C = (float)std::atof(std::getenv("COLD")); }
        if (std::getenv("DEAD")) p.comp.injDeadTime_ms = (float)std::atof(std::getenv("DEAD"));
        if (std::getenv("COIL")) p.comp.coilEnergy_mJ = (float)std::atof(std::getenv("COIL"));
        if (std::getenv("HEAT")) p.comp.plugHeatRange = std::atoi(std::getenv("HEAT"));
        if (std::getenv("SPRING")) { float s = (float)std::atof(std::getenv("SPRING")); p.comp.springOpen_N *= s; p.comp.springSeat_N *= s; }
        if (std::getenv("ONLY") && std::atoi(std::getenv("ONLY")) != i) continue;
        p.ignition = 1; p.starter = 1;
        Simulator sim(p);
        double crank = 0;
        double caught = 0;
        for (; crank < 6.0; crank += 0.01) {
            sim.render(buf.data(), 480); sim.getTelemetry(t);
            caught = t.running && t.rpm > starterReleaseRpm(p, t.snsValue[kSnsECT]) ? caught + 0.01 : 0.0;
            if (caught > 0.15) break;
        }
        p.starter = 0; sim.setParams(p);
        if (std::getenv("TRACE"))
            for (int k = 0; k < 25; ++k) {
                runSeconds(sim, std::getenv("FAST") ? 0.02 : 0.08); sim.getTelemetry(t);
                std::printf("  t+%.2f rpm %5.0f map %5.1f afr %5.2f mis %d tgt %.2f duty %.3f wu %.2f ve %.1f lam %.2f %.2f fuelP %.2f bat %.1f\n", 0.08 * (k + 1), t.rpm, t.mapKPa, t.afr,
                            t.misfires, t.targetAfr, t.injDuty, t.ecuWarmup, t.ecuVe, t.cylLambda[0], t.cylLambda[1], t.fuelBar, t.battery);
            }
        runSeconds(sim, 5.0); sim.getTelemetry(t);
        float idleRpm = t.rpm, idleMap = t.mapKPa, idleAfr = t.afr;
        p.throttle = 1; sim.setParams(p);
        if (std::getenv("BLIPTRACE"))
            for (int k = 0; k < 12; ++k) {
                runSeconds(sim, 0.05); sim.getTelemetry(t);
                std::printf("  blip+%.2f rpm %5.0f map %5.1f tgt %5.2f duty %.2f lam %.2f %.2f mis %d fail %d sync %d fuelP %.2f\n", 0.05 * (k + 1), t.rpm, t.mapKPa,
                            t.targetAfr, t.injDuty, t.cylLambda[0], t.cylLambda[1], t.misfires, t.failure, (int)t.syncLost, t.fuelBar, t.ecuVe, t.ecuAccel, t.iac, t.ecuWarmup);
            }
        else runSeconds(sim, 0.6);
        sim.getTelemetry(t);
        float blip = t.rpm;
        p.throttle = 0; sim.setParams(p); runSeconds(sim, 4.0); sim.getTelemetry(t);
        std::printf("%-36s crank %.2fs idle %5.0f rpm map %5.1f afr %5.2f | blip %5.0f -> back %5.0f rpm afr %5.2f load %.2f %s\n",
            p.name, crank, idleRpm, idleMap, idleAfr, blip, t.rpm, t.afr, t.simLoad, (idleRpm < 450 || t.rpm < 450) ? "  <-- STALL" : "");
    }
}

// derived component characteristics of every preset; flags presets whose default parts are unsafe
static void partsReport() {
    int bad = 0;
    std::printf("%-44s %7s %7s %7s %7s %6s %6s %6s %5s %6s %6s\n", "preset", "limit", "safe", "fltIn", "fltEx", "spark", "need", "oil", "Jx", "dF1k", "dFlim");
    for (int i = 0; i < presetCount(); ++i) {
        EngineParams p = makePreset(i);
        ComponentModel m; m.build(p);
        double rpm = p.revLimit;
        double avail = m.sparkEnergy(p, rpm);
        double boostBar = isBoosted(p) ? (p.induction == Induction::Turbo ? p.wastegateBar : 0.8) : 0.0;
        double need = ComponentModel::sparkNeeded(p, (14.0 + 14.0 * boostBar) * 1e5, 0.85);
        bool ok = !m.piston || (m.safeRpm > rpm && m.floatRpmIn > rpm && m.floatRpmEx > rpm);
        ok = ok && avail > need;
        if (!ok) ++bad;
        std::printf("%-44.44s %7.0f %7.0f %7.0f %7.0f %6.1f %6.1f %6.2f %5.3f %6.1f %6.1f %s\n", p.name, rpm, m.safeRpm, m.floatRpmIn, m.floatRpmEx,
                    avail, need, ComponentModel::oilPressure(p, 800, 90), m.extraInertia,
                    m.extraFmep(p, 1000, 2 * p.stroke_mm * 1e-3 * 1000 / 60, ratedDisplacementL(p) * 5e-4) * 1e-3,
                    m.extraFmep(p, rpm, 2 * p.stroke_mm * 1e-3 * rpm / 60, ratedDisplacementL(p) * 5e-4) * 1e-3, ok ? "" : "  <-- UNSAFE DEFAULTS");
    }
    std::printf(bad ? "%d presets with unsafe default parts\n" : "all default parts safe\n", bad);
    if (bad) ++failures;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "parts") { partsReport(); return failures ? 1 : 0; }
    if (argc == 2 && std::string(argv[1]) == "survey") { survey(); return 0; }
    if (argc >= 2 && std::string(argv[1]) == "fuzz") { fuzz(argc > 2 ? std::atoi(argv[2]) : 60); return failures ? 1 : 0; }
    if (argc == 2 && std::string(argv[1]) == "audit") { massAudit(); return 0; }
    if (argc > 2) { debugRun(std::atoi(argv[1]), (float)std::atof(argv[2]), argc > 3 ? std::atof(argv[3]) : 0); return 0; }
    testMotoredCycle();
    if (argc > 1) {
        wotTable(std::atoi(argv[1]), 0, 1e9);
    } else {
        wotTable(0, 150, 260);   // I4 2.0  ~ 190-210 Nm real
        wotTable(4, 380, 650);   // V8 5.7
        wotTable(10, 25, 70);    // single 500cc
    }
    std::printf(failures ? "\n%d FAILURES\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}

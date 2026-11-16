// Phase 1b: every hardware component must change the simulation in the expected direction.
// usage: parts_test
#include "sim/Simulator.h"
#include "ecu/BaseMap.h"
#include <cstdio>
#include <vector>

using namespace es;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } else { std::printf("  ok:   " __VA_ARGS__); std::printf("\n"); } } while (0)

static void run(Simulator& sim, double s) {
    std::vector<float> buf(480 * 2);
    for (double t = 0; t < s; t += 0.01) sim.render(buf.data(), 480);
}

// start the engine, then hold it at rpm with full throttle on the dyno; returns telemetry
static Telemetry wotHold(EngineParams p, float rpm, double seconds = 2.0) {
    p.ignition = 1; p.starter = 1;
    Simulator sim(p);
    run(sim, 1.5);
    p.starter = 0; p.throttle = 1; p.dynoMode = DynoMode::Hold; p.dynoHoldRpm = rpm;
    sim.setParams(p);
    run(sim, seconds);
    // average the cycle torque over a second: the dyno absorber keeps the speed only on average
    Telemetry t, avg; double tq = 0, duty = 0; int n = 0;
    for (int k = 0; k < 20; ++k) { run(sim, 0.05); sim.getTelemetry(t); tq += t.torque; duty += t.injDuty; ++n; }
    avg = t; avg.torque = (float)(tq / n); avg.injDuty = (float)(duty / n);
    return avg;
}

int main() {
    // ---- valve springs: weak springs float the valves -> torque drops at high rpm
    {
        std::printf("[valve springs] I4 at 6500 rpm WOT\n");
        EngineParams p = makeCalibratedPreset(0);
        Telemetry a = wotHold(p, 6500);
        EngineParams w = p; w.comp.springOpen_N *= 0.35f; w.comp.springSeat_N *= 0.35f;
        ComponentModel m; m.build(w);
        Telemetry b = wotHold(w, 6500);
        std::printf("  default %.1f Nm (float %.2f)  weak springs %.1f Nm (float %.2f, onset %.0f rpm)\n", a.torque, a.floatIn, b.torque, b.floatIn, m.floatRpmIn);
        CHECK(m.floatRpmIn < 6000, "weak springs float below 6000 rpm");
        CHECK(b.floatIn > 0.5 && b.torque < a.torque * 0.92, "valve float costs torque");
    }
    // ---- stiffer springs and high-tension rings cost friction (lower WOT torque at mid rpm)
    {
        // coast-down: rev past 4500, switch the ignition off, time the fall from 4400 to 3000 rpm (a fixed band,
        // so both engines are compared over the same speeds whatever rpm they had when the ignition went off)
        std::printf("[friction] I4 coast-down 4400 -> 3000 rpm, ignition off\n");
        auto coast = [](EngineParams p) {
            p.ignition = 1; p.starter = 1;
            Simulator sim(p); run(sim, 1.5); p.starter = 0; p.throttle = 0.35f; sim.setParams(p);
            Telemetry t;
            for (int k = 0; k < 400; ++k) { run(sim, 0.01); sim.getTelemetry(t); if (t.rpm > 4500) break; }
            p.ignition = 0; p.throttle = 0; sim.setParams(p);
            double t1 = -1, t2 = -1;
            for (double tt = 0; tt < 3.0 && t2 < 0; tt += 0.002) {
                run(sim, 0.002); sim.getTelemetry(t);
                if (t1 < 0 && t.rpm < 4400) t1 = tt;
                if (t1 >= 0 && t.rpm < 3000) t2 = tt;
            }
            return (float)(t2 - t1);
        };
        EngineParams p = makeCalibratedPreset(0);
        float a = coast(p);
        EngineParams h = p; h.comp.springOpen_N *= 2.0f; h.comp.springSeat_N *= 2.0f; h.comp.ringTension = 1.6f; h.comp.oilPumpCapacity = 1.6f;
        float b = coast(h);
        std::printf("  4400 -> 3000 rpm takes: default %.3f s, heavy springs + rings + oil pump %.3f s\n", a, b);
        CHECK(a > 0 && b > 0 && b < a * 0.95, "extra friction slows the engine down faster");
    }
    // ---- ignition: a distributor coil with a slow charge can't fire a V8 at high rpm and boost-like pressure
    {
        std::printf("[coil] V8 crossplane at 6000 rpm WOT\n");
        EngineParams p = makeCalibratedPreset(4);
        p.comp.coilType = CoilType::Distributor; p.comp.coilEnergy_mJ = 30; p.comp.coilCharge_ms = 6; p.comp.plugGap_mm = 1.3f;
        ComponentModel m; m.build(p);
        Telemetry b = wotHold(p, 6000);
        EngineParams g = makeCalibratedPreset(4);
        Telemetry a = wotHold(g, 6000);
        std::printf("  stock: %.1f Nm misfires %d, spark margin %.2f | weak coil: %.1f Nm misfires %d, margin %.2f (avail %.1f mJ)\n",
                    a.torque, a.misfires, a.sparkMargin, b.torque, b.misfires, b.sparkMargin, m.sparkEnergy(p, 6000));
        CHECK(b.sparkMargin < 1.0 && b.misfires > a.misfires + 20, "weak coil + wide gap misfires at high rpm");
    }
    // ---- spark plug heat range: a much too hot plug knocks under load
    {
        std::printf("[plug heat range] I4 turbo at 5000 rpm WOT\n");
        EngineParams p = makeCalibratedPreset(16);
        Telemetry a = wotHold(p, 5000, 7.0);
        EngineParams h = p; h.comp.plugHeatRange = 4;
        Telemetry b = wotHold(h, 5000, 7.0);
        std::printf("  heat %d: tip %.0f C knock %.2f | heat 4: tip %.0f C knock %.2f\n", p.comp.plugHeatRange, a.plugTempC, a.knock, b.plugTempC, b.knock);
        CHECK(b.plugTempC > a.plugTempC + 100 && b.knock > a.knock + 0.5, "hot plug -> pre-ignition -> knock");
    }
    // ---- drive-by-wire: the throttle follows the pedal with the motor's time constant
    {
        std::printf("[throttle body] tip-in response\n");
        for (int dbw = 0; dbw < 2; ++dbw) {
            EngineParams p = makeCalibratedPreset(0);
            p.comp.throttleType = dbw ? ThrottleType::DriveByWire : ThrottleType::Cable; p.comp.dbwResponse_ms = 120;
            p.ignition = 1; p.starter = 1;
            Simulator sim(p); run(sim, 1.5); p.starter = 0; sim.setParams(p); run(sim, 1.0);
            p.throttle = 1; sim.setParams(p); run(sim, 0.05);
            Telemetry t; sim.getTelemetry(t);
            std::printf("  %s: MAP %.1f kPa 50 ms after tip-in\n", dbw ? "drive-by-wire (120 ms)" : "cable", t.mapKPa);
            static float mapCable = 0;
            if (!dbw) mapCable = t.mapKPa;
            else CHECK(t.mapKPa < mapCable - 5, "slow DBW motor delays the manifold filling");
        }
    }
    // ---- injector dead time: a slower injector than the ECU is calibrated for runs lean;
    //      updating the ECU's dead-time table restores the mixture (and raises the duty)
    {
        std::printf("[injector dead time] I4 at 4000 rpm WOT\n");
        EngineParams p = makeCalibratedPreset(0);
        p.cal.autoTune = 0;
        Telemetry a = wotHold(p, 4000);
        EngineParams d = p; d.comp.injDeadTime_ms = 2.0f;
        Telemetry b = wotHold(d, 4000);
        EngineParams f = d;
        for (int i = 0; i < f.cal.deadTime.n; ++i) f.cal.deadTime.v[i] = 2.0f * (1.0f + 0.12f * (14.0f - f.cal.deadTime.x[i]));
        Telemetry cc = wotHold(f, 4000);
        std::printf("  stock: AFR %.2f duty %.2f | 2 ms injector, old calibration: AFR %.2f | recalibrated: AFR %.2f duty %.2f\n",
                    a.afr, a.injDuty, b.afr, cc.afr, cc.injDuty);
        CHECK(b.afr > a.afr + 0.4, "uncalibrated dead time -> lean");
        CHECK(std::fabs(cc.afr - a.afr) < 0.35 && cc.injDuty > a.injDuty + 0.04, "dead-time table corrected -> mixture back, duty up");
    }
    // ---- mechanical failure: heavy pistons + no limiter -> rods let go when failure is enabled
    {
        std::printf("[mechanical failure] I4 heavy pistons over-revved\n");
        EngineParams p = makeCalibratedPreset(0);
        p.comp.pistonMass_g *= 2.5f; p.comp.allowFailure = 1; p.revLimit = 9500;
        ComponentModel m; m.build(p);
        p.ignition = 1; p.starter = 1;
        Simulator sim(p); run(sim, 1.5); p.starter = 0; p.throttle = 1; sim.setParams(p); run(sim, 5.0);
        Telemetry t; sim.getTelemetry(t);
        std::printf("  safe rpm %.0f, stress %.2f, damage %.2f, failure %d, rpm now %.0f\n", m.safeRpm, t.stress, t.damage, t.failure, t.rpm);
        CHECK(t.failure != 0 && t.rpm < 300, "over-stressed parts fail and stop the engine");
        // repair
        p.comp.repairCount++; p.starter = 1; p.throttle = 0; sim.setParams(p); run(sim, 1.5);
        sim.getTelemetry(t);
        CHECK(t.failure == 0, "repair clears the failure");
    }
    // ---- oil pressure follows pump capacity
    {
        EngineParams p = makeCalibratedPreset(0);
        double a = ComponentModel::oilPressure(p, 900, 90);
        p.comp.oilPumpCapacity = 0.5f;
        double b = ComponentModel::oilPressure(p, 900, 90);
        std::printf("[oil pump] idle oil pressure standard %.2f bar, half-size pump %.2f bar\n", a, b);
        CHECK(b < a * 0.6, "smaller pump -> lower oil pressure");
    }
    std::printf(failures ? "%d FAILURES\n" : "ALL PASSED\n", failures);
    return failures ? 1 : 0;
}
